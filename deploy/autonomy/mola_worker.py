#!/usr/bin/env python3
"""Continuously turn coherent onboard map snapshots into MOLA metric maps.

The capture/graph process owns ``snapshot.json`` and immutable chunks. This
worker only consumes them. Each component is imported independently, then one
atomic index replacement publishes the complete generation. A source change
during import discards the generation and is picked up by a later poll.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

try:
    from mola_process import MolaProcessError, NativeRequestError, PersistentImporter
except ModuleNotFoundError:  # Imported as deploy.autonomy.mola_worker in tests.
    from deploy.autonomy.mola_process import (
        MolaProcessError,
        NativeRequestError,
        PersistentImporter,
    )

SCHEMA = "swarmdeck.autonomy.v1"
MAX_SNAPSHOT_BYTES = 4 * 1024 * 1024
DEFAULT_MAX_OUTPUT_BYTES = 1024 * 1024 * 1024
DEFAULT_MAX_POINTS_PER_MAP = 2_000_000
DEFAULT_MAX_RESIDENT_POINTS = 8_000_000
DEFAULT_MAX_MAPS = 256


class WorkerError(RuntimeError):
    """The source contract or native importer failed."""


Runner = Callable[[Sequence[str], float], None]


def _default_runner(command: Sequence[str], timeout_s: float) -> None:
    try:
        subprocess.run(
            command,
            check=True,
            timeout=timeout_s,
            stdout=subprocess.DEVNULL,
            # Inherit diagnostics so the compatibility path cannot deadlock or
            # accumulate an unbounded captured stderr payload.
            stderr=None,
        )
    except subprocess.TimeoutExpired as exc:
        raise WorkerError(f"MOLA import exceeded {timeout_s:g}s") from exc
    except subprocess.CalledProcessError as exc:
        raise WorkerError(f"native importer failed with status {exc.returncode}") from exc


def _strict_nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise WorkerError(f"{name} must be a non-negative integer")
    return value


def _sha256_json(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _manifest_sha256(manifest: dict[str, object]) -> str:
    return _sha256_json(manifest)


def _geometry_fingerprint(manifest: dict[str, object]) -> str:
    """Hash the declared geometry membership without trusting its summary hash."""

    geometry = dict(manifest)
    geometry.pop("graph_revision", None)
    submaps = geometry.get("submaps")
    if not isinstance(submaps, list):
        raise WorkerError("manifest submaps must be a list")
    membership: list[dict[str, object]] = []
    for submap in submaps:
        if not isinstance(submap, dict):
            raise WorkerError("manifest submap must be an object")
        chunks = submap.get("chunks")
        if not isinstance(chunks, list):
            raise WorkerError("submap chunks must be a list")
        item = dict(submap)
        item.pop("T_component_submap", None)
        item.pop("pose_revision", None)
        membership.append(item)
    geometry["submaps"] = membership
    return _sha256_json(geometry)


def _canonical_mission_id(value: str) -> str:
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError) as exc:
        raise ValueError("mission_id must be a canonical UUID") from exc
    canonical = str(parsed)
    if value != canonical:
        raise ValueError("mission_id must be a canonical UUID")
    return canonical


def _bounded_file_bytes(path: Path, maximum: int, label: str) -> bytes:
    try:
        expected_size = path.stat().st_size
        if expected_size > maximum:
            raise WorkerError(f"{label} exceeds {maximum} byte limit")
        with path.open("rb") as stream:
            raw = stream.read(maximum + 1)
            if len(raw) > maximum:
                raise WorkerError(f"{label} exceeds {maximum} byte limit")
            if stream.read(1):
                raise WorkerError(f"{label} exceeds {maximum} byte limit")
    except FileNotFoundError as exc:
        raise WorkerError(f"{label} disappeared while reading") from exc
    if len(raw) != expected_size or path.stat().st_size != len(raw):
        raise WorkerError(f"{label} changed while reading")
    return raw


def _source_matches(path: Path, expected: bytes) -> bool:
    try:
        return _bounded_file_bytes(path, MAX_SNAPSHOT_BYTES, "snapshot") == expected
    except (OSError, WorkerError):
        return False


def _read_snapshot(path: Path) -> tuple[bytes, dict[str, object]]:
    raw = _bounded_file_bytes(path, MAX_SNAPSHOT_BYTES, "snapshot")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise WorkerError("snapshot is not valid JSON") from exc
    if not isinstance(value, dict) or value.get("schema") != SCHEMA:
        raise WorkerError("unsupported autonomy snapshot schema")
    snapshot_id = value.get("snapshot_id")
    if (
        not isinstance(snapshot_id, str)
        or len(snapshot_id) != 64
        or any(char not in "0123456789abcdef" for char in snapshot_id)
    ):
        raise WorkerError("snapshot_id must be lowercase SHA-256")
    manifests = value.get("manifests")
    if not isinstance(manifests, list) or len(manifests) > 256:
        raise WorkerError("manifests must be a bounded list")
    seen: set[str] = set()
    for manifest in manifests:
        if not isinstance(manifest, dict):
            raise WorkerError("manifest must be an object")
        revision = manifest.get("graph_revision")
        if not isinstance(revision, dict):
            raise WorkerError("manifest graph_revision is required")
        component = revision.get("component_id")
        if not isinstance(component, str) or not component or len(component) > 128:
            raise WorkerError("component_id must be a nonempty bounded string")
        if component in seen:
            raise WorkerError("snapshot repeats a component_id")
        seen.add(component)
        _strict_nonnegative_int(revision.get("epoch"), "component epoch")
        _strict_nonnegative_int(revision.get("revision"), "component revision")
        geometry = manifest.get("geometry_revision")
        if (
            not isinstance(geometry, str)
            or len(geometry) != 64
            or any(char not in "0123456789abcdef" for char in geometry)
        ):
            raise WorkerError("geometry_revision must be lowercase SHA-256")
    return raw, value


def _sha256_file(path: Path, maximum: int) -> tuple[int, str]:
    expected_size = path.stat().st_size
    if expected_size <= 0 or expected_size > maximum:
        raise WorkerError(f"native artifact has invalid size {expected_size}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while size <= maximum:
            block = stream.read(min(1024 * 1024, maximum + 1 - size))
            if not block:
                break
            size += len(block)
            digest.update(block)
    if size <= 0 or size > maximum or size != expected_size:
        raise WorkerError(f"native artifact has invalid size {size}")
    if path.stat().st_size != size:
        raise WorkerError("native artifact changed while hashing")
    return size, digest.hexdigest()


def _atomic_json(path: Path, value: object) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class ProcessResult:
    published: bool
    snapshot_id: str
    component_count: int


class MolaWorker:
    def __init__(
        self,
        maps_root: Path,
        *,
        importer: Path = Path(
            "/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import"
        ),
        timeout_s: float = 120.0,
        poll_s: float = 1.0,
        retry_s: float = 5.0,
        max_output_bytes: int = DEFAULT_MAX_OUTPUT_BYTES,
        max_points_per_map: int = DEFAULT_MAX_POINTS_PER_MAP,
        max_resident_points: int = DEFAULT_MAX_RESIDENT_POINTS,
        max_maps: int = DEFAULT_MAX_MAPS,
        keep_generations: int = 2,
        mode: str = "persistent",
        mission_id: str | None = None,
        runner: Runner = _default_runner,
    ):
        if timeout_s <= 0 or poll_s <= 0 or retry_s < 0:
            raise ValueError("worker timing values are invalid")
        if (
            max_output_bytes <= 0
            or max_points_per_map <= 0
            or max_resident_points <= 0
            or max_maps <= 0
            or keep_generations < 1
        ):
            raise ValueError("worker runtime bounds are invalid")
        if mode not in ("persistent", "oneshot"):
            raise ValueError("worker mode must be persistent or oneshot")
        self.maps_root = Path(maps_root)
        self.importer = Path(importer)
        self.timeout_s = timeout_s
        self.poll_s = poll_s
        self.retry_s = retry_s
        self.max_output_bytes = max_output_bytes
        self.max_points_per_map = max_points_per_map
        self.max_resident_points = max_resident_points
        self.max_maps = max_maps
        self.keep_generations = keep_generations
        self.mode = mode
        self.mission_id = (
            _canonical_mission_id(mission_id) if mission_id is not None else None
        )
        self.runner = runner
        self._runtime = (
            PersistentImporter(
                self.importer,
                self.timeout_s,
                max_points_per_map=self.max_points_per_map,
                max_resident_points=self.max_resident_points,
                max_maps=self.max_maps,
                max_output_bytes=self.max_output_bytes,
            )
            if mode == "persistent"
            else None
        )
        self._resident_maps: set[str] = set()
        self._resident_by_peer: dict[Path, set[str]] = {}
        self._completed: dict[Path, str] = {}
        self._retry_after: dict[Path, float] = {}

    def discover(self) -> tuple[Path, ...]:
        """Return peer roots matching /maps/<mission>/<robot>/snapshot.json."""

        if self.mission_id is not None:
            mission_root = self.maps_root / self.mission_id
            return tuple(
                sorted(path.parent for path in mission_root.glob("*/snapshot.json"))
            )
        return tuple(
            sorted(path.parent for path in self.maps_root.glob("*/*/snapshot.json"))
        )

    @staticmethod
    def _map_id(peer_root: Path, component_id: str) -> str:
        peer_digest = hashlib.sha256(str(peer_root.resolve()).encode()).hexdigest()
        return f"{peer_digest}:{component_id}"

    def _published_artifacts(
        self, mola_root: Path, components_root: Path
    ) -> dict[str, dict[str, object]]:
        index_path = mola_root / "index.json"
        try:
            raw = _bounded_file_bytes(index_path, MAX_SNAPSHOT_BYTES, "MOLA index")
            value = json.loads(raw)
        except (OSError, WorkerError, UnicodeDecodeError, json.JSONDecodeError):
            return {}
        artifacts = value.get("artifacts") if isinstance(value, dict) else None
        if not isinstance(artifacts, list):
            return {}
        valid: dict[str, dict[str, object]] = {}
        for item in artifacts:
            if not isinstance(item, dict):
                continue
            component_id, relative = item.get("component_id"), item.get("path")
            expected_size, expected_sha = item.get("size_bytes"), item.get("sha256")
            if (
                not isinstance(component_id, str)
                or not isinstance(relative, str)
                or not isinstance(expected_size, int)
                or isinstance(expected_size, bool)
                or not isinstance(expected_sha, str)
            ):
                continue
            path = mola_root / relative
            if path.resolve().parent != components_root.resolve():
                continue
            valid[component_id] = dict(item)
        return valid

    def _artifact_matches(self, mola_root: Path, item: dict[str, object]) -> bool:
        try:
            size, digest = _sha256_file(
                mola_root / str(item["path"]), self.max_output_bytes
            )
        except (KeyError, OSError, WorkerError):
            return False
        return size == item.get("size_bytes") and digest == item.get("sha256")

    @staticmethod
    def _validate_runtime_response(
        response: dict[str, object],
        *,
        request_mode: str,
        map_id: str,
        snapshot_id: str,
        source_sha: str,
        manifest: dict[str, object],
    ) -> None:
        revision = manifest["graph_revision"]
        assert isinstance(revision, dict)
        expected = {
            "mode": request_mode,
            "map_id": map_id,
            "source_snapshot_id": snapshot_id,
            "source_sha256": source_sha,
            "component_id": revision["component_id"],
            "epoch": revision["epoch"],
            "revision": revision["revision"],
            "geometry_revision": manifest["geometry_revision"],
        }
        for name in ("epoch", "revision"):
            _strict_nonnegative_int(response.get(name), f"native response {name}")
        if any(response.get(name) != value for name, value in expected.items()):
            raise WorkerError("native runtime response does not match the request")
        result = response.get("result")
        allowed = {
            "replace": {"replaced", "duplicate"},
            "pose_only": {"corrected", "duplicate"},
        }
        if result not in allowed[request_mode]:
            raise WorkerError("native runtime returned an invalid apply result")
        for name in ("submaps", "points", "output_size_bytes"):
            _strict_nonnegative_int(response.get(name), f"native response {name}")
        submaps = manifest.get("submaps")
        assert isinstance(submaps, list)
        if response["submaps"] != len(submaps):
            raise WorkerError("native runtime returned an invalid submap count")
        digest = response.get("output_sha256")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)
        ):
            raise WorkerError("native response output_sha256 is invalid")

    def _persistent_import(
        self,
        *,
        mode: str,
        map_id: str,
        input_path: Path,
        input_sha: str,
        chunks: Path,
        output_path: Path,
        snapshot_id: str,
        manifest: dict[str, object],
    ) -> dict[str, object]:
        assert self._runtime is not None
        request = {
            "mode": mode,
            "map_id": map_id,
            "snapshot_path": str(input_path),
            "snapshot_sha256": input_sha,
            "chunks_dir": str(chunks),
            "output_path": str(output_path),
        }
        try:
            response = self._runtime.apply(request)
        except NativeRequestError as exc:
            if mode != "pose_only" or exc.code != "cache_miss":
                raise WorkerError(str(exc)) from exc
            mode = "replace"
            request["mode"] = mode
            try:
                response = self._runtime.apply(request)
            except MolaProcessError as retry_exc:
                self._invalidate_runtime()
                raise WorkerError(str(retry_exc)) from retry_exc
        except MolaProcessError as exc:
            self._invalidate_runtime()
            raise WorkerError(str(exc)) from exc
        try:
            self._validate_runtime_response(
                response,
                request_mode=mode,
                map_id=map_id,
                snapshot_id=snapshot_id,
                source_sha=input_sha,
                manifest=manifest,
            )
        except WorkerError:
            self._invalidate_runtime()
            raise
        self._resident_maps.add(map_id)
        return response

    def _invalidate_runtime(self) -> None:
        if self._runtime is not None:
            self._runtime.close()
        self._resident_maps.clear()
        self._resident_by_peer.clear()

    def _release_inactive_maps(self, peer_root: Path, active: set[str]) -> None:
        known = self._resident_by_peer.get(peer_root, set())
        inactive = known - active
        if self._runtime is not None:
            try:
                for map_id in sorted(inactive):
                    self._runtime.release(map_id)
            except MolaProcessError:
                # Publication has already committed. A full restart safely
                # bounds native state if targeted reclamation fails.
                self._invalidate_runtime()
                return
        self._resident_maps.difference_update(inactive)
        if active & self._resident_maps:
            self._resident_by_peer[peer_root] = active & self._resident_maps
        else:
            self._resident_by_peer.pop(peer_root, None)

    def process_peer(self, peer_root: Path) -> ProcessResult:
        peer_root = Path(peer_root)
        source = peer_root / "snapshot.json"
        chunks = peer_root / "geometry" / "chunks"
        if not chunks.is_dir():
            raise WorkerError("geometry/chunks directory is missing")
        raw, snapshot = _read_snapshot(source)
        source_sha = hashlib.sha256(raw).hexdigest()
        snapshot_id = str(snapshot["snapshot_id"])
        manifests = snapshot["manifests"]
        assert isinstance(
            manifests, list
        )  # narrowed by _read_snapshot; never a test predicate

        mola_root = peer_root / "mola"
        components_root = mola_root / "components"
        mola_root.mkdir(exist_ok=True)
        components_root.mkdir(exist_ok=True)
        staging = mola_root / f".staging-{uuid.uuid4().hex}"
        staging.mkdir()
        published = self._published_artifacts(mola_root, components_root)
        staged: list[tuple[Path, Path]] = []
        artifacts: list[dict[str, object]] = []
        try:
            for ordinal, manifest in enumerate(manifests):
                assert isinstance(manifest, dict)
                revision = manifest["graph_revision"]
                assert isinstance(revision, dict)
                component_id = revision["component_id"]
                assert isinstance(component_id, str)
                manifest_sha = _manifest_sha256(manifest)
                geometry_fingerprint = _geometry_fingerprint(manifest)
                prior = published.get(component_id)
                if (
                    prior is not None
                    and prior.get("manifest_sha256") == manifest_sha
                    and self._artifact_matches(mola_root, prior)
                ):
                    artifacts.append(prior)
                    continue
                component_hash = hashlib.sha256(component_id.encode()).hexdigest()[:20]
                filename = (
                    f"{component_hash}-e{revision['epoch']}-r{revision['revision']}-"
                    f"{manifest_sha[:16]}-{uuid.uuid4().hex[:16]}.metricmap"
                )
                component_snapshot = {
                    "schema": SCHEMA,
                    "snapshot_id": snapshot_id,
                    "generated_at_ns": snapshot.get("generated_at_ns", 0),
                    "manifests": [manifest],
                }
                input_path = staging / f"component-{ordinal}.json"
                input_bytes = json.dumps(
                    component_snapshot, sort_keys=True, separators=(",", ":")
                ).encode()
                input_path.write_bytes(input_bytes)
                input_sha = hashlib.sha256(input_bytes).hexdigest()
                output_path = staging / filename
                map_id = self._map_id(peer_root, component_id)
                request_mode = (
                    "pose_only"
                    if map_id in self._resident_maps
                    and prior is not None
                    and prior.get("geometry_fingerprint") == geometry_fingerprint
                    else "replace"
                )
                response: dict[str, object] | None = None
                if self.mode == "persistent":
                    response = self._persistent_import(
                        mode=request_mode,
                        map_id=map_id,
                        input_path=input_path,
                        input_sha=input_sha,
                        chunks=chunks,
                        output_path=output_path,
                        snapshot_id=snapshot_id,
                        manifest=manifest,
                    )
                    self._resident_by_peer.setdefault(peer_root, set()).add(map_id)
                else:
                    self.runner(
                        (
                            str(self.importer),
                            str(input_path),
                            str(chunks),
                            str(output_path),
                        ),
                        self.timeout_s,
                    )
                size, digest = _sha256_file(output_path, self.max_output_bytes)
                if response is not None and (
                    response.get("output_size_bytes") != size
                    or response.get("output_sha256") != digest
                ):
                    self._invalidate_runtime()
                    raise WorkerError("native artifact does not match its response")
                final_path = components_root / filename
                staged.append((output_path, final_path))
                artifacts.append(
                    {
                        "component_id": component_id,
                        "epoch": revision["epoch"],
                        "revision": revision["revision"],
                        "geometry_revision": manifest["geometry_revision"],
                        "manifest_sha256": manifest_sha,
                        "geometry_fingerprint": geometry_fingerprint,
                        "path": f"components/{filename}",
                        "size_bytes": size,
                        "sha256": digest,
                    }
                )

            # The bridge replaces snapshot.json atomically. Byte equality is a
            # stronger guard than revision alone and catches same-ID corruption.
            if not _source_matches(source, raw):
                # Some staged requests may already have advanced native state.
                # Restarting prevents a discarded generation from becoming the
                # base for the next pose-only correction.
                self._invalidate_runtime()
                return ProcessResult(False, snapshot_id, len(manifests))
            for staged_path, final_path in staged:
                os.replace(staged_path, final_path)
            if not _source_matches(source, raw):
                self._invalidate_runtime()
                return ProcessResult(False, snapshot_id, len(manifests))
            index = {
                "version": 1,
                "source_snapshot_id": snapshot_id,
                "source_sha256": source_sha,
                "generated_at_ns": time.time_ns(),
                "artifacts": artifacts,
            }
            index_path = mola_root / "index.json"
            _atomic_json(index_path, index)
            # No cross-process transaction can cover both source and index.
            # Recheck immediately and retract only our own index on a race.
            if not _source_matches(source, raw):
                try:
                    current = json.loads(index_path.read_text())
                    if current.get("source_sha256") == source_sha:
                        index_path.unlink()
                except (FileNotFoundError, json.JSONDecodeError, OSError):
                    pass
                self._invalidate_runtime()
                return ProcessResult(False, snapshot_id, len(manifests))
            self._prune(
                components_root,
                {mola_root / str(item["path"]) for item in artifacts},
            )
            active_map_ids = {
                self._map_id(peer_root, str(manifest["graph_revision"]["component_id"]))
                for manifest in manifests
            }
            self._release_inactive_maps(peer_root, active_map_ids)
            return ProcessResult(True, snapshot_id, len(manifests))
        except FileNotFoundError as exc:
            self._invalidate_runtime()
            raise WorkerError("snapshot or native artifact disappeared") from exc
        except OSError:
            self._invalidate_runtime()
            raise
        finally:
            shutil.rmtree(staging, ignore_errors=True)

    def _prune(self, directory: Path, current: set[Path]) -> None:
        previous = sorted(
            (path for path in directory.glob("*.metricmap") if path not in current),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        keep_previous = max(0, self.keep_generations - 1) * max(1, len(current))
        for path in previous[keep_previous:]:
            path.unlink(missing_ok=True)

    def run_once(self) -> dict[Path, str]:
        """Process changed peers once; errors are retained for status/logging."""

        errors: dict[Path, str] = {}
        now = time.monotonic()
        peers = self.discover()
        for missing_peer in self._resident_by_peer.keys() - set(peers):
            self._release_inactive_maps(missing_peer, set())
            self._completed.pop(missing_peer, None)
            self._retry_after.pop(missing_peer, None)
        for peer in peers:
            source = peer / "snapshot.json"
            try:
                raw = _bounded_file_bytes(source, MAX_SNAPSHOT_BYTES, "snapshot")
                source_sha = hashlib.sha256(raw).hexdigest()
            except (OSError, WorkerError) as exc:
                errors[peer] = str(exc)
                continue
            if self._completed.get(peer) == source_sha:
                continue
            if now < self._retry_after.get(peer, 0):
                continue
            try:
                result = self.process_peer(peer)
                if result.published:
                    self._completed[peer] = source_sha
                else:
                    self._retry_after.pop(peer, None)
            except (WorkerError, OSError, KeyError, TypeError) as exc:
                errors[peer] = f"{type(exc).__name__}: {exc}"
                self._retry_after[peer] = now + self.retry_s
        return errors

    def run_forever(self) -> None:
        try:
            while True:
                for peer, error in self.run_once().items():
                    print(f"swarmdeck-mola-worker: {peer}: {error}", flush=True)
                time.sleep(self.poll_s)
        finally:
            self.close()

    def close(self) -> None:
        self._invalidate_runtime()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--maps-root",
        type=Path,
        default=Path(os.getenv("SWARMDECK_MAPS_ROOT", "/maps")),
    )
    parser.add_argument(
        "--importer",
        type=Path,
        default=Path(
            os.getenv(
                "SWARMDECK_MOLA_IMPORTER",
                "/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import",
            )
        ),
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll", type=float, default=1.0)
    parser.add_argument("--retry", type=float, default=5.0)
    parser.add_argument("--keep-generations", type=int, default=2)
    parser.add_argument(
        "--max-output-bytes",
        type=int,
        default=int(
            os.getenv("SWARMDECK_MOLA_MAX_OUTPUT_BYTES", DEFAULT_MAX_OUTPUT_BYTES)
        ),
    )
    parser.add_argument(
        "--max-points-per-map",
        type=int,
        default=int(
            os.getenv(
                "SWARMDECK_MOLA_MAX_POINTS_PER_MAP", DEFAULT_MAX_POINTS_PER_MAP
            )
        ),
    )
    parser.add_argument(
        "--max-resident-points",
        type=int,
        default=int(
            os.getenv(
                "SWARMDECK_MOLA_MAX_RESIDENT_POINTS",
                DEFAULT_MAX_RESIDENT_POINTS,
            )
        ),
    )
    parser.add_argument(
        "--max-maps",
        type=int,
        default=int(os.getenv("SWARMDECK_MOLA_MAX_MAPS", DEFAULT_MAX_MAPS)),
    )
    parser.add_argument(
        "--mode",
        choices=("persistent", "oneshot"),
        default=os.getenv("SWARMDECK_MOLA_MODE", "persistent"),
        help="native importer lifecycle; oneshot is the explicit compatibility mode",
    )
    parser.add_argument(
        "--mission-id",
        default=os.getenv("SWARMDECK_MISSION_ID"),
        help="canonical mission UUID to process (defaults to SWARMDECK_MISSION_ID)",
    )
    parser.add_argument(
        "--all-missions",
        action="store_true",
        help="explicitly enable legacy discovery across every mission directory",
    )
    args = parser.parse_args()
    if args.all_missions and args.mission_id is not None:
        parser.error("--all-missions conflicts with --mission-id/SWARMDECK_MISSION_ID")
    if not args.all_missions and args.mission_id is None:
        parser.error("--mission-id or SWARMDECK_MISSION_ID is required")
    MolaWorker(
        args.maps_root,
        importer=args.importer,
        timeout_s=args.timeout,
        poll_s=args.poll,
        retry_s=args.retry,
        keep_generations=args.keep_generations,
        max_output_bytes=args.max_output_bytes,
        max_points_per_map=args.max_points_per_map,
        max_resident_points=args.max_resident_points,
        max_maps=args.max_maps,
        mode=args.mode,
        mission_id=None if args.all_missions else args.mission_id,
    ).run_forever()


if __name__ == "__main__":
    main()
