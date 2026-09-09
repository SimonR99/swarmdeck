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

SCHEMA = "swarmdeck.autonomy.v1"
MAX_SNAPSHOT_BYTES = 4 * 1024 * 1024
DEFAULT_MAX_OUTPUT_BYTES = 1024 * 1024 * 1024


class WorkerError(RuntimeError):
    """The source contract or native importer failed."""


Runner = Callable[[Sequence[str], float], None]


def _default_runner(command: Sequence[str], timeout_s: float) -> None:
    try:
        subprocess.run(
            command,
            check=True,
            timeout=timeout_s,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except subprocess.TimeoutExpired as exc:
        raise WorkerError(f"MOLA import exceeded {timeout_s:g}s") from exc
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or "native importer failed").strip()
        raise WorkerError(detail[-2000:]) from exc


def _strict_nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise WorkerError(f"{name} must be a non-negative integer")
    return value


def _read_snapshot(path: Path) -> tuple[bytes, dict[str, object]]:
    try:
        size = path.stat().st_size
        if size > MAX_SNAPSHOT_BYTES:
            raise WorkerError("snapshot exceeds 4 MiB limit")
        raw = path.read_bytes()
    except FileNotFoundError as exc:
        raise WorkerError("snapshot disappeared while reading") from exc
    if len(raw) != size:
        raise WorkerError("snapshot changed while reading")
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
    size = path.stat().st_size
    if size <= 0 or size > maximum:
        raise WorkerError(f"native artifact has invalid size {size}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
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
        keep_generations: int = 2,
        runner: Runner = _default_runner,
    ):
        if timeout_s <= 0 or poll_s <= 0 or retry_s < 0:
            raise ValueError("worker timing values are invalid")
        if max_output_bytes <= 0 or keep_generations < 1:
            raise ValueError("worker output bounds are invalid")
        self.maps_root = Path(maps_root)
        self.importer = Path(importer)
        self.timeout_s = timeout_s
        self.poll_s = poll_s
        self.retry_s = retry_s
        self.max_output_bytes = max_output_bytes
        self.keep_generations = keep_generations
        self.runner = runner
        self._completed: dict[Path, str] = {}
        self._retry_after: dict[Path, float] = {}

    def discover(self) -> tuple[Path, ...]:
        """Return peer roots matching /maps/<mission>/<robot>/snapshot.json."""

        return tuple(
            sorted(path.parent for path in self.maps_root.glob("*/*/snapshot.json"))
        )

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
        staged: list[tuple[Path, Path, dict[str, object]]] = []
        try:
            for ordinal, manifest in enumerate(manifests):
                revision = manifest["graph_revision"]
                component_id = revision["component_id"]
                component_hash = hashlib.sha256(component_id.encode()).hexdigest()[:20]
                filename = (
                    f"{component_hash}-e{revision['epoch']}-r{revision['revision']}-"
                    f"{snapshot_id[:16]}.metricmap"
                )
                component_snapshot = {
                    "schema": SCHEMA,
                    "snapshot_id": snapshot_id,
                    "generated_at_ns": snapshot.get("generated_at_ns", 0),
                    "manifests": [manifest],
                }
                input_path = staging / f"component-{ordinal}.json"
                input_path.write_text(
                    json.dumps(
                        component_snapshot, sort_keys=True, separators=(",", ":")
                    )
                )
                output_path = staging / filename
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
                final_path = components_root / filename
                staged.append(
                    (
                        output_path,
                        final_path,
                        {
                            "component_id": component_id,
                            "epoch": revision["epoch"],
                            "revision": revision["revision"],
                            "geometry_revision": manifest["geometry_revision"],
                            "path": f"components/{filename}",
                            "size_bytes": size,
                            "sha256": digest,
                        },
                    )
                )

            # The bridge replaces snapshot.json atomically. Byte equality is a
            # stronger guard than revision alone and catches same-ID corruption.
            if source.read_bytes() != raw:
                return ProcessResult(False, snapshot_id, len(manifests))
            for staged_path, final_path, _ in staged:
                os.replace(staged_path, final_path)
            if source.read_bytes() != raw:
                return ProcessResult(False, snapshot_id, len(manifests))
            index = {
                "version": 1,
                "source_snapshot_id": snapshot_id,
                "source_sha256": source_sha,
                "generated_at_ns": time.time_ns(),
                "artifacts": [item for _, _, item in staged],
            }
            index_path = mola_root / "index.json"
            _atomic_json(index_path, index)
            # No cross-process transaction can cover both source and index.
            # Recheck immediately and retract only our own index on a race.
            if source.read_bytes() != raw:
                try:
                    current = json.loads(index_path.read_text())
                    if current.get("source_sha256") == source_sha:
                        index_path.unlink()
                except (FileNotFoundError, json.JSONDecodeError, OSError):
                    pass
                return ProcessResult(False, snapshot_id, len(manifests))
            self._prune(components_root, {path for _, path, _ in staged})
            return ProcessResult(True, snapshot_id, len(manifests))
        except FileNotFoundError as exc:
            raise WorkerError("snapshot or native artifact disappeared") from exc
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
        for peer in self.discover():
            source = peer / "snapshot.json"
            try:
                raw = source.read_bytes()
                source_sha = hashlib.sha256(raw).hexdigest()
            except OSError as exc:
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
        while True:
            for peer, error in self.run_once().items():
                print(f"swarmdeck-mola-worker: {peer}: {error}", flush=True)
            time.sleep(self.poll_s)


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
    args = parser.parse_args()
    MolaWorker(
        args.maps_root,
        importer=args.importer,
        timeout_s=args.timeout,
        poll_s=args.poll,
        retry_s=args.retry,
        keep_generations=args.keep_generations,
    ).run_forever()


if __name__ == "__main__":
    main()
