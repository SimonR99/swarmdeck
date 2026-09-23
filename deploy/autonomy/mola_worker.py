#!/usr/bin/env python3
"""Continuously turn coherent onboard map snapshots into MOLA metric maps.

The capture/graph process owns ``snapshot.json`` and immutable chunks. This
worker only consumes them. Each component is imported independently, then the
complete generation is published as a self-described product under
``<peer>/mola/``:

1. the component artifacts (``components/*.metricmap`` and, with planner maps,
   ``components/*.sdpg``), each written once and never modified;
2. ``source.json``: the exact bytes of the ``snapshot.json`` the generation was
   built from, written to a temporary file in the same directory and
   ``os.replace``d;
3. ``index.json``: atomically replaced last; its ``source_sha256`` and
   ``source_snapshot_id`` describe ``source.json``.

Readers (``autonomy/mola_mapping.py`` for the indexed map server and the MGG
planner's ``MolaMap`` loader) read ``index.json`` and then ``source.json`` and
require ``sha256(source.json bytes) == index.source_sha256`` and
``source.snapshot_id == index.source_snapshot_id``. A mismatch means the pair
is mid-replacement and the reader retries briefly. Readers never read
``snapshot.json``: that file is the bridge's newest input and may be ahead of
the product.

``mola/worker.json`` is not part of the product: it records the outcome of
this worker's last build attempt for the peer (``error`` empty after a
published build), so a component that has outgrown the point budget, whose
product then stays at the last revision that fit, is reported by the bridge
in ``status.json`` as ``product_error`` instead of only in this log.

A completed build is therefore always published, even when ``snapshot.json``
moved on during the build. The earlier protocol validated ``index.json``
against the current ``snapshot.json`` bytes, so a build whose source changed
mid-way had to be discarded together with the native runtime state. On the
benchbot deployment (2026-09-18) the Swarm-SLAM bridge replaced
``snapshot.json`` about once per second while a robot drove, while one native
build took about 1.5 s for 90 keyframes; most builds were discarded, products
were published only 10 to 27 s apart, and 62% of MGG plan requests ran without
a map. Publishing the older-but-coherent generation keeps a map available
while the next build catches up.

Peers are built concurrently, each through its own native runtime. Every
product re-imports every keyframe, so one build takes about 1.2 s per 60
keyframes and grows linearly with the keyframe count (benchbot, 2026-09-18).
Building four robots in turn through one runtime made a robot's product
interval the sum of four builds: 2 to 6 s at 60 keyframes, reaching 20 s or
more later in a mission, which is how far the planner map then lags the robot.
``run_once`` decides on the calling thread which peers need a build and runs
those builds on a thread pool of ``parallel_peers`` workers
(``--parallel-peers``, default 4). Each build touches only its peer's
directories and its peer's ``swarmdeck-mola-import --serve`` process, so a
native failure on one peer restarts that runtime alone.
"""

from __future__ import annotations

import argparse
import concurrent.futures
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
from autonomy.map_epochs import (
    map_epoch_lock,
    read_map_epoch,
    snapshot_epoch_dependencies,
    assert_map_epoch_dependencies,
)

try:
    from mola_process import MolaProcessError, NativeRequestError, PersistentImporter
except ModuleNotFoundError:  # Imported as deploy.autonomy.mola_worker in tests.
    from deploy.autonomy.mola_process import (
        MolaProcessError,
        NativeRequestError,
        PersistentImporter,
    )

SCHEMA = "swarmdeck.autonomy.v1"
MAX_SNAPSHOT_BYTES = 64 * 1024 * 1024
DEFAULT_MAX_OUTPUT_BYTES = 1024 * 1024 * 1024
# Shared native materialized-point limit, not the sum of historical captures.
# Metric geometry and planner surface extrema are compacted independently;
# original qualified rays feed retained planner evidence. MGG bounds materialized
# samples through map.mola.max_voxels and verifies raw source counts separately.
DEFAULT_MAX_POINTS_PER_MAP = 2_000_000
# A replacement holds old and new metric geometry and planner evidence at once.
DEFAULT_MAX_RESIDENT_POINTS = 4 * DEFAULT_MAX_POINTS_PER_MAP
DEFAULT_MAX_MAPS = 256
DEFAULT_PARALLEL_PEERS = 4
# ``<peer>/mola/worker.json``: the outcome of this worker's last build attempt
# for the peer, written after every attempt. ``error`` is empty after a
# published build and the failure message otherwise; ``source_sha256`` names
# the ``snapshot.json`` bytes the attempt read. The bridge folds ``error``
# into the peer's ``status.json`` as ``product_error``.
WORKER_STATUS_VERSION = 1
WORKER_STATUS_MAX_ERROR_CHARS = 2000


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
        raise WorkerError(
            f"native importer failed with status {exc.returncode}"
        ) from exc


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


def _file_identity(path: Path) -> tuple[int, int, int, int] | None:
    """Fields an atomic replacement changes, or None when the file is absent."""

    try:
        value = path.stat()
    except OSError:
        return None
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)


def _stat_matches(path: Path, expected_size: object, maximum: int) -> bool:
    """True when ``path`` is a file of exactly ``expected_size`` bytes.

    Two callers trust a sha256 they never recompute from `path`'s bytes: a
    previously published artifact this worker wrote once and never modifies
    (module docstring), trusting its own recorded sha256; and a component or
    planner map the persistent native runtime just wrote in this build,
    trusting the hash it reported for the exact bytes it wrote. Re-hashing a
    component that may be several megabytes, on every poll that finds
    nothing else to build, or a second time right after the process that
    wrote it already hashed it, costs as much as the import itself for a
    hash that would only ever match; `stat` still catches a missing,
    truncated or oversized file either way. Local hashing
    (`_sha256_file`) remains only for the legacy non-persistent runner mode,
    which has no reported hash to trust.
    """

    if not isinstance(expected_size, int) or not 0 < expected_size <= maximum:
        return False
    try:
        size = path.stat().st_size
    except OSError:
        return False
    return size == expected_size


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


def _write_fsynced(path: Path, payload: bytes) -> None:
    """Create ``path`` holding ``payload``, with its data fsynced."""

    with path.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _fsync_directory(path: Path) -> None:
    directory = os.open(path, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def _atomic_bytes(path: Path, payload: bytes) -> None:
    """Replace ``path`` with ``payload`` through a fsynced sibling temporary."""

    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        _write_fsynced(temporary, payload)
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def _json_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"


def _atomic_json(path: Path, value: object) -> None:
    _atomic_bytes(path, _json_bytes(value))


@dataclass(frozen=True)
class ProcessResult:
    published: bool
    snapshot_id: str
    component_count: int
    # SHA-256 of the snapshot bytes the product was built from, which is also
    # the published source.json. run_once records it so an unchanged snapshot
    # is not rebuilt, even when snapshot.json moved on during the build.
    source_sha256: str


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
        planner_maps: bool = False,
        mission_id: str | None = None,
        runner: Runner = _default_runner,
        parallel_peers: int = DEFAULT_PARALLEL_PEERS,
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
        if (
            not isinstance(parallel_peers, int)
            or isinstance(parallel_peers, bool)
            or parallel_peers < 1
        ):
            raise ValueError("parallel_peers must be a positive integer")
        if mode not in ("persistent", "oneshot"):
            raise ValueError("worker mode must be persistent or oneshot")
        if planner_maps and mode != "persistent":
            raise ValueError("planner maps require the persistent native runtime")
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
        self.planner_maps = planner_maps
        self.mission_id = (
            _canonical_mission_id(mission_id) if mission_id is not None else None
        )
        self.runner = runner
        self.parallel_peers = parallel_peers
        # Per-peer native state: one ``swarmdeck-mola-import --serve`` client
        # per peer root, created on the peer's first native build and closed
        # when the peer disappears or its runtime is invalidated, and the map
        # ids resident in that runtime. A build reads and writes only its own
        # peer's entries, so builds of different peers may run on different
        # threads without a lock; run_once iterates these dicts only while no
        # build is running.
        self._runtimes: dict[Path, PersistentImporter] = {}
        self._resident_by_peer: dict[Path, set[str]] = {}
        self._peer_runs: dict[Path, str | None] = {}
        # Poll bookkeeping, touched only on the thread that calls run_once.
        self._completed: dict[Path, str] = {}
        self._retry_after: dict[Path, float] = {}
        # snapshot.json's identity (`_file_identity`) the last time it was
        # actually read and hashed for a completed build. Unchanged between
        # polls, most of the time while a peer is parked, so the next poll
        # can skip re-reading and re-hashing the whole file.
        self._snapshot_identity: dict[Path, tuple[int, int, int, int]] = {}
        # The failure last logged per peer. A failing build is retried every
        # ``retry_s`` and the same message would otherwise repeat on every
        # retry (198 and 216 identical lines per robot on benchbot, mission
        # 1a8cc114); it is logged when it first appears, when it changes, and
        # once more when the peer publishes again. ``mola/worker.json``
        # carries the current state continuously.
        self._logged_errors: dict[Path, str] = {}

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
            path = mola_root / str(item["path"])
        except KeyError:
            return False
        digest = item.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            return False
        # The published sha256 was already trusted when this artifact was
        # written (the native runtime's own response, or a prior `stat`
        # check); it never changes underneath an unmodified file, so
        # reconfirming reuse only needs to know the file is still there at
        # its recorded size.
        return _stat_matches(path, item.get("size_bytes"), self.max_output_bytes)

    def _planner_matches(self, mola_root: Path, item: dict[str, object]) -> bool:
        planner = item.get("planner")
        if not isinstance(planner, dict):
            return False
        expected = Path(str(item["path"])).with_suffix(".sdpg")
        if planner.get("path") != str(expected):
            return False
        return self._artifact_matches(mola_root, planner)

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

    def _runtime_for(self, peer_root: Path) -> PersistentImporter:
        """Return the peer's native runtime client, creating it on first use.

        The client starts its ``--serve`` process on the first request, with
        the same limits for every peer.
        """

        runtime = self._runtimes.get(peer_root)
        if runtime is None:
            runtime = PersistentImporter(
                self.importer,
                self.timeout_s,
                max_points_per_map=self.max_points_per_map,
                max_resident_points=self.max_resident_points,
                max_maps=self.max_maps,
                max_output_bytes=self.max_output_bytes,
            )
            self._runtimes[peer_root] = runtime
        return runtime

    def _invalidate_runtime(self, peer_root: Path) -> None:
        """Close the peer's native runtime and forget its resident maps.

        Other peers' runtimes are untouched: a native failure or artifact
        mismatch on one peer must not restart the others. The next build of
        this peer starts a fresh runtime and rebuilds from immutable chunks.
        """

        runtime = self._runtimes.pop(peer_root, None)
        if runtime is not None:
            runtime.close()
        self._resident_by_peer.pop(peer_root, None)

    def _persistent_import(
        self,
        *,
        peer_root: Path,
        mode: str,
        map_id: str,
        input_path: Path,
        input_sha: str,
        chunks: Path,
        output_path: Path,
        planner_output_path: Path | None,
        snapshot_id: str,
        manifest: dict[str, object],
    ) -> dict[str, object]:
        runtime = self._runtime_for(peer_root)
        request = {
            "mode": mode,
            "map_id": map_id,
            "snapshot_path": str(input_path),
            "snapshot_sha256": input_sha,
            "chunks_dir": str(chunks),
            "output_path": str(output_path),
        }
        if planner_output_path is not None:
            request["planner_output_path"] = str(planner_output_path)
        try:
            response = runtime.apply(request)
        except NativeRequestError as exc:
            if mode != "pose_only" or exc.code != "cache_miss":
                raise WorkerError(str(exc)) from exc
            mode = "replace"
            request["mode"] = mode
            try:
                response = runtime.apply(request)
            except MolaProcessError as retry_exc:
                self._invalidate_runtime(peer_root)
                raise WorkerError(str(retry_exc)) from retry_exc
        except MolaProcessError as exc:
            self._invalidate_runtime(peer_root)
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
            self._invalidate_runtime(peer_root)
            raise
        self._resident_by_peer.setdefault(peer_root, set()).add(map_id)
        return response

    def _release_inactive_maps(self, peer_root: Path, active: set[str]) -> None:
        known = self._resident_by_peer.get(peer_root, set())
        inactive = known - active
        runtime = self._runtimes.get(peer_root)
        if runtime is not None and inactive:
            try:
                for map_id in sorted(inactive):
                    runtime.release(map_id)
            except MolaProcessError:
                # Publication has already committed. Restarting this peer's
                # runtime safely bounds its native state if targeted
                # reclamation fails.
                self._invalidate_runtime(peer_root)
                return
        remaining = known & active
        if remaining:
            self._resident_by_peer[peer_root] = remaining
        else:
            self._resident_by_peer.pop(peer_root, None)

    def process_peer(self, peer_root: Path) -> ProcessResult:
        """Build and publish one generation from the peer's current snapshot.

        The snapshot bytes are read once at the start; the generation built
        from them is published whatever ``snapshot.json`` contains by the end
        (see the module docstring for why). Publication order is artifacts,
        then ``source.json`` holding those exact bytes, then ``index.json``.

        After a published build the resident native state is the published
        generation, so the next pose-only correction for a component with
        unchanged geometry bases on the generation ``index.json`` describes.
        A build that fails part way keeps the previous ``index.json`` and
        ``source.json`` and invalidates this peer's native runtime wherever
        native state may have advanced past the published generation (process
        failures, response or artifact mismatches, disappearing files); the
        next attempt then rebuilds from immutable chunks.

        This method touches only the peer's own directories, its own runtime
        and its own entries of the per-peer dicts, so ``run_once`` may call it
        for different peers on different threads at the same time.
        """

        peer_root = Path(peer_root)
        source = peer_root / "snapshot.json"
        # Read (up to MAX_SNAPSHOT_BYTES) and parsed before map_epoch_lock,
        # which the bridge's authority heartbeat takes before every send.
        # Reading it first is still safe: a snapshot read before a new epoch
        # was claimed names the retired run_id and is rejected below, and the
        # lifetime is re-checked under the lock again before publication.
        raw, snapshot = _read_snapshot(source)
        dependencies = snapshot_epoch_dependencies(snapshot)
        with map_epoch_lock(peer_root):
            lifetime = read_map_epoch(peer_root)
            chunks = (peer_root / "geometry" / "chunks").resolve()
            if not chunks.is_dir():
                raise WorkerError("geometry/chunks directory is missing")
            assert_map_epoch_dependencies(peer_root, dependencies)
        run_id = None if lifetime is None else lifetime["run_id"]
        if self._peer_runs.get(peer_root) != run_id:
            self._invalidate_runtime(peer_root)
            self._peer_runs[peer_root] = run_id
        if lifetime is not None:
            if snapshot.get("run_id") != run_id:
                raise WorkerError("snapshot belongs to a retired robot map epoch")
            for manifest in snapshot["manifests"]:
                for submap in manifest["submaps"]:
                    for key in submap["keyframes"]:
                        if (
                            key["robot_id"] == lifetime["robot_id"]
                            and key["session_id"] != run_id
                        ):
                            raise WorkerError(
                                "snapshot belongs to a retired robot map epoch"
                            )
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
                    and (
                        not self.planner_maps or self._planner_matches(mola_root, prior)
                    )
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
                planner_path = (
                    output_path.with_suffix(".sdpg") if self.planner_maps else None
                )
                map_id = self._map_id(peer_root, component_id)
                request_mode = (
                    "pose_only"
                    if map_id in self._resident_by_peer.get(peer_root, ())
                    and prior is not None
                    and prior.get("geometry_fingerprint") == geometry_fingerprint
                    else "replace"
                )
                response: dict[str, object] | None = None
                if self.mode == "persistent":
                    response = self._persistent_import(
                        peer_root=peer_root,
                        mode=request_mode,
                        map_id=map_id,
                        input_path=input_path,
                        input_sha=input_sha,
                        chunks=chunks,
                        output_path=output_path,
                        planner_output_path=planner_path,
                        snapshot_id=snapshot_id,
                        manifest=manifest,
                    )
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
                if response is not None:
                    # `_validate_runtime_response` already checked
                    # `output_size_bytes` and `output_sha256` are well-formed;
                    # the native runtime computed that hash itself from the
                    # exact bytes it wrote, so trust it rather than re-reading
                    # and re-hashing what may be a multi-megabyte artifact
                    # only to compare against a hash that came from the same
                    # process in the first place. `stat` still catches a
                    # truncated, missing or oversized write.
                    size, digest = response["output_size_bytes"], response["output_sha256"]
                    if not _stat_matches(output_path, size, self.max_output_bytes):
                        self._invalidate_runtime(peer_root)
                        raise WorkerError("native artifact does not match its response")
                else:
                    size, digest = _sha256_file(output_path, self.max_output_bytes)
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
                if planner_path is not None:
                    assert response is not None
                    # `_validate_runtime_response` does not cover the planner
                    # fields (only sent when planner_maps is on), so their
                    # shape is checked here before trusting them the same way.
                    planner_size = response.get("planner_output_size_bytes")
                    planner_sha = response.get("planner_output_sha256")
                    if (
                        not isinstance(planner_sha, str)
                        or len(planner_sha) != 64
                        or any(c not in "0123456789abcdef" for c in planner_sha)
                        or not _stat_matches(
                            planner_path, planner_size, self.max_output_bytes
                        )
                    ):
                        self._invalidate_runtime(peer_root)
                        raise WorkerError(
                            "native planner artifact does not match its response"
                        )
                    planner_final = components_root / planner_path.name
                    staged.append((planner_path, planner_final))
                    artifacts[-1]["planner"] = {
                        "path": f"components/{planner_path.name}",
                        "size_bytes": planner_size,
                        "sha256": planner_sha,
                        "source_sha256": input_sha,
                        "source_snapshot_id": snapshot_id,
                    }

            # source.json and index.json are written and fsynced into the
            # staging directory first; under map_epoch_lock only renames and
            # one directory fsync remain.
            _write_fsynced(staging / "source.json", raw)
            index = {
                "version": 1,
                "source_snapshot_id": snapshot_id,
                "source_sha256": source_sha,
                "generated_at_ns": time.time_ns(),
                "artifacts": artifacts,
            }
            _write_fsynced(staging / "index.json", _json_bytes(index))
            refused = None
            with map_epoch_lock(peer_root):
                if read_map_epoch(peer_root) != lifetime:
                    refused = "robot map epoch advanced during native build"
                else:
                    try:
                        assert_map_epoch_dependencies(peer_root, dependencies)
                    except (OSError, ValueError):
                        refused = "peer map epoch advanced during native build"
                if refused is None:
                    for staged_path, final_path in staged:
                        os.replace(staged_path, final_path)
                    # A newer graph revision may supersede this build, but a
                    # new robot lifetime may never receive an old build's
                    # geometry.
                    os.replace(staging / "source.json", mola_root / "source.json")
                    # source.json is durable before index.json names it.
                    _fsync_directory(mola_root)
                    os.replace(staging / "index.json", mola_root / "index.json")
            if refused is not None:
                # Decided under the lock, torn down after it: runtime.close()
                # may wait on the native process, and the bridge's heartbeat
                # needs map_epoch_lock.
                self._invalidate_runtime(peer_root)
                raise WorkerError(refused)
            _fsync_directory(mola_root)
            self._prune(
                components_root,
                {mola_root / str(item["path"]) for item in artifacts},
            )
            active_map_ids = {
                self._map_id(peer_root, str(manifest["graph_revision"]["component_id"]))
                for manifest in manifests
            }
            self._release_inactive_maps(peer_root, active_map_ids)
            return ProcessResult(True, snapshot_id, len(manifests), source_sha)
        except FileNotFoundError as exc:
            self._invalidate_runtime(peer_root)
            raise WorkerError("snapshot or native artifact disappeared") from exc
        except OSError:
            self._invalidate_runtime(peer_root)
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
            path.with_suffix(".sdpg").unlink(missing_ok=True)
        for path in directory.glob("*.sdpg"):
            if not path.with_suffix(".metricmap").exists():
                path.unlink(missing_ok=True)

    def _attempt(self, peer: Path) -> ProcessResult | str:
        """Build one peer on the calling thread; a handled failure is its message."""

        try:
            return self.process_peer(peer)
        except (WorkerError, OSError, KeyError, TypeError, ValueError) as exc:
            return f"{type(exc).__name__}: {exc}"

    @staticmethod
    def _write_worker_status(peer: Path, source_sha: str, error: str) -> None:
        """Record the last attempt's outcome in ``<peer>/mola/worker.json``.

        The bridge reports ``error`` as ``product_error`` in the peer's
        ``status.json``, so a component that outgrows the point budget (its
        product frozen at the last revision that fit) is visible to the
        operator rather than only in this process's log. A peer whose
        directory is gone is not worth a failure of its own.
        """

        mola_root = peer / "mola"
        status = {
            "version": WORKER_STATUS_VERSION,
            "updated_at_ns": time.time_ns(),
            "source_sha256": source_sha,
            "error": error[:WORKER_STATUS_MAX_ERROR_CHARS],
        }
        try:
            mola_root.mkdir(exist_ok=True)
            _atomic_json(mola_root / "worker.json", status)
        except OSError:
            pass

    def _log_outcome(self, peer: Path, error: str | None) -> None:
        """Log a peer's failure when it appears or changes, and its recovery."""

        previous = self._logged_errors.get(peer)
        if error is None:
            if previous is not None:
                self._logged_errors.pop(peer, None)
                print(f"swarmdeck-mola-worker: {peer}: published again", flush=True)
            return
        if error != previous:
            self._logged_errors[peer] = error
            print(f"swarmdeck-mola-worker: {peer}: {error}", flush=True)

    def run_once(self) -> dict[Path, str]:
        """Process changed peers once; errors are retained for status/logging.

        Which peers need a build is decided here, on the calling thread. The
        builds then run concurrently, up to ``parallel_peers`` at a time, each
        confined to its own peer's directories and native runtime; their
        outcomes update ``_completed``, ``_retry_after``, ``mola/worker.json``
        and the returned errors back on the calling thread. With
        ``parallel_peers == 1`` (or a single due peer) the builds run in peer
        order on the calling thread. Every failure is returned; it is logged
        only when it differs from what was last logged for that peer.
        """

        errors: dict[Path, str] = {}
        now = time.monotonic()
        peers = self.discover()
        known = (
            self._runtimes.keys()
            | self._resident_by_peer.keys()
            | self._completed.keys()
            | self._retry_after.keys()
            | self._logged_errors.keys()
        )
        for missing_peer in known - set(peers):
            self._invalidate_runtime(missing_peer)
            self._completed.pop(missing_peer, None)
            self._retry_after.pop(missing_peer, None)
            self._logged_errors.pop(missing_peer, None)
            self._peer_runs.pop(missing_peer, None)
            self._snapshot_identity.pop(missing_peer, None)
        due: list[tuple[Path, str, tuple[int, int, int, int] | None]] = []
        for peer in peers:
            source = peer / "snapshot.json"
            identity = _file_identity(source)
            if (
                identity is not None
                and identity == self._snapshot_identity.get(peer)
                and peer in self._completed
            ):
                # snapshot.json's size, inode and mtime have not changed
                # since the last completed build read it: a parked peer's
                # common case. `os.replace` (the only way this file is
                # written) always changes its identity, so this is exact,
                # not a heuristic, and skips reading and hashing the whole
                # file for nothing.
                continue
            try:
                raw = _bounded_file_bytes(source, MAX_SNAPSHOT_BYTES, "snapshot")
                source_sha = hashlib.sha256(raw).hexdigest()
            except (OSError, WorkerError) as exc:
                errors[peer] = str(exc)
                self._write_worker_status(peer, "", errors[peer])
                self._log_outcome(peer, errors[peer])
                continue
            if self._completed.get(peer) == source_sha:
                # Confirmed by actually reading it, not merely by `stat`: safe
                # to trust `stat` alone the next time this exact identity
                # recurs.
                if identity is not None:
                    self._snapshot_identity[peer] = identity
                continue
            if now < self._retry_after.get(peer, 0):
                continue
            due.append((peer, source_sha, identity))

        def record(
            peer: Path,
            source_sha: str,
            identity: tuple[int, int, int, int] | None,
            outcome: ProcessResult | str,
        ) -> None:
            if isinstance(outcome, ProcessResult):
                # Record the bytes the product was built from, which process_peer
                # read itself. If snapshot.json moved on meanwhile, the next poll
                # sees a digest that differs from this one and builds it.
                self._completed[peer] = outcome.source_sha256
                self._retry_after.pop(peer, None)
                self._write_worker_status(peer, outcome.source_sha256, "")
                self._log_outcome(peer, None)
                if identity is not None and outcome.source_sha256 == source_sha:
                    # This poll's `stat` and content agreed with what was
                    # just published: the next poll may trust that identity
                    # alone. A snapshot that moved on between this read and
                    # process_peer's own leaves no identity cached, so the
                    # next poll reads and hashes for real.
                    self._snapshot_identity[peer] = identity
                else:
                    self._snapshot_identity.pop(peer, None)
            else:
                errors[peer] = outcome
                self._retry_after[peer] = now + self.retry_s
                self._write_worker_status(peer, source_sha, outcome)
                self._log_outcome(peer, outcome)
                self._snapshot_identity.pop(peer, None)

        if self.parallel_peers == 1 or len(due) < 2:
            for peer, source_sha, identity in due:
                record(peer, source_sha, identity, self._attempt(peer))
            return errors
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=min(self.parallel_peers, len(due)),
            thread_name_prefix="swarmdeck-mola-build",
        ) as pool:
            futures = [
                (peer, source_sha, identity, pool.submit(self._attempt, peer))
                for peer, source_sha, identity in due
            ]
            for peer, source_sha, identity, future in futures:
                record(peer, source_sha, identity, future.result())
        return errors

    def run_forever(self) -> None:
        try:
            while True:
                self.run_once()
                time.sleep(self.poll_s)
        finally:
            self.close()

    def close(self) -> None:
        """Close every peer's native runtime."""

        for peer_root in tuple(self._runtimes):
            self._invalidate_runtime(peer_root)
        self._resident_by_peer.clear()


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{value!r} is not an integer") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


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
        "--planner-maps",
        action="store_true",
        default=os.getenv("SWARMDECK_MOLA_PLANNER_MAPS", "false").lower() == "true",
        help="publish native planner grids alongside MOLA metric maps",
    )
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
            os.getenv("SWARMDECK_MOLA_MAX_POINTS_PER_MAP", DEFAULT_MAX_POINTS_PER_MAP)
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
        "--parallel-peers",
        type=_positive_int,
        # A string default goes through the type conversion, so an invalid
        # environment value is rejected like an invalid flag.
        default=os.getenv("SWARMDECK_MOLA_PARALLEL_PEERS", str(DEFAULT_PARALLEL_PEERS)),
        help=(
            "build up to this many peers' products at the same time, one native "
            "runtime per peer (defaults to SWARMDECK_MOLA_PARALLEL_PEERS or "
            f"{DEFAULT_PARALLEL_PEERS}; minimum 1)"
        ),
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
        planner_maps=args.planner_maps,
        mission_id=None if args.all_missions else args.mission_id,
        parallel_peers=args.parallel_peers,
    ).run_forever()


if __name__ == "__main__":
    main()
