"""Verified provider for native MOLA planner-grid publications.

The worker publishes a self-described product under ``<peer>/mola/``: the
component artifacts, then ``source.json`` (the exact snapshot bytes the
generation was built from), then ``index.json``. This provider reads
``index.json`` and ``source.json`` and requires ``sha256(source.json) ==
index.source_sha256`` and ``source.snapshot_id == index.source_snapshot_id``.
A mismatch means the pair is mid-replacement, reported as
:class:`PublicationPending` so the registry retries on its next poll. The
provider never reads ``snapshot.json``: that file is the bridge's newest input
and may be ahead of the product, and serving the product it does describe is
the point of the protocol (see ``deploy/autonomy/mola_worker.py``).
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import struct
import time
from pathlib import Path
from typing import Mapping

from .contracts import SCHEMA_VERSION
from .indexed_mapping import IndexedGrid, IndexedMapView, SnapshotKey
from .map_provider import PublicationPending

GRID_MAGIC = b"SDMGRID1"
GRID_SCHEMA = "swarmdeck.mola_planner_grid.v1"
MAX_INDEX_BYTES = 4 * 1024 * 1024
MAX_GRID_BYTES = 256 * 1024 * 1024
MAX_METADATA_BYTES = 64 * 1024
MAX_POINTS = 1_000_000
MAX_VOXELS = 2_000_000
MAX_RAY_STEPS = 4_000_000
MAX_COMPONENTS = 256

_COUNT = struct.Struct("<I")
_VOXEL = struct.Struct("<qqq")
_SURFACE = struct.Struct("<qqd")
_METADATA_FIELDS = {
    "schema",
    "graph_version",
    "identity",
    "source_stamp_ns",
    "resolution_m",
    "ray_angular_resolution_rad",
    "ray_step_fraction",
    "point_count",
    "occupied_count",
    "free_count",
    "surface_count",
    "retired_count",
    "ray_steps",
    "qualified_ray_keyframes",
}


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _sha(value: object, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        raise ValueError(f"{field} must be lowercase SHA-256")
    return value


def _uint(value: object, field: str, *, maximum: int | None = None) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    if maximum is not None and value > maximum:
        raise ValueError(f"{field} exceeds its limit")
    return value


def _object(value: object, field: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _canonical(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()


def _json(raw: bytes, field: str) -> dict[str, object]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"nonfinite JSON constant in {field}: {value}")

    def reject_duplicate_keys(
        pairs: list[tuple[str, object]],
    ) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key in {field}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            raw,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field} is not valid JSON") from exc
    return _object(value, field)


def _stat_identity(value: os.stat_result) -> tuple[int, int, int, int, int]:
    """Identity fields that change for replacement and in-place mutation."""

    return (
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mtime_ns,
        value.st_ctime_ns,
    )


def _bounded_stable_read_with_identity(
    path: Path, maximum: int, field: str
) -> tuple[bytes, tuple[int, int, int, int, int]]:
    before = path.stat()
    if (
        not stat.S_ISREG(before.st_mode)
        or before.st_size <= 0
        or before.st_size > maximum
    ):
        raise ValueError(f"{field} has invalid size")
    with path.open("rb") as stream:
        opened = os.fstat(stream.fileno())
        raw = stream.read(maximum + 1)
        after = os.fstat(stream.fileno())
    identity_before = _stat_identity(before)
    identity_opened = _stat_identity(opened)
    identity_after = _stat_identity(after)
    if identity_before != identity_opened or identity_opened != identity_after:
        raise PublicationPending(f"{field} changed while reading")
    if len(raw) != before.st_size:
        raise PublicationPending(f"{field} changed while reading")
    return raw, identity_after


def _bounded_stable_read(path: Path, maximum: int, field: str) -> bytes:
    return _bounded_stable_read_with_identity(path, maximum, field)[0]


def _worker_manifest_digest(manifest: Mapping[str, object]) -> str:
    return _sha256(_canonical(manifest))


def _qualified_ray_keyframes(manifest: Mapping[str, object]) -> int:
    submaps = manifest.get("submaps")
    if not isinstance(submaps, list):
        raise ValueError("manifest submaps must be a list")
    qualified = 0
    for raw in submaps:
        submap = _object(raw, "submap")
        evidence = submap.get("ray_evidence")
        origins = submap.get("sensor_origins")
        if (
            isinstance(evidence, dict)
            and evidence.get("return_semantics") == "first_return"
            and evidence.get("deskew") in ("deskewed", "not_required")
            and evidence.get("origin_association") == "single_capture"
            and isinstance(origins, list)
            and len(origins) == 1
        ):
            qualified += 1
    return qualified


def _snapshot(raw: bytes) -> tuple[dict[str, object], dict[str, dict[str, object]]]:
    """Validate the published source, which is a snapshot document by content."""

    value = _json(raw, "MOLA source")
    if value.get("schema") != SCHEMA_VERSION:
        raise ValueError("unsupported snapshot schema")
    manifests = value.get("manifests")
    if not isinstance(manifests, list):
        raise ValueError("snapshot manifests must be a list")
    if len(manifests) > MAX_COMPONENTS:
        raise ValueError("snapshot exceeds the component limit")
    canonical_manifests = []
    for item in manifests:
        manifest = _object(item, "manifest")
        if manifest.get("schema", SCHEMA_VERSION) != SCHEMA_VERSION:
            raise ValueError("unsupported manifest schema")
        canonical_manifests.append({**manifest, "schema": SCHEMA_VERSION})
    snapshot_id = _sha(value.get("snapshot_id"), "snapshot_id")
    if snapshot_id != _sha256(_canonical(canonical_manifests)):
        raise ValueError("snapshot_id does not match canonical manifests")
    by_component: dict[str, dict[str, object]] = {}
    for raw_manifest in manifests:
        manifest = _object(raw_manifest, "manifest")
        revision = _object(manifest.get("graph_revision"), "graph_revision")
        component = revision.get("component_id")
        if not isinstance(component, str) or not component:
            raise ValueError("manifest component_id is invalid")
        if component in by_component:
            raise ValueError("snapshot repeats a component_id")
        by_component[component] = manifest
    return value, by_component


class MolaDirectorySource:
    """Import native grids without rereading or reconstructing XYZ chunks."""

    def __init__(
        self,
        peer_root: str | Path,
        *,
        max_index_bytes: int = MAX_INDEX_BYTES,
        max_grid_bytes: int = MAX_GRID_BYTES,
        clock_ns=time.monotonic_ns,
    ):
        for value, maximum, field in (
            (max_index_bytes, MAX_INDEX_BYTES, "max_index_bytes"),
            (max_grid_bytes, MAX_GRID_BYTES, "max_grid_bytes"),
        ):
            if (
                not isinstance(value, int)
                or isinstance(value, bool)
                or value <= 0
                or value > maximum
            ):
                raise ValueError(f"{field} must be a positive bounded integer")
        self.peer_root = Path(peer_root)
        # The product's own copy of its source, not the bridge's snapshot.json.
        self.source_path = self.peer_root / "mola" / "source.json"
        self.publication_path = self.peer_root / "mola" / "index.json"
        self.components_path = self.peer_root / "mola" / "components"
        self.max_index_bytes = max_index_bytes
        self.max_grid_bytes = max_grid_bytes
        self._clock_ns = clock_ns
        self._cache: dict[str, tuple[tuple[object, ...], IndexedGrid]] = {}

    def _publication(
        self,
    ) -> tuple[
        bytes,
        bytes,
        tuple[int, int, int, int, int],
        tuple[int, int, int, int, int],
        dict[str, object],
        dict[str, dict[str, object]],
        dict[str, dict[str, object]],
    ]:
        # index.json is replaced last, so read it first: a source.json that
        # then disagrees with it is either the older generation still in
        # place or the newer one already there. Either way the digest check
        # below reports the pair as pending and the next poll reads again.
        index_raw, index_identity = _bounded_stable_read_with_identity(
            self.publication_path, self.max_index_bytes, "MOLA index"
        )
        source_raw, source_identity = _bounded_stable_read_with_identity(
            self.source_path, MAX_INDEX_BYTES, "MOLA source"
        )
        index = _json(index_raw, "MOLA index")
        if index.get("version") != 1:
            raise ValueError("unsupported MOLA index version")
        if index.get("source_sha256") != _sha256(source_raw):
            raise PublicationPending(
                "MOLA index does not match its published source bytes"
            )
        snapshot, manifests = _snapshot(source_raw)
        if index.get("source_snapshot_id") != snapshot.get("snapshot_id"):
            raise PublicationPending(
                "MOLA index does not match its published source identity"
            )
        artifacts = index.get("artifacts")
        if not isinstance(artifacts, list) or len(artifacts) > MAX_COMPONENTS:
            raise ValueError("MOLA index artifacts must be a list")
        by_component: dict[str, dict[str, object]] = {}
        for raw_item in artifacts:
            item = _object(raw_item, "MOLA artifact")
            component = item.get("component_id")
            if not isinstance(component, str) or component not in manifests:
                raise ValueError("MOLA artifact component_id is invalid")
            if component in by_component:
                raise ValueError("MOLA index repeats a component_id")
            manifest = manifests[component]
            revision = _object(manifest.get("graph_revision"), "graph_revision")
            epoch = _uint(revision.get("epoch"), "manifest epoch")
            graph_revision = _uint(revision.get("revision"), "manifest graph revision")
            geometry_revision = _sha(
                manifest.get("geometry_revision"), "manifest geometry_revision"
            )
            expected = {
                "epoch": epoch,
                "revision": graph_revision,
                "geometry_revision": geometry_revision,
                "manifest_sha256": _worker_manifest_digest(manifest),
            }
            if any(item.get(name) != value for name, value in expected.items()):
                raise ValueError("MOLA artifact does not match current manifest")
            if not isinstance(item.get("planner"), dict):
                raise ValueError("MOLA planner grid is absent")
            by_component[component] = item
        if set(by_component) != set(manifests):
            raise ValueError("MOLA index does not cover the current components")
        for component in set(self._cache) - set(manifests):
            self._cache.pop(component, None)
        return (
            source_raw,
            index_raw,
            source_identity,
            index_identity,
            snapshot,
            manifests,
            by_component,
        )

    def component_ids(self) -> tuple[str, ...]:
        _, _, _, _, _, manifests, _ = self._publication()
        return tuple(sorted(manifests))

    def signature(self) -> tuple[int, ...]:
        values: list[int] = []
        for path in (self.source_path, self.publication_path):
            stat = path.stat()
            values.extend((stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns))
        return tuple(values)

    def refresh(self, view: IndexedMapView, component_id: str) -> SnapshotKey:
        try:
            (
                source_raw,
                index_raw,
                source_identity,
                index_identity,
                _,
                manifests,
                artifacts,
            ) = self._publication()
            manifest = manifests.get(component_id)
            item = artifacts.get(component_id)
            if manifest is None or item is None:
                raise ValueError("component is absent from MOLA publication")
            planner = _object(item.get("planner"), "planner grid record")
            path, expected_size, digest = self._planner_descriptor(planner)
            artifact_identity = self._artifact_identity(path, expected_size)
            cache_identity = (
                str(path),
                expected_size,
                digest,
                planner.get("source_snapshot_id"),
                planner.get("source_sha256"),
                _worker_manifest_digest(manifest),
                artifact_identity,
            )
            cached = self._cache.get(component_id)
            grid = (
                cached[1]
                if cached is not None and cached[0] == cache_identity
                else None
            )
            if grid is None:
                raw = self._planner_bytes(path, expected_size, digest)
                grid = self._decode_grid(raw, planner, manifest)
                self._cache[component_id] = (cache_identity, grid)
            if (
                not self._publication_unchanged(
                    source_raw,
                    index_raw,
                    source_identity,
                    index_identity,
                )
                or self._artifact_identity(path, expected_size) != artifact_identity
            ):
                raise PublicationPending("MOLA publication changed while reading")
            return view.publish(
                grid.refreshed(
                    source_stamp_ns=grid.source_stamp_ns,
                    received_monotonic_ns=self._clock_ns(),
                )
            )
        except Exception as exc:
            view.invalidate(str(exc))
            raise

    def _publication_unchanged(
        self,
        source_raw: bytes,
        index_raw: bytes,
        source_identity: tuple[int, int, int, int, int],
        index_identity: tuple[int, int, int, int, int],
    ) -> bool:
        current_index, current_index_identity = _bounded_stable_read_with_identity(
            self.publication_path, self.max_index_bytes, "MOLA index"
        )
        current_source, current_source_identity = _bounded_stable_read_with_identity(
            self.source_path, MAX_INDEX_BYTES, "MOLA source"
        )
        # Re-stat both after the pair has been read. This closes the window in
        # which source.json could be replaced while index.json was checked.
        final_source_identity = _stat_identity(self.source_path.stat())
        final_index_identity = _stat_identity(self.publication_path.stat())
        return (
            current_source == source_raw
            and current_index == index_raw
            and current_source_identity == source_identity
            and current_index_identity == index_identity
            and final_source_identity == source_identity
            and final_index_identity == index_identity
        )

    def _planner_descriptor(
        self, planner: Mapping[str, object]
    ) -> tuple[Path, int, str]:
        relative = planner.get("path")
        if not isinstance(relative, str) or not relative:
            raise ValueError("planner grid path is invalid")
        path = (self.peer_root / "mola" / relative).resolve()
        if path.parent != self.components_path.resolve() or path.suffix != ".sdpg":
            raise ValueError("planner grid path escapes the component directory")
        expected_size = _uint(
            planner.get("size_bytes"), "planner size_bytes", maximum=self.max_grid_bytes
        )
        if expected_size == 0:
            raise ValueError("planner size_bytes must be positive")
        expected_sha = _sha(planner.get("sha256"), "planner sha256")
        return path, expected_size, expected_sha

    def _artifact_identity(
        self, path: Path, expected_size: int
    ) -> tuple[int, int, int, int, int]:
        value = path.stat()
        if not stat.S_ISREG(value.st_mode) or value.st_size != expected_size:
            raise ValueError("planner grid has invalid size")
        return _stat_identity(value)

    def _planner_bytes(
        self, path: Path, expected_size: int, expected_sha: str
    ) -> bytes:
        raw = _bounded_stable_read(path, self.max_grid_bytes, "planner grid")
        if len(raw) != expected_size or _sha256(raw) != expected_sha:
            raise ValueError("planner grid integrity check failed")
        return raw

    def _decode_grid(
        self,
        raw: bytes,
        planner: Mapping[str, object],
        manifest: Mapping[str, object],
    ) -> IndexedGrid:
        if len(raw) < len(GRID_MAGIC) + _COUNT.size or raw[:8] != GRID_MAGIC:
            raise ValueError("planner grid magic is invalid")
        metadata_size = _COUNT.unpack_from(raw, 8)[0]
        if metadata_size <= 0 or metadata_size > MAX_METADATA_BYTES:
            raise ValueError("planner grid metadata size is invalid")
        body_offset = 8 + _COUNT.size + metadata_size
        if body_offset > len(raw):
            raise ValueError("planner grid metadata is truncated")
        metadata_raw = raw[12:body_offset]
        metadata = _json(metadata_raw, "planner grid metadata")
        if set(metadata) != _METADATA_FIELDS or metadata.get("schema") != GRID_SCHEMA:
            raise ValueError("planner grid metadata schema is invalid")

        graph = _object(metadata.get("graph_version"), "graph_version")
        identity = _object(metadata.get("identity"), "identity")
        if set(graph) != {"component_id", "epoch", "revision", "digest"}:
            raise ValueError("planner graph_version fields are invalid")
        if set(identity) != {
            "geometry_revision",
            "native_geometry_digest",
            "canonical_manifest_digest",
            "source_snapshot_id",
            "source_sha256",
            "reference_frame",
        }:
            raise ValueError("planner identity fields are invalid")
        revision = _object(manifest.get("graph_revision"), "graph_revision")
        expected_graph = {
            "component_id": revision.get("component_id"),
            "epoch": revision.get("epoch"),
            "revision": revision.get("revision"),
        }
        expected_identity = {
            "geometry_revision": manifest.get("geometry_revision"),
            "source_snapshot_id": planner.get("source_snapshot_id"),
            "source_sha256": planner.get("source_sha256"),
            "reference_frame": manifest.get("frame_id"),
        }
        if any(
            graph.get(name) != value for name, value in expected_graph.items()
        ) or any(
            identity.get(name) != value for name, value in expected_identity.items()
        ):
            raise ValueError("planner grid identity does not match its source")
        _sha(graph.get("digest"), "planner graph digest")
        _sha(
            identity.get("native_geometry_digest"),
            "planner native_geometry_digest",
        )
        _sha(
            identity.get("canonical_manifest_digest"),
            "planner canonical_manifest_digest",
        )
        _sha(planner.get("source_snapshot_id"), "planner source_snapshot_id")
        _sha(planner.get("source_sha256"), "planner source_sha256")

        source_stamp_ns = _uint(metadata.get("source_stamp_ns"), "source_stamp_ns")
        point_count = _uint(
            metadata.get("point_count"), "point_count", maximum=MAX_POINTS
        )
        occupied_count = _uint(
            metadata.get("occupied_count"), "occupied_count", maximum=MAX_VOXELS
        )
        free_count = _uint(metadata.get("free_count"), "free_count", maximum=MAX_VOXELS)
        if occupied_count + free_count > MAX_VOXELS:
            raise ValueError("planner voxel count exceeds its limit")
        surface_count = _uint(
            metadata.get("surface_count"), "surface_count", maximum=MAX_POINTS
        )
        retired_count = _uint(
            metadata.get("retired_count"), "retired_count", maximum=MAX_POINTS
        )
        ray_steps = _uint(metadata.get("ray_steps"), "ray_steps", maximum=MAX_RAY_STEPS)
        qualified = _uint(
            metadata.get("qualified_ray_keyframes"), "qualified_ray_keyframes"
        )
        submaps = manifest.get("submaps")
        if not isinstance(submaps, list):
            raise ValueError("manifest submaps must be a list")
        expected_points = 0
        expected_stamp = 0
        for raw_submap in submaps:
            submap = _object(raw_submap, "submap")
            expected_stamp = max(
                expected_stamp,
                _uint(submap.get("observed_at_ns", 0), "observed_at_ns"),
            )
            chunks = submap.get("chunks")
            if not isinstance(chunks, list):
                raise ValueError("submap chunks must be a list")
            for raw_chunk in chunks:
                chunk = _object(raw_chunk, "chunk")
                expected_points += _uint(chunk.get("point_count"), "chunk point_count")
                if expected_points > MAX_POINTS:
                    raise ValueError("manifest point count exceeds its limit")
        if point_count != expected_points or source_stamp_ns != expected_stamp:
            raise ValueError(
                "planner point count or source stamp does not match manifest"
            )
        # point_count stays the manifest's stored point count. Every stored
        # point is either a surface sample or an endpoint the builder retired
        # because later qualified rays saw through its voxel.
        if surface_count + retired_count != point_count:
            raise ValueError(
                "planner surface and retired counts do not match point count"
            )
        if qualified != _qualified_ray_keyframes(manifest):
            raise ValueError("planner qualified-ray count does not match manifest")
        # Retirement rests on the same evidence free space does: only a
        # qualified capture's rays can prove an endpoint was seen through.
        if qualified == 0 and (free_count != 0 or ray_steps != 0 or retired_count != 0):
            raise ValueError("planner free space lacks qualified ray evidence")

        resolution = metadata.get("resolution_m")
        angular = metadata.get("ray_angular_resolution_rad")
        fraction = metadata.get("ray_step_fraction")
        if (
            not isinstance(resolution, (int, float))
            or isinstance(resolution, bool)
            or not math.isfinite(float(resolution))
            or float(resolution) <= 0
            or not isinstance(angular, (int, float))
            or isinstance(angular, bool)
            or not math.isclose(
                float(angular), math.radians(5.0), rel_tol=0.0, abs_tol=1e-12
            )
            or not isinstance(fraction, (int, float))
            or isinstance(fraction, bool)
            or float(fraction) != 0.75
        ):
            raise ValueError("planner grid options are unsupported")

        expected_body = (
            occupied_count * _VOXEL.size
            + free_count * _VOXEL.size
            + surface_count * _SURFACE.size
        )
        if body_offset + expected_body != len(raw):
            raise ValueError("planner grid body size does not match its counts")
        offset = body_offset
        occupied, offset = self._voxels(raw, offset, occupied_count, "occupied")
        free, offset = self._voxels(raw, offset, free_count, "free")
        if occupied & free:
            raise ValueError("planner occupied and free voxels overlap")
        columns: dict[tuple[int, int], list[float]] = {}
        prior_surface: tuple[int, int, float] | None = None
        for _ in range(surface_count):
            x, y, z = _SURFACE.unpack_from(raw, offset)
            offset += _SURFACE.size
            value = (x, y, z)
            if not math.isfinite(z):
                raise ValueError("planner surface contains a nonfinite height")
            if prior_surface is not None and value < prior_surface:
                raise ValueError("planner surface records are not sorted")
            surface_voxel = (x, y, math.floor(z / float(resolution)))
            # Retirement drops a voxel's surface samples together with the
            # voxel itself, so every surviving sample still stands on an
            # occupied voxel. A retired voxel appears in the free set instead,
            # which the occupied/free overlap check above already separates.
            if surface_voxel not in occupied:
                raise ValueError("planner surface has no occupied endpoint")
            prior_surface = value
            columns.setdefault((x, y), []).append(z)
        if offset != len(raw):
            raise ValueError("planner grid contains trailing bytes")

        key = SnapshotKey(
            str(graph["component_id"]),
            _uint(graph["epoch"], "epoch"),
            _uint(graph["revision"], "revision"),
            str(identity["geometry_revision"]),
        )
        return IndexedGrid(
            key,
            str(identity["canonical_manifest_digest"]),
            source_stamp_ns,
            0,
            occupied,
            free,
            columns,
            float(resolution),
            point_count,
        )

    @staticmethod
    def _voxels(
        raw: bytes, offset: int, count: int, field: str
    ) -> tuple[set[tuple[int, int, int]], int]:
        result: set[tuple[int, int, int]] = set()
        prior: tuple[int, int, int] | None = None
        for _ in range(count):
            value = _VOXEL.unpack_from(raw, offset)
            offset += _VOXEL.size
            if prior is not None and value <= prior:
                raise ValueError(f"planner {field} voxels are not strictly sorted")
            prior = value
            result.add(value)
        return result, offset
