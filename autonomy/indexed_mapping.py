"""Bounded, correction-aware batch queries over coherent map snapshots.

The native MOLA bridge and this query view consume the same immutable XYZ
chunks and corrected submap poses. MOLA owns metric-map serialization; this
module builds a compact voxel index for the planner's one batched final route
check. It deliberately fails closed when source freshness or work bounds cannot
be proven.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass, replace
from enum import IntEnum
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np

from .contracts import (
    MapManifest,
    MapSnapshot,
    Matrix4,
    SCHEMA_VERSION,
    SubmapId,
    validate_se3,
)
from .mapping import MAX_CHUNK_BYTES, XYZ_F32_ENCODING, decode_xyz_f32


class QueryStatus(IntEnum):
    OK = 0
    STALE = 1
    UNAVAILABLE = 2


class VoxelOccupancy(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


@dataclass(frozen=True)
class SnapshotKey:
    component_id: str
    epoch: int
    graph_revision: int
    geometry_revision: str

    def __post_init__(self) -> None:
        if not isinstance(self.component_id, str) or not self.component_id:
            raise ValueError("component_id must be nonempty")
        for name in ("epoch", "graph_revision"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if (
            not isinstance(self.geometry_revision, str)
            or len(self.geometry_revision) != 64
            or any(c not in "0123456789abcdef" for c in self.geometry_revision)
        ):
            raise ValueError("geometry_revision must be lowercase SHA-256")


@dataclass(frozen=True)
class QueryRequest:
    key: SnapshotKey
    samples: tuple[tuple[float, float, float], ...]
    body_size_xyz: tuple[float, float, float]
    stop_at_unknown: bool = True
    source_stamp_ns: int | None = None
    now_monotonic_ns: int | None = None
    max_snapshot_age_ns: int | None = None

    def __post_init__(self) -> None:
        points = tuple(tuple(float(v) for v in p) for p in self.samples)
        if not points or any(
            len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points
        ):
            raise ValueError("samples must contain finite XYZ triples")
        body = tuple(float(v) for v in self.body_size_xyz)
        if len(body) != 3 or not all(math.isfinite(v) and v > 0 for v in body):
            raise ValueError("body_size_xyz must contain three finite positive values")
        if not isinstance(self.stop_at_unknown, bool):
            raise ValueError("stop_at_unknown must be boolean")
        for name in ("source_stamp_ns", "now_monotonic_ns", "max_snapshot_age_ns"):
            value = getattr(self, name)
            if value is not None and (
                not isinstance(value, int) or isinstance(value, bool) or value < 0
            ):
                raise ValueError(f"{name} must be a non-negative integer or None")
        object.__setattr__(self, "samples", points)
        object.__setattr__(self, "body_size_xyz", body)


@dataclass(frozen=True)
class QueryResult:
    status: QueryStatus
    key: SnapshotKey | None
    occupancy: tuple[int, ...] = ()
    ground_z: tuple[float, ...] = ()
    roughness: tuple[float, ...] = ()
    clearance: tuple[float, ...] = ()
    step: tuple[bool, ...] = ()
    drop: tuple[bool, ...] = ()
    detail: str = ""


@dataclass(frozen=True)
class IndexStats:
    chunk_loads: int
    rebuilds: int
    point_count: int
    occupied_voxels: int
    free_voxels: int


@dataclass(frozen=True)
class _SubmapInput:
    submap_id: SubmapId
    geometry_revision: int
    pose: Matrix4
    chunks: tuple[tuple[str, int, int, str], ...]
    origins: tuple[tuple[float, float, float], ...]
    observed_at_ns: int


@dataclass(frozen=True)
class _Index:
    key: SnapshotKey
    manifest_digest: str
    source_stamp_ns: int
    received_monotonic_ns: int
    occupied: frozenset[tuple[int, int, int]]
    free: frozenset[tuple[int, int, int]]
    columns: Mapping[tuple[int, int], tuple[float, ...]]
    resolution_m: float
    point_count: int


ChunkLoader = Callable[[str], bytes]


def _manifest_digest(submaps: Sequence[_SubmapInput], tombstones: Sequence[str]) -> str:
    value = {
        "submaps": [
            {
                "submap_id": submap.submap_id.stable_id,
                "geometry_revision": submap.geometry_revision,
                "pose": submap.pose,
                "chunks": submap.chunks,
                "origins": submap.origins,
                "observed_at_ns": submap.observed_at_ns,
            }
            for submap in submaps
        ],
        "tombstones": list(tombstones),
    }
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _transform(pose: Matrix4, points: np.ndarray) -> np.ndarray:
    matrix = np.asarray(pose, dtype=np.float64)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _strict_uint(value: object, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


class IndexedMapView:
    """Thread-safe voxel index with immutable snapshot publication.

    Chunk decoding is cached by content hash. A pose correction builds and
    publishes one new immutable index, while concurrent queries keep using the
    old object. Full-ray expansion is bounded to avoid exhausting an onboard
    process on an unexpectedly dense snapshot.
    """

    def __init__(
        self,
        *,
        resolution_m: float = 0.2,
        max_points: int = 1_000_000,
        max_voxels: int = 2_000_000,
        max_ray_steps: int = 4_000_000,
        max_build_s: float = 8.0,
        max_query_s: float = 0.25,
        max_samples: int = 4096,
        max_query_voxels: int = 1_000_000,
        terrain_radius_m: float = 0.35,
        ray_angular_resolution_rad: float = math.radians(5.0),
        max_step_m: float = 0.20,
        max_drop_m: float = 0.20,
        max_roughness_m: float = 0.08,
    ):
        positive = {
            "resolution_m": resolution_m,
            "max_points": max_points,
            "max_voxels": max_voxels,
            "max_ray_steps": max_ray_steps,
            "max_build_s": max_build_s,
            "max_query_s": max_query_s,
            "max_samples": max_samples,
            "max_query_voxels": max_query_voxels,
            "terrain_radius_m": terrain_radius_m,
            "ray_angular_resolution_rad": ray_angular_resolution_rad,
        }
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in positive.values()):
            raise ValueError("index bounds and resolutions must be positive")
        self.resolution_m = float(resolution_m)
        self.max_points = int(max_points)
        self.max_voxels = int(max_voxels)
        self.max_ray_steps = int(max_ray_steps)
        self.max_build_s = float(max_build_s)
        self.max_query_s = float(max_query_s)
        self.max_samples = int(max_samples)
        self.max_query_voxels = int(max_query_voxels)
        self.terrain_radius_m = float(terrain_radius_m)
        self.ray_angular_resolution_rad = float(ray_angular_resolution_rad)
        self.max_step_m = float(max_step_m)
        self.max_drop_m = float(max_drop_m)
        self.max_roughness_m = float(max_roughness_m)
        self._lock = threading.RLock()
        self._chunk_cache: dict[str, np.ndarray] = {}
        self._index: _Index | None = None
        self._unavailable_key: SnapshotKey | None = None
        self._unavailable_detail = "no snapshot loaded"
        self._chunk_loads = 0
        self._rebuilds = 0

    @property
    def key(self) -> SnapshotKey | None:
        with self._lock:
            return self._index.key if self._index is not None else self._unavailable_key

    @property
    def stats(self) -> IndexStats:
        with self._lock:
            index = self._index
            return IndexStats(
                self._chunk_loads,
                self._rebuilds,
                index.point_count if index else 0,
                len(index.occupied) if index else 0,
                len(index.free) if index else 0,
            )

    def invalidate(self, detail: str) -> None:
        """Fail closed after the coherent source can no longer be validated."""

        with self._lock:
            self._unavailable_key = self._index.key if self._index is not None else None
            self._unavailable_detail = detail

    def refresh(
        self,
        snapshot: MapSnapshot | Mapping[str, object],
        component_id: str,
        chunk_loader: ChunkLoader,
        *,
        received_monotonic_ns: int | None = None,
    ) -> SnapshotKey:
        """Build from one coherent manifest and atomically publish it.

        Repeating the same key is a no-op. A failed newer build records that
        key as unavailable, ensuring callers cannot accidentally use the prior
        geometry while requesting the failed revision.
        """

        try:
            key, source_stamp_ns, submaps, tombstones = self._extract(
                snapshot, component_id
            )
        except Exception as exc:
            self.invalidate(str(exc))
            raise
        manifest_digest = _manifest_digest(submaps, tombstones)
        received = (
            time.monotonic_ns()
            if received_monotonic_ns is None
            else received_monotonic_ns
        )
        if not isinstance(received, int) or isinstance(received, bool) or received < 0:
            raise ValueError("received_monotonic_ns must be a non-negative integer")
        with self._lock:
            if self._index is not None and self._index.key == key:
                if self._index.manifest_digest != manifest_digest:
                    self._unavailable_key = key
                    self._unavailable_detail = (
                        "snapshot key was reused with different map content"
                    )
                    raise ValueError(self._unavailable_detail)
                # A coherent republish may refresh liveness without changing
                # geometry. Preserve the expensive index and update only its
                # immutable publication metadata.
                self._index = replace(
                    self._index,
                    source_stamp_ns=source_stamp_ns,
                    received_monotonic_ns=received,
                )
                self._unavailable_key = None
                self._unavailable_detail = ""
                return key
        try:
            built = self._build(
                key,
                manifest_digest,
                source_stamp_ns,
                submaps,
                chunk_loader,
                received,
            )
        except Exception as exc:
            with self._lock:
                self._unavailable_key = key
                self._unavailable_detail = str(exc)
            raise
        with self._lock:
            self._index = built
            self._unavailable_key = None
            self._unavailable_detail = ""
            self._rebuilds += 1
        return key

    def query(self, request: QueryRequest) -> QueryResult:
        with self._lock:
            index = self._index
            unavailable_key = self._unavailable_key
            unavailable_detail = self._unavailable_detail
        if unavailable_key is not None:
            if unavailable_key == request.key:
                return QueryResult(
                    QueryStatus.UNAVAILABLE, unavailable_key, detail=unavailable_detail
                )
            return QueryResult(
                QueryStatus.STALE,
                unavailable_key,
                detail="a different snapshot failed indexed publication",
            )
        if index is None:
            return QueryResult(
                QueryStatus.UNAVAILABLE, None, detail="no indexed snapshot"
            )
        if index.key != request.key:
            return QueryResult(
                QueryStatus.STALE, index.key, detail="requested snapshot is not current"
            )
        if (
            request.source_stamp_ns is not None
            and request.source_stamp_ns != index.source_stamp_ns
        ):
            return QueryResult(
                QueryStatus.STALE,
                index.key,
                detail="source stamp does not match indexed snapshot",
            )
        now = (
            time.monotonic_ns()
            if request.now_monotonic_ns is None
            else request.now_monotonic_ns
        )
        if request.max_snapshot_age_ns is not None:
            age = max(0, now - index.received_monotonic_ns)
            if age > request.max_snapshot_age_ns:
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="indexed snapshot exceeded coherent-read age",
                )
        if len(request.samples) > self.max_samples:
            return QueryResult(
                QueryStatus.UNAVAILABLE, index.key, detail="sample budget exceeded"
            )

        started = time.monotonic()
        occupancy: list[int] = []
        ground: list[float] = []
        roughness: list[float] = []
        clearance: list[float] = []
        step: list[bool] = []
        drop: list[bool] = []
        prior_ground: float | None = None
        work = 0
        half = np.asarray(request.body_size_xyz, dtype=np.float64) / 2.0
        for sample_index, sample_value in enumerate(request.samples):
            if time.monotonic() - started > self.max_query_s:
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query time budget exceeded",
                )
            sample = np.asarray(sample_value, dtype=np.float64)
            lo = np.floor((sample - half + 1e-9) / index.resolution_m).astype(np.int64)
            hi = np.floor((sample + half - 1e-9) / index.resolution_m).astype(np.int64)
            cells = [
                (x, y, z)
                for x in range(int(lo[0]), int(hi[0]) + 1)
                for y in range(int(lo[1]), int(hi[1]) + 1)
                for z in range(int(lo[2]), int(hi[2]) + 1)
            ]
            work += len(cells)
            if work > self.max_query_voxels:
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query voxel budget exceeded",
                )
            terrain = self._terrain(index, sample, half)
            # The lowest fitted surface is support, not a body collision. At
            # map resolution, omit its voxel layer from the body volume; low
            # obstacles that share that voxel are intentionally unresolved.
            collision_cells = cells
            if terrain is not None:
                support_top = terrain[0] + index.resolution_m
                collision_cells = [
                    cell
                    for cell in cells
                    if (cell[2] + 0.5) * index.resolution_m > support_top
                ]
            if any(cell in index.occupied for cell in collision_cells):
                state = VoxelOccupancy.OCCUPIED
            elif collision_cells and all(
                cell in index.free for cell in collision_cells
            ):
                state = VoxelOccupancy.FREE
            else:
                state = VoxelOccupancy.UNKNOWN
            occupancy.append(int(state))

            if terrain is None:
                ground.append(math.nan)
                roughness.append(math.nan)
                clearance.append(math.nan)
                step.append(False)
                drop.append(False)
                prior_ground = None
            else:
                height, surface_roughness, overhead = terrain
                if math.isnan(overhead) and state is VoxelOccupancy.FREE:
                    # A fully ray-observed body AABB proves free volume up to
                    # its top even when no ceiling/obstacle return exists.
                    overhead = max(0.0, float(sample[2] + half[2] - height))
                ground.append(height)
                roughness.append(surface_roughness)
                clearance.append(overhead)
                rise = (
                    prior_ground is not None and height - prior_ground > self.max_step_m
                )
                fall = (
                    prior_ground is not None and prior_ground - height > self.max_drop_m
                )
                step.append(rise or surface_roughness > self.max_roughness_m)
                drop.append(fall)
                prior_ground = height
            if state is VoxelOccupancy.UNKNOWN and request.stop_at_unknown:
                remaining = len(request.samples) - sample_index - 1
                occupancy.extend([int(VoxelOccupancy.UNKNOWN)] * remaining)
                ground.extend([math.nan] * remaining)
                roughness.extend([math.nan] * remaining)
                clearance.extend([math.nan] * remaining)
                step.extend([False] * remaining)
                drop.extend([False] * remaining)
                break

        return QueryResult(
            QueryStatus.OK,
            index.key,
            tuple(occupancy),
            tuple(ground),
            tuple(roughness),
            tuple(clearance),
            tuple(step),
            tuple(drop),
        )

    def _terrain(
        self, index: _Index, sample: np.ndarray, half_body: np.ndarray
    ) -> tuple[float, float, float] | None:
        radius = max(self.terrain_radius_m, float(max(half_body[0], half_body[1])))
        center = np.floor(sample[:2] / index.resolution_m).astype(np.int64)
        cell_radius = int(math.ceil(radius / index.resolution_m))
        support: list[tuple[float, float, float]] = []
        all_z: list[float] = []
        for x in range(int(center[0]) - cell_radius, int(center[0]) + cell_radius + 1):
            for y in range(
                int(center[1]) - cell_radius, int(center[1]) + cell_radius + 1
            ):
                zs = index.columns.get((x, y))
                if not zs:
                    continue
                cx = (x + 0.5) * index.resolution_m
                cy = (y + 0.5) * index.resolution_m
                if math.hypot(cx - sample[0], cy - sample[1]) > radius:
                    continue
                support.append((cx, cy, zs[0]))
                all_z.extend(zs)
        if len(support) < 3:
            return None
        values = np.asarray(support, dtype=np.float64)
        design = np.column_stack((values[:, 0], values[:, 1], np.ones(len(values))))
        coefficients, _, _, _ = np.linalg.lstsq(design, values[:, 2], rcond=None)
        predicted = float(
            coefficients[0] * sample[0] + coefficients[1] * sample[1] + coefficients[2]
        )
        residual = values[:, 2] - design @ coefficients
        roughness = float(np.sqrt(np.mean(residual * residual)))
        higher = [z for z in all_z if z > predicted + index.resolution_m]
        overhead = min(higher) - predicted if higher else math.nan
        return predicted, roughness, float(overhead)

    def _build(
        self,
        key: SnapshotKey,
        manifest_digest: str,
        source_stamp_ns: int,
        submaps: tuple[_SubmapInput, ...],
        loader: ChunkLoader,
        received: int,
    ) -> _Index:
        started = time.monotonic()
        occupied: set[tuple[int, int, int]] = set()
        free: set[tuple[int, int, int]] = set()
        columns: dict[tuple[int, int], list[float]] = {}
        point_count = 0
        ray_steps = 0
        resolution = self.resolution_m
        for submap in submaps:
            transformed_origins = (
                _transform(submap.pose, np.asarray(submap.origins, dtype=np.float64))
                if submap.origins
                else np.empty((0, 3))
            )
            origin = transformed_origins[0] if len(transformed_origins) == 1 else None
            for digest, size, declared_count, encoding in submap.chunks:
                if encoding != XYZ_F32_ENCODING or size > MAX_CHUNK_BYTES:
                    raise ValueError("unsupported or oversized map chunk")
                with self._lock:
                    local = self._chunk_cache.get(digest)
                if local is None:
                    payload = loader(digest)
                    if (
                        len(payload) != size
                        or hashlib.sha256(payload).hexdigest() != digest
                    ):
                        raise ValueError(f"chunk integrity failed: {digest}")
                    local = decode_xyz_f32(payload)
                    if len(local) != declared_count or not np.isfinite(local).all():
                        raise ValueError(
                            f"chunk point count or values invalid: {digest}"
                        )
                    local.setflags(write=False)
                    with self._lock:
                        self._chunk_cache.setdefault(digest, local)
                        local = self._chunk_cache[digest]
                        self._chunk_loads += 1
                points = _transform(submap.pose, local)
                point_count += len(points)
                if point_count > self.max_points:
                    raise ValueError("index point budget exceeded")
                voxel_values = np.floor(points / resolution).astype(np.int64)
                for point, voxel_value in zip(points, voxel_values):
                    voxel = tuple(int(v) for v in voxel_value)
                    occupied.add(voxel)
                    columns.setdefault((voxel[0], voxel[1]), []).append(float(point[2]))
                if len(occupied) > self.max_voxels:
                    raise ValueError("occupied voxel budget exceeded")
                if origin is not None:
                    # Keep one actual return per angular bin. Every retained
                    # ray remains valid evidence, while dense spinning lidars
                    # cannot multiply the work by every nearly parallel beam.
                    representatives: dict[tuple[int, int], tuple[float, np.ndarray]] = (
                        {}
                    )
                    vectors = points - origin
                    ranges = np.linalg.norm(vectors, axis=1)
                    for endpoint, vector, distance in zip(points, vectors, ranges):
                        if distance <= resolution:
                            continue
                        azimuth = math.atan2(float(vector[1]), float(vector[0]))
                        elevation = math.asin(
                            float(np.clip(vector[2] / distance, -1, 1))
                        )
                        angular_bin = (
                            math.floor(azimuth / self.ray_angular_resolution_rad),
                            math.floor(elevation / self.ray_angular_resolution_rad),
                        )
                        previous = representatives.get(angular_bin)
                        if previous is None or distance > previous[0]:
                            representatives[angular_bin] = (float(distance), endpoint)
                    for distance, endpoint in representatives.values():
                        steps = max(1, int(math.ceil(distance / (resolution * 0.75))))
                        ray_steps += max(0, steps - 1)
                        if ray_steps > self.max_ray_steps:
                            raise ValueError("free-space ray budget exceeded")
                        for ordinal in range(1, steps):
                            point = origin + (endpoint - origin) * (ordinal / steps)
                            free.add(
                                tuple(int(v) for v in np.floor(point / resolution))
                            )
                        if len(free) + len(occupied) > self.max_voxels:
                            raise ValueError("total voxel budget exceeded")
                        if time.monotonic() - started > self.max_build_s:
                            raise ValueError("index build time budget exceeded")
        free.difference_update(occupied)
        immutable_columns = {
            key_value: tuple(sorted(values)) for key_value, values in columns.items()
        }
        return _Index(
            key,
            manifest_digest,
            source_stamp_ns,
            received,
            frozenset(occupied),
            frozenset(free),
            immutable_columns,
            resolution,
            point_count,
        )

    @staticmethod
    def _extract(
        snapshot: MapSnapshot | Mapping[str, object], component_id: str
    ) -> tuple[SnapshotKey, int, tuple[_SubmapInput, ...], tuple[str, ...]]:
        if isinstance(snapshot, MapSnapshot):
            matches = [
                m
                for m in snapshot.manifests
                if m.graph_revision.component_id == component_id
            ]
            if len(matches) != 1:
                raise ValueError(
                    "snapshot must contain exactly one manifest for the component"
                )
            manifest: MapManifest = matches[0]
            key = SnapshotKey(
                component_id,
                manifest.graph_revision.epoch,
                manifest.graph_revision.revision,
                manifest.geometry_revision,
            )
            submaps = tuple(
                _SubmapInput(
                    submap.submap_id,
                    submap.geometry_revision,
                    submap.T_component_submap,
                    tuple(
                        (c.sha256, c.size_bytes, c.point_count, c.encoding)
                        for c in submap.chunks
                    ),
                    submap.sensor_origins,
                    submap.observed_at_ns,
                )
                for submap in manifest.submaps
            )
            source_stamp_ns = max(
                (submap.observed_at_ns for submap in manifest.submaps), default=0
            )
            return key, source_stamp_ns, submaps, manifest.tombstones

        if snapshot.get("schema") != SCHEMA_VERSION:
            raise ValueError("unsupported snapshot schema")
        manifests = snapshot.get("manifests")
        if not isinstance(manifests, list):
            raise ValueError("snapshot manifests must be a list")
        snapshot_id = snapshot.get("snapshot_id")
        if (
            not isinstance(snapshot_id, str)
            or len(snapshot_id) != 64
            or any(c not in "0123456789abcdef" for c in snapshot_id)
        ):
            raise ValueError("snapshot_id must be lowercase SHA-256")
        canonical_manifests = [
            (
                {"schema": SCHEMA_VERSION, **manifest}
                if isinstance(manifest, dict)
                else manifest
            )
            for manifest in manifests
        ]
        expected_snapshot_id = hashlib.sha256(
            json.dumps(
                canonical_manifests, sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        if snapshot_id != expected_snapshot_id:
            raise ValueError("snapshot_id does not match canonical manifests")
        matches = [
            value
            for value in manifests
            if isinstance(value, dict)
            and isinstance(value.get("graph_revision"), dict)
            and value["graph_revision"].get("component_id") == component_id
        ]
        if len(matches) != 1:
            raise ValueError(
                "snapshot must contain exactly one manifest for the component"
            )
        manifest_value = matches[0]
        revision = manifest_value["graph_revision"]
        assert isinstance(revision, dict)
        key = SnapshotKey(
            component_id,
            _strict_uint(revision.get("epoch"), "epoch"),
            _strict_uint(revision.get("revision"), "graph revision"),
            str(manifest_value.get("geometry_revision")),
        )
        submap_values = manifest_value.get("submaps")
        if not isinstance(submap_values, list):
            raise ValueError("manifest submaps must be a list")
        submaps: list[_SubmapInput] = []
        source_stamp_ns = 0
        geometry_members: list[tuple[str, int, list[str]]] = []
        seen_submaps: set[str] = set()
        for submap in submap_values:
            if not isinstance(submap, dict):
                raise ValueError("submap must be an object")
            submap_id_value = submap.get("submap_id")
            if not isinstance(submap_id_value, dict):
                raise ValueError("submap_id must be an object")
            submap_id = SubmapId.from_dict(submap_id_value)
            if submap_id.stable_id in seen_submaps:
                raise ValueError("manifest repeats a submap_id")
            seen_submaps.add(submap_id.stable_id)
            submap_geometry_revision = _strict_uint(
                submap.get("geometry_revision"), "submap geometry revision"
            )
            source_stamp_ns = max(
                source_stamp_ns,
                _strict_uint(submap.get("observed_at_ns", 0), "observed_at_ns"),
            )
            pose_revision = submap.get("pose_revision")
            if (
                not isinstance(pose_revision, dict)
                or pose_revision.get("component_id") != key.component_id
                or _strict_uint(pose_revision.get("epoch"), "pose epoch") != key.epoch
                or _strict_uint(pose_revision.get("revision"), "pose revision")
                != key.graph_revision
            ):
                raise ValueError("submap pose revision does not match manifest")
            pose = validate_se3(submap.get("T_component_submap"), "T_component_submap")  # type: ignore[arg-type]
            origins_value = submap.get("sensor_origins", [])
            if not isinstance(origins_value, list):
                raise ValueError("sensor_origins must be a list")
            origins = tuple(tuple(float(v) for v in origin) for origin in origins_value)
            if any(
                len(origin) != 3 or not all(math.isfinite(v) for v in origin)
                for origin in origins
            ):
                raise ValueError("sensor origins must be finite XYZ triples")
            chunks_value = submap.get("chunks")
            if not isinstance(chunks_value, list):
                raise ValueError("submap chunks must be a list")
            chunks: list[tuple[str, int, int, str]] = []
            for chunk in chunks_value:
                if not isinstance(chunk, dict):
                    raise ValueError("chunk must be an object")
                digest = chunk.get("sha256")
                if (
                    not isinstance(digest, str)
                    or len(digest) != 64
                    or any(c not in "0123456789abcdef" for c in digest)
                ):
                    raise ValueError("invalid chunk sha256")
                chunks.append(
                    (
                        digest,
                        _strict_uint(chunk.get("size_bytes"), "chunk size"),
                        _strict_uint(chunk.get("point_count"), "point count"),
                        str(chunk.get("encoding")),
                    )
                )
            geometry_members.append(
                (submap_id.stable_id, submap_geometry_revision, [c[0] for c in chunks])
            )
            submaps.append(
                _SubmapInput(
                    submap_id,
                    submap_geometry_revision,
                    pose,
                    tuple(chunks),
                    origins,
                    _strict_uint(submap.get("observed_at_ns", 0), "observed_at_ns"),
                )
            )
        expected_geometry = hashlib.sha256(
            json.dumps(
                sorted(geometry_members), sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        if key.geometry_revision != expected_geometry:
            raise ValueError("geometry_revision does not match active submaps")
        tombstones_value = manifest_value.get("tombstones", [])
        if not isinstance(tombstones_value, list) or any(
            not isinstance(value, str) for value in tombstones_value
        ):
            raise ValueError("manifest tombstones must be strings")
        return key, source_stamp_ns, tuple(submaps), tuple(tombstones_value)


class SnapshotDirectorySource:
    """Bounded atomic-file adapter used by the onboard ROS query process."""

    def __init__(
        self, peer_root: str | Path, *, max_snapshot_bytes: int = 4 * 1024 * 1024
    ):
        self.peer_root = Path(peer_root)
        self.snapshot_path = self.peer_root / "snapshot.json"
        self.chunks_path = self.peer_root / "geometry" / "chunks"
        self.max_snapshot_bytes = max_snapshot_bytes
        self._last_sha: str | None = None

    def refresh(self, view: IndexedMapView, component_id: str) -> SnapshotKey:
        size = self.snapshot_path.stat().st_size
        if size <= 0 or size > self.max_snapshot_bytes:
            raise ValueError("snapshot file has invalid size")
        raw = self.snapshot_path.read_bytes()
        if len(raw) != size:
            raise ValueError("snapshot changed while reading")
        digest = hashlib.sha256(raw).hexdigest()

        def reject_constant(value: str) -> None:
            raise ValueError(f"nonfinite JSON constant: {value}")

        value = json.loads(raw, parse_constant=reject_constant)
        if not isinstance(value, dict):
            raise ValueError("snapshot root must be an object")
        key = view.refresh(value, component_id, self.get_chunk)
        if self.snapshot_path.read_bytes() != raw:
            view.invalidate("snapshot changed during index build")
            raise ValueError("snapshot changed during index build")
        self._last_sha = digest
        return key

    def get_chunk(self, digest: str) -> bytes:
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise KeyError(digest)
        path = self.chunks_path / digest
        size = path.stat().st_size
        if size > MAX_CHUNK_BYTES:
            raise ValueError("chunk exceeds interchange limit")
        return path.read_bytes()
