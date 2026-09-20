"""Bounded, correction-aware batch queries over coherent map snapshots.

The native MOLA bridge and this query view consume the same immutable XYZ
chunks and corrected submap poses. MOLA owns metric-map serialization; this
module builds a compact voxel index for the planner's one batched final route
check. It deliberately fails closed when source freshness or work bounds cannot
be proven.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import deque

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Sequence

import numpy as np

from .contracts import (
    MapManifest,
    MapSnapshot,
    Matrix4,
    RayEvidence,
    SCHEMA_VERSION,
    SubmapId,
    validate_se3,
)
from .mapping import (
    MAX_CHUNK_BYTES,
    XYZ_F32_ENCODING,
    XYZRGBA_F32_U8_ENCODING,
    decode_xyz_f32,
    decode_xyzrgba_f32_u8,
)


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
    max_step_m: float | None = None
    max_drop_m: float | None = None

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
        for name in ("max_step_m", "max_drop_m"):
            value = getattr(self, name)
            if value is not None and (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(float(value))
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive or None")
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
    ray_evidence_qualified: bool


@dataclass(frozen=True)
class IndexedGrid:
    """Immutable planner-grid publication shared by map providers.

    Providers own construction and source-integrity checks. The query view
    owns publication ordering, freshness, work bounds, and terrain semantics.
    The constructor takes defensive copies so a provider cannot mutate a grid
    after it has been published to concurrent queries.
    """

    key: SnapshotKey
    manifest_digest: str
    source_stamp_ns: int
    received_monotonic_ns: int
    occupied: frozenset[tuple[int, int, int]]
    free: frozenset[tuple[int, int, int]]
    columns: Mapping[tuple[int, int], tuple[float, ...]]
    resolution_m: float
    point_count: int

    def refreshed(
        self, *, source_stamp_ns: int, received_monotonic_ns: int
    ) -> "IndexedGrid":
        """Reuse grid storage while updating only verified liveness metadata."""

        _strict_uint(source_stamp_ns, "source_stamp_ns")
        _strict_uint(received_monotonic_ns, "received_monotonic_ns")
        result = object.__new__(type(self))
        for name in (
            "key",
            "manifest_digest",
            "occupied",
            "free",
            "columns",
            "resolution_m",
            "point_count",
        ):
            object.__setattr__(result, name, getattr(self, name))
        object.__setattr__(result, "source_stamp_ns", source_stamp_ns)
        object.__setattr__(result, "received_monotonic_ns", received_monotonic_ns)
        return result

    def __post_init__(self) -> None:
        if not isinstance(self.key, SnapshotKey):
            raise ValueError("key must be a SnapshotKey")
        if (
            not isinstance(self.manifest_digest, str)
            or len(self.manifest_digest) != 64
            or any(c not in "0123456789abcdef" for c in self.manifest_digest)
        ):
            raise ValueError("manifest_digest must be lowercase SHA-256")
        for name in ("source_stamp_ns", "received_monotonic_ns", "point_count"):
            _strict_uint(getattr(self, name), name)
        if (
            not isinstance(self.resolution_m, (int, float))
            or isinstance(self.resolution_m, bool)
            or not math.isfinite(self.resolution_m)
            or self.resolution_m <= 0
        ):
            raise ValueError("resolution_m must be finite and positive")

        occupied = frozenset(_voxel(value, "occupied voxel") for value in self.occupied)
        free = frozenset(_voxel(value, "free voxel") for value in self.free)
        if occupied & free:
            raise ValueError("occupied and free voxels must be disjoint")
        columns: dict[tuple[int, int], tuple[float, ...]] = {}
        for key, raw_values in self.columns.items():
            column = _column(key)
            values = tuple(float(value) for value in raw_values)
            if not values or any(not math.isfinite(value) for value in values):
                raise ValueError("terrain columns must contain finite heights")
            if any(left > right for left, right in zip(values, values[1:])):
                raise ValueError("terrain column heights must be sorted")
            columns[column] = values
        object.__setattr__(self, "occupied", occupied)
        object.__setattr__(self, "free", free)
        object.__setattr__(self, "columns", MappingProxyType(columns))


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
                "ray_evidence_qualified": submap.ray_evidence_qualified,
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


def _voxel(value: object, field: str) -> tuple[int, int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{field} must contain three integers")
    result = tuple(value)
    if any(not isinstance(axis, int) or isinstance(axis, bool) for axis in result):
        raise ValueError(f"{field} must contain three integers")
    if any(abs(axis) >= 2**63 for axis in result):
        raise ValueError(f"{field} exceeds signed 64-bit range")
    return result  # type: ignore[return-value]


def _column(value: object) -> tuple[int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError("terrain column key must contain two integers")
    result = tuple(value)
    if any(not isinstance(axis, int) or isinstance(axis, bool) for axis in result):
        raise ValueError("terrain column key must contain two integers")
    if any(abs(axis) >= 2**63 for axis in result):
        raise ValueError("terrain column key exceeds signed 64-bit range")
    return result  # type: ignore[return-value]


class IndexedMapView:
    """Thread-safe voxel index with immutable snapshot publication.

    Chunk decoding is cached by content hash. A pose correction builds and
    publishes one new immutable index, while concurrent queries keep using the
    old object. Full-ray expansion is bounded to avoid exhausting an onboard
    process on an unexpectedly dense snapshot.

    A publication that replaces the current key retains the superseded index
    for a bounded grace. A planner plans for a second or two under the key its
    map authority advertised and then validates the whole route under that
    key; a route binds to the geometry it was checked against, and the
    superseded product's geometry for its own key is unchanged, so a query
    for a key that was current moments ago is answered from the retained
    index. The grace is measured from the coherent-read time of the product
    that superseded it (the same monotonic clock as the request's
    ``now_monotonic_ns``), and the coherent-read age is not applied to a
    retained index because it is no longer refreshed. Any invalidation drops
    the retained history: nothing superseded is served once the source can no
    longer be validated.
    """

    def __init__(
        self,
        *,
        resolution_m: float = 0.2,
        # The component point budget: one number with the MOLA worker's
        # ``DEFAULT_MAX_POINTS_PER_MAP`` and the reader's ``MAX_POINTS``
        # (``autonomy/mola_mapping.py``). A view budget below the product's
        # refuses every product past it: the Scout's return was refused with
        # ``index refresh failed: index point budget exceeded`` after 100 m
        # of driving (benchbot 2026-09-19) while this stayed at 1,000,000.
        max_points: int = 2_000_000,
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
        superseded_keep: int = 2,
        superseded_grace_ns: int = 15_000_000_000,
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
        # Retained as a public configuration value for callers which use the
        # query's explicit roughness metric.  Roughness is not a geometric step:
        # consumers choose their own platform-specific roughness limit.
        self.max_roughness_m = float(max_roughness_m)
        self.superseded_keep = _strict_uint(superseded_keep, "superseded_keep")
        self.superseded_grace_ns = _strict_uint(
            superseded_grace_ns, "superseded_grace_ns"
        )
        self._lock = threading.RLock()
        self._chunk_cache: dict[str, np.ndarray] = {}
        self._index: IndexedGrid | None = None
        # Superseded indexes with the monotonic time each was replaced, oldest
        # first. Keys are unique: a key republished as current drops its
        # retained copy.
        self._superseded: deque[tuple[IndexedGrid, int]] = deque(
            maxlen=self.superseded_keep
        )
        self._unavailable_key: SnapshotKey | None = None
        self._unavailable_detail = "no snapshot loaded"
        self._chunk_loads = 0
        self._rebuilds = 0

    @property
    def key(self) -> SnapshotKey | None:
        with self._lock:
            return self._index.key if self._index is not None else self._unavailable_key

    @property
    def superseded_keys(self) -> tuple[SnapshotKey, ...]:
        """Keys of the retained superseded indexes, oldest first."""

        with self._lock:
            return tuple(grid.key for grid, _ in self._superseded)

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
            # A failed newer build records its key before the registry applies
            # a source-wide invalidation. Preserve that key so callers asking
            # for the failed revision receive UNAVAILABLE rather than seeing
            # it misreported as a merely stale older publication.
            if self._unavailable_key is None:
                self._unavailable_key = (
                    self._index.key if self._index is not None else None
                )
            self._unavailable_detail = detail
            # The superseded history belongs to the same source: once that
            # source cannot be validated, nothing retained is served either.
            self._superseded.clear()

    def publish(self, grid: IndexedGrid) -> SnapshotKey:
        """Atomically install one verified immutable provider publication."""

        if not isinstance(grid, IndexedGrid):
            raise TypeError("grid must be an IndexedGrid")
        detail = ""
        if not math.isclose(
            grid.resolution_m, self.resolution_m, rel_tol=0.0, abs_tol=1e-12
        ):
            detail = "grid resolution does not match query view"
        elif grid.point_count > self.max_points:
            detail = "index point budget exceeded"
        elif len(grid.occupied) > self.max_voxels:
            detail = "occupied voxel budget exceeded"
        elif len(grid.occupied) + len(grid.free) > self.max_voxels:
            detail = "total voxel budget exceeded"
        if detail:
            with self._lock:
                self._unavailable_key = grid.key
                self._unavailable_detail = detail
            raise ValueError(detail)

        with self._lock:
            current = self._index
            if current is not None and current.key == grid.key:
                if current.manifest_digest != grid.manifest_digest:
                    self._unavailable_key = grid.key
                    self._unavailable_detail = (
                        "snapshot key was reused with different map content"
                    )
                    raise ValueError(self._unavailable_detail)
                rebuilt = False
            else:
                rebuilt = True
                # Retain only an index that was served right up to this
                # replacement. One refused meanwhile (a source invalidation
                # or a failed newer build) was not answering its key, and the
                # moment it stopped being current is not known here.
                self._retain(current if self._unavailable_key is None else None, grid)
            self._index = grid
            self._unavailable_key = None
            self._unavailable_detail = ""
            if rebuilt:
                self._rebuilds += 1
        return grid.key

    def _retain(self, current: IndexedGrid | None, grid: IndexedGrid) -> None:
        """Keep the index ``grid`` replaces, stamped with when it was superseded.

        The superseding time is the new product's coherent-read time: it is at
        or before the replacement itself, so the grace it starts can only be
        shorter than the configured bound, and it is on the same monotonic
        clock as the request's ``now_monotonic_ns``. Called under the lock.
        """

        now = grid.received_monotonic_ns
        # The new publication is authoritative for its own key, and an entry
        # whose grace has already run out will never be served again.
        kept = [
            entry
            for entry in self._superseded
            if entry[0].key != grid.key and now - entry[1] <= self.superseded_grace_ns
        ]
        self._superseded.clear()
        self._superseded.extend(kept)
        if current is not None:
            self._superseded.append((current, now))

    def _retained(self, key: SnapshotKey, now: int) -> IndexedGrid | None:
        """The retained index for ``key`` while its grace lasts. Under the lock."""

        for grid, superseded_at in self._superseded:
            if grid.key == key:
                if now - superseded_at <= self.superseded_grace_ns:
                    return grid
                return None
        return None

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
                self._index = self._index.refreshed(
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
        return self.publish(built)

    def query(self, request: QueryRequest) -> QueryResult:
        now = (
            time.monotonic_ns()
            if request.now_monotonic_ns is None
            else request.now_monotonic_ns
        )
        with self._lock:
            index = self._index
            unavailable_key = self._unavailable_key
            unavailable_detail = self._unavailable_detail
            retained = self._retained(request.key, now)
        # Order of checks. The failed key itself is refused first: a retained
        # copy of a key whose republication failed is not trusted over that
        # failure. A retained key within its grace is served next, so a newer
        # build's failure does not refuse geometry that was verified for the
        # requested key; a request for anything else then meets the existing
        # fail-closed answers.
        if unavailable_key is not None and unavailable_key == request.key:
            return QueryResult(
                QueryStatus.UNAVAILABLE, unavailable_key, detail=unavailable_detail
            )
        if retained is not None:
            index = retained
        elif unavailable_key is not None:
            return QueryResult(
                QueryStatus.STALE,
                unavailable_key,
                detail="a different snapshot failed indexed publication",
            )
        elif index is None:
            return QueryResult(
                QueryStatus.UNAVAILABLE, None, detail="no indexed snapshot"
            )
        elif index.key != request.key:
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
        # The coherent-read age proves the current index is still being
        # refreshed. A retained index is not refreshed any more; its bound is
        # the time it was superseded plus the grace, checked above.
        if retained is None and request.max_snapshot_age_ns is not None:
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
        max_step_m = (
            self.max_step_m if request.max_step_m is None else float(request.max_step_m)
        )
        max_drop_m = (
            self.max_drop_m if request.max_drop_m is None else float(request.max_drop_m)
        )
        work = 0
        half_xyz = tuple(v / 2.0 for v in request.body_size_xyz)
        half = np.asarray(half_xyz, dtype=np.float64)
        terrain_radius = max(self.terrain_radius_m, half_xyz[0], half_xyz[1])
        scaled_radius = terrain_radius / index.resolution_m
        if not math.isfinite(scaled_radius):
            return QueryResult(
                QueryStatus.UNAVAILABLE, index.key, detail="query voxel budget exceeded"
            )
        column_count = (2 * math.ceil(scaled_radius) + 1) ** 2
        if column_count > self.max_query_voxels:
            return QueryResult(
                QueryStatus.UNAVAILABLE, index.key, detail="query voxel budget exceeded"
            )
        for sample_index, sample_value in enumerate(request.samples):
            if time.monotonic() - started > self.max_query_s:
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query time budget exceeded",
                )
            sample = np.asarray(sample_value, dtype=np.float64)
            try:
                lo = tuple(
                    math.floor((v - h + 1e-9) / index.resolution_m)
                    for v, h in zip(sample_value, half_xyz)
                )
                hi = tuple(
                    math.floor((v + h - 1e-9) / index.resolution_m)
                    for v, h in zip(sample_value, half_xyz)
                )
            except (OverflowError, ValueError):
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query coordinates exceed index range",
                )
            if any(abs(v) >= 2**63 for v in (*lo, *hi)):
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query coordinates exceed index range",
                )
            cell_count = math.prod(max(0, b - a + 1) for a, b in zip(lo, hi))
            work += cell_count + column_count
            if work > self.max_query_voxels:
                return QueryResult(
                    QueryStatus.UNAVAILABLE,
                    index.key,
                    detail="query voxel budget exceeded",
                )
            cells = [
                (x, y, z)
                for x in range(int(lo[0]), int(hi[0]) + 1)
                for y in range(int(lo[1]), int(hi[1]) + 1)
                for z in range(int(lo[2]), int(hi[2]) + 1)
            ]
            terrain = self._terrain(index, sample, half)
            # The fitted surface beneath this body is support, not a collision,
            # and so is terrain the platform's step limit admits: a kerb top one
            # voxel above the gutter, half a metre from a robot standing beside
            # it, sat inside the body's axis-aligned box and was reported as an
            # obstacle at the robot's own cell (the Bunker on the Bistro gutter,
            # 2026-09-20). The body volume omits the surface's voxel layer and
            # every voxel that begins below the step limit above the surface;
            # the step, drop and roughness tests own what that band contains,
            # and a low obstacle inside it is unresolved at map resolution. A
            # voxel that begins at or above the limit still counts, so a wall
            # in front of a low body is seen in the layer above the band.
            collision_cells = cells
            if terrain is not None:
                support_top = terrain[0] + index.resolution_m
                climbable_top = terrain[0] + max_step_m
                collision_cells = [
                    cell
                    for cell in cells
                    if (cell[2] + 0.5) * index.resolution_m > support_top
                    and cell[2] * index.resolution_m >= climbable_top - 1e-9
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
                rise = prior_ground is not None and height - prior_ground > max_step_m
                fall = prior_ground is not None and prior_ground - height > max_drop_m
                step.append(rise)
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
        self, index: IndexedGrid, sample: np.ndarray, half_body: np.ndarray
    ) -> tuple[float, float, float] | None:
        radius = max(self.terrain_radius_m, float(max(half_body[0], half_body[1])))
        center = np.floor(sample[:2] / index.resolution_m).astype(np.int64)
        cell_radius = int(math.ceil(radius / index.resolution_m))
        support: list[tuple[float, float, float]] = []
        overhead_z: list[float] = []
        # A column may contain multiple floors. Select the highest observed
        # surface under the body's bottom (with half-voxel quantization margin),
        # never a wall return at or above its center. Always choosing zs[0]
        # projects an upstairs robot onto the downstairs floor.
        support_ceiling = min(
            float(sample[2]) - 1e-9,
            float(sample[2] - half_body[2]) + 0.5 * index.resolution_m,
        )
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
                ordinal = bisect_right(zs, support_ceiling)
                if ordinal:
                    height = zs[ordinal - 1]
                    support.append((cx, cy, height))
                    above = bisect_right(zs, height + index.resolution_m)
                    if above < len(zs):
                        overhead_z.append(zs[above])
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
        higher = [z for z in overhead_z if z > predicted + index.resolution_m]
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
    ) -> IndexedGrid:
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
            origin = (
                transformed_origins[0]
                if submap.ray_evidence_qualified and len(transformed_origins) == 1
                else None
            )
            for digest, size, declared_count, encoding in submap.chunks:
                if (
                    encoding not in {XYZ_F32_ENCODING, XYZRGBA_F32_U8_ENCODING}
                    or size > MAX_CHUNK_BYTES
                ):
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
                    local = (
                        decode_xyz_f32(payload)
                        if encoding == XYZ_F32_ENCODING
                        else decode_xyzrgba_f32_u8(payload)[0]
                    )
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
        return IndexedGrid(
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
                    submap.ray_evidence.certifies_free_space,
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
            evidence_value = submap.get("ray_evidence", {})
            if not isinstance(evidence_value, dict):
                raise ValueError("ray_evidence must be an object")
            evidence = RayEvidence(**evidence_value)
            if evidence.certifies_free_space and len(origins) != 1:
                raise ValueError(
                    "qualified ray evidence requires exactly one sensor origin"
                )
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
                    evidence.certifies_free_space,
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
        self.publication_path = self.snapshot_path
        self.chunks_path = self.peer_root / "geometry" / "chunks"
        self.max_snapshot_bytes = max_snapshot_bytes

    def component_ids(self) -> tuple[str, ...]:
        value, _ = self._read_snapshot()
        manifests = value.get("manifests")
        if not isinstance(manifests, list):
            raise ValueError("snapshot manifests are missing")
        result: list[str] = []
        for manifest in manifests:
            revision = (
                manifest.get("graph_revision") if isinstance(manifest, dict) else None
            )
            component = (
                revision.get("component_id") if isinstance(revision, dict) else None
            )
            if not isinstance(component, str) or not component:
                raise ValueError("manifest component_id is invalid")
            result.append(component)
        if len(set(result)) != len(result):
            raise ValueError("snapshot repeats a component_id")
        return tuple(result)

    def signature(self) -> tuple[int, ...]:
        stat = self.publication_path.stat()
        return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)

    def refresh(self, view: IndexedMapView, component_id: str) -> SnapshotKey:
        value, raw = self._read_snapshot()
        key = view.refresh(value, component_id, self.get_chunk)
        if self.snapshot_path.read_bytes() != raw:
            view.invalidate("snapshot changed during index build")
            raise ValueError("snapshot changed during index build")
        return key

    def _read_snapshot(self) -> tuple[dict[str, object], bytes]:
        size = self.snapshot_path.stat().st_size
        if size <= 0 or size > self.max_snapshot_bytes:
            raise ValueError("snapshot file has invalid size")
        raw = self.snapshot_path.read_bytes()
        if len(raw) != size:
            raise ValueError("snapshot changed while reading")

        def reject_constant(value: str) -> None:
            raise ValueError(f"nonfinite JSON constant: {value}")

        value = json.loads(raw, parse_constant=reject_constant)
        if not isinstance(value, dict):
            raise ValueError("snapshot root must be an object")
        return value, raw

    def get_chunk(self, digest: str) -> bytes:
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise KeyError(digest)
        path = self.chunks_path / digest
        size = path.stat().st_size
        if size > MAX_CHUNK_BYTES:
            raise ValueError("chunk exceeds interchange limit")
        return path.read_bytes()
