"""Keep the deployment-frame 2D raster beside replicated 3D products.

The Global 3D view renders replicated keyframes directly. This module rasterizes
the same geometry for the 2D dashboard and stores it in the optimized-map
store: a verified component as ``component:<id>``, or the deployment composite
as ``deployment:<session>``. Both interfaces therefore show the same product.

Lifecycle: ``deployment_raster_loop`` calls ``tick`` periodically. Registry and
map-service placements are read on the event loop; catalogue assembly, chunk
decoding and the numpy raster run in a worker thread. A rebuild happens only
when the composite snapshot or placement changes, or when the scope is absent.
An aggregate sparse contribution is cached per scope. Append-only publications
process only new submaps; a correction or removal streams that scope once and
replaces its bounded aggregate, so historical per-submap point products are
never retained.
Scopes remain valid while the mission is unchanged and are retired on mission
or reset.

Budgets: the composite's source/submap catalogue budgets still apply. Raster
materialisation is bounded by ``MAX_CELLS`` output cells, decoded chunks by the
LRU byte budget, and each cell's weighted height histogram by a fixed bin cap;
the raster never retains the lifetime raw point cloud.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time
from collections import OrderedDict
from dataclasses import dataclass
from threading import Lock
from typing import Any, Mapping

import numpy as np

from autonomy.mapping import decode_chunk_points
from ..mapsvc.grid_meta import GridMeta
from ..mapsvc.keyframe_raster import (
    DEFAULT_CELL_M,
    DEFAULT_MARGIN_M,
    MAX_CELLS,
    OBSTACLE_MAX_M,
    OBSTACLE_MIN_M,
    GROUND_PERCENTILE,
    KeyframeRaster,
    composite_world_points,
    swept_free_cells,
)
from . import map_routes
from . import replica_views

log = logging.getLogger(__name__)

REFRESH_INTERVAL_S = 3.0
MAX_POINTS = 8_000_000
CACHE_BYTES = 256 * 1024 * 1024
POSE_REBUILD_TRANSLATION_M = 0.05
POSE_REBUILD_YAW_RAD = 0.01


class _ChunkCache:
    """Decoded chunk points by hash, evicting least recently used past a budget."""

    def __init__(self, max_bytes: int):
        self.max_bytes = max_bytes
        self.used = 0
        self._entries: OrderedDict[str, np.ndarray] = OrderedDict()
        self._lock = Lock()

    def get(self, digest: str) -> np.ndarray | None:
        with self._lock:
            points = self._entries.get(digest)
            if points is not None:
                self._entries.move_to_end(digest)
            return points

    def put(self, digest: str, points: np.ndarray) -> None:
        with self._lock:
            previous = self._entries.pop(digest, None)
            if previous is not None:
                self.used -= previous.nbytes
            self._entries[digest] = points
            self.used += points.nbytes
            while self.used > self.max_bytes and len(self._entries) > 1:
                _, evicted = self._entries.popitem(last=False)
                self.used -= evicted.nbytes

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()
            self.used = 0


def se2_of(matrix: Any) -> dict[str, float]:
    """The ``(x, y, yaw)`` the 2D map expects from a planar SE(3) placement."""
    rows = np.asarray(matrix, dtype=np.float64)
    return {
        "x": float(rows[0, 3]),
        "y": float(rows[1, 3]),
        "yaw": float(math.atan2(rows[1, 0], rows[0, 0])),
    }


def active_session_id() -> str | None:
    return os.environ.get("SWARMDECK_MISSION_ID") or None


def component_frames(session_id: str | None) -> dict[str, tuple[str, Any]]:
    """Each robot's live component and its navigation frame in that component."""
    if not session_id:
        return {}
    frames: dict[str, tuple[str, Any]] = {}
    for robot in list(replica_views.registry.robots.values()):
        live = robot.live_mapping
        if live is None or live["mission_id"] != session_id:
            continue
        try:
            transform = replica_views.validate_se3(
                live["T_component_navigation"], "T_component_navigation"
            )
        except (KeyError, TypeError, ValueError):
            continue
        frames[robot.robot_id] = (str(live["component_id"]), transform)
    return frames


def verified_component(catalogue, session_id: str) -> str | None:
    """The largest component two or more robots publish, or None.

    A component with several publishers is a verified inter-robot merge: the
    Global 3D view prefers it, and so does the 2D raster.
    """
    best: tuple[int, int, str] | None = None
    for (session, component_id), sources in catalogue.groups.items():
        if session != session_id or replica_views.is_deployment_component(component_id):
            continue
        robots = {source["robot_id"] for source in sources}
        if len(robots) < 2:
            continue
        rank = (len(robots), len(sources), component_id)
        if best is None or rank > best:
            best = rank
    return None if best is None else best[2]


HIST_BIN_M = 0.001
MAX_HIST_BINS = 512


@dataclass
class _HeightHistogram:
    bins: dict[int, int]
    shift: int = 0

    def merge(self, other: "_HeightHistogram") -> None:
        shift = max(self.shift, other.shift)
        if shift != self.shift:
            reduced: dict[int, int] = {}
            for key, count in self.bins.items():
                bucket = key >> (shift - self.shift)
                reduced[bucket] = reduced.get(bucket, 0) + count
            self.bins = reduced
            self.shift = shift
        for key, count in other.bins.items():
            bucket = key >> (shift - other.shift)
            self.bins[bucket] = self.bins.get(bucket, 0) + count
        while len(self.bins) > MAX_HIST_BINS:
            reduced = {}
            for key, count in self.bins.items():
                bucket = key >> 1
                reduced[bucket] = reduced.get(bucket, 0) + count
            self.bins = reduced
            self.shift += 1


@dataclass
class _SubmapContribution:
    key: str
    submap_keys: dict[str, str]
    histograms: dict[int, _HeightHistogram]
    rays: set[int]
    low_x: float
    low_y: float
    high_x: float
    high_y: float
    points: int
    histogram_bins: int = 0


class DeploymentRasterRefresher:
    """Incremental aggregate cells; corrected/removed submaps rebuild the scope."""

    def __init__(
        self,
        *,
        max_points: int = MAX_POINTS,
        cache_bytes: int = CACHE_BYTES,
    ):
        self.max_points = max_points
        self.chunks = _ChunkCache(cache_bytes)
        self.built: tuple[str, str] | None = None
        self.failed: tuple[str, str] | None = None
        # Per-robot rasters, keyed by scope: the snapshot they were built from.
        self.robot_built: dict[str, str] = {}
        # Aggregate cell histograms bound retained history independently of raw
        # point count; no per-submap point products remain resident.
        self.contributions: dict[str, _SubmapContribution] = {}
        self._placement_cache: dict[str, dict[str, Mapping[str, Any]]] = {}
        self._submap_key_cache: OrderedDict[tuple[Any, ...], dict[str, str]] = (
            OrderedDict()
        )

    def reset(self) -> None:
        self.built = None
        self.failed = None
        self.robot_built.clear()
        self.contributions.clear()
        self._placement_cache.clear()
        self._submap_key_cache.clear()
        self.chunks.clear()

    @staticmethod
    def _declared_points(view: Mapping[str, Any]) -> int:
        return sum(
            int(chunk["point_count"])
            for submap in view["selected"]["submaps"]
            for chunk in submap["chunks"]
        )

    @staticmethod
    def _submap_key(submap: Mapping[str, Any]) -> str:
        return json.dumps(
            {
                key: value
                for key, value in submap.items()
                if key not in {"pose_revision", "graph_revision"}
            },
            sort_keys=True,
            separators=(",", ":"),
        )

    def _submap_keys(self, view: Mapping[str, Any]) -> dict[str, str]:
        submap_ids = tuple(
            str(submap["submap_id"]) for submap in view["selected"]["submaps"]
        )
        cache_key = (view.get("component_id"), view.get("snapshot_id"), submap_ids)
        cached = self._submap_key_cache.get(cache_key)
        if cached is not None:
            self._submap_key_cache.move_to_end(cache_key)
            return cached
        keys = {
            str(submap["submap_id"]): self._submap_key(submap)
            for submap in view["selected"]["submaps"]
        }
        self._submap_key_cache[cache_key] = keys
        self._submap_key_cache.move_to_end(cache_key)
        while len(self._submap_key_cache) > 16:
            self._submap_key_cache.popitem(last=False)
        return keys

    @staticmethod
    def _compress_histogram(hist: dict[int, int]) -> _HeightHistogram:
        result = _HeightHistogram({})
        result.merge(_HeightHistogram(hist))
        return result

    def _contribution(
        self, submap: Mapping[str, Any], key: str
    ) -> tuple[_SubmapContribution | None, dict[str, Any]]:
        one = {"selected": {"submaps": [submap]}}
        points, gathered = composite_world_points(
            one, self._chunk_points, max_points=self.max_points
        )
        if not len(points):
            return None, gathered
        cols = np.floor(points[:, 0] / DEFAULT_CELL_M).astype(np.int64)
        rows = np.floor(points[:, 1] / DEFAULT_CELL_M).astype(np.int64)
        cells = (rows << np.int64(32)) ^ (cols & np.int64(0xFFFFFFFF))
        zbins = np.rint(points[:, 2] / HIST_BIN_M).astype(np.int64)
        pairs = np.column_stack((cells, zbins))
        unique, counts = np.unique(pairs, axis=0, return_counts=True)
        by_cell: dict[int, dict[int, int]] = {}
        for (cell, zbin), count in zip(unique, counts, strict=True):
            by_cell.setdefault(int(cell), {})[int(zbin)] = int(count)
        histograms = {
            cell: self._compress_histogram(bins) for cell, bins in by_cell.items()
        }
        low_x = float(points[:, 0].min() - DEFAULT_MARGIN_M)
        low_y = float(points[:, 1].min() - DEFAULT_MARGIN_M)
        high_x = float(points[:, 0].max() + DEFAULT_MARGIN_M)
        high_y = float(points[:, 1].max() + DEFAULT_MARGIN_M)
        origin_x = math.floor(low_x / DEFAULT_CELL_M) * DEFAULT_CELL_M
        origin_y = math.floor(low_y / DEFAULT_CELL_M) * DEFAULT_CELL_M
        width = int(math.ceil((high_x - origin_x) / DEFAULT_CELL_M)) + 1
        height = int(math.ceil((high_y - origin_y) / DEFAULT_CELL_M)) + 1
        if width * height > MAX_CELLS:
            raise ValueError("submap extent exceeds raster cell budget")
        swept = swept_free_cells(
            points,
            gathered["origins"],
            gathered["spans"],
            origin_x=origin_x,
            origin_y=origin_y,
            cell=DEFAULT_CELL_M,
            width=width,
            height=height,
        )
        ray_rows, ray_cols = np.nonzero(swept.reshape(height, width))
        row0 = int(math.floor(origin_y / DEFAULT_CELL_M))
        col0 = int(math.floor(origin_x / DEFAULT_CELL_M))
        ray_cells = ((ray_rows.astype(np.int64) + row0) << np.int64(32)) ^ (
            (ray_cols.astype(np.int64) + col0) & np.int64(0xFFFFFFFF)
        )
        rays = {int(cell) for cell in ray_cells.tolist()}
        return (
            _SubmapContribution(
                key=key,
                submap_keys={},
                histograms=histograms,
                rays=rays,
                low_x=origin_x,
                low_y=origin_y,
                high_x=origin_x + (width - 1) * DEFAULT_CELL_M,
                high_y=origin_y + (height - 1) * DEFAULT_CELL_M,
                points=int(len(points)),
            ),
            gathered,
        )

    @staticmethod
    def _add_contribution(
        target: _SubmapContribution, item: _SubmapContribution
    ) -> None:
        target.rays.update(item.rays)
        target.low_x = min(target.low_x, item.low_x)
        target.low_y = min(target.low_y, item.low_y)
        target.high_x = max(target.high_x, item.high_x)
        target.high_y = max(target.high_y, item.high_y)
        target.points += item.points
        for cell, bins in item.histograms.items():
            histogram = target.histograms.setdefault(cell, _HeightHistogram({}))
            before = len(histogram.bins)
            histogram.merge(bins)
            target.histogram_bins += len(histogram.bins) - before

    def _merge_contributions(
        self, contributions: Mapping[str, _SubmapContribution]
    ) -> KeyframeRaster:
        if not contributions:
            raise ValueError("no finite points to rasterize")
        low_x = min(item.low_x for item in contributions.values())
        low_y = min(item.low_y for item in contributions.values())
        high_x = max(item.high_x for item in contributions.values())
        high_y = max(item.high_y for item in contributions.values())
        resolution = DEFAULT_CELL_M
        coarsened = 0
        while True:
            origin_x = math.floor(low_x / resolution) * resolution
            origin_y = math.floor(low_y / resolution) * resolution
            width = int(math.ceil((high_x - origin_x) / resolution)) + 1
            height = int(math.ceil((high_y - origin_y) / resolution)) + 1
            if width * height <= MAX_CELLS:
                break
            resolution *= 2.0
            coarsened += 1
            if coarsened > 4:
                raise ValueError("incremental raster extent exceeds cell budget")
        merged: dict[int, _HeightHistogram] = {}
        rays: set[int] = set()
        for item in contributions.values():
            rays.update(item.rays)
            for cell, bins in item.histograms.items():
                row = int(cell) >> 32
                col = int(cell) & 0xFFFFFFFF
                if col >= 1 << 31:
                    col -= 1 << 32
                out_col = int(
                    math.floor(((col + 0.5) * DEFAULT_CELL_M - origin_x) / resolution)
                )
                out_row = int(
                    math.floor(((row + 0.5) * DEFAULT_CELL_M - origin_y) / resolution)
                )
                if 0 <= out_col < width and 0 <= out_row < height:
                    out_cell = (out_row << 32) ^ (out_col & 0xFFFFFFFF)
                    merged.setdefault(out_cell, _HeightHistogram({})).merge(bins)
        cells = np.full((height, width), -1, dtype=np.int8)
        flat = cells.reshape(-1)
        occupied = np.zeros(width * height, dtype=bool)
        for cell, bins in merged.items():
            row = int(cell) >> 32
            col = int(cell) & 0xFFFFFFFF
            if col >= 1 << 31:
                col -= 1 << 32
            index = row * width + col
            ordered = sorted(
                (zbin * (1 << bins.shift), count) for zbin, count in bins.bins.items()
            )
            total = sum(count for _zbin, count in ordered)
            target = GROUND_PERCENTILE / 100.0 * (total - 1)
            lower, upper = math.floor(target), math.ceil(target)
            cumulative = 0
            low_value = high_value = ordered[-1][0]
            for zbin, count in ordered:
                if cumulative <= lower < cumulative + count:
                    low_value = zbin
                if cumulative <= upper < cumulative + count:
                    high_value = zbin
                    break
                cumulative += count
            ground = low_value + (high_value - low_value) * (target - lower)
            has_obstacle = any(
                ground + OBSTACLE_MIN_M / HIST_BIN_M
                <= zbin
                <= ground + OBSTACLE_MAX_M / HIST_BIN_M
                for zbin, _count in ordered
            )
            flat[index] = 100 if has_obstacle else 0
            occupied[index] = has_obstacle
        for cell in rays:
            row = int(cell) >> 32
            col = int(cell) & 0xFFFFFFFF
            if col >= 1 << 31:
                col -= 1 << 32
            out_col = int(
                math.floor(((col + 0.5) * DEFAULT_CELL_M - origin_x) / resolution)
            )
            out_row = int(
                math.floor(((row + 0.5) * DEFAULT_CELL_M - origin_y) / resolution)
            )
            if 0 <= out_col < width and 0 <= out_row < height:
                index = out_row * width + out_col
                if not occupied[index]:
                    flat[index] = 0
        return KeyframeRaster(
            meta=GridMeta(resolution, width, height, origin_x, origin_y),
            cells=cells,
            points=sum(item.points for item in contributions.values()),
            known_cells=int((flat != -1).sum()),
            occupied_cells=int(occupied.sum()),
            coarsened=coarsened,
        )

    def _incremental_raster(
        self, scope: str, view: Mapping[str, Any]
    ) -> tuple[KeyframeRaster, dict[str, Any]]:
        previous = self.contributions.pop(scope, None)
        current_keys = self._submap_keys(view)
        append_only = (
            previous is not None
            and all(
                previous.submap_keys.get(submap_id) == key
                for submap_id, key in current_keys.items()
                if submap_id in previous.submap_keys
            )
            and set(previous.submap_keys).issubset(current_keys)
        )
        if append_only:
            aggregate = previous
            pending = [
                submap
                for submap in view["selected"]["submaps"]
                if str(submap["submap_id"]) not in previous.submap_keys
            ]
        else:
            aggregate = None
            pending = list(view["selected"]["submaps"])
        chunks = missing = rebuilt = 0
        for submap in pending:
            submap_id = str(submap["submap_id"])
            key = current_keys[submap_id]
            contribution, gathered = self._contribution(submap, key)
            chunks += gathered["chunks"]
            missing += gathered["missing_chunks"]
            if contribution is None:
                continue
            if aggregate is None:
                aggregate = _SubmapContribution(
                    key="",
                    submap_keys={},
                    histograms={},
                    rays=set(),
                    low_x=contribution.low_x,
                    low_y=contribution.low_y,
                    high_x=contribution.high_x,
                    high_y=contribution.high_y,
                    points=0,
                )
            self._add_contribution(aggregate, contribution)
            if (
                len(aggregate.histograms) > MAX_CELLS
                or len(aggregate.rays) > MAX_CELLS
                or aggregate.histogram_bins > self.max_points
            ):
                raise ValueError("incremental raster contribution exceeds cell budget")
            aggregate.submap_keys[submap_id] = key
            rebuilt += 1
        if aggregate is None:
            raise ValueError("no finite points to rasterize")
        aggregate.submap_keys = current_keys
        raster = self._merge_contributions({"scope": aggregate})
        self.contributions[scope] = aggregate
        return raster, {
            "chunks": chunks,
            "missing_chunks": missing,
            "declared": self._declared_points(view),
            "rebuilt_submaps": rebuilt,
        }

    @staticmethod
    def _pose_delta_exceeds(a: Mapping[str, Any], b: Mapping[str, Any]) -> bool:
        a_nav = se2_of(a["T_world_navigation"])
        b_nav = se2_of(b["T_world_navigation"])
        a_component = se2_of(a["T_world_component"])
        b_component = se2_of(b["T_world_component"])
        for first, second in ((a_nav, b_nav), (a_component, b_component)):
            if (
                math.hypot(first["x"] - second["x"], first["y"] - second["y"])
                >= POSE_REBUILD_TRANSLATION_M
            ):
                return True
            yaw_delta = math.atan2(
                math.sin(first["yaw"] - second["yaw"]),
                math.cos(first["yaw"] - second["yaw"]),
            )
            if abs(yaw_delta) >= POSE_REBUILD_YAW_RAD:
                return True
        return False

    def _placements_for_raster(
        self, session_id: str, placements: Mapping[str, Mapping[str, Any]]
    ) -> Mapping[str, Mapping[str, Any]]:
        previous = self._placement_cache.get(session_id)
        if previous is None or set(previous) != set(placements):
            current = dict(placements)
            self._placement_cache = {session_id: current}
            return current
        for robot_id, placement in placements.items():
            old = previous[robot_id]
            if (
                old.get("component_id") != placement.get("component_id")
                or old.get("solution_order") != placement.get("solution_order")
                or self._pose_delta_exceeds(old, placement)
            ):
                current = dict(placements)
                self._placement_cache = {session_id: current}
                return current
        return previous

    # ------------------------------------------------------------ event loop

    async def tick(self) -> dict[str, Any]:
        """One refresh: placements on the loop, everything else in a thread."""
        session_id = active_session_id()
        placements = replica_views.deployment_placements(session_id)
        frames = component_frames(session_id)
        return await asyncio.to_thread(self.refresh, session_id, placements, frames)

    # ---------------------------------------------------------------- thread

    def _chunk_points(self, chunk: Mapping[str, Any]) -> np.ndarray | None:
        digest = str(chunk["sha256"])
        cached = self.chunks.get(digest)
        if cached is not None:
            return cached
        try:
            payload = replica_views.store().read_chunk(digest)
            points = decode_chunk_points(payload, str(chunk["encoding"]))
        except (OSError, ValueError):
            return None
        points = np.ascontiguousarray(points, dtype=np.float32)
        self.chunks.put(digest, points)
        return points

    def refresh(
        self,
        session_id: str | None,
        placements: Mapping[str, Mapping[str, Any]],
        frames: Mapping[str, tuple[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Rebuild the active mission's raster when its source changed.

        ``frames`` maps each robot to ``(component_id, T_component_navigation)``
        from its live authority, for a verified component's transforms.
        Returns a small report (``status`` plus counts) for logs and tests.
        """
        generation = map_routes.raster_generation()
        frames = frames or {}
        if not session_id:
            retired = map_routes.retire_server_scopes(keep=None)
            if retired:
                log.info("Retired fleet raster(s) %s", ", ".join(retired))
                self.chunks.clear()
                self.contributions.clear()
            self.built = None
            self._placement_cache.clear()
            return {"status": "no mission", "retired": retired}

        # A verified merge (a component two or more robots publish) is the
        # fleet map; the composite is what there is without one.
        try:
            catalogue = replica_views.current_catalogue(session_id)
        except (OverflowError, ValueError, KeyError, TypeError) as exc:
            scope = replica_views.deployment_component_id(session_id)
            return self._skip(scope, (session_id, "catalogue"), str(exc), [])
        merged = verified_component(catalogue, session_id)
        if merged is not None:
            # The catalogue's component ids already carry the ``component:``
            # prefix the back-end's scopes use.
            scope = merged if merged.startswith("component:") else f"component:{merged}"
        else:
            scope = replica_views.deployment_component_id(session_id)
        # Each robot's own component is also rasterized (``robot:<id>``) for
        # the 2D local view: the map that robot navigates, in its own frame.
        robot_scopes = {
            robot: f"robot:{robot}"
            for robot, (_component, _transform) in frames.items()
        }
        retired = map_routes.retire_server_scopes(keep={scope, *robot_scopes.values()})
        if retired:
            log.info("Retired fleet raster(s) %s", ", ".join(retired))
            self.chunks.clear()
            self.robot_built = {
                key: value
                for key, value in self.robot_built.items()
                if key not in retired
            }
            self.contributions = {
                key: value
                for key, value in self.contributions.items()
                if key not in retired
            }
        raster_placements = self._placements_for_raster(session_id, placements)
        robot_reports = self.refresh_robots(
            session_id, catalogue, frames, robot_scopes, generation
        )

        try:
            if merged is not None:
                view = catalogue.view(session_id, merged)
                robots = tuple(
                    sorted(
                        robot
                        for robot, (component, _) in frames.items()
                        if component == merged
                    )
                )
                transforms = {robot: se2_of(frames[robot][1]) for robot in robots}
            else:
                if len(raster_placements) < 2:
                    return {
                        "status": "no composite",
                        "scope": scope,
                        "retired": retired,
                        "robot_rasters": robot_reports,
                    }
                view = replica_views.deployment_view(
                    catalogue, session_id, raster_placements
                )
                if view is None:
                    return {
                        "status": "no composite",
                        "scope": scope,
                        "retired": retired,
                        "robot_rasters": robot_reports,
                    }
                # ``members`` carries each placement's T_world_navigation: the
                # same transform ``robot_state`` applies to that robot's
                # telemetry, so the 2D overlay's re-projection is the identity.
                robots = tuple(member["robot_id"] for member in view["members"])
                transforms = {
                    member["robot_id"]: se2_of(member["T_world_navigation"])
                    for member in view["members"]
                }
        except (OverflowError, ValueError, LookupError, TypeError) as exc:
            # A merge in progress raises LookupError until every publisher
            # carries the same accepted solution; the last raster stays.
            return {
                **self._skip(scope, (session_id, "catalogue"), str(exc), retired),
                "robot_rasters": robot_reports,
            }

        key = (session_id, str(view["snapshot_id"]))
        if key == self.built and map_routes.has_optimized_map(scope):
            return {
                "status": "unchanged",
                "scope": scope,
                "retired": retired,
                "robot_rasters": robot_reports,
            }
        if key == self.failed:
            return {
                "status": "skipped",
                "scope": scope,
                "retired": retired,
                "robot_rasters": robot_reports,
            }

        started = time.perf_counter()
        try:
            raster, gathered = self._incremental_raster(scope, view)
        except (OverflowError, ValueError) as exc:
            return {
                **self._skip(scope, key, str(exc), retired),
                "robot_rasters": robot_reports,
            }
        published = map_routes.publish_optimized_map(
            scope,
            raster.meta,
            raster.cells,
            robots,
            transforms,
            expected_generation=generation,
        )
        if not published:
            return {"status": "superseded", "scope": scope, "retired": retired}
        self.built = key
        self.failed = None
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        log.info(
            "Fleet raster %s: %d robots, %d chunks (%d missing), %d points, "
            "%dx%d cells at %.2f m (%d known, %d occupied, coarsened %d), %.0f ms",
            scope,
            len(robots),
            gathered["chunks"],
            gathered["missing_chunks"],
            raster.points,
            raster.meta.width,
            raster.meta.height,
            raster.meta.resolution,
            raster.known_cells,
            raster.occupied_cells,
            raster.coarsened,
            elapsed_ms,
        )
        return {
            "status": "built",
            "scope": scope,
            "robots": list(robots),
            "points": raster.points,
            "cells": raster.meta.width * raster.meta.height,
            "known_cells": raster.known_cells,
            "occupied_cells": raster.occupied_cells,
            "missing_chunks": gathered["missing_chunks"],
            "elapsed_ms": elapsed_ms,
            "retired": retired,
            "robot_rasters": robot_reports,
        }

    def refresh_robots(
        self,
        session_id: str,
        catalogue,
        frames: Mapping[str, tuple[str, Any]],
        robot_scopes: Mapping[str, str],
        generation: int,
    ) -> dict[str, str]:
        """Rasterize each robot's own component into ``robot:<id>``.

        The raster frame is the robot's component frame, so its transform
        header is ``T_component_navigation``: the same matrix the robot's
        live authority carries, which the 2D overlay applies to its pose.
        """
        reports: dict[str, str] = {}
        for robot, (component, transform) in frames.items():
            scope = robot_scopes[robot]
            try:
                view = catalogue.view(session_id, component)
            except (LookupError, TypeError, ValueError) as exc:
                reports[robot] = f"skipped: {exc}"
                continue
            snapshot = str(view.get("snapshot_id") or "")
            if not snapshot:
                reports[robot] = "skipped: no publication identity"
                continue
            owned = [
                submap
                for submap in view["selected"]["submaps"]
                if submap["submap_id"].startswith(f"{robot}/")
            ]
            view = {**view, "selected": {**view["selected"], "submaps": owned}}
            snapshot = replica_views.digest(
                [
                    component,
                    transform,
                    [
                        self._submap_keys(view)[str(submap["submap_id"])]
                        for submap in owned
                    ],
                ]
            )
            if self.robot_built.get(scope) == snapshot and map_routes.has_optimized_map(
                scope
            ):
                reports[robot] = "unchanged"
                continue
            try:
                raster, _gathered = self._incremental_raster(scope, view)
            except (OverflowError, ValueError) as exc:
                reports[robot] = f"skipped: {exc}"
                continue
            if map_routes.publish_optimized_map(
                scope,
                raster.meta,
                raster.cells,
                (robot,),
                {robot: se2_of(transform)},
                expected_generation=generation,
            ):
                self.robot_built[scope] = snapshot
                reports[robot] = "built"
            else:
                reports[robot] = "superseded"
        return reports

    def _skip(
        self, scope: str, key: tuple[str, str], detail: str, retired: list[str]
    ) -> dict[str, Any]:
        # One warning per failing publication, not one per tick.
        if key != self.failed:
            log.warning("Fleet raster %s skipped: %s", scope, detail)
            self.failed = key
        return {
            "status": "skipped",
            "scope": scope,
            "detail": detail,
            "retired": retired,
        }


refresher = DeploymentRasterRefresher()


async def deployment_raster_loop(interval_s: float = REFRESH_INTERVAL_S) -> None:
    """Refresh the deployment raster every few seconds, never blocking the loop."""
    while True:
        await asyncio.sleep(interval_s)
        try:
            await refresher.tick()
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("Deployment raster refresh failed")
