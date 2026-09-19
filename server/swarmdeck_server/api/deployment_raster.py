"""Keep a 2D raster of the deployment composite in the optimized-map store.

The Global 3D view renders the deployment composite (``replica_views``): every
replicated single-robot component of the active mission placed in the surveyed
deployment frame. With inter-robot closures off it is the only fleet-wide map
there is, and the 2D map's "optimized" source (``/api/map/optimized``) had
nothing to show because no ``component:<n>`` scope holds two robots. This
module rasterizes that same composite (``mapsvc.keyframe_raster``) and stores
it as the scope ``deployment:<session>`` beside the back-end's grids, so both
interfaces show the same merged geometry.

Lifecycle: ``deployment_raster_loop`` in ``api.app`` calls ``tick`` every
``REFRESH_INTERVAL_S``. Registry and map-service placements are read on the
event loop; catalogue assembly, chunk decoding and the numpy raster run in a
worker thread. A rebuild happens only when the composite's ``snapshot_id``
changed (it digests every member's replica snapshot and placement) or when the
scope is missing from the store (a map reset cleared it). The scope is kept
while the mission is unchanged, even through a moment without a composite
(a member's authority and replica can briefly disagree on a frame revision),
and retired when the active mission changes or is unset; the SLAM back-end's
scope list never touches it (``map_routes._prune_optimized_maps``).

Budgets: the composite's own submap and source budgets apply (an overflow
skips the build), the declared point total is capped by ``MAX_POINTS`` before
any chunk is read, the raster's cell count is capped by ``keyframe_raster``,
and decoded chunks are cached by hash under ``CACHE_BYTES`` because chunks
are immutable while pose corrections move whole submaps.
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import time
from collections import OrderedDict
from threading import Lock
from typing import Any, Mapping

import numpy as np

from autonomy.mapping import decode_chunk_points
from ..mapsvc.keyframe_raster import (
    KeyframeRaster,
    composite_world_points,
    rasterize_points,
)
from . import map_routes
from . import replica_views

log = logging.getLogger(__name__)

REFRESH_INTERVAL_S = 3.0
MAX_POINTS = 8_000_000
CACHE_BYTES = 256 * 1024 * 1024


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


class DeploymentRasterRefresher:
    def __init__(
        self,
        *,
        max_points: int = MAX_POINTS,
        cache_bytes: int = CACHE_BYTES,
        rasterize=rasterize_points,
    ):
        self.max_points = max_points
        self.rasterize = rasterize
        self.built: tuple[str, str] | None = None
        self.failed: tuple[str, str] | None = None
        self.chunks = _ChunkCache(cache_bytes)

    def reset(self) -> None:
        self.built = None
        self.failed = None
        self.chunks.clear()

    # ------------------------------------------------------------ event loop

    async def tick(self) -> dict[str, Any]:
        """One refresh: placements on the loop, everything else in a thread."""
        session_id = active_session_id()
        placements = replica_views.deployment_placements(session_id)
        return await asyncio.to_thread(self.refresh, session_id, placements)

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
        self, session_id: str | None, placements: Mapping[str, Mapping[str, Any]]
    ) -> dict[str, Any]:
        """Rebuild the active mission's raster when its composite changed.

        Returns a small report (``status`` plus counts) for logs and tests.
        """
        scope = (
            replica_views.deployment_component_id(session_id) if session_id else None
        )
        retired = map_routes.retire_deployment_scopes(keep=scope)
        if retired:
            log.info("Retired deployment raster(s) %s", ", ".join(retired))
            self.chunks.clear()
        if scope is None:
            self.built = None
            return {"status": "no mission", "retired": retired}
        if len(placements) < 2:
            return {"status": "no composite", "scope": scope, "retired": retired}

        try:
            catalogue = replica_views.current_catalogue(session_id)
            view = replica_views.deployment_view(catalogue, session_id, placements)
        except (OverflowError, ValueError, KeyError, TypeError) as exc:
            return self._skip(scope, (session_id, "catalogue"), str(exc), retired)
        if view is None:
            return {"status": "no composite", "scope": scope, "retired": retired}

        key = (session_id, str(view["snapshot_id"]))
        if key == self.built and map_routes.has_optimized_map(scope):
            return {"status": "unchanged", "scope": scope, "retired": retired}
        if key == self.failed:
            return {"status": "skipped", "scope": scope, "retired": retired}

        started = time.perf_counter()
        try:
            points, gathered = composite_world_points(
                view, self._chunk_points, max_points=self.max_points
            )
            raster: KeyframeRaster = self.rasterize(points)
        except (OverflowError, ValueError) as exc:
            return self._skip(scope, key, str(exc), retired)

        # ``members`` carries each placement's T_world_navigation: the same
        # transform ``robot_state`` applies to that robot's telemetry, so the
        # 2D overlay's re-projection onto this raster is the identity.
        robots = tuple(member["robot_id"] for member in view["members"])
        transforms = {
            member["robot_id"]: se2_of(member["T_world_navigation"])
            for member in view["members"]
        }
        map_routes.publish_optimized_map(
            scope, raster.meta, raster.cells, robots, transforms
        )
        self.built = key
        self.failed = None
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        log.info(
            "Deployment raster %s: %d robots, %d chunks (%d missing), %d points, "
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
        }

    def _skip(
        self, scope: str, key: tuple[str, str], detail: str, retired: list[str]
    ) -> dict[str, Any]:
        # One warning per failing publication, not one per tick.
        if key != self.failed:
            log.warning("Deployment raster %s skipped: %s", scope, detail)
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
