"""Deployment-frame transforms and robot-local network telemetry.

The server no longer builds or plans occupancy maps.  Three-dimensional replica
products are rasterized by :mod:`api.deployment_raster`; this service only keeps
the surveyed deployment placement used by that view and the independent Wi-Fi
heatmap stream.
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import Any

from .grid_meta import GridMeta
from .network_grid import NetworkGridAccumulator
from .output import network_robot_ids, network_snapshot, take_network_patch


class MapService:
    def __init__(self, resolution: float = 0.25, size_m: float = 30.0) -> None:
        self._state_lock = threading.RLock()
        # Deployment placements retain the surveyed vertical origin as well as
        # x/y/yaw.  Consumers that publish the 2D map header intentionally
        # project this to SE(2), while 3D replica composition uses all four
        # values to build an SE(3) transform.
        self.transforms: dict[str, tuple[float, float, float, float]] = {}
        self._network_resolution = float(resolution)
        self._network_size = float(size_m)
        self._network_grids: dict[str, NetworkGridAccumulator] = {}
        self._network_prev: dict[str, Any] = {}
        self._network_seq: dict[str, int] = {}
        self._ingest_lock = asyncio.Lock()

    @staticmethod
    def _wrap_yaw(yaw: float) -> float:
        return (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi

    def set_transform(
        self, robot_id: str, x: float, y: float, yaw: float, z: float = 0.0
    ) -> None:
        values = (float(x), float(y), float(z), self._wrap_yaw(float(yaw)))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("deployment transform must be finite")
        with self._state_lock:
            self.transforms[str(robot_id)] = values

    @staticmethod
    def _placement(values) -> tuple[float, float, float, float]:
        """Return the canonical surveyed ``(x, y, z, yaw)`` placement."""
        x, y, z, yaw = values
        return float(x), float(y), float(z), float(yaw)

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            transforms = {}
            for robot_id, values in sorted(self.transforms.items()):
                x, y, _z, yaw = self._placement(values)
                transforms[robot_id] = {"x": x, "y": y, "yaw": yaw}
        return {"transforms": transforms, "members": sorted(transforms)}

    def robot_to_world(
        self, robot_id: str, point: dict[str, float]
    ) -> dict[str, float]:
        with self._state_lock:
            values = self.transforms.get(robot_id, (0.0, 0.0, 0.0, 0.0))
        tx, ty, tz, yaw = self._placement(values)
        c, s = math.cos(yaw), math.sin(yaw)
        result = dict(point)
        x, y = float(point["x"]), float(point["y"])
        result["x"], result["y"] = tx + x * c - y * s, ty + x * s + y * c
        if "z" in point:
            result["z"] = tz + float(point["z"])
        if "yaw" in point:
            result["yaw"] = self._wrap_yaw(float(point["yaw"]) + yaw)
        return result

    def world_to_robot(
        self, robot_id: str, point: dict[str, float]
    ) -> dict[str, float]:
        with self._state_lock:
            values = self.transforms.get(robot_id, (0.0, 0.0, 0.0, 0.0))
        tx, ty, tz, yaw = self._placement(values)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = float(point["x"]) - tx, float(point["y"]) - ty
        result = dict(point)
        result["x"], result["y"] = dx * c + dy * s, -dx * s + dy * c
        if "z" in point:
            result["z"] = float(point["z"]) - tz
        if "yaw" in point:
            result["yaw"] = self._wrap_yaw(float(point["yaw"]) - yaw)
        return result

    def ingest_network_sample(
        self, robot_id: str, x: float, y: float, quality_pct: float
    ) -> bool:
        if not robot_id or not all(
            math.isfinite(float(value)) for value in (x, y, quality_pct)
        ):
            return False
        with self._state_lock:
            grid = self._network_grids.get(robot_id)
            if grid is None:
                grid = NetworkGridAccumulator(
                    0.0,
                    0.0,
                    resolution=self._network_resolution,
                    size_m=self._network_size,
                )
                self._network_grids[robot_id] = grid
            return grid.integrate(float(x), float(y), float(quality_pct))

    def network_robot_ids(self) -> list[str]:
        return network_robot_ids(self)

    def network_snapshot(self, robot_id: str) -> dict[str, Any] | None:
        return network_snapshot(self, robot_id)

    def take_network_patch(self, robot_id: str) -> dict[str, Any] | None:
        return take_network_patch(self, robot_id)

    def reset_robot(self, robot_id: str | None = None) -> list[str]:
        with self._state_lock:
            if robot_id is None:
                ids = sorted(set(self.transforms) | set(self._network_grids))
                self._network_grids.clear()
                self._network_prev.clear()
                self._network_seq.clear()
                return ids
            existed = robot_id in self.transforms or robot_id in self._network_grids
            self._network_grids.pop(robot_id, None)
            self._network_prev.pop(robot_id, None)
            self._network_seq.pop(robot_id, None)
            return [robot_id] if existed else []

    async def reset_robot_async(self, robot_id: str | None = None) -> list[str]:
        async with self._ingest_lock:
            return await asyncio.to_thread(self.reset_robot, robot_id)

    async def reset_async(self) -> None:
        await self.reset_robot_async()


map_service = MapService()
