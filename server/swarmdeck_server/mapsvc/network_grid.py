"""Spatial accumulation of robot-side Wi-Fi link quality."""

from __future__ import annotations

import math

import numpy as np

from .grid_meta import GridMeta


NO_DATA = 255
EXPAND_MARGIN_M = 3.0
MAX_SIDE_M = 400.0
MAX_CELL_WEIGHT = 6.0


class NetworkGridAccumulator:
    """A coarse, smoothed quality grid in one robot's local map frame.

    Each sample is stamped with the pose captured in the same ``robot_state``
    packet.  A small radial kernel makes the travelled path legible as a
    heatmap without pretending that an unvisited room has been measured.
    """

    def __init__(
        self,
        origin_x: float,
        origin_y: float,
        *,
        resolution: float = 0.25,
        size_m: float = 30.0,
        radius_m: float = 0.75,
    ) -> None:
        if resolution <= 0 or size_m <= 0 or radius_m <= 0:
            raise ValueError("resolution, size_m and radius_m must be positive")
        n = max(1, int(math.ceil(size_m / resolution)))
        self.meta = GridMeta(
            resolution=resolution,
            width=n,
            height=n,
            origin_x=origin_x - n * resolution / 2,
            origin_y=origin_y - n * resolution / 2,
        )
        self.radius_m = radius_m
        self._sum = np.zeros((n, n), dtype=np.float32)
        self._weight = np.zeros((n, n), dtype=np.float32)
        self.revision = 0

    def _expand_to_cover(self, min_x: float, max_x: float, min_y: float, max_y: float) -> bool:
        meta = self.meta
        res = meta.resolution
        cur_max_x = meta.origin_x + meta.width * res
        cur_max_y = meta.origin_y + meta.height * res
        if (
            min_x >= meta.origin_x
            and max_x < cur_max_x
            and min_y >= meta.origin_y
            and max_y < cur_max_y
        ):
            return True

        new_min_x = min(meta.origin_x, math.floor((min_x - EXPAND_MARGIN_M) / res) * res)
        new_min_y = min(meta.origin_y, math.floor((min_y - EXPAND_MARGIN_M) / res) * res)
        new_max_x = max(cur_max_x, math.ceil((max_x + EXPAND_MARGIN_M) / res) * res)
        new_max_y = max(cur_max_y, math.ceil((max_y + EXPAND_MARGIN_M) / res) * res)
        if new_max_x - new_min_x > MAX_SIDE_M or new_max_y - new_min_y > MAX_SIDE_M:
            return False

        width = int(round((new_max_x - new_min_x) / res))
        height = int(round((new_max_y - new_min_y) / res))
        sums = np.zeros((height, width), dtype=np.float32)
        weights = np.zeros((height, width), dtype=np.float32)
        off_x = int(round((meta.origin_x - new_min_x) / res))
        off_y = int(round((meta.origin_y - new_min_y) / res))
        sums[off_y : off_y + meta.height, off_x : off_x + meta.width] = self._sum
        weights[off_y : off_y + meta.height, off_x : off_x + meta.width] = self._weight
        self._sum = sums
        self._weight = weights
        self.meta = GridMeta(res, width, height, new_min_x, new_min_y)
        return True

    def integrate(self, x: float, y: float, quality_pct: float) -> bool:
        """Add one link sample. Returns whether the grid changed."""
        if not all(math.isfinite(v) for v in (x, y, quality_pct)):
            return False
        if not 0.0 <= quality_pct <= 100.0:
            return False
        radius = self.radius_m
        if not self._expand_to_cover(x - radius, x + radius, y - radius, y + radius):
            return False

        meta = self.meta
        gx0 = max(0, int(math.floor((x - radius - meta.origin_x) / meta.resolution)))
        gx1 = min(meta.width, int(math.ceil((x + radius - meta.origin_x) / meta.resolution)))
        gy0 = max(0, int(math.floor((y - radius - meta.origin_y) / meta.resolution)))
        gy1 = min(meta.height, int(math.ceil((y + radius - meta.origin_y) / meta.resolution)))
        if gx0 >= gx1 or gy0 >= gy1:
            return False

        xs = meta.origin_x + (np.arange(gx0, gx1) + 0.5) * meta.resolution
        ys = meta.origin_y + (np.arange(gy0, gy1) + 0.5) * meta.resolution
        xx, yy = np.meshgrid(xs, ys)
        distance = np.hypot(xx - x, yy - y)
        mask = distance <= radius
        if not mask.any():
            return False
        # Keep a small non-zero edge weight so adjacent samples join into a
        # continuous band while the centre remains the strongest evidence.
        kernel = np.where(mask, np.maximum(0.08, 1.0 - distance / radius), 0.0).astype(np.float32)
        target_sum = self._sum[gy0:gy1, gx0:gx1]
        target_weight = self._weight[gy0:gy1, gx0:gx1]
        target_sum += kernel * quality_pct
        target_weight += kernel
        overflow = target_weight > MAX_CELL_WEIGHT
        if overflow.any():
            scale = np.where(overflow, MAX_CELL_WEIGHT / np.maximum(target_weight, 1e-6), 1.0).astype(np.float32)
            target_sum *= scale
            target_weight *= scale
        self.revision += 1
        return True

    def quality_grid(self) -> np.ndarray:
        """Return uint8 quality percentages; 255 marks never-sampled cells."""
        out = np.full(self._sum.shape, NO_DATA, dtype=np.uint8)
        sampled = self._weight > 0
        if sampled.any():
            out[sampled] = np.clip(
                np.rint(self._sum[sampled] / self._weight[sampled]), 0, 100
            ).astype(np.uint8)
        return out
