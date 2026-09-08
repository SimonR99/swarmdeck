"""Warp a common-frame occupancy grid into one robot's map frame for Nav2.

The pose-graph back-end renders occupancy in the component/world frame. Nav2
plans in the robot's own ``map`` / ``map_frame``. These are related by the
``T_world_map`` the optimizer already publishes: this module applies that
transform to the *grid*, so the robot can load an OccupancyGrid without a new
TF frame.

Local costmaps must not consume this product. Collision authority stays on
live sensors; a foreign map here is a global-planner hypothesis.
"""

from __future__ import annotations

import math

import numpy as np

from .grid_meta import GridMeta

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)


def warp_to_robot_frame(
    meta: GridMeta,
    cells: np.ndarray,
    t_world_map: tuple[float, float, float],
    *,
    padding_m: float = 1.0,
) -> tuple[GridMeta, np.ndarray]:
    """Express a world-frame occupancy grid in the robot map frame.

    ``t_world_map`` is ``(x, y, yaw)`` such that a map-frame point maps to
    world as ``p_world = R(yaw) @ p_map + t``. Occupied wins over free when
    a destination cell lands on more than one source interpretation: the
    nearest-neighbour sample of the source grid is used, so this is a
    resampling, not a vote.
    """
    src = np.asarray(cells, dtype=np.int8)
    if src.shape != (meta.height, meta.width):
        raise ValueError("grid cells shape does not match metadata")
    res = float(meta.resolution)
    if res <= 0.0:
        raise ValueError("resolution must be positive")

    tx, ty, yaw = (float(t_world_map[0]), float(t_world_map[1]), float(t_world_map[2]))
    c, s = math.cos(yaw), math.sin(yaw)
    src_w = meta.width * res
    src_h = meta.height * res
    corners = (
        (meta.origin_x, meta.origin_y),
        (meta.origin_x + src_w, meta.origin_y),
        (meta.origin_x, meta.origin_y + src_h),
        (meta.origin_x + src_w, meta.origin_y + src_h),
    )
    mapped_x: list[float] = []
    mapped_y: list[float] = []
    for wx, wy in corners:
        dx, dy = wx - tx, wy - ty
        mapped_x.append(dx * c + dy * s)
        mapped_y.append(-dx * s + dy * c)

    pad = max(0.0, float(padding_m))
    min_x = math.floor((min(mapped_x) - pad) / res) * res
    min_y = math.floor((min(mapped_y) - pad) / res) * res
    max_x = math.ceil((max(mapped_x) + pad) / res) * res
    max_y = math.ceil((max(mapped_y) + pad) / res) * res
    width = max(1, int(round((max_x - min_x) / res)))
    height = max(1, int(round((max_y - min_y) / res)))

    xs = min_x + (np.arange(width, dtype=np.float64) + 0.5) * res
    ys = min_y + (np.arange(height, dtype=np.float64) + 0.5) * res
    wx = tx + xs[None, :] * c - ys[:, None] * s
    wy = ty + xs[None, :] * s + ys[:, None] * c
    col = np.floor((wx - meta.origin_x) / res).astype(np.int64)
    row = np.floor((wy - meta.origin_y) / res).astype(np.int64)
    valid = (col >= 0) & (col < meta.width) & (row >= 0) & (row < meta.height)

    out = np.full((height, width), UNKNOWN, dtype=np.int8)
    out[valid] = src[row[valid], col[valid]]
    return GridMeta(res, width, height, min_x, min_y), out
