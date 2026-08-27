"""Small, ROS-independent helpers for visualising Nav2 costmaps.

Nav2 publishes an ``OccupancyGrid`` in the costmap's frame.  The dashboard
expects an axis-aligned image in the robot's navigation-map frame, so the
adapters do the frame conversion once, before uploading the snapshot.  Keeping
this code independent of ROS makes the wire format and the rasterisation easy
to test without a ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


# A malformed or accidentally unbounded costmap must not make an adapter spend
# hundreds of megabytes rasterising it or make the server retain one forever.
MAX_COSTMAP_CELLS = 16_000_000


@dataclass(frozen=True)
class CostmapSnapshot:
    """A normalized costmap, with cells in standard bottom-up grid order."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    cells: np.ndarray
    frame_id: str


def _yaw_from_quaternion(q: Any) -> float:
    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _transform_points(
    x: np.ndarray, y: np.ndarray, transform: tuple[float, float, float]
) -> tuple[np.ndarray, np.ndarray]:
    tx, ty, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def normalize_costmap(
    msg: Any,
    *,
    target_frame: str,
    transform: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> CostmapSnapshot:
    """Convert a ROS ``OccupancyGrid`` into a dashboard costmap snapshot.

    ``transform`` is the planar transform ``target <- source``.  Values below
    zero remain unknown; all other values are clamped to Nav2's 0..100 range.
    The returned cell array remains bottom-up, matching ROS's grid convention.
    The adapter upload path flips it to the browser's top-down image convention.
    """

    info = getattr(msg, "info", None)
    if info is None:
        raise ValueError("costmap has no info")

    resolution = float(getattr(info, "resolution", 0.0))
    width = int(getattr(info, "width", 0))
    height = int(getattr(info, "height", 0))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("costmap resolution must be positive")
    if width <= 0 or height <= 0 or width * height > MAX_COSTMAP_CELLS:
        raise ValueError("costmap dimensions are invalid")

    raw = np.asarray(getattr(msg, "data", ()), dtype=np.int16).reshape(-1)
    if raw.size != width * height:
        raise ValueError("costmap data size mismatch")
    cells = np.where(raw < 0, -1, np.clip(raw, 0, 100)).astype(np.int8)

    origin = getattr(info, "origin", None)
    position = getattr(origin, "position", None)
    source_x = float(getattr(position, "x", 0.0))
    source_y = float(getattr(position, "y", 0.0))
    source_yaw = _yaw_from_quaternion(getattr(origin, "orientation", None))
    tx, ty, target_yaw = (float(value) for value in transform)
    if not all(math.isfinite(value) for value in (source_x, source_y, source_yaw, tx, ty, target_yaw)):
        raise ValueError("costmap geometry is not finite")

    frame = str(target_frame or "").lstrip("/")
    if not frame:
        header = getattr(msg, "header", None)
        frame = str(getattr(header, "frame_id", "") or "").lstrip("/")

    # The common case is already an axis-aligned map-frame costmap. Avoid a
    # potentially expensive raster pass and retain its exact geometry.
    if abs(source_yaw) < 1e-9 and abs(tx) < 1e-9 and abs(ty) < 1e-9 and abs(target_yaw) < 1e-9:
        return CostmapSnapshot(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=source_x,
            origin_y=source_y,
            cells=np.ascontiguousarray(cells.reshape(height, width)),
            frame_id=frame,
        )

    # Transform the four outer corners to find an axis-aligned target grid.
    corner_x = source_x + np.array([0.0, width * resolution, 0.0, width * resolution])
    corner_y = source_y + np.array([0.0, 0.0, height * resolution, height * resolution])
    source_c = math.cos(source_yaw)
    source_s = math.sin(source_yaw)
    rotated_x = source_x + source_c * (corner_x - source_x) - source_s * (corner_y - source_y)
    rotated_y = source_y + source_s * (corner_x - source_x) + source_c * (corner_y - source_y)
    target_x, target_y = _transform_points(rotated_x, rotated_y, transform)
    min_x = float(np.min(target_x))
    min_y = float(np.min(target_y))
    max_x = float(np.max(target_x))
    max_y = float(np.max(target_y))
    output_origin_x = math.floor(min_x / resolution) * resolution
    output_origin_y = math.floor(min_y / resolution) * resolution
    output_width = max(1, int(math.ceil((max_x - output_origin_x) / resolution - 1e-9)))
    output_height = max(1, int(math.ceil((max_y - output_origin_y) / resolution - 1e-9)))
    if output_width * output_height > MAX_COSTMAP_CELLS:
        raise ValueError("normalized costmap dimensions are too large")

    output = np.full(output_width * output_height, -1, dtype=np.int8)
    known = cells >= 0
    if np.any(known):
        ys, xs = np.nonzero(known.reshape(height, width))
        local_x = (xs.astype(np.float64) + 0.5) * resolution
        local_y = (ys.astype(np.float64) + 0.5) * resolution
        source_center_x = source_x + source_c * local_x - source_s * local_y
        source_center_y = source_y + source_s * local_x + source_c * local_y
        dest_x, dest_y = _transform_points(
            source_center_x, source_center_y, transform
        )
        ix = np.floor((dest_x - output_origin_x) / resolution).astype(np.int64)
        iy = np.floor((dest_y - output_origin_y) / resolution).astype(np.int64)
        valid = (
            (ix >= 0)
            & (ix < output_width)
            & (iy >= 0)
            & (iy < output_height)
        )
        flat = iy[valid] * output_width + ix[valid]
        # Rotating a grid can put more than one source cell into an output
        # cell; the most restrictive cost is the safe visual choice.
        np.maximum.at(output, flat, cells[known][valid])

    return CostmapSnapshot(
        resolution=resolution,
        width=output_width,
        height=output_height,
        origin_x=output_origin_x,
        origin_y=output_origin_y,
        cells=output.reshape(output_height, output_width),
        frame_id=frame,
    )
