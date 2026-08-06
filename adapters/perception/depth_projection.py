"""Depth-camera helpers shared by the ROS 1 and ROS 2 hardware adapters.

The detector deliberately stays an RGB-only component.  This module joins its
normalised image box to an *organised, RGB-aligned* PointCloud2 and transforms
the resulting camera-space point into the robot's map frame.  Keeping the ROS
message duck-typed makes the geometry independently unit-testable.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def _bbox_pixels(
    bbox: Sequence[float], width: int, height: int, inset: float
) -> tuple[int, int, int, int] | None:
    try:
        bx, by, bw, bh = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (bx, by, bw, bh)) or bw <= 0 or bh <= 0:
        return None
    inset = max(0.0, min(0.45, float(inset)))
    x0 = max(0, min(width - 1, int(math.floor((bx + bw * inset) * width))))
    x1 = max(x0 + 1, min(width, int(math.ceil((bx + bw * (1.0 - inset)) * width))))
    y0 = max(0, min(height - 1, int(math.floor((by + bh * inset) * height))))
    y1 = max(y0 + 1, min(height, int(math.ceil((by + bh * (1.0 - inset)) * height))))
    return x0, y0, x1, y1


def _foreground_mask(ranges: np.ndarray) -> np.ndarray | None:
    if len(ranges) < 3:
        return None
    foreground_range = float(np.percentile(ranges, 30.0))
    tolerance = max(0.06, foreground_range * 0.06)
    selected = np.abs(ranges - foreground_range) <= tolerance
    if int(np.count_nonzero(selected)) < 3:
        nearest = np.argsort(np.abs(ranges - foreground_range))[: min(5, len(ranges))]
        selected = np.zeros(len(ranges), dtype=bool)
        selected[nearest] = True
    return selected


def point_for_depth_image(
    image,
    camera_info,
    bbox: Sequence[float],
    *,
    min_range_m: float = 0.15,
    max_range_m: float = 8.0,
    depth_scale: float | None = None,
    inset: float = 0.18,
) -> np.ndarray | None:
    """Deproject an RGB-aligned depth-image crop into camera-frame XYZ.

    Supports the standard ROS ``16UC1`` (millimetres) and ``32FC1`` (metres)
    encodings.  CameraInfo supplies the color-camera intrinsics because the
    depth image is required to be aligned to the RGB pixels.
    """
    width = int(getattr(image, "width", 0))
    height = int(getattr(image, "height", 0))
    if width < 2 or height < 2:
        return None
    pixels = _bbox_pixels(bbox, width, height, inset)
    if pixels is None:
        return None
    x0, y0, x1, y1 = pixels

    encoding = str(getattr(image, "encoding", "")).upper()
    if encoding in ("16UC1", "MONO16"):
        scalar_dtype = "u2"
        default_scale = 0.001
    elif encoding == "32FC1":
        scalar_dtype = "f4"
        default_scale = 1.0
    else:
        return None
    scale = default_scale if depth_scale is None else float(depth_scale)
    if not math.isfinite(scale) or scale <= 0.0:
        return None

    endian = ">" if bool(getattr(image, "is_bigendian", False)) else "<"
    dtype = np.dtype(f"{endian}{scalar_dtype}")
    step = int(getattr(image, "step", width * dtype.itemsize))
    if step < width * dtype.itemsize:
        return None
    try:
        depth = np.ndarray(
            (height, width),
            dtype=dtype,
            buffer=memoryview(image.data),
            strides=(step, dtype.itemsize),
        )[y0:y1, x0:x1].astype(np.float64)
    except (BufferError, TypeError, ValueError):
        return None
    depth *= scale

    k = list(getattr(camera_info, "K", getattr(camera_info, "k", ())))
    if len(k) != 9:
        return None
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
        return None

    yy, xx = np.mgrid[y0:y1, x0:x1]
    valid = np.isfinite(depth) & (depth >= min_range_m) & (depth <= max_range_m)
    z = depth[valid]
    if len(z) < 3:
        return None
    u = xx[valid].astype(np.float64)
    v = yy[valid].astype(np.float64)
    selected = _foreground_mask(z)
    if selected is None:
        return None
    z, u, v = z[selected], u[selected], v[selected]
    points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    return np.median(points, axis=0).astype(np.float64)


def point_for_bbox(
    cloud,
    bbox: Sequence[float],
    *,
    min_range_m: float = 0.15,
    max_range_m: float = 8.0,
    inset: float = 0.18,
) -> np.ndarray | None:
    """Return a robust foreground XYZ sample for a normalised RGB box.

    ``cloud`` must be organised (height > 1) and pixel-aligned with the RGB
    image.  A central inset avoids box edges, where background depth dominates.
    From that crop we select the nearest coherent depth band rather than the
    absolute nearest point, which rejects isolated flying pixels while keeping
    a small object in front of a wall.
    """
    width = int(getattr(cloud, "width", 0))
    height = int(getattr(cloud, "height", 0))
    point_step = int(getattr(cloud, "point_step", 0))
    row_step = int(getattr(cloud, "row_step", width * point_step))
    if width < 2 or height < 2 or point_step <= 0 or row_step < width * point_step:
        return None

    offsets = {
        field.name: int(field.offset)
        for field in getattr(cloud, "fields", ())
        if field.name in ("x", "y", "z")
    }
    if len(offsets) != 3 or any(offset + 4 > point_step for offset in offsets.values()):
        return None

    pixels = _bbox_pixels(bbox, width, height, inset)
    if pixels is None:
        return None
    x0, y0, x1, y1 = pixels

    endian = ">" if bool(getattr(cloud, "is_bigendian", False)) else "<"
    dtype = np.dtype(f"{endian}f4")
    try:
        buffer = memoryview(cloud.data)
        axes = [
            np.ndarray(
                (height, width),
                dtype=dtype,
                buffer=buffer,
                offset=offsets[axis],
                strides=(row_step, point_step),
            )[y0:y1, x0:x1]
            for axis in ("x", "y", "z")
        ]
    except (BufferError, TypeError, ValueError):
        return None

    points = np.stack(axes, axis=-1).reshape(-1, 3).astype(np.float64, copy=False)
    ranges = np.linalg.norm(points, axis=1)
    valid = (
        np.isfinite(points).all(axis=1)
        & np.isfinite(ranges)
        & (ranges >= float(min_range_m))
        & (ranges <= float(max_range_m))
    )
    points = points[valid]
    ranges = ranges[valid]
    selected = _foreground_mask(ranges)
    if selected is None:
        return None
    foreground = points[selected]
    return np.median(foreground, axis=0).astype(np.float64)


def transform_point(point: Sequence[float], transform) -> np.ndarray | None:
    """Apply a geometry_msgs-style rigid transform to one XYZ point."""
    vector = np.asarray(point, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        return None

    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    quaternion /= norm
    q_xyz = quaternion[:3]
    twice_cross = 2.0 * np.cross(q_xyz, vector)
    rotated = vector + quaternion[3] * twice_cross + np.cross(q_xyz, twice_cross)
    translation = transform.translation
    result = rotated + np.array(
        [translation.x, translation.y, translation.z], dtype=np.float64
    )
    return result if np.isfinite(result).all() else None
