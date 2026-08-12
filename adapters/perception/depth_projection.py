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


def _polygon_interior(
    polygon: Sequence[Sequence[float]],
    width: int,
    height: int,
    bounds: tuple[int, int, int, int],
) -> np.ndarray | None:
    """Rasterize a normalized outline over one crop, or return ``None``.

    Even-odd crossing test, vectorized over pixels and looped over the handful
    of polygon edges.  Kept in numpy on purpose: this module is pure geometry
    and unit-testable without OpenCV or ROS.

    ``None`` means "no usable outline" and asks the caller to fall back to the
    box.  A mask that survives here but covers almost nothing is the dangerous
    case -- a few stray pixels would give a confident median in mid-air -- so
    too-small interiors are rejected rather than trusted.
    """
    if not isinstance(polygon, (list, tuple)) or len(polygon) < 3:
        return None
    try:
        points = np.array(
            [[float(x) * width, float(y) * height] for x, y in polygon],
            dtype=np.float64,
        )
    except (TypeError, ValueError):
        return None
    if points.shape[0] < 3 or not np.isfinite(points).all():
        return None

    x0, y0, x1, y1 = bounds
    yy, xx = np.mgrid[y0:y1, x0:x1]
    # Pixel centres, so a polygon edge lying on a pixel boundary is unambiguous.
    xs = xx.astype(np.float64) + 0.5
    ys = yy.astype(np.float64) + 0.5

    inside = np.zeros(xs.shape, dtype=bool)
    following = np.roll(points, -1, axis=0)
    for (ax, ay), (bx, by) in zip(points, following):
        if ay == by:
            continue  # horizontal edge: contributes no crossing
        straddles = (ay > ys) != (by > ys)
        crossing_x = ax + (ys - ay) * (bx - ax) / (by - ay)
        inside ^= straddles & (xs < crossing_x)

    if int(np.count_nonzero(inside)) < 12:
        return None
    return inside


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


def _sample_region(
    bbox: Sequence[float],
    polygon: Sequence[Sequence[float]] | None,
    width: int,
    height: int,
    inset: float,
) -> tuple[tuple[int, int, int, int], np.ndarray | None] | None:
    """Choose which pixels of a detection to read depth from.

    With a segmentation outline we take the object's own pixels and the whole
    box; without one we fall back to a central inset of the box, which is the
    best a rectangle can do.  The difference is not cosmetic: a pool noodle
    lying diagonally measured 27% mask-to-box, so nearly three quarters of the
    "object" a box-only reading averages over is the floor behind it.
    """
    full = _bbox_pixels(bbox, width, height, 0.0)
    if full is not None and polygon:
        interior = _polygon_interior(polygon, width, height, full)
        if interior is not None:
            return full, interior
    inset_box = _bbox_pixels(bbox, width, height, inset)
    return None if inset_box is None else (inset_box, None)


def _intrinsics(camera_info) -> tuple[float, float, float, float] | None:
    k = list(getattr(camera_info, "K", getattr(camera_info, "k", ())))
    if len(k) != 9:
        return None
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
        return None
    return fx, fy, cx, cy


def _depth_metres(image) -> np.ndarray | None:
    """Decode a ROS depth image into a float64 metres array, or None."""
    width = int(getattr(image, "width", 0))
    height = int(getattr(image, "height", 0))
    if width < 2 or height < 2:
        return None
    encoding = str(getattr(image, "encoding", "")).upper()
    if encoding in ("16UC1", "MONO16"):
        scalar_dtype = "u2"
        scale = 0.001
    elif encoding == "32FC1":
        scalar_dtype = "f4"
        scale = 1.0
    else:
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
        ).astype(np.float64)
    except (BufferError, TypeError, ValueError):
        return None
    depth *= scale
    return depth


def _point_for_unaligned_depth_image(
    image,
    depth_info,
    color_info,
    depth_to_color,
    bbox: Sequence[float],
    *,
    polygon: Sequence[Sequence[float]] | None,
    min_range_m: float,
    max_range_m: float,
    depth_scale: float | None,
    inset: float,
    stride: int,
) -> np.ndarray | None:
    """Join a colour bbox to an unsynced, unaligned depth image.

    Colour pixels and depth pixels are not the same grid: different
    resolution, different intrinsics, and a stereo baseline.  The operator
    video can therefore publish immediately while this path, at detector
    rate, deprojects depth, transforms into the colour optical frame, and
    keeps the points that land inside the detection.
    """
    depth = _depth_metres(image)
    if depth is None:
        return None
    if depth_scale is not None:
        default = 0.001 if str(getattr(image, "encoding", "")).upper() in (
            "16UC1",
            "MONO16",
        ) else 1.0
        if not math.isfinite(depth_scale) or depth_scale <= 0.0:
            return None
        depth *= float(depth_scale) / default

    depth_k = _intrinsics(depth_info)
    color_k = _intrinsics(color_info)
    if depth_k is None or color_k is None:
        return None
    fx_d, fy_d, cx_d, cy_d = depth_k
    fx_c, fy_c, cx_c, cy_c = color_k

    color_w = int(getattr(color_info, "width", 0))
    color_h = int(getattr(color_info, "height", 0))
    if color_w < 2 or color_h < 2:
        return None
    region = _sample_region(bbox, polygon, color_w, color_h, inset)
    if region is None:
        return None
    (x0, y0, x1, y1), interior = region

    step = max(1, int(stride))
    height, width = depth.shape
    yy, xx = np.mgrid[0:height:step, 0:width:step]
    z = depth[::step, ::step]
    valid = np.isfinite(z) & (z >= min_range_m) & (z <= max_range_m)
    if int(np.count_nonzero(valid)) < 3:
        return None
    z_v = z[valid]
    u_d = xx[valid].astype(np.float64)
    v_d = yy[valid].astype(np.float64)
    points_depth = np.column_stack(
        ((u_d - cx_d) * z_v / fx_d, (v_d - cy_d) * z_v / fy_d, z_v)
    )
    points_color = transform_points(points_depth, depth_to_color)
    if points_color is None:
        return None
    z_c = points_color[:, 2]
    in_front = z_c > 1e-6
    u_c = fx_c * points_color[:, 0] / np.where(in_front, z_c, np.nan) + cx_c
    v_c = fy_c * points_color[:, 1] / np.where(in_front, z_c, np.nan) + cy_c
    in_box = in_front & (u_c >= x0) & (u_c < x1) & (v_c >= y0) & (v_c < y1)
    if interior is not None:
        ui = np.floor(u_c - x0).astype(np.int64)
        vi = np.floor(v_c - y0).astype(np.int64)
        inside = np.zeros(len(u_c), dtype=bool)
        in_crop = in_box & (ui >= 0) & (vi >= 0) & (ui < interior.shape[1]) & (
            vi < interior.shape[0]
        )
        inside[in_crop] = interior[vi[in_crop], ui[in_crop]]
        in_box = inside
    if int(np.count_nonzero(in_box)) < 3:
        return None
    selected = _foreground_mask(z_v[in_box])
    if selected is None:
        return None
    return np.median(points_depth[in_box][selected], axis=0).astype(np.float64)


def point_for_depth_image(
    image,
    camera_info,
    bbox: Sequence[float],
    *,
    polygon: Sequence[Sequence[float]] | None = None,
    min_range_m: float = 0.15,
    max_range_m: float = 8.0,
    depth_scale: float | None = None,
    inset: float = 0.18,
    color_camera_info=None,
    depth_to_color=None,
    stride: int = 2,
) -> np.ndarray | None:
    """Deproject a depth-image crop into camera-frame XYZ.

    Supports the standard ROS ``16UC1`` (millimetres) and ``32FC1`` (metres)
    encodings.

    When ``color_camera_info`` is omitted, the depth image must be aligned to
    the RGB pixels and ``camera_info`` is the colour-camera intrinsics.

    When ``color_camera_info`` is supplied, the bbox lives in colour pixels
    and the depth image may be a different size, rate, and optical frame.
    ``depth_to_color`` is then the rigid transform taking depth-optical XYZ
    into colour-optical XYZ; omit it only when those frames coincide.
    The returned point stays in the *depth* optical frame so the caller can
    keep using the depth image's ``frame_id`` for the map lookup.
    """
    if color_camera_info is not None:
        if depth_to_color is None:
            depth_to_color = type(
                "Identity",
                (),
                {
                    "translation": type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                    "rotation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                },
            )()
        return _point_for_unaligned_depth_image(
            image,
            camera_info,
            color_camera_info,
            depth_to_color,
            bbox,
            polygon=polygon,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            depth_scale=depth_scale,
            inset=inset,
            stride=stride,
        )
    width = int(getattr(image, "width", 0))
    height = int(getattr(image, "height", 0))
    if width < 2 or height < 2:
        return None
    region = _sample_region(bbox, polygon, width, height, inset)
    if region is None:
        return None
    (x0, y0, x1, y1), interior = region

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
    if interior is not None:
        valid &= interior
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
    polygon: Sequence[Sequence[float]] | None = None,
    min_range_m: float = 0.15,
    max_range_m: float = 8.0,
    inset: float = 0.18,
) -> np.ndarray | None:
    """Return a robust foreground XYZ sample for a normalised RGB box.

    ``cloud`` must be organised (height > 1) and pixel-aligned with the RGB
    image.  Given a segmentation outline we read the object's own pixels;
    otherwise a central inset avoids box edges, where background depth
    dominates.  From that crop we select the nearest coherent depth band rather
    than the absolute nearest point, which rejects isolated flying pixels while
    keeping a small object in front of a wall.
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

    region = _sample_region(bbox, polygon, width, height, inset)
    if region is None:
        return None
    (x0, y0, x1, y1), interior = region

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
    if interior is not None:
        valid &= interior.reshape(-1)
    points = points[valid]
    ranges = ranges[valid]
    selected = _foreground_mask(ranges)
    if selected is None:
        return None
    foreground = points[selected]
    return np.median(foreground, axis=0).astype(np.float64)


def transform_points(points: np.ndarray, transform) -> np.ndarray | None:
    """Apply a geometry_msgs-style rigid transform to an (N, 3) point array."""
    vector = np.asarray(points, dtype=np.float64)
    if vector.ndim != 2 or vector.shape[1] != 3 or vector.shape[0] == 0:
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
    return result


def transform_point(point: Sequence[float], transform) -> np.ndarray | None:
    """Apply a geometry_msgs-style rigid transform to one XYZ point."""
    vector = np.asarray(point, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        return None
    transformed = transform_points(vector.reshape(1, 3), transform)
    if transformed is None:
        return None
    result = transformed[0]
    return result if np.isfinite(result).all() else None
