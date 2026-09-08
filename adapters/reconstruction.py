"""Calibrated color projection shared by capture and cloud upload (no ROS imports)."""

from __future__ import annotations
import numpy as np


def colorize_points(
    points, rgb, intrinsics, camera_from_points, *, depth=None, tolerance=0.08
):
    """Project into a rectified camera, keeping only the nearest surface per pixel.

    RGB is uint8 HxWx3; optional aligned depth is in metres. Returns colors and
    an explicit visibility mask so unobserved samples never acquire fake colors.
    """
    points = np.asarray(points)
    rgb = np.asarray(rgb)
    k = np.asarray(intrinsics, dtype=float)
    transform = np.asarray(camera_from_points, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or rgb.ndim != 3
        or rgb.shape[2] != 3
        or rgb.dtype != np.uint8
    ):
        raise ValueError("expected Nx3 points and uint8 RGB image")
    if (
        k.shape != (3, 3)
        or transform.shape != (4, 4)
        or not np.isfinite(k).all()
        or not np.isfinite(transform).all()
        or k[0, 0] <= 0
        or k[1, 1] <= 0
    ):
        raise ValueError("invalid calibration")
    h, w = rgb.shape[:2]
    if depth is not None and np.shape(depth) != (h, w):
        raise ValueError("depth must be registered to the RGB pixel grid")
    camera = points @ transform[:3, :3].T + transform[:3, 3]
    valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > 0.05)
    idx = np.flatnonzero(valid)
    projected = camera[idx] @ k.T
    pixels = np.rint(projected[:, :2] / projected[:, 2, None]).astype(np.int64)
    inside = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < w)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < h)
    )
    idx, pixels = idx[inside], pixels[inside]
    flat = pixels[:, 1] * w + pixels[:, 0]
    nearest = np.full(h * w, np.inf)
    np.minimum.at(nearest, flat, camera[idx, 2])
    visible = camera[idx, 2] <= nearest[flat] + tolerance
    if depth is not None:
        observed = np.asarray(depth)[pixels[:, 1], pixels[:, 0]]
        visible &= (
            np.isfinite(observed)
            & (observed > 0)
            & (np.abs(camera[idx, 2] - observed) <= tolerance)
        )
    colors = np.full((len(points), 3), 148, dtype=np.uint8)
    mask = np.zeros(len(points), dtype=bool)
    mask[idx[visible]] = True
    colors[idx[visible]] = rgb[pixels[visible, 1], pixels[visible, 0]]
    return colors, mask


def colorize_ros_rgbd(points, image, depth_image, info, camera_from_points):
    """RGBA for rectified, aligned ROS RGB-D; alpha marks measured colors only."""
    from adapters.perception.depth_projection import _depth_metres

    encoding = str(image.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None or any(abs(float(d)) > 1e-8 for d in info.d):
        return None
    width, height = int(image.width), int(image.height)
    if (width, height) != (int(info.width), int(info.height)):
        return None
    if int(image.step) < width * channels:
        return None
    pixels = np.ndarray(
        (height, width, channels),
        dtype=np.uint8,
        buffer=memoryview(image.data),
        strides=(int(image.step), channels, 1),
    )
    rgb = pixels[:, :, :3]
    if encoding.startswith("bgr"):
        rgb = rgb[:, :, ::-1]
    depth = _depth_metres(depth_image)
    if depth is None or depth.shape != (height, width):
        return None
    colors, visible = colorize_points(
        points,
        rgb,
        np.asarray(info.k).reshape(3, 3),
        camera_from_points,
        depth=depth,
        tolerance=0.15,
    )
    if not visible.any():
        return None
    return np.column_stack((colors, visible.astype(np.uint8) * 255))
