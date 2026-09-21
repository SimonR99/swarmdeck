"""Read-side map products and transport rendering.

These functions deliberately accept a ``MapService`` collaborator instead of
owning map state.  They take the service's short state lock only while copying
mutable accumulators; PNG/JSON compression happens after the lock is released.
"""

from __future__ import annotations

import base64
import io
import zlib
from typing import Any

import numpy as np

from .grid_meta import GridMeta

UNKNOWN = -1

UNKNOWN_RGB = (214, 218, 224)
FREE_RGB = (255, 255, 255)
OCCUPIED_RGB = (52, 58, 68)


def consolidate_voxel_centroids(
    points: np.ndarray, voxel_size: float = 0.04
) -> np.ndarray:
    """Downsample and consolidate 3D points onto voxel centroids, denoising planar surfaces."""
    if points.shape[0] <= 1:
        return points
    voxel_coords = np.floor(points / voxel_size).astype(np.int64)
    _, unique_indices, inverse_indices, counts = np.unique(
        voxel_coords, axis=0, return_index=True, return_inverse=True, return_counts=True
    )
    sum_points = np.zeros((len(unique_indices), 3), dtype=np.float64)
    np.add.at(sum_points, inverse_indices, points.astype(np.float64))
    centroids = sum_points / counts[:, None]
    return centroids.astype(np.float32)


def network_robot_ids(service: Any) -> list[str]:
    with service._state_lock:
        return list(service._network_grids)


def _network_payload(
    robot_id: str,
    meta: GridMeta,
    display: np.ndarray,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    seq: int,
) -> dict[str, Any]:
    sub = np.ascontiguousarray(display[y0:y1, x0:x1])
    return {
        "type": "network_patch",
        "robot_id": robot_id,
        "seq": seq,
        "resolution": meta.resolution,
        "origin": {"x": meta.origin_x, "y": meta.origin_y},
        "width": meta.width,
        "height": meta.height,
        "x0": x0,
        "y0": y0,
        "w": x1 - x0,
        "h": y1 - y0,
        "data": base64.b64encode(zlib.compress(sub.tobytes(), 1)).decode(),
    }


def take_network_patch(service: Any, robot_id: str) -> dict[str, Any] | None:
    """Return the changed top-down heatmap rectangle for one robot."""
    with service._state_lock:
        acc = service._network_grids.get(robot_id)
        if acc is None:
            return None
        # Accumulators use ordinary Cartesian row order (low y first); browser
        # canvases are top-down, so patches are flipped here.
        display = np.flipud(acc.quality_grid())
        previous = service._network_prev.get(robot_id)
        if previous is None or previous.shape != display.shape:
            y0, x0 = 0, 0
            y1, x1 = display.shape
        else:
            changed = display != previous
            if not changed.any():
                return None
            ys, xs = np.where(changed)
            y0, y1 = int(ys.min()), int(ys.max()) + 1
            x0, x1 = int(xs.min()), int(xs.max()) + 1
        service._network_prev[robot_id] = display.copy()
        seq = service._network_seq.get(robot_id, 0) + 1
        service._network_seq[robot_id] = seq
        meta = GridMeta(
            acc.meta.resolution,
            acc.meta.width,
            acc.meta.height,
            acc.meta.origin_x,
            acc.meta.origin_y,
        )
    return _network_payload(robot_id, meta, display, x0, y0, x1, y1, seq)


def network_snapshot(service: Any, robot_id: str) -> dict[str, Any] | None:
    """Return a full heatmap without disturbing websocket dirty tracking."""
    with service._state_lock:
        acc = service._network_grids.get(robot_id)
        if acc is None:
            return None
        display = np.flipud(acc.quality_grid())
        height, width = display.shape
        meta = GridMeta(
            acc.meta.resolution,
            acc.meta.width,
            acc.meta.height,
            acc.meta.origin_x,
            acc.meta.origin_y,
        )
        seq = service._network_seq.get(robot_id, 0)
    return _network_payload(robot_id, meta, display, 0, 0, width, height, seq)


def grid_png(meta: GridMeta, cells: np.ndarray) -> bytes:
    """Render one occupancy grid in the frontend's top-down orientation."""
    from PIL import Image

    img = np.zeros((meta.height, meta.width, 3), dtype=np.uint8)
    img[...] = UNKNOWN_RGB
    img[cells == 0] = FREE_RGB
    img[cells >= 50] = OCCUPIED_RGB
    img = np.flipud(img)
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="PNG", optimize=True)
    return buf.getvalue()
