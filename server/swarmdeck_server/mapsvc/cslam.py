"""Collaborative-SLAM state and common-frame helpers.

The map compositor owns the final merge, but collaborative pose-graph state has
its own lifecycle and transport semantics.  These functions keep that policy
out of the occupancy-grid registration implementation while accepting the
service as a narrow state/remerge collaborator.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .grid_meta import GridMeta

# `cslam` is the legacy Swarm-SLAM path (robots already in a common frame).
# `graph` is the new pose-graph back-end in slam/: occupancy is rendered from
# optimized trajectories, never stitched from local grids.
POSE_GRAPH_MODES = frozenset({"cslam", "graph"})


def is_pose_graph_mode(mode: str) -> bool:
    return mode in POSE_GRAPH_MODES


def set_common_pose(service: Any, robot_id: str, pose: dict[str, float]) -> None:
    with service._state_lock:
        service.common_poses[robot_id] = {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
        }


def common_to_world(service: Any) -> tuple[float, float, float]:
    """Where the collaborative common frame sits in the configured world."""
    with service._state_lock:
        reference = service.reference
        priors = dict(service.transform_priors)
    if reference is None:
        return (0.0, 0.0, 0.0)
    return priors.get(reference, (0.0, 0.0, 0.0))


def common_pose(service: Any, robot_id: str) -> dict[str, float] | None:
    """Return a usable common-frame pose for a robot in the merged cluster."""
    with service._state_lock:
        merge_mode = service.merge_mode
    if not is_pose_graph_mode(merge_mode) or robot_id not in service.global_members():
        return None
    with service._state_lock:
        pose = service.common_poses.get(robot_id)
        pose = dict(pose) if pose is not None else None
    if pose is None:
        return None
    tx, ty, tyaw = common_to_world(service)
    c, s = math.cos(tyaw), math.sin(tyaw)
    return {
        "x": tx + pose["x"] * c - pose["y"] * s,
        "y": ty + pose["x"] * s + pose["y"] * c,
        "yaw": service._wrap_yaw(pose["yaw"] + tyaw),
    }


def set_global_grid(service: Any, meta: GridMeta, cells: np.ndarray) -> None:
    """Adopt a collaborative backend's already-merged common-frame grid."""
    with service._state_lock:
        if not is_pose_graph_mode(service.merge_mode):
            return
        stored_meta = GridMeta(
            meta.resolution,
            meta.width,
            meta.height,
            meta.origin_x,
            meta.origin_y,
        )
        stored_cells = np.array(cells, dtype=np.int8, copy=True)
        if stored_cells.shape != (stored_meta.height, stored_meta.width):
            raise ValueError("grid cells shape does not match metadata")
        service.global_grid = (stored_meta, stored_cells)
    service._remerge()


def set_cslam_origin(
    service: Any, robot_id: str, x: float, y: float, yaw: float, frame: str
) -> None:
    """Record a graph-provided robot transform and cluster frame."""
    with service._state_lock:
        service.cslam_frames[robot_id] = frame
        if not is_pose_graph_mode(service.merge_mode):
            return
        service.transforms[robot_id] = (x, y, yaw)
        if service.reference is None:
            service.reference = robot_id
    service._remerge()


def majority_frame(service: Any) -> str | None:
    counts: dict[str, int] = {}
    with service._state_lock:
        frames = dict(service.cslam_frames)
    for rid, frame in frames.items():
        if frame and service.in_common_frame(rid):
            counts[frame] = counts.get(frame, 0) + 1
    if not counts:
        return None
    return max(counts.items(), key=lambda kv: kv[1])[0]


def set_slam_graph(service: Any, robot_id: str, graph: dict[str, Any]) -> None:
    """Record one adapter's latest collaborative pose-graph view."""
    with service._state_lock:
        service.slam_graphs[robot_id] = dict(graph)


def in_common_frame(service: Any, robot_id: str) -> bool:
    with service._state_lock:
        if service.merge_mode == "graph":
            # Reference is not automatically a member: a singleton has not
            # merged with anyone, and putting it on the fleet map would look
            # like a merge that has not happened. Membership is the back-end's
            # `in_common_frame` flag, set only for robots in a multi-robot
            # component.
            return bool(service.slam_graphs.get(robot_id, {}).get("in_common_frame"))
        if robot_id == service.reference:
            return True
        return bool(service.slam_graphs.get(robot_id, {}).get("in_common_frame"))
