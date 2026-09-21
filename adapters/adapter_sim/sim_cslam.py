"""Collaborative Swarm-SLAM graph side channel."""

from __future__ import annotations

import json

SLAM_GRAPHS: dict[str, dict] = {}


def on_slam_graph(msg) -> None:
    """Store a graph summary arriving as JSON on a string topic."""
    try:
        graph = json.loads(msg.data)
    except (ValueError, TypeError):
        return
    rid = graph.get("robot_id")
    if isinstance(rid, str):
        SLAM_GRAPHS[rid] = graph


def slam_graph_payload(
    robot_id: str,
    t0: float,
    graph: dict,
    origin: dict | None,
    now: float,
) -> dict:
    payload = {
        "type": "slam_graph",
        "robot_id": robot_id,
        "t_mono": round(now - t0, 4),
        "keyframes": graph.get("keyframes", 0),
        "in_common_frame": graph.get("in_common_frame", False),
        "residual": graph.get("residual"),
        "inter_robot": graph.get("inter_robot", []),
    }
    if origin is not None:
        payload["origin"] = origin
    common = graph.get("common")
    if isinstance(common, dict) and graph.get("in_common_frame"):
        payload["common_pose"] = {
            "x": float(common.get("x", 0.0)),
            "y": float(common.get("y", 0.0)),
            "yaw": float(common.get("yaw", 0.0)),
        }
    return payload
