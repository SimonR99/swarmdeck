"""Collaborative-SLAM side channel used only when Swarm-SLAM is running."""

from __future__ import annotations

import json
import urllib.request
import zlib

import numpy as np

from adapters.runtime import TRANSPORT_DEFAULTS

SLAM_GRAPHS: dict[str, dict] = {}
CSLAM_GRID: dict[str, object] = {}


def on_slam_graph(msg) -> None:
    """cslam pose-graph summary, arriving as JSON on a std_msgs/String."""
    try:
        graph = json.loads(msg.data)
    except (ValueError, TypeError):
        return
    rid = graph.get("robot_id")
    if isinstance(rid, str):
        SLAM_GRAPHS[rid] = graph


def on_cslam_grid(msg) -> None:
    CSLAM_GRID["grid"] = msg
    CSLAM_GRID["dirty"] = True


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


def upload_cslam_grid(http_url: str) -> None:
    """Push the back end's merged grid straight to the backend, unmerged."""
    if not CSLAM_GRID.get("dirty"):
        return
    CSLAM_GRID["dirty"] = False
    g = CSLAM_GRID.get("grid")
    if g is None:
        return
    cells = np.array(g.data, dtype=np.int8)
    body = zlib.compress(np.ascontiguousarray(cells).tobytes())
    url = (
        f"{http_url}/api/adapter/global_map?resolution={g.info.resolution}"
        f"&width={g.info.width}&height={g.info.height}"
        f"&origin_x={g.info.origin.position.x}&origin_y={g.info.origin.position.y}"
    )
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                url, data=body, headers={"Content-Type": "application/octet-stream"}
            ),
            timeout=float(TRANSPORT_DEFAULTS["upload_timeout_s"]),
        ).read()
    except Exception as exc:
        print(f"[adapter_sim] cslam grid upload failed: {exc}")
