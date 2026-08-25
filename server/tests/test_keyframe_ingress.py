"""Keyframe ingress: identity check, drop-queue, slam update adoption."""

from __future__ import annotations

import numpy as np
import pytest
from fastapi.testclient import TestClient

from swarmdeck_protocol import encode_keyframe
from swarmdeck_server.api.app import app, load_config, map_service
from swarmdeck_server.mapsvc.service import GridMeta


@pytest.fixture(autouse=True)
def _reset_maps():
    load_config()
    yield
    load_config()


def _blob(robot_id: str = "botman_0", seq: int = 1) -> bytes:
    xs = np.linspace(1.0, 4.0, 80)
    points = np.stack([xs, np.zeros(80), np.full(80, 0.4)], axis=1).astype(np.float32)
    return encode_keyframe(
        robot_id=robot_id,
        seq=seq,
        stamp=1.0,
        points=points,
        t_odom_base=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )


def test_keyframe_requires_robot_id():
    with TestClient(app) as c:
        r = c.post("/api/adapter/keyframe", content=_blob())
        assert r.status_code == 400


def test_keyframe_rejects_identity_mismatch():
    with TestClient(app) as c:
        r = c.post(
            "/api/adapter/keyframe?robot_id=tars_0",
            content=_blob("botman_0"),
        )
        assert r.status_code == 400
        assert "mismatch" in r.json()["error"]


def test_keyframe_rejects_garbage_without_crashing():
    with TestClient(app) as c:
        r = c.post("/api/adapter/keyframe?robot_id=r0", content=b"not a keyframe")
        assert r.status_code == 400


def test_matching_keyframe_is_accepted_even_if_slam_is_down():
    """A missing slam process must not 500 or stall the adapter."""
    with TestClient(app) as c:
        r = c.post(
            "/api/adapter/keyframe?robot_id=botman_0",
            content=_blob("botman_0", seq=7),
        )
        assert r.status_code == 200
        body = r.json()
        assert body["ok"] is True
        assert body["seq"] == 7


def test_graph_mode_adopts_a_slam_update_and_rendered_grid():
    n = 20
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[5:15, 5:15] = 0
    cells[8:12, 8:12] = 100
    meta = GridMeta(0.1, n, n, -1.0, -1.0)
    with TestClient(app) as c:
        map_service.set_mode("graph")
        update = c.post(
            "/api/slam/update",
            json={
                "graphs": {
                    "botman_0": {
                        "keyframes": 12,
                        "in_common_frame": True,
                        "inter_robot": ["tars_0"],
                    },
                    "tars_0": {
                        "keyframes": 11,
                        "in_common_frame": True,
                        "inter_robot": ["botman_0"],
                    },
                },
                "origins": {
                    "botman_0": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "component-0"},
                    "tars_0": {"x": 1.5, "y": 0.2, "yaw": 0.1, "frame": "component-0"},
                },
                "common_poses": {
                    "botman_0": {"x": 0.4, "y": 0.1, "yaw": 0.0},
                    "tars_0": {"x": 1.9, "y": 0.3, "yaw": 0.1},
                },
            },
        )
        assert update.status_code == 200
        map_service.set_global_grid(meta, cells)
        status = c.get("/api/map/status").json()
        assert status["mode"] == "graph"
        assert "botman_0" in status["global_members"]
        assert "tars_0" in status["global_members"]
        png = c.get("/api/map")
        assert png.status_code == 200
        assert png.headers["content-type"] == "image/png"


def test_graph_mode_does_not_overlay_unmerged_robots():
    with TestClient(app) as c:
        map_service.set_mode("graph")
        c.post(
            "/api/slam/update",
            json={
                "graphs": {
                    "botman_0": {
                        "keyframes": 4,
                        "in_common_frame": False,
                        "inter_robot": [],
                    }
                },
                "origins": {
                    "botman_0": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "component-0"}
                },
                "common_poses": {},
            },
        )
        members = c.get("/api/map/status").json()["global_members"]
        assert "botman_0" not in members
