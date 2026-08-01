"""Backend tests. None of these import ROS — acceptance criterion 12."""

from __future__ import annotations

import base64
import zlib

import numpy as np
import pytest
from fastapi.testclient import TestClient

from swarmdeck_server.api.app import app, load_config
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.mapsvc.service import GridMeta, MapService


@pytest.fixture(autouse=True)
def _cfg():
    load_config()


def test_backend_imports_no_ros():
    import sys

    assert not any(m.startswith(("rclpy", "rospy", "ros2")) for m in sys.modules)


def test_config_endpoint():
    with TestClient(app) as c:
        r = c.get("/api/config")
        assert r.status_code == 200
        assert r.json()["protocol"] == 1


def test_map_png_roundtrip():
    with TestClient(app) as c:
        r = c.get("/api/map")
        assert r.status_code == 200
        assert r.headers["content-type"] == "image/png"


def test_registry_capabilities_and_neglect():
    reg = Registry()
    reg.hello(
        {"robot_id": "r0", "robot_type": "spot", "capabilities": ["navigate", "map"]}, sink=None
    )
    assert reg.can("r0", "navigate")
    assert not reg.can("r0", "camera")
    assert reg.robots["r0"].unattended_s < 1.0


def test_duplicate_goal_rejected():
    reg = Registry()
    reg.hello({"robot_id": "r0", "capabilities": ["navigate"]}, sink=None)
    reg.hello({"robot_id": "r1", "capabilities": ["navigate"]}, sink=None)
    reg.update_state({"robot_id": "r0", "goal": {"x": 5.0, "y": 5.0}})
    assert reg.goal_taken({"x": 5.1, "y": 5.1}, exclude="r1") == "r0"
    assert reg.goal_taken({"x": 9.0, "y": 9.0}, exclude="r1") is None


def test_map_merge_two_robots():
    """Occupied wins over free; free wins over unknown."""
    svc = MapService(resolution=0.1, size_m=10.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)

    a = np.full((n, n), -1, dtype=np.int8)
    a[10:20, 10:20] = 0
    b = np.full((n, n), -1, dtype=np.int8)
    b[15:25, 15:25] = 100

    svc.set_transform("r0", 0, 0, 0)
    svc.set_transform("r1", 0, 0, 0)
    svc.ingest("r0", meta, a)
    svc.ingest("r1", meta, b)

    assert svc.merged[12, 12] == 0
    assert svc.merged[17, 17] == 100  # occupied beats free in the overlap
    assert svc.merged[2, 2] == -1


def test_patch_is_incremental():
    svc = MapService(resolution=0.1, size_m=10.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    svc.set_transform("r0", 0, 0, 0)

    cells = np.full((n, n), -1, dtype=np.int8)
    cells[10:20, 10:20] = 0
    svc.ingest("r0", meta, cells)

    p1 = svc.take_patch()
    assert p1 is not None
    assert p1["w"] == 10 and p1["h"] == 10  # bounding box of the change only
    raw = zlib.decompress(base64.b64decode(p1["data"]))
    assert np.frombuffer(raw, dtype=np.int8).size == 100

    assert svc.take_patch() is None  # nothing changed since
