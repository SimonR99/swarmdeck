"""Backend tests. None of these import ROS — acceptance criterion 12."""

from __future__ import annotations

import asyncio
import base64
import io
import math
import zlib

import numpy as np
import pytest
from fastapi.testclient import TestClient
from PIL import Image

from swarmdeck_server.api.app import (
    app,
    handle_gui_message,
    load_config,
    map_service,
    robot_state,
)
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.fleet.registry import registry as app_registry
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
        image = Image.open(io.BytesIO(r.content))
        assert image.getpixel((0, 0)) == (229, 232, 236)


def test_local_map_png_and_info_roundtrip():
    n = 40
    meta = GridMeta(0.1, n, n, -2.0, -2.0)
    cells = np.full((n, n), -1, np.int8)
    cells[10:20, 12:18] = 0
    cells[10:12, 12:18] = 100
    map_service.ingest("robot_0", meta, cells)

    with TestClient(app) as c:
        info = c.get("/api/map/local/robot_0/info")
        assert info.status_code == 200
        assert info.json()["width"] == n
        assert info.json()["origin"] == {"x": -2.0, "y": -2.0}

        image_response = c.get("/api/map/local/robot_0")
        assert image_response.status_code == 200
        image = Image.open(io.BytesIO(image_response.content))
        assert image.size == (n, n)

        assert c.get("/api/map/local/missing").status_code == 404


def test_camera_preview_roundtrip():
    frame = b"\xff\xd8preview\xff\xd9"
    with TestClient(app) as c:
        missing_id = c.post("/api/adapter/camera", content=frame, headers={"content-type": "image/jpeg"})
        assert missing_id.status_code == 400

        uploaded = c.post(
            "/api/adapter/camera?robot_id=r0",
            content=frame,
            headers={"content-type": "image/jpeg"},
        )
        assert uploaded.status_code == 200
        assert uploaded.json()["bytes"] == len(frame)

        preview = c.get("/api/camera/r0")
        assert preview.status_code == 200
        assert preview.headers["content-type"] == "image/jpeg"
        assert preview.headers["cache-control"].startswith("no-store")
        assert preview.content == frame


def test_registry_capabilities_and_neglect():
    reg = Registry()
    reg.hello(
        {"robot_id": "r0", "robot_type": "spot", "capabilities": ["navigate", "map"]}, sink=None
    )
    assert reg.can("r0", "navigate")
    assert not reg.can("r0", "camera")
    assert reg.robots["r0"].coordinate_frame == "local"
    assert reg.robots["r0"].unattended_s < 1.0

    reg.hello({"robot_id": "mock", "coordinate_frame": "merged"}, sink=None)
    assert reg.robots["mock"].coordinate_frame == "merged"


def test_duplicate_goal_rejected():
    reg = Registry()
    reg.hello({"robot_id": "r0", "capabilities": ["navigate"]}, sink=None)
    reg.hello({"robot_id": "r1", "capabilities": ["navigate"]}, sink=None)
    reg.update_state({"robot_id": "r0", "goal": {"x": 5.0, "y": 5.0}})
    assert reg.goal_taken({"x": 5.1, "y": 5.1}, exclude="r1") == "r0"
    assert reg.goal_taken({"x": 9.0, "y": 9.0}, exclude="r1") is None


def test_back_to_back_duplicate_goal_is_reserved_immediately():
    class Sink:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    first, second = Sink(), Sink()
    app_registry.robots.clear()
    app_registry._sinks.clear()
    try:
        app_registry.hello({"robot_id": "r0", "capabilities": ["navigate"]}, first)
        app_registry.hello({"robot_id": "r1", "capabilities": ["navigate"]}, second)
        goal = {"x": 2.0, "y": 3.0}
        asyncio.run(handle_gui_message({"type": "set_goal", "robot_id": "r0", "payload": goal}))
        asyncio.run(handle_gui_message({"type": "set_goal", "robot_id": "r1", "payload": goal}))

        assert len(first.messages) == 1
        assert first.messages[0]["type"] == "navigate_to"
        assert second.messages == []
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()


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


def test_robot_world_transform_roundtrip():
    svc = MapService()
    svc.set_transform("r0", 4.0, -2.0, math.pi / 2)

    local = {"x": 2.0, "y": 1.0, "yaw": -0.4}
    world = svc.robot_to_world("r0", local)
    assert world["x"] == pytest.approx(3.0)
    assert world["y"] == pytest.approx(0.0)
    assert world["yaw"] == pytest.approx(math.pi / 2 - 0.4)

    restored = svc.world_to_robot("r0", world)
    assert restored == pytest.approx(local)


def test_goal_routing_converts_shared_coordinates_to_robot_frame():
    class Sink:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    sink = Sink()
    app_registry.robots.clear()
    app_registry._sinks.clear()
    try:
        robot = app_registry.hello(
            {"robot_id": "r0", "capabilities": ["navigate"]}, sink=sink
        )
        map_service.set_transform("r0", 4.0, -2.0, math.pi / 2)

        asyncio.run(
            handle_gui_message(
                {"type": "set_goal", "robot_id": "r0", "payload": {"x": 3.0, "y": 0.0}}
            )
        )
        assert sink.messages[0]["type"] == "navigate_to"
        assert sink.messages[0]["goal"] == pytest.approx({"x": 2.0, "y": 1.0})

        robot.pose = {"x": 2.0, "y": 1.0, "yaw": 0.0}
        assert robot_state(robot)["pose"] == pytest.approx(
            {"x": 3.0, "y": 0.0, "yaw": math.pi / 2}
        )

        # Synthetic adapters can explicitly bypass conversion when their data
        # already uses the shared map frame.
        sink.messages.clear()
        robot.coordinate_frame = "merged"
        asyncio.run(
            handle_gui_message(
                {"type": "set_goal", "robot_id": "r0", "payload": {"x": 7.0, "y": 8.0}}
            )
        )
        assert sink.messages[0]["goal"] == {"x": 7.0, "y": 8.0}
        robot.pose = {"x": 7.0, "y": 8.0, "yaw": -0.2}
        assert robot_state(robot)["pose"] == robot.pose
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()


def test_drive_routing_is_bounded():
    class Sink:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    sink = Sink()
    app_registry.robots.clear()
    app_registry._sinks.clear()
    try:
        app_registry.hello({"robot_id": "r0", "capabilities": ["navigate"]}, sink=sink)
        asyncio.run(
            handle_gui_message(
                {
                    "type": "drive",
                    "robot_id": "r0",
                    "payload": {"linear": 99, "angular": -99},
                }
            )
        )
        assert sink.messages[0]["type"] == "drive"
        assert sink.messages[0]["linear"] == 0.45
        assert sink.messages[0]["angular"] == -1.2
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()
