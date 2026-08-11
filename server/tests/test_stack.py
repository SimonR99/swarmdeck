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
    detection_position,
    handle_gui_message,
    load_config,
    map_service,
    robot_state,
)
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.fleet.registry import registry as app_registry
from swarmdeck_server.mapsvc.service import GridMeta, MapService
from swarmdeck_server.mapsvc.scan_grid import ScanGridAccumulator


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
        # A protocol-1 adapter must stay valid: v2 only adds an optional message.
        assert r.json()["protocol"] == 2
        assert 1 in r.json()["supported_protocols"]


def test_detection_classes_endpoint_describes_the_catalog():
    """The dashboard's class toggles are built from this, not from a UI copy."""
    with TestClient(app) as c:
        r = c.get("/api/detection/classes")
        assert r.status_code == 200
        classes = r.json()["classes"]

    assert {"name": "disc_cone", "label": "Disc cone"} in classes
    assert all({"name", "label"} == set(item) for item in classes)


def test_map_png_roundtrip():
    with TestClient(app) as c:
        r = c.get("/api/map")
        assert r.status_code == 200
        assert r.headers["content-type"] == "image/png"
        image = Image.open(io.BytesIO(r.content))
        assert image.getpixel((0, 0)) == MapService.UNKNOWN_RGB


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


def test_targeted_map_reset_discards_only_one_robots_accumulated_products():
    svc = MapService(resolution=0.1, size_m=10.0)
    meta = GridMeta(0.1, 20, 20, -1.0, -1.0)
    r0_cells = np.full((20, 20), -1, dtype=np.int8)
    r0_cells[8:12, 8:12] = 100
    r1_cells = np.full((20, 20), -1, dtype=np.int8)
    r1_cells[4:16, 4:16] = 0

    svc.set_transform("r0", 0.0, 0.0, 0.0)
    svc.set_transform("r1", 2.0, 0.0, 0.0)
    svc.ingest("r0", meta, r0_cells)
    svc.ingest("r1", meta, r1_cells)
    svc.set_cloud("r0", np.array([[0.0, 0.0, 0.5]], dtype=np.float32))
    svc.set_cloud("r1", np.array([[1.0, 0.0, 0.5]], dtype=np.float32))
    svc._scan_grids["r0"] = ScanGridAccumulator(0.0, 0.0, resolution=0.1, size_m=2.0)
    svc.set_slam_graph("r0", {"in_common_frame": False})

    assert svc.reset_robot("r0") == ["r0"]

    assert "r0" not in svc.robot_grids
    assert "r0" not in svc.robot_clouds
    assert "r0" not in svc._scan_grids
    assert "r0" not in svc.slam_graphs
    assert "r1" in svc.robot_grids
    assert "r1" in svc.robot_clouds
    assert svc.transforms["r0"] == (0.0, 0.0, 0.0)
    assert np.any(svc.merged == 0)


def test_map_reset_api_refuses_navigation_then_resets_target_only():
    app_registry.robots.clear()
    app_registry._sinks.clear()
    meta = GridMeta(0.1, 10, 10, -0.5, -0.5)
    cells = np.zeros((10, 10), dtype=np.int8)
    map_service.robot_grids["botman"] = (meta, cells)
    map_service.robot_grids["tars"] = (meta, cells.copy())
    try:
        robot = app_registry.hello(
            {"robot_id": "botman", "capabilities": ["navigate", "map"]},
            sink=None,
        )
        robot.nav_status = "active"
        robot.goal = {"x": 1.0, "y": 0.0}

        with TestClient(app) as c:
            blocked = c.post("/api/map/reset/botman")
            assert blocked.status_code == 409
            assert blocked.json()["robots"] == ["botman"]
            assert "botman" in map_service.robot_grids

            robot.nav_status = "idle"
            robot.goal = None
            reset = c.post("/api/map/reset/botman")
            assert reset.status_code == 200
            assert reset.json()["robots"] == ["botman"]
            assert "botman" not in map_service.robot_grids
            assert "tars" in map_service.robot_grids
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()


def test_reset_all_maps_api_clears_every_robot_product():
    app_registry.robots.clear()
    app_registry._sinks.clear()
    meta = GridMeta(0.1, 10, 10, -0.5, -0.5)
    cells = np.zeros((10, 10), dtype=np.int8)
    map_service.robot_grids["botman"] = (meta, cells)
    map_service.robot_grids["tars"] = (meta, cells.copy())
    map_service.robot_clouds["botman"] = np.zeros((1, 3), dtype=np.float32)

    try:
        with TestClient(app) as c:
            reset = c.post("/api/map/reset")

        assert reset.status_code == 200
        assert reset.json()["robots"] == ["botman", "tars"]
        assert map_service.robot_grids == {}
        assert map_service.robot_clouds == {}
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()


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


def test_detection_position_is_normalized_into_the_merged_map():
    app_registry.robots.clear()
    try:
        app_registry.hello({"robot_id": "r0", "coordinate_frame": "local"}, sink=None)
        map_service.set_transform("r0", 10.0, -2.0, math.pi / 2)

        position = detection_position("r0", {"x": 2.0, "y": 1.0})

        assert position == pytest.approx({"x": 9.0, "y": 0.0})
        assert detection_position("r0", None) is None
        assert detection_position("r0", {"x": float("nan"), "y": 1.0}) is None
    finally:
        app_registry.robots.clear()


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


def test_ingest_async_does_not_block_the_event_loop():
    """Registration is hundreds of ms of numpy; inline it stalls every socket."""
    import time

    svc = MapService(resolution=0.1, size_m=10.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[10:20, 10:20] = 0

    def slow_ingest(*_args, **_kwargs):
        time.sleep(0.30)

    svc.ingest = slow_ingest  # type: ignore[method-assign]

    async def scenario() -> int:
        ticks = 0

        async def heartbeat() -> None:
            nonlocal ticks
            while True:
                await asyncio.sleep(0.01)
                ticks += 1

        beat = asyncio.create_task(heartbeat())
        await svc.ingest_async("r0", meta, cells)
        beat.cancel()
        return ticks

    # A blocked loop would let through nearly none of these.
    assert asyncio.run(scenario()) > 10


def test_auto_mode_locks_on_and_stops_searching_all_rotations():
    """Once registered, a robot's transform is refined, not re-derived.

    A full 360 deg sweep on every upload is both wasted work and a fresh chance
    to jump onto a symmetric alias.
    """
    import swarmdeck_server.mapsvc.service as service_module
    from swarmdeck_server.mapsvc.registration import Registration

    calls: list[dict] = []

    def fake_register(*_args, **kwargs):
        calls.append(kwargs)
        return Registration(
            dx=0.0, dy=0.0, dyaw=0.2, score=0.9, overlap=500,
            ratio=0.1, yaw_ratio=0.2, support=0.9,
        )

    original = service_module.register
    service_module.register = fake_register
    try:
        svc = MapService(resolution=0.1, size_m=10.0)
        svc.set_mode("auto")
        n = svc.meta.width
        meta = GridMeta(0.1, n, n, -5.0, -5.0)
        cells = np.full((n, n), -1, dtype=np.int8)
        cells[10:30, 10:30] = 0
        cells[10:12, 10:30] = 100

        svc.ingest("ref", meta, cells)
        svc.ingest("mov", meta, cells)
        svc.ingest("mov", meta, cells)
    finally:
        service_module.register = original

    assert len(calls) == 2
    assert calls[0]["yaw_prior"] is None, "no prior configured, so search widely"
    assert calls[1]["yaw_prior"] == pytest.approx(0.2), "should refine the accepted yaw"
    assert calls[1]["yaw_window_deg"] < calls[0]["yaw_window_deg"]


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


# --------------------------------------------------------------- cslam mode


def _plan_grid(n: int) -> np.ndarray:
    """A small asymmetric floor plan, so registration has something to lock on."""
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[5:n - 5, 5:n - 5] = 0
    cells[5, 5:n - 5] = 100
    cells[n - 6, 5:n - 5] = 100
    cells[5:n - 5, 5] = 100
    cells[5:n - 5, n - 6] = 100
    cells[5:n // 2, n // 3] = 100          # an interior wall, off centre
    cells[n // 2 + 4, n // 3:n - 8] = 100  # and a second, so the plan is not symmetric
    return cells


def _cslam_service() -> tuple[MapService, GridMeta, np.ndarray]:
    svc = MapService(resolution=0.1, size_m=6.0)
    svc.set_mode("cslam")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -3.0, -3.0)
    # Robots running a collaborative back end publish in the common frame, so
    # their transforms are identity — mapsvc is bookkeeping, not an estimator.
    svc.set_transform("r0", 0.0, 0.0, 0.0)
    svc.set_transform("r1", 0.0, 0.0, 0.0)
    return svc, meta, _plan_grid(n)


def test_cslam_mode_is_accepted():
    svc = MapService()
    svc.set_mode("cslam")
    assert svc.merge_mode == "cslam"
    svc.set_mode("nonsense")
    assert svc.merge_mode == "static"


def test_cslam_excludes_a_robot_that_has_not_joined_the_common_frame():
    """Absent an inter-robot loop closure, a robot is not in the shared map.
    Overlaying it at its configured start pose is the confident-but-wrong merge
    the rejection tests exist to prevent."""
    svc, meta, cells = _cslam_service()
    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells)
    assert svc.global_members() == {"r0"}          # r0 is the reference
    assert svc.status()["view_by_robot"]["r1"] == "local"


def test_cslam_admits_a_robot_once_its_graph_says_so():
    svc, meta, cells = _cslam_service()
    svc.ingest("r0", meta, cells)
    svc.set_slam_graph("r1", {"keyframes": 40, "in_common_frame": True,
                              "residual": 0.02, "inter_robot": [{"other": "r0", "count": 3}]})
    svc.ingest("r1", meta, cells)
    assert svc.global_members() == {"r0", "r1"}


def test_cslam_reports_disagreement_rather_than_correcting_it():
    """Grid correlation stops being the estimator and becomes an independent
    check, using evidence (free space over the whole map) the loop closures did
    not use. Two identical grids in one frame must agree at ~zero, and the
    transform must be left alone either way."""
    svc, meta, cells = _cslam_service()
    svc.ingest("r0", meta, cells)
    svc.set_slam_graph("r1", {"keyframes": 40, "in_common_frame": True})
    svc.ingest("r1", meta, cells)

    assert svc.transforms["r1"] == (0.0, 0.0, 0.0), "the check must not move anything"
    disagreement = svc.status()["cslam_disagreement"]
    assert "r1" in disagreement, "identical grids should be checkable"
    assert disagreement["r1"]["metres"] < 0.2
    assert abs(disagreement["r1"]["degrees"]) < 5.0


def test_cslam_disagreement_is_visible_when_the_alignment_is_wrong():
    """A robot whose grid is offset from where the pose graph puts it must show
    up as a number, not as a silently smeared map."""
    svc, meta, cells = _cslam_service()
    svc.ingest("r0", meta, cells)
    svc.set_slam_graph("r1", {"keyframes": 40, "in_common_frame": True})
    shifted = np.full_like(cells, -1)
    shifted[:, 6:] = cells[:, :-6]          # 6 cells = 0.6 m of error
    svc.ingest("r1", meta, shifted)

    disagreement = svc.status()["cslam_disagreement"]
    assert "r1" in disagreement
    assert disagreement["r1"]["metres"] > 0.3


def test_slam_graph_survives_into_status_in_every_mode():
    svc = MapService()
    svc.set_slam_graph("r0", {"keyframes": 12, "in_common_frame": True,
                              "inter_robot": [{"other": "r1", "count": 2}]})
    assert svc.status()["slam_graphs"]["r0"]["keyframes"] == 12


# ------------------------------------------------------------- 3D cloud


def test_cloud_upload_and_merge_roundtrip():
    """A cloud uploaded in a robot's own frame comes back in the merged one."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("static")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[10:20, 10:20] = 0

    # r1 sits 2 m along x, rotated a quarter turn.
    svc.set_transform("r0", 0.0, 0.0, 0.0)
    svc.set_transform("r1", 2.0, 0.0, math.pi / 2)
    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells)
    svc.set_cloud("r0", np.array([[1.0, 0.0, 0.5]], dtype=np.float32))
    svc.set_cloud("r1", np.array([[1.0, 0.0, 0.5]], dtype=np.float32))

    points, indices, names = svc.merged_cloud()
    assert names == ["r0", "r1"]
    assert len(points) == 2
    assert points[0] == pytest.approx([1.0, 0.0, 0.5], abs=1e-5)
    # (1, 0) rotated 90 deg then offset by (2, 0) lands at (2, 1). z is untouched:
    # the merge frame is SE(2), so height passes straight through.
    assert points[1] == pytest.approx([2.0, 1.0, 0.5], abs=1e-5)
    assert list(indices) == [0, 1]


def test_merged_cloud_excludes_unregistered_robots():
    """Drawing an unregistered robot's cloud in the shared frame would render a
    guess as though it were a measurement — the same rule the 2D merge uses."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("cslam")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[10:20, 10:20] = 0
    svc.set_transform("r0", 0.0, 0.0, 0.0)
    svc.set_transform("r1", 0.0, 0.0, 0.0)
    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells)
    svc.set_cloud("r0", np.ones((5, 3), dtype=np.float32))
    svc.set_cloud("r1", np.ones((5, 3), dtype=np.float32))

    # r1 has not joined the common frame, so only the reference contributes.
    _, _, names = svc.merged_cloud()
    assert names == ["r0"]


def test_cloud_endpoints_roundtrip():
    import zlib

    points = np.array([[1.0, 2.0, 0.5], [-1.0, 0.0, 1.25]], dtype=np.float32)
    body = zlib.compress(np.round(points / 0.01).astype(np.int16).tobytes())
    # `static` is the operator asserting the transforms are valid, which is what
    # makes robot_0 a member. In `auto` a lone robot has nothing to register
    # against and is deliberately excluded, so the merged cloud would be empty —
    # correct behaviour, but it would be testing membership rather than the
    # endpoints these assertions are about.
    map_service.set_mode("static")
    n = 40
    map_service.ingest("robot_0", GridMeta(0.1, n, n, -2.0, -2.0),
                       np.full((n, n), -1, np.int8))
    with TestClient(app) as c:
        assert c.post("/api/adapter/cloud", content=body).status_code == 400
        posted = c.post("/api/adapter/cloud?robot_id=robot_0", content=body)
        assert posted.status_code == 200
        assert posted.json()["points"] == 2

        got = c.get("/api/map/cloud")
        assert got.status_code == 200
        assert int(got.headers["X-Cloud-Points"]) >= 2
        assert "robot_0" in got.headers["X-Cloud-Robots"]


def test_malformed_cloud_is_refused_not_crashed():
    with TestClient(app) as c:
        assert c.post("/api/adapter/cloud?robot_id=r0", content=b"not zlib").status_code == 400
        import zlib
        odd = zlib.compress(np.zeros(4, dtype=np.int16).tobytes())  # not a triple
        assert c.post("/api/adapter/cloud?robot_id=r0", content=odd).status_code == 400


def test_scan_endpoint_builds_a_local_map_for_a_robot_with_no_native_grid():
    """The whole point of /api/adapter/scan: a robot that never uploads
    /api/adapter/map still ends up with a local grid the GUI can show."""
    points = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=np.float32)
    body = zlib.compress(np.round(points / 0.01).astype(np.int16).tobytes())
    with TestClient(app) as c:
        assert c.post("/api/adapter/scan?robot_id=r0", content=body).status_code == 400
        posted = c.post(
            "/api/adapter/scan?robot_id=r0&origin_x=0&origin_y=0", content=body
        )
        assert posted.status_code == 200
        assert posted.json()["points"] == 2

        info = c.get("/api/map/local/r0/info")
        assert info.status_code == 200
        png = c.get("/api/map/local/r0")
        assert png.status_code == 200
        assert png.headers["content-type"] == "image/png"


def test_malformed_scan_is_refused_not_crashed():
    with TestClient(app) as c:
        assert (
            c.post("/api/adapter/scan?robot_id=r0&origin_x=0&origin_y=0",
                   content=b"not zlib").status_code == 400
        )
        odd = zlib.compress(np.zeros(3, dtype=np.int16).tobytes())  # not a pair
        assert (
            c.post("/api/adapter/scan?robot_id=r0&origin_x=0&origin_y=0",
                   content=odd).status_code == 400
        )


# ------------------------------------------------- cslam drives the merge


def _grid(svc, rid, transform=(0.0, 0.0, 0.0)):
    n = svc.meta.width
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[10:30, 10:30] = 0
    svc.set_transform(rid, *transform)
    svc.ingest(rid, GridMeta(svc.meta.resolution, n, n,
                             svc.meta.origin_x, svc.meta.origin_y), cells)


def test_cslam_origin_replaces_the_registration_transform():
    """In cslam mode the transform comes from the pose graph, not correlation."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("cslam")
    _grid(svc, "r0")
    _grid(svc, "r1")
    svc.set_slam_graph("r1", {"in_common_frame": True, "inter_robot": [{"other": "r0"}]})
    svc.set_cslam_origin("r1", 2.0, -1.0, math.pi / 2, "robot0_map")
    assert svc.transforms["r1"] == pytest.approx((2.0, -1.0, math.pi / 2))


def test_cslam_origin_is_ignored_outside_cslam_mode():
    """`auto` asked for correlation; silently overriding it would be a trap."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("auto")
    _grid(svc, "r0", (1.0, 1.0, 0.0))
    svc.set_cslam_origin("r0", 9.0, 9.0, 1.0, "robot0_map")
    assert svc.transforms["r0"] == pytest.approx((1.0, 1.0, 0.0))


def test_disjoint_cslam_clusters_are_not_overlaid():
    """Two groups that never met have two unrelated common frames.

    Drawing them together would place robots confidently in the wrong building,
    which is worse than showing fewer robots.
    """
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("cslam")
    for rid in ("r0", "r1", "r2"):
        _grid(svc, rid)
        svc.set_slam_graph(rid, {"in_common_frame": True,
                                 "inter_robot": [{"other": "other"}]})
    # r0 and r1 met each other; r2 is its own island.
    svc.set_cslam_origin("r0", 0.0, 0.0, 0.0, "robot0_map")
    svc.set_cslam_origin("r1", 1.0, 0.0, 0.0, "robot0_map")
    svc.set_cslam_origin("r2", 5.0, 5.0, 0.0, "robot2_map")

    members = svc.global_members()
    assert {"r0", "r1"} <= members
    assert "r2" not in members


def test_cslam_membership_still_requires_a_loop_closure():
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("cslam")
    _grid(svc, "r0")
    _grid(svc, "r1")
    svc.set_slam_graph("r0", {"in_common_frame": True, "inter_robot": [{"other": "r1"}]})
    svc.set_slam_graph("r1", {"in_common_frame": False, "inter_robot": []})
    svc.set_cslam_origin("r0", 0.0, 0.0, 0.0, "robot0_map")
    svc.set_cslam_origin("r1", 3.0, 0.0, 0.0, "robot0_map")
    assert "r1" not in svc.global_members()


# ------------------------------------------------- 3D band registration


def _symmetric_building(with_asymmetric_furniture: bool):
    """A plan-symmetric corridor: identical at 0 and 180 deg from above.

    Optionally add furniture in ONE low band only, which breaks the symmetry in
    3D while leaving the 2D projection just as ambiguous as before.
    """
    pts = []
    for x in np.arange(-6.0, 6.0, 0.05):
        for z in (0.3, 0.8, 1.5):
            pts.append((x, -2.0, z))
            pts.append((x, 2.0, z))
    for y in np.arange(-2.0, 2.0, 0.05):
        for z in (0.3, 0.8, 1.5):
            pts.append((-6.0, y, z))
            pts.append((6.0, y, z))
    if with_asymmetric_furniture:
        for x in np.arange(-5.0, -3.0, 0.05):
            for y in np.arange(-1.0, 1.0, 0.05):
                pts.append((x, y, 0.35))
    return np.array(pts, dtype=np.float32)


def test_height_bands_break_rotational_aliasing():
    """The failure this exists for: a plan-symmetric building.

    Flattened to 2D the truth and a 180 deg alias score nearly the same. Vertical
    structure that is NOT symmetric resolves it, and the bands agreeing is what
    reports that resolution.
    """
    from swarmdeck_server.mapsvc.registration import register_3d

    pts = _symmetric_building(with_asymmetric_furniture=True)
    meta = (400, 400, -10.0, -10.0)
    result = register_3d(pts, pts, 0.05, meta)
    # Registering a cloud against itself must find the identity and say so.
    assert abs(result.dyaw) < math.radians(3.0)
    assert result.yaw_ratio < 0.8, "identity match should not look ambiguous"


def test_height_band_grid_selects_only_its_band():
    from swarmdeck_server.mapsvc.registration import height_band_grid

    pts = np.array([[0.0, 0.0, 0.3], [0.5, 0.5, 1.5]], dtype=np.float32)
    meta = (40, 40, -1.0, -1.0)
    low = height_band_grid(pts, (0.10, 0.55), 0.05, meta)
    high = height_band_grid(pts, (1.10, 1.80), 0.05, meta)
    assert int((low == 100).sum()) == 1
    assert int((high == 100).sum()) == 1
    # And the bands must not be the same cell — they are different structure.
    assert not np.array_equal(low, high)


def test_register_3d_degrades_gracefully_without_points():
    from swarmdeck_server.mapsvc.registration import register_3d

    empty = np.zeros((0, 3), dtype=np.float32)
    result = register_3d(empty, empty, 0.05, (40, 40, -1.0, -1.0))
    assert not result.confident


# --------------------------------------------------------------------- reset


def test_map_reset_drops_maps_but_keeps_the_operator_s_configuration():
    """A reset restarts the run; it does not reconfigure it."""
    # `static` rather than `auto`: in auto mode a lone reference robot has no
    # accepted registration, so global_members() is empty and nothing merges —
    # there would be no map for the reset to clear.
    svc = MapService(resolution=0.5, size_m=10.0)
    svc.set_mode("static")
    svc.set_transform("r0", 1.0, 2.0, 0.0)
    svc.set_transform("r1", -1.0, 0.0, 0.5)
    meta = GridMeta(0.5, 20, 20, -5.0, -5.0)
    cells = np.zeros((20, 20), dtype=np.int8)
    cells[5:9, 5:9] = 100
    svc.ingest("r0", meta, cells)
    svc.robot_clouds["r0"] = np.zeros((4, 3), dtype=np.float32)
    svc.slam_graphs["r0"] = {"keyframes": 12}
    assert (svc.merged != -1).any(), "precondition: something was mapped"

    svc.reset()

    assert (svc.merged == -1).all(), "every cell back to unknown"
    assert svc.robot_grids == {}
    assert svc.robot_clouds == {}
    assert svc.slam_graphs == {}
    assert svc.registrations == {}
    # Configuration survives: resolution, extent and the start poses the operator
    # set. Dropping the transforms would move every robot to the origin.
    assert svc.meta.resolution == 0.5
    assert svc.transforms == {"r0": (1.0, 2.0, 0.0), "r1": (-1.0, 0.0, 0.5)}

    # And the merge mode, checked against a non-default value so the assertion
    # cannot pass by accident.
    other = MapService(resolution=0.05, size_m=30.0)
    other.set_mode("cslam")
    other.reset()
    assert other.merge_mode == "cslam"


def test_map_reset_emits_a_patch_that_clears_the_browser():
    """The GUI must clear through the same path it draws through."""
    svc = MapService(resolution=0.5, size_m=10.0)
    meta = GridMeta(0.5, 20, 20, -5.0, -5.0)
    cells = np.zeros((20, 20), dtype=np.int8)
    cells[5:9, 5:9] = 100
    svc.ingest("r0", meta, cells)
    svc.take_patch()  # browser is now up to date with the mapped grid

    svc.reset()
    patch = svc.take_patch()

    assert patch is not None, "a reset that emits no patch leaves a stale map on screen"
    decoded = np.frombuffer(
        zlib.decompress(base64.b64decode(patch["data"])), dtype=np.int8
    )
    assert (decoded == -1).all()


def _reset_sink(app_module):
    class Sink:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    return Sink()


def test_reset_waits_for_adapters_before_clearing_the_map():
    """The ordering that stops a cleared map coming straight back.

    The backend holds each robot's last uploaded grid. If it cleared them when it
    SENT `reset`, an upload already in flight would restore the old map a moment
    later. So it clears on `reset_done`, and this pins that: while the adapter is
    still working, the map is untouched.
    """
    from swarmdeck_server.api import app as app_module

    sink = _reset_sink(app_module)
    app_registry.robots.clear()
    app_registry._sinks.clear()
    try:
        app_registry.hello(
            {"robot_id": "r0", "capabilities": ["map", "reset"]}, sink=sink
        )
        meta = GridMeta(0.05, 40, 40, -1.0, -1.0)
        cells = np.full((40, 40), 100, dtype=np.int8)
        map_service.ingest("r0", meta, cells)
        assert "r0" in map_service.robot_grids

        async def scenario():
            task = asyncio.create_task(app_module.reset_fleet())
            # Let reset_fleet dispatch the command and start waiting.
            for _ in range(20):
                await asyncio.sleep(0)
                if app_module._reset_pending:
                    break
            assert app_module._reset_pending == {"r0"}
            assert any(m["type"] == "reset" for m in sink.messages)
            assert "r0" in map_service.robot_grids, (
                "map cleared before the adapter confirmed — this is the race"
            )

            # Now the adapter reports in, exactly as adapter_sim does.
            app_module._reset_pending.discard("r0")
            app_module._reset_done.set()
            return await task

        result = asyncio.run(scenario())

        assert result["ok"] is True
        assert result["reset"] == ["r0"]
        assert result["failed"] == []
        assert map_service.robot_grids == {}
        assert (map_service.merged == -1).all()
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()
        app_module._reset_pending.clear()
        map_service.reset()


def test_reset_is_never_sent_to_a_robot_without_the_capability():
    """The safety gate. `reset` on real hardware is not a thing that can happen."""
    from swarmdeck_server.api import app as app_module

    sim = _reset_sink(app_module)
    hardware = _reset_sink(app_module)
    app_registry.robots.clear()
    app_registry._sinks.clear()
    try:
        app_registry.hello(
            {"robot_id": "sim_0", "capabilities": ["map", "reset"]}, sink=sim
        )
        # Exactly what adapter_ros2 advertises: no `reset`, ever.
        app_registry.hello(
            {"robot_id": "duckie_0", "capabilities": ["navigate", "map", "battery"]},
            sink=hardware,
        )

        async def scenario():
            task = asyncio.create_task(app_module.reset_fleet())
            for _ in range(20):
                await asyncio.sleep(0)
                if app_module._reset_pending:
                    break
            app_module._reset_pending.clear()
            app_module._reset_done.set()
            return await task

        result = asyncio.run(scenario())

        assert [m["type"] for m in hardware.messages] == [], (
            "a hardware adapter was sent a reset"
        )
        assert any(m["type"] == "reset" for m in sim.messages)
        assert result["skipped"] == ["duckie_0"]
    finally:
        app_registry.robots.clear()
        app_registry._sinks.clear()
        app_module._reset_pending.clear()
        map_service.reset()


def test_reset_clears_and_reports_when_an_adapter_never_answers():
    """A stuck adapter must not leave a spinner up forever — but must be named."""
    from swarmdeck_server.api import app as app_module

    sink = _reset_sink(app_module)
    app_registry.robots.clear()
    app_registry._sinks.clear()
    original_timeout = app_module.RESET_TIMEOUT_S
    app_module.RESET_TIMEOUT_S = 0.05
    try:
        app_registry.hello(
            {"robot_id": "r0", "capabilities": ["map", "reset"]}, sink=sink
        )
        meta = GridMeta(0.05, 40, 40, -1.0, -1.0)
        map_service.ingest("r0", meta, np.full((40, 40), 100, dtype=np.int8))

        result = asyncio.run(app_module.reset_fleet())

        assert result["timed_out"] is True
        assert result["ok"] is False
        assert result["no_response"] == ["r0"]
        assert result["failed"] == ["r0"]
        # Cleared anyway: a map nobody can clear is worse than one that returns.
        assert map_service.robot_grids == {}
    finally:
        app_module.RESET_TIMEOUT_S = original_timeout
        app_registry.robots.clear()
        app_registry._sinks.clear()
        app_module._reset_pending.clear()
        map_service.reset()


# ------------------------------------------------------- merge conflict voting


def _three_robot_service(conflict: str) -> MapService:
    svc = MapService(resolution=0.5, size_m=10.0)
    svc.set_mode("static")
    svc.set_conflict_mode(conflict)
    for rid in ("r0", "r1", "r2"):
        svc.set_transform(rid, 0.0, 0.0, 0.0)
    return svc


def _uniform_grid(fill: int) -> tuple[GridMeta, np.ndarray]:
    meta = GridMeta(0.5, 20, 20, -5.0, -5.0)
    return meta, np.full((20, 20), fill, dtype=np.int8)


def test_a_ghost_is_erased_once_two_robots_have_driven_through_it():
    """The artefact this exists to remove.

    One robot recorded another robot as a wall and drove on. Everyone else has
    since driven through that spot and reports free space. Under the old
    `maximum` rule the single stale cell outvoted all of them forever, because
    100 > 0.
    """
    svc = _three_robot_service("majority")
    meta, ghost = _uniform_grid(0)
    ghost[10, 10] = 100  # r0 saw something here, once
    svc.ingest("r0", meta, ghost)
    svc.ingest("r1", *_uniform_grid(0))
    svc.ingest("r2", *_uniform_grid(0))

    assert svc.merged[10, 10] == 0, "ghost survived two contradicting observations"


def test_an_obstacle_only_one_robot_has_seen_is_kept():
    """The other half: unknown must abstain, not vote for free.

    r1 and r2 have never observed this cell at all. If absence counted as a
    vote for free, a real wall seen by one robot would be erased by two robots
    that never looked at it.
    """
    svc = _three_robot_service("majority")
    meta, seen = _uniform_grid(0)
    seen[10, 10] = 100
    svc.ingest("r0", meta, seen)
    meta_u, unknown = _uniform_grid(-1)
    svc.ingest("r1", meta_u, unknown)
    svc.ingest("r2", meta_u, unknown)

    assert svc.merged[10, 10] == 100


def test_a_tie_stays_occupied():
    """With two robots disagreeing there is no majority, and reporting a space
    clear when a robot says otherwise is the worse error."""
    svc = _three_robot_service("majority")
    meta, seen = _uniform_grid(0)
    seen[10, 10] = 100
    svc.ingest("r0", meta, seen)
    svc.ingest("r1", *_uniform_grid(0))

    assert svc.merged[10, 10] == 100


def test_occupied_mode_restores_the_old_any_vote_wins_rule():
    svc = _three_robot_service("occupied")
    meta, ghost = _uniform_grid(0)
    ghost[10, 10] = 100
    svc.ingest("r0", meta, ghost)
    svc.ingest("r1", *_uniform_grid(0))
    svc.ingest("r2", *_uniform_grid(0))

    assert svc.merged[10, 10] == 100


def test_unobserved_cells_stay_unknown_in_both_modes():
    for mode in ("majority", "occupied"):
        svc = _three_robot_service(mode)
        meta, unknown = _uniform_grid(-1)
        svc.ingest("r0", meta, unknown)
        assert (svc.merged == -1).all(), mode


def test_conflict_mode_comes_from_config_and_is_reported():
    svc = MapService(resolution=0.5, size_m=10.0)
    svc.set_conflict_mode("occupied")
    assert svc.status()["merge_conflict"] == "occupied"
    # An unknown value must not silently disable voting.
    svc.set_conflict_mode("consensus-please")
    assert svc.merge_conflict == "majority"
