"""Focused backend contracts that survived the navigation/map consolidation."""

from __future__ import annotations

import asyncio
import math
import time

import pytest
from fastapi.testclient import TestClient

from swarmdeck_server.api.app import (
    app,
    handle_adapter_message,
    handle_gui_message,
    load_config,
    map_service,
    review_store,
    robot_state,
    settings_store,
    state_loop_tick,
)
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.fleet.registry import registry as app_registry


@pytest.fixture(autouse=True)
def _cfg(monkeypatch, tmp_path):
    load_config()
    monkeypatch.setattr(settings_store, "path", tmp_path / "settings.json")
    monkeypatch.setattr(settings_store, "value", settings_store.validate({}))
    from swarmdeck_server.api import app as app_module

    monkeypatch.setattr(app_module, "REVIEW_PATH", tmp_path / "detections.json")
    review_store.reset()
    map_service.reset_robot()
    app_registry.robots.clear()
    app_registry._sinks.clear()
    app_module._gui_clients.clear()
    app_module._state_loop_cache.clear()
    app_module._alerts.clear()
    yield
    app_registry.robots.clear()
    app_registry._sinks.clear()
    map_service.reset_robot()
    review_store.reset()
    app_module._gui_clients.clear()
    app_module._state_loop_cache.clear()
    app_module._alerts.clear()


def test_registry_preserves_capabilities_frame_and_footprint():
    reg = Registry()
    footprint = [[0.5, 0.3], [0.5, -0.3], [-0.5, -0.3], [-0.5, 0.3]]
    robot = reg.hello(
        {
            "robot_id": "r0",
            "robot_type": "spot",
            "capabilities": ["navigate", "map"],
            "footprint_radius": 0.6,
            "footprint": footprint,
        },
        sink=None,
    )

    assert reg.can("r0", "navigate")
    assert not reg.can("r0", "camera")
    assert robot.coordinate_frame == "local"
    assert robot.footprint == footprint
    assert robot.to_state()["footprint"] == footprint


def test_robot_state_network_sample_is_stored_at_the_same_pose():
    app_registry.hello({"robot_id": "r0", "capabilities": ["network"]}, sink=None)
    asyncio.run(
        handle_adapter_message(
            {
                "type": "robot_state",
                "robot_id": "r0",
                "pose": {"x": 2.5, "y": -3.0, "yaw": 0.1},
                "network": {
                    "interface": "wlan0",
                    "quality_pct": 42.0,
                    "rssi_dbm": -69.0,
                },
            },
            None,
        )
    )

    state = app_registry.robots["r0"].to_state()
    assert state["network"]["rssi_dbm"] == -69.0
    snapshot = map_service.network_snapshot("r0")
    assert snapshot is not None
    assert snapshot["type"] == "network_patch"
    assert snapshot["robot_id"] == "r0"
    assert snapshot["w"] == snapshot["width"]
    assert snapshot["h"] == snapshot["height"]


def test_network_patch_skips_when_revision_has_not_changed():
    assert map_service.ingest_network_sample("r0", 0.0, 0.0, 50.0)
    first = map_service.take_network_patch("r0")
    assert first is not None
    assert map_service.take_network_patch("r0") is None
    assert map_service.ingest_network_sample("r0", 0.5, 0.0, 60.0)
    second = map_service.take_network_patch("r0")
    assert second is not None
    assert second["seq"] == first["seq"] + 1


def test_stop_all_reaches_every_registered_robot(monkeypatch):
    from swarmdeck_server.api import app as app_module

    registry = Registry()
    for robot_id in ("r0", "r1"):
        registry.hello({"robot_id": robot_id}, sink=None)
    sent = []

    async def send(robot_id, message):
        sent.append((robot_id, message))
        return True

    monkeypatch.setattr(app_module, "registry", registry)
    monkeypatch.setattr(registry, "send", send)
    asyncio.run(handle_gui_message({"type": "stop_all"}))

    assert {robot_id for robot_id, _ in sent} == {"r0", "r1"}
    assert all(message["type"] == "stop" for _, message in sent)


def test_robot_state_leaves_already_merged_pose_untouched():
    state = {"pose": {"x": 4.0, "y": -2.0, "z": 1.5, "yaw": 0.2}}

    class MergedRobot:
        coordinate_frame = "merged"
        robot_id = "r0"

        def to_state(self):
            return dict(state)

    assert robot_state(MergedRobot()) == state


def test_map_status_reports_deployment_transforms_and_roundtrips_pose():
    # The public optimized-map header remains SE(2), while the canonical
    # placement used by 3D composition retains the surveyed start height.
    map_service.set_transform("r0", 4.0, -2.0, math.pi / 2, z=1.5)

    with TestClient(app) as client:
        response = client.get("/api/map/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["transforms"]["r0"] == {
        "x": 4.0,
        "y": -2.0,
        "yaw": math.pi / 2,
    }
    assert "r0" in payload["members"]
    local = {"x": 2.0, "y": 1.0, "z": -0.25, "yaw": -0.4}
    world = map_service.robot_to_world("r0", local)
    assert world == pytest.approx(
        {"x": 3.0, "y": 0.0, "z": 1.25, "yaw": math.pi / 2 - 0.4}
    )
    assert map_service.world_to_robot("r0", world) == pytest.approx(local)


def test_state_loop_sends_changes_and_one_hz_keepalive(monkeypatch):
    from swarmdeck_server.api import app as app_module

    sent = []

    async def capture(message):
        sent.append(message)

    monkeypatch.setattr(app_module, "broadcast", capture)
    app_module._gui_clients.add(object())
    robot = app_registry.hello({"robot_id": "r0"}, sink=None)

    asyncio.run(state_loop_tick(now=10.0))
    asyncio.run(state_loop_tick(now=10.2))
    robot.pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    asyncio.run(state_loop_tick(now=10.4))
    asyncio.run(state_loop_tick(now=11.4))

    assert [message["pose"]["x"] for message in sent] == [0.0, 1.0, 1.0]


def test_state_loop_skips_robot_state_without_gui_clients(monkeypatch):
    from swarmdeck_server.api import app as app_module

    def should_not_build(_robot):
        raise AssertionError("robot_state should not be built without GUI clients")

    monkeypatch.setattr(app_module, "robot_state", should_not_build)
    app_registry.hello({"robot_id": "r0"}, sink=None)
    asyncio.run(state_loop_tick(now=10.0))


def test_state_loop_keeps_alerts_and_logs_without_gui_clients(monkeypatch):
    from swarmdeck_server.api import app as app_module
    from swarmdeck_server.fleet.registry import OFFLINE_AFTER_S

    logged = []
    monkeypatch.setattr(app_module.events, "log", lambda kind, payload: logged.append((kind, payload)))
    settings_store.value["unattended_threshold_s"] = 1.0
    unattended = app_registry.hello({"robot_id": "r0"}, sink=None)
    unattended.last_attended = time.monotonic() - 5.0
    disconnected = app_registry.hello({"robot_id": "r1"}, sink=None)
    disconnected.last_seen = time.monotonic() - OFFLINE_AFTER_S - 1.0

    asyncio.run(state_loop_tick(now=10.0))

    assert app_module._alerts["unattended_r0"]["kind"] == "unattended"
    assert app_module._alerts["disconnect_r1"]["kind"] == "adapter_disconnect"
    logged_kinds = [payload["alert"]["kind"] for kind, payload in logged if kind == "alert"]
    assert "unattended" in logged_kinds
    assert "adapter_disconnect" in logged_kinds
