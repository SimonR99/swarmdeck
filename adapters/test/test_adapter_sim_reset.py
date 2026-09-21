"""Simulation reset, quiescence, and navigation-generation contracts."""

from __future__ import annotations

import math
import threading
import time
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def sim(sim_module):
    sim_module.SLAM_GRAPHS.clear()
    return sim_module


def make_bridge(sim):
    bridge = sim.RobotBridge.__new__(sim.RobotBridge)
    bridge.node = MagicMock()
    bridge.id = "robot_0"
    bridge.t0 = 0.0
    bridge.pub_cmd = MagicMock()
    bridge._goal_handle = None
    bridge._goal_generation = 0
    bridge._goal_lock = threading.RLock()
    bridge._goal_request_future = None
    bridge._goal_request_generation = None
    bridge._cancel_events = {}
    bridge._nav_quiet_unknown = False
    bridge._last_drive_at = 12.0
    bridge._upload_lock = threading.Lock()
    bridge._costmap_lock = threading.Lock()
    bridge._costmaps = {}
    bridge._costmap_dirty = set()
    bridge._service_clients = {}
    bridge._reset_report = None
    bridge.http_url = "http://server:8080"
    bridge.platform = "bunker"
    bridge.robot_type = "agilex_bunker"
    bridge.footprint_radius = 0.643
    bridge.goal = {"x": 3.0, "y": 1.0}
    bridge.planned_path = [{"x": 1.0, "y": 1.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge._camera_frame = object()
    bridge._camera_dirty = True
    bridge._camera_depth = object()
    bridge._detections = [{"id": "duck_0"}]
    bridge._escape_from = None
    bridge._escape_started_at = 0.0
    bridge._escape_progress_at = 0.0
    return bridge


def test_reset_refuses_and_reports_supervisor_required(sim):
    """A per-robot reset never moves the world; it is refused and reported."""
    bridge = make_bridge(sim)
    assert bridge.take_reset_report() is None

    steps = bridge.reset()

    assert steps == {"supervisor_required": False}
    bridge.pub_cmd.publish.assert_not_called()
    report = bridge.take_reset_report()
    assert report["type"] == "reset_done"
    assert report["robot_id"] == "robot_0"
    assert report["ok"] is False
    assert report["steps"] == {"supervisor_required": False}
    assert bridge.take_reset_report() is None


def test_reset_readiness_requires_follow_path_and_planner_service(sim):
    bridge = make_bridge(sim)
    bridge.path_client = MagicMock()
    bridge.objective_planner = MagicMock()
    bridge.path_client.server_is_ready.return_value = True
    bridge.objective_planner.client.service_is_ready.return_value = False

    assert bridge.navigation_ready() is False
    bridge.objective_planner.client.service_is_ready.return_value = True
    assert bridge.navigation_ready() is True


def test_costmap_upload_is_skipped_while_reset_holds_quiescence_lock(sim):
    bridge = make_bridge(sim)
    bridge._costmap_lock = threading.Lock()
    bridge._upload_lock.acquire()
    try:
        bridge.upload_costmaps()
    finally:
        bridge._upload_lock.release()

    assert bridge._costmaps == {}


class _Twist:
    def __init__(self):
        self.linear = type("Vec", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.angular = type("Vec", (), {"x": 0.0, "y": 0.0, "z": 0.0})()


def test_a_failed_goal_arms_the_escape(sim):
    bridge = make_bridge(sim)
    bridge.map_pose = lambda: {"x": 1.0, "y": 2.0, "yaw": 0.0}

    bridge._finish_goal("failed", bridge._goal_generation)

    assert bridge._escape_from == (1.0, 2.0)
    assert bridge.mode == "recover"
    assert bridge.nav_status == "failed"


@pytest.mark.parametrize("status", ["succeeded", "cancelled"])
def test_only_failed_goal_arms_the_escape(sim, status):
    bridge = make_bridge(sim)
    bridge.map_pose = lambda: {"x": 1.0, "y": 2.0, "yaw": 0.0}

    bridge._finish_goal(status, bridge._goal_generation)

    assert bridge._escape_from is None
    assert bridge.mode == "idle"


def test_escape_reverses_without_turning(sim, monkeypatch):
    monkeypatch.setattr(sim, "Twist", _Twist)
    bridge = make_bridge(sim)
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._finish_goal("failed", bridge._goal_generation)

    bridge.escape_tick()

    command = bridge.pub_cmd.publish.call_args.args[0]
    assert command.linear.x == sim.ESCAPE_SPEED < 0
    assert command.angular.z == 0.0


def test_operator_command_invalidates_escape_generation(sim, monkeypatch):
    monkeypatch.setattr(sim, "Twist", _Twist)
    bridge = make_bridge(sim)
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._finish_goal("failed", bridge._goal_generation)
    assert bridge._escape_from is not None

    bridge.drive(0.2, 0.0)
    bridge.pub_cmd.publish.reset_mock()
    bridge.escape_tick()

    assert bridge._escape_from is None
    assert bridge.mode == "teleop"
    bridge.pub_cmd.publish.assert_not_called()


def test_escape_does_nothing_when_never_armed(sim, monkeypatch):
    monkeypatch.setattr(sim, "Twist", _Twist)
    bridge = make_bridge(sim)

    bridge.escape_tick()

    bridge.pub_cmd.publish.assert_not_called()
