from __future__ import annotations

import math

import sys

import threading

from pathlib import Path

from types import SimpleNamespace

from unittest.mock import MagicMock, patch

import pytest

import yaml

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "rclpy",
    "rclpy.action",
    "rclpy.duration",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.time",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "sensor_msgs",
    "sensor_msgs.msg",
    "action_msgs",
    "action_msgs.msg",
    "tf2_ros",
    "websockets",
    "cv2",
    "std_srvs",
    "std_srvs.srv",
    "spot_msgs",
    "spot_msgs.action",
    "spot_msgs.srv",
]


@pytest.fixture(scope="module")
def mod():
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    for name in _STUBBED:
        sys.modules[name] = MagicMock()
    sys.path.insert(0, str(REPO / "adapters" / "adapter_ros2"))
    try:
        import importlib

        module = importlib.import_module("adapter_ros2")
        yield module
    finally:
        sys.modules.pop("adapter_ros2", None)
        sys.modules.pop("ros2_defaults", None)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def _bridge(mod, cfg_override=None):
    cfg = mod.deep_merge(mod.DEFAULTS, cfg_override or {})
    bridge = mod.HardwareBridge.__new__(mod.HardwareBridge)
    bridge.cfg = cfg
    bridge.id = "r0"
    bridge.navigation_frame = cfg["navigation_frame"]
    bridge.base_frame = cfg["base_frame"]
    bridge.node = MagicMock()
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.path_client = (
        MagicMock() if cfg.get("actions", {}).get("follow_path") else None
    )
    bridge.traj_client = (
        MagicMock() if cfg.get("actions", {}).get("trajectory") else None
    )
    bridge.tf_buffer = MagicMock()
    bridge.mode = "idle"
    bridge.nav_status = "idle"
    bridge.goal = None
    bridge._goal_generation = 0
    # __new__ bypasses HardwareBridge.__init__, but command/callback tests
    # exercise the same lock that protects generation ownership.
    bridge._goal_lock = threading.RLock()
    bridge._nav_execution_enabled = False
    bridge._goal_handle = None
    bridge._trajectory_target = None
    bridge._trajectory_step = ""
    bridge._trajectory_step_count = 0
    bridge._trajectory_step_error = None
    bridge._last_drive_at = 0.0
    # A connected robot, which is what every test that is not about the link
    # itself means to model. The class default is deliberately stale.
    bridge._last_link_at = __import__("time").monotonic()
    bridge._scan_points = None
    # Mirrors __init__: the pose the scan points were captured at.
    bridge._scan_origin = None
    bridge._scan_dirty = False
    bridge._cloud_points = None
    bridge._cloud_dirty = False
    bridge._last_cloud_prepare_at = 0.0
    bridge._global_planned_path = []
    bridge._local_planned_path = []
    bridge._plan_frame_warned = False
    bridge._body_clients = {}
    bridge._velocity_client = (
        MagicMock() if cfg.get("services", {}).get("max_velocity") else None
    )
    bridge._camera_depth_image = None
    bridge._camera_info = None
    bridge._camera_color_info = None
    bridge._camera_depth_cloud = None
    bridge._last_depth_warning_at = 0.0
    bridge._pose_warned = False
    bridge._odom_pose = {"x": 4.0, "y": -2.0, "yaw": 0.5}
    bridge._odom_frame = bridge.navigation_frame
    bridge.http_url = "http://backend"
    bridge.t0 = 0.0
    return bridge


class _ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.value


def _trigger_client(order, name):
    client = MagicMock()
    client.wait_for_service.return_value = True

    def call_async(_req):
        order.append(name)
        future = MagicMock()
        future.done.return_value = True
        resp = MagicMock()
        resp.success = True
        resp.message = ""
        future.result.return_value = resp
        return future

    client.call_async.side_effect = call_async
    return client


def _identity_tf():
    rot = type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
    trans = type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
    transform = type("X", (), {"rotation": rot, "translation": trans})()
    return type("TF", (), {"transform": transform})()


def _yaw_tf(yaw):
    rot = type(
        "Q",
        (),
        {
            "x": 0.0,
            "y": 0.0,
            "z": __import__("math").sin(yaw / 2.0),
            "w": __import__("math").cos(yaw / 2.0),
        },
    )()
    trans = type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
    transform = type("X", (), {"rotation": rot, "translation": trans})()
    return type("TF", (), {"transform": transform})()


def test_trajectory_goal_is_transformed_into_body(mod):
    """spot_driver rejects any Trajectory frame_id other than body."""
    bridge = _bridge(mod, {"actions": {"trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True

    bridge._navigate_trajectory({"x": 1.5, "y": -2.0, "yaw": 0.0})

    bridge.traj_client.send_goal_async.assert_called_once()
    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.header.frame_id == "body"
    assert msg.target_pose.pose.position.x == pytest.approx(1.5)
    assert msg.target_pose.pose.position.y == pytest.approx(-2.0)
    assert msg.duration.sec >= 1
    assert bridge.nav_status == "active"
    assert bridge.goal == {"x": 1.5, "y": -2.0}


def test_trajectory_point_goal_preserves_current_heading(mod):
    """A click without yaw must not silently become absolute map yaw zero."""
    bridge = _bridge(mod, {"actions": {"trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-0.8)
    bridge.traj_client.server_is_ready.return_value = True

    bridge._navigate_trajectory({"x": 1.5, "y": -2.0})

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.orientation.z == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.w == pytest.approx(1.0)


def test_trajectory_explicit_yaw_is_still_honoured(mod):
    bridge = _bridge(mod, {"actions": {"trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-0.8)
    bridge.traj_client.server_is_ready.return_value = True

    bridge._navigate_trajectory({"x": 1.5, "y": -2.0, "yaw": 0.0})

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.orientation.z == pytest.approx(
        __import__("math").sin(-0.8 / 2.0)
    )
    assert msg.target_pose.pose.orientation.w == pytest.approx(
        __import__("math").cos(-0.8 / 2.0)
    )


@pytest.mark.parametrize("code, message", [(4, "Failed to make progress"), (105, "")])
def test_follow_path_result_exposes_nav2_failure_reason(
    mod, monkeypatch, code, message
):
    bridge = _bridge(mod)
    bridge._goal_generation = 11
    goal_status = type("GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5})
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    result_future = MagicMock()
    result_future.result.return_value = type(
        "Outcome",
        (),
        {
            "status": 6,
            "result": type(
                "Result",
                (),
                {
                    "success": False,
                    "error_msg": message,
                    "error_code": code,
                    "FAILED_TO_MAKE_PROGRESS": 105,
                },
            )(),
        },
    )()

    bridge._on_goal_result(result_future, 11)

    assert bridge.nav_status == "failed"
    assert bridge._nav_failure_reason == f"Failed to make progress; error_code={code}"


def test_stale_follow_path_result_cannot_replace_current_failure_reason(
    mod, monkeypatch
):
    bridge = _bridge(mod)
    bridge._goal_generation = 12
    bridge._nav_failure_reason = "current goal failure"
    goal_status = type("GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5})
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    result_future = MagicMock()
    result_future.result.return_value = type(
        "Outcome",
        (),
        {
            "status": 6,
            "result": type(
                "Result", (), {"error_msg": "stale failure", "error_code": 9}
            )(),
        },
    )()

    bridge._on_goal_result(result_future, 11)

    assert bridge.nav_status == "idle"
    assert bridge._nav_failure_reason == "current goal failure"


def test_trajectory_goal_without_tf_is_dropped(mod):
    bridge = _bridge(mod, {"actions": {"trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    bridge.traj_client.server_is_ready.return_value = True

    bridge._navigate_trajectory({"x": 1.0, "y": 1.0})

    bridge.traj_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "failed"


def test_cancel_trajectory_calls_spot_stop(mod):
    """Clearpath's ROS 2 Trajectory server does not honour cancel/preempt."""
    bridge = _bridge(
        mod,
        {
            "actions": {"trajectory": "/trajectory"},
            "services": {"stop": "/stop"},
        },
    )
    order: list[str] = []
    bridge._body_clients = {"stop": _trigger_client(order, "stop")}
    handle = MagicMock()
    bridge._goal_handle = handle
    bridge.nav_status = "active"
    bridge.goal = {"x": 2.0, "y": 2.0}

    bridge.cancel_goal()

    handle.cancel_goal_async.assert_called_once_with()
    assert order == ["stop"]
    zero = bridge.pub_cmd.publish.call_args[0][0]
    assert zero.linear.x == pytest.approx(0.0)
    assert zero.linear.y == pytest.approx(0.0)
    assert zero.angular.z == pytest.approx(0.0)
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None


def test_cancel_trajectory_does_not_wait_for_stop_response(mod):
    """Manual drive must publish even while Spot's /stop call is in flight."""
    bridge = _bridge(
        mod,
        {
            "actions": {"trajectory": "/trajectory"},
            "services": {"stop": "/stop"},
        },
    )
    client = MagicMock()
    client.wait_for_service.return_value = True
    future = MagicMock()
    future.done.return_value = False
    client.call_async.return_value = future
    bridge._body_clients = {"stop": client}
    bridge.nav_status = "active"
    bridge._nav_execution_enabled = True

    bridge.drive(0.2, 0.1)

    client.call_async.assert_called_once()
    assert bridge.pub_cmd.publish.call_count == 2
    manual = bridge.pub_cmd.publish.call_args_list[-1].args[0]
    assert manual.linear.x == pytest.approx(0.2)
    assert manual.angular.z == pytest.approx(0.1)
    assert bridge.mode == "teleop"


def test_drive_watchdog_stops_a_robot_whose_operator_vanished(mod):
    """The failure this prevents is a robot that keeps driving after link loss."""
    import time

    bridge = _bridge(mod, {"drive_timeout_s": 0.05})
    bridge.drive(0.3, 0.0)
    assert bridge.mode == "teleop"

    time.sleep(0.08)
    bridge.drive_watchdog()
    assert bridge.mode == "idle"
    # Last publish must be a zero twist.
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_drive_watchdog_leaves_an_active_operator_alone(mod):
    bridge = _bridge(mod, {"drive_timeout_s": 5.0})
    bridge.drive(0.3, 0.0)
    bridge.drive_watchdog()
    assert bridge.mode == "teleop"


def _plan_msg(frame, points):
    msg = MagicMock()
    msg.header.frame_id = frame
    msg.header.stamp = MagicMock()
    msg.poses = []
    for x, y, z in points:
        pose = MagicMock()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        msg.poses.append(pose)
    return msg


def test_conditional_follow_path_submission_failure_keeps_reserved_generation(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 7
    bridge.path_client.server_is_ready.return_value = True
    bridge.path_client.send_goal_async.side_effect = RuntimeError("send failed")

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_ownership_plan(), expected_generation=7)

    assert result is False
    assert bridge._goal_generation == 7
    assert bridge.nav_status == "active"
    assert bridge.mode == "idle"
    assert bridge.goal is None
    assert bridge.planned_path == []


def test_immediate_follow_path_rejection_leaves_terminal_failure(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge.path_client.server_is_ready.return_value = True
    rejected = MagicMock()
    rejected.accepted = False
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(rejected)

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_ownership_plan())

    assert type(result) is int
    assert result == bridge._goal_generation
    assert bridge.nav_status == "failed"
    assert bridge.mode == "idle"
    assert bridge.goal is None
    assert bridge.planned_path == []


def _ownership_plan(frame="map"):
    from adapters.exploration import PlannerPath, PlannerPose

    return PlannerPath(
        frame,
        1,
        (
            PlannerPose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            PlannerPose(2.0, 2.5, 0.0, 0.0, 0.0, 0.1, 0.995),
        ),
    )


def test_stale_conditional_follow_path_does_not_preempt_newer_goal(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    newer_handle = MagicMock()
    bridge._goal_generation = 9
    bridge._goal_handle = newer_handle
    bridge.goal = {"x": 8.0, "y": 9.0}
    bridge.planned_path = [{"x": 8.0, "y": 9.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.path_client.server_is_ready.return_value = True

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_ownership_plan(), expected_generation=8)

    assert result is None
    bridge.path_client.send_goal_async.assert_not_called()
    newer_handle.cancel_goal_async.assert_not_called()
    assert bridge._goal_generation == 9
    assert bridge._goal_handle is newer_handle
    assert bridge.goal == {"x": 8.0, "y": 9.0}
    assert bridge.planned_path == [{"x": 8.0, "y": 9.0}]
    assert bridge.nav_status == "active"


def test_cancel_while_follow_path_readiness_is_pending_wins_without_blocking(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 4
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.path_client.server_is_ready.return_value = False
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    cancel_done = threading.Event()
    result = []

    def wait_for_server(*, timeout_sec):
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        return True

    bridge.path_client.wait_for_server.side_effect = wait_for_server
    worker = threading.Thread(
        target=lambda: result.append(
            bridge.follow_path(_ownership_plan(), expected_generation=4)
        )
    )
    stopper = threading.Thread(target=lambda: (bridge.cancel_goal(), cancel_done.set()))
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
            stopper.start()
            assert cancel_done.wait(timeout=1.0), "cancel waited on readiness"
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)
            if stopper.ident is not None:
                stopper.join(timeout=2.0)

    assert not worker.is_alive()
    assert not stopper.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "cancelled"
    assert bridge.mode == "idle"


def test_stop_while_follow_path_readiness_is_pending_estops_hardware(mod):
    """The inherited protocol Stop must preempt a worker waiting on Nav2."""
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 4
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.path_client.server_is_ready.return_value = False
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    stop_done = threading.Event()
    result = []

    def wait_for_server(*, timeout_sec):
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        return True

    bridge.path_client.wait_for_server.side_effect = wait_for_server
    worker = threading.Thread(
        target=lambda: result.append(
            bridge.follow_path(_ownership_plan(), expected_generation=4)
        )
    )
    stopper = threading.Thread(target=lambda: (bridge.stop(), stop_done.set()))
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
            stopper.start()
            assert stop_done.wait(
                timeout=1.0
            ), "HardwareBridge.stop waited on readiness"
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)
            if stopper.ident is not None:
                stopper.join(timeout=2.0)

    assert not worker.is_alive()
    assert not stopper.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "cancelled"
    assert bridge.mode == "estop"
    assert bridge.goal is None
    assert bridge.planned_path == []
    assert bridge._goal_handle is None
    # Stop's zero command is the final motion authority while the delayed
    # readiness worker is unwinding.
    bridge.pub_cmd.publish.assert_called()


def test_manual_drive_during_pending_follow_path_invalidates_cancelled_generation(mod):
    """Teleop must supersede a planner handoff even after planner cancellation."""
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 7
    bridge.nav_status = "cancelled"
    bridge.mode = "idle"
    bridge.path_client.server_is_ready.return_value = False
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    result = []

    def wait_for_server(*, timeout_sec):
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        return True

    bridge.path_client.wait_for_server.side_effect = wait_for_server
    worker = threading.Thread(
        target=lambda: result.append(
            bridge.follow_path(_ownership_plan(), expected_generation=7)
        )
    )
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
            bridge.drive(0.2, 0.0)
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge._goal_generation == 8
    assert bridge.nav_status == "cancelled"
    assert bridge.mode == "teleop"
    bridge.pub_cmd.publish.assert_called_once()


def test_follow_path_expiry_during_readiness_prevents_send(mod, monkeypatch):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 6
    bridge.path_client.server_is_ready.return_value = False
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    result = []
    clock = {"now": 100.0}
    monkeypatch.setattr(mod.time, "monotonic", lambda: clock["now"])

    def wait_for_server(*, timeout_sec):
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        clock["now"] = 106.0
        return True

    bridge.path_client.wait_for_server.side_effect = wait_for_server
    worker = threading.Thread(
        target=lambda: result.append(
            bridge.follow_path(
                _ownership_plan(), expected_generation=6, not_after=105.0
            )
        )
    )
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge._goal_generation == 6


def test_stale_readiness_failure_preserves_estop(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge._goal_generation = 10
    bridge.nav_status = "estop"
    bridge.mode = "idle"
    bridge.goal = None
    bridge.path_client.server_is_ready.return_value = False
    bridge.path_client.wait_for_server.return_value = False

    result = bridge.follow_path(_ownership_plan(), expected_generation=9)

    assert result is None
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge._goal_generation == 10
    assert bridge.nav_status == "estop"
    assert bridge.mode == "idle"


def test_accepted_callback_after_cancel_is_canceled_without_resurrecting_state(mod):
    bridge = _bridge(mod, {"actions": {"follow_path": "follow_path"}})
    bridge._goal_generation = 12
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.goal = {"x": 1.0, "y": 2.0}
    bridge.planned_path = [{"x": 1.0, "y": 2.0}]
    bridge.cancel_goal()
    bridge.nav_status = "estop"
    accepted_handle = MagicMock()
    accepted_handle.accepted = True
    future = MagicMock()
    future.result.return_value = accepted_handle

    bridge._on_goal_response(future, generation=12)

    accepted_handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None
    assert bridge.nav_status == "estop"
    assert bridge.mode == "idle"
    assert bridge.goal is None
    assert bridge.planned_path == []


def test_cancel_nav_goal_disables_relay_and_publishes_zero(mod):
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}},
    )
    handle = MagicMock()
    bridge._goal_handle = handle
    bridge._goal_generation = 5
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge._nav_execution_enabled = True

    generation = bridge.cancel_goal()

    assert generation == 6
    handle.cancel_goal_async.assert_called_once_with()
    assert bridge._nav_execution_enabled is False
    bridge._on_nav_cmd_vel(MagicMock())
    bridge.pub_cmd.publish.assert_called_once()
    zero = bridge.pub_cmd.publish.call_args.args[0]
    assert zero.linear.x == 0.0
    assert zero.angular.z == 0.0


def test_stale_conditional_cancel_does_not_zero_newer_motion(mod):
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}},
    )
    bridge._goal_generation = 8
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge._nav_execution_enabled = True
    newer_handle = MagicMock()
    bridge._goal_handle = newer_handle

    assert bridge.cancel_goal_if_current(7) is None

    newer_handle.cancel_goal_async.assert_not_called()
    bridge.pub_cmd.publish.assert_not_called()
    assert bridge._goal_generation == 8
    assert bridge._nav_execution_enabled is True
    assert bridge.nav_status == "active"


def test_stale_conditional_cancel_and_status_cannot_change_newer_state(mod):
    bridge = _bridge(mod, {"actions": {"follow_path": "follow_path"}})
    newer_handle = MagicMock()
    bridge._goal_generation = 15
    bridge._goal_handle = newer_handle
    bridge.goal = {"x": 3.0, "y": 4.0}
    bridge.planned_path = [{"x": 3.0, "y": 4.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"

    assert bridge.cancel_goal_if_current(14) is None
    assert bridge.set_nav_status_if_current(14, "failed") is False
    newer_handle.cancel_goal_async.assert_not_called()
    assert bridge._goal_generation == 15
    assert bridge._goal_handle is newer_handle
    assert bridge.goal == {"x": 3.0, "y": 4.0}
    assert bridge.planned_path == [{"x": 3.0, "y": 4.0}]
    assert bridge.nav_status == "active"
    assert bridge.mode == "nav"


def test_empty_follow_path_fails_without_canceling_active_motion(mod):
    from adapters.exploration import PlannerPath

    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    handle = MagicMock()
    bridge._goal_handle = handle

    assert bridge.follow_path(PlannerPath("map", 1, ())) is False
    handle.cancel_goal_async.assert_not_called()
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "idle"


def test_nav_cmd_vel_relay_forwards_only_while_navigating(mod):
    """Nav2 output must not reach the driver outside an active action goal."""
    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    twist = MagicMock()

    bridge.nav_status = "idle"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_not_called()

    bridge.nav_status = "active"
    # A planner may keep the public status active while a replacement action
    # is being prepared. Only an accepted, currently executing action may
    # reopen the isolated Nav2 velocity relay.
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_not_called()

    bridge._nav_execution_enabled = True
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_called_once_with(twist)


def test_teleop_cancels_an_active_action_goal(mod):
    bridge = _bridge(mod, {"actions": {"follow_path": "follow_path"}})
    handle = MagicMock()
    bridge._goal_handle = handle
    bridge.nav_status = "active"
    bridge.goal = {"x": 2.0, "y": 2.0}

    bridge.drive(0.1, 0.0)

    handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None
    assert bridge.mode == "teleop"


def test_stale_accepted_action_response_is_cancelled(mod):
    bridge = _bridge(mod, {"actions": {"follow_path": "follow_path"}})
    bridge._goal_generation = 2
    handle = MagicMock()
    handle.accepted = True
    future = MagicMock()
    future.result.return_value = handle

    bridge._on_goal_response(future, generation=1)

    handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None


def test_teleop_zero_command_does_not_touch_an_idle_nav_state(mod):
    """drive(0, 0) is sent routinely (deadman, initial state) — it must not
    spuriously cancel a goal that isn't even active."""
    bridge = _bridge(mod)
    bridge.nav_status = "idle"

    bridge.drive(0.0, 0.0)
    assert bridge.nav_status == "idle"


def test_link_watchdog_stops_autonomy_when_the_operator_link_goes_stale(mod):
    """The Botman accident, 2026-08-12: a goal ran on after the link dropped.

    `drive_timeout_s` cannot cover this. Held teleop repeats, so its silence is
    detectable; an active goal sends nothing, and the relay is gated on
    `nav_status` alone — which nothing revoked when the link wedged.
    """
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge.nav_status = "active"
    bridge._nav_execution_enabled = True
    bridge._goal_handle = MagicMock(accepted=True)
    bridge.goal = {"x": 1.0, "y": 2.0}
    bridge.planned_path = [{"x": 1.0, "y": 2.0}]
    bridge.note_link_activity()

    # Fresh link: autonomy is relayed as before.
    bridge._on_nav_cmd_vel(MagicMock())
    assert bridge.pub_cmd.publish.call_count == 1

    time.sleep(0.08)
    bridge.link_watchdog()

    assert bridge.nav_status == "cancelled", (
        "the goal must be cancelled, not merely overridden: the relay is gated "
        "on nav_status, so leaving it active lets Nav2's next sample overwrite "
        "the stop within milliseconds"
    )
    assert bridge._goal_handle is None
    assert bridge.goal is None
    assert bridge.planned_path == []
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_link_watchdog_leaves_a_healthy_link_navigating(mod):
    """A deadman that trips on a working link is worse than none."""
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 5.0},
    )
    bridge.nav_status = "active"
    bridge._nav_execution_enabled = True
    bridge.note_link_activity()

    bridge.link_watchdog()
    assert bridge.nav_status == "active"

    bridge._on_nav_cmd_vel(MagicMock())
    assert bridge.pub_cmd.publish.call_count == 1


def test_link_watchdog_ignores_a_robot_that_is_not_navigating(mod):
    """No goal, nothing to cancel — teleop keeps its own separate deadman."""
    import time

    bridge = _bridge(mod, {"link_timeout_s": 0.05})
    bridge.nav_status = "idle"
    time.sleep(0.08)

    bridge.link_watchdog()
    assert bridge.nav_status == "idle"
    bridge.pub_cmd.publish.assert_not_called()


class _HeldGoalLock:
    """Hold ``bridge._goal_lock`` from another thread, as a session worker does."""

    def __init__(self, bridge):
        self._lock = bridge._goal_lock
        self._taken = threading.Event()
        self._release = threading.Event()
        self._thread = threading.Thread(target=self._hold, daemon=True)

    def _hold(self):
        with self._lock:
            self._taken.set()
            self._release.wait(5.0)

    def __enter__(self):
        self._thread.start()
        assert self._taken.wait(1.0)
        return self

    def __exit__(self, *_exc):
        self._release.set()
        self._thread.join(1.0)


def test_ros_thread_watchdog_never_waits_for_a_held_goal_lock(mod):
    """C1: the 20 Hz timer shares the ROS thread with the replies a lock
    holder may be waiting on, so it must skip a busy lock, not block on it."""
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge._pending_drive = None
    bridge.nav_status = "active"
    bridge._nav_execution_enabled = True
    bridge._goal_handle = MagicMock(accepted=True)
    bridge.goal = {"x": 1.0, "y": 2.0}
    bridge._last_link_at = time.monotonic() - 1.0

    with _HeldGoalLock(bridge):
        started = time.monotonic()
        bridge._watchdogs()
        assert time.monotonic() - started < 0.1
        assert bridge.nav_status == "active"
        bridge.pub_cmd.publish.assert_not_called()

    bridge._watchdogs()  # The next tick after release cancels the stale link.
    assert bridge.nav_status == "cancelled"
    assert bridge._goal_handle is None
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_pending_drive_waits_for_the_next_tick_while_the_goal_lock_is_held(mod):
    import time

    bridge = _bridge(mod)
    bridge._pending_drive = None
    bridge.note_drive_command(0.2, -0.1)

    with _HeldGoalLock(bridge):
        started = time.monotonic()
        bridge.apply_pending_drive()
        assert time.monotonic() - started < 0.1
        assert bridge._pending_drive == (0.2, -0.1)
        bridge.pub_cmd.publish.assert_not_called()

    bridge.apply_pending_drive()
    assert bridge._pending_drive is None
    sent = bridge.pub_cmd.publish.call_args[0][0]
    assert sent.linear.x == 0.2 and sent.angular.z == -0.1


def test_nav_cmd_vel_sample_is_dropped_while_the_goal_lock_is_held(mod):
    import time

    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    bridge.nav_status = "active"
    bridge._nav_execution_enabled = True
    twist = MagicMock()

    with _HeldGoalLock(bridge):
        started = time.monotonic()
        bridge._on_nav_cmd_vel(twist)
        assert time.monotonic() - started < 0.1
        bridge.pub_cmd.publish.assert_not_called()

    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_called_once_with(twist)


def _link_bridge(mod, hooks):
    """A bridge that is nothing but the surface `run_robot` drives."""

    class _Bridge:
        cfg = mod.deep_merge(
            mod.DEFAULTS,
            # A fast pump and an always-due map upload, so the scenario reaches
            # the blocking call immediately instead of waiting out a real period.
            {
                "rates": {
                    "state_hz": 50.0,
                    "map_period_s": 0.0,
                    "camera_period_s": 3600.0,
                }
            },
        )
        id = "r0"
        t0 = 0.0
        # Part of the surface since camera uploads became demand-driven: an
        # adapter defaults to "watched" so a backend that never sends
        # `camera_interest` keeps uploading.
        camera_watched = True

        def __init__(self):
            self.node = MagicMock()

        def state(self):
            return {"type": "robot_state", "robot_id": "r0"}

        def hello(self):
            return {"type": "hello", "robot_id": "r0"}

        def capabilities(self):
            return []

        def note_link_activity(self):
            pass

        def upload_map(self):
            return hooks["upload_map"]()

        def upload_scan(self):
            pass

        def upload_cloud(self):
            pass

        def upload_camera(self):
            pass

        def run_detection(self):
            pass

        def take_detections(self):
            return None

        def refresh_settings(self):
            pass

        def cancel_goal(self):
            pass

        def drive(self, *_args):
            pass

    return _Bridge()


def _settings_response(mod, monkeypatch, settings: dict):
    """Serve one /api/settings body to refresh_settings()."""
    import contextlib
    import json

    @contextlib.contextmanager
    def urlopen(_url, timeout=None):
        class Response:
            def read(self):
                return json.dumps({"settings": settings}).encode()

        yield Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)


def test_losing_the_socket_cancels_the_goal_before_zeroing_cmd_vel(mod, monkeypatch):
    """A robot that keeps driving after losing its operator is the one that hurts
    someone. Order matters: while nav_status is still "active" the relay in
    `_on_nav_cmd_vel` overwrites a zero Twist with Nav2's next sample, so
    zeroing without cancelling stops the robot for milliseconds and no more.
    """
    import asyncio

    class _Conn:
        async def __aenter__(self):
            raise ConnectionResetError("network went away mid-goal")

        async def __aexit__(self, *_exc):
            return False

    monkeypatch.setattr(mod.websockets, "connect", lambda *a, **k: _Conn())

    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    bridge.nav_status = "active"
    order = []
    real_cancel = bridge.cancel_goal

    def traced_cancel():
        order.append("cancel")
        return real_cancel()

    bridge.cancel_goal = traced_cancel
    real_drive = bridge.drive

    def traced_drive(lin, ang):
        order.append(("drive", lin, ang))
        return real_drive(lin, ang)

    bridge.drive = traced_drive

    async def scenario():
        task = asyncio.ensure_future(mod.run_robot(bridge, "ws://test"))
        await asyncio.sleep(0.2)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(scenario())

    assert order, "link loss produced no stop at all"
    assert order[0] == "cancel", f"cancel must precede the zero Twist, got {order}"
    assert ("drive", 0.0, 0.0) in order, f"cmd_vel was never zeroed, got {order}"


def _stamp(seconds: float):
    return type(
        "Stamp", (), {"sec": int(seconds), "nanosec": int((seconds % 1) * 1e9)}
    )()


def _depth_image(mod, *, stamp: float, frame: str = "map"):
    values = mod.np.full((8, 8), 2000, dtype="<u2")
    header = type("Header", (), {"stamp": _stamp(stamp), "frame_id": frame})()
    return type(
        "Image",
        (),
        {
            "width": 8,
            "height": 8,
            "encoding": "16UC1",
            "is_bigendian": False,
            "step": 16,
            "data": values.tobytes(),
            "header": header,
        },
    )()


def _route_bridge(mod):
    bridge = _bridge(
        mod,
        {"actions": {"follow_path": "follow_path"}},
    )
    bridge.path_client.server_is_ready.return_value = True
    bridge._pending_drive = None
    bridge.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge.map_pose = lambda: dict(bridge.pose)
    return bridge


def _route_plan(frame="map"):
    from adapters.exploration import PlannerPath, PlannerPose

    return PlannerPath(
        frame,
        1,
        tuple(PlannerPose(0.5 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) for i in range(21)),
    )


def _submit_route(bridge, plan, expected_generation=None):
    handle = MagicMock()
    handle.accepted = True
    handle.get_result_async.return_value = MagicMock()
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(handle)
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        generation = bridge.follow_path(plan, expected_generation=expected_generation)
    assert type(generation) is int
    assert bridge._goal_handle is handle
    return generation, handle


def _rock_hardware(bridge, clock, seconds, at=2.5):
    for tick in range(int(round(seconds / 0.2))):
        clock[0] += 0.2
        bridge._last_link_at = clock[0]
        bridge.pose = {"x": at - (0.3 if tick % 2 else 0.0), "y": 0.0, "yaw": 0.0}
        bridge._watchdogs()


def test_hardware_capabilities_never_advertise_reset(mod):
    bridge = _bridge(mod)
    assert "reset" not in bridge.capabilities()
    assert "reset" not in bridge.hello()["capabilities"]


def test_pose_lookup_uses_navigation_frame_and_base_frame(mod):
    bridge = _bridge(mod)
    transform = _yaw_tf(0.5)
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.75
    bridge.tf_buffer.lookup_transform.return_value = transform

    pose = bridge.map_pose()

    bridge.tf_buffer.lookup_transform.assert_called_once_with(
        "odom", "base_link", mod.rclpy.time.Time()
    )
    assert pose["x"] == pytest.approx(1.25)
    assert pose["y"] == pytest.approx(-0.75)
    assert pose["yaw"] == pytest.approx(0.5)


def test_hardware_bridge_constructs_with_default_odometry_and_plan_topics(mod):
    """__init__ subscribes odom and plan, so their message types must import."""
    node = MagicMock()
    bridge = mod.HardwareBridge(node, "r0", mod.load_config(None), "http://backend")

    subscribed = {
        call.args[1]: call.args[0] for call in node.create_subscription.call_args_list
    }
    assert subscribed["odom"] is mod.Odometry
    assert subscribed["plan"] is mod.NavPath
    assert bridge.navigation_frame == "odom"


# -- goal ownership: shared with the simulation bridge ----------------------
#
# Both bridges close the Nav2 velocity relay at once on cancel, so nothing
# waits for the action server and replacement routes stay publicly "active".


@pytest.mark.parametrize("pending, status", [(False, "cancelled"), (True, "active")])
def test_hardware_conditional_cancel_reports_pending_as_active(mod, pending, status):
    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    bridge._goal_generation = 3
    bridge.nav_status, bridge.mode = "active", "nav"
    bridge._nav_execution_enabled = True

    assert bridge.cancel_goal_if_current(3, pending=pending) == 4

    assert bridge._goal_generation == 4
    assert bridge.nav_status == status
    assert bridge.mode == "idle"
    assert bridge._nav_execution_enabled is False
    bridge.pub_cmd.publish.assert_called_once()


def test_hardware_status_write_closes_the_relay_unless_active(mod):
    bridge = _bridge(mod)
    bridge._goal_generation = 3
    bridge._nav_execution_enabled = True

    assert bridge.set_nav_status_if_current(3, "active") is True
    assert bridge._nav_execution_enabled is True
    assert bridge.set_nav_status_if_current(3, "failed") is True
    assert bridge.nav_status == "failed"
    assert bridge._nav_execution_enabled is False


def test_hardware_pending_goal_reports_active_with_the_relay_closed(mod):
    bridge = _bridge(mod)
    bridge._goal_generation = 3
    bridge.nav_status = "failed"
    bridge._nav_execution_enabled = True

    assert bridge.set_goal_pending_if_current(2) is False
    assert bridge.nav_status == "failed"
    assert bridge._nav_execution_enabled is True
    assert bridge.set_goal_pending_if_current(3) is True
    assert bridge.nav_status == "active"
    assert bridge._nav_execution_enabled is False


@pytest.mark.parametrize("terminal", ["success", "abort", "failure"])
def test_hardware_terminal_result_stops_an_open_gate_once(mod, monkeypatch, terminal):
    bridge = _route_bridge(mod)
    generation, handle = _submit_route(bridge, _route_plan())
    velocities = []
    bridge.pub_cmd = SimpleNamespace(publish=velocities.append)
    command = SimpleNamespace(linear=SimpleNamespace(x=0.3))
    bridge._on_nav_cmd_vel(command)
    assert velocities == [command]
    velocities.clear()
    statuses = SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_CANCELED=5)
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", statuses)
    monkeypatch.setattr(
        sys.modules["geometry_msgs.msg"],
        "Twist",
        lambda: SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace()),
    )
    outcome = {
        "success": SimpleNamespace(status=4),
        "abort": SimpleNamespace(status=6),
        "failure": RuntimeError("result channel failed"),
    }[terminal]
    future = MagicMock()
    if isinstance(outcome, Exception):
        future.result.side_effect = outcome
    else:
        future.result.return_value = outcome
    bridge._on_goal_result(future, generation, handle)
    assert len(velocities) == 1
    assert vars(velocities[0].linear) == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert vars(velocities[0].angular) == {"x": 0.0, "y": 0.0, "z": 0.0}
    bridge._on_goal_result(future, generation, handle)
    bridge._on_nav_cmd_vel(command)
    assert len(velocities) == 1


def test_hardware_state_probes_link_quality_towards_the_backend(mod, monkeypatch):
    import adapters.runtime as runtime

    calls = []
    monkeypatch.setattr(
        runtime,
        "read_link_quality",
        lambda iface, **target: calls.append((iface, target)) or {"quality": 1.0},
    )
    bridge = _bridge(mod, {"network_iface": "wlan0"})
    bridge.http_url = "http://backend:8080"
    bridge.battery = None
    bridge.planned_path = []

    assert bridge.state()["network"] == {"quality": 1.0}
    assert calls == [("wlan0", {"host": "backend", "port": 8080})]


def test_hardware_reports_controller_acceptance_and_terminal_to_exploration(
    mod, monkeypatch
):
    """Exploration replans on the controller's terminal event, as in simulation."""
    bridge = _route_bridge(mod)
    bridge.exploration = MagicMock()
    generation, handle = _submit_route(bridge, _route_plan())

    bridge.exploration.controller_accepted.assert_called_once_with(generation)
    bridge.exploration.controller_finished.assert_not_called()

    goal_status = type("GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5})
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    outcome = MagicMock()
    outcome.result.return_value = SimpleNamespace(
        status=4, result=SimpleNamespace(success=True)
    )
    bridge._on_goal_result(outcome, generation, handle)

    assert bridge.nav_status == "succeeded"
    bridge.exploration.controller_finished.assert_called_once_with()


def test_hardware_route_stall_reports_the_controller_terminal(mod):
    bridge = _route_bridge(mod)
    bridge.exploration = MagicMock()
    generation, _handle = _submit_route(bridge, _route_plan())

    assert bridge._fail_route_progress(generation, "stalled") is True

    assert bridge.nav_status == "failed"
    bridge.exploration.controller_finished.assert_called_once_with()


def test_rejected_or_cancelled_hardware_goals_do_not_claim_acceptance(mod):
    bridge = _route_bridge(mod)
    bridge.exploration = MagicMock()
    rejected = MagicMock()
    rejected.accepted = False
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(rejected)

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        bridge.follow_path(_route_plan())

    bridge.exploration.controller_accepted.assert_not_called()
    bridge.exploration.controller_finished.assert_called_once_with()
    bridge.cancel_goal()
    bridge.exploration.controller_finished.assert_called_once_with()
