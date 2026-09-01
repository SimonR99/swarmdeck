"""Hardware adapter tests, with ROS stubbed out.

`adapter_ros2` imports the whole ROS stack at module scope, so this stubs it the
same way `test_adapter_sim_pose.py` does. What is testable without a robot is
exactly the part most likely to be wrong on one: capability advertisement,
config merging, battery normalisation, and the deadman.

What these CANNOT test is whether the topic names, QoS choices and frame names
are right for any particular robot. That needs hardware — see
docs/operations/hardware-bringup.md.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

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
    bridge.map_frame = cfg["map_frame"]
    bridge.base_frame = cfg["base_frame"]
    bridge.node = MagicMock()
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.nav_client = (
        MagicMock() if cfg.get("actions", {}).get("navigate_to_pose") else None
    )
    bridge.traj_client = (
        MagicMock() if cfg.get("actions", {}).get("trajectory") else None
    )
    bridge.tf_buffer = MagicMock()
    bridge.mode = "idle"
    bridge.nav_status = "idle"
    bridge.goal = None
    bridge._goal_generation = 0
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
    return bridge


def test_config_merge_is_deep_not_shallow(mod):
    """Overriding one topic must not delete the others.

    A shallow merge would silently drop `map` and `cmd_vel` when a robot config
    sets only `odom`, disabling capabilities the robot actually has.
    """
    merged = mod.deep_merge(mod.DEFAULTS, {"topics": {"odom": "wheel_odom"}})
    assert merged["topics"]["odom"] == "wheel_odom"
    assert merged["topics"]["map"] == "map"
    assert merged["topics"]["cmd_vel"] == "cmd_vel"


def test_hello_uses_the_shared_protocol_envelope(mod):
    from adapters.runtime import PROTOCOL_VERSION, TRANSPORT_DEFAULTS

    bridge = _bridge(mod)
    msg = bridge.hello()
    assert msg["protocol"] == PROTOCOL_VERSION
    assert msg["adapter"] == "adapter_ros2/0.1.0"
    assert "reset" not in msg["capabilities"]
    assert bridge.cfg["ping_interval_s"] == TRANSPORT_DEFAULTS["ping_interval_s"]


def test_capabilities_reflect_configuration_only(mod):
    """Protocol rule 4: never advertise a capability you cannot honour."""
    full = _bridge(
        mod,
        {
            "network_iface": "auto",
            "topics": {
                "battery": "battery_state",
                "camera_compressed": "cam/compressed",
            },
        },
    )
    caps = full.capabilities()
    assert {"navigate", "map", "camera", "battery", "network", "estop"} <= set(caps)

    bare = _bridge(
        mod,
        {
            "topics": {
                "odom": "odom",
                "map": "",
                "cmd_vel": "",
                "battery": "",
                "camera": "",
                "camera_compressed": "",
            },
            "actions": {"navigate_to_pose": ""},
        },
    )
    assert bare.capabilities() == []


def test_body_capability_needs_configured_services(mod):
    """Empty Trigger names must not advertise Claim/Stand the GUI cannot honour."""
    none = _bridge(mod)
    assert "body" not in none.capabilities()
    spot = _bridge(
        mod,
        {
            "services": {
                "claim": "/claim",
                "release": "/release",
                "sit": "/sit",
                "stand": "/stand",
                "power_on": "/power_on",
            }
        },
    )
    assert "body" in spot.capabilities()


def test_state_pose_tags_robot_side_network_sample(mod, monkeypatch):
    bridge = _bridge(mod, {"network_iface": "auto"})
    bridge.t0 = __import__("time").monotonic()
    bridge.battery = None
    bridge.planned_path = []
    bridge.map_pose = lambda: {"x": 1.5, "y": -2.0, "yaw": 0.25}
    monkeypatch.setattr(
        mod,
        "read_link_quality",
        lambda _iface: {"interface": "wlan0", "quality_pct": 72.0, "rssi_dbm": -58.0},
    )

    state = bridge.state()
    assert state["pose"] == {"x": 1.5, "y": -2.0, "yaw": 0.25}
    assert state["network"]["quality_pct"] == 72.0


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


def test_body_command_stand_powers_on_first(mod):
    """Clearpath /stand fails if the motors are still off."""
    bridge = _bridge(
        mod,
        {
            "services": {
                "claim": "/claim",
                "release": "/release",
                "sit": "/sit",
                "stand": "/stand",
                "power_on": "/power_on",
            }
        },
    )
    order: list[str] = []
    bridge._body_clients = {
        name: _trigger_client(order, name)
        for name in ("claim", "release", "sit", "stand", "power_on")
    }
    bridge.body_command("stand")
    assert order == ["power_on", "stand"]
    bridge.body_command("claim")
    assert order[-1] == "claim"
    bridge.body_command("not-a-thing")
    assert order[-1] == "claim"


def test_body_command_claim_clears_tablet_keepalive(mod):
    """Tablet Stop leaves a keepalive that blocks /power_on after Claim."""
    names = (
        "claim",
        "release",
        "sit",
        "stand",
        "power_on",
        "estop_release",
        "clear_keepalive",
    )
    bridge = _bridge(mod, {"services": {name: f"/{name}" for name in names}})
    order: list[str] = []
    bridge._body_clients = {name: _trigger_client(order, name) for name in names}
    bridge.body_command("claim")
    assert order == ["claim", "estop_release", "clear_keepalive"]
    order.clear()
    bridge.body_command("stand")
    assert order == ["estop_release", "clear_keepalive", "power_on", "stand"]


def test_set_stand_height_and_clamping(mod, monkeypatch):
    """Stand height is clamped to [-0.15, 0.15] and sent to SetStandHeight."""
    class FakeSetStandHeight:
        class Request:
            def __init__(self):
                self.height = 0.0

        class Response:
            def __init__(self):
                self.success = True
                self.message = "ok"

    monkeypatch.setattr(mod, "SetStandHeight", FakeSetStandHeight)
    bridge = _bridge(
        mod,
        {"services": {"set_stand_height": "/set_stand_height"}},
    )
    calls: list[float] = []
    client = MagicMock()
    client.wait_for_service.return_value = True

    def call_async(req):
        calls.append(req.height)
        fut = MagicMock()
        fut.done.return_value = True
        fut.result.return_value = FakeSetStandHeight.Response()
        return fut

    client.call_async.side_effect = call_async
    bridge._stand_height_client = client
    bridge.pub_body_pose = MagicMock()

    # Normal range
    assert bridge.set_stand_height(0.10) is True
    assert calls[-1] == pytest.approx(0.10)
    assert bridge.pub_body_pose.publish.call_args[0][0].position.z == pytest.approx(0.10)

    # Clamped above max (0.15)
    assert bridge.set_stand_height(0.30) is True
    assert calls[-1] == pytest.approx(0.15)
    assert bridge.pub_body_pose.publish.call_args[0][0].position.z == pytest.approx(0.15)

    # Clamped below min (-0.15)
    assert bridge.set_stand_height(-0.25) is True
    assert calls[-1] == pytest.approx(-0.15)
    assert bridge.pub_body_pose.publish.call_args[0][0].position.z == pytest.approx(-0.15)

    # Via body_command
    bridge.body_command("set_height", height=0.08)
    assert calls[-1] == pytest.approx(0.08)
    assert bridge.pub_body_pose.publish.call_args[0][0].position.z == pytest.approx(0.08)

    bridge.body_command("stand", height=-0.05)
    assert calls[-1] == pytest.approx(-0.05)
    assert bridge.pub_body_pose.publish.call_args[0][0].position.z == pytest.approx(-0.05)


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


def test_trajectory_action_advertises_navigate(mod):
    """Spot has no Nav2 server; /trajectory is what click-to-pose talks to."""
    none = _bridge(mod, {"actions": {"navigate_to_pose": "", "trajectory": ""}})
    assert "navigate" not in none.capabilities()
    spot = _bridge(
        mod,
        {
            "topics": {"cmd_vel": ""},
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
        },
    )
    assert "navigate" in spot.capabilities()
    assert "estop" not in spot.capabilities()


def test_trajectory_goal_is_transformed_into_body(mod):
    """spot_driver rejects any Trajectory frame_id other than body."""
    bridge = _bridge(
        mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}}
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.5, "y": -2.0, "yaw": 0.0})

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
    bridge = _bridge(
        mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}}
    )
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-0.8)
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.5, "y": -2.0})

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.orientation.z == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.w == pytest.approx(1.0)


def test_trajectory_explicit_yaw_is_still_honoured(mod):
    bridge = _bridge(
        mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}}
    )
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-0.8)
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.5, "y": -2.0, "yaw": 0.0})

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.orientation.z == pytest.approx(
        __import__("math").sin(-0.8 / 2.0)
    )
    assert msg.target_pose.pose.orientation.w == pytest.approx(
        __import__("math").cos(-0.8 / 2.0)
    )


def test_trajectory_applies_configured_velocity_limit(mod):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "services": {"max_velocity": "/max_velocity"},
            "trajectory": {
                "control_mode": "differential",
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.05,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True
    bridge._velocity_client.wait_for_service.return_value = True
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = type(
        "Response",
        (),
        {
            "success": True,
            "message": "",
        },
    )()
    bridge._velocity_client.call_async.return_value = future

    bridge.navigate_to({"x": 1.0, "y": 2.0})

    request = bridge._velocity_client.call_async.call_args[0][0]
    assert request.velocity_limit.linear.x == pytest.approx(0.25)
    assert request.velocity_limit.linear.y == pytest.approx(0.05)
    assert request.velocity_limit.angular.z == pytest.approx(0.5)
    bridge.traj_client.send_goal_async.assert_called_once()


def test_differential_trajectory_turns_before_driving(mod):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "services": {"max_velocity": "/max_velocity"},
            "trajectory": {
                "control_mode": "differential",
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.05,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True
    bridge._velocity_client.wait_for_service.return_value = True
    velocity_future = MagicMock()
    velocity_future.done.return_value = True
    velocity_future.result.return_value = type(
        "Response", (), {"success": True, "message": ""}
    )()
    bridge._velocity_client.call_async.return_value = velocity_future

    bridge.navigate_to({"x": 0.0, "y": 2.0})

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.position.x == pytest.approx(0.0)
    assert msg.target_pose.pose.position.y == pytest.approx(0.0)
    assert msg.target_pose.pose.position.z == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.z == pytest.approx(
        math.sin(math.pi / 4.0)
    )
    assert msg.target_pose.pose.orientation.w == pytest.approx(
        math.cos(math.pi / 4.0)
    )
    assert bridge._trajectory_step == "align"


def test_progress_frame_defaults_to_trajectory_frame(mod):
    bridge = _bridge(mod, {"trajectory": {"frame": "body"}})
    assert bridge._progress_frame() == "body"


def test_progress_frame_overrides_trajectory_frame(mod):
    bridge = _bridge(
        mod, {"trajectory": {"frame": "body", "progress_frame": "body_fast"}}
    )
    assert bridge._progress_frame() == "body_fast"


def test_diff_drive_progress_check_reads_progress_frame_not_body(mod):
    """The driver rejects any outgoing frame_id but body; progress_frame must
    only steer the internal re-check TF lookup, never the sent command."""
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "services": {"max_velocity": "/max_velocity"},
            "trajectory": {
                "control_mode": "differential",
                "progress_frame": "body_fast",
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.05,
                    "angular_z": 0.5,
                },
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True
    bridge._velocity_client.wait_for_service.return_value = True
    velocity_future = MagicMock()
    velocity_future.done.return_value = True
    velocity_future.result.return_value = type(
        "Response", (), {"success": True, "message": ""}
    )()
    bridge._velocity_client.call_async.return_value = velocity_future

    bridge.navigate_to({"x": 0.0, "y": 2.0})

    # The initial validity check (send_trajectory) and the diff-drive
    # continuation both read TF; the continuation must use body_fast.
    frames_looked_up = [
        call.args[0] for call in bridge.tf_buffer.lookup_transform.call_args_list
    ]
    assert "body_fast" in frames_looked_up

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.header.frame_id == "body"


def test_explicit_differential_mode_does_not_depend_on_zero_lateral_limit(mod):
    bridge = _bridge(
        mod,
        {
            "trajectory": {
                "control_mode": "differential",
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.05,
                    "angular_z": 0.5,
                },
            }
        },
    )

    assert bridge._trajectory_is_diff_drive() is True


def test_zero_lateral_trajectory_drives_only_on_body_x(mod):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "trajectory": {
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge._trajectory_target = {"x": 2.0, "y": 0.0}
    bridge._goal_generation = 4

    bridge._continue_diff_trajectory(4)

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.position.x == pytest.approx(2.0)
    assert msg.target_pose.pose.position.y == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.z == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.w == pytest.approx(1.0)
    assert bridge._trajectory_step == "drive"


def test_small_off_axis_goal_skips_unnecessary_in_place_turn(mod):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "trajectory": {
                "position_tolerance_m": 0.15,
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.5,
                },
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge._trajectory_target = {"x": 1.0, "y": 0.12}
    bridge._goal_generation = 5

    bridge._continue_diff_trajectory(5)

    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.pose.position.x == pytest.approx(1.0)
    assert msg.target_pose.pose.position.y == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.z == pytest.approx(0.0)
    assert msg.target_pose.pose.orientation.w == pytest.approx(1.0)
    assert bridge._trajectory_step == "drive"


def test_zero_lateral_trajectory_recomputes_target_after_turn(mod, monkeypatch):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "trajectory": {
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge._trajectory_target = {"x": 0.0, "y": 2.0}
    bridge._goal_generation = 7
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge._continue_diff_trajectory(7)
    assert bridge._trajectory_step == "align"

    # After a 90-degree left turn, map->body is rotated -90 degrees. The same
    # map target is now directly ahead and the next action must be straight.
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-math.pi / 2.0)
    goal_status = type(
        "GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5}
    )
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    result_future = MagicMock()
    result_future.result.return_value = type(
        "Outcome",
        (),
        {
            "status": goal_status.STATUS_SUCCEEDED,
            "result": type("Result", (), {"success": True, "message": ""})(),
        },
    )()

    bridge._on_goal_result(result_future, 7)

    assert bridge.traj_client.send_goal_async.call_count == 2
    drive = bridge.traj_client.send_goal_async.call_args[0][0]
    assert drive.target_pose.pose.position.x == pytest.approx(2.0)
    assert drive.target_pose.pose.position.y == pytest.approx(0.0)
    assert drive.target_pose.pose.orientation.z == pytest.approx(0.0)
    assert drive.target_pose.pose.orientation.w == pytest.approx(1.0)
    assert bridge._trajectory_step == "drive"


def test_partial_alignment_abort_continues_only_after_progress(mod, monkeypatch):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "trajectory": {
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge._trajectory_target = {"x": 2.0, "y": 0.4}
    bridge._goal_generation = 9
    bridge._trajectory_step = "align"
    bridge._trajectory_step_error = math.atan2(0.4, 2.0)
    # A partial turn reduced the bearing from ~0.197 rad to ~0.097 rad.
    bridge.tf_buffer.lookup_transform.return_value = _yaw_tf(-0.1)
    goal_status = type(
        "GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5}
    )
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    result_future = MagicMock()
    result_future.result.return_value = type(
        "Outcome",
        (),
        {
            "status": 6,
            "result": type(
                "Result", (), {"success": False, "message": "not at goal"}
            )(),
        },
    )()

    bridge._on_goal_result(result_future, 9)

    bridge.traj_client.send_goal_async.assert_called_once()
    assert bridge.nav_status != "failed"


def test_alignment_abort_fails_when_spot_made_no_progress(mod, monkeypatch):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "trajectory": {
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.0,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge._trajectory_target = {"x": 2.0, "y": 0.4}
    bridge._goal_generation = 10
    bridge._trajectory_step = "align"
    bridge._trajectory_step_error = math.atan2(0.4, 2.0)
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    goal_status = type(
        "GoalStatus", (), {"STATUS_SUCCEEDED": 4, "STATUS_CANCELED": 5}
    )
    monkeypatch.setattr(sys.modules["action_msgs.msg"], "GoalStatus", goal_status)
    result_future = MagicMock()
    result_future.result.return_value = type(
        "Outcome",
        (),
        {
            "status": 6,
            "result": type(
                "Result", (), {"success": False, "message": "not at goal"}
            )(),
        },
    )()

    bridge._on_goal_result(result_future, 10)

    bridge.traj_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "failed"


def test_trajectory_does_not_run_unlimited_when_limit_service_is_down(mod):
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
            "services": {"max_velocity": "/max_velocity"},
            "trajectory": {
                "velocity_limit": {
                    "linear_x": 0.25,
                    "linear_y": 0.25,
                    "angular_z": 0.5,
                }
            },
        },
    )
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True
    bridge._velocity_client.wait_for_service.return_value = False

    bridge.navigate_to({"x": 1.0, "y": 2.0})

    bridge.traj_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "failed"


def test_spot_config_enables_conservative_trajectory_limits():
    config = yaml.safe_load(
        (REPO / "adapters/adapter_ros2/config/spot.yaml").read_text()
    )

    assert config["topics"]["battery"] == "/status/battery_states"
    assert config["services"]["max_velocity"] == "/max_velocity"
    assert config["trajectory"]["control_mode"] == "differential"
    assert config["trajectory"]["velocity_limit"] == {
        "linear_x": 0.25,
        "linear_y": 0.05,
        "angular_z": 0.5,
    }


def test_spot_config_declares_realsense_camera_topics():
    config = yaml.safe_load(
        (REPO / "adapters/adapter_ros2/config/spot.yaml").read_text()
    )
    assert config["topics"]["camera"] == "/d435/camera/color/image_raw"
    assert (
        config["topics"]["camera_compressed"]
        == "/d435/camera/color/image_raw/compressed"
    )
    assert (
        config["topics"]["camera_depth"]
        == "/d435/camera/aligned_depth_to_color/image_raw"
    )
    assert (
        config["topics"]["camera_info"]
        == "/d435/camera/aligned_depth_to_color/camera_info"
    )


def test_hardware_configs_declare_four_corner_footprints():
    for name in ("bunker", "aslan_bunker", "spot"):
        config = yaml.safe_load(
            (REPO / f"adapters/adapter_ros2/config/{name}.yaml").read_text()
        )
        assert len(config["footprint"]) == 4
        assert all(len(point) == 2 for point in config["footprint"])


def test_hardware_configs_declare_the_live_battery_topics():
    expected = {
        "adapters/adapter_ros1/config/scout_mini.yaml": "/scout_status",
        "adapters/adapter_ros2/config/bunker.yaml": "/bunker_status",
        "adapters/adapter_ros2/config/aslan_bunker.yaml": "/bunker_status",
        "adapters/adapter_ros2/config/spot.yaml": "/status/battery_states",
    }
    for relative_path, topic in expected.items():
        config = yaml.safe_load((REPO / relative_path).read_text())
        assert config["topics"]["battery"] == topic


def test_hardware_configs_declare_physical_map_height_bands():
    expected = {
        "bunker": (-0.630, 0.630, 1.800),
        "aslan_bunker": (-0.630, 0.630, 1.800),
        "spot": (-0.500, 0.500, 1.800),
        "scout_mini": (-0.575, 0.575, 1.800),
    }
    for name, (floor_z, lidar_height, max_height) in expected.items():
        config = yaml.safe_load(
            (REPO / f"adapters/adapter_ros2/config/{name}.yaml").read_text()
        )
        band = config["map_cloud_height_band"]
        assert band["floor_z"] == pytest.approx(floor_z)
        assert band["min_z"] == pytest.approx(0.150)
        assert band["max_z"] == pytest.approx(max_height)
        assert config["lidar_height_m"] == pytest.approx(lidar_height)


def test_trajectory_goal_without_tf_is_dropped(mod):
    bridge = _bridge(
        mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}}
    )
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.0, "y": 1.0})

    bridge.traj_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "failed"


def test_cancel_trajectory_calls_spot_stop(mod):
    """Clearpath's ROS 2 Trajectory server does not honour cancel/preempt."""
    bridge = _bridge(
        mod,
        {
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
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
            "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
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

    bridge.drive(0.2, 0.1)

    client.call_async.assert_called_once()
    assert bridge.pub_cmd.publish.call_count == 2
    manual = bridge.pub_cmd.publish.call_args_list[-1].args[0]
    assert manual.linear.x == pytest.approx(0.2)
    assert manual.angular.z == pytest.approx(0.1)
    assert bridge.mode == "teleop"


def test_camera_capability_needs_a_real_topic(mod):
    """An empty topic string must not advertise a camera the GUI will poll."""
    no_cam = _bridge(mod, {"topics": {"camera": "", "camera_compressed": ""}})
    assert "camera" not in no_cam.capabilities()
    raw_only = _bridge(mod, {"topics": {"camera": "image_raw"}})
    assert "camera" in raw_only.capabilities()


def test_battery_normalisation_handles_percent_and_nan(mod):
    """Drivers disagree about 0..1 vs 0..100, and NaN means 'unknown'."""
    bridge = _bridge(mod)

    bridge._on_battery(type("M", (), {"percentage": 0.82})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": 82.0})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": float("nan")})())
    assert bridge.battery is None, "NaN must not be reported as a real level"


def test_spot_battery_array_uses_the_lowest_valid_pack(mod):
    """Spot reports pack percentages in spot_msgs/BatteryStateArray."""
    bridge = _bridge(mod)
    pack = lambda percentage: type("Pack", (), {"charge_percentage": percentage})()

    bridge._on_battery(type("M", (), {"battery_states": [pack(82.0), pack(75.0)]})())
    assert bridge.battery == pytest.approx(0.75)

    bridge._on_battery(type("M", (), {"battery_states": []})())
    assert bridge.battery is None


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


def test_map_frame_odometry_is_an_explicit_tf_fallback(mod):
    bridge = _bridge(mod)
    bridge.map_frame = "map"
    bridge.base_frame = "os_lidar"
    bridge._pose_warned = False
    bridge.tf_buffer = MagicMock()
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    msg = MagicMock()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    bridge._on_odom(msg)

    assert bridge.map_pose() == {"x": 1.25, "y": -0.5, "yaw": 0.0}
    warning = bridge.node.get_logger().warn.call_args[0][0]
    assert "using direct map-frame odometry" in warning
    assert "DRIFTS" not in warning


def test_shipped_configs_are_valid_and_disable_what_they_lack(mod):
    import yaml

    for name in ("generic", "duckiebot", "bunker"):
        path = REPO / "adapters" / "adapter_ros2" / "config" / f"{name}.yaml"
        cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
        assert cfg["map_frame"] and cfg["base_frame"], f"{name} needs both frames"
        assert cfg["rates"]["state_hz"] > 0
        # Detection poll; never faster than the old 5 Hz preview cap.
        assert cfg["rates"]["camera_period_s"] >= 0.2, f"{name} exceeds the 5 Hz cap"


def test_registered_cloud_alone_advertises_map(mod):
    bridge = _bridge(mod, {"topics": {"map": "", "map_cloud": "/registered_scan"}})
    assert "map" in bridge.capabilities()

    neither = _bridge(mod, {"topics": {"map": "", "map_cloud": ""}})
    assert "map" not in neither.capabilities()


class _FakeField:
    def __init__(self, name: str, offset: int) -> None:
        self.name = name
        self.offset = offset


def _fake_cloud(points, extra_fields=(), frame=""):
    import struct

    names = list(extra_fields) + ["x", "y", "z"]
    fields = [_FakeField(name, index * 4) for index, name in enumerate(names)]
    point_step = len(names) * 4
    data = b"".join(
        struct.pack("<%df" % len(names), *([0.0] * len(extra_fields)), *point)
        for point in points
    )
    attrs = {"fields": fields, "point_step": point_step, "data": data}
    if frame:
        header = MagicMock()
        header.frame_id = frame
        header.stamp = MagicMock()
        attrs["header"] = header
    return type("M", (), attrs)()


def test_registered_cloud_uses_declared_offsets_and_height_band(mod):
    bridge = _bridge(mod, {"map_cloud_height_band": {"min_z": 0.0, "max_z": 1.0}})
    msg = _fake_cloud(
        [(1.0, 2.0, -0.5), (3.0, 4.0, 0.5), (5.0, 6.0, 2.5)],
        extra_fields=["intensity"],
    )
    bridge._on_map_cloud(msg)

    assert bridge._scan_points.shape == (1, 2)
    assert bridge._scan_points[0].tolist() == pytest.approx([3.0, 4.0])
    assert bridge._cloud_points.shape == (3, 3)
    assert bridge._cloud_points[0].tolist() == pytest.approx([1.0, 2.0, -0.5])
    assert bridge._cloud_points[1].tolist() == pytest.approx([3.0, 4.0, 0.5])
    assert bridge._cloud_points[2].tolist() == pytest.approx([5.0, 6.0, 2.5])


def test_registered_cloud_height_band_can_use_a_map_floor_reference(mod):
    bridge = _bridge(
        mod,
        {
            "map_cloud_height_band": {
                "floor_z": -0.4,
                "min_z": 0.15,
                "max_z": 0.55,
            }
        },
    )
    msg = _fake_cloud(
        [
            (1.0, 2.0, -0.2),  # 0.20 m above the floor: keep
            (3.0, 4.0, 0.1),  # 0.50 m above the floor: keep
            (5.0, 6.0, 0.2),  # 0.60 m above the floor: drop
        ]
    )
    bridge._on_map_cloud(msg)

    assert bridge._scan_points.tolist() == [[1.0, 2.0], [3.0, 4.0]]


def test_sensor_frame_cloud_is_transformed_to_map_before_keyframing(mod):
    bridge = _bridge(
        mod,
        {
            "map_frame": "world",
            "base_frame": "base_link",
            "map_cloud_height_band": {"min_z": 0.0, "max_z": 2.0},
        },
    )
    tf = _yaw_tf(math.pi / 2.0)
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 5.0
    bridge.tf_buffer.lookup_transform.side_effect = [tf, tf]
    bridge._keyframes = MagicMock()

    bridge._on_map_cloud(_fake_cloud([(1.0, 0.0, 0.5)], frame="livox_frame"))

    import numpy as np

    np.testing.assert_allclose(bridge._scan_points, [[10.0, 6.0]], atol=1e-6)
    considered_points = bridge._keyframes.consider.call_args.args[0]
    np.testing.assert_allclose(considered_points, [[10.0, 6.0, 0.5]], atol=1e-6)
    assert bridge.tf_buffer.lookup_transform.call_args_list[0].args[:2] == (
        "world",
        "livox_frame",
    )


def test_asimov_profile_advertises_transformed_lidar_map(mod):
    path = REPO / "adapters" / "adapter_ros2" / "config" / "unitree_g1.yaml"
    cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
    bridge = _bridge(mod, cfg)

    assert cfg["topics"]["map_cloud"] == "/utlidar/cloud_livox_mid360"
    assert "map" in bridge.capabilities()
    assert "navigate" in bridge.capabilities()


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


def test_ros2_plan_in_map_frame_is_visible_without_a_tf_lookup(mod):
    bridge = _bridge(mod, {"map_frame": "map", "topics": {"plan": "/botman_0/plan"}})
    bridge._on_plan(_plan_msg("map", [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]))

    assert bridge.planned_path == [
        {"x": 1.0, "y": 2.0},
        {"x": 3.0, "y": 4.0},
    ]
    bridge.tf_buffer.lookup_transform.assert_not_called()


def test_ros2_plan_in_another_frame_is_transformed_before_upload(mod):
    bridge = _bridge(mod, {"map_frame": "map", "topics": {"plan": "/plan"}})
    transform = _yaw_tf(math.pi / 2.0)
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 5.0
    bridge.tf_buffer.lookup_transform.return_value = transform

    bridge._on_plan(_plan_msg("base_link", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))

    assert bridge.planned_path == [
        {"x": 10.0, "y": 6.0},
        {"x": 10.0, "y": 7.0},
    ]


def test_ros2_local_plan_is_preferred_over_the_global_route(mod):
    bridge = _bridge(mod, {"map_frame": "map"})
    bridge.t0 = __import__("time").monotonic()
    bridge.battery = None
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._on_plan(_plan_msg("map", [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)]))
    bridge._on_local_plan(
        _plan_msg(
            "map",
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.5, 0.0),
                (2.0, 1.0, 0.0),
            ],
        )
    )

    assert bridge.planned_path == [
        {"x": 0.0, "y": 0.0},
        {"x": 1.0, "y": 0.5},
        {"x": 2.0, "y": 1.0},
    ]
    assert bridge.state()["global_planned_path"] == [
        {"x": 0.0, "y": 0.0},
        {"x": 5.0, "y": 0.0},
    ]
    assert bridge.state()["local_planned_path"] == bridge.planned_path


def test_ros2_empty_local_plan_falls_back_to_global_route(mod):
    bridge = _bridge(mod, {"map_frame": "map"})
    bridge.t0 = __import__("time").monotonic()
    bridge.battery = None
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._on_plan(_plan_msg("map", [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)]))
    bridge._on_local_plan(_plan_msg("map", []))

    assert bridge.planned_path == [
        {"x": 0.0, "y": 0.0},
        {"x": 5.0, "y": 0.0},
    ]
    assert bridge.state()["global_planned_path"] == bridge.planned_path
    assert bridge.state()["local_planned_path"] == []


def test_nav_goal_waits_briefly_for_a_starting_nav2_server(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "navigate_to_pose"}})
    bridge.nav_client.server_is_ready.return_value = False
    bridge.nav_client.wait_for_server.return_value = True

    bridge.navigate_to({"x": 1.0, "y": 2.0})

    bridge.nav_client.wait_for_server.assert_called_once_with(timeout_sec=3.0)
    bridge.nav_client.send_goal_async.assert_called_once()
    assert bridge.nav_status == "active"


def test_nav_cmd_vel_relay_forwards_only_while_navigating(mod):
    """Nav2 output must not reach the driver outside an active action goal."""
    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    twist = MagicMock()

    bridge.nav_status = "idle"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_not_called()

    bridge.nav_status = "active"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_called_once_with(twist)


def test_teleop_cancels_an_active_action_goal(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "navigate_to_pose"}})
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
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "navigate_to_pose"}})
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


def test_detection_batch_survives_concurrent_collection(mod):
    """The websocket may collect a batch while the ROS callback builds it."""
    bridge = _bridge(mod)
    bridge._detections = None
    bridge._depth_map_position = MagicMock(return_value=None)

    class Detection:
        bbox = (0, 0, 1, 1)
        polygon = ()
        label = "rubber_duck"

        def as_protocol(self, detection_id):
            return {"id": detection_id}

    class Detector:
        def detect_bgr(self, _frame):
            yield Detection()
            bridge.take_detections()
            yield Detection()

    bridge._detector = Detector()
    bridge._detect_bgr(MagicMock(), due_checked=True)

    assert [item["id"] for item in bridge.take_detections()] == [
        "rubber_duck_0",
        "rubber_duck_1",
    ]


def test_upload_scan_excludes_returns_inside_robot_footprint(mod, monkeypatch):
    bridge = _bridge(mod, {"footprint_radius": 0.65, "retain_free_space": True})
    bridge.http_url = "http://backend"
    bridge.map_pose = MagicMock(return_value={"x": 1.0, "y": 2.0, "yaw": 0.0})
    bridge._scan_points = mod.np.array(
        [[1.1, 2.1], [1.3, 2.4], [1.7, 2.0]], dtype=mod.np.float32
    )
    bridge._scan_dirty = True
    captured = {}

    class Response:
        def read(self):
            return b"{}"

    def urlopen(request, timeout):
        captured["request"] = request
        return Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)
    bridge.upload_scan()

    request = captured["request"]
    assert "origin_x=1.0&origin_y=2.0" in request.full_url
    assert "retain_free_space=1" in request.full_url
    decoded = mod.np.frombuffer(mod.zlib.decompress(request.data), dtype=mod.np.int16)
    assert decoded.reshape(-1, 2).tolist() == [[170, 200]]


def test_upload_cloud_uses_shared_xyz_transport(mod, monkeypatch):
    bridge = _bridge(mod)
    bridge.http_url = "http://backend"
    bridge._cloud_points = mod.np.array([[1.25, -2.5, 0.75]], dtype=mod.np.float32)
    bridge._cloud_dirty = True
    captured = {}

    class Response:
        def read(self):
            return b"{}"

    def urlopen(request, timeout):
        captured["request"] = request
        return Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)
    bridge.upload_cloud()

    request = captured["request"]
    assert request.full_url == "http://backend/api/adapter/cloud?robot_id=r0&scale=0.01"
    decoded = mod.np.frombuffer(mod.zlib.decompress(request.data), dtype=mod.np.int16)
    assert decoded.reshape(-1, 3).tolist() == [[125, -250, 75]]


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
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_nav_relay_refuses_to_drive_while_the_link_is_stale(mod):
    """Belt and braces: the invariant holds in the relay, not just in a timer."""
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()
    time.sleep(0.08)

    bridge._on_nav_cmd_vel(MagicMock())
    bridge.pub_cmd.publish.assert_not_called()


def test_link_watchdog_leaves_a_healthy_link_navigating(mod):
    """A deadman that trips on a working link is worse than none."""
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 5.0},
    )
    bridge.nav_status = "active"
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


def test_stop_for_exit_cancels_and_zeroes_before_the_process_dies(mod):
    """A restarted adapter must not leave a driving robot behind.

    SIGTERM's default action kills the interpreter without running `finally`,
    so `docker stop` and every Compose recreate used to end with the base still
    executing its last velocity and no deadman left anywhere to countermand it.
    """
    bridge = _bridge(
        mod,
        {
            "topics": {"nav_cmd_vel": "cmd_vel_nav"},
            "actions": {"navigate_to_pose": "navigate_to_pose"},
        },
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()
    bridge.drive(0.4, 0.2)

    bridge.stop_for_exit()

    assert bridge.nav_status == "cancelled"
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


# --------------------------------------------------------------- link plumbing


class _FakeWs:
    """A socket that records what was written and never delivers anything."""

    def __init__(self) -> None:
        self.sent: list[dict] = []

    async def send(self, raw: str) -> None:
        import json

        self.sent.append(json.loads(raw))

    def __aiter__(self):
        return self

    async def __anext__(self):
        import asyncio

        await asyncio.sleep(3600)  # the operator sends nothing in this scenario
        raise StopAsyncIteration


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


def test_a_blocking_upload_does_not_stall_the_state_pump(mod, monkeypatch):
    """The four-robot regression: uploads must never gate telemetry.

    `upload_map` blocks for up to its 5 s timeout, and it blocks for that long
    precisely when the fleet is large — every robot's map queues behind one lock
    on the server. While state and uploads shared a coroutine, that stalled the
    5 Hz pump too, and the backend declares a robot offline after 4 s
    (OFFLINE_AFTER_S) while `link_watchdog` cancels its active goal after 1.5 s.
    So a fourth robot connecting took control of the first one away.
    """
    import asyncio
    import json
    import threading

    ws = _FakeWs()

    class _Conn:
        async def __aenter__(self):
            return ws

        async def __aexit__(self, *_exc):
            return False

    monkeypatch.setattr(mod.websockets, "connect", lambda *a, **k: _Conn())

    started = threading.Event()
    release = threading.Event()

    def blocking_upload():
        started.set()
        release.wait(5.0)  # stands in for the real 5 s urllib timeout
        return None

    bridge = _link_bridge(mod, {"upload_map": blocking_upload})

    def count_states():
        return sum(1 for m in ws.sent if m.get("type") == "robot_state")

    async def scenario():
        task = asyncio.ensure_future(mod.run_robot(bridge, "ws://test"))
        loop = asyncio.get_running_loop()
        # Wait for the upload to actually be in flight before measuring.
        await loop.run_in_executor(None, started.wait, 5.0)
        before = count_states()
        await asyncio.sleep(0.4)
        during = count_states()
        release.set()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return before, during

    before, during = asyncio.run(asyncio.wait_for(scenario(), 15))

    assert started.is_set(), "the scenario never reached the blocking upload"
    # 0.4 s at 50 Hz is ~20 frames; anything above a handful proves the pump ran.
    assert during - before >= 5, (
        f"state stopped while an upload was in flight ({during - before} frames "
        "sent during a 0.4 s block) — uploads are gating telemetry again"
    )


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


def test_refresh_settings_takes_the_capture_floor_not_the_display_floor(
    mod, monkeypatch
):
    """The sidecar must keep returning what the operator is currently hiding.

    The backend enforces the display floor against stored detections, so the
    robot has to go on capturing below it — that band is what lets an operator
    lower a floor again and see the markers come back, instead of waiting for
    the object to be driven past a second time.
    """
    bridge = _bridge(mod)
    bridge.http_url = "http://backend:8080"
    bridge._detection_enabled = True
    bridge._detector = mod.ObjectDetector()
    _settings_response(
        mod,
        monkeypatch,
        {
            "detection_enabled": True,
            "detection_classes": ["rubber_duck"],
            "detection_class_floors": {"rubber_duck": 0.80},
            # Deliberately not the catalog default: refresh_settings swallows its
            # own errors, so a floor that matched the constructor's would let this
            # pass without the request ever having succeeded.
            "detection_capture_floors": {"rubber_duck": 0.15},
        },
    )

    bridge.refresh_settings()

    bridge.node.get_logger().warn.assert_not_called()
    assert bridge._detector.class_floors["rubber_duck"] == 0.15


def test_refresh_settings_falls_back_when_the_backend_is_older(mod, monkeypatch):
    """A new adapter against a backend that only serves display floors."""
    bridge = _bridge(mod)
    bridge.http_url = "http://backend:8080"
    bridge._detection_enabled = True
    bridge._detector = mod.ObjectDetector()
    _settings_response(
        mod,
        monkeypatch,
        {
            "detection_enabled": True,
            "detection_classes": ["rubber_duck"],
            "detection_class_floors": {"rubber_duck": 0.80},
        },
    )

    bridge.refresh_settings()

    bridge.node.get_logger().warn.assert_not_called()
    assert bridge._detector.class_floors["rubber_duck"] == 0.80


def test_websocket_keepalive_is_tight_enough_to_matter(mod):
    """The library defaults leave ~40 s of driving with nobody watching.

    `link_ok()` is fed by `note_link_activity()` after every completed send, and
    a completed send only proves the frame reached the kernel write buffer. At
    ~400 B and 5 Hz a hard cut takes on the order of a hundred seconds to fill
    a normal socket buffer, so backpressure does not catch this — the ping does,
    and only once the socket actually closes does the `except` in `run_robot`
    cancel the goal and zero cmd_vel.
    """
    cfg = mod.deep_merge(mod.DEFAULTS, {})

    interval = float(cfg["ping_interval_s"])
    timeout = float(cfg["ping_timeout_s"])

    # Worst case is interval + timeout: a cut just after a successful pong.
    assert (
        interval + timeout <= 8.0
    ), f"link loss would go undetected for up to {interval + timeout:.0f}s"
    # But not so tight that a degraded link reconnects constantly — every
    # reconnect cancels the active goal.
    assert interval >= 1.0 and timeout >= 2.0


def test_connect_actually_passes_the_keepalive_settings(mod, monkeypatch):
    """A default that never reaches websockets.connect protects nothing."""
    import asyncio

    seen = {}

    class _Conn:
        async def __aenter__(self):
            raise RuntimeError("stop here — the connect kwargs are the subject")

        async def __aexit__(self, *_exc):
            return False

    def fake_connect(url, **kwargs):
        seen.update(kwargs)
        return _Conn()

    monkeypatch.setattr(mod.websockets, "connect", fake_connect)
    bridge = _link_bridge(mod, {"upload_map": lambda: None})

    async def scenario():
        task = asyncio.ensure_future(mod.run_robot(bridge, "ws://test"))
        await asyncio.sleep(0.05)
        task.cancel()
        try:
            await task
        except (asyncio.CancelledError, Exception):
            pass

    asyncio.run(scenario())

    assert seen.get("ping_interval") == float(bridge.cfg["ping_interval_s"])
    assert seen.get("ping_timeout") == float(bridge.cfg["ping_timeout_s"])


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


def test_a_backlog_of_drive_commands_does_not_replay_after_the_link_recovers(mod):
    """The incident: the robot never stopped moving.

    The GUI repeats `drive` every 120 ms while a key is held. Neither the
    browser's WebSocket nor TCP drops those when the link congests — they
    queue. So a stall while holding forward accumulates dozens of identical
    commands, and the `drive(0, 0)` sent on release goes to the BACK of that
    queue. Executing each one inline drives the robot forward through the whole
    backlog, and because every one refreshes `_last_drive_at`, `drive_watchdog`
    never fires either.
    """
    bridge = _bridge(mod)

    # 30 s of held-forward at the GUI's 120 ms cadence, then the release.
    for _ in range(250):
        bridge.note_drive_command(0.4, 0.0)
    bridge.note_drive_command(0.0, 0.0)

    assert bridge.pub_cmd.publish.call_count == 0, (
        "nothing may reach cmd_vel before the timer runs — that is what stops "
        "a backlog being replayed"
    )

    bridge.apply_pending_drive()

    assert bridge.pub_cmd.publish.call_count == 1, "only the newest intent is applied"
    applied = bridge.pub_cmd.publish.call_args[0][0]
    assert applied.linear.x == 0.0 and applied.angular.z == 0.0, (
        "the operator released; the robot must be stopped, not driven forward "
        "through 250 stale commands"
    )


def test_latching_still_delivers_normal_teleop(mod):
    """The fix must not cost responsiveness: the timer runs at 20 Hz and the
    GUI repeats at ~8 Hz, so every command an operator sends is still applied."""
    bridge = _bridge(mod)

    bridge.note_drive_command(0.3, -0.2)
    bridge.apply_pending_drive()

    assert bridge.pub_cmd.publish.call_count == 1
    twist = bridge.pub_cmd.publish.call_args[0][0]
    assert twist.linear.x == pytest.approx(0.3)
    assert twist.angular.z == pytest.approx(-0.2)
    assert bridge.mode == "teleop"

    # An idle tick actuates nothing rather than re-publishing stale velocity.
    bridge.apply_pending_drive()
    assert bridge.pub_cmd.publish.call_count == 1


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


def test_aligned_depth_detection_becomes_a_map_position(mod):
    bridge = _bridge(mod)
    bridge._camera_depth_image = _depth_image(mod, stamp=10.0)
    bridge._camera_info = type(
        "CameraInfo",
        (),
        {"K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0]},
    )()
    image_header = type("Header", (), {"stamp": _stamp(10.1)})()

    position = bridge._depth_map_position((0.25, 0.25, 0.5, 0.5), image_header)

    assert position == pytest.approx({"x": 0.0, "y": 0.0}, abs=0.3)


def test_stale_depth_is_not_attached_to_a_new_detection(mod):
    bridge = _bridge(mod, {"perception": {"depth_max_age_s": 0.2}})
    bridge._camera_depth_image = _depth_image(mod, stamp=10.0)
    bridge._camera_info = type(
        "CameraInfo",
        (),
        {"K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0]},
    )()
    image_header = type("Header", (), {"stamp": _stamp(10.5)})()

    assert bridge._depth_map_position((0.25, 0.25, 0.5, 0.5), image_header) is None


def test_tf_lookup_falls_back_to_latest_when_exact_stamp_fails(mod):
    """When TF at the camera timestamp cannot be interpolated, fallback to latest."""
    bridge = _bridge(mod)
    bridge._camera_depth_image = _depth_image(
        mod, stamp=10.0, frame="oak_stereo_camera_optical_frame"
    )
    bridge._camera_info = type(
        "CameraInfo",
        (),
        {"K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0]},
    )()
    transform = type(
        "TransformStamped",
        (),
        {
            "transform": type(
                "Transform",
                (),
                {
                    "translation": type(
                        "Vector3", (), {"x": 1.0, "y": 2.0, "z": 0.0}
                    )(),
                    "rotation": type(
                        "Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    )(),
                },
            )()
        },
    )()

    calls = []

    def lookup_side_effect(target, source, time_val, *args, **kwargs):
        calls.append((target, source, time_val))
        if len(calls) == 1:
            raise RuntimeError("ExtrapolationException: time not in buffer")
        return transform

    bridge.tf_buffer.lookup_transform.side_effect = lookup_side_effect
    image_header = type("Header", (), {"stamp": _stamp(10.1)})()

    position = bridge._depth_map_position((0.25, 0.25, 0.5, 0.5), image_header)
    assert position is not None
    assert position["x"] == pytest.approx(1.0, abs=0.3)
    assert position["y"] == pytest.approx(2.0, abs=0.3)


def test_unaligned_depth_without_optical_tf_is_not_treated_as_aligned(mod):
    """A colour box on an unaligned depth image is the wrong pixels.

    If the depth-to-colour TF is missing, skip the pin rather than sampling
    the depth image as if it were RGB-aligned.
    """
    bridge = _bridge(mod)
    bridge._camera_depth_image = _depth_image(mod, stamp=10.0, frame="depth_optical")
    bridge._camera_info = type(
        "CameraInfo",
        (),
        {"K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0]},
    )()
    bridge._camera_color_info = type(
        "CameraInfo",
        (),
        {
            "header": type("Header", (), {"frame_id": "color_optical"})(),
            "width": 8,
            "height": 8,
            "K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0],
        },
    )()
    bridge.tf_buffer = MagicMock()
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    image_header = type("Header", (), {"stamp": _stamp(10.1)})()

    assert bridge._depth_map_position((0.25, 0.25, 0.5, 0.5), image_header) is None
