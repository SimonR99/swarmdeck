from __future__ import annotations

import math

import sys

from pathlib import Path

from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "rospy",
    "actionlib",
    "move_base_msgs",
    "move_base_msgs.msg",
    "actionlib_msgs",
    "actionlib_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "tf2_ros",
    "websockets",
    "cv2",
]


@pytest.fixture(scope="module")
def mod():
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    for name in _STUBBED:
        sys.modules[name] = MagicMock()
    sys.path.insert(0, str(REPO / "adapters" / "adapter_ros1"))
    try:
        import importlib

        module = importlib.import_module("adapter_ros1")
        yield module
    finally:
        sys.modules.pop("adapter_ros1", None)
        sys.modules.pop("ros1_defaults", None)
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
    bridge.tf_buffer = MagicMock()
    bridge._pose_warned = False
    bridge._odom_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_frame = bridge.navigation_frame
    bridge.http_url = "http://backend"
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.pub_nav_goal = MagicMock() if cfg["topics"].get("nav_goal") else None
    bridge.pub_nav_stop = MagicMock() if cfg["topics"].get("nav_stop") else None
    bridge.pub_nav_joy = MagicMock() if cfg["topics"].get("nav_joy") else None
    bridge._nav_joy_throttle = float(cfg.get("nav_joy_throttle", 0.5))
    # Mirrors HardwareBridge.__init__: nav_goal (topic-based) takes priority
    # over actions.navigate_to_pose (actionlib) when both are configured.
    bridge.nav_client = (
        MagicMock()
        if bridge.pub_nav_goal is None
        and cfg.get("actions", {}).get("navigate_to_pose")
        else None
    )
    bridge.mode = "idle"
    bridge.nav_status = "idle"
    bridge.goal = None
    bridge._goal_generation = 0
    bridge._last_drive_at = 0.0
    # A connected robot, which is what every test that is not about the
    # link itself means to model. The class default is deliberately stale.
    bridge._last_link_at = __import__("time").monotonic()
    bridge._camera_depth_image = None
    bridge._camera_info = None
    bridge._camera_color_info = None
    bridge._camera_depth_cloud = None
    bridge._scan_points = None
    # Mirrors __init__: the pose the scan points were captured at.
    bridge._scan_origin = None
    bridge._scan_dirty = False
    return bridge


def test_config_merge_is_deep_not_shallow(mod):
    merged = mod.deep_merge(mod.DEFAULTS, {"topics": {"odom": "wheel_odom"}})
    assert merged["topics"]["odom"] == "wheel_odom"
    assert merged["topics"]["cmd_vel"] == "cmd_vel"


def test_hello_uses_the_shared_protocol_envelope(mod):
    from adapters.runtime import PROTOCOL_VERSION, TRANSPORT_DEFAULTS

    bridge = _bridge(mod)
    msg = bridge.hello()
    assert msg["protocol"] == PROTOCOL_VERSION
    assert msg["adapter"] == "adapter_ros1/0.1.0"
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
    assert {"camera", "battery", "network", "estop"} <= set(caps)

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


def _stamp(seconds: float):
    return type("Stamp", (), {"to_sec": lambda self: seconds})()


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


def test_goal_done_ignores_stale_generations(mod):
    """A superseded goal's late server response must not clobber a newer one.

    `SimpleActionClient` tracks only its most recent goal, but a stale `done_cb`
    for an already-cancelled/replaced goal can still fire — this is the same
    staleness guard `adapter_ros2` needs for action futures.
    """
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "move_base"}})
    bridge._goal_generation = 2
    bridge.nav_status = "active"
    bridge.goal = {"x": 1.0, "y": 2.0}

    bridge._on_goal_done(status=3, generation=1)  # stale (generation 1, current is 2)
    assert (
        bridge.nav_status == "active"
    ), "a stale done_cb must not overwrite newer state"
    assert bridge.goal == {"x": 1.0, "y": 2.0}


def test_cancel_goal_bumps_generation_and_clears_state(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "move_base"}})
    bridge._goal_generation = 0
    bridge.goal = {"x": 1.0, "y": 2.0}
    bridge.nav_status = "active"

    bridge.cancel_goal()
    assert bridge.goal is None
    assert bridge.nav_status == "cancelled"
    assert bridge.mode == "idle"
    bridge.nav_client.cancel_goal.assert_called_once()


def test_nav_goal_topic_takes_priority_over_actionlib(mod):
    """A robot only ever has one real navigation stack — configuring both by
    accident must not silently create two clients fighting over goals."""
    bridge = _bridge(
        mod,
        {
            "topics": {"nav_goal": "move_base_simple/goal"},
            "actions": {"navigate_to_pose": "move_base"},
        },
    )
    assert bridge.pub_nav_goal is not None
    assert bridge.nav_client is None
    assert "navigate" in bridge.capabilities()


def test_navigate_to_topic_publishes_pose_and_releases_any_prior_stop(mod):
    bridge = _bridge(
        mod, {"topics": {"nav_goal": "move_base_simple/goal", "nav_stop": "stop"}}
    )
    bridge._navigate_to_topic({"x": 3.0, "y": -1.0, "yaw": 1.5})

    assert bridge.goal == {"x": 3.0, "y": -1.0}
    assert bridge.nav_status == "active"
    assert bridge.mode == "nav"
    published = bridge.pub_nav_goal.publish.call_args[0][0]
    assert published.pose.position.x == 3.0
    assert published.pose.position.y == -1.0
    bridge.pub_nav_stop.publish.assert_called_once()
    mod.Int8.assert_any_call(data=0)


def test_topic_nav_progress_declares_arrival_within_tolerance(mod):
    bridge = _bridge(
        mod,
        {"topics": {"nav_goal": "move_base_simple/goal"}, "nav_goal_tolerance_m": 0.5},
    )
    bridge.goal = {"x": 5.0, "y": 5.0}
    bridge.nav_status = "active"
    bridge.map_pose = lambda: {"x": 5.3, "y": 5.1, "yaw": 0.0}  # 0.32 m away

    bridge._check_topic_nav_progress()
    assert bridge.nav_status == "succeeded"
    assert bridge.goal is None
    assert bridge.mode == "idle"


def test_topic_nav_progress_stays_active_when_far(mod):
    bridge = _bridge(
        mod,
        {"topics": {"nav_goal": "move_base_simple/goal"}, "nav_goal_tolerance_m": 0.5},
    )
    bridge.goal = {"x": 5.0, "y": 5.0}
    bridge.nav_status = "active"
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}

    bridge._check_topic_nav_progress()
    assert bridge.nav_status == "active"
    assert bridge.goal == {"x": 5.0, "y": 5.0}


def test_cancel_goal_halts_a_topic_based_nav_stack(mod):
    bridge = _bridge(
        mod, {"topics": {"nav_goal": "move_base_simple/goal", "nav_stop": "stop"}}
    )
    bridge.nav_status = "active"
    bridge.goal = {"x": 1.0, "y": 1.0}

    bridge.cancel_goal()
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None
    bridge.pub_nav_stop.publish.assert_called_once()
    mod.Int8.assert_any_call(data=1)


def test_teleop_preempts_an_active_topic_based_nav_goal(mod):
    """Operator input must always win over autonomy sharing the same cmd_vel."""
    bridge = _bridge(
        mod, {"topics": {"nav_goal": "move_base_simple/goal", "nav_stop": "stop"}}
    )
    bridge.nav_status = "active"
    bridge.goal = {"x": 2.0, "y": 2.0}

    bridge.drive(0.3, 0.0)
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None
    assert bridge.mode == "teleop"
    mod.Int8.assert_any_call(data=1)


def test_teleop_zero_command_does_not_touch_an_idle_nav_state(mod):
    """drive(0, 0) is sent routinely (deadman, initial state) — it must not
    spuriously cancel a goal that isn't even active."""
    bridge = _bridge(
        mod, {"topics": {"nav_goal": "move_base_simple/goal", "nav_stop": "stop"}}
    )
    bridge.nav_status = "idle"

    bridge.drive(0.0, 0.0)
    bridge.pub_nav_stop.publish.assert_not_called()
    assert bridge.nav_status == "idle"


def _bearing_of(sent) -> float:
    return math.degrees(math.atan2(sent.axes[2], sent.axes[1]))


def test_teleop_preempts_move_base_without_a_nav_stop_topic(mod):
    """Operator motion must cancel autonomy on EVERY ROS 1 nav stack.

    The `nav_status`/`goal` reset used to live inside the `pub_nav_stop` branch,
    but `nav_stop` is a `local_planner` concept and is empty on every move_base
    robot — including `config/generic.yaml`. So on a stock ROS 1 robot the
    operator grabbed the joystick, the actionlib goal stayed live, and move_base
    (which publishes straight to the real cmd_vel) went on fighting teleop for
    the topic. `adapter_ros2.drive` has always cancelled unconditionally.
    """
    bridge = _bridge(
        mod,
        {
            "topics": {"cmd_vel": "cmd_vel", "nav_stop": ""},
            "actions": {"navigate_to_pose": "move_base"},
        },
    )
    assert bridge.pub_nav_stop is None, "this is the configuration that regressed"
    bridge.nav_status = "active"
    bridge.goal = {"x": 4.0, "y": 1.0}

    bridge.drive(0.25, 0.0)

    assert bridge.mode == "teleop"
    assert bridge.goal is None
    assert bridge.nav_status == "cancelled"
    bridge.nav_client.cancel_goal.assert_called_once()


def test_teleop_does_not_cancel_when_nothing_is_navigating(mod):
    """Driving an idle robot must not emit a spurious cancellation."""
    bridge = _bridge(
        mod,
        {
            "topics": {"cmd_vel": "cmd_vel"},
            "actions": {"navigate_to_pose": "move_base"},
        },
    )
    bridge.drive(0.25, 0.0)

    assert bridge.mode == "teleop"
    bridge.nav_client.cancel_goal.assert_not_called()


def _plan_msg(frame, points, stamp=0.0):
    """A nav_msgs/Path stand-in: header.frame_id plus (x, y, z) poses."""
    msg = MagicMock()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.poses = []
    for x, y, z in points:
        pose = MagicMock()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        msg.poses.append(pose)
    return msg


def _plan_bridge(mod, frame="odom"):
    bridge = _bridge(mod, {"navigation_frame": frame, "topics": {"plan": "/path"}})
    bridge.planned_path = []
    bridge._plan_frame_warned = False
    bridge.tf_buffer = MagicMock()
    return bridge


def test_plan_in_navigation_frame_is_passed_through(mod):
    """A plan already in navigation_frame needs no transform lookup."""
    bridge = _plan_bridge(mod)
    bridge._on_plan(_plan_msg("odom", [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]))

    assert bridge.planned_path == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
    bridge.tf_buffer.lookup_transform.assert_not_called()


def test_plan_in_a_vehicle_frame_is_transformed_into_navigation_frame(mod):
    """A local planner vehicle-frame path is transformed into navigation_frame."""
    bridge = _plan_bridge(mod)
    transform = MagicMock()
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 5.0
    transform.transform.translation.z = 0.0
    bridge.tf_buffer.lookup_transform.return_value = transform

    bridge._on_plan(_plan_msg("chassis_link", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))

    assert bridge.planned_path == [{"x": 10.0, "y": 6.0}, {"x": 10.0, "y": 7.0}]


def test_plan_is_dropped_rather_than_drawn_in_the_wrong_frame(mod):
    """No transform means no route — never the untransformed coordinates.

    A route drawn confidently somewhere the robot is not is worse than no route:
    the operator uses it to decide whether the planner is steering around an
    obstacle or through it.
    """
    bridge = _plan_bridge(mod)
    bridge.planned_path = [{"x": 9.0, "y": 9.0}]
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no such frame")

    bridge._on_plan(_plan_msg("chassis_link", [(1.0, 0.0, 0.0)]))

    assert bridge.planned_path == []


def test_empty_plan_clears_the_route(mod):
    """local_planner publishes an empty path when it finds no clear route."""
    bridge = _plan_bridge(mod)
    bridge.planned_path = [{"x": 1.0, "y": 1.0}]
    bridge._local_planned_path = bridge.planned_path.copy()
    bridge._global_planned_path = [{"x": 5.0, "y": 2.0}]

    bridge._on_plan(_plan_msg("chassis_link", []))

    assert bridge.planned_path == []
    assert bridge._local_planned_path == []
    assert bridge._global_planned_path == [{"x": 5.0, "y": 2.0}]


def test_link_watchdog_stops_autonomy_when_the_operator_link_goes_stale(mod):
    """Same deadman as adapter_ros2, same reason — see the Botman accident.

    pathFollower publishes continuously at ~27 Hz, so leaving `nav_status`
    active while the operator is gone means the robot simply keeps going.
    """
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()

    bridge._on_nav_cmd_vel(MagicMock())
    assert bridge.pub_cmd.publish.call_count == 1

    time.sleep(0.08)
    bridge.link_watchdog()

    assert bridge.nav_status == "cancelled"
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_link_watchdog_leaves_a_healthy_link_navigating(mod):
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


def _give_route_map(bridge):
    import time
    from types import SimpleNamespace
    import numpy as np

    bridge._nav_map = SimpleNamespace(
        last_success_at=time.monotonic(),
        cached=SimpleNamespace(
            cells=np.zeros((300, 300), dtype=np.int8),
            resolution=0.1,
            origin_x=-15.0,
            origin_y=-15.0,
        ),
    )


def test_hardware_capabilities_never_advertise_reset(mod):
    bridge = _bridge(mod)
    assert "reset" not in bridge.capabilities()
    assert "reset" not in bridge.hello()["capabilities"]


def test_pose_lookup_uses_navigation_frame_and_base_frame(mod):
    bridge = _bridge(mod)
    transform = MagicMock()
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.75
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = math.sin(0.5 / 2)
    transform.transform.rotation.w = math.cos(0.5 / 2)
    bridge.tf_buffer.lookup_transform.return_value = transform

    pose = bridge.map_pose()

    bridge.tf_buffer.lookup_transform.assert_called_once()
    assert bridge.tf_buffer.lookup_transform.call_args.args[:2] == ("odom", "base_link")
    assert pose["x"] == pytest.approx(1.25)
    assert pose["y"] == pytest.approx(-0.75)
    assert pose["yaw"] == pytest.approx(0.5)


def test_hardware_bridge_constructs_with_default_odometry_and_plan_topics(mod):
    """__init__ subscribes odom and plan, so their message types must import."""
    mod.rospy.Subscriber.reset_mock()
    bridge = mod.HardwareBridge("r0", mod.load_config(None), "http://backend")

    subscribed = {
        call.args[0]: call.args[1] for call in mod.rospy.Subscriber.call_args_list
    }
    assert subscribed[bridge.cfg["topics"]["odom"]] is mod.Odometry
    assert subscribed[bridge.cfg["topics"]["plan"]] is mod.NavPath
