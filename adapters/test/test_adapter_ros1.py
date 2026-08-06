"""Hardware adapter tests, with ROS stubbed out.

`adapter_ros1` imports the whole ROS 1 stack at module scope, so this stubs it
the same way `test_adapter_ros2.py` and `test_adapter_sim_pose.py` do. What is
testable without a robot is exactly the part most likely to be wrong on one:
capability advertisement, config merging, battery normalisation, and the
deadman — the protocol layer, which is identical between the ROS 1 and ROS 2
adapters by construction.

What these CANNOT test is whether the topic names, QoS/latching and frame
names are right for any particular robot. That needs hardware — see
docs/hardware-bringup.md.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "rospy",
    "actionlib",
    "move_base_msgs", "move_base_msgs.msg",
    "actionlib_msgs", "actionlib_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "sensor_msgs", "sensor_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "tf2_ros", "websockets", "cv2",
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
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.pub_nav_goal = MagicMock() if cfg["topics"].get("nav_goal") else None
    bridge.pub_nav_stop = MagicMock() if cfg["topics"].get("nav_stop") else None
    # Mirrors HardwareBridge.__init__: nav_goal (topic-based) takes priority
    # over actions.navigate_to_pose (actionlib) when both are configured.
    bridge.nav_client = (
        MagicMock()
        if bridge.pub_nav_goal is None and cfg.get("actions", {}).get("navigate_to_pose")
        else None
    )
    bridge.mode = "idle"
    bridge.nav_status = "idle"
    bridge.goal = None
    bridge._goal_generation = 0
    bridge._last_drive_at = 0.0
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


def test_capabilities_reflect_configuration_only(mod):
    """Protocol rule 4: never advertise a capability you cannot honour."""
    full = _bridge(mod, {"topics": {"battery": "battery_state",
                                    "camera_compressed": "cam/compressed"}})
    caps = full.capabilities()
    assert {"navigate", "map", "camera", "battery", "estop"} <= set(caps)

    bare = _bridge(mod, {
        "topics": {"odom": "odom", "map": "", "cmd_vel": "", "battery": "",
                   "camera": "", "camera_compressed": ""},
        "actions": {"navigate_to_pose": ""},
    })
    assert bare.capabilities() == []


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
    assert bridge.nav_status == "active", "a stale done_cb must not overwrite newer state"
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
    bridge = _bridge(mod, {
        "topics": {"nav_goal": "move_base_simple/goal"},
        "actions": {"navigate_to_pose": "move_base"},
    })
    assert bridge.pub_nav_goal is not None
    assert bridge.nav_client is None
    assert "navigate" in bridge.capabilities()


def test_navigate_to_topic_publishes_pose_and_releases_any_prior_stop(mod):
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal",
                                       "nav_stop": "stop"}})
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
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal"},
                            "nav_goal_tolerance_m": 0.5})
    bridge.goal = {"x": 5.0, "y": 5.0}
    bridge.nav_status = "active"
    bridge.map_pose = lambda: {"x": 5.3, "y": 5.1, "yaw": 0.0}  # 0.32 m away

    bridge._check_topic_nav_progress()
    assert bridge.nav_status == "succeeded"
    assert bridge.goal is None
    assert bridge.mode == "idle"


def test_topic_nav_progress_stays_active_when_far(mod):
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal"},
                            "nav_goal_tolerance_m": 0.5})
    bridge.goal = {"x": 5.0, "y": 5.0}
    bridge.nav_status = "active"
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}

    bridge._check_topic_nav_progress()
    assert bridge.nav_status == "active"
    assert bridge.goal == {"x": 5.0, "y": 5.0}


def test_cancel_goal_halts_a_topic_based_nav_stack(mod):
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal",
                                       "nav_stop": "stop"}})
    bridge.nav_status = "active"
    bridge.goal = {"x": 1.0, "y": 1.0}

    bridge.cancel_goal()
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None
    bridge.pub_nav_stop.publish.assert_called_once()
    mod.Int8.assert_any_call(data=1)


def test_teleop_preempts_an_active_topic_based_nav_goal(mod):
    """Operator input must always win over autonomy sharing the same cmd_vel."""
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal",
                                       "nav_stop": "stop"}})
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
    bridge = _bridge(mod, {"topics": {"nav_goal": "move_base_simple/goal",
                                       "nav_stop": "stop"}})
    bridge.nav_status = "idle"

    bridge.drive(0.0, 0.0)
    bridge.pub_nav_stop.publish.assert_not_called()
    assert bridge.nav_status == "idle"


def test_shipped_configs_are_valid_and_disable_what_they_lack(mod):
    import yaml

    for name in ("generic", "scout_mini"):
        path = REPO / "adapters" / "adapter_ros1" / "config" / f"{name}.yaml"
        cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
        assert cfg["map_frame"] and cfg["base_frame"], f"{name} needs both frames"
        assert cfg["rates"]["state_hz"] > 0
        # The preview cap in the protocol is 5 Hz; never configure faster.
        assert cfg["rates"]["camera_period_s"] >= 0.2, f"{name} exceeds the 5 Hz cap"


def test_map_cloud_alone_still_advertises_the_map_capability(mod):
    """A robot with no OccupancyGrid publisher, only a registered cloud, must
    still show up as map-capable — the backend builds the grid server-side."""
    bridge = _bridge(mod, {"topics": {"map": "", "map_cloud": "cloud_registered"}})
    assert "map" in bridge.capabilities()

    neither = _bridge(mod, {"topics": {"map": "", "map_cloud": ""}})
    assert "map" not in neither.capabilities()


class _FakeField:
    def __init__(self, name: str, offset: int) -> None:
        self.name = name
        self.offset = offset


def _fake_cloud(points, extra_fields=()):
    """A minimal PointCloud2-shaped object: enough for `_cloud_xyz` to parse.

    `extra_fields` lets a field (like intensity) sit BEFORE x/y/z in the byte
    layout, so the test can catch code that assumes x/y/z are the first three
    floats instead of reading `field.offset` like the real message requires.
    """
    import struct

    prefix_names = list(extra_fields)
    field_names = prefix_names + ["x", "y", "z"]
    fields = [_FakeField(name, i * 4) for i, name in enumerate(field_names)]
    point_step = len(field_names) * 4
    data = b"".join(
        struct.pack("<%df" % len(field_names), *([0.0] * len(prefix_names)), *p)
        for p in points
    )
    return type("M", (), {
        "fields": fields, "point_step": point_step, "data": data,
    })()


def test_cloud_xyz_reads_field_offsets_not_position(mod):
    """x/y/z are not necessarily the first fields — LVI-SAM's cloud carries
    intensity too, and reading fixed offsets would silently misread it."""
    msg = _fake_cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)], extra_fields=["intensity"])
    points = mod.HardwareBridge._cloud_xyz(msg)
    assert points.shape == (2, 3)
    assert points[0].tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert points[1].tolist() == pytest.approx([4.0, 5.0, 6.0])


def test_on_map_cloud_drops_points_outside_the_height_band(mod):
    bridge = _bridge(mod, {"map_cloud_height_band": {"min_z": 0.0, "max_z": 1.0}})
    msg = _fake_cloud([(1.0, 0.0, -0.5), (2.0, 0.0, 0.5), (3.0, 0.0, 5.0)])
    bridge._on_map_cloud(msg)
    assert bridge._scan_points.shape[0] == 1
    assert bridge._scan_points[0].tolist() == pytest.approx([2.0, 0.0])
    assert bridge._scan_dirty is True


def test_on_map_cloud_dedups_onto_the_grid_lattice(mod):
    bridge = _bridge(mod, {"map_cloud_height_band": {"min_z": -1.0, "max_z": 1.0}})
    # Two points a millimetre apart land in the same 5 cm cell.
    msg = _fake_cloud([(1.000, 0.000, 0.0), (1.001, 0.001, 0.0), (5.0, 5.0, 0.0)])
    bridge._on_map_cloud(msg)
    assert bridge._scan_points.shape[0] == 2
