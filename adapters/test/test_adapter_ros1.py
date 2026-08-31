"""Hardware adapter tests, with ROS stubbed out.

`adapter_ros1` imports the whole ROS 1 stack at module scope, so this stubs it
the same way `test_adapter_ros2.py` and `test_adapter_sim_pose.py` do. What is
testable without a robot is exactly the part most likely to be wrong on one:
capability advertisement, config merging, battery normalisation, and the
deadman — the protocol layer, which is identical between the ROS 1 and ROS 2
adapters by construction.

What these CANNOT test is whether the topic names, QoS/latching and frame
names are right for any particular robot. That needs hardware — see
docs/operations/hardware-bringup.md.
"""

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
    bridge.map_frame = cfg["map_frame"]
    bridge.base_frame = cfg["base_frame"]
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
    bridge._last_cloud_prepare_at = 0.0
    bridge._native_map_frame_warned = False
    bridge.grid = None
    bridge._grid_dirty = False
    bridge._cloud_points = None
    bridge._cloud_dirty = False
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


def test_camera_capability_needs_a_real_topic(mod):
    """An empty topic string must not advertise a camera the GUI will poll."""
    no_cam = _bridge(mod, {"topics": {"camera": "", "camera_compressed": ""}})
    assert "camera" not in no_cam.capabilities()
    raw_only = _bridge(mod, {"topics": {"camera": "image_raw"}})
    assert "camera" in raw_only.capabilities()


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


def test_unaligned_depth_without_optical_tf_is_not_treated_as_aligned(mod):
    """A colour box on an unaligned depth image is the wrong pixels.

    If the depth-to-colour TF is missing, skip the pin rather than sampling
    the depth image as if it were RGB-aligned — that would look confident
    and be in the wrong place.
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


def test_battery_normalisation_handles_percent_and_nan(mod):
    """Drivers disagree about 0..1 vs 0..100, and NaN means 'unknown'."""
    bridge = _bridge(mod)

    bridge._on_battery(type("M", (), {"percentage": 0.82})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": 82.0})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": float("nan")})())
    assert bridge.battery is None, "NaN must not be reported as a real level"

    bridge.cfg["battery_voltage_min"] = 23.0
    bridge.cfg["battery_voltage_max"] = 29.2
    bridge._on_battery(type("M", (), {"battery_voltage": 26.1})())
    assert bridge.battery == pytest.approx(0.5, abs=0.01)

    bridge._on_battery(type("M", (), {"battery_voltage": 23.0})())
    assert bridge.battery == pytest.approx(0.0)

    bridge._on_battery(type("M", (), {"battery_voltage": 30.0})())
    assert bridge.battery == pytest.approx(1.0)


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


def test_nav_cmd_vel_relay_forwards_only_while_navigating(mod):
    """The whole reason this relay exists: pathFollower publishes cmd_vel
    continuously even when idle, so its output must never reach the real
    topic except while this adapter has actually asked it to drive."""
    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    twist = MagicMock()

    bridge.nav_status = "idle"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_not_called()

    bridge.nav_status = "active"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_called_once_with(twist)


def _bearing_of(sent) -> float:
    return math.degrees(math.atan2(sent.axes[2], sent.axes[1]))


def test_nav_joy_is_zero_when_idle_or_no_goal(mod):
    bridge = _bridge(mod, {"topics": {"nav_joy": "joy"}, "nav_joy_throttle": 0.5})
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}

    bridge.nav_status = "idle"
    bridge.goal = {"x": 5.0, "y": 0.0}
    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert sent.axes == [0.0, 0.0, 0.0]

    bridge.nav_status = "active"
    bridge.goal = None
    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert sent.axes == [0.0, 0.0, 0.0]


def test_nav_joy_encodes_the_real_bearing_to_the_goal(mod):
    """The bug this fixes: a constant axes[1]/zero axes[2] always signalled
    "goal straight ahead" to localPlanner's joyDir = atan2(axes[2], axes[1]),
    so a goal placed behind the robot still drove it forward."""
    bridge = _bridge(mod, {"topics": {"nav_joy": "joy"}, "nav_joy_throttle": 0.5})
    bridge.nav_status = "active"

    # Goal straight ahead (body frame == world frame, yaw 0).
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge.goal = {"x": 5.0, "y": 0.0}
    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert _bearing_of(sent) == pytest.approx(0.0, abs=1e-6)
    assert sent.axes[1] > 0  # forward component must be positive when ahead

    # Goal directly behind — this is exactly the case that used to fail.
    bridge.goal = {"x": -5.0, "y": 0.0}
    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert abs(_bearing_of(sent)) == pytest.approx(180.0, abs=1e-6)
    assert sent.axes[1] < 0  # forward component must flip sign when behind

    # Goal to the left, robot facing +x.
    bridge.goal = {"x": 0.0, "y": 5.0}
    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert _bearing_of(sent) == pytest.approx(90.0, abs=1e-6)


def test_nav_joy_bearing_accounts_for_robot_heading(mod):
    """A goal that's "ahead" in world coordinates but the robot is already
    facing away from it must report a bearing near 180 deg, not 0."""
    bridge = _bridge(mod, {"topics": {"nav_joy": "joy"}, "nav_joy_throttle": 0.5})
    bridge.nav_status = "active"
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": math.pi}  # facing -x
    bridge.goal = {"x": 5.0, "y": 0.0}  # world +x, i.e. behind this heading

    bridge._pump_nav_joy()
    sent = bridge.pub_nav_joy.publish.call_args[0][0]
    assert abs(_bearing_of(sent)) == pytest.approx(180.0, abs=1e-4)


def test_nav_joy_is_a_noop_when_unconfigured(mod):
    bridge = _bridge(mod, {"topics": {"nav_joy": ""}})
    bridge.nav_status = "active"
    bridge._pump_nav_joy()  # must not raise with pub_nav_joy None


def test_shipped_configs_are_valid_and_disable_what_they_lack(mod):
    import yaml

    for name in ("generic", "scout_mini"):
        path = REPO / "adapters" / "adapter_ros1" / "config" / f"{name}.yaml"
        cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
        assert cfg["map_frame"] and cfg["base_frame"], f"{name} needs both frames"
        assert cfg["rates"]["state_hz"] > 0
        # Detection poll; never faster than the old 5 Hz preview cap.
        assert cfg["rates"]["camera_period_s"] >= 0.2, f"{name} exceeds the 5 Hz cap"


def test_hardware_configs_declare_physical_map_height_bands():
    import yaml

    config = yaml.safe_load(
        (REPO / "adapters/adapter_ros1/config/scout_mini.yaml").read_text()
    )
    band = config["map_cloud_height_band"]
    assert band["floor_z"] == pytest.approx(-0.575)
    assert band["min_z"] == pytest.approx(0.150)
    assert band["max_z"] == pytest.approx(1.800)
    assert config["lidar_height_m"] == pytest.approx(0.575)


def test_map_cloud_alone_still_advertises_the_map_capability(mod):
    """A robot with no OccupancyGrid publisher, only a registered cloud, must
    still show up as map-capable — the backend builds the grid server-side."""
    bridge = _bridge(mod, {"topics": {"map": "", "map_cloud": "cloud_registered"}})
    assert "map" in bridge.capabilities()

    neither = _bridge(mod, {"topics": {"map": "", "map_cloud": ""}})
    assert "map" not in neither.capabilities()


def test_global_map_cloud_alone_still_advertises_the_map_capability(mod):
    bridge = _bridge(
        mod,
        {"topics": {"map": "", "map_cloud": "", "map_cloud_global": "map_global"}},
    )
    assert "map" in bridge.capabilities()


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
    return type(
        "M",
        (),
        {
            "fields": fields,
            "point_step": point_step,
            "data": data,
        },
    )()


def _fake_cloud_with_header(points, frame="odom_lidar", stamp=0.0):
    msg = _fake_cloud(points)
    msg.header = type("Header", (), {"frame_id": frame, "stamp": _stamp(stamp)})()
    return msg


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


def test_on_map_cloud_keeps_xyz_for_the_3d_view(mod):
    """The 2D obstacle band must not flatten or filter the display cloud."""
    bridge = _bridge(mod, {"map_cloud_height_band": {"min_z": 0.0, "max_z": 1.0}})
    # First two returns share a 10 cm display voxel; the ceiling return is
    # outside the 2D band but must remain visible in 3D.
    msg = _fake_cloud(
        [
            (1.000, 2.000, 0.02),
            (1.001, 2.001, 0.021),
            (3.000, 4.000, 2.50),
        ]
    )
    bridge._on_map_cloud(msg)

    assert bridge._cloud_points.shape == (2, 3)
    assert bridge._cloud_points[:, 2].tolist() == pytest.approx([0.02, 2.5])
    assert bridge._cloud_dirty is True
    assert bridge._scan_points.shape == (1, 2)


def test_global_map_cloud_projects_only_the_configured_height_slice(mod):
    bridge = _bridge(
        mod,
        {
            "map_frame": "odom_lidar",
            "topics": {"map": "", "map_cloud": "", "map_cloud_global": "map_global"},
            "map_cloud_height_band": {"min_z": 0.0, "max_z": 1.0},
            "native_map_resolution": 0.5,
            "native_map_padding_m": 0.5,
        },
    )
    bridge._on_global_map_cloud(
        _fake_cloud_with_header(
            [
                (1.0, 2.0, 0.5),  # retained occupied return
                (1.1, 2.1, 0.6),  # same 0.5 m cell
                (4.0, 4.0, 2.0),  # above the requested band
                (8.0, 8.0, -1.0),  # below the requested band
            ]
        )
    )

    assert bridge._scan_dirty is False
    assert bridge._grid_dirty is True
    assert bridge.grid.header.frame_id == "odom_lidar"
    assert bridge.grid.info.resolution == pytest.approx(0.5)
    assert bridge.grid.info.origin.position.x == pytest.approx(0.5)
    assert bridge.grid.info.origin.position.y == pytest.approx(1.5)
    cells = mod.np.asarray(bridge.grid.data, dtype=mod.np.int8).reshape(
        bridge.grid.info.height, bridge.grid.info.width
    )
    assert int((cells == 100).sum()) == 1
    assert (cells == -1).sum() > 0


def test_global_map_cloud_drops_a_cloud_in_the_wrong_frame(mod):
    bridge = _bridge(
        mod,
        {"map_frame": "odom_lidar", "topics": {"map_cloud_global": "map_global"}},
    )
    bridge._on_global_map_cloud(_fake_cloud_with_header([(1.0, 2.0, 0.5)], frame="map"))
    assert bridge.grid is None
    assert bridge._grid_dirty is False


def test_3d_downsampling_is_limited_to_the_upload_rate(mod, monkeypatch):
    bridge = _bridge(mod, {"rates": {"cloud_period_s": 4.0}})
    now = [100.0]
    monkeypatch.setattr(mod.time, "monotonic", lambda: now[0])

    bridge._on_map_cloud(_fake_cloud([(1.0, 2.0, 3.0)]))
    assert bridge._cloud_points[0].tolist() == pytest.approx([1.0, 2.0, 3.0])

    # The 2D scan still updates at lidar rate, but this second cloud is too soon
    # to spend another full XYZ voxelisation for a 0.25 Hz display upload.
    now[0] = 101.0
    bridge._on_map_cloud(_fake_cloud([(4.0, 5.0, 6.0)]))
    assert bridge._cloud_points[0].tolist() == pytest.approx([1.0, 2.0, 3.0])

    now[0] = 104.0
    bridge._on_map_cloud(_fake_cloud([(4.0, 5.0, 6.0)]))
    assert bridge._cloud_points[0].tolist() == pytest.approx([4.0, 5.0, 6.0])


def test_upload_cloud_uses_the_shared_xyz_transport(mod, monkeypatch):
    bridge = _bridge(mod)
    bridge.http_url = "http://backend"
    bridge._cloud_points = mod.np.array(
        [[1.25, -2.5, 0.75], [3.0, 4.0, 5.0]], dtype=mod.np.float32
    )
    bridge._cloud_dirty = True
    captured = {}

    class Response:
        def read(self):
            return b"{}"

    def urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)
    bridge.upload_cloud()

    request = captured["request"]
    assert request.full_url == (
        "http://backend/api/adapter/cloud?robot_id=r0&scale=0.01"
    )
    decoded = mod.np.frombuffer(mod.zlib.decompress(request.data), dtype=mod.np.int16)
    assert decoded.reshape(-1, 3).tolist() == [[125, -250, 75], [300, 400, 500]]
    assert bridge._cloud_dirty is False


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


def _plan_bridge(mod, frame="odom_lidar"):
    bridge = _bridge(mod, {"map_frame": frame, "topics": {"plan": "/path"}})
    bridge.planned_path = []
    bridge._plan_frame_warned = False
    bridge.tf_buffer = MagicMock()
    return bridge


def test_plan_in_map_frame_is_passed_through(mod):
    """Nav2's global plan already arrives in map_frame and must not be moved."""
    bridge = _plan_bridge(mod)
    bridge._on_plan(_plan_msg("odom_lidar", [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]))

    assert bridge.planned_path == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
    bridge.tf_buffer.lookup_transform.assert_not_called()


def test_plan_in_a_vehicle_frame_is_transformed_into_map_frame(mod):
    """TARS's local_planner publishes /path in chassis_link, not the map frame.

    Copying those numbers through drew the route as though the robot were parked
    at the map origin facing +x. Here the robot sits at (10, 5) yawed 90 degrees,
    so a path 2 m straight ahead of the vehicle belongs at (10, 7) on the map.
    """
    bridge = _plan_bridge(mod)
    transform = MagicMock()
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = math.sin(math.pi / 4)  # yaw = +90 deg
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

    bridge._on_plan(_plan_msg("chassis_link", []))

    assert bridge.planned_path == []


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


def test_nav_relay_refuses_to_drive_while_the_link_is_stale(mod):
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


# --------------------------------------------------------------- link plumbing


def test_a_blocking_upload_does_not_stall_state_or_nav_joy(mod, monkeypatch):
    """The four-robot regression, in its ROS 1 form.

    `upload_map` blocks for up to its 5 s timeout, and it blocks for that long
    precisely when the fleet is large — every robot's map queues behind one lock
    on the server. While state and uploads shared a coroutine, that stalled the
    5 Hz pump, and the backend declares a robot offline after 4 s
    (OFFLINE_AFTER_S) while `link_watchdog` cancels its active goal after 1.5 s.

    `_pump_nav_joy` makes it worse here than on ROS 2: pathFollower reads its
    speed and localPlanner its steering direction from that message, so a
    stalled loop did not merely stop REPORTING the robot, it stopped STEERING a
    robot that was mid-goal.
    """
    import asyncio
    import json
    import threading

    class _FakeWs:
        def __init__(self):
            self.sent = []

        async def send(self, raw):
            self.sent.append(json.loads(raw))

        def __aiter__(self):
            return self

        async def __anext__(self):
            await asyncio.sleep(3600)  # the operator sends nothing here
            raise StopAsyncIteration

    ws = _FakeWs()

    class _Conn:
        async def __aenter__(self):
            return ws

        async def __aexit__(self, *_exc):
            return False

    monkeypatch.setattr(mod.websockets, "connect", lambda *a, **k: _Conn())

    started = threading.Event()
    release = threading.Event()
    joy_pumps = []

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

        def _check_topic_nav_progress(self):
            pass

        def _pump_nav_joy(self):
            joy_pumps.append(1)

        def session_state_tick(self):
            self._check_topic_nav_progress()
            self._pump_nav_joy()

        def upload_map(self):
            started.set()
            release.wait(5.0)  # stands in for the real 5 s urllib timeout
            return None

        def upload_scan(self):
            pass

        def upload_cloud(self):
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

    bridge = _Bridge()

    def count_states():
        return sum(1 for m in ws.sent if m.get("type") == "robot_state")

    async def scenario():
        task = asyncio.ensure_future(mod.run_robot(bridge, "ws://test"))
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, started.wait, 5.0)
        before, joy_before = count_states(), len(joy_pumps)
        await asyncio.sleep(0.4)
        during, joy_during = count_states(), len(joy_pumps)
        release.set()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return before, during, joy_before, joy_during

    before, during, joy_before, joy_during = asyncio.run(
        asyncio.wait_for(scenario(), 15)
    )

    assert started.is_set(), "the scenario never reached the blocking upload"
    # 0.4 s at 50 Hz is ~20 iterations; a handful is already proof the loop ran.
    assert during - before >= 5, (
        f"state stopped while an upload was in flight ({during - before} frames "
        "sent during a 0.4 s block) — uploads are gating telemetry again"
    )
    assert joy_during - joy_before >= 5, (
        f"nav joy stopped while an upload was in flight ({joy_during - joy_before} "
        "pumps during a 0.4 s block) — an upload can steer the robot again"
    )


def test_websocket_keepalive_is_tight_enough_to_matter(mod):
    """See the matching test in test_adapter_ros2.py.

    The library defaults (20 s interval, 20 s timeout) leave ~40 s in which
    `link_ok()` reads true off our own completing sends, while pathFollower's
    ~27 Hz stream is relayed to the driver with no operator attached.
    """
    cfg = mod.deep_merge(mod.DEFAULTS, {})

    interval = float(cfg["ping_interval_s"])
    timeout = float(cfg["ping_timeout_s"])

    assert (
        interval + timeout <= 8.0
    ), f"link loss would go undetected for up to {interval + timeout:.0f}s"
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

    monkeypatch.setattr(
        mod.websockets, "connect", lambda url, **kw: (seen.update(kw), _Conn())[1]
    )

    bridge = _bridge(mod)

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
