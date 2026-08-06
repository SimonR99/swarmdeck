"""Hardware adapter tests, with ROS stubbed out.

`adapter_ros2` imports the whole ROS stack at module scope, so this stubs it the
same way `test_adapter_sim_pose.py` does. What is testable without a robot is
exactly the part most likely to be wrong on one: capability advertisement,
config merging, battery normalisation, and the deadman.

What these CANNOT test is whether the topic names, QoS choices and frame names
are right for any particular robot. That needs hardware — see
docs/hardware-bringup.md.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "rclpy", "rclpy.action", "rclpy.node", "rclpy.qos", "rclpy.time",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "nav2_msgs", "nav2_msgs.action",
    "sensor_msgs", "sensor_msgs.msg",
    "action_msgs", "action_msgs.msg",
    "tf2_ros", "websockets", "cv2",
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
    bridge.node = MagicMock()
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.nav_client = MagicMock() if cfg.get("actions", {}).get("navigate_to_pose") else None
    bridge.mode = "idle"
    bridge._last_drive_at = 0.0
    bridge._scan_points = None
    bridge._scan_dirty = False
    bridge._cloud_points = None
    bridge._cloud_dirty = False
    bridge._last_cloud_prepare_at = 0.0
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
        # The preview cap in the protocol is 5 Hz; never configure faster.
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


def _fake_cloud(points, extra_fields=()):
    import struct

    names = list(extra_fields) + ["x", "y", "z"]
    fields = [_FakeField(name, index * 4) for index, name in enumerate(names)]
    point_step = len(names) * 4
    data = b"".join(
        struct.pack("<%df" % len(names), *([0.0] * len(extra_fields)), *point)
        for point in points
    )
    return type("M", (), {"fields": fields, "point_step": point_step, "data": data})()


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
