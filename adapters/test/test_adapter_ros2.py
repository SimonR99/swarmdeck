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


def test_shipped_configs_are_valid_and_disable_what_they_lack(mod):
    import yaml

    for name in ("generic", "duckiebot"):
        path = REPO / "adapters" / "adapter_ros2" / "config" / f"{name}.yaml"
        cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
        assert cfg["map_frame"] and cfg["base_frame"], f"{name} needs both frames"
        assert cfg["rates"]["state_hz"] > 0
        # The preview cap in the protocol is 5 Hz; never configure faster.
        assert cfg["rates"]["camera_period_s"] >= 0.2, f"{name} exceeds the 5 Hz cap"
