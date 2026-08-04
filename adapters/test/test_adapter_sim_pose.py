"""The pose adapter_sim reports must come from the TF chain, not the odom topic.

Regression test for a defect that put every robot's marker in the wrong place on
the GUI map. `map_frame -> odom` is published by SLAM, `odom -> base_link` by the
EKF; `/<ns>/odom` separately carries the drive plugin's raw wheel integration.
Composing SLAM's correction with the wheel topic mixes two chains, and the result
is wrong by however far wheel odometry has diverged from the filter — measured
live at 0.18-0.48 m per robot, and unbounded when a jammed drive spins its wheels.

adapter_sim imports the whole ROS stack at module scope, so this stubs the
imports rather than requiring a sourced workspace: the logic under test is pure
SE(2) arithmetic and deserves to run in `make test`.
"""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "cv2", "websockets", "rclpy", "rclpy.action", "rclpy.node", "rclpy.qos",
    "action_msgs", "action_msgs.msg", "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg", "nav2_msgs", "nav2_msgs.action",
    "sensor_msgs", "sensor_msgs.msg", "tf2_msgs", "tf2_msgs.msg",
    "std_msgs", "std_msgs.msg",
]


@pytest.fixture(scope="module")
def bridge_cls():
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    for name in _STUBBED:
        sys.modules[name] = MagicMock()
    sys.path.insert(0, str(REPO / "adapters" / "adapter_sim"))
    try:
        import importlib

        module = importlib.import_module("adapter_sim")
        yield module.RobotBridge
    finally:
        sys.path.remove(str(REPO / "adapters" / "adapter_sim"))
        sys.modules.pop("adapter_sim", None)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def make_bridge(bridge_cls):
    """A bridge with its subscriptions stubbed out."""
    bridge = bridge_cls.__new__(bridge_cls)
    bridge.node = MagicMock()
    bridge.id = "robot_0"
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = None
    bridge._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
    return bridge


def tf_message(pairs):
    """A TFMessage-shaped stub: [(parent, child, x, y, yaw), ...]."""
    transforms = []
    for parent, child, x, y, yaw in pairs:
        stamped = types.SimpleNamespace()
        stamped.header = types.SimpleNamespace(frame_id=parent)
        stamped.child_frame_id = child
        stamped.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y, z=0.0),
            rotation=types.SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
            ),
        )
        transforms.append(stamped)
    return types.SimpleNamespace(transforms=transforms)


def test_tf_base_link_wins_over_the_wheel_odometry_topic(bridge_cls):
    """The exact defect: both sources present and disagreeing."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(tf_message([
        ("robot_0/map_frame", "robot_0/odom", 0.06, -0.12, 0.0),
        ("robot_0/odom", "robot_0/base_link", 10.56, 0.18, 0.0),
    ]))
    # The wheel topic says something 0.47 m away, as measured on a live run.
    bridge._on_odom(types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=10.49, y=0.65, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))
    ))
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(10.62)
    assert pose["y"] == pytest.approx(0.06)


def test_composition_applies_the_map_frame_rotation(bridge_cls):
    """map->odom is a full SE(2) transform; a yaw correction must rotate the
    translation, not just add to the heading."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(tf_message([
        ("robot_0/map_frame", "robot_0/odom", 1.0, 2.0, math.pi / 2),
        ("robot_0/odom", "robot_0/base_link", 3.0, 0.0, 0.0),
    ]))
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(1.0)
    assert pose["y"] == pytest.approx(5.0)
    assert pose["yaw"] == pytest.approx(math.pi / 2)


def test_yaw_stays_wrapped(bridge_cls):
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(tf_message([
        ("robot_0/map_frame", "robot_0/odom", 0.0, 0.0, 3.0),
        ("robot_0/odom", "robot_0/base_link", 0.0, 0.0, 3.0),
    ]))
    assert -math.pi <= bridge.map_pose()["yaw"] <= math.pi


def test_falls_back_to_wheel_odometry_but_says_so(bridge_cls):
    """Reporting the map origin forever is worse than reporting a drifting pose,
    so the fallback exists — but it must not be silent."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(tf_message([
        ("robot_0/map_frame", "robot_0/odom", 0.5, 0.0, 0.0),
    ]))
    bridge._on_odom(types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=2.0, y=1.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))
    ))
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(2.5)
    assert pose["y"] == pytest.approx(1.0)
    assert bridge.node.get_logger.return_value.warn.called


def test_transforms_for_other_robots_are_ignored(bridge_cls):
    """One /tf topic per robot, but a shared graph is a normal deployment and a
    neighbour's transform must never be mistaken for this robot's."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(tf_message([
        ("robot_1/map_frame", "robot_1/odom", 99.0, 99.0, 1.0),
        ("robot_1/odom", "robot_1/base_link", 99.0, 99.0, 1.0),
    ]))
    assert bridge._odom_to_base is None
    assert bridge._map_to_odom == {"x": 0.0, "y": 0.0, "yaw": 0.0}
