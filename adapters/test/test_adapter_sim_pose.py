"""Simulation pose reporting follows the robot's odom navigation frame."""

from __future__ import annotations

import math
import types
from unittest.mock import MagicMock

import pytest


@pytest.fixture(scope="module")
def bridge_cls(sim_module):
    return sim_module.RobotBridge


def make_bridge(bridge_cls):
    bridge = bridge_cls.__new__(bridge_cls)
    bridge.node = MagicMock()
    bridge.id = "robot_0"
    bridge.navigation_frame = "robot_0/odom"
    bridge._odom_to_base = None
    bridge._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
    return bridge


def tf_message(pairs):
    transforms = []
    for parent, child, x, y, yaw in pairs:
        transforms.append(
            types.SimpleNamespace(
                header=types.SimpleNamespace(frame_id=parent),
                child_frame_id=child,
                transform=types.SimpleNamespace(
                    translation=types.SimpleNamespace(x=x, y=y, z=0.0),
                    rotation=types.SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw / 2),
                        w=math.cos(yaw / 2),
                    ),
                ),
            )
        )
    return types.SimpleNamespace(transforms=transforms)


def test_navigation_frame_is_the_robot_odom_frame(bridge_cls):
    bridge = make_bridge(bridge_cls)
    assert bridge.navigation_frame == "robot_0/odom"


def test_tf_base_link_is_preferred_over_the_wheel_odometry_topic(bridge_cls):
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [("robot_0/map", "robot_0/odom", 99.0, 99.0, 0.0),
             ("robot_0/odom", "robot_0/base_link", 10.56, 0.18, 0.0)]
        )
    )
    bridge._on_odom(
        types.SimpleNamespace(
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=10.49, y=0.65, z=0.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
    )

    assert bridge.map_pose() == {"x": pytest.approx(10.56), "y": pytest.approx(0.18), "yaw": 0.0}


def test_wheel_odometry_is_a_loud_fallback_when_tf_is_missing(bridge_cls):
    bridge = make_bridge(bridge_cls)
    bridge._on_odom(
        types.SimpleNamespace(
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=2.0, y=1.0, z=0.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
    )

    assert bridge.map_pose() == {"x": 2.0, "y": 1.0, "yaw": 0.0}
    bridge.node.get_logger.return_value.warn.assert_called_once()


def test_transforms_for_other_robots_are_ignored(bridge_cls):
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [("robot_1/odom", "robot_1/base_link", 99.0, 99.0, 1.0)]
        )
    )

    assert bridge._odom_to_base is None
    assert bridge.map_pose() == {"x": 0.0, "y": 0.0, "yaw": 0.0}
