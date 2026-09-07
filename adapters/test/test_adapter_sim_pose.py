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
from collections import deque
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def bridge_cls(sim_module):
    # The stub list lives in conftest.py: adapter_sim's ROS imports are a shared
    # hazard across every test module that has to fake them.
    return sim_module.RobotBridge


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
    bridge._on_tf(
        tf_message(
            [
                ("robot_0/map_frame", "robot_0/odom", 0.06, -0.12, 0.0),
                ("robot_0/odom", "robot_0/base_link", 10.56, 0.18, 0.0),
            ]
        )
    )
    # The wheel topic says something 0.47 m away, as measured on a live run.
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
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(10.62)
    assert pose["y"] == pytest.approx(0.06)


def test_composition_applies_the_map_frame_rotation(bridge_cls):
    """map->odom is a full SE(2) transform; a yaw correction must rotate the
    translation, not just add to the heading."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [
                ("robot_0/map_frame", "robot_0/odom", 1.0, 2.0, math.pi / 2),
                ("robot_0/odom", "robot_0/base_link", 3.0, 0.0, 0.0),
            ]
        )
    )
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(1.0)
    assert pose["y"] == pytest.approx(5.0)
    assert pose["yaw"] == pytest.approx(math.pi / 2)


def test_yaw_stays_wrapped(bridge_cls):
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [
                ("robot_0/map_frame", "robot_0/odom", 0.0, 0.0, 3.0),
                ("robot_0/odom", "robot_0/base_link", 0.0, 0.0, 3.0),
            ]
        )
    )
    assert -math.pi <= bridge.map_pose()["yaw"] <= math.pi


def test_falls_back_to_wheel_odometry_but_says_so(bridge_cls):
    """Reporting the map origin forever is worse than reporting a drifting pose,
    so the fallback exists — but it must not be silent."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [
                ("robot_0/map_frame", "robot_0/odom", 0.5, 0.0, 0.0),
            ]
        )
    )
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
    pose = bridge.map_pose()
    assert pose["x"] == pytest.approx(2.5)
    assert pose["y"] == pytest.approx(1.0)
    assert bridge.node.get_logger.return_value.warn.called


def test_transforms_for_other_robots_are_ignored(bridge_cls):
    """One /tf topic per robot, but a shared graph is a normal deployment and a
    neighbour's transform must never be mistaken for this robot's."""
    bridge = make_bridge(bridge_cls)
    bridge._on_tf(
        tf_message(
            [
                ("robot_1/map_frame", "robot_1/odom", 99.0, 99.0, 1.0),
                ("robot_1/odom", "robot_1/base_link", 99.0, 99.0, 1.0),
            ]
        )
    )
    assert bridge._odom_to_base is None
    assert bridge._map_to_odom == {"x": 0.0, "y": 0.0, "yaw": 0.0}


def stamped_tf(pairs, stamp):
    """`tf_message`, but with a ROS 2 stamp on every transform."""
    msg = tf_message(pairs)
    for transform in msg.transforms:
        transform.header.stamp = types.SimpleNamespace(
            sec=int(stamp), nanosec=int(round((stamp - int(stamp)) * 1e9))
        )
    return msg


def _turning_bridge(bridge_cls):
    """A robot spinning in place at 1.0 rad/s, sampled every 100 ms."""
    bridge = make_bridge(bridge_cls)
    bridge._map_to_odom_log = deque(maxlen=128)
    bridge._odom_to_base_log = deque(maxlen=128)
    bridge.pose_lookup_gap = {"odom_base": 0.0, "map_odom": 0.0}
    bridge.pose_lookup_empty = {"odom_base": 0, "map_odom": 0}
    bridge.pose_lookup_stale = {"odom_base": 0, "map_odom": 0}
    for i in range(11):
        t = 100.0 + i * 0.1
        bridge._on_tf(
            stamped_tf(
                [
                    ("robot_0/map_frame", "robot_0/odom", 0.0, 0.0, 0.0),
                    ("robot_0/odom", "robot_0/base_link", 0.0, 0.0, i * 0.1),
                ],
                t,
            )
        )
    return bridge


def test_scan_is_posed_at_its_own_stamp_not_the_newest_one(bridge_cls):
    """A scan taken mid-turn must not be registered at the pose it arrives at.

    The bridge hands over a lidar frame that can be up to one lidar period old,
    so pairing it with the newest TF rotates every return about the robot by
    the yaw accrued in between. That is what puts a rigidly rotated copy of the
    building into the merged map, and only ever while turning.
    """
    bridge = _turning_bridge(bridge_cls)

    # Newest reading is yaw 1.0 rad; the scan was taken 0.5 s earlier at 0.5.
    assert bridge.map_pose()["yaw"] == pytest.approx(1.0, abs=1e-6)
    assert bridge.map_pose_at(100.5)["yaw"] == pytest.approx(0.5, abs=1e-6)


def test_pose_lookup_falls_back_rather_than_reaching_for_a_distant_sample(bridge_cls):
    """Outside the history the newest reading is the honest answer."""
    bridge = _turning_bridge(bridge_cls)

    # No stamp at all, and a stamp far outside the retained window.
    assert bridge.map_pose_at(None)["yaw"] == pytest.approx(1.0, abs=1e-6)
    assert bridge.map_pose_at(5.0)["yaw"] == pytest.approx(1.0, abs=1e-6)


def test_pairing_diagnostics_record_how_far_the_lookup_reached(bridge_cls):
    """The instrument that resolves the arithmetic, so it cannot go unnoticed.

    The turn gate caps accepted captures at 8 deg/s and the simulator's capture
    lag is a constant 100 ms, which bounds the pose error at 0.8 deg. Keyframes
    were measured 5 deg out. Either the gate, the lag, or the pairing is not
    what it claims, and only the pairing was unmeasured.
    """
    bridge = _turning_bridge(bridge_cls)

    # A stamp on the sample grid: odom->base must land exactly. That link is
    # the one that moves during a turn, so any gap there is a real mismatch.
    bridge.map_pose_at(100.5)
    assert bridge.pose_lookup_gap["odom_base"] == pytest.approx(0.0, abs=1e-9)
    assert bridge.pose_lookup_stale["odom_base"] == 0

    # A stamp far outside the history: falls back to the newest, and says so
    # rather than passing off a distant sample as the pose at that instant.
    bridge.map_pose_at(5.0)
    assert bridge.pose_lookup_stale["odom_base"] > 0

    # An empty history is a different failure and is counted separately: it
    # means the pairing never happened at all, silently, which is what a robot
    # whose map->odom is missing looks like.
    bridge._odom_to_base_log.clear()
    bridge.map_pose_at(100.5)
    assert bridge.pose_lookup_empty["odom_base"] > 0


def test_pose_between_samples_is_interpolated_not_snapped(bridge_cls):
    """A stamp falling between TF samples must not take the nearer one whole.

    Snapping makes the pose error the distance to the chosen sample times the
    turn rate. Measured live, lookups reached 100 to 500 ms past the stamp they
    asked for and the rotated copy in the merged map tracked that reach: 300 ms
    gave 4.0 deg, 200 ms gave 3.0 deg, 100 ms gave 2.5 deg. Interpolation
    leaves only the curvature over the interval, which for a steady turn is
    nothing.
    """
    bridge = _turning_bridge(bridge_cls)  # 1.0 rad/s, sampled every 100 ms

    # 100.55 is squarely between the 100.5 and 100.6 samples.
    yaw = bridge.map_pose_at(100.55)["yaw"]
    assert yaw == pytest.approx(0.55, abs=1e-6)
    # Snapping would have returned one of the endpoints.
    assert abs(yaw - 0.5) > 1e-3 and abs(yaw - 0.6) > 1e-3


def test_interpolation_survives_a_hole_in_the_history(bridge_cls):
    """A dropped sample is the case this exists for, so cover it explicitly."""
    bridge = _turning_bridge(bridge_cls)
    # Drop the samples around 100.4-100.6, as a starved executor would.
    kept = [s for s in bridge._odom_to_base_log if not (100.35 < s[0] < 100.65)]
    bridge._odom_to_base_log.clear()
    bridge._odom_to_base_log.extend(kept)

    # Still linear across the hole, because the bracket is still there.
    assert bridge.map_pose_at(100.5)["yaw"] == pytest.approx(0.5, abs=1e-6)


def test_yaw_interpolation_takes_the_short_way_round(bridge_cls):
    """Wrapping must not send the pose the long way through 2 pi."""
    bridge = make_bridge(bridge_cls)
    bridge._map_to_odom_log = deque(maxlen=128)
    bridge._odom_to_base_log = deque(maxlen=128)
    bridge.pose_lookup_gap = {"odom_base": 0.0, "map_odom": 0.0}
    bridge.pose_lookup_empty = {"odom_base": 0, "map_odom": 0}
    bridge.pose_lookup_stale = {"odom_base": 0, "map_odom": 0}
    for t, yaw in ((100.0, math.pi - 0.1), (100.1, -math.pi + 0.1)):
        bridge._on_tf(
            stamped_tf(
                [
                    ("robot_0/map_frame", "robot_0/odom", 0.0, 0.0, 0.0),
                    ("robot_0/odom", "robot_0/base_link", 0.0, 0.0, yaw),
                ],
                t,
            )
        )
    yaw = bridge.map_pose_at(100.05)["yaw"]
    assert abs(abs(yaw) - math.pi) < 1e-6, f"went the long way: {yaw}"
