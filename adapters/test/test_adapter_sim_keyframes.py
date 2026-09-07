"""adapter_sim produces keyframes from a lidar scan the same way hardware does."""

from __future__ import annotations

import math
import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from adapters.keyframe_producer import KeyframeUploader


def test_a_room_scan_enqueues_a_keyframe(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
    bridge._map_to_odom_log = deque([(100.0, bridge._map_to_odom)])
    bridge._odom_to_base_log = deque(
        [(100.0, bridge._odom_to_base)] if bridge._odom_to_base is not None else []
    )
    bridge.pose_lookup_gap = bridge.pose_lookup_age = 0.0
    bridge._pose_lookup_misses = 0
    bridge.lidar_x = 0.0
    bridge.lidar_z = 0.45
    bridge._scan_cloud_at = 0.0
    bridge._keyframes = KeyframeUploader(
        "robot_0", "http://backend", min_period_s=0.0, min_points=20
    )

    n = 180
    ranges = np.full(n, 4.0)
    # A corner, so GICP/Scan Context have structure rather than a circle.
    ranges[: n // 4] = 2.0
    scan = MagicMock()
    scan.ranges = ranges.tolist()
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / n
    scan.range_min = 0.1
    scan.range_max = 30.0
    scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0))

    bridge._on_scan(scan)
    assert bridge._keyframes.pending() == 1


def test_scan_cloud_and_keyframe_pose_are_sampled_atomically(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._odom_to_base = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    bridge.lidar_x = 0.0
    bridge.lidar_z = 0.45
    bridge._scan_cloud_at = 0.0
    bridge._keyframes = KeyframeUploader(
        "robot_0", "http://backend", min_period_s=0.0, min_points=20
    )
    bridge.map_pose_at = MagicMock(return_value={"x": 1.0, "y": 2.0, "yaw": 0.3})

    scan = MagicMock()
    scan.ranges = [2.0] * 90
    scan.angle_min = -1.0
    scan.angle_increment = 0.02
    scan.range_min = 0.1
    scan.range_max = 30.0
    scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0))

    bridge._on_scan(scan)

    assert bridge._keyframes.pending() == 1
    bridge.map_pose_at.assert_called_once_with(100.0, require_history=True)


def test_keyframes_wait_for_tf_before_using_wheel_odometry(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = None
    bridge._odom_topic_pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
    bridge._map_to_odom_log = deque([(100.0, bridge._map_to_odom)])
    bridge._odom_to_base_log = deque(
        [(100.0, bridge._odom_to_base)] if bridge._odom_to_base is not None else []
    )
    bridge.pose_lookup_gap = bridge.pose_lookup_age = 0.0
    bridge._pose_lookup_misses = 0
    bridge.lidar_x = 0.0
    bridge.lidar_z = 0.45
    bridge._scan_cloud_at = 0.0
    bridge._keyframes = KeyframeUploader(
        "robot_0", "http://backend", min_period_s=0.0, min_points=20
    )
    n = 180
    ranges = np.full(n, 4.0)
    ranges[: n // 4] = 2.0
    scan = MagicMock()
    scan.ranges = ranges.tolist()
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / n
    scan.range_min = 0.1
    scan.range_max = 30.0
    scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0))
    bridge._on_scan(scan)
    assert bridge._keyframes.pending() == 0


def test_a_live_3d_cloud_suppresses_the_planar_fallback(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
    bridge._map_to_odom_log = deque([(100.0, bridge._map_to_odom)])
    bridge._odom_to_base_log = deque(
        [(100.0, bridge._odom_to_base)] if bridge._odom_to_base is not None else []
    )
    bridge.pose_lookup_gap = bridge.pose_lookup_age = 0.0
    bridge._pose_lookup_misses = 0
    bridge.lidar_x = 0.0
    bridge.lidar_z = 0.45
    bridge._scan_cloud_at = time.monotonic()
    bridge._keyframes = KeyframeUploader("robot_0", "http://backend", min_period_s=0.0)

    scan = MagicMock()
    scan.ranges = [2.0] * 90
    scan.angle_min = -1.0
    scan.angle_increment = 0.02
    scan.range_min = 0.1
    scan.range_max = 30.0
    scan.header = SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0))
    bridge._on_scan(scan)
    assert bridge._keyframes.pending() == 0


@pytest.mark.parametrize("cloud", [False, True])
def test_wire_keyframe_keeps_capture_yaw_for_both_scan_formats(
    sim_module, monkeypatch, cloud
):
    from swarmdeck_protocol import decode_keyframe

    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 2.0, "y": 3.0, "yaw": 0.2}
    bridge._odom_to_base = {"x": 0.0, "y": 0.0, "yaw": 1.0}
    bridge._map_to_odom_log = deque([(100.0, bridge._map_to_odom)])
    bridge._odom_to_base_log = deque(
        [
            (100.0, {"x": 0.0, "y": 0.0, "yaw": 0.0}),
            (100.1, bridge._odom_to_base),
        ]
    )
    bridge.pose_lookup_gap = bridge.pose_lookup_age = 0.0
    bridge._pose_lookup_misses = 0
    bridge._scan_cloud_at = 0.0
    bridge.lidar_x = 0.15
    bridge.lidar_z = 0.45
    bridge._keyframes = KeyframeUploader("robot_0", "http://unused", min_points=20)
    angles = np.arange(90) * 0.02 - 1
    raw = np.column_stack(
        [2 * np.cos(angles), 2 * np.sin(angles), np.zeros(90)]
    ).astype(np.float32)
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=50_000_000)),
        ranges=[2.0] * 90,
        angle_min=-1.0,
        angle_increment=0.02,
        range_min=0.1,
        range_max=30.0,
    )
    monkeypatch.setattr(sim_module, "cloud_xyz", lambda _: raw)
    (bridge._on_scan_cloud if cloud else bridge._on_scan)(msg)
    packet = decode_keyframe(bridge._keyframes._queue[0])
    # Capture yaw 0.5 + map correction 0.2; arrival yaw would be 1.2.
    np.testing.assert_allclose(
        packet.t_odom_base[5:], [math.sin(0.35), math.cos(0.35)], atol=1e-6
    )
    assert packet.stamp == 100.05
    # Wire geometry stays in base coordinates; no yaw is applied twice.
    expected = raw + [bridge.lidar_x, 0, bridge.lidar_z]
    if not cloud:
        expected = np.vstack([expected + [0, 0, dz] for dz in (0, 0.12, 0.24)])
    distance = np.linalg.norm(packet.points[:, None] - expected[None], axis=2).min(
        axis=1
    )
    # XYZ is quantized to 1 cm on the wire.
    assert distance.max() <= math.sqrt(3) * 0.005 + 1e-6
    # A stale scan cannot reuse the arrival pose and add a second map copy.
    msg.header.stamp.sec = 5
    (bridge._on_scan_cloud if cloud else bridge._on_scan)(msg)
    assert bridge._keyframes.pending() == 1
    assert bridge._pose_lookup_rejected == 1
