"""adapter_sim produces keyframes from a lidar scan the same way hardware does."""

from __future__ import annotations

import math
import time
from unittest.mock import MagicMock

import numpy as np

from adapters.keyframe_producer import KeyframeUploader


def test_a_room_scan_enqueues_a_keyframe(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
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
    scan.header = MagicMock()

    bridge._on_scan(scan)
    assert bridge._keyframes.pending() == 1


def test_keyframes_wait_for_tf_before_using_wheel_odometry(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.node = MagicMock()
    bridge._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._odom_to_base = None
    bridge._odom_topic_pose = {"x": 1.0, "y": 0.0, "yaw": 0.0}
    bridge._warned_no_tf_base = False
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
    scan.header = MagicMock()
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
    scan.header = MagicMock()
    bridge._on_scan(scan)
    assert bridge._keyframes.pending() == 0
