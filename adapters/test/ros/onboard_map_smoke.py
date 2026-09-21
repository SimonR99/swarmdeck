"""Real ROS 2 smoke for callback-driven onboard map publication.

This creates only OccupancyGrid endpoints on an isolated ROS domain. It does
not start a websocket, Nav2 action, simulator, or velocity publisher.
"""

from __future__ import annotations

import sys
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "adapters" / "adapter_sim"))

from adapter_sim import RobotBridge  # noqa: E402


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("swarmdeck_onboard_map_probe")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received = []
    bridge = RobotBridge.__new__(RobotBridge)
    bridge.id = "probe"
    bridge.node = node
    bridge.navigation_frame = "probe/navigation_frame"
    bridge.onboard_mapping = True
    bridge._onboard_map_warned_at = 0.0
    bridge.pub_local_costmap = node.create_publisher(
        OccupancyGrid, "/probe/local_costmap", qos
    )
    node.create_subscription(OccupancyGrid, "/probe/map", bridge._on_map, qos)
    node.create_subscription(
        OccupancyGrid, "/probe/local_costmap", received.append, qos
    )
    source = node.create_publisher(OccupancyGrid, "/probe/map", qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_for(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    try:
        wrong = OccupancyGrid()
        wrong.header.frame_id = "other/map"
        wrong.info.resolution = 0.25
        wrong.info.width = wrong.info.height = 1
        wrong.data = [100]
        source.publish(wrong)
        spin_for(0.4)
        assert not received, "a grid from another frame reached Nav2"

        aligned = OccupancyGrid()
        aligned.header.frame_id = bridge.navigation_frame
        aligned.header.stamp.sec = 12
        aligned.header.stamp.nanosec = 34
        aligned.info.resolution = 0.2
        aligned.info.width = 2
        aligned.info.height = 1
        aligned.info.origin.position.x = -1.5
        aligned.info.origin.orientation.w = 1.0
        aligned.data = [-1, 100]
        source.publish(aligned)
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received, "aligned onboard grid was not republished"
        forwarded = received[-1]
        assert forwarded.header.frame_id == "probe/navigation_frame"
        assert (forwarded.header.stamp.sec, forwarded.header.stamp.nanosec) == (
            12,
            34,
        )
        assert math.isclose(forwarded.info.resolution, 0.2, abs_tol=1e-6)
        assert math.isclose(forwarded.info.origin.position.x, -1.5, abs_tol=1e-9)
        assert list(forwarded.data) == [-1, 100]
        print("PASS: onboard OccupancyGrid callback preserves its aligned snapshot")
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
