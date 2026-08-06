#!/usr/bin/env python3
"""Summarise Swarm-SLAM's pose graph onto one plain topic the adapter can read.

    ros2 run swarmdeck_cslam graph_reporter.py --ros-args -p robots:=4

Why this exists rather than having `adapter_sim` subscribe to cslam directly:
cslam's messages live in `cslam_common_interfaces`, which is built inside the
cslam image and is *not* installed in the Gazebo image where the adapter runs.
A subscriber cannot deserialise a type it does not have, so the adapter would
see the topic and never decode a message — the same silent failure mode that
has already cost this project a day.

So this node runs on the cslam side, where the types exist, and republishes a
JSON summary on `/swarmdeck/slam_graph` as a `std_msgs/String`. `std_msgs` is
universally available, `adapter_sim` stays the only thing that speaks the
SwarmDeck protocol, and neither image grows a dependency on the other.

What is summarised is deliberately what an operator can act on: how much graph
each robot has, who it has actually met, and whether it is in the common frame.
"""

from __future__ import annotations

import json
import math

import rclpy
from cslam_common_interfaces.msg import InterRobotLoopClosure, KeyframeOdom
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class GraphReporter(Node):
    def __init__(self) -> None:
        super().__init__("swarmdeck_graph_reporter")
        self.declare_parameter("robots", 4)
        self.declare_parameter("publish_period_s", 2.0)
        count = int(self.get_parameter("robots").value)
        period = float(self.get_parameter("publish_period_s").value)

        self.keyframes: dict[int, int] = {i: 0 for i in range(count)}
        # Each robot's pose in whatever frame cslam has placed it in, plus the
        # name of that frame. The frame is `robot<N>_map` of the ORIGIN robot of
        # this robot's cluster — so robots that have never met report their own
        # frame, and two disjoint clusters produce two different common frames.
        # That distinction is the whole point: only robots sharing an origin can
        # be drawn on one map.
        self.common_pose: dict[int, dict] = {}
        # (a, b) -> verified closure count, a < b.
        self.closures: dict[tuple[int, int], int] = {}
        self.last_closure_t: dict[tuple[int, int], float] = {}

        for i in range(count):
            self.create_subscription(
                KeyframeOdom,
                f"/r{i}/cslam/keyframe_odom",
                (lambda r: lambda _m: self._on_keyframe(r))(i),
                100,
            )
        for i in range(count):
            self.create_subscription(
                PoseStamped,
                f"/r{i}/cslam/current_pose_estimate",
                (lambda r: lambda m: self._on_pose(r, m))(i),
                10,
            )
        # One shared topic for the whole fleet — cslam publishes inter-robot
        # closures globally, not per robot.
        self.create_subscription(
            InterRobotLoopClosure, "/cslam/inter_robot_loop_closure", self._on_closure, 100
        )
        self.pub = self.create_publisher(String, "/swarmdeck/slam_graph", 10)
        self.create_timer(period, self._publish)
        self.get_logger().info(f"reporting the pose graph for {count} robots")

    @staticmethod
    def _yaw_of(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_pose(self, robot: int, msg: PoseStamped) -> None:
        self.common_pose[robot] = {
            "frame": msg.header.frame_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": self._yaw_of(msg.pose.orientation),
        }

    def _on_keyframe(self, robot: int) -> None:
        self.keyframes[robot] = self.keyframes.get(robot, 0) + 1

    def _on_closure(self, msg: InterRobotLoopClosure) -> None:
        # Only VERIFIED closures count. cslam publishes rejected candidates on
        # the same topic with success=False, and counting those would report
        # agreement that the geometry check explicitly refused.
        if not getattr(msg, "success", False):
            return
        key = tuple(sorted((int(msg.robot0_id), int(msg.robot1_id))))
        self.closures[key] = self.closures.get(key, 0) + 1
        self.last_closure_t[key] = self.get_clock().now().nanoseconds * 1e-9

    def _publish(self) -> None:
        for robot, keyframes in sorted(self.keyframes.items()):
            links = []
            for (a, b), n in sorted(self.closures.items()):
                if robot not in (a, b):
                    continue
                other = b if robot == a else a
                links.append(
                    {
                        "other": f"robot_{other}",
                        "count": n,
                        "last_t": round(self.last_closure_t.get((a, b), 0.0), 2),
                    }
                )
            self.pub.publish(
                String(
                    data=json.dumps(
                        {
                            "robot_id": f"robot_{robot}",
                            "keyframes": keyframes,
                            # A robot is in the common frame once something has
                            # actually tied it to another robot. Before that its
                            # graph is private, however healthy it looks.
                            "in_common_frame": bool(links),
                            # Pose in the cluster's common frame, and which
                            # frame that is. The adapter turns this into the
                            # map-frame transform the merge needs.
                            "common": self.common_pose.get(robot),
                            "residual": None,
                            "inter_robot": links,
                        }
                    )
                )
            )


def main() -> None:
    rclpy.init()
    node = GraphReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
