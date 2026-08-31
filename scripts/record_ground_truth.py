#!/usr/bin/env python3
"""Record `/<ns>/ground_truth` to CSV, so a captured keyframe run can be scored.

`slam/swarmdeck_slam/evaluation.py` measures ATE, RPE and inter-robot transform
error against ground truth, but nothing in the live path carries it: Gazebo
publishes `<ns>/ground_truth` and `adapter_sim` never subscribes, so the true
pose stops at the ROS boundary. This node is the bridge across that boundary,
and it exists as a separate recorder rather than an adapter subscription on
purpose -- ground truth must never enter the wire contract the hardware
adapters also speak, or the backend could silently be scored against an input
real robots cannot provide.

Stamps are the message header's, under `use_sim_time`, which is the same clock
`adapter_sim` stamps keyframes with. That is what lets the replay harness join
the two by time without any shared sequence number.

Run inside the Gazebo container, alongside a capture-enabled SLAM service:

    ros2 run ... no -- this is not an installed entry point. Copy and run it:
    docker cp scripts/record_ground_truth.py swarmdeck-gazebo-1:/tmp/
    docker exec swarmdeck-gazebo-1 bash -lc \
        'source /opt/ros/jazzy/setup.bash && \
         python3 /tmp/record_ground_truth.py --robots 2 --out /app/sessions/gt.csv'
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GroundTruthRecorder(Node):
    def __init__(self, robots: int, prefix: str, out_path: str) -> None:
        super().__init__("ground_truth_recorder")
        # use_sim_time is passed as a ROS arg, not declared here: rclpy declares
        # it for every node automatically, so a second declare_parameter raises
        # ParameterAlreadyDeclaredException before a single row is written.
        output = Path(out_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self._handle = output.open("w", newline="")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(
            ["robot_id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
        )
        self._rows = 0
        for index in range(robots):
            robot_id = f"{prefix}{index}"
            self.create_subscription(
                Odometry,
                f"/{robot_id}/ground_truth",
                self._make_callback(robot_id),
                qos_profile_sensor_data,
            )
        self.get_logger().info(f"recording {robots} robot(s) to {out_path}")

    def _make_callback(self, robot_id: str):
        def callback(msg: Odometry) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self._writer.writerow(
                [robot_id, f"{stamp:.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            )
            self._rows += 1
            # Flush steadily: this node is normally killed with SIGINT/SIGTERM
            # when the run ends, and a buffered tail is a silently short dataset.
            if self._rows % 200 == 0:
                self._handle.flush()

        return callback

    def close(self) -> None:
        self._handle.flush()
        self._handle.close()
        self.get_logger().info(f"wrote {self._rows} rows")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=2)
    parser.add_argument("--prefix", default="robot_")
    parser.add_argument("--out", required=True)
    # Unknown args are left for rclpy: `--ros-args -p use_sim_time:=true` has to
    # reach init(), and argparse would otherwise reject it.
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = GroundTruthRecorder(args.robots, args.prefix, args.out)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
