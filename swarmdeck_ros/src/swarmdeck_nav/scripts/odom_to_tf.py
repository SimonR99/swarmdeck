#!/usr/bin/env python3
"""Publish the pose carried by an Odometry message as a TF transform.

Some SLAM systems expose a perfectly usable map-frame ``nav_msgs/Odometry``
but do not broadcast the equivalent transform. Nav2 consumes TF, so this small
adapter publishes exactly that missing edge without filtering or integrating
the pose a second time.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def yaw_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    """Return the planar quaternion (z, w) for an arbitrary orientation."""
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdometryTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tf")
        self.declare_parameter("odom_topic", "/laser_odometry")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "os_lidar")
        self.declare_parameter("planar", True)
        self.declare_parameter("use_receive_time", False)

        self._parent_frame = str(self.get_parameter("parent_frame").value)
        self._child_frame = str(self.get_parameter("child_frame").value)
        self._planar = bool(self.get_parameter("planar").value)
        self._use_receive_time = bool(
            self.get_parameter("use_receive_time").value
        )
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )

    def _on_odometry(self, msg: Odometry) -> None:
        transform = TransformStamped()
        # Some localization pipelines publish a pose whose source timestamp is
        # consistently older than live sensor messages. A TF stamped with that
        # old time can never satisfy costmap message filters. Hardware launch
        # files may opt into receipt time while the default preserves the exact
        # source timestamp for normal odometry streams.
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self._use_receive_time
            else msg.header.stamp
        )
        transform.header.frame_id = self._parent_frame or msg.header.frame_id
        transform.child_frame_id = self._child_frame or msg.child_frame_id

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = 0.0 if self._planar else position.z

        if self._planar:
            qz, qw = yaw_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
        else:
            transform.transform.rotation = orientation

        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdometryTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
