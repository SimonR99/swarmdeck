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


def rotate_vector(
    qx: float, qy: float, qz: float, qw: float, v: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Rotate ``v`` by the quaternion, via v + 2q_vec x (q_vec x v + w v)."""
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class OdometryTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tf")
        self.declare_parameter("odom_topic", "/laser_odometry")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "os_lidar")
        self.declare_parameter("planar", True)
        self.declare_parameter("use_receive_time", False)
        # Lever arm from child_frame's origin to the origin of the frame the
        # odometry pose actually describes, expressed in child_frame. Zero keeps
        # the historical relabel-only behaviour for every caller that does not
        # set it.
        self.declare_parameter("sensor_offset_x", 0.0)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.declare_parameter("sensor_offset_z", 0.0)

        self._parent_frame = str(self.get_parameter("parent_frame").value)
        self._child_frame = str(self.get_parameter("child_frame").value)
        self._planar = bool(self.get_parameter("planar").value)
        self._use_receive_time = bool(self.get_parameter("use_receive_time").value)
        self._offset = (
            float(self.get_parameter("sensor_offset_x").value),
            float(self.get_parameter("sensor_offset_y").value),
            float(self.get_parameter("sensor_offset_z").value),
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

        # The pose describes the SENSOR, not child_frame. Publishing it under
        # child_frame's name without removing the lever arm makes an in-place
        # rotation look like the robot driving a circle of that radius: the
        # sensor genuinely does sweep such a circle about the base centre. On
        # aslan 2026-09-03, with a 0.16 m forward-mounted Ouster, that was a
        # 32 cm-diameter phantom orbit. Subtract the arm, rotated into the map
        # frame by the pose's own orientation.
        px, py, pz = position.x, position.y, position.z
        if any(self._offset):
            if self._planar:
                yaw = math.atan2(
                    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
                )
                ox, oy, oz = self._offset
                dx = math.cos(yaw) * ox - math.sin(yaw) * oy
                dy = math.sin(yaw) * ox + math.cos(yaw) * oy
                dz = oz
            else:
                dx, dy, dz = rotate_vector(
                    orientation.x, orientation.y, orientation.z, orientation.w, self._offset
                )
            px, py, pz = px - dx, py - dy, pz - dz

        transform.transform.translation.x = px
        transform.transform.translation.y = py
        transform.transform.translation.z = 0.0 if self._planar else pz

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
