#!/usr/bin/env python3
"""Publish Chris's Unitree ``rt/odommodestate`` as standard ROS odometry.

Chris's installed G1 bridge has a correct control/state implementation, but
its generic odometry child subscribes to ``rt/sportmodestate``. The robot's
live SportModeState publisher is on ``rt/odommodestate`` instead (confirmed by
the read-only SDK probe on the robot). Keep this bridge small and owned by
SwarmDeck so deployment does not patch or rebuild the external workspace.

This process only subscribes to native state and publishes ROS messages; it
does not call a locomotion API.
"""

# The SDK must be imported before rclpy; see g1_ros2_bridge.dds_init.
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

from g1_ros2_bridge.dds_init import finalize_dds, prepare_dds


class ChrisOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("g1_odom_bridge")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("native_topic", "rt/odommodestate")

        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        self.native_topic = str(self.get_parameter("native_topic").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)

        self.odom_pub = self.create_publisher(Odometry, "/odom", QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscriber = None

    def on_sport_state(self, msg: SportModeState_) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])

        # Unitree's quaternion order is [w, x, y, z].
        q = msg.imu_state.quaternion
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    domain, interface = prepare_dds()
    rclpy.init()
    node = ChrisOdomBridge()
    finalize_dds(domain, interface)
    node.subscriber = ChannelSubscriber(node.native_topic, SportModeState_)
    node.subscriber.Init(node.on_sport_state)
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{interface or '<default>'}'); "
        f"subscribed to {node.native_topic} -> /odom"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
