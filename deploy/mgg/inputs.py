#!/usr/bin/env python3
"""Supply MGG with map-frame odometry and bounded live sensor clouds."""

import copy
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class Inputs(Node):
    def __init__(self):
        super().__init__("mgg_inputs")
        self.frame = self.declare_parameter("map_frame", "map").value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.odom = self.create_publisher(Odometry, "map_odometry", 10)
        self.cloud = self.create_publisher(
            PointCloud2, "mapping_cloud", qos_profile_sensor_data
        )
        self.latest = None
        self.create_subscription(
            Odometry, "input_odometry", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "input_cloud", self.on_cloud, qos_profile_sensor_data
        )
        self.create_timer(0.5, self.publish_cloud)

    def on_odom(self, msg):
        try:
            tf = self.buffer.lookup_transform(
                self.frame, msg.child_frame_id, Time.from_msg(msg.header.stamp)
            )
        except TransformException:
            return  # No latest-TF fallback: one pose and one capture time.
        output = copy.deepcopy(msg)
        output.header.frame_id = self.frame
        t = tf.transform.translation
        (
            output.pose.pose.position.x,
            output.pose.pose.position.y,
            output.pose.pose.position.z,
        ) = (t.x, t.y, t.z)
        output.pose.pose.orientation = tf.transform.rotation
        self.odom.publish(output)

    def on_cloud(self, msg):
        self.latest = msg

    def publish_cloud(self):
        if self.latest is not None:
            self.cloud.publish(self.latest)
            self.latest = None


def main():
    rclpy.init()
    node = Inputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
