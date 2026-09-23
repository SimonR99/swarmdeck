"""Publish the fleet's fixed sensor mounts once, retaining them for late joiners."""

import json
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


def publish_mounts(node, fleet: dict) -> list:
    """Keep one transient-local sample containing all four edges per robot.

    TF topics remain namespaced: merging them onto global /tf_static would
    silently disconnect the existing Nav2 and peer listeners.
    """
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = []
    stamp = node.get_clock().now().to_msg()
    for namespace, mounts in fleet.items():
        transforms = []
        for mount in mounts:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = mount["frame_id"]
            transform.child_frame_id = mount["child_frame_id"]
            transform.transform.translation.x = float(mount["x"])
            transform.transform.translation.z = float(mount["z"])
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        publisher = node.create_publisher(TFMessage, f"/{namespace}/tf_static", qos)
        publisher.publish(TFMessage(transforms=transforms))
        publishers.append(publisher)
    return publishers


def main():
    fleet = json.loads(sys.argv[1])
    rclpy.init(args=sys.argv[2:])
    node = rclpy.create_node("sensor_mounts")
    try:
        publish_mounts(node, fleet)
        # Transient-local history belongs to the live writer, not a ROS master.
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
