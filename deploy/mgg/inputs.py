#!/usr/bin/env python3
"""Supply MGG with map-frame odometry and bounded live sensor clouds."""

import copy
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    StaticTransformBroadcaster,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo


class Inputs(Node):
    def __init__(self):
        super().__init__("mgg_inputs")
        self.frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.sim_depth = self.declare_parameter("sim_depth", False).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.odom = self.create_publisher(Odometry, "map_odometry", 10)
        self.cloud = self.create_publisher(
            PointCloud2, "mapping_cloud", qos_profile_sensor_data
        )
        self.pending_odom = deque(maxlen=10)
        self.latest = None
        self.depth = None
        self.intrinsics = None
        if self.sim_depth:
            # Share the relay's node for the missing ARGoS camera extrinsic,
            # instead of starting another ROS process per robot.
            self.camera_tf = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.frame_id = self.base_frame
            transform.child_frame_id = transform.header.frame_id + "/camera"
            transform.transform.translation.x = self.declare_parameter(
                "camera_x", 0.0
            ).value
            transform.transform.translation.z = self.declare_parameter(
                "camera_z", 0.0
            ).value
            transform.transform.rotation.w = 1.0
            self.camera_tf.sendTransform(transform)
        if self.declare_parameter("depth_enabled", self.sim_depth).value:
            self.create_subscription(
                Image, "depth", self.on_depth, qos_profile_sensor_data
            )
            self.create_subscription(
                CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data
            )
        self.create_subscription(
            Odometry, "input_odometry", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "input_cloud", self.on_cloud, qos_profile_sensor_data
        )
        self.create_timer(0.1, self.publish_odom)
        self.create_timer(0.5, self.publish_cloud)

    def on_odom(self, msg):
        self.pending_odom.append(msg)

    def publish_odom(self):
        # Odometry can arrive before the TF broadcast for that same tick.
        # Wait briefly for that exact transform; never substitute latest TF.
        while self.pending_odom:
            msg = self.pending_odom[0]
            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf = self.buffer.lookup_transform(
                    self.frame, self.base_frame or msg.child_frame_id, stamp
                )
            except TransformException:
                if (
                    self.get_clock().now().nanoseconds - stamp.nanoseconds
                    > 1_000_000_000
                ):
                    self.pending_odom.popleft()
                    continue
                return
            self.pending_odom.popleft()
            output = copy.deepcopy(msg)
            output.header.frame_id = self.frame
            output.child_frame_id = self.base_frame or msg.child_frame_id
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

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.intrinsics = msg

    def publish_cloud(self):
        if self.latest is not None:
            self.cloud.publish(self.latest)
            self.latest = None
        depth, info = self.depth, self.intrinsics
        self.depth = None
        if depth is None or info is None or depth.encoding not in ("32FC1", "16UC1"):
            return
        if depth.header.frame_id != info.header.frame_id:
            return
        if (depth.width, depth.height) != (info.width, info.height):
            return
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if fx <= 0 or fy <= 0:
            return
        # Real cameras use optical axes (right/down/forward); ARGoS alone
        # labels its camera frame forward/left/up. Preserve sensor origin/time.
        is_mm = depth.encoding == "16UC1"
        item_size = 2 if is_mm else 4
        z = np.ndarray(
            (depth.height, depth.width),
            dtype=(">" if depth.is_bigendian else "<") + ("u2" if is_mm else "f4"),
            buffer=depth.data,
            strides=(depth.step, item_size),
        )[::4, ::4]
        if is_mm:
            z = z.astype(np.float32) * 0.001
        v, u = np.mgrid[0 : depth.height : 4, 0 : depth.width : 4]
        valid = np.isfinite(z) & (z > 0.05) & (z < 20.0)
        points = np.column_stack(
            (
                z[valid],
                -(u[valid] - cx) * z[valid] / fx,
                -(v[valid] - cy) * z[valid] / fy,
            )
        ).astype("<f4")
        if not self.sim_depth:
            points = points[:, [1, 2, 0]] * [-1, -1, 1]
            points = points.astype("<f4")
        cloud = PointCloud2()
        cloud.header = depth.header
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z"))
        ]
        cloud.point_step, cloud.row_step = 12, 12 * len(points)
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.cloud.publish(cloud)


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
