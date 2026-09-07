#!/usr/bin/env python3
"""Record rectified, RGB-aligned depth and capture-time TF for UMAMI export.

Run inside the robot's ROS 2 environment; only bounded/latest frames are queued.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
import numpy as np


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_ros import Buffer, TransformListener
    import message_filters

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb", required=True)
    parser.add_argument("--depth", required=True)
    parser.add_argument("--camera-info", required=True)
    parser.add_argument("--world-frame", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=2000)
    args = parser.parse_args()
    if args.period <= 0 or args.max_frames <= 0:
        parser.error("positive period and max-frames required")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("swarmdeck_rgbd_capture")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pool = ThreadPoolExecutor(max_workers=1)
    state = {"info": None, "last": 0.0, "count": 0, "future": None}
    info_subscription = node.create_subscription(
        CameraInfo,
        args.camera_info,
        lambda msg: state.update(info=msg),
        qos_profile_sensor_data,
    )
    rgb_sub = message_filters.Subscriber(
        node, Image, args.rgb, qos_profile=qos_profile_sensor_data
    )
    depth_sub = message_filters.Subscriber(
        node, Image, args.depth, qos_profile=qos_profile_sensor_data
    )
    sync = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], queue_size=3, slop=0.03
    )

    def receive(rgb_msg, depth_msg):
        if (
            state["count"] >= args.max_frames
            or time.monotonic() - state["last"] < args.period
        ):
            return
        if state["future"] is not None:
            if not state["future"].done():
                return
            try:
                state["future"].result()
            except Exception as exc:
                node.get_logger().error(f"Capture write failed: {exc}")
        info = state["info"]
        if info is None:
            return
        try:
            if rgb_msg.encoding not in ("rgb8", "bgr8") or depth_msg.encoding not in (
                "16UC1",
                "32FC1",
            ):
                raise ValueError("expected RGB8/BGR8 and metric 16UC1(mm)/32FC1(m)")
            if (rgb_msg.width, rgb_msg.height) != (
                depth_msg.width,
                depth_msg.height,
            ) or (info.width, info.height) != (rgb_msg.width, rgb_msg.height):
                raise ValueError(
                    "RGB and depth must be registered to the same pixel grid"
                )
            if (
                rgb_msg.header.frame_id != depth_msg.header.frame_id
                or info.header.frame_id != rgb_msg.header.frame_id
                or any(abs(d) > 1e-8 for d in info.d)
            ):
                raise ValueError(
                    "rectified, registered RGB/depth optical frames required"
                )
            stamp = rclpy.time.Time.from_msg(rgb_msg.header.stamp)
            tf = buffer.lookup_transform(
                args.world_frame,
                rgb_msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            ).transform
            q = tf.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            twc = np.eye(4)
            twc[:3, :3] = [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
            twc[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
            rgb = np.ndarray(
                (rgb_msg.height, rgb_msg.width, 3),
                dtype=np.uint8,
                buffer=rgb_msg.data,
                strides=(rgb_msg.step, 3, 1),
            ).copy()
            if rgb_msg.encoding == "bgr8":
                rgb = rgb[:, :, ::-1].copy()
            item = 2 if depth_msg.encoding == "16UC1" else 4
            dtype = (">" if depth_msg.is_bigendian else "<") + (
                "u2" if item == 2 else "f4"
            )
            depth = np.ndarray(
                (depth_msg.height, depth_msg.width),
                dtype=dtype,
                buffer=depth_msg.data,
                strides=(depth_msg.step, item),
            ).astype(np.float32)
            if item == 2:
                depth *= 0.001
            path = args.output / f"{state['count']:08d}.npz"
            state["future"] = pool.submit(
                np.savez_compressed,
                path,
                rgb=rgb,
                depth_m=depth,
                K=np.asarray(info.k).reshape(3, 3),
                T_world_camera=twc,
                stamp=stamp.nanoseconds / 1e9,
            )
            state["count"] += 1
            state["last"] = time.monotonic()
        except Exception as exc:
            node.get_logger().warning(f"Skipping RGB-D frame: {exc}")

    sync.registerCallback(receive)
    try:
        while rclpy.ok() and state["count"] < args.max_frames:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        pool.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
