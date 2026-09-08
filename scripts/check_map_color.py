#!/usr/bin/env python3
"""Passively check calibrated camera/LiDAR joins on ROS 1 or ROS 2.

Run inside the robot adapter container with ROS sourced. Prints only aggregate
counts/timing; never saves or uploads images, maps, or commands to the robot.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from adapters.map_color import CameraColorizer
from adapters.perception.depth_projection import transform_points
from adapters.runtime import AdapterSensorMixin, unique_row_index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros", choices=["1", "2"], required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration", type=float, default=20)
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    topics = cfg["topics"]
    color = CameraColorizer(cfg.get("map_color", {}))
    from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
    from tf2_ros import Buffer, TransformListener

    latest = {}
    subs = []
    if args.ros == "2":
        import rclpy
        from rclpy.qos import qos_profile_sensor_data

        rclpy.init()
        node = rclpy.create_node("swarmdeck_color_check")
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def subscribe(topic, cls, callback):
            subs.append(
                node.create_subscription(cls, topic, callback, qos_profile_sensor_data)
            )

        def tick():
            rclpy.spin_once(node, timeout_sec=0.05)

        def stamp(header):
            return rclpy.time.Time.from_msg(header.stamp)

    else:
        import rospy

        rospy.init_node("swarmdeck_color_check", anonymous=True)
        buffer = Buffer()
        listener = TransformListener(buffer)

        def subscribe(topic, cls, callback):
            subs.append(
                rospy.Subscriber(
                    topic, cls, callback, queue_size=1, buff_size=8 * 1024 * 1024
                )
            )

        def tick():
            time.sleep(0.05)

        def stamp(header):
            return header.stamp

    subscribe(
        topics["camera_compressed"],
        CompressedImage,
        lambda msg: color.push(bytes(msg.data), msg.header),
    )
    subscribe(
        topics["camera_color_info"], CameraInfo, lambda msg: latest.update(info=msg)
    )
    subscribe(topics["map_cloud"], PointCloud2, lambda msg: latest.update(cloud=msg))

    def lookup(header):
        return buffer.lookup_transform(header.frame_id, cfg["map_frame"], stamp(header))

    start = time.monotonic()
    last = start + 2  # allow TF and camera history to arrive
    successes = 0
    try:
        while time.monotonic() - start < args.duration:
            tick()
            now = time.monotonic()
            if now - last < 2 or "cloud" not in latest:
                continue
            last = now
            cloud = latest.pop("cloud")
            points = AdapterSensorMixin._cloud_xyz(cloud)
            try:
                if cloud.header.frame_id.lstrip("/") != cfg["map_frame"].lstrip("/"):
                    tf = buffer.lookup_transform(
                        cfg["map_frame"], cloud.header.frame_id, stamp(cloud.header)
                    )
                    points = transform_points(points, tf.transform)
                points = points[
                    unique_row_index(np.round(points / 0.1).astype(np.int32))
                ]
                before = time.perf_counter()
                rgba = color.colorize(points, cloud.header, latest.get("info"), lookup)
                elapsed = (time.perf_counter() - before) * 1000
                count = int((rgba[:, 3] > 0).sum()) if rgba is not None else 0
                successes += count > 0
                print(
                    json.dumps(
                        {
                            "points": len(points),
                            "colored": count,
                            "projection_ms": round(elapsed, 1),
                            "status": color.last_status,
                        }
                    ),
                    flush=True,
                )
            except Exception as exc:
                print(json.dumps({"status": str(exc)}), flush=True)
    finally:
        if args.ros == "2":
            node.destroy_node()
            rclpy.shutdown()
    if not successes:
        print(
            "No colored samples. Check timestamps, RGB CameraInfo, and capture-time TF.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
