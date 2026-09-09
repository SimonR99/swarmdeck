#!/usr/bin/env python3
"""Record rectified, RGB-aligned depth and capture-time TF for UMAMI export.

Run inside the robot's ROS 2 environment; only bounded/latest frames are queued.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import time
import uuid
import numpy as np


_CAPTURE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


def _validate_capture_id(value, name, parser):
    if not value:
        return ""
    if not _CAPTURE_ID.fullmatch(value):
        parser.error(f"{name} must be a safe non-empty identifier")
    return value


def _validate_session_id(value, parser):
    if not value:
        return ""
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError):
        parser.error("session-id must be a canonical UUID")
    if str(parsed) != value.lower() or str(parsed) != value:
        parser.error("session-id must use canonical lowercase UUID spelling")
    return value


def associate_preceding_keyframe(records, stamp_ns, max_age_s):
    """Select the newest metadata record at or before a capture timestamp."""
    preceding = [item for item in records if int(item["stamp_ns"]) <= int(stamp_ns)]
    if not preceding:
        raise ValueError("no preceding onboard keyframe metadata is available")
    associated = max(preceding, key=lambda item: int(item["stamp_ns"]))
    if int(stamp_ns) - int(associated["stamp_ns"]) > float(max_age_s) * 1e9:
        raise ValueError("preceding onboard keyframe is older than the association limit")
    return associated


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_ros import Buffer, TransformListener
    import message_filters
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb", required=True)
    parser.add_argument("--depth", required=True)
    parser.add_argument("--camera-info", required=True)
    parser.add_argument("--world-frame", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=2000)
    # These fields are optional so old capture commands continue to work.  They
    # are copied into both the capture manifest and each frame where possible.
    parser.add_argument("--capture-id")
    parser.add_argument("--robot-id", default="")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--submap-id", default="")
    parser.add_argument("--calibration-version", default="")
    parser.add_argument("--keyframe-id", default="")
    parser.add_argument(
        "--keyframe-metadata-topic",
        default="",
        help="JSON String topic from the onboard mapper: keyframe_id, stamp_ns, odom_frame, T_odom_keyframe",
    )
    parser.add_argument(
        "--keyframe-max-age-s",
        "--keyframe-association-window-s",
        dest="keyframe_max_age_s",
        type=float,
        default=2.5,
    )
    parser.add_argument(
        "--keyframe-frame",
        help="optional SLAM keyframe TF frame; preserves T_keyframe_camera per capture",
    )
    parser.add_argument("--metadata-json", type=Path)
    args = parser.parse_args()
    if args.period <= 0 or args.max_frames <= 0 or args.keyframe_max_age_s <= 0:
        parser.error("positive period, max-frames, and keyframe association window required")
    if args.keyframe_metadata_topic and (args.keyframe_id or args.keyframe_frame):
        parser.error(
            "keyframe-id/keyframe-frame cannot be combined with dynamic keyframe metadata"
        )
    args.capture_id = _validate_capture_id(args.capture_id, "capture-id", parser)
    args.robot_id = _validate_capture_id(args.robot_id, "robot-id", parser)
    args.session_id = _validate_session_id(args.session_id, parser)
    args.submap_id = _validate_capture_id(args.submap_id, "submap-id", parser)
    args.calibration_version = _validate_capture_id(
        args.calibration_version, "calibration-version", parser
    )
    args.keyframe_id = _validate_capture_id(args.keyframe_id, "keyframe-id", parser)
    args.output.mkdir(parents=True, exist_ok=False)
    supplied_metadata = {}
    if args.metadata_json:
        supplied_metadata = json.loads(args.metadata_json.read_text())
        if not isinstance(supplied_metadata, dict):
            parser.error("metadata-json must contain a JSON object")
    capture_metadata = {
        "schema_version": 2,
        "capture_id": args.capture_id or uuid.uuid4().hex,
        "robot_id": args.robot_id,
        "session_id": args.session_id,
        "submap_id": args.submap_id,
        "calibration_version": args.calibration_version,
        "world_frame": args.world_frame,
        "optical_frame": "",
        "keyframe_id": args.keyframe_id if not args.keyframe_metadata_topic else "",
        "keyframe_metadata_topic": args.keyframe_metadata_topic,
        **supplied_metadata,
    }
    (args.output / "capture_manifest.json").write_text(
        json.dumps(capture_metadata, indent=2, sort_keys=True) + "\n"
    )
    rclpy.init()
    node = Node("swarmdeck_rgbd_capture")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pool = ThreadPoolExecutor(max_workers=1)
    state = {"info": None, "last": 0.0, "count": 0, "future": None, "keyframes": []}
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

    def keyframe_metadata(message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("metadata must be an object")
            keyframe_id = str(value.get("keyframe_id", ""))
            parts = keyframe_id.split("/")
            if len(parts) != 3 or not parts[0] or not parts[1]:
                raise ValueError("keyframe_id must be robot/session/seq")
            parsed_session = uuid.UUID(parts[1])
            if str(parsed_session) != parts[1]:
                raise ValueError("session in keyframe_id must be canonical lowercase UUID")
            sequence = int(parts[2])
            if sequence < 0 or not parts[2].isdigit() or not _CAPTURE_ID.fullmatch(parts[0]):
                raise ValueError("invalid keyframe identity")
            if "stamp_ns" in value:
                stamp_ns = int(value["stamp_ns"])
            else:
                stamp_ns = round(float(value["stamp"]) * 1e9)
            odom_frame = str(value.get("odom_frame", ""))
            T_odom_keyframe = np.asarray(value.get("T_odom_keyframe"), dtype=float)
            if (
                not odom_frame
                or T_odom_keyframe.shape != (4, 4)
                or not np.isfinite(T_odom_keyframe).all()
                or not np.allclose(T_odom_keyframe[3], [0, 0, 0, 1])
                or not np.allclose(
                    T_odom_keyframe[:3, :3].T @ T_odom_keyframe[:3, :3], np.eye(3), atol=1e-5
                )
                or not np.isclose(np.linalg.det(T_odom_keyframe[:3, :3]), 1)
            ):
                raise ValueError("odom_frame and rigid T_odom_keyframe are required")
            state["keyframes"].append(
                {
                    "keyframe_id": keyframe_id,
                    "stamp_ns": stamp_ns,
                    "odom_frame": odom_frame,
                    "T_odom_keyframe": T_odom_keyframe,
                    "component_id": str(value.get("component_id", "")),
                }
            )
            state["keyframes"] = state["keyframes"][-256:]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            node.get_logger().warning(f"Ignoring malformed keyframe metadata: {exc}")

    if args.keyframe_metadata_topic:
        node.create_subscription(String, args.keyframe_metadata_topic, keyframe_metadata, 20)
    sync = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], queue_size=3, slop=0.03
    )

    def transform_matrix(transform):
        q = transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
        return matrix

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
            keyframe_id = args.keyframe_id
            keyframe_frame = args.keyframe_frame
            keyframe_component = ""
            dynamic_keyframe_camera = None
            if args.keyframe_metadata_topic:
                stamp_ns = stamp.nanoseconds
                associated = associate_preceding_keyframe(
                    state["keyframes"], stamp_ns, args.keyframe_max_age_s
                )
                keyframe_id = associated["keyframe_id"]
                keyframe_component = associated["component_id"]
            tf = buffer.lookup_transform(
                args.world_frame,
                rgb_msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            ).transform
            twc = transform_matrix(tf)
            if args.keyframe_metadata_topic:
                odom_tf = buffer.lookup_transform(
                    associated["odom_frame"],
                    rgb_msg.header.frame_id,
                    stamp,
                    timeout=Duration(seconds=0.05),
                ).transform
                dynamic_keyframe_camera = (
                    np.linalg.inv(associated["T_odom_keyframe"]) @ transform_matrix(odom_tf)
                )
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
            frame_data = {
                "rgb": rgb,
                "depth_m": depth,
                "depth_valid_mask": np.isfinite(depth) & (depth > 0),
                "K": np.asarray(info.k).reshape(3, 3),
                "D": np.asarray(info.d),
                "distortion_model": np.asarray(info.distortion_model),
                "depth_units": np.asarray("metres"),
                "T_world_camera": twc,
                "stamp": stamp.nanoseconds / 1e9,
                "capture_id": np.asarray(capture_metadata["capture_id"]),
                "robot_id": np.asarray(capture_metadata["robot_id"]),
                "session_id": np.asarray(capture_metadata["session_id"]),
                "submap_id": np.asarray(capture_metadata["submap_id"]),
                "calibration_version": np.asarray(
                    capture_metadata["calibration_version"]
                ),
                "keyframe_id": np.asarray(keyframe_id),
                "optical_frame": np.asarray(rgb_msg.header.frame_id),
            }
            if keyframe_component:
                frame_data["component_id"] = np.asarray(keyframe_component)
            if dynamic_keyframe_camera is not None:
                frame_data["T_keyframe_camera"] = dynamic_keyframe_camera
            elif keyframe_frame:
                keyframe_tf = buffer.lookup_transform(
                    keyframe_frame,
                    rgb_msg.header.frame_id,
                    stamp,
                    timeout=Duration(seconds=0.05),
                ).transform
                frame_data["T_keyframe_camera"] = transform_matrix(keyframe_tf)
            path = args.output / f"{state['count']:08d}.npz"
            state["future"] = pool.submit(np.savez_compressed, path, **frame_data)
            if not capture_metadata["optical_frame"]:
                capture_metadata["optical_frame"] = rgb_msg.header.frame_id
                (args.output / "capture_manifest.json").write_text(
                    json.dumps(capture_metadata, indent=2, sort_keys=True) + "\n"
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
