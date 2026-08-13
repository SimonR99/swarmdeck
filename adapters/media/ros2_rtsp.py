#!/usr/bin/env python3
"""Push a ROS 2 camera topic to an RTSP server with low latency.

`--topic` is a JPEG CompressedImage (OAK / RealSense). `--raw-topic` is a
sensor_msgs/Image fallback for drivers such as usb_cam that may never publish
compressed JPEG. Prefer the compressed topic when both are present.
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from pathlib import Path

import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jpeg_frame import image_to_jpeg  # noqa: E402


class Ros2JpegRtspPublisher(Node):
    """Keep only the newest eligible JPEG and encode it for MediaMTX."""

    def __init__(
        self,
        robot_id: str,
        topic: str,
        rtsp_url: str,
        bitrate_kbps: int,
        fps: int,
        raw_topic: str = "",
    ) -> None:
        safe_id = re.sub(r"[^a-zA-Z0-9_]", "_", robot_id)
        super().__init__(f"swarmdeck_media_{safe_id}")
        Gst.init(None)
        self._frame_period_s = 1.0 / fps
        self._last_frame_at = 0.0
        pipeline = (
            "appsrc name=source is-live=true block=false format=time do-timestamp=true "
            f'max-bytes=100000 caps="image/jpeg,framerate={fps}/1" '
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! jpegparse ! jpegdec "
            "! videoconvert "
            f"! video/x-raw,format=I420,framerate={fps}/1 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
            f"key-int-max={fps} bframes=0 rc-lookahead=0 sync-lookahead=0 "
            "sliced-threads=true byte-stream=true "
            "! video/x-h264,profile=baseline "
            "! h264parse config-interval=-1 "
            f'! rtspclientsink location="{rtsp_url}" protocols=tcp latency=0'
        )
        self.pipeline = Gst.parse_launch(pipeline)
        self.source = self.pipeline.get_by_name("source")
        self.bus = self.pipeline.get_bus()
        self._failed = threading.Event()
        self._monitor = threading.Thread(target=self._monitor_bus, daemon=True)

        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer pipeline failed to enter PLAYING state")
        self._monitor.start()
        self.subscription = self.create_subscription(
            CompressedImage, topic, self._on_frame, qos_profile_sensor_data
        )
        if raw_topic:
            self.create_subscription(
                Image, raw_topic, self._on_raw_frame, qos_profile_sensor_data
            )

    def _push_jpeg(self, payload: bytes) -> None:
        now = time.monotonic()
        if now - self._last_frame_at < self._frame_period_s:
            return
        # appsrc owns a queue before the explicitly leaky GStreamer queue.
        # Never allow an old teleoperation frame to wait in either one.
        if self.source.get_property("current-level-bytes") > 0:
            return
        self._last_frame_at = now
        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        flow = self.source.emit("push-buffer", buffer)
        if flow not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
            self.get_logger().warning(f"camera encoder rejected a frame: {flow}")

    def _on_frame(self, msg: CompressedImage) -> None:
        if self._failed.is_set() or (msg.format and "jpeg" not in msg.format.lower()):
            return
        self._push_jpeg(bytes(msg.data))

    def _on_raw_frame(self, msg: Image) -> None:
        if self._failed.is_set():
            return
        payload = image_to_jpeg(msg)
        if payload:
            self._push_jpeg(payload)

    def _monitor_bus(self) -> None:
        while rclpy.ok() and not self._failed.is_set():
            message = self.bus.timed_pop_filtered(
                1 * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.get_logger().error(
                    f"camera RTSP pipeline failed: {error}; {debug or ''}"
                )
            else:
                self.get_logger().error("camera RTSP pipeline ended unexpectedly")
            self._failed.set()
            rclpy.try_shutdown()

    def close(self) -> None:
        self.source.emit("end-of-stream")
        self.pipeline.set_state(Gst.State.NULL)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument(
        "--raw-topic",
        default="",
        help="sensor_msgs/Image fallback when --topic has no JPEG",
    )
    parser.add_argument("--rtsp-url", required=True)
    parser.add_argument("--bitrate-kbps", type=int, default=1200)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--robot-id", default="robot")
    args = parser.parse_args()

    rclpy.init()
    publisher = Ros2JpegRtspPublisher(
        args.robot_id,
        args.topic,
        args.rtsp_url,
        args.bitrate_kbps,
        max(1, args.fps),
        raw_topic=args.raw_topic,
    )
    source = args.topic + (f" or {args.raw_topic}" if args.raw_topic else "")
    publisher.get_logger().info(f"streaming {source} to {args.rtsp_url}")
    try:
        rclpy.spin(publisher)
    finally:
        publisher.close()
        publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
