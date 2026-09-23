#!/usr/bin/env python3
"""Push a ROS 2 camera topic to an H.264 RTSP server with low latency.

`--topic` is a JPEG CompressedImage (OAK / RealSense). `--raw-topic` is a
sensor_msgs/Image fallback that enters the H.264 encoder as raw pixels, without
a JPEG round trip. Prefer the compressed topic when both are present.
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
from jpeg_frame import raw_frame_bytes  # noqa: E402

# Keep JPEG preferred through brief delivery gaps (20 frames at the default
# 10 fps), but resume raw video if that camera's compressed stream disappears.
COMPRESSED_PREFERENCE_TIMEOUT_S = 2.0


class Ros2JpegRtspPublisher(Node):
    """Keep only the newest eligible frame and encode it as H.264 for MediaMTX."""

    def __init__(
        self,
        robot_id: str,
        topic: str,
        rtsp_url: str,
        bitrate_kbps: int,
        fps: int,
        raw_topic: str = "",
        width: int = 640,
        height: int = 480,
    ) -> None:
        safe_id = re.sub(r"[^a-zA-Z0-9_]", "_", robot_id)
        super().__init__(f"swarmdeck_media_{safe_id}")
        Gst.init(None)
        self._frame_period_s = 1.0 / fps
        self._last_frame_at = 0.0
        pipeline = (
            "input-selector name=selector sync-streams=false cache-buffers=false "
            "! videoconvert ! videoscale method=bilinear "
            f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
            f"key-int-max={fps} bframes=0 rc-lookahead=0 sync-lookahead=0 "
            "sliced-threads=true byte-stream=true "
            "! video/x-h264,profile=baseline "
            "! h264parse config-interval=-1 "
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=100000000 leaky=downstream "
            # Keep the RTP packets on the RTSP TCP connection.  The robot and
            # backend are separated by Wi-Fi, and UDP loss can leave
            # rtspclientsink's internal reconnect with a live control socket
            # but no publishing session.  TCP also makes a broken session
            # observable to the bus monitor so the outer loop can rebuild it.
            f'! rtspclientsink location="{rtsp_url}" protocols=tcp '
            "latency=0 tcp-timeout=5000000 "
            "appsrc name=source is-live=true block=false format=time do-timestamp=true "
            f'max-bytes=100000 caps="image/jpeg,framerate={fps}/1" '
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! jpegparse ! jpegdec ! selector.sink_0 "
            "appsrc name=raw_source is-live=true block=false format=time do-timestamp=true "
            "max-bytes=1000000 "
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! selector.sink_1"
        )
        self.pipeline = Gst.parse_launch(pipeline)
        self.source = self.pipeline.get_by_name("source")
        self.raw_source = self.pipeline.get_by_name("raw_source")
        self.selector = self.pipeline.get_by_name("selector")
        self._raw_caps = None
        self._last_compressed_at = 0.0
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

    def _can_push(self, source) -> bool:
        return (
            time.monotonic() - self._last_frame_at >= self._frame_period_s
            and source.get_property("current-level-bytes") == 0
        )

    def _push_frame(self, source, payload: bytes, pad_name: str) -> None:
        self.selector.set_property("active-pad", self.selector.get_static_pad(pad_name))
        self._last_frame_at = time.monotonic()
        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        flow = source.emit("push-buffer", buffer)
        if flow not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
            self.get_logger().warning(f"camera encoder rejected a frame: {flow}")

    def _on_frame(self, msg: CompressedImage) -> None:
        if self._failed.is_set() or (msg.format and "jpeg" not in msg.format.lower()):
            return
        self._last_compressed_at = time.monotonic()
        if self._can_push(self.source):
            self._push_frame(self.source, bytes(msg.data), "sink_0")

    def _on_raw_frame(self, msg: Image) -> None:
        if (
            self._failed.is_set()
            or time.monotonic() - self._last_compressed_at < COMPRESSED_PREFERENCE_TIMEOUT_S
            or not self._can_push(self.raw_source)
        ):
            return
        frame = raw_frame_bytes(msg)
        if frame is None:
            return
        format_name, payload = frame
        caps = (format_name, msg.width, msg.height)
        if caps != self._raw_caps:
            self.raw_source.set_property(
                "caps", Gst.Caps.from_string(
                    f"video/x-raw,format={format_name},width={msg.width},"
                    f"height={msg.height},framerate={round(1 / self._frame_period_s)}/1"
                ),
            )
            self._raw_caps = caps
        self._push_frame(self.raw_source, payload, "sink_1")

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

    def close(self) -> None:
        try:
            self.source.emit("end-of-stream")
            self.raw_source.emit("end-of-stream")
        except Exception:
            pass
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument(
        "--raw-topic",
        default="",
        help="sensor_msgs/Image fallback when --topic has no JPEG",
    )
    parser.add_argument("--rtsp-url", required=True)
    parser.add_argument("--bitrate-kbps", type=int, default=700)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--robot-id", default="robot")
    args = parser.parse_args()

    if args.width < 1 or args.height < 1:
        parser.error("--width and --height must be positive")

    source = args.topic + (f" or {args.raw_topic}" if args.raw_topic else "")
    rclpy.init()
    try:
        while rclpy.ok():
            publisher = None
            try:
                publisher = Ros2JpegRtspPublisher(
                    args.robot_id,
                    args.topic,
                    args.rtsp_url,
                    args.bitrate_kbps,
                    max(1, args.fps),
                    raw_topic=args.raw_topic,
                    width=args.width,
                    height=args.height,
                )
                publisher.get_logger().info(f"streaming {source} to {args.rtsp_url}")
                while rclpy.ok() and not publisher._failed.is_set():
                    rclpy.spin_once(publisher, timeout_sec=0.5)
            except KeyboardInterrupt:
                break
            except Exception as exc:
                print(
                    f"RTSP publisher error: {exc}; retrying in 2s...", file=sys.stderr
                )
            finally:
                if publisher is not None:
                    publisher.close()
                    publisher.destroy_node()
            if rclpy.ok():
                time.sleep(2.0)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
