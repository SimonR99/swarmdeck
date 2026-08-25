#!/usr/bin/env python3
"""Push a ROS 1 camera topic to an H.264 RTSP server with low latency.

The process is intentionally separate from perception.  A slow detector can
drop frames without ever stalling the operator's video stream.
"""

from __future__ import annotations

import argparse
import threading
import time

import gi
import rospy
from sensor_msgs.msg import CompressedImage

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class RosJpegRtspPublisher:
    """Read compressed camera frames and publish only H.264 over RTSP."""

    def __init__(
        self,
        topic: str,
        rtsp_url: str,
        bitrate_kbps: int,
        fps: int,
        width: int,
        height: int,
    ) -> None:
        Gst.init(None)
        self._frame_period_s = 1.0 / fps
        self._last_frame_at = 0.0
        pipeline = (
            "appsrc name=source is-live=true block=false format=time do-timestamp=true "
            f'max-bytes=100000 caps="image/jpeg,framerate={fps}/1" '
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! jpegparse ! jpegdec "
            "! videoconvert ! videoscale method=bilinear "
            f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
            f"key-int-max={fps} bframes=0 rc-lookahead=0 sync-lookahead=0 "
            "sliced-threads=true byte-stream=true "
            "! video/x-h264,profile=baseline "
            "! h264parse config-interval=-1 "
            "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=100000000 leaky=downstream "
            f'! rtspclientsink location="{rtsp_url}" protocols=udp latency=0'
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
        self.subscriber = rospy.Subscriber(
            topic, CompressedImage, self._on_frame, queue_size=1, buff_size=2**20
        )

    def _on_frame(self, msg: CompressedImage) -> None:
        if self._failed.is_set() or (msg.format and "jpeg" not in msg.format.lower()):
            return
        now = time.monotonic()
        if now - self._last_frame_at < self._frame_period_s:
            return
        # appsrc owns an internal queue in front of the explicitly leaky
        # GStreamer queue. Never let an old camera frame wait there: for
        # teleoperation, dropping a frame is always preferable to displaying it
        # late. current-level-bytes is supported by the Noetic image's appsrc.
        if self.source.get_property("current-level-bytes") > 0:
            return
        self._last_frame_at = now
        payload = bytes(msg.data)
        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        flow = self.source.emit("push-buffer", buffer)
        if flow not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
            rospy.logwarn_throttle(5, f"camera encoder rejected a frame: {flow}")

    def _monitor_bus(self) -> None:
        while not rospy.is_shutdown() and not self._failed.is_set():
            message = self.bus.timed_pop_filtered(
                1 * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if message is None:
                continue
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                rospy.logerr(f"camera RTSP pipeline failed: {error}; {debug or ''}")
            else:
                rospy.logerr("camera RTSP pipeline ended unexpectedly")
            self._failed.set()
            rospy.signal_shutdown("RTSP pipeline stopped")

    def close(self) -> None:
        self.source.emit("end-of-stream")
        self.pipeline.set_state(Gst.State.NULL)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--topic", required=True)
    parser.add_argument("--rtsp-url", required=True)
    parser.add_argument("--bitrate-kbps", type=int, default=700)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--robot-id", default="robot")
    args = parser.parse_args()

    if args.width < 1 or args.height < 1:
        parser.error("--width and --height must be positive")

    rospy.init_node(f"swarmdeck_media_{args.robot_id}", anonymous=False)
    publisher = RosJpegRtspPublisher(
        args.topic,
        args.rtsp_url,
        args.bitrate_kbps,
        max(1, args.fps),
        args.width,
        args.height,
    )
    rospy.loginfo(f"streaming {args.topic} to {args.rtsp_url}")
    try:
        rospy.spin()
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
