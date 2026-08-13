"""Turn a ROS Image into a JPEG payload, honouring row padding.

usb_cam (Spot's D435 colour node) publishes rgb8 and may never emit a
CompressedImage unless image_transport's jpeg plugin is loaded. The RTSP
publisher therefore JPEG-encodes the raw topic itself. `step` is not optional:
ROS images may pad rows, and reshaping as (height, width, channels) either
raises or silently skews the frame.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def image_to_jpeg(msg: Any, quality: int = 70) -> bytes | None:
    """Return a JPEG byte string, or None if the encoding is unusable."""
    try:
        import cv2
    except ImportError:
        return None
    encoding = str(getattr(msg, "encoding", "")).lower()
    channels = {
        "rgb8": 3, "8uc3": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1,
    }.get(encoding)
    if channels is None:
        return None
    try:
        step = int(getattr(msg, "step", 0)) or msg.width * channels
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, step)
        frame = rows[:, : msg.width * channels].reshape(
            msg.height, msg.width, channels
        )
    except (ValueError, TypeError):
        return None
    if encoding in ("rgb8", "8uc3"):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    elif encoding == "mono8":
        frame = frame.reshape(msg.height, msg.width)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    return buf.tobytes()
