"""Turn a ROS Image into tightly packed raw pixels, honouring row padding.

usb_cam (Spot's D435 colour node) publishes rgb8 and may never emit a
CompressedImage unless image_transport's jpeg plugin is loaded. The RTSP
publisher therefore feeds the raw topic to GStreamer itself. `step` is not
optional: ROS images may pad rows, and reshaping as (height, width, channels)
either raises or silently skews the frame.

Detection decodes the same messages with `adapters.runtime.image_to_bgr`.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def raw_frame_bytes(msg: Any) -> tuple[str, bytes] | None:
    """Return tightly packed raw pixels and their GStreamer format, dropping padding."""
    formats = {
        "rgb8": ("RGB", 3),
        "8uc3": ("RGB", 3),
        "bgr8": ("BGR", 3),
        "rgba8": ("RGBA", 4),
        "bgra8": ("BGRA", 4),
        "mono8": ("GRAY8", 1),
    }
    encoding = str(getattr(msg, "encoding", "")).lower()
    if encoding not in formats:
        return None
    format_name, channels = formats[encoding]
    try:
        row_bytes = int(msg.width) * channels
        step = int(msg.step) or row_bytes
        if step < row_bytes:
            return None
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), step)
        pixels = rows[:, :row_bytes]
    except (TypeError, ValueError):
        return None
    return format_name, pixels.tobytes() if step != row_bytes else bytes(msg.data)
