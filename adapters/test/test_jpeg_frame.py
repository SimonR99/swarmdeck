"""JPEG encoding for the RTSP publisher, without GStreamer or ROS."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "media"))
from jpeg_frame import image_to_jpeg, raw_frame_bytes  # noqa: E402


def _rgb8(width: int, height: int, *, pad: int = 0) -> SimpleNamespace:
    channels = 3
    step = width * channels + pad
    data = np.zeros((height, step), dtype=np.uint8)
    data[:, : width * channels] = 64
    data[0, 0:3] = (255, 0, 0)
    return SimpleNamespace(
        encoding="rgb8",
        width=width,
        height=height,
        step=step,
        data=data.tobytes(),
    )


def test_rgb8_becomes_a_jpeg():
    jpeg = image_to_jpeg(_rgb8(8, 4))
    assert jpeg is not None
    assert jpeg.startswith(b"\xff\xd8")


def test_padded_rows_do_not_skew_or_raise():
    jpeg = image_to_jpeg(_rgb8(8, 4, pad=16))
    assert jpeg is not None
    assert jpeg.startswith(b"\xff\xd8")


def test_raw_rgb_bytes_strip_padding_without_jpeg_encoding():
    msg = _rgb8(2, 2, pad=4)
    assert raw_frame_bytes(msg) == ("RGB", bytes([255, 0, 0, 64, 64, 64] + [64] * 6))


def test_unknown_raw_encoding_is_dropped():
    msg = _rgb8(2, 2)
    msg.encoding = "yuv422"
    assert raw_frame_bytes(msg) is None


def test_unknown_encoding_is_dropped():
    msg = _rgb8(4, 2)
    msg.encoding = "yuv422"
    assert image_to_jpeg(msg) is None
