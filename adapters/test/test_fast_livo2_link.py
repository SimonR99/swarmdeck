"""Tests for Fast-LIVO2 socket wire protocol and link logic.

Verifies:
  1. AEBR frame header parsing and magic validation.
  2. ACK reply binary struct packing (pose, twist, tick).
  3. Lossless PNG image encoding and channel ordering.
  4. PointCloud point step and field offsets.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path
import pytest

REPO = Path(__file__).resolve().parents[2]
FAST_LIVO_DIR = REPO / "deploy" / "docker" / "fast_livo2"
sys.path.insert(0, str(FAST_LIVO_DIR))

import fast_livo_link as fll  # noqa: E402


def test_magic_constants():
    assert fll.MAGIC == b"AEBR"
    assert fll.ACK == b"ACK\0"
    assert fll.POINT_STEP == 26


def test_png_encoding_roundtrip():
    w, h = 4, 4
    rgb = bytes([i % 256 for i in range(w * h * 3)])
    png_bytes = fll.png_encode_rgb(w, h, rgb, bgr=True)

    # Valid PNG header signature: 89 50 4E 47 0D 0A 1A 0A
    assert png_bytes.startswith(b"\x89PNG\r\n\x1a\n")
    assert b"IHDR" in png_bytes
    assert b"IDAT" in png_bytes
    assert png_bytes.endswith(b"IEND\xaeB`\x82")


def test_build_reply_with_unconverged_estimator():
    """When an estimator has not converged yet, valid byte is 0."""

    class DummyPubs:
        estimate = None

    class DummyLink:
        ticks_per_second = 100.0
        poses_returned = 0
        robots = {"robot_0": DummyPubs()}

    build_reply = fll.FastLivoLink.build_reply.__get__(DummyLink())
    reply = build_reply(["robot_0"])

    assert reply.startswith(fll.ACK)
    # Header: "ACK\0" (4 bytes) + robot_count (4 bytes)
    magic, count = struct.unpack("<4sI", reply[:8])
    assert magic == fll.ACK
    assert count == 1

    # Robot 0 entry: id_len (1), id (7), tick (4), pose (56), twist (48), valid (1)
    offset = 8
    id_len = reply[offset]
    assert id_len == len(b"robot_0")
    offset += 1 + id_len

    tick, *rest = struct.unpack("<I7d6dB", reply[offset : offset + 4 + 56 + 48 + 1])
    assert tick == 0
    valid = rest[-1]
    assert valid == 0


def test_build_reply_with_converged_pose():
    """When an estimate arrives, pose and twist are packed accurately."""

    class DummyPubs:
        # stamp_ns, pos, quat, twist
        estimate = (
            5_000_000_000,
            (1.5, 2.5, 0.72),
            (1.0, 0.0, 0.0, 0.0),
            (0.5, 0.0, 0.0, 0.0, 0.0, 0.1),
        )

    class DummyLink:
        ticks_per_second = 100.0
        poses_returned = 0
        robots = {"robot_0": DummyPubs()}

    dummy_link = DummyLink()
    build_reply = fll.FastLivoLink.build_reply.__get__(dummy_link)
    reply = build_reply(["robot_0"])

    magic, count = struct.unpack("<4sI", reply[:8])
    assert count == 1

    offset = 8 + 1 + len(b"robot_0")
    tick, x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, valid = struct.unpack(
        "<I7d6dB", reply[offset : offset + 4 + 56 + 48 + 1]
    )

    assert tick == 500  # 5.0 s * 100 tps
    assert x == pytest.approx(1.5)
    assert y == pytest.approx(2.5)
    assert z == pytest.approx(0.72)
    assert qw == pytest.approx(1.0)
    assert vx == pytest.approx(0.5)
    assert wz == pytest.approx(0.1)
    assert valid == 1
    assert dummy_link.poses_returned == 1


def test_path_cannot_erase_odometry_velocity_at_same_stamp():
    from types import SimpleNamespace as NS

    robot = fll.RobotIO.__new__(fll.RobotIO)
    velocity = (1.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    robot.estimate = (100, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), velocity)
    header = NS(stamp=NS(sec=0, nanosec=100))
    pose = NS(
        position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(w=1.0, x=0.0, y=0.0, z=0.0)
    )
    robot.on_path(NS(header=header, poses=[NS(header=header, pose=pose)]))
    assert robot.estimate[-1] == velocity
    header.stamp.nanosec = 101
    robot.on_path(NS(header=header, poses=[NS(header=header, pose=pose)]))
    assert robot.estimate[0] == 101
    assert robot.estimate[-1] == (0.0,) * 6


@pytest.mark.parametrize("subscribers", [0, 1])
@pytest.mark.parametrize("raw_subscribers", [0, 1])
def test_camera_compresses_only_on_demand_and_preserves_rgb(
    monkeypatch, subscribers, raw_subscribers
):
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    import zlib

    def message():
        return NS(header=NS(stamp=NS(sec=0, nanosec=0)))

    for name in ("Image", "CompressedImage", "CameraInfo"):
        monkeypatch.setattr(fll, name, message)
    encoder = Mock(wraps=fll.png_encode_rgb)
    monkeypatch.setattr(fll, "png_encode_rgb", encoder)
    pubs = NS(frame_id="camera", color_compressed=Mock(), color_raw=Mock(), info=Mock())
    pubs.color_compressed.get_subscription_count.return_value = subscribers
    pubs.color_raw.get_subscription_count.return_value = raw_subscribers
    fll.FastLivoLink.publish_frame(None, pubs, 1, 1, b"\xff\x00\x00", 60.0, 1, 2)
    assert encoder.call_count == subscribers
    assert pubs.color_compressed.publish.call_count == subscribers
    assert pubs.color_raw.publish.call_count == raw_subscribers
    pubs.info.publish.assert_called_once()
    if raw_subscribers:
        raw = pubs.color_raw.publish.call_args.args[0]
        assert raw.encoding == "bgr8" and raw.data == b"\x00\x00\xff"
    if subscribers:
        compressed = pubs.color_compressed.publish.call_args.args[0]
        assert compressed.format == "rgb8; png compressed rgb8"
        start = compressed.data.index(b"IDAT")
        length = struct.unpack(">I", compressed.data[start - 4 : start])[0]
        assert (
            zlib.decompress(compressed.data[start + 4 : start + 4 + length])
            == b"\x00\xff\x00\x00"
        )
