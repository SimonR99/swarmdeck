"""ROS-free tests for keyframe production: motion gate, drop queue, frame change."""

from __future__ import annotations

import math
from unittest.mock import patch

import numpy as np
import pytest

from adapters.keyframe_producer import (
    KeyframeUploader,
    laser_scan_to_map_points,
    points_lidar_to_map,
    points_map_to_base,
    pose7_from_xy_yaw,
    voxel_downsample,
)
from swarmdeck_protocol import decode_keyframe, peek_keyframe_header


def _wall(n: int = 30) -> np.ndarray:
    xs, ys = np.meshgrid(np.linspace(1.0, 5.0, n), np.linspace(-2.0, 2.0, n))
    return np.stack(
        [xs.ravel(), ys.ravel(), np.full(xs.size, 0.5)], axis=1
    ).astype(np.float32)


def test_peek_header_does_not_require_a_valid_body():
    from swarmdeck_protocol import encode_keyframe

    blob = encode_keyframe(
        robot_id="botman_0",
        seq=3,
        stamp=1.5,
        points=_wall(),
        t_odom_base=pose7_from_xy_yaw(1.0, 2.0, 0.3),
    )
    header = peek_keyframe_header(blob)
    assert header["robot_id"] == "botman_0"
    assert header["seq"] == 3
    packet = decode_keyframe(blob)
    assert packet.robot_id == "botman_0"
    np.testing.assert_allclose(packet.t_odom_base[:2], [1.0, 2.0], atol=1e-6)


def test_map_to_base_puts_the_sensor_origin_at_zero():
    pose = pose7_from_xy_yaw(3.0, 4.0, math.pi / 2)
    origin = np.array([[3.0, 4.0, 0.0]], dtype=np.float32)
    base = points_map_to_base(origin, pose)
    np.testing.assert_allclose(base[0], [0.0, 0.0, 0.0], atol=1e-5)


def test_voxel_downsample_collapses_points_in_one_cell():
    pts = np.zeros((50, 3), dtype=np.float32)
    pts[:, 0] = np.linspace(0.0, 0.04, 50)
    out = voxel_downsample(pts, 0.2)
    assert out.shape[0] == 1


def test_motion_gate_skips_a_parked_robot():
    uploader = KeyframeUploader("r0", "http://backend", min_period_s=0.0)
    pose = pose7_from_xy_yaw(0.0, 0.0, 0.0)
    assert uploader.consider(_wall(), pose, 0.0)
    assert not uploader.consider(_wall(), pose, 1.0)
    moved = pose7_from_xy_yaw(1.0, 0.0, 0.0)
    assert uploader.consider(_wall(), moved, 2.0)


def test_full_queue_drops_the_oldest_and_never_blocks():
    uploader = KeyframeUploader(
        "r0", "http://backend", queue_size=2, min_period_s=0.0, min_translation_m=0.1
    )
    for i in range(5):
        pose = pose7_from_xy_yaw(float(i), 0.0, 0.0)
        uploader.consider(_wall(), pose, float(i))
    assert uploader.pending() == 2
    assert uploader.dropped >= 3


def test_upload_one_posts_the_blob_and_identifies_the_robot():
    uploader = KeyframeUploader("botman_0", "http://backend", min_period_s=0.0)
    assert uploader.consider(_wall(), pose7_from_xy_yaw(0.0, 0.0, 0.0), 0.0)
    captured: dict = {}

    class _Resp:
        def read(self):
            return b"{}"

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def fake_urlopen(request, timeout=0):
        captured["url"] = request.full_url
        captured["body"] = request.data
        return _Resp()

    with patch("adapters.keyframe_producer.urllib.request.urlopen", fake_urlopen):
        assert uploader.upload_one()
    assert "robot_id=botman_0" in captured["url"]
    header = peek_keyframe_header(captured["body"])
    assert header["robot_id"] == "botman_0"
    assert uploader.pending() == 0


def test_lidar_points_land_in_the_map_frame_at_the_sensor_pose():
    pose = (2.0, 3.0, math.pi / 2)
    origin = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    mapped = points_lidar_to_map(origin, pose, lidar_x=0.15, lidar_z=0.4)
    # lidar_x along base x, yaw +90: world +y.
    np.testing.assert_allclose(mapped[0], [2.0, 3.15, 0.4], atol=1e-5)


def test_a_planar_scan_becomes_a_thickened_map_cloud():
    ranges = np.array([2.0, 2.0, 2.0], dtype=np.float64)
    points = laser_scan_to_map_points(
        ranges,
        angle_min=-0.1,
        angle_increment=0.1,
        range_min=0.1,
        range_max=10.0,
        pose_xy_yaw=(0.0, 0.0, 0.0),
        lidar_x=0.0,
        lidar_z=0.5,
        z_layers=(0.0, 0.12),
    )
    assert points.shape[0] == 6
    assert points.shape[1] == 3
    assert np.max(points[:, 0]) > 1.5
    zs = set(np.round(points[:, 2], 2).tolist())
    assert any(math.isclose(z, 0.5, abs_tol=0.02) for z in zs)
    assert any(math.isclose(z, 0.62, abs_tol=0.02) for z in zs)
