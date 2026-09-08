"""Capture-time joins and calibrated raw-pixel projection without ROS/OpenCV."""

from types import SimpleNamespace as NS
from unittest.mock import Mock
import sys

import numpy as np
import pytest

from adapters.map_color import CameraColorizer, MapColorMixin
from adapters.reconstruction import colorize_points


def header(stamp=10.0, frame="optical"):
    return NS(frame_id=frame, stamp=NS(to_sec=lambda: stamp))


def calibration(frame="optical"):
    return NS(
        header=header(frame=frame),
        width=3,
        height=3,
        K=[1, 0, 1, 0, 1, 1, 0, 0, 1],
        D=[0.0] * 5,
        distortion_model="plumb_bob",
    )


@pytest.fixture
def decoder(monkeypatch):
    decode = Mock(return_value=np.full((3, 3, 3), [10, 30, 200], dtype=np.uint8))
    monkeypatch.setitem(sys.modules, "cv2", NS(imdecode=decode, IMREAD_COLOR=1))
    return decode


def identity(_header):
    return NS(
        transform=NS(translation=NS(x=0, y=0, z=0), rotation=NS(x=0, y=0, z=0, w=1))
    )


def test_delayed_scan_uses_nearest_frame_and_caches_decode(decoder):
    color = CameraColorizer({"enabled": True})
    color.push(b"capture", header(10.02))
    color.push(b"latest", header(10.65))
    lookup = Mock(side_effect=identity)
    pts = np.array([[0, 0, 1], [0, 0, 2], [0, 0, -1]])
    rgba = color.colorize(pts, header(10), calibration(), lookup)
    assert rgba.tolist() == [[200, 30, 10, 255], [148, 148, 148, 0], [148, 148, 148, 0]]
    assert lookup.call_args.args[0].stamp.to_sec() == 10.02
    assert bytes(decoder.call_args.args[0]) == b"capture"
    color.colorize(pts, header(10), calibration(), lookup)
    assert decoder.call_count == 1
    assert color.colorize(pts, header(9), calibration(), lookup) is None
    assert lookup.call_count == 2


@pytest.mark.parametrize(
    "info_frame,override,ok",
    [
        ("optical", "", True),
        ("", "", False),
        ("", "optical", True),
        ("depth", "optical", False),
        ("optical", "wrong", False),
    ],
)
def test_calibration_frame_requires_explicit_empty_frame_override(
    decoder, info_frame, override, ok
):
    color = CameraColorizer({"enabled": True, "camera_frame": override})
    color.push(b"image", header())
    result = color.colorize(
        np.array([[0, 0, 1]]), header(), calibration(info_frame), identity
    )
    assert (result is not None) == ok


def test_history_is_bounded_in_time_count_and_bytes_and_handles_clock_reset():
    color = CameraColorizer({"enabled": True})
    for i in range(100):
        color.push(b"x", header(10 + i * 0.01))
    assert len(color._images) == 60
    color.push(b"y", header(15))
    assert len(color._images) == 1
    for i in range(20):
        color.push(b"x" * 1_000_000, header(15 + i * 0.01))
    assert color._bytes <= 16 * 1024 * 1024
    color.push(b"reset", header(1))
    assert len(color._images) == 1
    assert color._bytes == 5
    disabled = CameraColorizer({})
    disabled.push(b"image", header())
    assert not disabled._images


def test_dimensions_and_historical_tf_failure_preserve_geometry(decoder):
    color = CameraColorizer({"enabled": True})
    color.push(b"image", header())
    info = calibration()
    info.width = 4
    lookup = Mock(side_effect=RuntimeError("no historical TF"))
    points = np.array([[0.0, 0, 1]])
    assert color.colorize(points, header(), info, lookup) is None
    lookup.assert_not_called()
    assert color.colorize(points, header(), calibration(), lookup) is None
    assert "historical TF" in color.last_status
    np.testing.assert_array_equal(points, [[0, 0, 1]])


@pytest.mark.parametrize(
    "model,d,u",
    [
        ("plumb_bob", [1, 0, 0, 0, 0], 5),
        ("rational_polynomial", [1, 0, 0, 0, 0, 1, 0, 0], 3),
        ("plumb_bob", [0, 0, 0, 0.5, 0], 6),
    ],
)
def test_raw_distortion_samples_the_correct_pixel(model, d, u):
    rgb = np.zeros((9, 9, 3), dtype=np.uint8)
    rgb[1, u] = [250, 100, 20]
    colors, mask = colorize_points(
        np.array([[1.0, 0, 1]]),
        rgb,
        np.array([[2.0, 0, 1], [0, 2, 1], [0, 0, 1]]),
        np.eye(4),
        distortion=d,
        distortion_model=model,
    )
    assert mask.tolist() == [True]
    assert colors[0].tolist() == [250, 100, 20]


@pytest.mark.parametrize("d", [[1, 0, 0, 0], [0, 0, 0, 0]])
def test_unsupported_distortion_does_not_silently_use_pinhole(d):
    with pytest.raises(ValueError, match="unsupported"):
        colorize_points(
            np.array([[0, 0, 1]]),
            np.zeros((3, 3, 3), dtype=np.uint8),
            np.eye(3),
            np.eye(4),
            distortion=d,
            distortion_model="equidistant",
        )


def test_keyframe_projection_restores_full_map_pose_only_when_called():
    bridge = MapColorMixin()
    bridge._map_color = NS(enabled=True)
    bridge._colorize_map_rgba = Mock(return_value=np.array([[1, 2, 3, 255]]))
    # 90 degree yaw + translation: [1,0,0] base -> [2,4,4] map.
    project = bridge._keyframe_colorizer(
        np.array([2, 3, 4, 0, 0, 2**-0.5, 2**-0.5]), header()
    )
    bridge._colorize_map_rgba.assert_not_called()
    assert project(np.array([[1, 0, 0]])).tolist() == [[1, 2, 3, 255]]
    np.testing.assert_allclose(bridge._colorize_map_rgba.call_args.args[0], [[2, 4, 4]])


def test_botman_calibration_export_removes_driver_optical_rotation():
    from scripts.calibration.run_botman_calibration import camera_mount_for_deployment

    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    raw_from_base = np.diag([-1.0, -1.0, 1.0])
    result = camera_mount_for_deployment(
        {
            "R_lidar_cam": raw_from_base @ optical,
            "translation": np.array([-0.089, 0.010, -0.284]),
        }
    )
    np.testing.assert_allclose(result["rpy_rad"], [0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(result["translation"], [0.089, -0.010, -0.284])


def test_botman_mount_env_preserves_measured_optical_rotation():
    import math
    from pathlib import Path
    import re

    content = (Path(__file__).parents[2] / "deploy/robots/botman.env").read_text()

    def rotation(r, p, y):
        cr, sr, cp, sp, cy, sy = (
            math.cos(r),
            math.sin(r),
            math.cos(p),
            math.sin(p),
            math.cos(y),
            math.sin(y),
        )
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )

    values = [
        float(re.search(r"BOTMAN_OAK_" + key + r":=([^}]+)", content).group(1))
        for key in ("ROLL", "PITCH", "YAW")
    ]
    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    np.testing.assert_allclose(
        rotation(*values) @ optical, rotation(-1.5504, 0.0023, -1.5677), atol=1e-8
    )
