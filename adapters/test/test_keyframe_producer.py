"""ROS-free tests for keyframe production: motion gate, drop queue, frame change."""

from __future__ import annotations

import math
from unittest.mock import patch

import numpy as np
import pytest

from adapters.keyframe_producer import (
    KeyframeUploader,
    mint_session,
    laser_scan_to_map_points,
    points_lidar_to_map,
    points_map_to_base,
    pose7_from_xy_yaw,
    voxel_downsample,
)
from swarmdeck_protocol import decode_keyframe, peek_keyframe_header


def _wall(n: int = 30) -> np.ndarray:
    xs, ys = np.meshgrid(np.linspace(1.0, 5.0, n), np.linspace(-2.0, 2.0, n))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 0.5)], axis=1).astype(
        np.float32
    )


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


def test_scan_novelty_captures_motion_when_reported_pose_is_frozen():
    """Low-quality odometry may report no motion while lidar geometry changes."""
    uploader = KeyframeUploader("r0", "http://backend", min_period_s=0.0)
    frozen = pose7_from_xy_yaw(0.0, 0.0, 0.0)
    assert uploader.consider(_wall(), frozen, 0.0)

    changed_view = _wall() + np.array([2.0, 0.0, 0.0], dtype=np.float32)
    assert uploader.consider(changed_view, frozen, 1.0)


def test_map_gauge_correction_does_not_look_like_physical_motion():
    """A loop closure shifts map pose and cloud together, not the body view."""
    uploader = KeyframeUploader("r0", "http://backend", min_period_s=0.0)
    initial = pose7_from_xy_yaw(0.0, 0.0, 0.0)
    corrected = pose7_from_xy_yaw(5.0, -2.0, 0.0)
    assert uploader.consider(_wall(), initial, 0.0)

    map_shift = np.array([5.0, -2.0, 0.0], dtype=np.float32)
    assert not uploader.consider(_wall() + map_shift, corrected, 1.0)


def test_pose_gate_remains_available_when_scan_gate_is_disabled():
    uploader = KeyframeUploader(
        "r0", "http://backend", min_period_s=0.0, min_scan_change_m=0.0
    )
    initial = pose7_from_xy_yaw(0.0, 0.0, 0.0)
    moved = pose7_from_xy_yaw(1.0, 0.0, 0.0)
    assert uploader.consider(_wall(), initial, 0.0)
    assert uploader.consider(_wall(), moved, 1.0)


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


def test_every_keyframe_of_one_run_carries_the_same_session():
    """``seq`` alone cannot identify a keyframe -- it restarts at zero every
    time this process does, and the back-end drops the repeats as duplicates.
    The session is what makes ``(robot_id, session, seq)`` unique, so it has to
    be minted once and then never move for the life of the uploader."""
    uploader = KeyframeUploader("botman_0", "http://backend", min_period_s=0.0)
    assert uploader.session

    sessions = []
    for i in range(3):
        assert uploader.consider(
            _wall(), pose7_from_xy_yaw(float(i), 0.0, 0.0), float(i)
        )
        sessions.append(peek_keyframe_header(uploader._queue[-1])["session"])
    assert sessions == [uploader.session] * 3

    seqs = [decode_keyframe(blob).seq for blob in uploader._queue]
    assert seqs == sorted(set(seqs)), "seq must stay unique inside one session"


def test_a_restarted_uploader_mints_a_different_session():
    """The reboot case, which is the whole point: same robot, same seq
    counter starting again from zero, and the only thing that says so is this
    field."""
    first = KeyframeUploader("botman_0", "http://backend", min_period_s=0.0)
    assert first.consider(_wall(), pose7_from_xy_yaw(0.0, 0.0, 0.0), 0.0)
    second = KeyframeUploader("botman_0", "http://backend", min_period_s=0.0)
    assert second.consider(_wall(), pose7_from_xy_yaw(0.0, 0.0, 0.0), 100.0)

    a, b = decode_keyframe(first._queue[0]), decode_keyframe(second._queue[0])
    assert a.seq == b.seq == 0
    assert a.robot_id == b.robot_id
    assert a.session != b.session
    assert a.trajectory != b.trajectory


def test_a_minted_session_is_wire_legal():
    """It ends up in query strings and scope names, so the protocol restricts
    the characters -- an id this function minted must always pass."""
    from swarmdeck_protocol import MAX_SESSION_CHARS, encode_keyframe

    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")
    for _ in range(5):
        session = mint_session()
        assert len(session) <= MAX_SESSION_CHARS
        assert not set(session) - allowed
        encode_keyframe(
            robot_id="botman_0",
            seq=0,
            stamp=0.0,
            points=_wall(),
            t_odom_base=pose7_from_xy_yaw(0.0, 0.0, 0.0),
            session=session,
        )
    assert len({mint_session() for _ in range(50)}) == 50


def test_an_explicit_session_overrides_the_minted_one():
    uploader = KeyframeUploader(
        "botman_0", "http://backend", min_period_s=0.0, session="named-run"
    )
    assert uploader.consider(_wall(), pose7_from_xy_yaw(0.0, 0.0, 0.0), 0.0)
    assert decode_keyframe(uploader._queue[0]).session == "named-run"


def test_keyframe_carries_ground_relative_band_and_lidar_height():
    uploader = KeyframeUploader(
        "botman_0",
        "http://backend",
        min_period_s=0.0,
        height_band={"floor_z": -0.520, "min_z": 0.150, "max_z": 1.800},
        lidar_height_m=0.520,
    )
    pose = pose7_from_xy_yaw(0.0, 0.0, 0.0, z=0.1)
    assert uploader.consider(_wall(), pose, 0.0)

    packet = decode_keyframe(uploader._queue[0])
    assert packet.ground_z == pytest.approx(-0.620)
    assert packet.min_height == pytest.approx(0.150)
    assert packet.max_height == pytest.approx(1.800)
    assert packet.lidar_height == pytest.approx(0.520)
    header_band = peek_keyframe_header(uploader._queue[0])["height_band"]
    assert header_band["ground_z"] == pytest.approx(-0.620)
    assert header_band["min_height"] == pytest.approx(0.150)
    assert header_band["max_height"] == pytest.approx(1.800)
    assert header_band["lidar_height"] == pytest.approx(0.520)


def test_duplicate_stamps_do_not_disable_the_turn_gate():
    """A run of unusable stamps must not leave the gate with no reference.

    The gate used to adopt the current sample even when the interval was
    unusable, so consecutive duplicate stamps replaced the only baseline the
    next call had. A fast turn sampled that way reads as no turn at all, and
    the gate fails open exactly when it matters.
    """
    from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw

    u = KeyframeUploader("r", "http://x", max_yaw_rate=math.radians(8.0))
    # Establish a reference at 100.0, then turn 50 degrees while the stamp is
    # stuck, and coast the last half degree once it moves again. Adopting the
    # unusable samples leaves the final interval measuring 0.5 deg over 0.1 s,
    # a comfortable 5 deg/s, and the gate waves through a keyframe taken in the
    # middle of a 50 degree swing.
    assert not u._turning_too_fast(pose7_from_xy_yaw(0, 0, 0.0), 100.0)
    for i in range(1, 4):
        u._turning_too_fast(
            pose7_from_xy_yaw(0, 0, math.radians(-50.0 * i / 3.0)), 100.0
        )
    assert u.unusable_stamps == 3
    assert u._turning_too_fast(pose7_from_xy_yaw(0, 0, math.radians(-50.5)), 100.1)


def test_gate_stats_report_what_the_gate_did():
    """spun was counted and reported nowhere; a silent gate cannot be audited."""
    from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw

    u = KeyframeUploader("r", "http://x", max_yaw_rate=math.radians(8.0))
    for i in range(6):
        u._turning_too_fast(pose7_from_xy_yaw(0, 0, math.radians(-55.0 * i * 0.1)), 100.0 + i * 0.1)
    stats = u.gate_stats()
    assert stats["max_yaw_rate_deg_s"] == pytest.approx(8.0, abs=1e-6)
    assert stats["peak_yaw_rate_deg_s"] > 8.0
    assert stats["last_yaw_rate_deg_s"] == pytest.approx(55.0, abs=0.5)
    # Nothing was accepted here, only observed. The two must not be conflated:
    # a fast turn seen is the gate working, a fast turn accepted is the bug.
    assert stats["max_accepted_yaw_rate_deg_s"] == 0.0


def test_lidar_spec_refuses_a_bare_lidar_block():
    """Passing the lidar block instead of the fleet config used to disable the
    turn gate, via a planar default, in a different module entirely."""
    import sys
    from pathlib import Path

    sys.path.insert(
        0, str(Path(__file__).resolve().parents[2] / "swarmdeck_ros" / "src"
               / "swarmdeck_sim" / "scenario")
    )
    from spawn_fleet import lidar_spec

    assert lidar_spec({"lidar": {"profile": "vlp16"}}).rings == 17
    with pytest.raises(ValueError, match="fleet config, not the lidar block"):
        lidar_spec({"profile": "vlp16", "h_samples": 900})


def test_unjudgeable_capture_is_rejected_not_waved_through():
    """A capture the gate cannot judge must not be treated as slow enough.

    Returning False on an unusable interval read as "no rate, so not too fast",
    and the capture sailed past the only gate meant to judge it. Measured live
    with the gate reporting its own numbers: 7 to 12 unusable stamps per robot
    per interval, and captures accepted at 170.9 and 256.9 deg/s against an
    8 deg/s limit, which at the simulator's 100 ms capture lag is 17 to 25
    degrees of pose error apiece.
    """
    from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw

    u = KeyframeUploader("r", "http://x", max_yaw_rate=math.radians(8.0))
    assert not u._turning_too_fast(pose7_from_xy_yaw(0, 0, 0.0), 100.0)
    # Same stamp: no usable interval, so no way to know. Reject.
    assert u._turning_too_fast(pose7_from_xy_yaw(0, 0, math.radians(-30.0)), 100.0)
    assert u.unusable_stamps == 1


def test_gate_disabled_still_means_disabled():
    """max_yaw_rate <= 0 opts out entirely, including out of the fail-closed path."""
    from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw

    u = KeyframeUploader("r", "http://x", max_yaw_rate=0.0)
    assert not u._turning_too_fast(pose7_from_xy_yaw(0, 0, 0.0), 100.0)
    assert not u._turning_too_fast(pose7_from_xy_yaw(0, 0, math.radians(-90.0)), 100.0)


@pytest.mark.parametrize("repeated_stamp", [100.1, 100.0, 100.1005])
def test_rejected_turn_cannot_be_uploaded_on_a_repeated_stamp(repeated_stamp):
    uploader = KeyframeUploader(
        "r",
        "http://x",
        min_period_s=0,
        min_scan_change_m=0,
        min_yaw_rad=math.radians(1),
        max_yaw_rate=math.radians(8),
    )
    assert uploader.consider(_wall(), pose7_from_xy_yaw(0, 0, 0), 100.0)
    turned = pose7_from_xy_yaw(0, 0, math.radians(30))
    assert not uploader.consider(_wall(), turned, 100.1)
    assert not uploader.consider(_wall(), turned, repeated_stamp)
    assert uploader.pending() == 1
    assert uploader.gate_stats()["max_accepted_yaw_rate_deg_s"] <= 8
    # A fresh scan after stopping remains usable; reject does not latch forever.
    assert uploader.consider(_wall(), turned, 100.2)
    assert decode_keyframe(uploader._queue[-1]).stamp == 100.2


@pytest.mark.parametrize("stamp", [float("nan"), float("inf"), -float("inf")])
def test_invalid_stamp_does_not_poison_the_turn_reference(stamp):
    uploader = KeyframeUploader("r", "http://x", min_period_s=0)
    assert not uploader.consider(_wall(), pose7_from_xy_yaw(0, 0, 0), stamp)
    assert uploader.consider(_wall(), pose7_from_xy_yaw(0, 0, 0), 100)
    assert not uploader.consider(_wall(), pose7_from_xy_yaw(0, 0, 1), 100.1)


def test_camera_colors_follow_voxelized_wire_points():
    from swarmdeck_protocol import Descriptor, encode_keyframe, ProtocolError

    points = np.array([[1, 2, 3], [1000, 0, 0], [4, 5, 6]], dtype=np.float32)
    colors = np.array(
        [[255, 20, 1, 255], [0, 255, 0, 255], [148, 148, 148, 0]], dtype=np.uint8
    )
    descriptor = Descriptor("test", np.array([[1, 2], [3, 4]], dtype=np.uint8), 80)
    packet = decode_keyframe(
        encode_keyframe(
            robot_id="r",
            seq=1,
            stamp=10,
            points=points,
            t_odom_base=pose7_from_xy_yaw(0, 0, 0),
            descriptor=descriptor,
            colors=colors,
        )
    )
    np.testing.assert_array_equal(packet.colors, colors[[0, 2]])
    np.testing.assert_array_equal(packet.descriptor.data, descriptor.data)
    with pytest.raises(ProtocolError, match="colors"):
        encode_keyframe(
            robot_id="r",
            seq=1,
            stamp=10,
            points=points,
            t_odom_base=pose7_from_xy_yaw(0, 0, 0),
            colors=colors[:1],
        )


def test_color_projection_runs_only_after_capture_gates():
    calls = []

    def colorize(points):
        calls.append(len(points))
        return np.tile(np.array([255, 0, 0, 255], dtype=np.uint8), (len(points), 1))

    uploader = KeyframeUploader("r", "http://unused", min_period_s=0)
    pose = pose7_from_xy_yaw(0, 0, 0)
    assert uploader.consider(_wall(), pose, 100, colorize=colorize)
    packet = decode_keyframe(uploader._queue[0])
    assert packet.colors.shape == (len(packet.points), 4)
    assert not uploader.consider(_wall(), pose, 101, colorize=colorize)
    assert len(calls) == 1


@pytest.mark.parametrize(
    "bad_colors", [[], np.zeros((1, 4), dtype=np.uint8), np.zeros((1, 3))]
)
def test_invalid_optional_colors_do_not_drop_geometry(bad_colors):
    uploader = KeyframeUploader("r", "http://unused", min_period_s=0)
    assert uploader.consider(
        _wall(), pose7_from_xy_yaw(0, 0, 0), 100, colorize=lambda _: bad_colors
    )
    packet = decode_keyframe(uploader._queue[0])
    assert len(packet.points) > 0
    assert packet.colors is None


def test_period_gate_updates_turn_reference_without_processing_the_cloud():
    uploader = KeyframeUploader("r", "http://unused", min_period_s=2)
    wall = _wall()
    with patch("adapters.keyframe_producer.time.monotonic", return_value=0):
        assert uploader.consider(wall, pose7_from_xy_yaw(0, 0, 0), 100)
    with patch("adapters.keyframe_producer.voxel_downsample") as downsample:
        with patch("adapters.keyframe_producer.time.monotonic", return_value=1.9):
            assert not uploader.consider(wall, pose7_from_xy_yaw(0, 0, 0), 101.9)
        with patch("adapters.keyframe_producer.time.monotonic", return_value=2):
            assert not uploader.consider(
                wall, pose7_from_xy_yaw(0, 0, math.radians(1)), 102
            )
        downsample.assert_not_called()
    assert uploader.pending() == 1


def test_ground_ring_does_not_mask_changing_walls():
    """A flat street moves through the sensor but its near-ground ring is constant."""
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    ground = np.column_stack(
        (np.cos(angles), np.sin(angles), np.full(720, -0.3))
    ).astype(np.float32)
    walls = np.column_stack(
        (5 * np.cos(angles), 5 * np.sin(angles), np.full(720, 0.5))
    ).astype(np.float32)
    first = np.vstack((ground, walls))
    second = np.vstack((ground, walls + np.array([1.0, 0.0, 0.0], dtype=np.float32)))
    uploader = KeyframeUploader(
        "r0",
        "http://backend",
        min_period_s=0.0,
        height_band={"floor_z": -0.3, "min_z": 0.15, "max_z": 1.8},
    )
    pose = pose7_from_xy_yaw(0.0, 0.0, 0.0)
    assert uploader.consider(first, pose, 1.0)
    packet = decode_keyframe(uploader._queue[-1])
    assert np.any(packet.points[:, 2] < 0.0)  # Ground stays in the uploaded map.
    assert not uploader.consider(first, pose, 2.0)
    assert uploader.consider(second, pose, 3.0)
    # Translating the map and pose together still represents the same observation.
    shift = np.array([2.0, 3.0, 0.2], dtype=np.float32)
    corrected = pose.copy()
    corrected[:3] += shift
    # floor_z is map-relative, so the map's ground reference moves with the gauge.
    uploader._height_band["floor_z"] += 0.2
    assert not uploader.consider(second + shift, corrected, 4.0)


def test_slow_simulation_keeps_capture_period_in_sensor_time():
    uploader = KeyframeUploader("r", "http://unused", min_period_s=2)
    pose = pose7_from_xy_yaw(0, 0, 0)
    with patch("adapters.keyframe_producer.time.monotonic", return_value=0):
        assert uploader.consider(_wall(), pose, 100)
    changed = _wall() + np.array([2, 0, 0])
    with patch("adapters.keyframe_producer.time.monotonic", return_value=10):
        assert not uploader.consider(changed, pose, 100.5)
    with patch("adapters.keyframe_producer.time.monotonic", return_value=11):
        assert uploader.consider(changed, pose, 102)
