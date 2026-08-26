"""ROS-free regression tests for the shared adapter protocol runtime."""

from __future__ import annotations

import math
import types

import numpy as np
import pytest

from adapters.runtime import (
    cloud_xyz,
    deep_merge,
    map_cloud_height_limits,
    project_occupied_cloud,
    stamp_seconds,
    yaw_of,
)


def test_deep_merge_preserves_profile_siblings():
    merged = deep_merge(
        {"topics": {"odom": "odom", "map": "map"}},
        {"topics": {"odom": "wheel_odom"}},
    )
    assert merged == {"topics": {"odom": "wheel_odom", "map": "map"}}


def test_stamp_seconds_accepts_both_ros_timestamp_shapes():
    ros1 = types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 2.5))
    ros2 = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=2, nanosec=500_000_000)
    )
    assert stamp_seconds(ros1) == 2.5
    assert stamp_seconds(ros2) == 2.5
    assert stamp_seconds(types.SimpleNamespace(stamp=None)) is None


def test_map_cloud_height_limits_can_be_expressed_above_a_floor():
    assert map_cloud_height_limits(
        {
            "floor_z": -0.5,
            "min_z": 0.15,
            "max_z": 0.65,
        }
    ) == pytest.approx((-0.35, 0.15))


def test_map_cloud_height_limits_preserves_legacy_map_frame_profiles():
    assert map_cloud_height_limits({"min_z": -0.3, "max_z": 0.5}) == pytest.approx(
        (-0.3, 0.5)
    )


def test_project_occupied_cloud_keeps_unknown_cells_unknown():
    result = project_occupied_cloud(
        np.array([[1.0, 2.0], [1.1, 2.1], [math.nan, 4.0]]),
        resolution=0.5,
        padding_m=0.5,
    )
    assert result is not None
    resolution, width, height, origin_x, origin_y, cells = result
    assert resolution == pytest.approx(0.5)
    assert (width, height) == (3, 3)
    assert (origin_x, origin_y) == pytest.approx((0.5, 1.5))
    assert int((cells == 100).sum()) == 1
    assert int((cells == -1).sum()) == 8


def test_cloud_xyz_honours_field_offsets_and_drops_nonfinite_rows():
    fields = [
        types.SimpleNamespace(name="intensity", offset=0),
        types.SimpleNamespace(name="x", offset=4),
        types.SimpleNamespace(name="y", offset=8),
        types.SimpleNamespace(name="z", offset=12),
    ]
    rows = np.zeros((2, 16), dtype=np.uint8)
    rows[0, 4:16] = np.frombuffer(
        np.array([1.0, -2.0, 3.0], dtype="<f4").tobytes(), dtype=np.uint8
    )
    rows[1, 4:16] = np.frombuffer(
        np.array([math.inf, 0.0, 1.0], dtype="<f4").tobytes(), dtype=np.uint8
    )
    msg = types.SimpleNamespace(fields=fields, data=rows.tobytes(), point_step=16)
    np.testing.assert_allclose(cloud_xyz(msg), [[1.0, -2.0, 3.0]])


def test_yaw_of_uses_planar_quaternion_component():
    q = types.SimpleNamespace(
        w=math.cos(math.pi / 4), z=math.sin(math.pi / 4), x=0.0, y=0.0
    )
    assert yaw_of(q) == pytest.approx(math.pi / 2)
