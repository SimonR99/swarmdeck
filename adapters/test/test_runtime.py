"""ROS-free regression tests for the shared adapter protocol runtime."""

from __future__ import annotations

import math
import types

import numpy as np
import pytest

from adapters.runtime import cloud_xyz, deep_merge, stamp_seconds, yaw_of


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


def test_cloud_xyz_honours_field_offsets_and_drops_nonfinite_rows():
    fields = [
        types.SimpleNamespace(name="intensity", offset=0),
        types.SimpleNamespace(name="x", offset=4),
        types.SimpleNamespace(name="y", offset=8),
        types.SimpleNamespace(name="z", offset=12),
    ]
    rows = np.zeros((2, 16), dtype=np.uint8)
    rows[0, 4:16] = np.frombuffer(np.array([1.0, -2.0, 3.0], dtype="<f4").tobytes(), dtype=np.uint8)
    rows[1, 4:16] = np.frombuffer(np.array([math.inf, 0.0, 1.0], dtype="<f4").tobytes(), dtype=np.uint8)
    msg = types.SimpleNamespace(fields=fields, data=rows.tobytes(), point_step=16)
    np.testing.assert_allclose(cloud_xyz(msg), [[1.0, -2.0, 3.0]])


def test_yaw_of_uses_planar_quaternion_component():
    q = types.SimpleNamespace(w=math.cos(math.pi / 4), z=math.sin(math.pi / 4), x=0.0, y=0.0)
    assert yaw_of(q) == pytest.approx(math.pi / 2)
