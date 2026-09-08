"""Unit tests for ARGoS bridge-side vectorized LiDAR scan projection.

Tests:
1. `project_laserscan_slice`: horizontal planar slice for SLAM Toolbox.
   - Preserves points in the horizontal band [-0.05, 0.05] m.
   - Rejects points outside the band.
   - Respects min/max range limits.
   - Bins into 360 angular bins (-pi to +pi).
2. `project_laserscan_proximity`: 2.5D obstacle projection for Nav2 costmaps.
   - Preserves obstacles in height band [0.15, 1.80] m above ground.
   - Rejects ground returns (< 0.15 m) and overhead structures (> 1.80 m).
   - Accurately applies extrinsic translation to base_link frame.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

NODES_DIR = Path(__file__).resolve().parents[1] / "nodes"
if str(NODES_DIR) not in sys.path:
    sys.path.insert(0, str(NODES_DIR))

import swarmdeck_argos_bridge as bridge


def test_empty_cloud_returns_all_inf():
    empty = np.empty(0, dtype=bridge.LIDAR_DTYPE)
    slice_scan = bridge.project_laserscan_slice(empty)
    assert len(slice_scan) == bridge.SCAN_BEAMS
    assert np.all(np.isinf(slice_scan))

    prox_scan = bridge.project_laserscan_proximity(
        empty, lidar_x=-0.07, lidar_z=0.402, base_height=0.138, prox_range_max=8.0
    )
    assert len(prox_scan) == bridge.SCAN_BEAMS
    assert np.all(np.isinf(prox_scan))


def test_slice_band_keeps_horizontal_rejects_elevated():
    pts = np.zeros(2, dtype=bridge.LIDAR_DTYPE)
    # Point 0: 3.0 m ahead at z = 0.0 (horizontal plane)
    pts[0]["x"] = 3.0
    pts[0]["y"] = 0.0
    pts[0]["z"] = 0.0
    pts[0]["hit"] = 1

    # Point 1: 3.0 m ahead at z = 0.2 m (above slice band)
    pts[1]["x"] = 3.0
    pts[1]["y"] = 0.0
    pts[1]["z"] = 0.2
    pts[1]["hit"] = 1

    # Bin for angle 0 (ahead)
    angle_0_bin = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))

    ranges = bridge.project_laserscan_slice(pts, range_max=30.0)
    assert np.isclose(ranges[angle_0_bin], 3.0)

    # Test when only point 1 exists (z = 0.2)
    ranges_elevated = bridge.project_laserscan_slice(pts[1:2], range_max=30.0)
    assert np.all(np.isinf(ranges_elevated))


def test_slice_range_limits():
    pts = np.zeros(2, dtype=bridge.LIDAR_DTYPE)
    # Point 0: closer than range_min (0.3 m < 0.45 m)
    pts[0]["x"] = 0.3
    pts[0]["y"] = 0.0
    pts[0]["z"] = 0.0
    pts[0]["hit"] = 1

    # Point 1: beyond range_max (35.0 m > 30.0 m)
    pts[1]["x"] = 35.0
    pts[1]["y"] = 0.0
    pts[1]["z"] = 0.0
    pts[1]["hit"] = 1

    ranges = bridge.project_laserscan_slice(pts, range_max=30.0)
    assert np.all(np.isinf(ranges))


def test_proximity_ground_filter():
    """Ground plane at z_floor = 0.05 m must be rejected (< 0.15 m threshold)."""
    lidar_x, lidar_z, base_height = -0.07, 0.402, 0.138
    # z_sensor = z_floor - lidar_z - base_height
    z_ground = 0.05 - lidar_z - base_height

    pts = np.zeros(1, dtype=bridge.LIDAR_DTYPE)
    pts[0]["x"] = 2.07  # x_b = 2.0
    pts[0]["y"] = 0.0
    pts[0]["z"] = z_ground
    pts[0]["hit"] = 1

    prox = bridge.project_laserscan_proximity(
        pts, lidar_x, lidar_z, base_height, prox_range_max=8.0
    )
    assert np.all(np.isinf(prox))


def test_proximity_detects_low_and_high_obstacles():
    """Obstacles between 0.15 and 1.80 m above ground must be detected."""
    lidar_x, lidar_z, base_height = -0.07, 0.402, 0.138
    angle_0_bin = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))

    pts = np.zeros(2, dtype=bridge.LIDAR_DTYPE)
    # Obstacle 1: Duck / bumper at 0.20 m above floor, 2.0 m ahead
    pts[0]["x"] = 2.0 - lidar_x
    pts[0]["y"] = 0.0
    pts[0]["z"] = 0.20 - lidar_z - base_height
    pts[0]["hit"] = 1

    # Obstacle 2: Tall wall at 1.50 m above floor, 4.0 m to the left (+y)
    pts[1]["x"] = 0.0 - lidar_x
    pts[1]["y"] = 4.0
    pts[1]["z"] = 1.50 - lidar_z - base_height
    pts[1]["hit"] = 1

    angle_left_bin = int(
        np.floor((math.pi / 2.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC)
    )

    prox = bridge.project_laserscan_proximity(
        pts, lidar_x, lidar_z, base_height, prox_range_max=8.0
    )
    assert np.isclose(prox[angle_0_bin], 2.0, atol=0.01)
    assert np.isclose(prox[angle_left_bin], 4.0, atol=0.01)


def test_proximity_rejects_overhead_obstacles():
    """Overhead ceiling / beams at > 1.80 m above ground must be rejected."""
    lidar_x, lidar_z, base_height = -0.07, 0.402, 0.138

    pts = np.zeros(1, dtype=bridge.LIDAR_DTYPE)
    pts[0]["x"] = 2.0 - lidar_x
    pts[0]["y"] = 0.0
    pts[0]["z"] = 2.20 - lidar_z - base_height
    pts[0]["hit"] = 1

    prox = bridge.project_laserscan_proximity(
        pts, lidar_x, lidar_z, base_height, prox_range_max=8.0
    )
    assert np.all(np.isinf(prox))


def test_proximity_minimum_distance_aggregation():
    """When multiple obstacles exist along the same bearing, the nearest wins."""
    lidar_x, lidar_z, base_height = -0.07, 0.402, 0.138
    angle_0_bin = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))

    pts = np.zeros(2, dtype=bridge.LIDAR_DTYPE)
    # Far obstacle at 5.0 m ahead
    pts[0]["x"] = 5.0 - lidar_x
    pts[0]["y"] = 0.0
    pts[0]["z"] = 0.5 - lidar_z - base_height
    pts[0]["hit"] = 1

    # Near obstacle at 1.8 m ahead
    pts[1]["x"] = 1.8 - lidar_x
    pts[1]["y"] = 0.0
    pts[1]["z"] = 0.3 - lidar_z - base_height
    pts[1]["hit"] = 1

    prox = bridge.project_laserscan_proximity(
        pts, lidar_x, lidar_z, base_height, prox_range_max=8.0
    )
    assert np.isclose(prox[angle_0_bin], 1.8, atol=0.01)
