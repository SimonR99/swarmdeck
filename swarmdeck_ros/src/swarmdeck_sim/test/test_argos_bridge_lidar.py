"""Unit tests for ARGoS bridge-side vectorized LiDAR scan projection.

Tests:
1. `project_laserscan_slice`: horizontal planar slice for SLAM Toolbox.
   - Preserves points in the horizontal band [-0.05, 0.05] m.
   - Rejects points outside the band.
   - Respects min/max range limits.
   - Bins into 360 angular bins (-pi to +pi).
2. `project_laserscan_proximity`: 2.5D obstacle projection for Nav2 costmaps.
   - Preserves obstacles above a conservatively capped ground-return band.
   - Rejects low ground returns and overhead structures (> 1.80 m).
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
from spawn_fleet import robot_spec


def test_empty_cloud_returns_all_inf():
    empty = np.empty(0, dtype=bridge.LIDAR_DTYPE)
    slice_scan = bridge.project_laserscan_slice(empty)
    assert len(slice_scan) == bridge.SCAN_BEAMS
    assert np.all(np.isinf(slice_scan))

    prox_scan = bridge.project_laserscan_proximity(
        empty,
        lidar_x=-0.07,
        lidar_z=0.402,
        base_height=0.138,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
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
    """Observed ground at z_floor = 0.05 m stays below the climb threshold."""
    lidar_x, lidar_z, base_height = -0.07, 0.402, 0.138
    # z_sensor = z_floor - lidar_z - base_height
    z_ground = 0.05 - lidar_z - base_height

    pts = np.zeros(1, dtype=bridge.LIDAR_DTYPE)
    pts[0]["x"] = 2.07  # x_b = 2.0
    pts[0]["y"] = 0.0
    pts[0]["z"] = z_ground
    pts[0]["hit"] = 1

    prox = bridge.project_laserscan_proximity(
        pts,
        lidar_x,
        lidar_z,
        base_height,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
    )
    assert np.all(np.isinf(prox))


def test_proximity_detects_low_and_high_obstacles():
    """Obstacles above an explicit 0.10 m filter through 1.80 m are detected."""
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
        pts,
        lidar_x,
        lidar_z,
        base_height,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
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
        pts,
        lidar_x,
        lidar_z,
        base_height,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
    )
    assert np.all(np.isinf(prox))


def test_proximity_keeps_max_height_and_filters_observed_floor():
    physical = robot_spec("bunker")
    projection = bridge.proximity_spec("bunker")
    args = (
        projection["lidar_x"],
        projection["lidar_z"],
        projection["base_height"],
    )

    def scan_at(height):
        point = np.zeros(1, dtype=bridge.LIDAR_DTYPE)
        point[0]["x"] = 2.0 - physical.lidar_x
        point[0]["z"] = height - physical.lidar_z - physical.base_height
        point[0]["hit"] = 1
        return bridge.project_laserscan_proximity(
            point,
            *args,
            prox_min_height=projection["prox_min_height"],
            prox_range_max=projection["prox_range_max"],
        )

    ahead = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))
    assert np.all(np.isinf(scan_at(0.0)))
    assert scan_at(bridge.PROX_MAX_HEIGHT)[ahead] == pytest.approx(2.0, abs=0.01)
    assert np.all(np.isinf(scan_at(bridge.PROX_MAX_HEIGHT + 0.01)))


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
        pts,
        lidar_x,
        lidar_z,
        base_height,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
    )
    assert np.isclose(prox[angle_0_bin], 1.8, atol=0.01)


@pytest.mark.parametrize(
    ("platform", "climbable_height", "blocking_height"),
    [
        ("bunker", 0.15, 0.16),
        ("scout_mini", 0.15, 0.16),
    ],
)
def test_proximity_uses_canonical_mount_and_bounded_ground_filter(
    platform, climbable_height, blocking_height
):
    """The small-platform limit bounds filtering; the next centimetre blocks."""
    physical = robot_spec(platform)
    projection = bridge.proximity_spec(platform)
    assert projection == {
        "lidar_x": physical.lidar_x,
        "lidar_z": physical.lidar_z,
        "base_height": physical.base_height,
        "prox_min_height": min(physical.max_step_height, bridge.PROX_GROUND_FILTER_CAP)
        + bridge.PROX_HEIGHT_EPSILON,
        "prox_range_max": physical.prox_range_max,
    }

    points = np.zeros(2, dtype=bridge.LIDAR_DTYPE)
    # Raw hits are in the lidar frame. The first exceeds this platform's step
    # capability; the second sits exactly on its climbable limit.
    for point, height in zip(points, (blocking_height, climbable_height)):
        point["x"] = 2.0 - physical.lidar_x
        point["z"] = height - physical.lidar_z - physical.base_height
        point["hit"] = 1

    args = (
        projection["lidar_x"],
        projection["lidar_z"],
        projection["base_height"],
    )
    blocking = bridge.project_laserscan_proximity(
        points[:1],
        *args,
        prox_min_height=projection["prox_min_height"],
        prox_range_max=projection["prox_range_max"],
    )
    climbable = bridge.project_laserscan_proximity(
        points[1:],
        *args,
        prox_min_height=projection["prox_min_height"],
        prox_range_max=projection["prox_range_max"],
    )
    ahead = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))
    assert blocking[ahead] == pytest.approx(2.0, abs=0.01)
    assert np.all(np.isinf(climbable))


def test_spot_proximity_keeps_low_robots_and_uncertified_steps_visible():
    """Spot's 0.30 m capability does not make every low return traversable."""
    physical = robot_spec("spot")
    projection = bridge.proximity_spec("spot")
    assert projection["prox_min_height"] == pytest.approx(
        bridge.PROX_GROUND_FILTER_CAP + bridge.PROX_HEIGHT_EPSILON
    )

    def scan_at(height):
        point = np.zeros(1, dtype=bridge.LIDAR_DTYPE)
        point[0]["x"] = 2.0 - physical.lidar_x
        point[0]["z"] = height - physical.lidar_z - physical.base_height
        point[0]["hit"] = 1
        return bridge.project_laserscan_proximity(
            point,
            projection["lidar_x"],
            projection["lidar_z"],
            projection["base_height"],
            prox_min_height=projection["prox_min_height"],
            prox_range_max=projection["prox_range_max"],
        )

    ahead = int(np.floor((0.0 - bridge.SCAN_ANGLE_MIN) * bridge.INV_ANGLE_INC))
    assert np.all(np.isinf(scan_at(0.15)))
    for height in (0.20, 0.245, 0.30, 0.31):
        assert scan_at(height)[ahead] == pytest.approx(2.0, abs=0.01)


def test_bridge_loads_each_fleet_platform_from_canonical_profiles(tmp_path):
    config = tmp_path / "fleet.yaml"
    config.write_text("""fleet:
  robot_count: 3
  robot_prefix: robot_
  robot_type: bunker
  robot_types:
    robot_1: scout_mini
    robot_2: spot
""")

    loaded = bridge.ArgosBridge._load_robot_specs(str(config))
    assert loaded == {
        f"robot_{index}": bridge.proximity_spec(platform)
        for index, platform in enumerate(("bunker", "scout_mini", "spot"))
    }
