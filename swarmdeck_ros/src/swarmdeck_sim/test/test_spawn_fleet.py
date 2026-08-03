"""Fleet SDF rendering. Pure Python — no ROS, no Gazebo, so `make test` runs it."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))

from spawn_fleet import lidar_scan_fields, render  # noqa: E402


def test_single_ring_is_horizontal():
    fields = lidar_scan_fields(1)
    assert fields == {"LIDAR_RINGS": "1", "LIDAR_VMIN": "0", "LIDAR_VMAX": "0"}


def test_odd_ring_count_spans_the_vertical_fov():
    fields = lidar_scan_fields(9)
    assert fields["LIDAR_RINGS"] == "9"
    assert float(fields["LIDAR_VMIN"]) == pytest.approx(-0.26)
    assert float(fields["LIDAR_VMAX"]) == pytest.approx(0.26)


@pytest.mark.parametrize("rings", [2, 4, 8, 16])
def test_even_ring_counts_are_refused(rings):
    """An even count leaves no ring at elevation 0, so every ring is tilted and
    the sliced 2D scan truncates at short range. Fail loudly, not silently."""
    with pytest.raises(ValueError, match="even"):
        lidar_scan_fields(rings)


@pytest.mark.parametrize("rings", [1, 9, 17])
def test_render_substitutes_every_placeholder(rings):
    sdf = render("robot_0", "0.2 0.7 0.9", rings)
    assert "{{" not in sdf, "unsubstituted template placeholder left in the SDF"
    assert "robot_0/scan" in sdf
    assert f"<samples>{rings}</samples>" in sdf


def test_render_keeps_the_proximity_lidar_planar():
    """Only the mapping lidar gains rings; the bumper scan stays 2D."""
    sdf = render("robot_0", "0.2 0.7 0.9", 9)
    proximity = sdf.split('<sensor name="proximity_lidar"')[1]
    assert "<samples>1</samples>" in proximity.split("</sensor>")[0]


def test_imu_is_fast_enough_for_inertial_odometry():
    """LIO packages need >= ~100 Hz; below that they refuse or drift."""
    sdf = render("robot_0", "0.2 0.7 0.9", 1)
    imu = sdf.split('<sensor name="imu"')[1].split("</sensor>")[0]
    rate = int(imu.split("<update_rate>")[1].split("</update_rate>")[0])
    assert rate >= 100
