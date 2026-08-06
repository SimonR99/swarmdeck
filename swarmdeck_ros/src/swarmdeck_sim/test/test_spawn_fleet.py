"""Fleet SDF rendering. Pure Python — no ROS, no Gazebo, so `make test` runs it."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))

from spawn_fleet import (  # noqa: E402
    LIDAR_PROFILES,
    LidarSpec,
    lidar_spec,
    render,
)


def test_single_ring_is_horizontal():
    fields = LidarSpec(rings=1).fields()
    assert fields["LIDAR_RINGS"] == "1"
    assert float(fields["LIDAR_VMIN"]) == 0.0
    assert float(fields["LIDAR_VMAX"]) == 0.0


def test_odd_ring_count_spans_the_vertical_fov():
    fields = LidarSpec(rings=9, vfov=0.26).fields()
    assert fields["LIDAR_RINGS"] == "9"
    assert float(fields["LIDAR_VMIN"]) == pytest.approx(-0.26)
    assert float(fields["LIDAR_VMAX"]) == pytest.approx(0.26)


@pytest.mark.parametrize("rings", [2, 4, 8, 16])
def test_even_ring_counts_are_refused(rings):
    """An even count leaves no ring at elevation 0, so every ring is tilted and
    the sliced 2D scan truncates at short range. Fail loudly, not silently."""
    with pytest.raises(ValueError, match="even"):
        LidarSpec(rings=rings, vfov=0.26)


def test_single_ring_with_a_vertical_fov_is_refused():
    """Self-contradictory: one sample spread over a non-zero span is still one
    horizontal ring, so the config would not mean what it says."""
    with pytest.raises(ValueError, match="horizontal"):
        LidarSpec(rings=1, vfov=0.26)


@pytest.mark.parametrize("rings", [1, 9, 17])
def test_render_substitutes_every_placeholder(rings):
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(rings=rings, vfov=0.26 * (rings > 1)))
    assert "{{" not in sdf, "unsubstituted template placeholder left in the SDF"
    assert "robot_0/scan" in sdf
    assert f"<samples>{rings}</samples>" in sdf


def test_render_keeps_the_proximity_lidar_planar():
    """Only the mapping lidar gains rings; the bumper scan stays 2D."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(rings=9, vfov=0.26))
    proximity = sdf.split('<sensor name="proximity_lidar"')[1]
    assert "<samples>1</samples>" in proximity.split("</sensor>")[0]


def test_horizontal_resolution_reaches_the_far_wall():
    """The default profile must put adjacent rays inside one 5 cm grid cell
    across the 24 m building, or distant walls come out dotted — the defect this
    whole profile mechanism exists to fix (docs/KNOWN_ISSUES.md #8)."""
    spec = lidar_spec({})
    spacing_at_12m = 12.0 * math.radians(spec.h_step_deg)
    assert spacing_at_12m < 0.05, (
        f"{spec.h_samples} samples/rev = {spec.h_step_deg:.3f} deg leaves "
        f"{spacing_at_12m * 100:.1f} cm between rays at 12 m"
    )


def test_legacy_profile_reproduces_the_shipped_sensor():
    """Kept as the A/B control for measuring the change, so it must not drift."""
    spec = LIDAR_PROFILES["legacy_360"]
    assert (spec.h_samples, spec.rings, spec.range_max) == (360, 1, 16.0)
    assert spec.h_step_deg == pytest.approx(1.003, abs=1e-3)


def test_lidar_rings_is_still_honoured_as_a_config_alias():
    """Pre-profile study configs must keep working."""
    assert lidar_spec({"lidar_rings": 9}).rings == 9
    assert lidar_spec({"lidar": {"profile": "generic_32"}, "lidar_rings": 1}).rings == 1


def test_profile_fields_can_be_overridden_individually():
    spec = lidar_spec({"lidar": {"profile": "generic_32", "h_samples": 2048}})
    assert spec.h_samples == 2048
    assert spec.rings == LIDAR_PROFILES["generic_32"].rings


def test_dropping_a_3d_profile_to_one_ring_makes_it_horizontal():
    """Otherwise the carried-over vfov would make the spec self-contradictory."""
    spec = lidar_spec({"lidar": {"profile": "generic_32", "rings": 1}})
    assert (spec.rings, spec.vfov) == (1, 0.0)


def test_unknown_profile_and_keys_are_refused():
    with pytest.raises(ValueError, match="unknown lidar profile"):
        lidar_spec({"lidar": {"profile": "nope"}})
    with pytest.raises(ValueError, match="unknown fleet.lidar keys"):
        lidar_spec({"lidar": {"hsamples": 1800}})


@pytest.mark.parametrize("name", sorted(LIDAR_PROFILES))
def test_every_profile_renders(name):
    sdf = render("robot_0", "0.2 0.7 0.9", LIDAR_PROFILES[name])
    assert "{{" not in sdf, f"{name}: unsubstituted placeholder"


def test_imu_is_fast_enough_for_inertial_odometry():
    """LIO packages need >= ~100 Hz; below that they refuse or drift."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec())
    imu = sdf.split('<sensor name="imu"')[1].split("</sensor>")[0]
    rate = int(imu.split("<update_rate>")[1].split("</update_rate>")[0])
    assert rate >= 100


def test_imu_is_noisy():
    """A noiseless Gazebo IMU reports the simulator's exact angular rate, so an
    EKF fused with it would be laundering ground truth and its accuracy would not
    transfer to hardware. Both channels must carry noise and a bias."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec())
    imu = sdf.split('<sensor name="imu"')[1].split("</sensor>")[0]
    for channel in ("angular_velocity", "linear_acceleration"):
        block = imu.split(f"<{channel}>")[1].split(f"</{channel}>")[0]
        assert block.count('<noise type="gaussian">') == 3, f"{channel}: needs x/y/z noise"
        assert "<bias_mean>" in block, f"{channel}: needs a bias, not just white noise"
        stddevs = [
            float(part.split("</stddev>")[0])
            for part in block.split("<stddev>")[1:]
        ]
        assert all(s > 0.0 for s in stddevs), f"{channel}: zero stddev is no noise"
