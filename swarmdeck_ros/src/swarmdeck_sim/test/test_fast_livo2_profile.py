"""Tests for Fast-LIVO2 profile and parameter generation.

Verifies:
  1. Sensor extrinsics (LiDAR and camera mounts) match RobotSpec and ARGoS definitions.
  2. Camera intrinsics derive correctly from vertical FOV and image aspect ratios.
  3. IMU noise model converts from ARGoS discrete per-sample variance to continuous-time densities.
  4. Heterogeneous fleet configurations (Bunker, Scout Mini, Spot) produce distinct, correct profiles.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
FAST_LIVO_DIR = REPO / "deploy" / "docker" / "fast_livo2"
sys.path.insert(0, str(FAST_LIVO_DIR))

import make_profile as mp  # noqa: E402


def test_camera_pinhole_intrinsics_math():
    """Camera vertical FOV dictates fy, square pixels dictate fx=fy."""
    res_str = "320,240"
    fov_deg = 60.0
    intrinsics = mp.compute_camera_intrinsics(res_str, fov_deg)

    assert intrinsics["width"] == 320
    assert intrinsics["height"] == 240
    expected_fy = 240.0 / (2.0 * math.tan(math.radians(30.0)))
    assert intrinsics["fy"] == pytest.approx(expected_fy, abs=1e-4)
    assert intrinsics["fx"] == pytest.approx(expected_fy, abs=1e-4)
    assert intrinsics["cx"] == pytest.approx(160.0, abs=1e-4)
    assert intrinsics["cy"] == pytest.approx(120.0, abs=1e-4)


def test_imu_noise_conversion_from_argos():
    """Continuous-time densities scale with sqrt(rate) and 1/sqrt(rate)."""
    argos_noise = [0.002, 0.02, 0.0002, 0.002]  # [gyro, acc, gyro_walk, acc_walk]
    rate = 100.0
    scale = 3.0
    imu_noise = mp.compute_imu_noise(argos_noise, rate, scale)

    root_rate = math.sqrt(rate)  # 10.0
    expected_gyr_n = (0.002 / 10.0) * 3.0
    expected_acc_n = (0.02 / 10.0) * 3.0
    expected_gyr_w = (0.0002 * 10.0) * 3.0
    expected_acc_w = (0.002 * 10.0) * 3.0

    assert imu_noise["gyr_n"] == pytest.approx(expected_gyr_n, rel=1e-4)
    assert imu_noise["acc_n"] == pytest.approx(expected_acc_n, rel=1e-4)
    assert imu_noise["gyr_w"] == pytest.approx(expected_gyr_w, rel=1e-4)
    assert imu_noise["acc_w"] == pytest.approx(expected_acc_w, rel=1e-4)
    assert imu_noise["cov_gyr"] == pytest.approx(expected_gyr_n ** 2, rel=1e-4)
    assert imu_noise["cov_acc"] == pytest.approx(expected_acc_n ** 2, rel=1e-4)


def test_heterogeneous_fleet_profiles():
    """Bunker, Scout Mini, and Spot have different mount heights."""
    platforms = {
        "scout_mini": {"lidar_z": 0.4525, "cam_z": 0.3525},
        "bunker": {"lidar_z": 0.7200, "cam_z": 0.6200},
        "spot": {"lidar_z": 0.9700, "cam_z": 0.8700},
    }
    intrinsics = mp.compute_camera_intrinsics("320,240", 60.0)
    imu_noise = mp.compute_imu_noise([0.002, 0.02, 0.0002, 0.002], 100.0)

    configs = {}
    for name, mounts in platforms.items():
        cfg = mp.build_fast_livo_config(
            lidar_in_body=[0.0, 0.0, mounts["lidar_z"]],
            camera_in_body=[0.15, 0.0, mounts["cam_z"]],
            cam_intrinsics=intrinsics,
            imu_noise=imu_noise,
            lidar_elev=[-15.0, 15.0],
            scan_line=17,
        )
        configs[name] = cfg

        # Verify LiDAR translation
        assert cfg["mapping"]["extrinsic_T"] == [0.0, 0.0, mounts["lidar_z"]]
        # Verify Camera translation
        assert cfg["mapping"]["camera_ext_t"] == [0.15, 0.0, mounts["cam_z"]]
        # Verify Camera rotation (optical to body)
        assert cfg["mapping"]["camera_ext_R"] == [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        # Verify scan line count
        assert cfg["preprocess"]["scan_line"] == 17

    # Ensure profiles are strictly distinct between platforms
    assert configs["scout_mini"]["mapping"]["extrinsic_T"] != configs["spot"]["mapping"]["extrinsic_T"]
    assert configs["bunker"]["mapping"]["extrinsic_T"] != configs["spot"]["mapping"]["extrinsic_T"]


def test_yaml_roundtrip_validity(tmp_path):
    """Generated configuration parses cleanly with PyYAML."""
    intrinsics = mp.compute_camera_intrinsics("640,480", 70.0)
    imu_noise = mp.compute_imu_noise(None, 100.0)
    cfg = mp.build_fast_livo_config(
        lidar_in_body=[0.0, 0.0, 0.72],
        camera_in_body=[0.15, 0.0, 0.62],
        cam_intrinsics=intrinsics,
        imu_noise=imu_noise,
        lidar_elev=[-15.0, 15.0],
    )
    out_file = tmp_path / "test_fast_livo.yaml"
    out_file.write_text(yaml.safe_dump(cfg))

    loaded = yaml.safe_load(out_file.read_text())
    assert loaded["common"]["lid_topic"] == "points"
    assert loaded["preprocess"]["blind"] == 0.2
    assert loaded["odometry"]["voxel_size"] == 0.5
    assert len(loaded["mapping"]["extrinsic_R"]) == 9
