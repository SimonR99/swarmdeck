"""Unit tests for canonical odometry contract in swarmdeck_protocol."""

from __future__ import annotations

import pytest
from swarmdeck_protocol import (
    DEFAULT_HARDWARE_ODOMETRY,
    DEFAULT_SIM_ODOMETRY,
    ODOMETRY_PROFILES,
    OdometrySpec,
    get_odometry_spec,
    resolve_odometry_types,
)


def test_odometry_profiles_registry_contains_canonical_systems():
    expected = {"fast_livo2", "superodometry", "ekf", "native", "drift", "icp"}
    assert expected.issubset(set(ODOMETRY_PROFILES))
    for name, spec in ODOMETRY_PROFILES.items():
        assert isinstance(spec, OdometrySpec)
        assert spec.name == name
        assert spec.source_type in {
            "livo",
            "lio",
            "fused_wheels_imu",
            "vendor",
            "synthetic",
            "icp",
        }
        assert isinstance(spec.topic, str) and spec.topic
        assert isinstance(spec.publishes_tf, bool)


def test_fast_livo2_spec_properties():
    spec = get_odometry_spec("fast_livo2")
    assert spec.source_type == "livo"
    assert spec.topic == "odometry"
    assert spec.publishes_tf is True
    assert "lidar" in spec.requires_sensors
    assert "imu" in spec.requires_sensors
    assert "camera" in spec.requires_sensors
    assert spec.medium == "uf"
    assert spec.implementation == "external"


def test_superodometry_spec_properties():
    spec = get_odometry_spec("superodometry")
    assert spec.source_type == "lio"
    assert spec.topic == "laser_odometry"
    assert spec.publishes_tf is False
    assert "lidar" in spec.requires_sensors
    assert "imu" in spec.requires_sensors
    assert spec.medium == ""


def test_get_odometry_spec_unknown_raises():
    with pytest.raises(ValueError, match="unknown odometry profile 'unknown_system'"):
        get_odometry_spec("unknown_system")


def test_resolve_odometry_types_default():
    types = resolve_odometry_types({}, 4, prefix="robot_")
    assert types == [DEFAULT_SIM_ODOMETRY] * 4


def test_resolve_odometry_types_fleet_level():
    cfg = {"odometry": "drift"}
    assert resolve_odometry_types(cfg, 3) == ["drift", "drift", "drift"]


def test_resolve_odometry_types_per_robot_override():
    cfg = {
        "odometry": "fast_livo2",
        "odometry_types": {
            "robot_0": "fast_livo2",
            "robot_1": "drift",
            "robot_2": "superodometry",
        },
    }
    types = resolve_odometry_types(cfg, 3)
    assert types == ["fast_livo2", "drift", "superodometry"]


def test_resolve_odometry_types_cli_override():
    cfg = {
        "odometry": "fast_livo2",
        "odometry_types": {
            "robot_1": "drift",
        },
    }
    types = resolve_odometry_types(cfg, 3, default_override="ekf")
    # robot_1 override preserved, others default to 'ekf'
    assert types == ["ekf", "drift", "ekf"]


def test_resolve_odometry_types_unknown_robot_raises():
    cfg = {
        "odometry_types": {
            "robot_9": "drift",
        }
    }
    with pytest.raises(ValueError, match="not in this fleet"):
        resolve_odometry_types(cfg, 4)


def test_resolve_odometry_types_invalid_profile_raises():
    cfg = {
        "odometry_types": {
            "robot_0": "non_existent",
        }
    }
    with pytest.raises(ValueError, match="invalid odometry profile 'non_existent'"):
        resolve_odometry_types(cfg, 2)
