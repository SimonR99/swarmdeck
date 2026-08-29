import math

import numpy as np

from swarmdeck_slam.odom_free import OdomFreeConfig, prepare_cloud, register_clouds
from swarmdeck_slam.types import se3_distance, se3_identity


def _scene(seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    walls = []
    for x, y, dx, dy, length in (
        (-5.0, -3.0, 1.0, 0.0, 10.0),
        (-5.0, 4.0, 1.0, 0.0, 6.0),
        (-5.0, -3.0, 0.0, 1.0, 7.0),
        (3.0, -3.0, 0.0, 1.0, 4.0),
    ):
        along = rng.uniform(0.0, length, 900)
        z = rng.uniform(-0.3, 1.1, 900)
        walls.append(
            np.column_stack([x + along * dx, y + along * dy, z])
            + rng.normal(0.0, 0.008, (900, 3))
        )
    return np.vstack(walls)


def _transform(yaw: float, x: float, y: float) -> np.ndarray:
    transform = se3_identity()
    c, s = math.cos(yaw), math.sin(yaw)
    transform[:2, :2] = [[c, -s], [s, c]]
    transform[:2, 3] = [x, y]
    return transform


def test_registration_recovers_translation_outside_gicp_basin() -> None:
    target_points = _scene()
    expected = _transform(math.radians(34.0), 3.4, -2.2)
    source_points = (target_points - expected[:3, 3]) @ expected[:3, :3]
    config = OdomFreeConfig(
        min_radius=0.0,
        max_radius=15.0,
        grid_half_extent=16.0,
        min_symmetric_overlap=0.65,
    )

    hypotheses = register_clouds(
        prepare_cloud(target_points, config),
        prepare_cloud(source_points, config),
        config,
    )

    assert hypotheses
    translation_error, rotation_error = se3_distance(
        expected, hypotheses[0].t_target_source
    )
    assert translation_error < 0.12
    assert rotation_error < math.radians(1.0)


def test_empty_filtered_cloud_has_no_registration() -> None:
    points = np.array([[1.0, 0.0, 5.0]], dtype=np.float64)
    config = OdomFreeConfig()
    prepared = prepare_cloud(points, config)
    assert prepared.points.shape == (0, 3)
    assert register_clouds(prepared, prepared, config) == []


def _planar_corridor() -> np.ndarray:
    along = np.linspace(-8.0, 8.0, 500)
    north = np.column_stack([along, np.full(along.shape, 1.8), np.full(along.shape, 0.52)])
    south = np.column_stack([along, np.full(along.shape, -1.8), np.full(along.shape, 0.52)])
    end = np.column_stack(
        [np.full(80, 8.0), np.linspace(-1.8, 1.8, 80), np.full(80, 0.52)]
    )
    return np.vstack([north, south, end])


def test_three_d_cloud_is_not_extruded() -> None:
    prepared = prepare_cloud(_scene(), OdomFreeConfig(min_radius=0.0, max_radius=20.0))
    assert prepared.coplanar is False
    assert float(np.ptp(prepared.points[:, 2])) > 0.5


def test_coplanar_ring_is_extruded_and_registers() -> None:
    target_points = _planar_corridor()
    expected = _transform(math.radians(12.0), 1.1, -0.4)
    source_points = (target_points - expected[:3, 3]) @ expected[:3, :3]
    config = OdomFreeConfig(
        min_radius=0.0,
        max_radius=20.0,
        grid_half_extent=16.0,
        min_z=0.0,
        max_z=1.0,
        min_symmetric_overlap=0.50,
    )
    target = prepare_cloud(target_points, config)
    source = prepare_cloud(source_points, config)
    assert target.coplanar and source.coplanar
    assert target.n_observed == len(target.points) // 3
    assert float(np.ptp(target.points[:, 2])) > config.voxel_size
    hypotheses = register_clouds(target, source, config)
    assert hypotheses
    translation_error, rotation_error = se3_distance(
        expected, hypotheses[0].t_target_source
    )
    assert translation_error < 0.15
    assert rotation_error < math.radians(2.0)
    assert abs(hypotheses[0].t_target_source[2, 3]) < 1e-9
