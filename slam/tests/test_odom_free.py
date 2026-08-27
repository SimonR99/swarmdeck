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
