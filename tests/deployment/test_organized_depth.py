import importlib.util
import math
from pathlib import Path
import warnings

import numpy as np
import pytest


def _module():
    path = Path("deploy/mgg/organized_depth.py")
    spec = importlib.util.spec_from_file_location("organized_depth", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _level_floor_depth(height, width, camera_height, fy, cy):
    """ARGoS axial depth for a level floor; unseen pixels keep its sentinel."""
    rows = np.arange(height, dtype=np.float64)[:, None]
    depth = np.full((height, width), 40.0, dtype=np.float32)
    visible = rows[:, 0] > cy
    depth[visible] = camera_height * fy / (rows[visible] - cy)
    return depth


def test_flat_adjacent_returns_fill_only_the_observed_quad():
    raster = _module().rasterize_flat_depth_quads
    # With fy=10 and cy=-1 these two ranges both lie on z=-1m.
    depth = np.array([[10.0, 10.0], [5.0, 5.0]], dtype=np.float32)
    points = raster(
        depth, fx=10.0, fy=10.0, cx=0.5, cy=-1.0,
        spacing_m=0.25, max_edge_m=6.0,
    )
    assert len(points) > 4
    assert np.allclose(points[:, 2], -1.0)
    assert points[:, 0].min() >= 5.0 and points[:, 0].max() <= 10.0
    assert points[:, 1].min() >= -0.5 and points[:, 1].max() <= 0.5


def test_realistic_floor_supplies_continuous_twelve_metre_support():
    raster = _module().rasterize_flat_depth_quads
    height, width = 240, 320
    fy = height / (2.0 * math.tan(math.radians(30.0)))
    fx, cx, cy = fy, width / 2.0, height / 2.0
    camera_height = 0.30
    depth = _level_floor_depth(height, width, camera_height, fy, cy)

    points = raster(depth, fx=fx, fy=fy, cx=cx, cy=cy)

    # Column cx is an even retained ray. Its adjacent full-pixel floor quad
    # crosses 12 m, and reconstruction must leave no greater-than-voxel gap.
    centreline = points[
        np.isclose(points[:, 1], 0.0, atol=1e-6)
        & np.isclose(points[:, 2], -camera_height, atol=1e-6)
    ]
    around_goal = np.sort(centreline[(centreline[:, 0] >= 10.0)
                                     & (centreline[:, 0] <= 13.0), 0])
    assert around_goal[0] <= 10.5 and around_goal[-1] >= 12.0
    assert np.diff(around_goal).max() <= 0.15 + 1e-6
    assert np.linalg.norm(
        points - np.array([12.0, 0.0, -camera_height]), axis=1
    ).min() <= 0.15


def test_realistic_full_frame_stays_finite_and_within_default_budgets():
    raster = _module().rasterize_flat_depth_quads
    height, width = 240, 320
    focal = height / (2.0 * math.tan(math.radians(30.0)))
    depth = _level_floor_depth(height, width, 0.60, focal, height / 2.0)

    points = raster(
        depth, fx=focal, fy=focal, cx=width / 2.0, cy=height / 2.0
    )

    assert points.dtype == np.float32
    assert 1_000 < len(points) < 1_000_000
    assert np.isfinite(points).all()
    assert np.allclose(points[:, 2], -0.60, atol=1e-5)


@pytest.mark.parametrize("tilt_degrees,accepted", [(4.0, True), (6.0, False)])
def test_surface_normal_guard_at_five_degrees(tilt_degrees, accepted):
    raster = _module().rasterize_flat_depth_quads
    fy = fx = 10.0
    cy = -1.0
    slope = math.tan(math.radians(tilt_degrees))
    rows = np.arange(2, dtype=np.float64)[:, None]
    # Plane z = -1 + slope*x intersected by each axial camera ray.
    depth = np.broadcast_to(1.0 / ((rows - cy) / fy + slope), (2, 2)).copy()
    points = raster(
        depth, fx=fx, fy=fy, cx=0.5, cy=cy,
        max_vertical_span_m=10.0, max_edge_m=10.0, spacing_m=0.25,
    )
    assert bool(len(points)) is accepted


def test_measured_six_centimetre_ground_change_is_continuous():
    raster = _module().rasterize_flat_depth_quads
    # Model the live R2 transition: adjacent image rows observe ground from
    # 5.60 m / -0.13 m to 3.75 m / -0.19 m. The 6 cm height change over
    # 1.85 m is a 1.9 degree surface, while its long edge remains below 3 m.
    far_x, near_x = 5.60, 3.75
    far_z, near_z = -0.13, -0.19
    inverse_fy = near_z / -near_x + far_z / far_x
    fy = 1.0 / inverse_fy
    cy = far_z * fy / far_x
    depth = np.array([[far_x, far_x], [near_x, near_x]], dtype=np.float32)

    points = raster(depth, fx=fy, fy=fy, cx=0.5, cy=cy)

    assert len(points) > 4
    assert points[:, 2].min() >= near_z - 1e-6
    assert points[:, 2].max() <= far_z + 1e-6


@pytest.mark.parametrize(
    "depth,vertical_span,max_edge",
    [
        (np.array([[10.0, 10.0], [5.5, 5.5]]), 0.08, 6.0),  # 10 cm step
        (np.array([[5.0, 10.0], [5.0, 10.0]]), 1.0, 2.0),  # occlusion edge
        (np.array([[5.0, 5.0], [5.0, 5.0]]), 1.0, 2.0),  # vertical wall
        (np.array([[40.0, 40.0], [40.0, 40.0]]), 1.0, 50.0),
        (np.array([[5.0, np.nan], [5.0, 5.0]]), 1.0, 6.0),
    ],
)
def test_step_discontinuity_far_plane_and_invalid_gaps_stay_unknown(
    depth, vertical_span, max_edge
):
    points = _module().rasterize_flat_depth_quads(
        depth, fx=10.0, fy=10.0, cx=0.5, cy=-1.0,
        max_vertical_span_m=vertical_span, max_edge_m=max_edge,
        spacing_m=0.25,
    )
    assert points.shape == (0, 3)


def test_nonfinite_pixels_do_not_emit_geometry_warnings():
    raster = _module().rasterize_flat_depth_quads
    depth = np.array(
        [[np.inf, np.inf, 5.0], [np.inf, np.nan, 5.0]], dtype=np.float32
    )
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        points = raster(depth, fx=10.0, fy=10.0, cx=1.0, cy=-1.0)
    assert points.shape == (0, 3)


def test_work_budgets_fail_without_a_partial_raster():
    raster = _module().rasterize_flat_depth_quads
    depth = np.array([[10.0, 10.0], [5.0, 5.0]], dtype=np.float32)
    with pytest.raises(ValueError, match="pixel budget"):
        raster(depth, fx=10, fy=10, cx=0.5, cy=0, max_pixels=3)
    with pytest.raises(ValueError, match="output budget"):
        raster(
            depth, fx=10, fy=10, cx=0.5, cy=-1, spacing_m=0.25,
            max_edge_m=6, max_output_points=2,
        )
    with pytest.raises(ValueError, match="quad budget"):
        raster(
            np.tile(depth, (1, 2)), fx=10, fy=10, cx=0.5, cy=-1,
            spacing_m=0.25, max_edge_m=6, max_candidate_quads=1,
        )
