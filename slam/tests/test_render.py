"""Tests for :mod:`swarmdeck_slam.render`.

Every test renders the shared synthetic fleet (`tests/synthetic.py`) against
known ground truth -- there are no mocks here, because the thing this module
has to prove is a *quantitative* claim (drifted trajectories produce blurrier
maps than optimized ones) that a mock cannot stand in for.

Two pose sets are built from the same fixture and compared throughout:

- ``_truth_poses`` -- exact ``T_world_base`` (what an optimizer converges to).
- ``_drifted_poses`` -- ``t_world_map_true @ keyframe.t_odom_base``: the
  robot's own start frame (assumed correctly known, so a component-merge
  error is not conflated with trajectory drift) composed with its raw,
  never-corrected odometry. This stands in for "before the pose graph has
  run" without needing `graph.py`'s optimizer, which is a different agent's
  module and not part of this one's contract.
"""

from __future__ import annotations

import time
import tracemalloc
from unittest import mock

import numpy as np
import pytest
from scipy.spatial import cKDTree

from swarmdeck_slam import render
from swarmdeck_slam.render import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    RenderConfig,
    RenderedGrid,
    _Meta,
    _rasterize_free,
    render_occupancy,
)
from swarmdeck_slam.types import Component, OptimizedGraph
from synthetic import WALL_HEIGHT, make_scene, simulate_robot, two_robot_fleet, yaw_pose

RESOLUTION = 0.1


def _all_keyframes(robots):
    return [kf for robot in robots for kf in robot.keyframes]


def _truth_poses(robots):
    return {kf.id: robot.truth[kf.id] for robot in robots for kf in robot.keyframes}


def _drifted_poses(robots):
    return {
        kf.id: robot.t_world_map_true @ kf.t_odom_base for robot in robots for kf in robot.keyframes
    }


def _graph(robots, poses, components):
    return OptimizedGraph(
        poses=poses,
        t_world_map={robot.robot_id: robot.t_world_map_true for robot in robots},
        components=components,
    )


def _merged_component(robots):
    anchor = robots[0].keyframes[0].id
    return [Component(0, frozenset(r.robot_id for r in robots), anchor)]


def _singleton_components(robots):
    return [
        Component(i, frozenset({robot.robot_id}), robot.keyframes[0].id)
        for i, robot in enumerate(robots)
    ]


def _cell_value(grid: RenderedGrid, x: float, y: float) -> int | None:
    gx = int(np.floor((x - grid.origin_x) / grid.resolution))
    gy = int(np.floor((y - grid.origin_y) / grid.resolution))
    if 0 <= gx < grid.width and 0 <= gy < grid.height:
        return int(grid.cells[gy, gx])
    return None


def _occupied_world_points(grid: RenderedGrid) -> np.ndarray:
    gy, gx = np.nonzero(grid.cells == OCCUPIED)
    xs = grid.origin_x + (gx + 0.5) * grid.resolution
    ys = grid.origin_y + (gy + 0.5) * grid.resolution
    return np.stack([xs, ys], axis=1)


@pytest.fixture(scope="module")
def fleet():
    return two_robot_fleet()


@pytest.fixture(scope="module")
def truth_grid(fleet) -> RenderedGrid:
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    cfg = RenderConfig(native_map_resolution=RESOLUTION)
    grids = render_occupancy(graph, _all_keyframes(robots), cfg)
    assert len(grids) == 1
    return next(iter(grids.values()))


def test_ground_truth_reproduces_scene_walls(fleet, truth_grid):
    """Every occupied cell lands within ~1 cell of the true wall surface, and
    two well-travelled walls are fully recovered there -- the correctness
    anchor the task asks for, checked both ways (no phantom walls, no gaps on
    walls the robots actually drove past)."""
    scene, robots = fleet
    occ_points = _occupied_world_points(truth_grid)
    assert len(occ_points) > 0

    # Precision: no occupied cell exists that isn't near real wall geometry.
    scene_tree = cKDTree(scene[:, :2])
    distance_to_truth, _ = scene_tree.query(occ_points)
    assert distance_to_truth.mean() < RESOLUTION
    assert np.percentile(distance_to_truth, 95) < 2 * RESOLUTION

    # Recall: segments both robots' paths hug are fully painted, not sparse.
    occ_tree = cKDTree(occ_points)
    well_covered_segments = [
        ((0.0, 24.0), (0.0, 0.0)),  # west wall, alpha's loop runs right beside it
        ((12.0, 9.0), (19.0, 6.0)),  # interior partition alpha passes close to
    ]
    for (x0, y0), (x1, y1) in well_covered_segments:
        t = np.linspace(0.0, 1.0, 100)
        samples = np.stack([x0 + t * (x1 - x0), y0 + t * (y1 - y0)], axis=1)
        distance_to_occupied, _ = occ_tree.query(samples)
        assert distance_to_occupied.max() < 2 * RESOLUTION


def test_drifted_odometry_blurs_the_wall(fleet, truth_grid):
    """The premise of the whole architecture: fixing the trajectory sharpens
    the map. Render the same fleet with uncorrected odometry poses and show
    the west wall -- a single straight surface -- becomes measurably thicker
    and more dispersed than in the ground-truth render.
    """
    _, robots = fleet
    graph = _graph(robots, _drifted_poses(robots), _merged_component(robots))
    cfg = RenderConfig(native_map_resolution=RESOLUTION)
    drift_grid = next(iter(render_occupancy(graph, _all_keyframes(robots), cfg).values()))

    def wall_x_dispersion(grid: RenderedGrid) -> tuple[float, int]:
        xs = grid.origin_x + (np.arange(grid.width) + 0.5) * grid.resolution
        ys = grid.origin_y + (np.arange(grid.height) + 0.5) * grid.resolution
        x_grid, y_grid = np.meshgrid(xs, ys)
        occupied = grid.cells == OCCUPIED
        # A band around the true wall (x=0), away from corners where two
        # walls' returns would legitimately mix.
        band = occupied & (x_grid >= -1.0) & (x_grid <= 1.0) & (y_grid >= 2.0) & (y_grid <= 22.0)
        return float(x_grid[band].std()), int(band.sum())

    truth_std, truth_count = wall_x_dispersion(truth_grid)
    drift_std, drift_count = wall_x_dispersion(drift_grid)

    assert truth_std <= 0.5 * RESOLUTION + 1e-9  # ground truth: essentially one cell wide
    assert drift_std > 2.5 * truth_std  # drifted: measurably smeared across many
    assert drift_count > 1.5 * truth_count  # more cells lit up painting the same wall


def test_corridor_interior_is_free_never_observed_is_unknown(truth_grid):
    # beta's path passes directly through/near this waypoint.
    assert _cell_value(truth_grid, 6.0, 12.0) == FREE
    # Deep in a room neither robot's loop enters or has line of sight into.
    assert _cell_value(truth_grid, 36.0, 8.0) == UNKNOWN
    assert _cell_value(truth_grid, 37.0, 20.0) == UNKNOWN


def test_height_band_excludes_returns_outside_it(fleet):
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    keyframes = _all_keyframes(robots)

    full_band = RenderConfig(min_z=0.0, max_z=WALL_HEIGHT, native_map_resolution=RESOLUTION)
    narrow_band = RenderConfig(min_z=0.05, max_z=0.15, native_map_resolution=RESOLUTION)
    outside_band = RenderConfig(min_z=10.0, max_z=20.0, native_map_resolution=RESOLUTION)

    full_grid = next(iter(render_occupancy(graph, keyframes, full_band).values()))
    narrow_grid = next(iter(render_occupancy(graph, keyframes, narrow_band).values()))
    outside_grid = next(iter(render_occupancy(graph, keyframes, outside_band).values()))

    full_occupied = int(np.sum(full_grid.cells == OCCUPIED))
    narrow_occupied = int(np.sum(narrow_grid.cells == OCCUPIED))
    assert 0 < narrow_occupied < full_occupied

    # A band with no scene returns in it at all must render nothing observed
    # -- not zero walls with confident free space, which would be worse than
    # silence: unknown is the honest statement of "we filtered this away".
    assert np.sum(outside_grid.cells == OCCUPIED) == 0
    assert np.sum(outside_grid.cells == FREE) == 0


def test_floor_z_offsets_the_band(fleet):
    """`floor_z` shifts min_z/max_z together, matching
    `adapters/runtime.py:map_cloud_height_limits` -- a band expressed relative
    to a floor 1m up should behave exactly like the same band without the
    offset once floor_z is subtracted back out."""
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    keyframes = _all_keyframes(robots)

    baseline = RenderConfig(min_z=0.05, max_z=0.15, native_map_resolution=RESOLUTION)
    offset = RenderConfig(floor_z=1.0, min_z=0.05 - 1.0, max_z=0.15 - 1.0, native_map_resolution=RESOLUTION)

    baseline_grid = next(iter(render_occupancy(graph, keyframes, baseline).values()))
    offset_grid = next(iter(render_occupancy(graph, keyframes, offset).values()))

    np.testing.assert_array_equal(baseline_grid.cells, offset_grid.cells)


def test_separate_components_never_overlay(fleet):
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _singleton_components(robots))
    cfg = RenderConfig(native_map_resolution=RESOLUTION)
    grids = render_occupancy(graph, _all_keyframes(robots), cfg)

    assert len(grids) == 2
    all_robots_seen: set[str] = set()
    for grid in grids.values():
        assert len(grid.robots) == 1  # never two robots sharing one grid here
        all_robots_seen |= grid.robots
    assert all_robots_seen == {"alpha", "beta"}


def test_merged_component_renders_one_consistent_grid(fleet):
    """Two robots WITH a verified transform land in one grid, and that grid
    actually contains geometry from both -- not just the anchor robot."""
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    cfg = RenderConfig(native_map_resolution=RESOLUTION)
    grids = render_occupancy(graph, _all_keyframes(robots), cfg)

    assert len(grids) == 1
    grid = next(iter(grids.values()))
    assert grid.robots == {"alpha", "beta"}

    # x=26 partition sits inside beta's loop and well outside alpha's (alpha
    # never leaves x in [3, 9]); it can only be painted by beta's returns.
    assert _cell_value(grid, 26.0, 12.0) == OCCUPIED


def test_max_cells_cap_coarsens_resolution_not_bounds(fleet):
    """Requesting a resolution far too fine for the cell budget must still
    produce a grid, honour the cap, and cover the full requested extent --
    degrading by coarsening, not by chopping off explored area (see
    `_fit_grid`'s docstring for why bounds are never the thing clamped)."""
    _, robots = fleet
    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    keyframes = _all_keyframes(robots)

    uncapped = RenderConfig(native_map_resolution=RESOLUTION, native_map_max_cells=8_000_000)
    capped = RenderConfig(native_map_resolution=0.01, native_map_max_cells=2_000)

    uncapped_grid = next(iter(render_occupancy(graph, keyframes, uncapped).values()))
    capped_grid = next(iter(render_occupancy(graph, keyframes, capped).values()))

    assert capped_grid.width * capped_grid.height <= 2_000
    assert capped_grid.resolution > 0.01  # coarsened well past what was asked for

    # Extent (in metres) is preserved, not truncated -- only fidelity dropped.
    uncapped_extent_x = uncapped_grid.width * uncapped_grid.resolution
    capped_extent_x = capped_grid.width * capped_grid.resolution
    assert capped_extent_x == pytest.approx(uncapped_extent_x, rel=0.05)


def test_wire_format_contract(truth_grid):
    """Cells must be exactly int8 in {-1, 0, 100}, row-major -- the format
    `adapters/protocol/README.md` and the browser UI already depend on."""
    assert truth_grid.cells.dtype == np.int8
    values = np.unique(truth_grid.cells)
    assert set(values.tolist()) <= {UNKNOWN, FREE, OCCUPIED}


def test_render_scales_to_a_realistic_keyframe_count():
    """Measures, rather than assumes, the scaling limit: ~300 keyframes
    (150 per robot) and the returns that go with them at 5cm resolution --
    the resolution `scout_mini.yaml` actually runs -- must render well under
    a Python-loop-over-rays budget and respect the default cell cap.
    """
    scene = make_scene(0)
    alpha = simulate_robot(
        scene, "alpha", [(3.0, 3.0), (9.0, 3.0), (9.0, 20.0), (3.0, 20.0), (3.0, 3.0)],
        seed=1, n_keyframes=150,
    )
    beta = simulate_robot(
        scene, "beta", [(6.0, 12.0), (20.0, 12.0), (22.0, 20.0), (8.0, 18.0), (6.0, 12.0)],
        seed=2, n_keyframes=150, start_in_world=yaw_pose(0.0, 0.0, 0.0),
    )
    robots = [alpha, beta]
    keyframes = _all_keyframes(robots)
    total_points = sum(kf.points.shape[0] for kf in keyframes)
    assert total_points > 100_000  # sanity: this is actually a large scene

    graph = _graph(robots, _truth_poses(robots), _merged_component(robots))
    cfg = RenderConfig(native_map_resolution=0.05)  # scout_mini.yaml's real value

    start = time.perf_counter()
    grids = render_occupancy(graph, keyframes, cfg)
    elapsed = time.perf_counter() - start

    grid = next(iter(grids.values()))
    cell_count = grid.width * grid.height
    assert cell_count <= cfg.native_map_max_cells
    # Measured locally at ~1.2s for 300 keyframes / ~200k points; bounded
    # generously for slower CI hardware while still catching an accidental
    # reintroduction of a per-ray or per-cell Python loop, which is orders of
    # magnitude slower than this at the same scale.
    assert elapsed < 15.0


def test_dense_long_range_keyframe_does_not_blow_up_memory():
    """A single keyframe must not size its ray matrix by its longest return.

    ``_rasterize_free`` walks every ray of a keyframe as one
    ``(steps, rays)`` matrix. Unbatched, ``steps`` is set by the single
    longest ray, so one 60 m return makes every short ray in the keyframe pay
    1200 rows of padding: 40k points at 5 cm measured 1.58 GB of peak
    allocation and 1.71 s for ONE keyframe, against a render that walks every
    keyframe of every robot from scratch on every optimize.

    Length-sorted batching bounds that regardless of density or range. This
    asserts the property (bounded peak), not the measured number -- the point
    is that peak stops tracking cloud size, so the bound is set well above
    what batching costs and far below what the unbatched walk did.
    """
    n_points = 40_000
    rng = np.random.default_rng(0)
    theta = rng.uniform(0.0, 2.0 * np.pi, n_points)
    radius = rng.uniform(1.0, 60.0, n_points)
    ends = np.stack([radius * np.cos(theta), radius * np.sin(theta)], axis=1)
    meta = _Meta(0.05, 4000, 4000, -100.0, -100.0)
    free = np.zeros((meta.height, meta.width), dtype=np.int32)

    tracemalloc.start()
    try:
        _rasterize_free(np.array([0.0, 0.0]), ends, meta, free)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()

    assert free.sum() > 0, "the walk must actually mark free cells"
    assert peak < 400e6, f"peak allocation {peak / 1e6:.0f} MB for one keyframe"


def test_batched_rays_mark_the_same_cells_as_a_single_batch():
    """Batching is an allocation strategy, not a change of result.

    Verified by shrinking the batch budget to a handful of elements, which
    forces many batches over the same rays, and requiring an identical grid.
    """
    rng = np.random.default_rng(7)
    ends = rng.uniform(-8.0, 8.0, size=(400, 2))
    meta = _Meta(0.1, 200, 200, -10.0, -10.0)
    origin = np.array([0.5, -0.5])

    one_batch = np.zeros((meta.height, meta.width), dtype=np.int32)
    many_batches = np.zeros((meta.height, meta.width), dtype=np.int32)
    _rasterize_free(origin, ends, meta, one_batch)
    with mock.patch.object(render, "_RAY_BATCH_ELEMENTS", 64):
        _rasterize_free(origin, ends, meta, many_batches)

    assert np.array_equal(one_batch, many_batches)
