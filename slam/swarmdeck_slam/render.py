"""Render occupancy grids from an :class:`OptimizedGraph` and its keyframes.

The old backend (``server/swarmdeck_server/mapsvc/registration.py``) treated an
occupancy grid as data: each robot built one independently, and a separate FFT
correlation stage then guessed the rigid transform that would make two grids
line up. That guess was fragile (see ``docs/architecture/collaborative-slam.md``)
and, worse, it was blind -- nothing about grid correlation can tell you whether
the transform it found is *right*, only that it scores well.

This module treats a grid as a *rendering*: it is what you get when you pose
every keyframe's cloud at its optimized ``T_world_base`` and rasterize the
result. Fix the trajectory (that is the pose graph's job, not this module's)
and the map merges itself -- there is no registration step here, and there
cannot be a wrong one, because two robots are only ever poured into the same
grid when :class:`~swarmdeck_slam.types.OptimizedGraph` has already proven
they share a frame.

Three partitions of one render
-------------------------------
The same posed points are grouped three ways: per component (the merged map),
per robot (that machine's whole coverage), and per trajectory (one unbroken
stretch of driving, for a robot that has restarted). :func:`render_all`
computes the poses once and groups them three times; the per-trajectory
partition is empty unless some robot actually restarted.

Per-component isolation
------------------------
:attr:`OptimizedGraph.components` partitions robots by verified relative
transform. Two robots in different components have no known relative pose,
so overlaying their clouds would draw a confident, uninspectable lie -- the
one thing this whole architecture exists to avoid (see
:class:`~swarmdeck_slam.types.Component`'s docstring). This module never does
that: it renders one grid per component and nothing merges across the
boundary. A robot that graph construction never placed in any component (no
verified inter-robot closure yet) still gets its own single-robot grid rather
than being silently dropped -- see :func:`_partition_robots`.

Incremental rendering (future work)
------------------------------------
Rendering is deliberately decomposed per keyframe: :func:`_render_component`
computes each keyframe's filtered, world-frame points and sensor origin
independently (the ``contributions`` list) before any grid buffer exists, and
only the final accumulation step (``_rasterize_free`` / cell indexing) touches
shared state. A future incremental renderer can exploit that structure directly:
cache each keyframe's contribution keyed by ``(KeyframeId, pose)``, and on the
next render only recompute keyframes whose optimized pose actually moved,
subtracting the stale contribution from the integer evidence accumulators
before adding the new one. This module always
renders from scratch -- proving the trajectory-fixes-the-map premise requires a
real from-scratch render, not a cached approximation of one -- but nothing here
would need to change shape to grow that cache later.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final, Iterable, NamedTuple

import numpy as np

from swarmdeck_slam.types import (
    Keyframe,
    KeyframeId,
    OptimizedGraph,
    TrajectoryId,
    transform_points,
)

# Occupancy wire format fixed by adapters/protocol/README.md: int8, row-major,
# -1 unknown / 0 free / 100 occupied. The browser UI already renders exactly
# this; changing it is a protocol change, not a rendering-module decision.
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

#: Cap on ``steps x rays`` per batch in :func:`_rasterize_free`. At float64 this
#: is ~32 MB per intermediate buffer and a handful of buffers live at once, so
#: peak stays in the low hundreds of MB regardless of cloud density or range --
#: the property that matters, since the previous unbatched walk scaled with the
#: single longest ray in a keyframe and reached 1.58 GB on a realistic one.
_RAY_BATCH_ELEMENTS: Final[int] = 4_000_000


@dataclass(slots=True, frozen=True)
class RenderConfig:
    """Rendering parameters, named to match ``map_cloud_height_band`` and the
    ``native_map_*`` / ``retain_free_space`` keys in
    ``adapters/adapter_ros1/config/scout_mini.yaml``.

    Those are real, calibrated hardware values (see that file's comments on
    the Ouster/LVI-SAM floor measurement); reusing the vocabulary means a
    robot's existing config can be handed to this module's tests and,
    eventually, its call site, without translation.
    """

    floor_z: float = 0.0
    min_z: float = -math.inf
    max_z: float = math.inf
    native_map_resolution: float = 0.05
    native_map_padding_m: float = 1.0
    native_map_max_cells: int = 8_000_000
    retain_free_space: bool = True
    # Not part of the yaml vocabulary: a defensive cap on ray length so one
    # bad return (a reflection interpreted as a 10 km range) cannot blow up
    # the vectorized DDA matrix, which is sized by the *longest* ray in a
    # keyframe. Mirrors the old accumulator's MAX_RAY_RANGE_M.
    max_range_m: float = 60.0

    def height_limits(self) -> tuple[float, float]:
        """``(min_z, max_z)`` in the frame the band was measured in.

        Matches ``adapters/runtime.py:map_cloud_height_limits`` -- floor_z, if
        given, is a physical offset added to both bounds, which is what lets a
        profile express "15 cm above the floor" independently of wherever the
        robot's frame origin happens to sit.
        """
        return self.floor_z + self.min_z, self.floor_z + self.max_z


@dataclass(slots=True, frozen=True)
class RenderedGrid:
    """One component's rendered occupancy grid. Wire-format-ready as-is."""

    component_id: int
    robots: frozenset[str]
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    cells: np.ndarray  # int8 [height, width], row-major, -1/0/100

    def __post_init__(self) -> None:
        if self.cells.dtype != np.int8:
            raise ValueError(f"cells must be int8, got {self.cells.dtype}")
        if self.cells.shape != (self.height, self.width):
            raise ValueError(
                f"cells shape {self.cells.shape} does not match "
                f"(height={self.height}, width={self.width})"
            )


class _Meta(NamedTuple):
    """Grid geometry, fixed before any cell is written -- see :func:`_fit_grid`."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float


def render_per_robot(
    graph: OptimizedGraph,
    keyframes: Iterable[Keyframe],
    config: RenderConfig | None = None,
) -> dict[str, RenderedGrid]:
    """One grid per robot, each rendered from that robot's keyframes alone.

    Same optimized poses as :func:`render_occupancy`, partitioned by robot
    instead of by component. This is what an operator needs that the merged map
    cannot show: a robot in a component of one has no merged map by design
    (publishing it would look like a merge that has not happened), and a robot
    inside a larger component contributes to a joint grid where its own
    coverage is no longer separable.

    Distinct from the local map an adapter uploads, which is that robot's own
    SLAM package's output in its own frame -- this one is posed by the
    collaborative solver, so where a merge exists these grids are directly
    comparable to each other and to the merged map.
    """
    config = config or RenderConfig()
    return _robot_grids(_contributions_of(graph, keyframes, config), config)


def render_occupancy(
    graph: OptimizedGraph,
    keyframes: Iterable[Keyframe],
    config: RenderConfig | None = None,
) -> dict[int, RenderedGrid]:
    """Render one grid per component of ``graph``.

    Every keyframe is posed at its optimized ``T_world_base`` -- ``graph.poses``
    -- and rasterized there; keyframes with no optimized pose (not yet solved)
    are skipped rather than guessed at. See the module docstring for why this
    replaces grid-registration entirely rather than complementing it.
    """
    config = config or RenderConfig()
    contributions = _contributions_of(graph, keyframes, config)
    return _component_grids(graph, contributions, config)


def render_per_trajectory(
    graph: OptimizedGraph,
    keyframes: Iterable[Keyframe],
    config: RenderConfig | None = None,
) -> dict[TrajectoryId, RenderedGrid]:
    """One grid per trajectory, for a robot that has more than one.

    A robot that reboots contributes several independent stretches of driving,
    and :func:`render_per_robot` pours all of them into one grid -- correct,
    because they are one machine's coverage, and useless for the question an
    operator actually has after a reboot, which is whether the two segments
    agree about the building. Rendering a segment alone answers it: two
    trajectory grids that look like the same corridor at the same coordinates
    are a merge that worked.

    Only robots with more than one trajectory get grids here. For a robot with
    exactly one, this grid and its ``robot:`` grid are pixel-identical renders
    of the same keyframes, and the ray walk that dominates a render is not
    worth paying twice to publish the same picture under two names.
    """
    config = config or RenderConfig()
    return _trajectory_grids(_contributions_of(graph, keyframes, config), config)


def render_all(
    graph: OptimizedGraph,
    keyframes: Iterable[Keyframe],
    config: RenderConfig | None = None,
) -> tuple[dict[int, RenderedGrid], dict[str, RenderedGrid], dict[TrajectoryId, RenderedGrid]]:
    """Every partition of the same render: ``(per component, per robot, per trajectory)``.

    The back-end publishes both on every cycle, and they are the same posed,
    height-filtered, range-capped points grouped two different ways. Calling
    :func:`render_occupancy` and :func:`render_per_robot` separately poses and
    filters every keyframe twice; this poses them once and groups them twice.

    Worth measuring before assuming it is a large win -- it is not. On a
    240-keyframe two-robot fleet the two separate calls take 4.04 s against
    3.79 s here, about 6%. The duplicated pass was the cheap one: what
    dominates a render is the ray walk in :func:`_rasterize_free`, and that is
    genuinely per-grid work, since a robot inside a merged component has to be
    rasterized into the component grid *and* into its own. No grouping can
    share that. The reason to prefer this entry point is that it states the
    relationship between the two partitions in one place, and it is the
    decomposition the module docstring already describes for a future
    incremental renderer -- not a speedup.

    :func:`render_occupancy`, :func:`render_per_robot` and
    :func:`render_per_trajectory` remain as single-partition wrappers for
    callers (tests, tools) that want only one, and none pays for a partition it
    did not ask for.

    The third partition is usually empty and usually free: a trajectory grid is
    only produced for a robot that has restarted, because for every other robot
    it would be an identical copy of that robot's own grid.
    """
    config = config or RenderConfig()
    contributions = _contributions_of(graph, keyframes, config)
    return (
        _component_grids(graph, contributions, config),
        _robot_grids(contributions, config),
        _trajectory_grids(contributions, config),
    )


def _contributions_of(
    graph: OptimizedGraph, keyframes: Iterable[Keyframe], config: RenderConfig
) -> dict[KeyframeId, _Contribution]:
    return _keyframe_contributions(graph, {kf.id: kf for kf in keyframes}, config)


def _component_grids(
    graph: OptimizedGraph,
    contributions: dict[KeyframeId, _Contribution],
    config: RenderConfig,
) -> dict[int, RenderedGrid]:
    robot_ids = {kf_id.robot_id for kf_id in contributions}
    return {
        component_id: _render_component(component_id, robots, contributions, config)
        for component_id, robots in _partition_robots(graph, robot_ids)
    }


def _robot_grids(
    contributions: dict[KeyframeId, _Contribution], config: RenderConfig
) -> dict[str, RenderedGrid]:
    robot_ids = sorted({kf_id.robot_id for kf_id in contributions})
    return {
        robot_id: _render_component(index, frozenset({robot_id}), contributions, config)
        for index, robot_id in enumerate(robot_ids)
    }


def _trajectory_grids(
    contributions: dict[KeyframeId, _Contribution], config: RenderConfig
) -> dict[TrajectoryId, RenderedGrid]:
    """One grid per trajectory, skipping robots that only have one.

    See :func:`render_per_trajectory` for why the single-trajectory case is
    skipped rather than rendered and thrown away.
    """
    trajectories = sorted({kf_id.trajectory for kf_id in contributions})
    per_robot: dict[str, int] = {}
    for trajectory in trajectories:
        per_robot[trajectory.robot_id] = per_robot.get(trajectory.robot_id, 0) + 1
    return {
        trajectory: _render_component(
            index,
            frozenset({trajectory.robot_id}),
            {k: v for k, v in contributions.items() if k.trajectory == trajectory},
            config,
        )
        for index, trajectory in enumerate(trajectories)
        if per_robot[trajectory.robot_id] > 1
    }


def _partition_robots(
    graph: OptimizedGraph, robot_ids: set[str]
) -> list[tuple[int, frozenset[str]]]:
    """Every robot with keyframes gets exactly one grid, and grids never merge
    across a component boundary.

    ``graph.components`` is expected to partition robots that share a verified
    transform, but says nothing about a robot with keyframes that no
    inter-robot closure has touched yet. Dropping that robot's map because it
    has no ``Component`` would be a silent data loss bug, and inventing a
    shared component for two such robots would be exactly the unproven merge
    this module exists to refuse. So an uncovered robot gets its own singleton
    component instead, with an id past the end of the real ones so it can
    never collide with -- or be confused for -- an actual verified merge.
    """
    parts = [(component.component_id, component.robots) for component in graph.components]
    covered = {robot_id for _, robots in parts for robot_id in robots}
    orphans = sorted(robot_ids - covered)
    next_id = max((component_id for component_id, _ in parts), default=-1) + 1
    parts.extend((next_id + i, frozenset({robot_id})) for i, robot_id in enumerate(orphans))
    return parts


class _Contribution(NamedTuple):
    """One keyframe's finished input to any grid it belongs in.

    Computed once per keyframe per render (see :func:`_keyframe_contributions`)
    and reused by every partition that includes it, because the posed points do
    not depend on which grouping is being drawn.
    """

    origin_xy: np.ndarray
    points_world: np.ndarray  # [n, 2] world XY, already filtered


def _keyframe_contributions(
    graph: OptimizedGraph,
    keyframes_by_id: dict[KeyframeId, Keyframe],
    config: RenderConfig,
) -> dict[KeyframeId, _Contribution]:
    """Pose and filter every solved keyframe once, in world frame.

    Keyframes with no optimized pose (not yet solved) are skipped rather than
    guessed at.
    """
    min_z, max_z = config.height_limits()
    contributions: dict[KeyframeId, _Contribution] = {}

    for kf_id, pose in graph.poses.items():
        keyframe = keyframes_by_id.get(kf_id)
        if keyframe is None:
            continue
        points = keyframe.points.astype(np.float64, copy=False)
        origin_xy = pose[:2, 3]

        # Height band and range are evaluated in the BASE frame at capture,
        # before the world transform: floor_z is a per-robot sensor-mount
        # calibration (see scout_mini.yaml), not a property of wherever the
        # optimizer happened to place this keyframe in world. This assumes
        # T_world_base does not roll or pitch the vertical axis, true for the
        # ground-vehicle fleets this module targets (see tests/synthetic.py's
        # planar yaw_pose) and the same assumption the height-band config
        # itself makes.
        planar_range = np.linalg.norm(points[:, :2], axis=1)
        in_band = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        keep = in_band & (planar_range <= config.max_range_m)

        if not np.any(keep):
            contributions[kf_id] = _Contribution(origin_xy, np.zeros((0, 2), dtype=np.float64))
            continue
        contributions[kf_id] = _Contribution(
            origin_xy, transform_points(pose, points[keep])[:, :2]
        )
    return contributions


def _render_component(
    component_id: int,
    robots: frozenset[str],
    all_contributions: dict[KeyframeId, _Contribution],
    config: RenderConfig,
) -> RenderedGrid:
    kf_ids = sorted(
        (kf_id for kf_id in all_contributions if kf_id.robot_id in robots),
        key=lambda kf_id: (kf_id.robot_id, kf_id.seq),
    )

    # Pass 1: bounds from the content this grid will actually hold -- every
    # sensor origin (so an entirely-empty keyframe still contributes its
    # trajectory position) plus every kept point. Bounds come from content, not
    # a fixed size: see the module docstring's "render as a rendering" framing,
    # an unexplored robot doesn't get a 40x24m grid just because that's what
    # some other component happened to need.
    contributions: list[_Contribution] = []
    bounds_min = np.array([math.inf, math.inf])
    bounds_max = np.array([-math.inf, -math.inf])

    for kf_id in kf_ids:
        origin_xy, points_world = all_contributions[kf_id]
        bounds_min = np.minimum(bounds_min, origin_xy)
        bounds_max = np.maximum(bounds_max, origin_xy)

        if points_world.shape[0]:
            bounds_min = np.minimum(bounds_min, points_world.min(axis=0))
            bounds_max = np.maximum(bounds_max, points_world.max(axis=0))
        contributions.append(_Contribution(origin_xy, points_world))

    if not contributions or not np.all(np.isfinite(bounds_min)):
        # No keyframe, or every keyframe's returns fell outside the height
        # band / range cap: there is nothing to say yet. A 1x1 UNKNOWN grid is
        # an honest statement of that, not an error.
        cells = np.full((1, 1), UNKNOWN, dtype=np.int8)
        return RenderedGrid(component_id, robots, config.native_map_resolution, 1, 1, 0.0, 0.0, cells)

    meta = _fit_grid(
        float(bounds_min[0]),
        float(bounds_max[0]),
        float(bounds_min[1]),
        float(bounds_max[1]),
        config.native_map_resolution,
        config.native_map_padding_m,
        config.native_map_max_cells,
    )

    # Pass 2: rasterize. Log-odds evidence accumulation.
    # Each hit adds positive evidence (+3), while free-space rays clear negative evidence (-1).
    free_counts = np.zeros((meta.height, meta.width), dtype=np.int32)
    hit_counts = np.zeros((meta.height, meta.width), dtype=np.int32)
    for origin_xy, points_world in contributions:
        if config.retain_free_space:
            _rasterize_free(origin_xy, points_world, meta, free_counts)
        if points_world.shape[0] == 0:
            continue
        grid_x, grid_y = _grid_index(points_world, meta)
        np.add.at(hit_counts, (grid_y, grid_x), 1)

    cells = np.full((meta.height, meta.width), UNKNOWN, dtype=np.int8)
    if config.retain_free_space:
        cells[free_counts > 0] = FREE
    occupied_mask = (hit_counts > 0) & ((hit_counts * 3) >= free_counts)
    cells[occupied_mask] = OCCUPIED

    return RenderedGrid(
        component_id, robots, meta.resolution, meta.width, meta.height, meta.origin_x, meta.origin_y, cells
    )


def _fit_grid(
    min_x: float, max_x: float, min_y: float, max_y: float,
    resolution: float, padding_m: float, max_cells: int,
) -> _Meta:
    """Bounds from content, plus a fixed cell-count cap that degrades by
    coarsening resolution rather than clamping the extent.

    Clamping bounds would silently cut off already-explored area -- exactly
    the kind of invisible lie this whole module exists to avoid; the operator
    would see a truncated map with no indication anything was cut. Coarsening
    is a visible, quantifiable degradation instead: the whole explored area is
    still there, just at lower fidelity. Cost scales with area, so one
    `sqrt(overflow_ratio)` jump lands very close to the cap; the loop is a
    bounded safety net for the rounding lost to `floor`/`ceil` on cell
    boundaries, not a per-cell operation.
    """
    min_x, max_x = min_x - padding_m, max_x + padding_m
    min_y, max_y = min_y - padding_m, max_y + padding_m
    res = resolution
    for _ in range(64):
        min_cell_x, max_cell_x = math.floor(min_x / res), math.floor(max_x / res)
        min_cell_y, max_cell_y = math.floor(min_y / res), math.floor(max_y / res)
        width = max_cell_x - min_cell_x + 1
        height = max_cell_y - min_cell_y + 1
        if width * height <= max_cells:
            return _Meta(res, width, height, min_cell_x * res, min_cell_y * res)
        res *= max(math.sqrt((width * height) / max_cells), 1.01)
    raise RuntimeError("could not fit render grid within max_cells by coarsening resolution")


def _grid_index(points_xy: np.ndarray, meta: _Meta) -> tuple[np.ndarray, np.ndarray]:
    """World XY to grid indices. Clip is a defensive no-op in the common case:
    bounds are derived from these same points, so it only fires on the rare
    float boundary that floors to exactly `width`/`height`."""
    grid_x = np.floor((points_xy[:, 0] - meta.origin_x) / meta.resolution).astype(np.int64)
    grid_y = np.floor((points_xy[:, 1] - meta.origin_y) / meta.resolution).astype(np.int64)
    np.clip(grid_x, 0, meta.width - 1, out=grid_x)
    np.clip(grid_y, 0, meta.height - 1, out=grid_y)
    return grid_x, grid_y


def _rasterize_free(origin_xy: np.ndarray, ends_xy: np.ndarray, meta: _Meta, free: np.ndarray) -> None:
    """Mark every cell strictly between ``origin_xy`` and each row of
    ``ends_xy`` free, for every ray of one keyframe at once.

    This is a vectorized supercover line walk: rather than Bresenham-stepping
    one ray at a time (a Python loop over rays, and then over cells within
    each ray -- the thing this module is explicitly required not to do), every
    ray is parametrized as ``origin + t * (end - origin)`` for
    ``t = 0, 1/n, ..., (n-1)/n`` where ``n`` is that ray's own Chebyshev
    distance in cells, stacked into one ``(max_steps, n_rays)`` matrix and
    floored to cells in a single call. Rays shorter than the batch's longest
    ray simply have their trailing steps masked off by ``valid``. The endpoint
    itself (``t == 1``) is never reached because ``t`` stops at
    ``(n-1)/n`` -- the caller marks it OCCUPIED separately, and a cell should
    never be both.
    """
    if ends_xy.shape[0] == 0:
        return
    origin_x = (origin_xy[0] - meta.origin_x) / meta.resolution
    origin_y = (origin_xy[1] - meta.origin_y) / meta.resolution
    end_x = (ends_xy[:, 0] - meta.origin_x) / meta.resolution
    end_y = (ends_xy[:, 1] - meta.origin_y) / meta.resolution
    delta_x = end_x - origin_x
    delta_y = end_y - origin_y

    n_steps = np.maximum(1, np.ceil(np.maximum(np.abs(delta_x), np.abs(delta_y))).astype(np.int64))

    # Rays are walked in length-sorted batches, not all at once. The step
    # matrix is (longest ray in the batch) x (rays in the batch), so one 60 m
    # return in a keyframe otherwise sizes the whole matrix: 40k points at 5 cm
    # measured 1.71 s and 1.58 GB of peak allocation for a SINGLE keyframe,
    # against a render that walks every keyframe of every robot from scratch
    # on every optimize. Sorting first means each batch's longest ray is close
    # to its own rays' lengths, so the padding that dominated that number is
    # gone as well.
    order = np.argsort(n_steps, kind="stable")
    start = 0
    while start < order.shape[0]:
        end = _batch_end(order, n_steps, start)
        rays = order[start:end]
        start = end

        batch_steps = n_steps[rays]
        step_index = np.arange(int(batch_steps[-1]), dtype=np.float64)[:, None]
        valid = step_index < batch_steps[None, :]
        t = step_index / batch_steps[None, :].astype(np.float64)

        grid_x = np.floor(origin_x + t * delta_x[rays][None, :]).astype(np.int64)
        grid_y = np.floor(origin_y + t * delta_y[rays][None, :]).astype(np.int64)
        valid &= (grid_x >= 0) & (grid_x < meta.width) & (grid_y >= 0) & (grid_y < meta.height)
        np.add.at(free, (grid_y[valid], grid_x[valid]), 1)


def _batch_end(order: np.ndarray, n_steps: np.ndarray, start: int) -> int:
    """Largest ``end`` whose batch fits :data:`_RAY_BATCH_ELEMENTS`.

    ``order`` is sorted ascending by ``n_steps``, so the batch's element count
    ``(end - start) * n_steps[order[end - 1]]`` is a product of two
    non-decreasing sequences and therefore monotone in ``end`` -- which is what
    makes a binary search valid here. Always returns at least ``start + 1`` so
    a single ray longer than the whole budget still makes progress instead of
    looping forever.
    """
    lo, hi = start + 1, order.shape[0]
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if (mid - start) * int(n_steps[order[mid - 1]]) <= _RAY_BATCH_ELEMENTS:
            lo = mid
        else:
            hi = mid - 1
    return lo
