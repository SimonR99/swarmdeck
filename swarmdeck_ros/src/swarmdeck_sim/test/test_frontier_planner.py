"""Deterministic tests for the ROS-free coordinated exploration planner."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))

from frontier_planner import (  # noqa: E402
    CommonGrid,
    Frontier,
    GridSnapshot,
    RobotState,
    allocate_frontiers,
    extract_frontiers,
    inverse_transform_point,
    merge_grids,
    rendezvous_slots,
    transform_point,
)


def _common(cells: np.ndarray, resolution: float = 0.25) -> CommonGrid:
    return CommonGrid(
        np.asarray(cells, dtype=np.int8),
        np.zeros_like(cells, dtype=np.uint8),
        resolution,
        0.0,
        0.0,
    )


def test_transform_round_trip_does_not_depend_on_odometry() -> None:
    transform = (4.2, -1.7, 0.63)
    point = (-2.0, 3.5)
    assert inverse_transform_point(transform_point(point, transform), transform) == (
        pytest.approx(point[0]),
        pytest.approx(point[1]),
    )


def test_rotated_robot_map_merges_in_the_shared_study_frame() -> None:
    cells = np.full((3, 3), -1, dtype=np.int8)
    cells[1, 2] = 100
    cells[1, 1] = 0
    merged = merge_grids(
        {"r": GridSnapshot(cells, 1.0, -1.5, -1.5)},
        {"r": (2.0, 1.0, math.pi / 2.0)},
        size_m=8.0,
        resolution=1.0,
    )
    # Local (+1, 0) rotates to world (2, 2).
    row, col = merged.cell(2.0, 2.0)
    assert merged.cells[row, col] == 100
    assert merged.observations[row, col] == 1


def test_occupied_wins_and_overlap_is_counted_once_per_robot() -> None:
    free = GridSnapshot(np.zeros((2, 2), dtype=np.int8), 1.0, -1.0, -1.0)
    occupied_cells = np.zeros((2, 2), dtype=np.int8)
    occupied_cells[0, 0] = 100
    occupied = GridSnapshot(occupied_cells, 1.0, -1.0, -1.0)
    merged = merge_grids(
        {"a": free, "b": occupied},
        {"a": (0.0, 0.0, 0.0), "b": (0.0, 0.0, 0.0)},
        size_m=4.0,
        resolution=1.0,
    )
    row, col = merged.cell(-0.5, -0.5)
    assert merged.cells[row, col] == 100
    assert merged.observations[row, col] == 2


def test_optimistic_union_is_only_a_free_precedence_reachability_layer() -> None:
    free = GridSnapshot(np.zeros((1, 1), dtype=np.int8), 1.0, 0.0, 0.0)
    occupied = GridSnapshot(np.full((1, 1), 100, dtype=np.int8), 1.0, 0.0, 0.0)
    transforms = {"free": (0.0, 0.0, 0.0), "wall": (0.0, 0.0, 0.0)}
    conservative = merge_grids(
        {"free": free, "wall": occupied},
        transforms,
        size_m=2.0,
        resolution=1.0,
    )
    optimistic = merge_grids(
        {"free": free, "wall": occupied},
        transforms,
        size_m=2.0,
        resolution=1.0,
        occupied_wins=False,
    )
    row, col = conservative.cell(0.5, 0.5)
    assert conservative.cells[row, col] == 100
    assert optimistic.cells[row, col] == 0
    assert conservative.observations[row, col] == 2
    assert optimistic.observations[row, col] == 2


def test_frontier_viewpoint_stays_on_safe_known_free_space() -> None:
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[20:60, 20:60] = 0
    # One obstacle well inside the known patch must not become a goal.
    cells[38:43, 38:43] = 100
    grid = _common(cells, 0.10)
    frontiers = extract_frontiers(
        grid,
        clearance_m=0.25,
        gain_radius_m=1.0,
        min_cluster_cells=5,
    )
    assert frontiers
    for frontier in frontiers:
        assert grid.cells[frontier.row, frontier.col] == 0
        assert not (35 <= frontier.row <= 45 and 35 <= frontier.col <= 45)
        assert frontier.information_m2 > 0.0


def test_joint_assignment_sends_robots_to_distinct_nearby_frontiers() -> None:
    cells = np.zeros((50, 100), dtype=np.int8)
    grid = _common(cells, 0.20)
    frontiers = [
        Frontier(25, 10, 2.1, 5.1, 10, 4.0),
        Frontier(25, 90, 18.1, 5.1, 10, 4.0),
    ]
    robots = [
        RobotState("left", 1.0, 5.0, 0.2),
        RobotState("right", 19.0, 5.0, 0.2),
    ]
    assignments = allocate_frontiers(grid, robots, frontiers)
    assert {item.robot_id for item in assignments} == {"left", "right"}
    by_robot = {item.robot_id: item for item in assignments}
    assert by_robot["left"].frontier.x < by_robot["right"].frontier.x


def test_reserved_goal_predicts_and_suppresses_teammate_overlap() -> None:
    cells = np.zeros((40, 80), dtype=np.int8)
    grid = _common(cells, 0.25)
    frontiers = [
        Frontier(20, 20, 5.1, 5.1, 10, 5.0),
        Frontier(20, 60, 15.1, 5.1, 10, 5.0),
    ]
    assignment = allocate_frontiers(
        grid,
        [RobotState("idle", 6.0, 5.0, 0.2)],
        frontiers,
        reserved=[(5.0, 5.0)],
        goal_separation_m=3.0,
    )
    assert len(assignment) == 1
    assert assignment[0].frontier.x > 10.0


def test_idle_robot_defers_deadhead_to_active_robot_near_frontier() -> None:
    cells = np.zeros((30, 100), dtype=np.int8)
    grid = _common(cells, 0.20)
    frontier = Frontier(15, 90, 18.1, 3.1, 10, 5.0)
    idle = RobotState("idle", 1.0, 3.0, 0.2)

    # The idle robot's ~17 m cross-map leg is unnecessary because a teammate
    # is already driving to x=16 and can receive this frontier next.
    assert allocate_frontiers(
        grid,
        [idle],
        [frontier],
        reserved=[(16.0, 3.0)],
    ) == []

    # When nobody is active, completeness wins and the distant leg is sent.
    assignment = allocate_frontiers(grid, [idle], [frontier])
    assert len(assignment) == 1
    assert assignment[0].frontier == frontier


def test_optimistic_reachability_prevents_false_early_termination() -> None:
    conservative_cells = np.zeros((30, 60), dtype=np.int8)
    conservative_cells[:, 29:31] = 100
    conservative = _common(conservative_cells, 0.20)
    navigation = _common(np.zeros_like(conservative_cells), 0.20)
    frontier = Frontier(15, 50, 10.1, 3.1, 10, 4.0)
    robot = RobotState("left", 1.0, 3.0, 0.2)

    assert allocate_frontiers(conservative, [robot], [frontier]) == []
    assignment = allocate_frontiers(
        conservative,
        [robot],
        [frontier],
        navigation_grid=navigation,
    )
    assert len(assignment) == 1
    assert assignment[0].frontier == frontier


def test_rectangular_robot_clearance_keeps_a_real_corridor_connected() -> None:
    cells = np.zeros((30, 60), dtype=np.int8)
    cells[:, 29:31] = 100
    cells[12:19, 29:31] = 0  # 1.4 m doorway at 0.20 m/cell
    grid = _common(cells, 0.20)
    frontier = Frontier(15, 50, 10.1, 3.1, 10, 4.0)

    circumscribed = RobotState("bunker", 1.0, 3.0, 0.643)
    assert allocate_frontiers(grid, [circumscribed], [frontier]) == []

    rectangular = RobotState(
        "bunker",
        1.0,
        3.0,
        0.643,
        navigation_clearance_m=0.509,
    )
    assignment = allocate_frontiers(grid, [rectangular], [frontier])
    assert len(assignment) == 1
    assert assignment[0].frontier == frontier


def test_rendezvous_preserves_order_and_requires_only_one_common_leg() -> None:
    starts = {
        "r0": (-9.0, 0.0, 0.0),
        "r1": (-3.0, 0.0, 0.0),
        "r2": (3.0, 0.0, math.pi),
        "r3": (9.0, 0.0, math.pi),
    }
    slots = rendezvous_slots(starts, 1.55)
    xs = [slots[f"r{i}"][0] for i in range(4)]
    assert xs == sorted(xs)
    assert np.diff(xs) == pytest.approx([1.55, 1.55, 1.55])
    assert sum(abs(slots[r][0] - starts[r][0]) for r in starts) < 18.0
