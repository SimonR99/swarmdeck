import math

import numpy as np
import pytest

from swarmdeck_server.mapsvc.grid_meta import GridMeta
from swarmdeck_server.mapsvc.planner import PathPlanningError, plan_global_path


def plan(grid, start, goal, clearance=0):
    return plan_global_path(
        grid,
        GridMeta(0.1, grid.shape[1], grid.shape[0], -1, -2),
        {"x": -1 + start[0] * 0.1, "y": -2 + start[1] * 0.1},
        {"x": -1 + goal[0] * 0.1, "y": -2 + goal[1] * 0.1},
        clearance,
    )


def test_open_room_has_straight_route_and_exact_endpoints():
    path = plan(np.zeros((50, 50)), (2.2, 3.7), (42.8, 31.1))
    assert len(path) == 2
    assert path[0] == pytest.approx({"x": -0.78, "y": -1.63})
    assert path[-1] == pytest.approx({"x": 3.28, "y": 1.11})


def test_unreachable_goal_does_not_fabricate_straight_route():
    grid = np.zeros((30, 30))
    grid[:, 15] = 100
    with pytest.raises(PathPlanningError):
        plan(grid, (5.5, 10.5), (25.5, 10.5))


@pytest.mark.parametrize("goal", [(15.5, 15.5), (30.5, 10.5), (-0.1, 10.5)])
def test_blocked_and_outside_goals_are_rejected(goal):
    grid = np.zeros((30, 30))
    grid[15, 15] = 100
    with pytest.raises(PathPlanningError):
        plan(grid, (5.5, 10.5), goal)


def test_diagonal_cannot_squeeze_between_touching_obstacles():
    grid = np.array([[0, 100], [100, 0]])
    with pytest.raises(PathPlanningError):
        plan(grid, (0.5, 0.5), (1.5, 1.5))


def test_same_cell_goal_still_checks_collision():
    with pytest.raises(PathPlanningError):
        plan(np.full((5, 5), 100), (2.1, 2.1), (2.8, 2.8))


def test_simplified_segments_keep_clearance_around_wall():
    grid = np.zeros((70, 70))
    grid[:45, 35] = 100
    path = plan(grid, (10.5, 10.5), (60.5, 10.5), clearance=0.2)
    assert len(path) > 2
    # Independently sample the returned world segments at sub-cell spacing.
    for a, b in zip(path, path[1:]):
        count = math.ceil(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / 0.002)
        for t in np.linspace(0, 1, count + 1):
            x = (a["x"] + t * (b["x"] - a["x"]) + 1) / 0.1
            y = (a["y"] + t * (b["y"] - a["y"]) + 2) / 0.1
            assert not (33 <= x < 38 and y < 47)


def test_clearance_is_enforced_for_subcell_radius_without_scipy():
    grid = np.zeros((20, 20))
    grid[10, 10] = 100
    with pytest.raises(PathPlanningError):
        plan(grid, (2.5, 2.5), (9.5, 10.5), clearance=0.01)
