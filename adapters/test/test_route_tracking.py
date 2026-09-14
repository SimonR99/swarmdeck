import math
from types import SimpleNamespace

import numpy as np
import pytest

from adapters.route_tracking import GridCollisionChecker, RouteTracker
from adapters.runtime import AdapterTelemetryMixin


def checker(cells=None, clearance=0.35):
    return GridCollisionChecker(
        SimpleNamespace(
            cells=np.zeros((300, 300), dtype=np.int8) if cells is None else cells,
            resolution=0.1,
            origin_x=-15.0,
            origin_y=-15.0,
        ),
        clearance,
    )


def path(*points):
    return [{"x": x, "y": y} for x, y in points]


def test_recorded_doorway_poses_keep_target_ahead():
    # Route geometry reconstructed from the logged clamped vertex and final
    # goal; these are measured poses from 11:53:20-11:53:28, not a simulated drive.
    tracker = RouteTracker(path((5.792, 3.464), (5.282, 1.534), (3.804, -2.399)))
    poses = [
        (5.792, 3.464, -0.596),
        (5.945, 2.685, -1.694),
        (5.756, 1.890, -1.836),
        (5.666, 1.493, -1.811),
        (5.591, 1.142, -1.812),
        (5.593, 1.171, -1.770),
        (5.636, 1.518, -1.565),
        (5.635, 1.824, -1.538),
        (5.638, 1.688, -1.623),
        (5.598, 1.354, -1.767),
        (5.563, 1.222, -1.764),
    ]
    old_progress = old_target = 0
    for x, y, yaw in poses:
        target = tracker.target({"x": x, "y": y}, checker())
        assert target is not None
        forward = (target["x"] - x) * math.cos(yaw) + (target["y"] - y) * math.sin(yaw)
        assert forward > 0
        assert tracker.progress >= old_progress
        assert tracker.target_progress >= old_target
        old_progress, old_target = tracker.progress, tracker.target_progress
    assert tracker.target_progress > math.dist((5.792, 3.464), (5.282, 1.534))


def test_connection_cannot_cut_occupied_or_unknown_corner():
    for occupancy in (100, -1):
        cells = np.zeros((300, 300), dtype=np.int8)
        cells[150:155, 155:160] = occupancy
        clear = checker(cells, clearance=0)
        assert not clear((0.05, 0.05), (1.05, 1.05))
        tracker = RouteTracker(
            path((0.05, 0.05), (0.05, 1.05), (1.05, 1.05)), lookahead_m=1.6
        )
        target = tracker.target({"x": 0.05, "y": 0.05}, clear)
        assert target is not None
        assert clear((0.05, 0.05), (target["x"], target["y"]))


def test_blocked_rejoin_holds_without_regressing_progress():
    tracker = RouteTracker(path((0, 0), (2, 0), (5, 0)))
    tracker.target({"x": 1, "y": 0.5}, checker())
    old = tracker.progress, tracker.target_progress
    assert tracker.target({"x": 2.5, "y": 0.6}, lambda a, b: False) is None
    assert (tracker.progress, tracker.target_progress) == old
    target = tracker.target({"x": 2.5, "y": 0.6}, checker())
    assert target["x"] > 2.5


def test_loop_intersection_does_not_jump_to_return_leg():
    tracker = RouteTracker(path((0, 0), (5, 0), (5, 5), (0, 5), (0, 0), (0, -5)))
    target = tracker.target({"x": 0, "y": 0}, checker())
    assert target["x"] > 0 and target["y"] == 0
    assert tracker.progress == 0


def test_clearance_and_boundary_are_enforced():
    cells = np.zeros((300, 300), dtype=np.int8)
    cells[153, 155] = 100
    assert checker(cells, 0)((0.05, 0.05), (1.05, 0.05))
    assert not checker(cells, 0.35)((0.05, 0.05), (1.05, 0.05))
    assert not checker()((-15, 0), (-14, 0))


def test_duplicate_points_and_exact_goal():
    tracker = RouteTracker(path((0, 0), (0, 0), (1, 0), (1, 0)))
    assert tracker.target({"x": 0.9, "y": 0}, checker()) == {"x": 1, "y": 0}


@pytest.mark.parametrize("distance_m", [20.0, 50.0, 100.0])
def test_distant_route_coordinates_stay_fixed_while_target_advances(distance_m):
    route = path((0.0, 0.0), (distance_m, 0.0))
    tracker = RouteTracker(route, lookahead_m=1.0)
    original = tuple(tracker.points)

    targets = [
        tracker.target({"x": x, "y": 0.0}, lambda _a, _b: True)["x"]
        for x in (0.0, 1.0, 2.0)
    ]

    assert targets == pytest.approx([1.0, 2.0, 3.0])
    assert tuple(tracker.points) == original
    assert tracker.points[-1] == (distance_m, 0.0)


def test_shared_adapter_api_holds_for_missing_or_stale_map():
    import time

    bridge = AdapterTelemetryMixin()
    bridge.cfg = {}
    route = path((0, 0), (5, 0))
    assert bridge.navigation_route_target(route, {"x": 0, "y": 0}) is None
    snapshot = checker().map
    bridge._nav_map = SimpleNamespace(
        cached=snapshot, last_success_at=time.monotonic() - 11
    )
    assert bridge.navigation_route_target(route, {"x": 0, "y": 0}) is None
    bridge._nav_map.last_success_at = time.monotonic()
    assert bridge.navigation_route_target(route, {"x": 0, "y": 0}) is not None
    assert not bridge._nav_route_blocked
