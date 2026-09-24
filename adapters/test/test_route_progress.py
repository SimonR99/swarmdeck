"""The route progress watchdog: progress along the controller's path.

Measured 2026-09-17 in the Bistro world: robot_1 reached a 7 cm ridge at 2.5 m
and rocked 0.3 m back and forth on it for three minutes with nav_status still
`active`. Nav2's SimpleProgressChecker measures displacement from a reference
pose, so the rocking satisfied it and `Failed to make progress` was never
raised; none of the adapter recovery ran. These tests cover the pure progress
function, the watchdog timing, the simulation bridge's tick, and the failure
reaching the exploration recovery path. What they cannot show is Nav2 itself
honouring the cancellation; that is a live test.
"""

from __future__ import annotations

from concurrent.futures import Future
import math
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from adapters.exploration import (
    PlannerPath,
    PlannerPose,
    is_physical_no_progress_failure,
)
from adapters.route_progress import (
    DEFAULT_MIN_PROGRESS_M,
    DEFAULT_TIMEOUT_S,
    RouteProgress,
    RouteProgressWatchdog,
    advance_route_progress,
    route_points,
    route_progress_tick,
)


def straight(length=10.0, step=0.5, y=0.0):
    """A route along +x, one point every ``step`` metres."""
    count = int(round(length / step)) + 1
    return tuple((i * step, y) for i in range(count))


def walk(points, poses):
    """Feed poses through advance_route_progress, returning progress distances."""
    progress = None
    reached = []
    for pose in poses:
        progress = advance_route_progress(points, pose, progress)
        reached.append(progress.distance_m)
    return reached


# ------------------------------------------------------------ pure function


def test_progress_grows_with_the_robot_along_the_route():
    reached = walk(straight(), [(0.0, 0.0), (1.0, 0.1), (2.0, -0.1), (2.5, 0.0)])
    assert reached == pytest.approx([0.0, 1.0, 2.0, 2.5])


def test_progress_is_monotone_under_back_and_forth_motion():
    """Rocking on a ridge keeps the furthest arc length reached, never less."""
    poses = [(2.5, 0.0)]
    for _ in range(20):
        poses += [(2.2, 0.05), (2.5, 0.0), (2.35, -0.05), (2.5, 0.02)]
    reached = walk(straight(), poses)
    assert reached[0] == pytest.approx(2.5)
    assert all(later >= earlier for earlier, later in zip(reached, reached[1:]))
    assert max(reached) == pytest.approx(2.5, abs=1e-6)


def test_route_starting_behind_the_robot_is_joined_where_the_robot_is():
    """A replacement route planned from a stale pose starts behind the robot."""
    points = straight()
    progress = advance_route_progress(points, (1.3, 0.2))
    assert progress.distance_m == pytest.approx(1.3)
    assert progress.index == 2
    assert progress.offset_m == pytest.approx(0.2)
    # The robot never went backwards to the start, and moving on counts.
    later = advance_route_progress(points, (1.8, 0.0), progress)
    assert later.distance_m == pytest.approx(1.8)


def test_progress_does_not_jump_to_a_later_leg_of_a_self_crossing_route():
    """A loop returning next to its start must not teleport progress."""
    points = ((0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.3), (-5.0, 0.3))
    progress = advance_route_progress(points, (0.1, 0.1))
    assert progress.index == 0
    assert progress.distance_m == pytest.approx(0.1)
    progress = advance_route_progress(points, (1.0, 0.1), progress)
    assert progress.index == 0
    assert progress.distance_m == pytest.approx(1.0)


def test_progress_is_kept_when_the_robot_leaves_the_route():
    points = straight()
    progress = advance_route_progress(points, (3.0, 0.0))
    strayed = advance_route_progress(points, (3.0, 2.0), progress)
    assert strayed.distance_m == pytest.approx(3.0)
    assert strayed.offset_m == pytest.approx(2.0)


def test_progress_is_clamped_to_the_end_of_the_route():
    points = straight(length=2.0)
    progress = advance_route_progress(points, (5.0, 0.0))
    assert progress.distance_m == pytest.approx(2.0)
    assert progress.index == len(points) - 2


@pytest.mark.parametrize(
    "points, pose",
    [
        ((), (0.0, 0.0)),
        (((1.0, 1.0),), (0.0, 0.0)),
        (straight(), None),
        (straight(), (math.nan, 0.0)),
        (straight(), {"x": 1.0}),
    ],
)
def test_degenerate_routes_and_poses_yield_no_progress(points, pose):
    assert advance_route_progress(points, pose) is None


def test_a_stale_previous_index_is_ignored_rather_than_trusted():
    points = straight(length=2.0)
    stale = RouteProgress(index=99, distance_m=50.0, offset_m=0.0)
    progress = advance_route_progress(points, (1.0, 0.0), stale)
    assert progress.distance_m == pytest.approx(1.0)


def test_route_points_accepts_planner_paths_dicts_and_pairs():
    plan = PlannerPath(
        "r0/navigation_frame",
        1,
        (
            PlannerPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            PlannerPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),  # duplicate dropped
            PlannerPose(1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0),
        ),
    )
    assert route_points(plan) == ((0.0, 0.0), (1.0, 0.0))
    assert route_points([{"x": 0, "y": 0}, {"x": 2, "y": 0}]) == (
        (0.0, 0.0),
        (2.0, 0.0),
    )
    assert route_points([(0, 0), (0, 3)]) == ((0.0, 0.0), (0.0, 3.0))
    assert route_points(None) == ()
    assert route_points([{"x": 0, "y": 0}, {"x": math.inf, "y": 0}]) == ()


# ------------------------------------------------------------------ watchdog


def rocking(amplitude=0.3, at=2.5):
    """Pose sequence generator: the robot rocks on a ridge at ``at`` metres."""
    step = 0
    while True:
        offset = amplitude if step % 2 else 0.0
        yield {"x": at - offset, "y": 0.02 * (step % 3), "yaw": 0.0}
        step += 1


def test_watchdog_fires_once_at_the_timeout_without_route_progress():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight()
    poses = rocking()
    fired_at = []
    now = 100.0
    while now <= 100.0 + 45.0:
        if watchdog.observe(now, next(poses), path, True):
            fired_at.append(now)
        now = round(now + 0.2, 6)
    assert fired_at == [pytest.approx(130.0)], "fires exactly once, at the timeout"
    assert watchdog.progress.distance_m == pytest.approx(2.5, abs=1e-6)


def test_watchdog_never_fires_while_the_robot_advances():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight(length=40.0)
    now, x = 0.0, 0.0
    for _ in range(int(120 / 0.2)):  # two minutes at 0.05 m/s, 1 m per 20 s
        assert not watchdog.observe(now, {"x": x, "y": 0.0}, path, True)
        now += 0.2
        x += 0.01


def test_watchdog_counts_only_progress_along_the_route():
    """Displacement off the route, Nav2's measure, is not progress here."""
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight()
    fired = []
    for tick in range(200):
        now = tick * 0.2
        pose = {"x": 2.5, "y": 1.5 if tick % 2 else -1.5}  # 3 m of displacement
        if watchdog.observe(now, pose, path, True):
            fired.append(now)
    assert fired == [pytest.approx(30.0)]


def test_watchdog_resets_on_a_new_path():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    first, second = straight(), straight(y=1.0)
    poses = rocking()
    for tick in range(100):  # 20 s stalled on the first route
        assert not watchdog.observe(tick * 0.2, next(poses), first, True)
    for tick in range(100, 200):  # 20 s more, on a replacement route
        assert not watchdog.observe(tick * 0.2, next(poses), second, True)
    assert watchdog.observe(200 * 0.2 + 10.0, next(poses), second, True)


def test_watchdog_resets_on_explicit_reset_even_for_the_same_path_object():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight()
    poses = rocking()
    for tick in range(100):
        watchdog.observe(tick * 0.2, next(poses), path, True)
    watchdog.reset()
    assert not watchdog.observe(35.0, next(poses), path, True)
    assert watchdog.observe(65.0, next(poses), path, True)


def test_watchdog_never_fires_when_inactive():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight()
    poses = rocking()
    for tick in range(400):  # 80 s, far past the timeout
        assert not watchdog.observe(tick * 0.2, next(poses), path, False)
    # Becoming active starts the clock fresh rather than inheriting the stall.
    assert not watchdog.observe(80.0, next(poses), path, True)
    assert not watchdog.observe(109.9, next(poses), path, True)
    assert watchdog.observe(110.0, next(poses), path, True)


def test_watchdog_going_inactive_clears_a_stall_in_progress():
    watchdog = RouteProgressWatchdog(timeout_s=30.0, min_progress_m=0.5)
    path = straight()
    poses = rocking()
    for tick in range(140):  # 28 s
        watchdog.observe(tick * 0.2, next(poses), path, True)
    assert not watchdog.observe(28.2, next(poses), path, False)
    assert not watchdog.observe(31.0, next(poses), path, True)


def test_watchdog_is_disabled_by_a_zero_timeout():
    watchdog = RouteProgressWatchdog(timeout_s=0.0)
    assert not watchdog.enabled
    path = straight()
    poses = rocking()
    for tick in range(1000):
        assert not watchdog.observe(tick * 0.2, next(poses), path, True)


def test_watchdog_without_a_pose_gathers_no_evidence():
    watchdog = RouteProgressWatchdog(timeout_s=30.0)
    path = straight()
    for tick in range(400):
        assert not watchdog.observe(tick * 0.2, None, path, True)
    assert watchdog.progress is None


def test_watchdog_defaults_and_bounds():
    watchdog = RouteProgressWatchdog()
    assert watchdog.timeout_s == DEFAULT_TIMEOUT_S == 30.0
    assert watchdog.min_progress_m == DEFAULT_MIN_PROGRESS_M == 0.5
    odd = RouteProgressWatchdog(timeout_s="soon", min_progress_m=-1.0)
    assert odd.timeout_s == DEFAULT_TIMEOUT_S
    assert odd.min_progress_m == DEFAULT_MIN_PROGRESS_M
    assert RouteProgressWatchdog(timeout_s=math.inf).timeout_s == DEFAULT_TIMEOUT_S


def test_reason_is_bounded_and_recognised_as_physical_no_progress():
    watchdog = RouteProgressWatchdog(timeout_s=30.0)
    path = straight()
    poses = rocking()
    for tick in range(160):
        watchdog.observe(tick * 0.2, next(poses), path, True)
    reason = watchdog.reason()
    assert reason.startswith("no progress along the route for 30 s")
    assert "2.50 m of 10.00 m" in reason
    assert len(reason) <= 160
    assert is_physical_no_progress_failure(reason)
    assert not is_physical_no_progress_failure("progress pending")
    assert not is_physical_no_progress_failure("route progress unknown")


# ------------------------------------------------------ simulation bridge tick


class _ImmediateFuture:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value

    def add_done_callback(self, callback):
        callback(self)


def _sim_bridge(sim_module, **cfg):
    from test_adapter_sim_goals import _bridge

    bridge = _bridge(sim_module)
    bridge.cfg = sim_module.deep_merge(sim_module.TRANSPORT_DEFAULTS, cfg)
    bridge.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge.map_pose = lambda: dict(bridge.pose)
    bridge.map_pose_at = lambda *_a, **_k: None
    return bridge


def _plan(frame="r0/navigation_frame", revision=1):
    poses = tuple(PlannerPose(0.5 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) for i in range(21))
    return PlannerPath(frame, revision, poses)


def _accepted_handle():
    handle = MagicMock()
    handle.accepted = True
    handle.get_result_async.return_value = Future()
    return handle


def _submit(bridge, plan, expected_generation=None):
    handle = _accepted_handle()
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(handle)
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        generation = bridge.follow_path(plan, expected_generation=expected_generation)
    assert type(generation) is int
    assert bridge._goal_handle is handle
    return generation, handle


def _rock(bridge, clock, seconds, amplitude=0.3, at=2.5):
    """Run the state tick for ``seconds`` while the robot rocks at ``at``."""
    ticks = int(round(seconds / 0.2))
    for tick in range(ticks):
        clock[0] += 0.2
        bridge.pose = {"x": at - (amplitude if tick % 2 else 0.0), "y": 0.0}
        bridge.session_state_tick()


@pytest.fixture
def clock(sim_module, monkeypatch):
    clock = [1000.0]
    monkeypatch.setattr(sim_module.time, "monotonic", lambda: clock[0])
    return clock


def test_sim_tick_cancels_a_stalled_route_as_a_no_progress_failure(sim_module, clock):
    """The Bistro ridge: 0.3 m of rocking at 2.5 m, nav_status active forever."""
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    generation, handle = _submit(bridge, _plan())

    _rock(bridge, clock, 29.6)
    assert bridge.nav_status == "active", "no verdict before the timeout"
    handle.cancel_goal_async.assert_not_called()

    _rock(bridge, clock, 1.0)

    assert bridge.nav_status == "failed"
    reason = bridge._nav_failure_reason
    assert is_physical_no_progress_failure(reason)
    assert reason.startswith("no progress along the route for 30 s")
    handle.cancel_goal_async.assert_called_once()
    assert bridge._goal_generation == generation, "recovery keeps goal ownership"
    assert bridge._goal_handle is None
    assert bridge.goal is None and bridge.planned_path == []
    assert bridge.mode == "recover", "a failed goal arms the wedge escape as usual"
    assert bridge._escape_from == (2.5, 0.0)
    bridge.node.get_logger().warn.assert_called()

    # The late CANCELED result of the cancelled goal settles quietly instead
    # of relabelling the failure the recovery paths are about to consume.
    outcome = NS(status=sim_module.GoalStatus.STATUS_CANCELED, result=None)
    bridge._goal_result(_ImmediateFuture(outcome), generation, handle)
    assert bridge.nav_status == "failed"
    assert bridge._nav_failure_reason == reason
    assert bridge._route_stalled_handle is None
    assert not bridge._nav_quiet_unknown

    # And the failure fires once; later ticks do not cancel anything else.
    _rock(bridge, clock, 40.0)
    handle.cancel_goal_async.assert_called_once()
    assert bridge.nav_status == "failed"


def test_sim_tick_leaves_a_route_alone_while_the_robot_advances(sim_module, clock):
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    generation, handle = _submit(bridge, _plan())
    for tick in range(600):  # two minutes at 0.05 m/s
        clock[0] += 0.2
        bridge.pose = {"x": 0.01 * tick, "y": 0.0}
        bridge.session_state_tick()
    assert bridge.nav_status == "active"
    handle.cancel_goal_async.assert_not_called()


def test_sim_tick_restarts_the_clock_for_a_replacement_route(sim_module, clock):
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    generation, first = _submit(bridge, _plan())
    _rock(bridge, clock, 20.0)
    # A replacement submitted under the same generation, as recovery does.
    generation, second = _submit(
        bridge, _plan(revision=2), expected_generation=generation
    )
    _rock(bridge, clock, 20.0)
    assert bridge.nav_status == "active"
    first.cancel_goal_async.assert_not_called()
    second.cancel_goal_async.assert_not_called()
    _rock(bridge, clock, 10.2)
    assert bridge.nav_status == "failed"
    second.cancel_goal_async.assert_called_once()
    first.cancel_goal_async.assert_not_called()


def test_sim_tick_waits_for_the_controller_to_accept_the_goal(sim_module, clock):
    """Before acceptance there is no handle to cancel, so there is no verdict."""
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    bridge.path_client.send_goal_async.return_value = Future()  # never answers
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        bridge.follow_path(_plan())
    assert bridge.nav_status == "active" and bridge._goal_handle is None
    _rock(bridge, clock, 60.0)
    assert bridge.nav_status == "active"


def test_sim_tick_is_disabled_by_a_zero_timeout(sim_module, clock):
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=0)
    generation, handle = _submit(bridge, _plan())
    _rock(bridge, clock, 120.0)
    assert bridge.nav_status == "active"
    handle.cancel_goal_async.assert_not_called()


def test_sim_tick_supervises_a_route_in_the_odometry_frame(sim_module, clock):
    """Routes are planned in the odometry frame: the odom -> base_link link is
    the pose there, with the wheel topic as the fallback map_pose uses."""
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    bridge.map_pose = lambda: {"x": 99.0, "y": 99.0, "yaw": 0.0}
    generation, handle = _submit(bridge, _plan(frame="r0/odom"))
    ticks = int(round(31.0 / 0.2))
    for tick in range(ticks):
        clock[0] += 0.2
        bridge._odom_to_base = {"x": 2.5 - (0.3 if tick % 2 else 0.0), "y": 0.0}
        bridge.session_state_tick()
    assert bridge.nav_status == "failed"
    handle.cancel_goal_async.assert_called_once()
    assert "no progress along the route" in bridge._nav_failure_reason

    fallback = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    fallback._odom_to_base = None
    fallback._odom_topic_pose = {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert fallback._route_progress_pose("r0/odom") == {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert fallback._route_progress_pose("other/frame") is None


def test_sim_tick_does_not_supervise_a_route_in_another_frame(sim_module, clock):
    """Neither the map nor the odometry frame: fail closed, warn once."""
    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    generation, handle = _submit(bridge, _plan(frame="r0/other"))
    _rock(bridge, clock, 60.0)
    assert bridge.nav_status == "active"
    handle.cancel_goal_async.assert_not_called()
    warned = [
        call.args[0]
        for call in bridge.node.get_logger().warn.call_args_list
        if "not supervised" in str(call.args[0])
    ]
    assert len(warned) == 1, "one warning per route, not one per tick"


def test_tick_helper_treats_a_missing_bridge_config_as_defaults(sim_module):
    bridge = _sim_bridge(sim_module)
    del bridge.cfg
    assert route_progress_tick(bridge, now=0.0) is False
    assert bridge._route_watchdog.timeout_s == DEFAULT_TIMEOUT_S


# ------------------------------------------- reaching the exploration recovery


def test_sim_watchdog_failure_reaches_exploration_recovery(sim_module, clock):
    """The chain the ridge never triggered: watchdog -> failed -> bounded replan."""
    from adapters.test.test_exploration import path as planner_msg, rig

    bridge = _sim_bridge(sim_module, route_progress_timeout_s=30.0)
    bridge.node.get_clock().now().nanoseconds = 1_000_000_000
    _mock_bridge, explorer = rig()
    explorer.bridge = bridge
    explorer.frame = "r0/navigation_frame"
    bridge.exploration = explorer
    replacement_request = Future()
    explorer.replan_client.call_async.return_value = replacement_request

    handle = _accepted_handle()
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(handle)
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        explorer.start()
        explorer.pending.set_result(NS(success=True, message=""))
        explorer.on_path(
            planner_msg(
                frame="r0/navigation_frame",
                stamp=2,
                points=[(0.5 * i, 0.0, 0.0) for i in range(21)],
            )
        )
    assert explorer.status == "exploring"
    first_goal_generation = explorer.executing_goal_generation
    assert first_goal_generation == bridge._goal_generation
    assert bridge._goal_handle is handle

    _rock(bridge, clock, 30.6)
    assert bridge.nav_status == "failed"
    assert is_physical_no_progress_failure(bridge._nav_failure_reason)

    explorer.last_link = clock[0]  # the state loop refreshes this every tick
    explorer.tick()

    assert explorer.active, (
        f"a physical no-progress failure is recoverable: "
        f"{explorer.status} {explorer.reason!r}"
    )
    assert explorer.status == "waiting"
    assert explorer.executing_plan is None
    assert explorer.controller_replan_attempts == 1
    explorer.replan_client.call_async.assert_called_once()

    # The replacement route executes under the same goal generation, and the
    # late result of the cancelled route cannot disturb it.
    replacement_request.set_result(NS(success=True, message=""))
    assert explorer.awaiting_replan_path
    replacement = _accepted_handle()
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(replacement)
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        explorer.on_path(
            planner_msg(
                frame="r0/navigation_frame",
                stamp=3,
                points=[(2.5 + 0.5 * i, 0.5, 0.0) for i in range(11)],
            )
        )
    assert explorer.executing_goal_generation == first_goal_generation
    assert bridge.nav_status == "active"
    assert bridge._goal_handle is replacement
    outcome = NS(status=sim_module.GoalStatus.STATUS_CANCELED, result=None)
    bridge._goal_result(_ImmediateFuture(outcome), first_goal_generation, handle)
    assert bridge.nav_status == "active"
    assert bridge._goal_handle is replacement
    explorer.last_link = clock[0]
    explorer.tick()
    assert explorer.active and explorer.status == "exploring"
