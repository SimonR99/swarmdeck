"""Exploration command/state races without ROS or robot motion."""

import asyncio
from concurrent.futures import Future
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

import pytest

from adapters.exploration import MggExploration, planner_path
from adapters.session import dispatch_command, _stop_on_disconnect


def rig():
    bridge = Mock()
    # Explicit callables avoid Python 3.14's executor shutdown waiting forever
    # on a dynamically-created Mock child after dispatch_command offloads it.
    bridge.navigate_to = Mock()
    bridge.follow_path = Mock()
    bridge.id = "r"
    bridge.cfg = {"link_timeout_s": 5}
    bridge._goal_generation = 0
    bridge.nav_status = "idle"
    bridge.node.get_clock().now().nanoseconds = 1000000000

    def cancel_goal():
        bridge._goal_generation += 1
        bridge.nav_status = "cancelled"
        return bridge._goal_generation

    def follow_path(_plan, *, expected_generation=None):
        if (
            expected_generation is not None
            and expected_generation != bridge._goal_generation
        ):
            return None
        if expected_generation is None:
            bridge._goal_generation += 1
        bridge.nav_status = "active"
        return bridge._goal_generation

    bridge.cancel_goal.side_effect = cancel_goal
    bridge.follow_path.side_effect = follow_path
    explorer = MggExploration.__new__(MggExploration)
    explorer.bridge = bridge
    explorer.active = False
    explorer.generation = 0
    explorer.started_ns = 0
    explorer.last_path_revision_ns = 0
    explorer.pending_plan = None
    explorer.executing_plan = None
    explorer.executing_goal_generation = None
    explorer.replan_requested_generation = -1
    explorer.replan_requested_recovery = False
    explorer.controller_replan_generation = -1
    explorer.controller_goal_generation = None
    explorer.controller_replan_attempts = 0
    explorer.controller_replan_deadline = 0.0
    explorer.controller_replan_due = 0.0
    explorer.awaiting_replan_path = False
    explorer.controller_replan_max_attempts = 3
    explorer.controller_replan_deadline_s = 15.0
    explorer.controller_replan_backoff_s = 0.0
    explorer.pending = None
    explorer.pending_kind = None
    explorer.pending_recovery = False
    explorer.pending_stop = None
    explorer.stop_deadline = 0
    explorer.deadline = 0
    explorer.last_link = time.monotonic()
    explorer.frame = "map"
    explorer.planar_tolerance_m = 0.05
    explorer.max_inclination_rad = math.radians(30.0)
    explorer.coordinator = None
    explorer.request_type = lambda: None
    explorer.start_client = Mock()
    explorer.replan_client = Mock()
    explorer.stop_client = Mock()
    explorer.start_client.service_is_ready.return_value = True
    explorer.replan_client.service_is_ready.return_value = True
    explorer.stop_client.service_is_ready.return_value = True
    explorer.start_client.call_async.return_value = Future()
    explorer.replan_client.call_async.return_value = Future()
    explorer.stop_client.call_async.return_value = Future()
    bridge.exploration = explorer
    return bridge, explorer


def path(frame="map", stamp=2, empty=False, points=None):
    points = points or [(1.0, 2.0, 0.0)]
    poses = [
        NS(
            header=NS(frame_id=frame),
            pose=NS(
                position=NS(x=x, y=y, z=z),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        for x, y, z in points
    ]
    return NS(
        header=NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0)),
        poses=[] if empty else poses,
    )


def test_start_stop_and_late_paths():
    bridge, explorer = rig()
    explorer.start()
    explorer.start()  # Idempotent while starting.
    explorer.start_client.call_async.assert_called_once()
    explorer.on_path(path())
    bridge.follow_path.assert_called_once()
    plan = bridge.follow_path.call_args.args[0]
    assert [(pose.x, pose.y, pose.z) for pose in plan.poses] == [(1.0, 2.0, 0.0)]
    explorer.stop()
    explorer.on_path(path(stamp=3))
    explorer.start_client.call_async.return_value.set_result(NS(success=True))
    assert not explorer.active
    bridge.follow_path.assert_called_once()
    bridge.drive.assert_called_with(0.0, 0.0)
    explorer.stop_client.call_async.assert_called_once()


def test_latched_wrong_frame_empty_and_lost_link():
    bridge, explorer = rig()
    explorer.on_path(path())
    bridge.follow_path.assert_not_called()
    explorer.start()
    explorer.on_path(path(stamp=0))
    bridge.follow_path.assert_not_called()
    explorer.on_path(path(frame="other_map"))
    assert not explorer.active
    for failure in ("empty", "link", "timeout"):
        bridge, explorer = rig()
        explorer.start()
        if failure == "empty":
            explorer.on_path(path(empty=True))
        elif failure == "link":
            explorer.last_link = time.monotonic() - 10
            explorer.tick()
        else:
            explorer.deadline = 0
            explorer.tick()
        assert not explorer.active
        bridge.cancel_goal.assert_called()


def test_stop_all_dispatch_disables_exploration_first():
    bridge, explorer = rig()
    explorer.start()

    async def run():
        await dispatch_command(bridge, {"type": "stop"}, asyncio.get_running_loop())

    asyncio.run(run())
    assert not explorer.active
    bridge.stop.assert_called_once()


def test_failed_start_and_disconnect_do_not_leave_autonomy_active():
    bridge, explorer = rig()
    explorer.start()
    explorer.pending.set_result(NS(success=False, message="no path"))
    assert not explorer.active
    bridge, explorer = rig()
    explorer.start()
    _stop_on_disconnect(bridge)
    assert not explorer.active


def test_stop_wins_during_navigation_dispatch():
    bridge, explorer = rig()
    explorer.start()
    bridge.follow_path.side_effect = lambda goal: explorer.stop()
    explorer.on_path(path())
    assert not explorer.active
    bridge.drive.assert_called_with(0.0, 0.0)


def test_start_waits_for_previous_requests_and_service_availability():
    bridge, explorer = rig()
    explorer.stop_client.service_is_ready.return_value = False
    explorer.start()
    assert not explorer.active
    explorer.stop_client.service_is_ready.return_value = True
    explorer.start()
    explorer.stop()
    explorer.start()
    assert not explorer.active
    explorer.start_client.call_async.assert_called_once()


def test_stop_service_failure_still_cancels_motion():
    bridge, explorer = rig()
    explorer.start()
    explorer.stop_client.call_async.side_effect = RuntimeError("DDS unavailable")
    explorer.stop()
    assert not explorer.active
    bridge.cancel_goal.assert_called()
    bridge.drive.assert_called_with(0.0, 0.0)


def test_manual_goal_preempts_exploration():
    bridge, explorer = rig()
    explorer.start()
    goal = {"x": 3, "y": 4}
    received = []
    bridge.navigate_to = received.append

    async def run():
        class InlineLoop:
            def run_in_executor(self, _executor, fn, *args):
                future = asyncio.get_running_loop().create_future()
                try:
                    future.set_result(fn(*args))
                except Exception as exc:
                    future.set_exception(exc)
                return future

        await dispatch_command(
            bridge, {"type": "navigate_to", "goal": goal}, InlineLoop()
        )

    asyncio.run(run())
    assert not explorer.active
    assert received == [goal]


def test_orphaned_requests_expire_after_planner_restart():
    bridge, explorer = rig()
    explorer.start()
    abandoned_start = explorer.pending
    explorer.stop()
    abandoned_stop = explorer.pending_stop
    explorer.deadline = explorer.stop_deadline = 0
    explorer.tick()
    assert abandoned_start.cancelled() and abandoned_stop.cancelled()
    assert explorer.pending is None and explorer.pending_stop is None
    explorer.on_path(path())
    bridge.follow_path.assert_not_called()
    explorer.start_client.call_async.return_value = Future()
    explorer.start()
    assert explorer.active


def test_terminal_status_survives_either_dds_delivery_order():
    import json

    for status_first in (True, False):
        bridge, explorer = rig()
        explorer.start()
        message = NS(data=json.dumps({"state": "blocked", "stamp_ns": 2000000000}))
        calls = [
            lambda: explorer.on_status(message),
            lambda: explorer.on_path(path(empty=True)),
        ]
        for call in calls if status_first else reversed(calls):
            call()
        assert not explorer.active
        assert explorer.status == "blocked"
        explorer.stop_client.call_async.assert_not_called()
        bridge.cancel_goal.assert_called()


def test_manual_stop_and_old_status_cannot_relabel_a_session():
    import json

    _, explorer = rig()
    explorer.start()
    explorer.on_status(NS(data=json.dumps({"state": "complete", "stamp_ns": 1})))
    assert explorer.active
    explorer.stop()
    explorer.on_status(
        NS(data=json.dumps({"state": "complete", "stamp_ns": 2000000000}))
    )
    assert explorer.status == "stopped"


def test_full_path_and_revision_are_preserved_without_ros():
    source = path(points=[(0.0, 0.0, 0.42), (1.0, 0.5, 0.43), (2.0, 1.0, 0.41)])
    plan = planner_path(source, "map", 0.05)
    assert plan.revision_ns == 2_000_000_000
    assert [(p.x, p.y, p.z) for p in plan.poses] == [
        (0.0, 0.0, 0.42),
        (1.0, 0.5, 0.43),
        (2.0, 1.0, 0.41),
    ]

    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(source)
    explorer.on_path(source)  # A latched duplicate must not reissue motion.
    explorer.on_path(path(stamp=1))  # Neither may an older revision.
    bridge.follow_path.assert_called_once()


def test_nonplanar_paths_are_rejected_instead_of_flattened():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path(points=[(0.0, 0.0, 0.0), (0.1, 0.0, 0.2)]))
    assert not explorer.active
    bridge.follow_path.assert_not_called()
    bridge.cancel_goal.assert_called()

    tilted = path()
    tilted.poses[0].pose.orientation.x = math.sin(0.1)
    tilted.poses[0].pose.orientation.w = math.cos(0.1)
    with pytest.raises(ValueError, match="not planar"):
        planner_path(tilted, "map", 0.05)


def test_ground_ramp_uses_segment_limits_not_total_elevation():
    source = path(
        points=[
            (0.0, 0.0, 0.40),
            (0.5, 0.0, 0.45),
            (1.0, 0.0, 0.50),
            (1.5, 0.0, 0.55),
        ]
    )
    plan = planner_path(source, "map", 0.10, math.radians(30.0))
    assert plan.poses[-1].z - plan.poses[0].z == pytest.approx(0.15)


def test_path_rejects_upside_down_and_conflicting_pose_frames():
    upside_down = path()
    upside_down.poses[0].pose.orientation.x = 1.0
    upside_down.poses[0].pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="not planar"):
        planner_path(upside_down, "map", 0.05)

    conflicting = path()
    conflicting.poses[0].header.frame_id = "old_map"
    with pytest.raises(ValueError, match="pose 0 frame"):
        planner_path(conflicting, "map", 0.05)


def test_path_normalizes_quaternions_and_rejects_invalid_stamp():
    scaled = path()
    scaled.poses[0].pose.orientation.z = 1.0
    scaled.poses[0].pose.orientation.w = 1.0
    pose = planner_path(scaled, "map", 0.05).poses[0]
    assert math.hypot(pose.qz, pose.qw) == pytest.approx(1.0)

    scaled.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(ValueError, match="timestamp"):
        planner_path(scaled, "map", 0.05)


class Coordinator:
    def __init__(self, *decisions):
        self.decisions = list(decisions)
        self.releases = []
        self.ticks = 0

    def reserve(self, _plan, _generation):
        return self.decisions.pop(0) if self.decisions else "granted"

    def release(self, generation):
        self.releases.append(generation)

    def tick(self):
        self.ticks += 1


def test_peer_reservation_holds_then_dispatches_without_blocking():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("pending", "granted")
    explorer.start()
    explorer.on_path(path())
    bridge.follow_path.assert_not_called()
    assert explorer.status == "waiting"

    explorer.tick()
    bridge.follow_path.assert_called_once()
    assert explorer.status == "exploring"
    assert explorer.executing_plan is not None


def test_lost_peer_reservation_cancels_active_path_and_waits():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("granted", "pending")
    explorer.start()
    explorer.on_path(path())
    bridge.cancel_goal.reset_mock()

    explorer.tick()
    bridge.cancel_goal.assert_called_once()
    assert explorer.executing_plan is None
    assert explorer.pending_plan is not None
    assert explorer.status == "waiting"


def test_rejected_peer_goal_requests_a_new_mgg_plan_after_start_finishes():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("rejected")
    explorer.start()
    first_request = explorer.pending
    explorer.on_path(path())
    assert explorer.replan_requested_generation == explorer.generation
    bridge.follow_path.assert_not_called()

    next_request = Future()
    explorer.replan_client.call_async.return_value = next_request
    first_request.set_result(NS(success=True))
    assert explorer.pending is next_request
    explorer.replan_client.call_async.assert_called_once()


def test_pending_peer_goal_rejected_on_tick_requests_replan():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("pending", "rejected")
    explorer.start()
    explorer.on_path(path())
    first_request = explorer.pending
    assert explorer.pending_plan is not None

    explorer.tick()
    assert explorer.pending_plan is None
    assert explorer.replan_requested_generation == explorer.generation
    bridge.follow_path.assert_not_called()

    next_request = Future()
    explorer.replan_client.call_async.return_value = next_request
    first_request.set_result(NS(success=True))
    assert explorer.pending is next_request


def test_peer_complete_is_only_local_exhaustion():
    import json

    _, explorer = rig()
    explorer.coordinator = Coordinator("granted")
    explorer.start()
    explorer.on_status(
        NS(data=json.dumps({"state": "complete", "stamp_ns": 2_000_000_000}))
    )
    assert not explorer.active
    assert explorer.status == "locally_exhausted"


def test_manual_stop_releases_peer_reservation_before_motion_cancel():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("granted")
    explorer.start()
    generation = explorer.generation
    explorer.on_path(path())
    explorer.stop()
    assert explorer.coordinator.releases[-1] > generation
    assert explorer.pending_plan is None and explorer.executing_plan is None


def _start_with_path(explorer, *, stamp=2):
    explorer.start()
    explorer.pending.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=stamp))


def test_owned_controller_failure_requests_bounded_replan_and_recovers():
    bridge, explorer = rig()
    replacement_request = Future()
    explorer.replan_client.call_async.return_value = replacement_request
    _start_with_path(explorer)
    first_goal_generation = explorer.executing_goal_generation

    bridge.nav_status = "failed"
    explorer.tick()

    assert explorer.active
    assert explorer.status == "waiting"
    assert explorer.executing_plan is None
    assert explorer.controller_replan_attempts == 1
    explorer.replan_client.call_async.assert_called_once()

    replacement_request.set_result(NS(success=True, message=""))
    assert explorer.awaiting_replan_path
    explorer.on_path(path(stamp=3, points=[(0.5, 0.0, 0.0), (1.5, 0.0, 0.0)]))
    assert explorer.executing_goal_generation == first_goal_generation
    assert not explorer.awaiting_replan_path
    assert bridge.follow_path.call_args.kwargs == {
        "expected_generation": first_goal_generation
    }

    bridge.nav_status = "succeeded"
    explorer.tick()
    assert explorer.active
    assert explorer.executing_plan is None
    assert explorer.controller_replan_generation == -1
    assert explorer.controller_replan_attempts == 0


def test_repeated_controller_failures_exhaust_replan_budget():
    bridge, explorer = rig()
    explorer.controller_replan_max_attempts = 2
    requests = [Future(), Future()]
    explorer.replan_client.call_async.side_effect = requests
    _start_with_path(explorer)

    for attempt, request in enumerate(requests, start=1):
        bridge.nav_status = "failed"
        explorer.tick()
        assert explorer.controller_replan_attempts == attempt
        request.set_result(NS(success=True, message=""))
        explorer.on_path(path(stamp=2 + attempt))

    bridge.nav_status = "failed"
    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"
    assert explorer.replan_client.call_async.call_count == 2


def test_controller_replan_times_out_if_no_replacement_path_arrives():
    bridge, explorer = rig()
    request = Future()
    explorer.replan_client.call_async.return_value = request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()
    request.set_result(NS(success=True, message=""))
    assert explorer.awaiting_replan_path

    explorer.controller_replan_deadline = time.monotonic() - 1.0
    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"


def test_replacement_goal_generation_retires_exploration_without_canceling_it():
    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.cancel_goal.reset_mock()
    bridge.drive.reset_mock()

    bridge._goal_generation += 1
    bridge.nav_status = "failed"
    explorer.tick()

    assert not explorer.active
    assert explorer.status == "stopped"
    explorer.replan_client.call_async.assert_not_called()
    bridge.cancel_goal.assert_not_called()
    bridge.drive.assert_not_called()


def test_replan_rejection_reports_blocked_instead_of_stopped():
    bridge, explorer = rig()
    request = Future()
    explorer.replan_client.call_async.return_value = request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()

    request.set_result(NS(success=False, message="no alternate frontier"))

    assert not explorer.active
    assert explorer.status == "blocked"


def test_replacement_path_wins_over_late_replan_rejection():
    bridge, explorer = rig()
    request = Future()
    explorer.replan_client.call_async.return_value = request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()

    explorer.on_path(path(stamp=3))
    replacement_generation = explorer.executing_goal_generation
    request.set_result(NS(success=False, message="late rejection"))

    assert explorer.active
    assert explorer.status == "exploring"
    assert explorer.executing_goal_generation == replacement_generation
    assert explorer.controller_replan_attempts == 1


def test_manual_goal_while_waiting_for_replan_path_wins_without_cancellation():
    bridge, explorer = rig()
    request = Future()
    explorer.replan_client.call_async.return_value = request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()
    request.set_result(NS(success=True, message=""))
    assert explorer.awaiting_replan_path
    bridge.cancel_goal.reset_mock()
    bridge.drive.reset_mock()
    prior_paths = bridge.follow_path.call_count

    bridge._goal_generation += 1
    bridge.nav_status = "active"
    explorer.on_path(path(stamp=3))

    assert not explorer.active
    assert bridge.follow_path.call_count == prior_paths
    bridge.cancel_goal.assert_not_called()
    bridge.drive.assert_not_called()


def test_manual_goal_wins_over_stale_replan_service_response():
    bridge, explorer = rig()
    request = Future()
    explorer.replan_client.call_async.return_value = request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()
    bridge.cancel_goal.reset_mock()
    bridge.drive.reset_mock()

    bridge._goal_generation += 1
    bridge.nav_status = "active"
    request.set_result(NS(success=True, message=""))

    assert not explorer.active
    assert not explorer.awaiting_replan_path
    bridge.cancel_goal.assert_not_called()
    bridge.drive.assert_not_called()
