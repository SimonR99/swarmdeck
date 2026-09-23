"""Exploration command/state races without ROS or robot motion."""

import asyncio
from concurrent.futures import Future
import json
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
from threading import RLock

import pytest

from adapters.exploration import (
    MggExploration,
    PlanTiming,
    is_physical_no_progress_failure,
    planner_path,
)
from adapters.session import dispatch_command, _stop_on_disconnect


def test_exploration_uses_configured_stable_planning_frame(monkeypatch):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    monkeypatch.setitem(
        sys.modules, "std_srvs.srv", NS(Trigger=NS(Request=lambda: None))
    )
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", NS(Path=object))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=object))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        NS(
            QoSProfile=lambda **kwargs: NS(**kwargs),
            DurabilityPolicy=NS(TRANSIENT_LOCAL="transient_local"),
        ),
    )
    node = Mock()
    node.create_client.return_value = Mock()
    bridge = NS(id="r0", navigation_frame="r0/navigation_frame", node=node)

    explorer = MggExploration(bridge, {})

    assert explorer.frame == "r0/odom"
    with pytest.raises(ValueError, match="configured MGG planning frame"):
        MggExploration(bridge, {"frame": "r0/navigation_frame"})


def rig():
    bridge = Mock()
    bridge._mapping_authority = None
    bridge._goal_lock = RLock()
    bridge.objective_planner = None
    # Explicit callables avoid Python 3.14's executor shutdown waiting forever
    # on a dynamically-created Mock child after dispatch_command offloads it.
    bridge.plan_objective = Mock()
    bridge.follow_path = Mock()
    bridge.id = "r"
    bridge.cfg = {"link_timeout_s": 5}
    bridge._goal_generation = 0
    bridge.nav_status = "idle"
    bridge._nav_failure_reason = "Failed to make progress; error_code=4"
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
    bridge.cancel_goal_if_current = Mock(
        side_effect=lambda expected_generation: (
            bridge.cancel_goal()
            if expected_generation == bridge._goal_generation
            else None
        )
    )
    bridge.follow_path.side_effect = follow_path
    explorer = MggExploration.__new__(MggExploration)
    explorer.bridge = bridge
    explorer.active = False
    explorer.generation = 0
    explorer.started_ns = 0
    explorer.last_path_revision_ns = 0
    explorer.pending_plan = None
    explorer.pending_authority_replan = None
    explorer.executing_plan = None
    explorer.executing_goal_generation = None
    explorer.completed_goal_generation = None
    explorer.replan_requested_generation = -1
    explorer.replan_requested_recovery = False
    explorer.controller_replan_generation = -1
    explorer.controller_goal_generation = None
    explorer.controller_replan_attempts = 0
    explorer.controller_replan_deadline = 0.0
    explorer.controller_replan_due = 0.0
    explorer.awaiting_replan_path = False
    explorer.controller_replan_max_attempts = 2
    explorer.controller_replan_deadline_s = 15.0
    explorer.controller_replan_backoff_s = 0.0
    explorer.pending = None
    explorer.pending_kind = None
    explorer.pending_recovery = False
    explorer.pending_stop = None
    explorer.stop_deadline = 0
    explorer.deadline = 0
    explorer.last_link = time.monotonic()
    explorer.timing = PlanTiming()
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


def test_unavailable_planner_is_blocked_with_a_reason():
    bridge, explorer = rig()
    explorer.start_client.service_is_ready.return_value = False
    explorer.start()
    assert explorer.status == "blocked"
    assert not explorer.active
    assert explorer.reason == "MGG start/stop services are unavailable"
    bridge.follow_path.assert_not_called()


def test_waiting_reason_is_retired_when_the_controller_starts():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_status(
        NS(
            data=json.dumps(
                {
                    "stamp_ns": 2_000_000_000,
                    "state": "waiting",
                    "reason": "No traversable frontier; retrying",
                }
            )
        )
    )
    assert explorer.status == "waiting"
    assert explorer.reason == "No traversable frontier; retrying"
    explorer.on_path(path())
    assert explorer.status == "exploring"
    assert explorer.reason is None
    bridge.follow_path.assert_called_once()


def test_native_waiting_reason_survives_successful_trigger_reply():
    _, explorer = rig()
    explorer.start()
    request = explorer.pending
    explorer.on_status(
        NS(
            data=json.dumps(
                {
                    "stamp_ns": 2_000_000_000,
                    "state": "waiting",
                    "reason": "planner returned no path; retrying automatically in 1.0 s",
                }
            )
        )
    )

    request.set_result(NS(success=True, message="exploration active"))

    assert explorer.active
    assert explorer.status == "waiting"
    assert explorer.reason == (
        "planner returned no path; retrying automatically in 1.0 s"
    )


def test_old_planner_status_cannot_replace_current_waiting_reason():
    _, explorer = rig()
    explorer.start()
    explorer.on_status(
        NS(
            data=json.dumps(
                {
                    "stamp_ns": 2_000_000_000,
                    "state": "waiting",
                }
            )
        )
    )
    reason = explorer.reason
    explorer.on_status(
        NS(
            data=json.dumps(
                {
                    "stamp_ns": 1,
                    "state": "waiting",
                    "reason": "old session",
                }
            )
        )
    )
    assert reason == explorer.reason


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


@pytest.mark.parametrize("cap", [0.10, 0.30])
def test_native_step_cap_survives_voxel_height_roundoff(cap):
    source = path(points=[(0, 0, 0.2), (0.01, 0, 0.2 + cap + 1e-12)])
    assert len(planner_path(source, "map", cap).poses) == 2
    source.poses[1].pose.position.z = 0.2 + cap + 1e-4
    with pytest.raises(ValueError, match="cannot traverse"):
        planner_path(source, "map", cap)


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


def test_newer_path_cannot_replace_active_controller_goal():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path(stamp=2, points=[(1.0, 0.0, 0.0)]))

    explorer.on_path(path(stamp=3, points=[(2.0, 0.0, 0.0)]))

    bridge.follow_path.assert_called_once()
    assert explorer.executing_plan[0].revision_ns == 2_000_000_000
    assert explorer.last_path_revision_ns == 3_000_000_000
    bridge.node.get_logger().warning.assert_called_with(
        "[r] exploration: ignoring replacement path while controller goal is active"
    )

    bridge.nav_status = "succeeded"
    explorer.tick()
    explorer.on_path(path(stamp=3, points=[(2.0, 0.0, 0.0)]))
    bridge.follow_path.assert_called_once()


def test_ignored_replacement_does_not_prevent_operator_stop():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path(stamp=2))
    explorer.on_path(path(stamp=3))
    bridge.cancel_goal.reset_mock()

    explorer.stop()

    bridge.cancel_goal.assert_called_once()
    assert not explorer.active
    assert explorer.executing_plan is None


def test_ignored_replacement_does_not_prevent_authority_revocation():
    bridge, explorer = rig()
    explorer.coordinator = Coordinator("granted", "pending")
    explorer.start()
    explorer.on_path(path(stamp=2))
    explorer.on_path(path(stamp=3))
    bridge.cancel_goal.reset_mock()

    explorer.tick()

    bridge.cancel_goal.assert_called_once()
    assert explorer.executing_plan is None
    assert explorer.pending_plan is not None


@pytest.mark.parametrize("state", ["complete", "blocked"])
def test_planner_terminal_status_cannot_complete_active_controller_goal(state):
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path(stamp=2))

    explorer.on_status(NS(data=json.dumps({"state": state, "stamp_ns": 3_000_000_000})))

    assert explorer.active
    assert explorer.executing_plan is not None
    bridge.cancel_goal.assert_called_once()  # Session start only.
    explorer.stop_client.call_async.assert_not_called()


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
    bridge.plan_objective = lambda _objective, goal: received.append(goal)

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
            bridge,
            {"type": "plan_objective", "objective": "navigate", "goal": goal},
            InlineLoop(),
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
    bridge.cancel_goal_if_current.assert_called_once()
    bridge.cancel_goal.assert_called_once()
    assert explorer.executing_plan is None
    assert explorer.pending_plan is not None
    assert explorer.status == "waiting"


def test_authority_return_replans_cancelled_path_before_executing():
    bridge, explorer = rig()
    fresh_request = Future()
    explorer.replan_client.call_async.return_value = fresh_request
    _start_with_path(explorer, stamp=2)
    explorer.coordinator = Coordinator("pending", "granted")

    explorer.tick()
    assert explorer.pending_authority_replan is explorer.pending_plan
    assert bridge.follow_path.call_count == 1

    explorer.tick()
    assert explorer.pending_plan is None
    assert explorer.pending_authority_replan is None
    assert explorer.pending is fresh_request
    assert bridge.follow_path.call_count == 1

    fresh_request.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))
    assert explorer.executing_plan[0].revision_ns == 3_000_000_000
    assert bridge.follow_path.call_count == 2


@pytest.mark.parametrize("fresh_pending", [False, True])
def test_new_path_supersedes_cancelled_authority_token_in_same_session(
    fresh_pending,
):
    bridge, explorer = rig()
    _start_with_path(explorer, stamp=2)
    explorer.coordinator = Coordinator(
        *("pending", "pending", "granted") if fresh_pending else ("pending", "granted")
    )

    explorer.tick()
    old_token = explorer.pending_authority_replan
    assert old_token is explorer.pending_plan

    explorer.on_path(path(stamp=3))

    assert explorer.pending_authority_replan is None
    if fresh_pending:
        assert explorer.pending_plan is not None
        assert explorer.pending_plan[0].revision_ns == 3_000_000_000
        assert bridge.follow_path.call_count == 1
        explorer.tick()
    assert explorer.pending_plan is None
    assert explorer.executing_plan[0].revision_ns == 3_000_000_000
    assert bridge.follow_path.call_count == 2
    explorer.replan_client.call_async.assert_not_called()


def test_recovery_authority_loss_gets_fresh_wait_without_new_attempt():
    bridge, explorer = rig()
    replacement_request, fresh_request = Future(), Future()
    explorer.replan_client.call_async.side_effect = [
        replacement_request,
        fresh_request,
    ]
    _start_with_path(explorer)

    bridge.nav_status = "failed"
    explorer.tick()
    replacement_request.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))
    assert explorer.controller_replan_attempts == 1

    # The replacement executes longer than the deadline that produced it.
    # A transient authority loss must wait from this new event, rather than
    # consuming another physical retry or timing out on the next tick.
    explorer.controller_replan_deadline = time.monotonic() - 100.0
    explorer.coordinator = Coordinator("pending", "granted")
    explorer.tick()

    assert explorer.active
    assert explorer.status == "waiting"
    assert explorer.pending_plan is not None
    assert explorer.controller_replan_attempts == 1
    assert explorer.controller_replan_deadline > time.monotonic()
    cancelled_generation = bridge._goal_generation
    assert explorer.controller_goal_generation == cancelled_generation

    explorer.tick()

    assert explorer.active
    assert explorer.status == "waiting"
    assert explorer.executing_plan is None
    assert explorer.pending is fresh_request
    assert explorer.controller_replan_attempts == 1
    assert bridge.follow_path.call_count == 2

    fresh_request.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=4))

    assert explorer.status == "exploring"
    assert explorer.executing_plan is not None
    assert explorer.executing_goal_generation == cancelled_generation
    assert bridge.follow_path.call_count == 3
    assert explorer.executing_plan[0].revision_ns == 4_000_000_000


def test_new_operator_goal_wins_authority_loss_without_cancellation():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path())
    new_owner = bridge._goal_generation + 1

    class RacingCoordinator(Coordinator):
        def reserve(self, _plan, _generation):
            bridge._goal_generation = new_owner
            return "pending"

    explorer.coordinator = RacingCoordinator()
    bridge.cancel_goal.reset_mock()

    explorer.tick()

    assert not explorer.active
    assert explorer.status == "stopped"
    assert bridge._goal_generation == new_owner
    bridge.cancel_goal_if_current.assert_called_once()
    bridge.cancel_goal.assert_not_called()


def test_manual_goal_while_authority_pending_wins_late_grant():
    bridge, explorer = rig()
    explorer.start()
    explorer.on_path(path())
    explorer.coordinator = Coordinator("pending", "granted")

    explorer.tick()
    cancelled_generation = bridge._goal_generation
    assert explorer.pending_plan is not None
    assert explorer.completed_goal_generation == cancelled_generation
    bridge.cancel_goal.reset_mock()

    manual_generation = cancelled_generation + 1
    bridge._goal_generation = manual_generation
    bridge.nav_status = "active"
    explorer.tick()

    assert not explorer.active
    assert explorer.status == "stopped"
    assert bridge._goal_generation == manual_generation
    bridge.cancel_goal.assert_not_called()


def test_recovery_authority_rejection_keeps_attempt_and_fresh_bound():
    bridge, explorer = rig()
    first_request, authority_request = Future(), Future()
    explorer.replan_client.call_async.side_effect = [
        first_request,
        authority_request,
    ]
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()
    first_request.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))
    explorer.controller_replan_deadline = time.monotonic() - 100.0
    explorer.coordinator = Coordinator("pending", "rejected")

    explorer.tick()
    fresh_deadline = explorer.controller_replan_deadline
    explorer.tick()

    assert explorer.active
    assert explorer.controller_replan_attempts == 1
    assert explorer.pending is authority_request
    assert explorer.pending_recovery
    assert explorer.deadline <= fresh_deadline


def test_recovery_authority_wait_times_out_on_its_fresh_deadline():
    bridge, explorer = rig()
    replacement_request = Future()
    explorer.replan_client.call_async.return_value = replacement_request
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    explorer.tick()
    replacement_request.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))
    explorer.coordinator = Coordinator("pending", "pending")
    explorer.tick()
    explorer.controller_replan_deadline = time.monotonic() - 1.0

    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"
    # Stop clears recovery bookkeeping; no second physical attempt was made.
    assert explorer.replan_client.call_async.call_count == 1


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
    assert bridge.follow_path.call_count == 3


@pytest.mark.parametrize(
    "reason",
    [None, "", "NO_VALID_CONTROL", "controller transport failed", "progress pending"],
)
def test_generic_controller_failure_does_not_enter_movement_retry(reason):
    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.nav_status = "failed"
    bridge._nav_failure_reason = reason

    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"
    assert explorer.controller_replan_attempts == 0
    explorer.replan_client.call_async.assert_not_called()
    assert bridge.nav_status == "failed"
    assert bridge._nav_failure_reason == reason
    assert "without no-progress evidence" in (
        bridge.node.get_logger().warning.call_args.args[0]
    )


def test_no_progress_classifier_matches_adapter_diagnostic_only():
    assert is_physical_no_progress_failure("Failed to make progress; error_code=4")
    assert not is_physical_no_progress_failure("NO_VALID_CONTROL; error_code=7")


def test_controller_execution_time_does_not_consume_next_replan_deadline():
    bridge, explorer = rig()
    requests = [Future(), Future()]
    explorer.replan_client.call_async.side_effect = requests
    _start_with_path(explorer)

    bridge.nav_status = "failed"
    explorer.tick()
    requests[0].set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))

    explorer.controller_replan_deadline = time.monotonic() - 100.0
    bridge.nav_status = "failed"
    explorer.tick()

    assert explorer.active
    assert explorer.controller_replan_attempts == 2
    assert explorer.controller_replan_deadline > time.monotonic()


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


def test_controller_replan_deadline_never_cancels_a_new_owner():
    bridge, explorer = rig()
    explorer.active = True
    explorer.controller_replan_generation = explorer.generation
    explorer.controller_goal_generation = bridge._goal_generation
    explorer.controller_replan_deadline = time.monotonic() - 1.0
    bridge._goal_generation += 1
    bridge.nav_status = "active"
    bridge.cancel_goal.reset_mock()

    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"
    bridge.cancel_goal.assert_not_called()


def test_pending_replan_timeout_without_execution_does_not_cancel_motion():
    bridge, explorer = rig()
    explorer.active = True
    request = Future()
    explorer.pending = request
    explorer.pending_kind = "replan"
    explorer.pending_recovery = True
    explorer.deadline = time.monotonic() - 1.0
    bridge.nav_status = "active"
    bridge.cancel_goal.reset_mock()

    explorer.tick()

    assert not explorer.active
    assert explorer.status == "blocked"
    bridge.cancel_goal.assert_not_called()
    explorer.replan_client.remove_pending_request.assert_called_once_with(request)


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


def test_only_controller_success_requests_the_next_exploration_path():
    bridge, explorer = rig()
    _start_with_path(explorer)
    for _ in range(5):
        explorer.tick()
    explorer.replan_client.call_async.assert_not_called()
    generation = bridge._goal_generation
    bridge.nav_status = "succeeded"
    explorer.tick()
    explorer.tick()
    explorer.replan_client.call_async.assert_called_once()
    assert explorer.status == "waiting"
    assert explorer.completed_goal_generation == generation
    explorer.pending.set_result(NS(success=True, message=""))
    explorer.on_path(path(stamp=3))
    assert explorer.executing_plan is not None
    assert explorer.completed_goal_generation is None
    assert bridge.follow_path.call_args.kwargs == {"expected_generation": generation}


@pytest.mark.parametrize("event", ["path", "empty_path", "status", "reply"])
def test_manual_command_wins_after_arrival_while_next_plan_is_pending(event):
    import json

    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.nav_status = "succeeded"
    explorer.tick()
    bridge._goal_generation += 1
    bridge.nav_status = "active"
    bridge.cancel_goal.reset_mock()
    bridge.drive.reset_mock()
    calls = bridge.follow_path.call_count
    if event in ("path", "empty_path"):
        explorer.on_path(path(stamp=3, empty=event == "empty_path"))
    elif event == "status":
        explorer.on_status(
            NS(data=json.dumps({"state": "blocked", "stamp_ns": 3000000000}))
        )
    else:
        explorer.pending.set_result(NS(success=True, message=""))
    assert not explorer.active
    assert bridge.follow_path.call_count == calls
    bridge.cancel_goal.assert_not_called()
    bridge.drive.assert_not_called()


def test_next_path_after_arrival_wins_over_late_service_rejection():
    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.nav_status = "succeeded"
    explorer.tick()
    request = explorer.pending
    explorer.on_path(path(stamp=3))
    request.set_result(NS(success=False, message="late reply"))
    assert explorer.active and explorer.executing_plan is not None
    assert bridge.nav_status == "active"


@pytest.mark.parametrize("decision", ["pending", "rejected"])
def test_peer_decision_on_next_path_wins_over_late_service_rejection(decision):
    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.nav_status = "succeeded"
    explorer.tick()
    request = explorer.pending
    explorer.replan_client.call_async.return_value = Future()
    explorer.coordinator = Coordinator(decision)
    bridge.cancel_goal.reset_mock()
    explorer.on_path(path(stamp=3))
    request.set_result(NS(success=False, message="late reply"))

    assert explorer.active and explorer.status == "waiting"
    bridge.cancel_goal.assert_not_called()
    if decision == "pending":
        assert explorer.pending_plan is not None
        explorer.replan_client.call_async.assert_called_once()
    else:
        assert explorer.pending is not request
        assert explorer.replan_client.call_async.call_count == 2


def test_peer_rejection_after_arrival_preserves_waiting_goal_ownership():
    bridge, explorer = rig()
    _start_with_path(explorer)
    bridge.nav_status = "succeeded"
    explorer.tick()
    generation = bridge._goal_generation
    explorer.pending.set_result(NS(success=True, message=""))
    explorer.replan_client.call_async.return_value = Future()
    explorer.coordinator = Coordinator("rejected")
    bridge.cancel_goal.reset_mock()
    explorer.on_path(path(stamp=3))
    assert explorer.active and explorer.status == "waiting"
    assert bridge._goal_generation == generation
    assert explorer.completed_goal_generation == generation
    bridge.cancel_goal.assert_not_called()


def test_plan_timing_reports_each_stage_once_per_path():
    now = [10.0]
    timing = PlanTiming(clock=lambda: now[0])
    timing.controller_finished()
    now[0] = 10.1
    timing.requested()
    now[0] = 10.3
    assert timing.received(7, 0.004) is None
    now[0] = 10.8
    timing.mark("granted")
    now[0] = 10.81
    timing.sent({"x": 1.0, "y": 1.0})
    now[0] = 10.82
    timing.mark("accepted")
    now[0] = 11.2
    assert timing.observe({"x": 1.05, "y": 1.05}) is None  # 0.07 m: parked
    now[0] = 11.4
    line = timing.observe({"x": 1.1, "y": 1.0})
    assert line == (
        "exploration timing: path 7 at t=10.300; age 0.004 s; "
        "terminal->replan 0.100 s; replan->path 0.200 s; ->granted 0.500 s; "
        "->sent 0.510 s; ->accepted 0.520 s; ->moved 1.100 s; moving"
    )
    assert timing.observe({"x": 5.0, "y": 5.0}) is None
    assert timing.finish("controller succeeded") is None
    assert timing.since_grant() == pytest.approx(0.6)


def test_plan_timing_lines_are_rate_limited_and_count_suppressions():
    now = [0.0]
    timing = PlanTiming(clock=lambda: now[0])
    timing.received(1, None)
    assert timing.finish("stopped").endswith("; stopped")
    now[0] = 0.5
    timing.received(2, None)
    assert timing.received(3, None) is None  # path 2 superseded within 1 s
    now[0] = 1.5
    line = timing.finish("reservation rejected")
    assert line.startswith("exploration timing: path 3 ")
    assert line.endswith("reservation rejected; 1 earlier line(s) suppressed")


def test_exploration_logs_one_timing_line_when_the_robot_moves():
    bridge, explorer = rig()
    pose = {"x": 0.0, "y": 0.0}
    bridge._route_progress_pose = Mock(side_effect=lambda frame: dict(pose))
    info = bridge.node.get_logger.return_value.info
    explorer.start()
    explorer.on_path(path())
    bridge.follow_path.assert_called_once()
    bridge._route_progress_pose.assert_called_with("map")
    explorer.controller_accepted(bridge._goal_generation)
    explorer.tick()
    assert not any("exploration timing" in str(c) for c in info.call_args_list)
    pose["x"] = 0.2
    explorer.tick()
    explorer.tick()
    lines = [c.args[0] for c in info.call_args_list if "timing" in c.args[0]]
    assert len(lines) == 1
    assert "->sent" in lines[0] and "->accepted" in lines[0]
    assert "->moved" in lines[0] and lines[0].endswith("; moving")
