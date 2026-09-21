"""MGG objective planning boundary without a ROS installation."""

from concurrent.futures import Future
from copy import deepcopy
import math
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from adapters import objective_planning
from adapters.objective_planning import MggObjectivePlanning


class Request:
    NAVIGATE = 1
    RETURN_HOME = 2

    def __init__(self):
        self.goal = pose(0, 0)
        self.mission_id = self.component_id = self.goal_landmark_id = ""
        self.graph_revision = self.map_revision = self.objective = 0
        self.map_epoch = self.mapping_graph_revision = 0
        self.geometry_revision = ""
        self.map_source_stamp = NS(sec=0, nanosec=0)


class Response:
    SUCCEEDED = 0
    UNREACHABLE = 1
    STALE_REVISION = 2
    UNSUPPORTED = 3
    BLOCKED = 4


class Service:
    Request = Request
    Response = Response


class ValidateRouteRequest:
    def __init__(self):
        self.mission_id = self.component_id = self.frame_id = ""
        self.path = []
        self.lookahead_m = 0.0


class ValidateRouteResponse:
    VALID = 0
    INVALID = 1
    UNAVAILABLE = 2


class ValidateRouteService:
    Request = ValidateRouteRequest
    Response = ValidateRouteResponse


class RefineRouteRequest:
    def __init__(self):
        self.mission_id = self.route_id = self.component_id = ""
        self.graph_revision = self.map_revision = 0
        self.map_epoch = self.mapping_graph_revision = 0
        self.geometry_revision = ""
        self.map_source_stamp = NS(sec=0, nanosec=0)


class RefineRouteResponse:
    SUCCEEDED = 0
    BLOCKED = 1
    UNSUPPORTED = 2
    STALE_REVISION = 3


class RefineRouteService:
    Request = RefineRouteRequest
    Response = RefineRouteResponse


def pose(x, y, z=0.4, yaw=0.0):
    return NS(
        position=NS(x=x, y=y, z=z),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    )


def rig(monkeypatch, response, *, refine_responses=None):
    package = ModuleType("mgg_msgs")
    services = ModuleType("mgg_msgs.srv")
    services.PlanObjective = Service
    if refine_responses is not None:
        services.RefineObjectiveRoute = RefineRouteService
    monkeypatch.setitem(sys.modules, "mgg_msgs", package)
    monkeypatch.setitem(sys.modules, "mgg_msgs.srv", services)
    client = Mock()
    client.wait_for_service.return_value = True

    def full_response(request):
        result = deepcopy(response)
        if result.status == Response.SUCCEEDED and not result.partial and result.path:
            result.path[-1].position.x = request.goal.position.x
            result.path[-1].position.y = request.goal.position.y
        future = Future()
        future.set_result(result)
        return future

    client.call_async.side_effect = full_response
    refine_client = Mock()
    refine_client.wait_for_service.return_value = True
    if refine_responses is not None:
        pending_refinements = list(refine_responses)

        def refine_response(request):
            result = deepcopy(pending_refinements.pop(0))
            result.component_id = request.component_id
            result.map_epoch = request.map_epoch
            result.mapping_graph_revision = request.mapping_graph_revision
            result.geometry_revision = request.geometry_revision
            result.map_source_stamp = deepcopy(request.map_source_stamp)
            future = Future()
            future.set_result(result)
            return future

        refine_client.call_async.side_effect = refine_response
    bridge = Mock()
    bridge.id = "robot_1"
    bridge.navigation_frame = "robot_1/navigation_frame"
    bridge.cfg = {}
    bridge._mapping_authority = NS(current=lambda: None)
    bridge._goal_generation = 4
    bridge._goal_lock = threading.RLock()
    bridge.node.create_client.side_effect = lambda service, _name: (
        refine_client if service is RefineRouteService else client
    )
    bridge.node.get_clock().now().to_msg.return_value = NS(sec=12, nanosec=34)

    def cancel():
        with bridge._goal_lock:
            bridge._goal_generation += 1
            bridge.nav_status = "cancelled"
            return bridge._goal_generation

    bridge.cancel_goal.side_effect = cancel

    def cancel_if_current(expected, *, pending=False):
        with bridge._goal_lock:
            if expected != bridge._goal_generation:
                return None
            generation = cancel()
            bridge.nav_status = "active" if pending else "cancelled"
            return generation

    def set_status(expected, status):
        with bridge._goal_lock:
            if expected != bridge._goal_generation:
                return False
            bridge.nav_status = status
            return True

    bridge.cancel_goal_if_current.side_effect = cancel_if_current
    bridge.set_nav_status_if_current.side_effect = set_status
    bridge.set_goal_pending_if_current.side_effect = lambda expected: set_status(
        expected, "active"
    )
    bridge.wait_goal_quiet.side_effect = (
        lambda expected, _deadline: expected == bridge._goal_generation
    )

    def follow(_plan, *, expected_generation=None, not_after=None, pre_submit=None):
        with bridge._goal_lock:
            if not_after is not None and time.monotonic() >= not_after:
                return None
            if (
                expected_generation is not None
                and expected_generation != bridge._goal_generation
            ):
                return None
            if pre_submit is not None and not pre_submit():
                return None
            bridge._goal_generation += 1
            bridge.nav_status = "active"
            return bridge._goal_generation

    bridge.follow_path.side_effect = follow
    planner = MggObjectivePlanning(
        bridge,
        {"component_id": "component-a", "objective_timeout_s": 0.1},
    )
    return bridge, planner, client


def success_path():
    return NS(
        status=Response.SUCCEEDED,
        component_id="component-a",
        graph_revision=7,
        map_revision=9,
        map_epoch=0,
        mapping_graph_revision=0,
        geometry_revision="",
        map_source_stamp=NS(sec=0, nanosec=0),
        reason="",
        partial=False,
        indexed_map_validated=False,
        path=[pose(0, 0, 0.4), pose(1, 0.5, 0.41), pose(2, 1, 0.39)],
    )


def planning_budget_exhausted():
    result = success_path()
    result.status = Response.BLOCKED
    result.reason = (
        "primary direct: grid refinement exceeded its cooperative deadline "
        "[grid evidence: expansions=46]"
    )
    result.path = []
    return result


def rolling_home_response(*, partial, path_x, global_path=None, indexed=True):
    return NS(
        status=RefineRouteResponse.SUCCEEDED,
        component_id="component-a",
        graph_revision=7,
        map_revision=9,
        map_epoch=0,
        mapping_graph_revision=0,
        geometry_revision="a" * 64,
        map_source_stamp=NS(sec=0, nanosec=0),
        reason="",
        partial=partial,
        indexed_map_validated=indexed,
        route_id="route-instance-1",
        path=[pose(x, 1.0, 0.4) for x in path_x],
        global_path=(
            [pose(x, 1.0, 0.4) for x in global_path] if global_path is not None else []
        ),
    )


def test_navigate_requests_snapshot_and_executes_complete_path(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    assert planner.navigate({"x": 2, "y": 1, "yaw": 0.3})

    request = client.call_async.call_args.args[0]
    assert request.objective == Request.NAVIGATE
    assert request.component_id == "component-a"
    assert request.goal.position.x == 2
    assert request.goal.orientation.z == pytest.approx(math.sin(0.15))
    plan = bridge.follow_path.call_args.args[0]
    assert plan.revision_ns == 12_000_000_034
    assert [(p.x, p.y, p.z) for p in plan.poses] == [
        (0, 0, 0.4),
        (1, 0.5, 0.41),
        (2, 1, 0.39),
    ]


def test_native_planner_rejection_reason_is_published_with_failed_state(monkeypatch):
    response = success_path()
    response.status = 7
    response.reason = (
        "current pose rejected: no mapped ground support at (0.01, 0.00, -0.00)"
    )
    bridge, planner, _ = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2, "y": 1})
    state = planner.decorate_state({"nav_status": "failed"})
    assert state["nav_failure_reason"] == response.reason


@pytest.mark.parametrize("objective", ["navigate", "return_home"])
def test_temporary_grid_budget_retries_without_reporting_unreachable(
    monkeypatch, objective
):
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.5
    planner.replan_max_attempts = 2
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    successful = success_path()
    if objective == "return_home":
        successful.path[-1] = pose(-2, 1)
    client.call_async.side_effect = [
        completed(planning_budget_exhausted()),
        completed(successful),
    ]

    target = {"x": 2, "y": 1}
    started = (
        planner.navigate(target)
        if objective == "navigate"
        else planner.return_home(target)
    )
    assert started
    assert wait_until(lambda: bridge.follow_path.call_count == 1)
    assert client.call_async.call_count == 2
    assert bridge.nav_status == "active"
    assert planner._blocked_retry_count == 0


def test_repeated_grid_budget_exhaustion_is_bounded_and_reported_honestly(
    monkeypatch,
):
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.5
    planner.replan_max_attempts = 2
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    client.call_async.side_effect = [
        completed(planning_budget_exhausted()) for _ in range(3)
    ]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.nav_status == "failed")
    assert client.call_async.call_count == 3  # Initial request plus two retries.
    assert bridge.follow_path.call_count == 0
    assert planner._blocked_retry_count == 0
    state = planner.decorate_state({"nav_status": "failed"})
    assert (
        "planning budget remained exhausted after 2 retries"
        in state["nav_failure_reason"]
    )
    assert "unreachable" not in state["nav_failure_reason"].lower()


def test_grid_budget_retry_stops_on_a_real_planner_rejection(monkeypatch):
    rejected = success_path()
    rejected.status = Response.BLOCKED
    rejected.reason = "goal footprint has known rise 0.143m above limit 0.100m"
    rejected.path = []
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.5
    planner.replan_max_attempts = 3
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    client.call_async.side_effect = [
        completed(planning_budget_exhausted()),
        completed(rejected),
    ]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.nav_status == "failed")
    assert client.call_async.call_count == 2
    assert (
        planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
        == rejected.reason
    )


def test_grid_budget_retry_cannot_adopt_authority_changed_during_initial_rpc(
    monkeypatch,
):
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])

    class SwitchingFuture:
        def done(self):
            return True

        def result(self):
            authority[0] = correction_authority(mission_id="replacement-mission")
            return planning_budget_exhausted()

    client.call_async.side_effect = lambda _request: SwitchingFuture()

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.nav_status == "failed")
    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 1
    bridge.follow_path.assert_not_called()
    assert (
        "mission or component changed"
        in planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
    )


def test_grid_budget_retry_accepts_same_component_authority_refresh(monkeypatch):
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])

    class CorrectingFuture:
        def done(self):
            return True

        def result(self):
            authority[0] = correction_authority(revision=2, x=0.1)
            return success_path()

    client.call_async.side_effect = [
        completed(planning_budget_exhausted()),
        CorrectingFuture(),
        completed(success_path()),
    ]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.follow_path.call_count == 1)
    assert client.call_async.call_count == 3
    assert bridge.nav_status == "active"
    assert planner._blocked_retry_count == 0


@pytest.mark.parametrize(
    "status,reason",
    [
        (Response.UNREACHABLE, "grid refinement exceeded its cooperative deadline"),
        (Response.BLOCKED, "grid refinement exceeded a time budget"),
    ],
)
def test_planning_budget_retry_requires_blocked_status_and_exact_native_reason(
    monkeypatch, status, reason
):
    response = success_path()
    response.status = status
    response.reason = reason
    response.path = []
    bridge, planner, client = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2, "y": 1})
    assert bridge.nav_status == "failed"
    assert client.call_async.call_count == 1


def test_stop_wins_while_grid_budget_retry_is_in_flight(monkeypatch):
    bridge, planner, client = rig(monkeypatch, planning_budget_exhausted())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.5
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    pending = Future()
    client.call_async.side_effect = [
        completed(planning_budget_exhausted()),
        pending,
    ]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: client.call_async.call_count == 2)
    bridge.cancel_goal()
    pending.set_result(success_path())

    assert wait_until(lambda: planner._recovery_thread is None)
    assert bridge.nav_status == "cancelled"
    bridge.follow_path.assert_not_called()
    assert planner._blocked_retry_count == 0


def test_controller_failure_reason_is_preserved_when_planner_has_none(monkeypatch):
    bridge, planner, _ = rig(monkeypatch, success_path())
    state = planner.decorate_state(
        {
            "nav_status": "failed",
            "nav_failure_reason": "FollowPath: Failed to make progress",
        }
    )
    assert state["nav_failure_reason"] == "FollowPath: Failed to make progress"


def test_planner_failure_reason_takes_precedence_over_controller_reason(monkeypatch):
    response = success_path()
    response.status = 7
    response.reason = "planner rejected route"
    bridge, planner, _ = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2, "y": 1})
    state = planner.decorate_state(
        {
            "nav_status": "failed",
            "nav_failure_reason": "FollowPath: Failed to make progress",
        }
    )
    assert state["nav_failure_reason"] == response.reason


def test_failure_reason_is_bounded_and_cleared_by_replacement(monkeypatch):
    response = success_path()
    response.status = 7
    response.reason = "native rejection: " + ("x" * 1000)
    bridge, planner, _ = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2, "y": 1})
    state = planner.decorate_state({"nav_status": "failed"})
    assert len(state["nav_failure_reason"]) == 512

    planner.claim_objective("navigate", {"x": 3, "y": 4})
    assert "nav_failure_reason" not in planner.decorate_state({"nav_status": "active"})


def test_late_native_rejection_cannot_overwrite_a_stopped_generation(monkeypatch):
    response = success_path()
    response.status = 7
    response.reason = "late native rejection"
    bridge, planner, client = rig(monkeypatch, response)
    pending = Future()
    client.call_async.side_effect = None
    client.call_async.return_value = pending

    claim = planner.claim_objective("navigate", {"x": 1, "y": 2})
    result = []
    worker = threading.Thread(
        target=lambda: result.append(planner.execute_claimed(claim)), daemon=True
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while not client.call_async.called and time.monotonic() < deadline:
        time.sleep(0.001)
    bridge.cancel_goal()
    pending.set_result(response)
    worker.join(1.0)

    assert result == [False]
    assert "nav_failure_reason" not in planner.decorate_state(
        {"nav_status": "cancelled"}
    )


def test_return_home_uses_configured_goal(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.home = {"x": -1, "y": -2}
    bridge.follow_path.return_value = True
    assert planner.return_home()
    request = client.call_async.call_args.args[0]
    assert request.objective == Request.RETURN_HOME
    assert request.goal.position.x == -1


def test_return_home_uses_corrected_authority_anchor(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = {
        "mission_id": "mission-1",
        "component_id": "component-a",
        "map_epoch": 0,
        "navigation_frame": "robot_1/navigation_frame",
        "mapping_graph_revision": 0,
        "geometry_revision": "a" * 64,
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "home": {
            "keyframe_id": "kf-home",
            "T_navigation_home": [
                [0.0, -1.0, 0.0, 3.0],
                [1.0, 0.0, 0.0, -2.0],
                [0.0, 0.0, 1.0, 0.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    }
    planner.authority_reader = NS(current=lambda: authority)
    bridge.exploration = None
    # A stale caller pose must not override the corrected authority anchor.
    assert planner.return_home({"x": 99.0, "y": 99.0})
    request = client.call_async.call_args.args[0]
    assert request.mission_id == "mission-1"
    assert request.goal_landmark_id == "kf-home"
    assert request.goal.position.x == 3.0
    assert request.goal.position.y == -2.0
    assert request.goal.orientation.z == pytest.approx(math.sin(math.pi / 4.0))


def test_objective_binds_and_validates_indexed_map_snapshot(monkeypatch):
    response = success_path()
    response.indexed_map_validated = True
    digest = "a" * 64
    response.map_epoch = 11
    response.mapping_graph_revision = 12
    response.geometry_revision = digest
    response.map_source_stamp = NS(sec=13, nanosec=14)
    bridge, planner, client = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    planner.authority_reader = NS(
        current=lambda: {
            "component_id": "component-a",
            "map_epoch": 11,
            "mapping_graph_revision": 12,
            "geometry_revision": digest,
            "map_source_stamp": {"sec": 13, "nanosec": 14},
        }
    )
    assert planner.navigate({"x": 2, "y": 1})
    request = client.call_async.call_args.args[0]
    assert (request.map_epoch, request.mapping_graph_revision) == (11, 12)
    assert request.geometry_revision == digest
    assert (request.map_source_stamp.sec, request.map_source_stamp.nanosec) == (
        13,
        14,
    )

    response.geometry_revision = "b" * 64
    assert not planner.navigate({"x": 2, "y": 1})


def test_native_only_route_does_not_claim_or_bind_to_indexed_snapshot(monkeypatch):
    response = success_path()
    response.geometry_revision = "b" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority(geometry="a" * 64)]
    planner.authority_reader = NS(current=lambda: authority[0])

    class PublishingFuture:
        published = False

        def done(self):
            if not self.published:
                authority[0] = correction_authority(geometry="c" * 64)
                self.published = True
            return True

        def result(self):
            return response

    client.call_async.side_effect = None
    client.call_async.return_value = PublishingFuture()

    assert planner.navigate({"x": 2.0, "y": 1.0})
    bridge.follow_path.assert_called_once()
    request = client.call_async.call_args.args[0]
    assert request.geometry_revision == "a" * 64
    assert response.geometry_revision != request.geometry_revision


@pytest.mark.parametrize(
    "field,value",
    [("partial", 1), ("indexed_map_validated", "true")],
)
def test_response_evidence_flags_are_strict_booleans(monkeypatch, field, value):
    response = success_path()
    setattr(response, field, value)
    bridge, planner, _ = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2.0, "y": 1.0})
    bridge.follow_path.assert_not_called()


def test_stop_generation_fences_late_service_result(monkeypatch):
    response = success_path()
    bridge, planner, client = rig(monkeypatch, response)

    class StoppingFuture:
        def done(self):
            bridge._goal_generation += 1
            return False

        def cancel(self):
            self.cancelled = True

    pending = StoppingFuture()
    client.call_async.side_effect = None
    client.call_async.return_value = pending
    assert not planner.navigate({"x": 1, "y": 2})
    assert pending.cancelled
    bridge.follow_path.assert_not_called()


def test_claimed_initial_plan_cannot_submit_after_stop(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    pending = Future()
    client.call_async.side_effect = None
    client.call_async.return_value = pending

    claim = planner.claim_objective("navigate", {"x": 1, "y": 2})
    assert claim.generation == bridge._goal_generation
    result = []
    worker = threading.Thread(
        target=lambda: result.append(planner.execute_claimed(claim)), daemon=True
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while not client.call_async.called and time.monotonic() < deadline:
        time.sleep(0.001)
    assert client.call_async.called

    bridge.cancel_goal()
    pending.set_result(success_path())
    worker.join(1.0)

    assert result == [False]
    bridge.follow_path.assert_not_called()


def test_claimed_initial_plan_retains_goal_until_planner_returns(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    pending = Future()
    client.call_async.side_effect = None
    client.call_async.return_value = pending
    goal = {"x": 1, "y": 2, "yaw": 0.3}

    claim = planner.claim_objective("navigate", goal)
    worker = threading.Thread(
        target=lambda: planner.execute_claimed(claim), daemon=True
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while not client.call_async.called and time.monotonic() < deadline:
        time.sleep(0.001)
    assert client.call_async.called

    public = planner.decorate_state({"nav_status": "idle", "goal": None})
    assert public["nav_status"] == "active"
    assert public["goal"] == {
        **goal,
        "frame_id": "robot_1/navigation_frame",
    }
    assert public["objective_continuation"] == {
        "objective": "navigate",
        "evidence_source": "mgg_native",
        "phase": "planning",
    }

    bridge.cancel_goal()
    stopped = planner.decorate_state({"nav_status": "idle", "goal": None})
    assert stopped == {"nav_status": "idle", "goal": None}
    pending.set_result(success_path())
    worker.join(1.0)
    bridge.follow_path.assert_not_called()


def test_manual_command_while_initial_home_plan_is_pending_wins(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    planner.authority_reader = NS(current=lambda: correction_authority())

    class ManualCommandFuture:
        replaced = False

        def done(self):
            if not self.replaced:
                self.replaced = True
                bridge.cancel_goal()
                bridge.nav_status = "active"
                bridge.mode = "teleop"
            return False

        def cancel(self):
            self.cancelled = True

    pending = ManualCommandFuture()
    client.call_async.side_effect = None
    client.call_async.return_value = pending

    assert not planner.return_home()
    assert pending.cancelled
    bridge.follow_path.assert_not_called()
    assert bridge.nav_status == "active"
    assert bridge.mode == "teleop"


def test_rejects_stale_component_and_nonplanar_result(monkeypatch):
    response = success_path()
    response.component_id = "component-b"
    bridge, planner, _ = rig(monkeypatch, response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def correction_authority(
    revision=1,
    x=0.0,
    geometry="a" * 64,
    mapping_graph_revision=0,
    home_x=-2.0,
    mission_id="mission-1",
    landmark_id="kf-home",
    map_epoch=0,
    navigation_frame="robot_1/navigation_frame",
    solution_order=(1, 0),
    yaw=0.0,
    home_yaw=0.0,
):
    return {
        "mission_id": mission_id,
        "component_id": "component-a",
        "navigation_frame": navigation_frame,
        "solution_order": list(solution_order),
        "correction_revision": revision,
        "map_epoch": map_epoch,
        "mapping_graph_revision": mapping_graph_revision,
        "geometry_revision": geometry,
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "home": {
            "keyframe_id": landmark_id,
            "T_navigation_home": [
                [math.cos(home_yaw), -math.sin(home_yaw), 0.0, home_x],
                [math.sin(home_yaw), math.cos(home_yaw), 0.0, 1.0],
                [0.0, 0.0, 1.0, 0.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        "T_component_navigation": [
            [math.cos(yaw), -math.sin(yaw), 0.0, x],
            [math.sin(yaw), math.cos(yaw), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }


def stable_planning_authority(*, map_x=0.0, planning_x=0.0, revision=1):
    authority = correction_authority(revision=revision, x=map_x)
    authority["planning_frame"] = "robot_1/odom"
    authority["T_component_planning"] = [
        [1.0, 0.0, 0.0, planning_x],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return authority


def wait_until(predicate, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def completed(response):
    future = Future()
    future.set_result(response)
    return future


def return_home_rig(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.25
    assert planner.return_home()
    bridge.nav_status = "active"
    return bridge, planner, client, authority


def test_correction_while_planning_retries_with_latest_authority(monkeypatch):
    response = success_path()
    response.path[-1] = pose(1.0, 2.0)
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])

    class CorrectingFuture:
        corrected = False

        def done(self):
            if not self.corrected:
                authority[0] = correction_authority(revision=2, x=0.1)
                self.corrected = True
            return True

        def result(self):
            return response

    client.call_async.side_effect = None
    client.call_async.return_value = CorrectingFuture()
    assert planner.navigate({"x": 1, "y": 2})
    assert wait_until(lambda: client.call_async.call_count == 2)
    assert wait_until(lambda: bridge.follow_path.call_count == 1)


def test_active_objective_ignores_metadata_correction_but_fails_on_authority_loss(
    monkeypatch,
):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    # A correction revision alone is metadata; the bound transform did not move.
    authority[0] = correction_authority(revision=2, x=0.0)
    planner._check_active_authority()
    bridge.cancel_goal_if_current.assert_not_called()
    assert bridge.nav_status == "active"

    # A second route also fails closed when the once-valid authority expires.
    bridge.nav_status = "idle"
    authority[0] = correction_authority(revision=3, x=0.1)
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"
    authority[0] = None
    planner._check_active_authority()
    assert bridge.cancel_goal_if_current.call_count == 1


def test_active_objective_ignores_geometry_and_noop_correction_updates(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    authority[0] = correction_authority(
        revision=1, geometry="b" * 64, mapping_graph_revision=99
    )
    planner._check_active_authority()

    assert bridge.cancel_goal.call_count == 1
    assert bridge.nav_status == "active"
    assert planner._active_route is not None


def test_new_goal_generation_wins_over_a_stale_authority_timer(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    initial = correction_authority()
    authority = [initial]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    replacement_generation = bridge._goal_generation + 1

    def replaced_while_timer_checked():
        bridge._goal_generation = replacement_generation
        return correction_authority(revision=2, x=0.1)

    planner.authority_reader = NS(current=replaced_while_timer_checked)
    prior_cancels = bridge.cancel_goal.call_count
    planner._check_active_authority()

    assert bridge.cancel_goal.call_count == prior_cancels
    assert bridge._goal_generation == replacement_generation


def test_reused_correction_revision_still_enforces_transform_tolerance(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=1, x=0.01)
    planner._check_active_authority()
    assert bridge.cancel_goal.call_count == 1
    authority[0] = correction_authority(revision=1, x=0.30)
    planner._check_active_authority()
    bridge.cancel_goal_if_current.assert_called_once()

    response = success_path()
    response.path[-1].position.x = 1.0
    response.path[-1].position.y = 0.5
    response.path[-1].position.z = 1.0
    bridge, planner, _ = rig(monkeypatch, response)
    planner.client.call_async.side_effect = None
    planner.client.call_async.return_value = completed(response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def test_return_home_replans_to_latest_authority_home_without_terminal_gap(
    monkeypatch,
):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    authority[0] = correction_authority(revision=2, x=0.03, home_x=-1.7)

    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert wait_until(lambda: planner._active_route is not None)
    retry = client.call_async.call_args.args[0]
    assert retry.goal.position.x == pytest.approx(-1.7)
    assert retry.mission_id == "mission-1"
    assert retry.goal_landmark_id == "kf-home"
    assert bridge.nav_status == "active"
    assert bridge.cancel_goal_if_current.call_count == 1


def test_return_home_uses_execution_budget_for_anchor_only_updates(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)

    authority[0] = correction_authority(revision=2, x=0.0, home_x=-2.0)
    planner._check_active_authority()
    assert client.call_async.call_count == 1
    bridge.cancel_goal_if_current.assert_not_called()

    authority[0] = correction_authority(revision=3, x=0.0, home_x=-1.8)
    planner._check_active_authority()
    assert client.call_async.call_count == 1
    bridge.cancel_goal_if_current.assert_not_called()

    authority[0] = correction_authority(revision=4, x=0.0, home_x=-1.7)
    planner._check_active_authority()
    assert wait_until(lambda: client.call_async.call_count == 2)
    assert client.call_async.call_args.args[0].goal.position.x == pytest.approx(-1.7)
    assert "home_anchor_shift=0.300m" in (
        bridge.node.get_logger().warning.call_args.args[0]
    )


def test_return_home_coalesces_corrections_before_the_worker_plans(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.05
    authority[0] = correction_authority(revision=2, x=0.03, home_x=-1.7)

    planner._check_active_authority()
    worker = planner._recovery_thread
    authority[0] = correction_authority(revision=3, x=0.06, home_x=-1.5)
    # Enqueuing the same retained intent cannot create a second worker.
    assert planner._start_recovery(
        planner._recovery_generation, planner._home_intent, cancel_route=False
    )
    assert planner._recovery_thread is worker

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert client.call_async.call_args.args[0].goal.position.x == pytest.approx(-1.5)
    bridge.cancel_goal()


def test_return_home_heading_uses_explicit_execution_tolerance(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.execution_goal_yaw_tolerance_rad = 0.25

    authority[0] = correction_authority(revision=2, home_yaw=0.20)
    planner._check_active_authority()
    bridge.cancel_goal_if_current.assert_not_called()

    authority[0] = correction_authority(revision=3, home_yaw=0.30)
    planner._check_active_authority()
    assert wait_until(lambda: client.call_async.call_count == 2)
    assert "home_anchor_yaw_shift=0.300rad" in (
        bridge.node.get_logger().warning.call_args.args[0]
    )


def test_return_home_recovery_retries_then_fails_with_a_bound(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_max_attempts = 2
    client.wait_for_service.side_effect = [False, False]
    authority[0] = correction_authority(revision=2, x=0.30)

    planner._check_active_authority()

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert client.wait_for_service.call_count == 3  # initial plan plus two retries
    assert client.call_async.call_count == 1
    warning = bridge.node.get_logger().warning.call_args.args[0]
    assert "exhausted after 2 attempts" in warning


def test_return_home_recovery_retries_synchronous_service_submission(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    retry_response = success_path()
    retry_response.path[-1] = pose(-2.0, 1.0)
    retry_response.geometry_revision = "a" * 64
    ready = Future()
    ready.set_result(retry_response)
    client.call_async.side_effect = [RuntimeError("transport reset"), ready]
    authority[0] = correction_authority(revision=2, x=0.30)

    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 3)
    assert wait_until(lambda: planner._active_route is not None)
    assert bridge.nav_status == "active"


@pytest.mark.parametrize(
    "replacement",
    [
        {"mission_id": "mission-2"},
        {"landmark_id": "kf-other"},
        {"map_epoch": 1},
        {"navigation_frame": "robot_1/replacement_map"},
    ],
)
def test_return_home_authority_identity_change_fails_closed(monkeypatch, replacement):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    authority[0] = correction_authority(
        mission_id=replacement.get("mission_id", "mission-1"),
        landmark_id=replacement.get("landmark_id", "kf-home"),
        map_epoch=replacement.get("map_epoch", 0),
        navigation_frame=replacement.get("navigation_frame", "robot_1/navigation_frame"),
    )

    planner._check_active_authority()

    assert bridge.nav_status == "failed"
    assert client.call_async.call_count == 1
    assert planner._recovery_thread is None


def test_stop_during_return_home_recovery_backoff_wins_without_status_write(
    monkeypatch,
):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.08
    authority[0] = correction_authority(revision=2, x=0.30)
    planner._check_active_authority()

    bridge.cancel_goal()
    bridge.nav_status = "idle"  # Stop's public terminal state.

    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 1
    assert bridge.nav_status == "idle"


def test_manual_navigate_during_recovery_backoff_replaces_home_intent(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.08
    authority[0] = correction_authority(revision=2, x=0.30)
    planner._check_active_authority()

    assert planner.navigate({"x": 4.0, "y": 5.0})

    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 2
    assert client.call_async.call_args.args[0].objective == Request.NAVIGATE
    assert bridge.nav_status == "active"
    assert planner._home_intent is None


def test_stop_while_recovery_service_future_is_pending_wins(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)

    class StoppingFuture:
        stopped = False

        def done(self):
            if not self.stopped:
                self.stopped = True
                bridge.cancel_goal()
                bridge.nav_status = "idle"
            return False

        def cancel(self):
            self.cancelled = True

    pending = StoppingFuture()
    client.call_async.side_effect = None
    client.call_async.return_value = pending
    authority[0] = correction_authority(revision=2, x=0.30)

    planner._check_active_authority()

    assert wait_until(lambda: getattr(pending, "cancelled", False))
    assert wait_until(lambda: planner._recovery_thread is None)
    assert bridge.follow_path.call_count == 1
    assert bridge.nav_status == "idle"


def test_recovery_keeps_a_plan_when_a_newer_product_of_the_same_epoch_lands(
    monkeypatch,
):
    # A product transition (new revision and geometry, same epoch, same frame)
    # during planning does not discard the plan: the map server keeps
    # answering the key it was validated against.
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    response_a = success_path()
    response_a.path[-1] = pose(-2.0, 1.0)
    response_a.indexed_map_validated = True
    response_a.geometry_revision = "a" * 64

    class SnapshotChangingFuture:
        changed = False

        def done(self):
            if not self.changed:
                self.changed = True
                authority[0] = correction_authority(
                    revision=2, x=0.30, geometry="b" * 64, mapping_graph_revision=1
                )
            return True

        def result(self):
            return response_a

    client.call_async.side_effect = [SnapshotChangingFuture()]
    authority[0] = correction_authority(revision=2, x=0.30)
    before = client.call_async.call_count

    planner._check_active_authority()

    assert wait_until(lambda: planner._active_route is not None)
    assert client.call_async.call_count == before + 1
    assert client.call_async.call_args.args[0].geometry_revision == "a" * 64
    assert bridge.nav_status == "active"


def test_pre_submit_validation_accepts_a_product_transition(monkeypatch):
    # A newer product of the same epoch landing between the plan and its
    # dispatch is not a frame change: the route goes out once.
    response = success_path()
    response.indexed_map_validated = True
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    planner.replan_backoff_s = 0.0
    original_follow = bridge.follow_path.side_effect

    def churn_before_submit(plan, **kwargs):
        authority[0] = correction_authority(geometry="b" * 64, mapping_graph_revision=1)
        return original_follow(plan, **kwargs)

    bridge.follow_path.side_effect = churn_before_submit

    assert planner.navigate({"x": 1.0, "y": 2.0})
    assert wait_until(lambda: bridge.nav_status == "active")
    assert bridge.follow_path.call_count == 1
    assert client.call_async.call_count == 1


def test_pre_submit_authority_validation_blocks_epoch_churn(monkeypatch):
    response = success_path()
    response.indexed_map_validated = True
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    planner.replan_backoff_s = 0.0
    original_follow = bridge.follow_path.side_effect
    epoch = [0]

    def churn_before_submit(plan, **kwargs):
        epoch[0] += 1
        authority[0] = correction_authority(geometry="b" * 64, map_epoch=epoch[0])
        return original_follow(plan, **kwargs)

    bridge.follow_path.side_effect = churn_before_submit

    assert planner.navigate({"x": 1.0, "y": 2.0})
    assert wait_until(lambda: bridge.nav_status == "failed")
    # Continuous epoch churn is bounded; an unchecked route is never sent.
    assert bridge.follow_path.call_count == 1
    assert client.call_async.call_count == 4


@pytest.mark.parametrize("objective", ["navigate", "return_home"])
def test_partial_route_is_rejected_before_dispatch(monkeypatch, objective):
    response = success_path()
    response.partial = True
    bridge, planner, _ = rig(monkeypatch, response)
    planner.home = {"x": 2.0, "y": 1.0}

    accepted = (
        planner.navigate({"x": 2.0, "y": 1.0})
        if objective == "navigate"
        else planner.return_home()
    )

    assert not accepted
    bridge.follow_path.assert_not_called()
    assert "installed ABI" in bridge.node.get_logger().warning.call_args.args[0]


def test_partial_navigate_refines_local_chunks_without_moving_global_goal(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
    )
    middle = rolling_home_response(partial=True, path_x=[-0.5, -1.2])
    final = rolling_home_response(partial=False, path_x=[-1.2, -2.0])
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=[middle, final])
    planner.authority_reader = NS(current=lambda: correction_authority())

    goal = {"x": -2.0, "y": 1.0}
    assert planner.navigate(goal)
    first_global = planner.global_display_plan()
    assert first_global.poses[-1].x == pytest.approx(goal["x"])
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-0.5)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert planner.global_display_plan() is first_global
    assert planner._objective_goal == goal
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-1.2)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.follow_path.call_count == 3)
    assert planner.global_display_plan() is first_global
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-2.0)
    assert (
        planner.decorate_state({"nav_status": "active"})["objective_continuation"][
            "phase"
        ]
        == "following_final"
    )

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert planner._active_route is None
    assert planner.global_display_plan() is None


def test_navigate_follows_short_validated_prefix_sections_to_a_far_goal(monkeypatch):
    """A goal beyond the measured floor arrives section by section.

    Native MGG shortens each section to the terrain its indexed authority has
    validated, so the sections are much shorter than the planning horizon. Only
    the final one owns the exact destination.
    """

    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, 2.0],
        global_path=[0.0, 2.0, 5.0, 8.0, 12.0],
    )
    sections = [
        rolling_home_response(partial=True, path_x=[2.0, 5.0]),
        rolling_home_response(partial=True, path_x=[5.0, 8.0]),
        rolling_home_response(partial=False, path_x=[8.0, 12.0]),
    ]
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=sections)
    planner.authority_reader = NS(current=lambda: correction_authority())

    goal = {"x": 12.0, "y": 1.0}
    assert planner.navigate(goal)
    retained_global = planner.global_display_plan()
    assert retained_global.poses[-1].x == pytest.approx(12.0)
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(2.0)

    for expected_x in (5.0, 8.0):
        assert (
            planner.decorate_state({"nav_status": "active"})["objective_continuation"][
                "phase"
            ]
            == "following_local"
        )
        bridge.nav_status = "succeeded"
        planner._check_active_authority()
        assert wait_until(
            lambda x=expected_x: bridge.follow_path.call_args.args[0].poses[-1].x
            == pytest.approx(x)
        )
        # The committed destination never moves while prefixes are followed.
        assert planner.global_display_plan() is retained_global
        assert planner._objective_goal == goal
        assert planner._rolling_no_progress_sections == 0

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.follow_path.call_count == 4)
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(12.0)
    assert (
        planner.decorate_state({"nav_status": "active"})["objective_continuation"][
            "phase"
        ]
        == "following_final"
    )

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert planner._active_route is None
    assert planner.global_display_plan() is None


def test_prefix_sections_without_progress_end_the_objective_with_a_reason(
    monkeypatch,
):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, 2.0],
        global_path=[0.0, 2.0, 12.0],
    )
    # Each continuation validates terrain that leads nowhere nearer the goal.
    stalled = [
        rolling_home_response(partial=True, path_x=[2.0, 2.0 - step * 0.01])
        for step in (1, 2, 3)
    ]
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=stalled)
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert planner.navigate({"x": 12.0, "y": 1.0})
    for expected_calls in (2, 3):
        bridge.nav_status = "succeeded"
        planner._check_active_authority()
        assert wait_until(lambda n=expected_calls: bridge.follow_path.call_count == n)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.nav_status == "failed")
    # The third stalled section is refused before dispatch.
    assert bridge.follow_path.call_count == 3
    reason = planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
    assert (
        "came no closer to its goal across 3 consecutive validated sections" in reason
    )


def test_partial_navigate_rejects_final_chunk_before_exact_endpoint(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
    )
    wrong_final = rolling_home_response(partial=False, path_x=[-0.5, -1.99])
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=[wrong_final])
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert planner.navigate({"x": -2.0, "y": 1.0})
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert bridge.follow_path.call_count == 1
    assert (
        "endpoint differs"
        in planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
    )


def test_partial_navigate_compares_final_chunk_in_stable_planning_frame(monkeypatch):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, 0.5],
        global_path=[0.0, 0.5, 1.0],
    )
    initial.global_path[-1].position.y = 2.0
    final = rolling_home_response(partial=False, path_x=[0.5, 1.0])
    final.path[-1].position.y = 2.0
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=[final])
    authority = stable_planning_authority(map_x=1.0)
    authority["T_component_navigation"] = [
        [0.0, -1.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    planner.authority_reader = NS(current=lambda: authority)

    caller_goal = {"x": 2.0, "y": 0.0, "z": 0.4, "yaw": 0.0}
    assert planner.navigate(caller_goal)
    assert planner._objective_goal == caller_goal
    assert planner._rolling_planned_goal["x"] == pytest.approx(1.0)
    assert planner._rolling_planned_goal["y"] == pytest.approx(2.0)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(1.0)
    assert bridge.follow_path.call_args.args[0].poses[-1].y == pytest.approx(2.0)


@pytest.mark.parametrize("indexed", [False, True])
def test_partial_home_refines_each_success_without_moving_global_goal(
    monkeypatch, indexed
):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
        indexed=indexed,
    )
    middle = rolling_home_response(partial=True, path_x=[-0.5, -1.2], indexed=indexed)
    final = rolling_home_response(partial=False, path_x=[-1.2, -2.0], indexed=indexed)
    bridge, planner, _ = rig(
        monkeypatch,
        initial,
        refine_responses=[middle, final],
    )
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_backoff_s = 0.0

    assert planner.return_home()
    first_global = planner.global_display_plan()
    assert first_global.poses[-1].x == pytest.approx(-2.0)
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-0.5)
    continuation = planner.decorate_state({"nav_status": "active"})[
        "objective_continuation"
    ]
    assert continuation["phase"] == "following_local"
    assert continuation["evidence_source"] == (
        "mola_indexed" if indexed else "mgg_native"
    )

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert planner.global_display_plan() is first_global
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-1.2)
    assert planner._blocked_retry_count == 0

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(
        lambda: (
            planner.decorate_state({"nav_status": "active"})
            .get("objective_continuation", {})
            .get("phase")
            == "following_final"
        )
    )
    assert bridge.follow_path.call_count == 3
    assert planner.global_display_plan() is first_global
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-2.0)

    requests = planner.refine_client.call_async.call_args_list
    assert len(requests) == 2
    assert all(call.args[0].route_id == "route-instance-1" for call in requests)
    assert all(call.args[0].graph_revision == 0 for call in requests)
    assert all(call.args[0].geometry_revision == "a" * 64 for call in requests)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert planner._active_route is None
    assert planner.global_display_plan() is None


def test_final_home_chunk_matches_route_goal_across_small_anchor_correction(
    monkeypatch,
):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
    )
    final = rolling_home_response(partial=False, path_x=[-0.5, -2.0])
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=[final])
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)

    assert planner.return_home()
    authority["home"]["T_navigation_home"][0][3] = -1.982
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert bridge.nav_status == "active"
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-2.0)
    assert planner._objective_goal["x"] == pytest.approx(-2.0)


@pytest.mark.parametrize("objective", ["navigate", "return_home"])
def test_partial_success_is_hidden_and_cancellation_retires_refinement(
    monkeypatch, objective
):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
    )
    bridge, planner, _ = rig(
        monkeypatch,
        initial,
        refine_responses=[rolling_home_response(partial=False, path_x=[-0.5, -2.0])],
    )
    planner.authority_reader = NS(current=lambda: correction_authority())
    pending = Future()
    planner.refine_client.call_async.side_effect = None
    planner.refine_client.call_async.return_value = pending

    accepted = (
        planner.navigate({"x": -2.0, "y": 1.0})
        if objective == "navigate"
        else planner.return_home()
    )
    assert accepted
    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: planner.refine_client.call_async.called)

    public = planner.decorate_state({"nav_status": "succeeded", "goal": None})
    assert public["nav_status"] == "active"
    assert public["goal"]["x"] == pytest.approx(-2.0)
    assert public["objective_continuation"]["phase"] == "planning"

    bridge.cancel_goal()
    assert wait_until(pending.cancelled)
    assert planner.global_display_plan() is None
    assert (
        planner.decorate_state({"nav_status": "cancelled"})["nav_status"] == "cancelled"
    )


def test_partial_home_starts_next_refinement_while_prior_worker_exits(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
    )
    final = rolling_home_response(partial=False, path_x=[-0.5, -2.0])
    bridge, planner, _ = rig(monkeypatch, initial, refine_responses=[final])
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert planner.return_home()
    # A short controller chunk can finish before the worker which submitted it
    # has cleared its thread identity in `finally`. That stale identity must
    # not suppress the next local refinement.
    prior_worker = object()
    planner._continuation_thread = prior_worker
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert planner._continuation_thread is not prior_worker
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-2.0)


def test_pending_index_refinement_keeps_route_until_same_goal_is_validated(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
    )
    pending = rolling_home_response(partial=False, path_x=[])
    pending.status = RefineRouteResponse.STALE_REVISION
    pending.reason = "requested snapshot is not current"
    loading = deepcopy(pending)
    loading.status = RefineRouteResponse.BLOCKED
    loading.reason = "odometry or planning map is unavailable"
    final = rolling_home_response(partial=False, path_x=[-0.5, -2.0])
    bridge, planner, client = rig(
        monkeypatch, initial, refine_responses=[pending, loading, final]
    )
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_backoff_s = 0.0

    assert planner.return_home()
    retained = planner.global_display_plan()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert client.call_async.call_count == 1
    assert planner.global_display_plan() is retained
    assert bridge.follow_path.call_args.args[0].poses[-1].x == pytest.approx(-2.0)
    assert bridge.nav_status == "active"


def test_stop_during_pending_index_refinement_cannot_dispatch_late_path(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
    )
    pending = rolling_home_response(partial=False, path_x=[])
    pending.status = RefineRouteResponse.STALE_REVISION
    pending.reason = "requested snapshot is not current"
    final = rolling_home_response(partial=False, path_x=[-0.5, -2.0])
    bridge, planner, client = rig(
        monkeypatch, initial, refine_responses=[pending, final]
    )
    planner.authority_reader = NS(current=lambda: correction_authority())
    waiting = threading.Event()
    resume = threading.Event()
    wait_for_continuation = planner._wait_for_continuation

    def wait_after_refusal(*args):
        waiting.set()
        assert resume.wait(timeout=2.0)
        return wait_for_continuation(*args)

    monkeypatch.setattr(planner, "_wait_for_continuation", wait_after_refusal)
    assert planner.return_home()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    try:
        assert waiting.wait(timeout=2.0)
        bridge.cancel_goal()
    finally:
        resume.set()
    assert wait_until(lambda: planner._continuation_thread is None)
    assert bridge.nav_status == "cancelled"
    assert bridge.follow_path.call_count == 1
    assert client.call_async.call_count == 1


def test_index_refinement_wait_expires_without_restarting_objective(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
    )
    pending = rolling_home_response(partial=False, path_x=[])
    pending.status = RefineRouteResponse.STALE_REVISION
    pending.reason = "requested snapshot is not current"
    bridge, planner, client = rig(monkeypatch, initial, refine_responses=[])
    planner.refine_client.call_async.side_effect = lambda _: completed(pending)
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_deadline_s = 0.05
    planner.replan_backoff_s = 0.05

    assert planner.return_home()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert wait_until(lambda: bridge.nav_status == "failed")
    assert bridge.follow_path.call_count == 1
    assert client.call_async.call_count == 1
    assert planner.global_display_plan() is None


def test_blocked_home_refinement_replans_the_global_route(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
    )
    blocked = rolling_home_response(partial=False, path_x=[])
    blocked.status = RefineRouteResponse.BLOCKED
    blocked.reason = "retained corridor is locally blocked"
    replacement = rolling_home_response(
        partial=False,
        path_x=[-0.5, -1.0, -2.0],
        global_path=[-0.5, -1.0, -2.0],
    )
    bridge, planner, client = rig(
        monkeypatch,
        initial,
        refine_responses=[blocked],
    )
    client.call_async.side_effect = [completed(initial), completed(replacement)]
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_backoff_s = 0.0

    assert planner.return_home()
    original_global = planner.global_display_plan()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert planner.global_display_plan() is not original_global
    assert planner.global_display_plan().poses[-1].x == pytest.approx(-2.0)
    assert planner._blocked_retry_count == 0


def window_blocked_response():
    blocked = rolling_home_response(partial=False, path_x=[])
    blocked.status = RefineRouteResponse.BLOCKED
    blocked.reason = "robot is outside the completed objective route window"
    return blocked


def window_replan_rig(monkeypatch, pose_of_call):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -1.5, -2.0],
    )
    bridge, planner, client = rig(
        monkeypatch,
        initial,
        refine_responses=[window_blocked_response() for _ in range(4)],
    )
    client.call_async.side_effect = [completed(initial) for _ in range(5)]
    calls = []

    def state():
        calls.append(len(calls))
        return {"pose": pose_of_call(len(calls))}

    bridge.state.side_effect = state
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_backoff_s = 0.0
    assert planner.return_home()
    return bridge, planner, client


def complete_section(bridge, planner):
    assert wait_until(lambda: planner._continuation_thread is None)
    bridge.nav_status = "succeeded"
    planner._check_active_authority()


def test_route_window_replans_without_motion_end_the_objective(monkeypatch):
    bridge, planner, client = window_replan_rig(
        monkeypatch, lambda _call: {"x": 0.0, "y": 1.0, "yaw": 0.0}
    )

    for cycle in (1, 2):
        complete_section(bridge, planner)
        assert wait_until(lambda: bridge.follow_path.call_count == cycle + 1)
    complete_section(bridge, planner)

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert bridge.follow_path.call_count == 3
    assert client.call_async.call_count == 3
    assert "completed without motion 3 times in a row" in planner._nav_failure_reason
    assert planner._objective_kind is None


def test_route_window_replans_with_motion_keep_replanning(monkeypatch):
    bridge, planner, client = window_replan_rig(
        monkeypatch, lambda call: {"x": 0.5 * call, "y": 1.0, "yaw": 0.0}
    )

    for cycle in (1, 2, 3):
        complete_section(bridge, planner)
        assert wait_until(lambda: bridge.follow_path.call_count == cycle + 1)

    assert bridge.nav_status == "active"
    assert client.call_async.call_count == 4
    assert planner._nav_failure_reason is None
    assert planner._window_replans == 1


def test_indexed_home_refinement_rejects_evidence_downgrade(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
        indexed=True,
    )
    downgraded = rolling_home_response(
        partial=False, path_x=[-0.5, -2.0], indexed=False
    )
    bridge, planner, client = rig(
        monkeypatch,
        initial,
        refine_responses=[downgraded],
    )
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert planner.return_home()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert (
        "evidence source"
        in planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
    )
    assert client.call_async.call_count == 1
    assert bridge.follow_path.call_count == 1


def test_indexed_home_refinement_replans_after_snapshot_mismatch(monkeypatch):
    initial = rolling_home_response(
        partial=True,
        path_x=[0.0, -0.5],
        global_path=[0.0, -0.5, -1.0, -2.0],
        indexed=True,
    )
    mismatched = rolling_home_response(partial=False, path_x=[-0.5, -2.0], indexed=True)
    mismatched.geometry_revision = "b" * 64
    bridge, planner, client = rig(
        monkeypatch,
        initial,
        refine_responses=[mismatched],
    )
    planner.refine_client.call_async.side_effect = None
    planner.refine_client.call_async.return_value = completed(mismatched)
    planner.authority_reader = NS(current=lambda: correction_authority())
    planner.replan_backoff_s = 0.0

    assert planner.return_home()
    original_global = planner.global_display_plan()
    bridge.nav_status = "succeeded"
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert wait_until(lambda: bridge.follow_path.call_count == 2)
    assert planner.global_display_plan() is not original_global
    assert bridge.nav_status == "active"


def test_malformed_full_route_is_rejected_before_dispatch(monkeypatch):
    response = success_path()
    response.path = []
    bridge, planner, _ = rig(monkeypatch, response)

    assert not planner.navigate({"x": 2.0, "y": 1.0})
    bridge.follow_path.assert_not_called()
    assert "path is empty" in bridge.node.get_logger().warning.call_args.args[0]


def test_full_route_endpoint_must_match_requested_xy_but_may_project_height(
    monkeypatch,
):
    response = success_path()
    bridge, planner, client = rig(monkeypatch, response)
    client.call_async.side_effect = None
    ready = Future()
    response.path[-1] = pose(3.0, 1.0, 0.5)
    ready.set_result(response)
    client.call_async.return_value = ready
    assert not planner.navigate({"x": 2.0, "y": 1.0, "z": 0.0})
    bridge.follow_path.assert_not_called()
    assert "endpoint differs" in bridge.node.get_logger().warning.call_args.args[0]

    response.path[-1] = pose(2.0, 1.0, 0.5)
    bridge, planner, _ = rig(monkeypatch, response)
    assert planner.navigate({"x": 2.0, "y": 1.0, "z": 0.0})


@pytest.mark.parametrize("distance_m", [20.0, 50.0, 100.0])
def test_distant_full_route_keeps_exact_fixed_frame_endpoint(monkeypatch, distance_m):
    response = success_path()
    response.path = [pose(index * distance_m / 100.0, 0.0) for index in range(101)]
    bridge, planner, client = rig(monkeypatch, response)
    goal = {"x": distance_m, "y": 0.0, "yaw": 0.3}

    assert planner.navigate(goal)

    bridge.follow_path.assert_called_once()
    dispatched = bridge.follow_path.call_args.args[0]
    assert dispatched.frame_id == planner.frame
    assert len(dispatched.poses) == 101
    assert dispatched.poses[-1].x == pytest.approx(distance_m)
    assert client.call_async.call_args.args[0].goal.position.x == pytest.approx(
        distance_m
    )
    assert planner._active_route[3]["x"] == pytest.approx(distance_m)

    public = planner.decorate_state({"nav_status": "active", "goal": None})
    assert public["goal"]["x"] == pytest.approx(distance_m)
    assert dispatched.poses[-1].x == pytest.approx(distance_m)

    bridge.nav_status = "succeeded"
    planner._check_active_authority()
    assert planner._active_route is None
    assert client.call_async.call_count == 1


def test_no_progress_replans_original_goal_twice_then_stops(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.replan_backoff_s = 0.0
    original = {"x": 2.0, "y": 1.0, "yaw": 0.3}
    assert planner.navigate(original)

    for expected_requests in (2, 3):
        bridge.nav_status = "failed"
        bridge._nav_failure_reason = "FollowPath: Failed to make progress"
        planner._check_active_authority()
        assert wait_until(lambda: client.call_async.call_count == expected_requests)
        assert wait_until(lambda: planner._active_route is not None)
        request = client.call_async.call_args.args[0]
        assert (request.goal.position.x, request.goal.position.y) == (2.0, 1.0)
        assert planner._objective_goal == original

    bridge.nav_status = "failed"
    bridge._nav_failure_reason = "FollowPath: Failed to make progress"
    planner._check_active_authority()

    assert client.call_async.call_count == 3  # initial route plus two retries
    assert planner._blocked_retry_count == 3
    assert planner._active_route is None
    assert planner._objective_goal is None


def test_current_map_invalid_route_cancels_and_replans_without_blockage_count(
    monkeypatch,
):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    validation = Mock()
    validation.service_is_ready.return_value = True
    result = Future()
    validation.call_async.return_value = result
    planner.route_validation_service_type = ValidateRouteService
    planner.route_validation_client = validation
    planner.replan_backoff_s = 10.0
    assert planner.navigate({"x": 2.0, "y": 1.0})
    active = planner._active_route

    assert not planner._check_route_validation(active)
    request = validation.call_async.call_args.args[0]
    assert request.mission_id == "mission-1"
    assert request.component_id == "component-a"
    assert request.frame_id == planner.frame
    assert request.lookahead_m == 3.0
    assert len(request.path) == len(response.path)
    result.set_result(NS(status=ValidateRouteResponse.INVALID, reason="known curb"))
    assert planner._check_route_validation(active)

    bridge.cancel_goal_if_current.assert_called_once()
    assert planner._blocked_retry_count == 0
    assert planner._objective_goal is not None
    planner._check_active_authority()
    assert planner._objective_goal is not None
    bridge.cancel_goal()


@pytest.mark.parametrize(
    "status", [ValidateRouteResponse.VALID, ValidateRouteResponse.UNAVAILABLE]
)
def test_route_validation_nonhazard_status_never_cancels(monkeypatch, status):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    planner.authority_reader = NS(current=lambda: correction_authority())
    validation = Mock()
    validation.service_is_ready.return_value = True
    result = Future()
    validation.call_async.return_value = result
    planner.route_validation_service_type = ValidateRouteService
    planner.route_validation_client = validation
    assert planner.navigate({"x": 2.0, "y": 1.0})
    active = planner._active_route

    planner._check_route_validation(active)
    result.set_result(NS(status=status, reason="map unavailable"))
    assert not planner._check_route_validation(active)

    bridge.cancel_goal_if_current.assert_not_called()
    assert planner._active_route == active


def test_route_validation_timeout_retires_pending_request_without_canceling_route(
    monkeypatch,
):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    planner.authority_reader = NS(current=lambda: correction_authority())
    validation = Mock()
    validation.service_is_ready.return_value = True
    pending = Future()
    validation.call_async.return_value = pending
    planner.route_validation_service_type = ValidateRouteService
    planner.route_validation_client = validation
    assert planner.navigate({"x": 2.0, "y": 1.0})
    active = planner._active_route

    planner._check_route_validation(active)
    planner._validation_started_at = time.monotonic() - 1.0
    assert not planner._check_route_validation(active)

    assert pending.cancelled()
    validation.remove_pending_request.assert_called_once_with(pending)
    bridge.cancel_goal_if_current.assert_not_called()
    assert planner._active_route == active


def test_planner_rejection_does_not_consume_physical_blockage_budget(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.replan_backoff_s = 0.0
    rejected = NS(status=1, reason="destination is not observed")
    client.call_async.side_effect = [
        completed(success_path()),
        completed(rejected),
        completed(rejected),
        completed(rejected),
    ]
    assert planner.navigate({"x": 2.0, "y": 1.0})

    bridge.nav_status = "failed"
    bridge._nav_failure_reason = "Failed to make progress; error_code=4"
    planner._check_active_authority()

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 4
    assert planner._blocked_retry_count == 1
    assert "destination is not observed" in planner._nav_failure_reason


def test_material_correction_replans_same_navigate_objective(monkeypatch):
    first = success_path()
    first.path[-1] = pose(4.0, 1.0)
    first.geometry_revision = "a" * 64
    second = success_path()
    second.path[-1] = pose(4.0, 1.0)
    second.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, first)
    client.call_async.side_effect = [completed(first), completed(second)]
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    original = {
        "x": 4.0,
        "y": 1.0,
        "mission_id": "mission-1",
        "component_id": "component-a",
    }
    assert planner.navigate(original)
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=2, x=0.30)
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    request = client.call_async.call_args.args[0]
    assert (request.goal.position.x, request.goal.position.y) == (4.0, 1.0)
    assert bridge.nav_status == "active"


def test_execution_correction_deadband_accumulates_from_accepted_route(monkeypatch):
    first = success_path()
    first.path[-1] = pose(4.0, 1.0)
    first.geometry_revision = "a" * 64
    second = success_path()
    second.path[-1] = pose(4.0, 1.0)
    second.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, first)
    client.call_async.side_effect = [completed(first), completed(second)]
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 4.0, "y": 1.0})
    bridge.nav_status = "active"

    for revision, shift in ((2, 0.10), (3, 0.20), (4, 0.25)):
        authority[0] = correction_authority(revision=revision, x=shift)
        planner._check_active_authority()
        bridge.cancel_goal_if_current.assert_not_called()
    authority[0] = correction_authority(revision=5, x=0.251)
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    warning = bridge.node.get_logger().warning.call_args.args[0]
    assert "translation=0.251m" in warning
    assert "max_route_shift=0.251m" in warning


def test_execution_correction_accounts_for_rotation_at_far_endpoint(monkeypatch):
    response = success_path()
    response.path[-1] = pose(10.0, 0.0)
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    client.call_async.side_effect = [completed(response), completed(response)]
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 10.0, "y": 0.0})
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=2, yaw=0.03)
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert (
        "max_route_shift=0.300m" in bridge.node.get_logger().warning.call_args.args[0]
    )


def test_execution_retains_frame_rotation_bound_for_short_route(monkeypatch):
    response = success_path()
    response.path = [pose(0.0, 0.0), pose(0.01, 0.0)]
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    client.call_async.side_effect = [completed(response), completed(response)]
    planner.replan_backoff_s = 0.0
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 0.01, "y": 0.0})
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=2, yaw=0.021)
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    warning = bridge.node.get_logger().warning.call_args.args[0]
    assert "rotation=0.021rad" in warning
    assert "max_route_shift=0.000m" in warning


@pytest.mark.parametrize("objective", ["navigate", "return_home"])
def test_stale_authority_stops_motion_then_recovers_same_objective(
    monkeypatch, objective
):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.plan(objective, {"x": 2.0, "y": 1.0})
    bridge.nav_status = "active"
    original = deepcopy(client.call_async.call_args.args[0].goal)

    authority[0] = None
    planner._check_active_authority()

    bridge.cancel_goal_if_current.assert_called_once()
    assert bridge.nav_status == "active"  # Pending; the controller was canceled.
    time.sleep(0.35)  # Several missed reads must not consume the planning budget.
    assert client.call_async.call_count == 1
    authority[0] = correction_authority()
    assert wait_until(lambda: planner._active_route is not None)
    assert client.call_async.call_count == 2
    goal = client.call_async.call_args.args[0].goal
    assert goal.position == original.position


@pytest.mark.parametrize(
    "resolution", ["timeout", "new_mission", "new_component", "new_frame", "stop"]
)
def test_stale_authority_recovery_remains_bounded_and_identity_fenced(
    monkeypatch, resolution
):
    bridge, planner, client = rig(monkeypatch, success_path())
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    planner.replan_deadline_s = 0.4
    planner.replan_backoff_s = 0.05
    assert planner.navigate({"x": 2.0, "y": 1.0})
    authority[0] = None
    planner._check_active_authority()
    worker = planner._recovery_thread
    if resolution == "new_mission":
        authority[0] = correction_authority(mission_id="mission-2")
    elif resolution == "new_component":
        authority[0] = {**correction_authority(), "component_id": "component-b"}
    elif resolution == "new_frame":
        authority[0] = correction_authority(navigation_frame="robot_1/replacement_map")
    elif resolution == "stop":
        bridge.cancel_goal()
        authority[0] = correction_authority()
    worker.join(1.0)
    assert not worker.is_alive()
    assert client.call_async.call_count == 1
    assert bridge.nav_status == ("cancelled" if resolution == "stop" else "failed")
    if resolution == "timeout":
        assert (
            "no fresh map authority"
            in planner.decorate_state({"nav_status": "failed"})["nav_failure_reason"]
        )


def test_component_goal_is_reresolved_after_correction(monkeypatch):
    first = success_path()
    first.path[-1] = pose(4.0, 1.0)
    first.geometry_revision = "a" * 64
    second = success_path()
    second.path[-1] = pose(3.0, 1.0)
    second.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, first)
    client.call_async.side_effect = [completed(first), completed(second)]
    planner.replan_backoff_s = 0.0
    authority = [correction_authority(x=1.0)]
    planner.authority_reader = NS(current=lambda: authority[0])
    goal = {
        # Server resolution can already be stale when the adapter receives it.
        "x": 99.0,
        "y": 1.0,
        "z": 0.0,
        "yaw": 0.0,
        "frame_id": "robot_1/navigation_frame",
        "mission_id": "mission-1",
        "component_id": "component-a",
        "solution_order": [1, 0],
        "component_goal": {"x": 5.0, "y": 1.0, "z": 0.0, "yaw": 0.0},
    }
    assert planner.navigate(goal)
    initial_request = client.call_async.call_args.args[0]
    assert initial_request.goal.position.x == pytest.approx(4.0)
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=2, x=2.0)
    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    request = client.call_async.call_args.args[0]
    assert request.goal.position.x == pytest.approx(3.0)
    assert request.goal.position.y == pytest.approx(1.0)
    public = planner.decorate_state({"nav_status": "active", "goal": {"x": 3.0}})
    assert public["goal"]["x"] == pytest.approx(3.0)
    assert public["goal"]["component_goal"] == goal["component_goal"]
    assert public["objective_continuation"]["phase"] == "following_final"


def test_component_click_is_rejected_if_frame_changes_before_initial_admission(
    monkeypatch,
):
    bridge, planner, client = rig(monkeypatch, success_path())
    authority = [correction_authority(solution_order=(1, 0))]
    planner.authority_reader = NS(current=lambda: authority[0])
    goal = {
        "x": 4.0,
        "y": 1.0,
        "frame_id": "robot_1/navigation_frame",
        "mission_id": "mission-1",
        "component_id": "component-a",
        "solution_order": [1, 0],
        "component_goal": {"x": 5.0, "y": 1.0, "z": 0.0, "yaw": 0.0},
    }
    claim = planner.claim_objective("navigate", goal)
    authority[0] = correction_authority(revision=2, solution_order=(2, 0))

    assert not planner.execute_claimed(claim)
    client.call_async.assert_not_called()
    bridge.follow_path.assert_not_called()
    assert "stale frame revision" in bridge.node.get_logger().warning.call_args.args[0]


def test_component_click_without_frame_revision_token_is_rejected(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.authority_reader = NS(current=lambda: correction_authority())
    goal = {
        "x": 4.0,
        "y": 1.0,
        "frame_id": "robot_1/navigation_frame",
        "mission_id": "mission-1",
        "component_id": "component-a",
        "component_goal": {"x": 5.0, "y": 1.0, "z": 0.0, "yaw": 0.0},
    }

    assert not planner.navigate(goal)
    client.call_async.assert_not_called()
    bridge.follow_path.assert_not_called()
    assert (
        "no frame revision token" in bridge.node.get_logger().warning.call_args.args[0]
    )


def test_component_goal_requires_matching_explicit_authority(monkeypatch):
    bridge, planner, _ = rig(monkeypatch, success_path())
    planner.authority_reader = NS(current=lambda: correction_authority())
    goal = {
        "x": 1.0,
        "y": 2.0,
        "frame_id": "robot_1/navigation_frame",
        "mission_id": "mission-other",
        "component_id": "component-a",
        "component_goal": {"x": 1.0, "y": 2.0},
    }
    assert not planner.navigate(goal)
    bridge.follow_path.assert_not_called()
    assert "mission differs" in bridge.node.get_logger().warning.call_args.args[0]


def test_component_goal_inverse_resolves_position_and_heading(monkeypatch):
    bridge, planner, _ = rig(monkeypatch, success_path())
    authority = correction_authority()
    authority["T_component_navigation"] = [
        [0.0, -1.0, 0.0, 10.0],
        [1.0, 0.0, 0.0, 20.0],
        [0.0, 0.0, 1.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    goal = {"component_goal": {"x": 8.0, "y": 23.0, "z": 5.0, "yaw": math.pi / 2}}

    resolved = planner._component_goal_in_navigation(goal, authority)

    assert (resolved["x"], resolved["y"], resolved["z"]) == pytest.approx(
        (3.0, 2.0, 4.0)
    )
    assert resolved["yaw"] == pytest.approx(0.0)


def test_map_goal_is_resolved_once_into_stable_planning_frame(monkeypatch):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    bridge, planner, client = rig(monkeypatch, success_path())
    authority = stable_planning_authority(map_x=1.0)
    # Rotate map coordinates into the component while planning odometry stays
    # fixed. The direct UI goal has no frame field and therefore uses navigation_frame.
    authority["T_component_navigation"] = [
        [0.0, -1.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    planner.authority_reader = NS(current=lambda: authority)

    assert planner.navigate({"x": 2.0, "y": 0.0, "z": 0.4, "yaw": 0.0})

    request = client.call_async.call_args.args[0]
    assert request.goal.position.x == pytest.approx(1.0)
    assert request.goal.position.y == pytest.approx(2.0)
    assert request.goal.position.z == pytest.approx(0.4)
    assert request.goal.orientation.z == pytest.approx(math.sin(math.pi / 4))
    plan = bridge.follow_path.call_args.args[0]
    assert plan.frame_id == "robot_1/odom"
    assert plan.poses[-1].x == pytest.approx(1.0)
    assert plan.poses[-1].y == pytest.approx(2.0)


def test_stable_route_ignores_map_gauge_but_replans_planning_correction(
    monkeypatch,
):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.replan_backoff_s = 0.0
    authority = [stable_planning_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    goal = {
        "x": 4.0,
        "y": 1.0,
        "z": 0.4,
        "yaw": 0.0,
        "frame_id": "robot_1/navigation_frame",
        "mission_id": "mission-1",
        "component_id": "component-a",
        "solution_order": [1, 0],
        "component_goal": {"x": 4.0, "y": 1.0, "z": 0.4, "yaw": 0.0},
    }
    assert planner.navigate(goal)
    bridge.nav_status = "active"
    bridge.cancel_goal_if_current.reset_mock()

    # A 27 cm UI-map gauge correction leaves C<-planning unchanged, so the
    # accepted controller path and its exact planner-frame endpoint remain.
    map_only = stable_planning_authority(map_x=0.27, revision=2)
    map_only["solution_order"] = [2, 0]
    authority[0] = map_only
    planner._check_active_authority()
    assert client.call_async.call_count == 1
    bridge.cancel_goal_if_current.assert_not_called()
    public = planner.decorate_state({"nav_status": "active", "goal": None})
    assert public["goal"]["frame_id"] == "robot_1/odom"
    assert public["goal"]["x"] == pytest.approx(4.0)
    assert public["goal"]["component_goal"] == goal["component_goal"]

    # Moving C<-planning changes the physical route and retains the existing
    # cancel/replan lifecycle. Component provenance produces the corrected
    # exact endpoint in the same stable frame.
    corrected = stable_planning_authority(map_x=0.27, planning_x=0.27, revision=3)
    corrected["solution_order"] = [3, 0]
    authority[0] = corrected
    planner._check_active_authority()
    assert wait_until(lambda: client.call_async.call_count == 2)
    bridge.cancel_goal_if_current.assert_called_once()
    request = client.call_async.call_args.args[0]
    assert request.goal.position.x == pytest.approx(3.73)
    assert request.goal.position.y == pytest.approx(1.0)


def test_stable_planning_frame_requires_qualified_authority(monkeypatch):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert not planner.navigate({"x": 2.0, "y": 0.0})
    client.call_async.assert_not_called()
    assert "no qualified frame transform" in (
        bridge.node.get_logger().warning.call_args.args[0]
    )


def planning_map_unavailable():
    response = success_path()
    response.status = Response.BLOCKED
    response.reason = "odometry or planning map is unavailable"
    response.path = []
    return response


def test_unavailable_planning_map_waits_instead_of_spending_attempts(monkeypatch):
    bridge, planner, client = rig(monkeypatch, planning_map_unavailable())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 5.0
    planner.replan_max_attempts = 2
    monkeypatch.setattr(objective_planning, "PLANNER_INPUT_RETRY_S", 0.0)
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    # More refusals than the attempt budget: the map returns on the sixth call.
    client.call_async.side_effect = [
        completed(planning_map_unavailable()) for _ in range(5)
    ] + [completed(success_path())]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.follow_path.call_count == 1)
    assert client.call_async.call_count == 6
    assert bridge.nav_status == "active"
    assert planner._blocked_retry_count == 0


def test_planning_map_that_never_returns_fails_at_the_deadline(monkeypatch):
    bridge, planner, client = rig(monkeypatch, planning_map_unavailable())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.3
    planner.replan_max_attempts = 2
    monkeypatch.setattr(objective_planning, "PLANNER_INPUT_RETRY_S", 0.02)
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    client.call_async.side_effect = lambda *_: completed(planning_map_unavailable())

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.nav_status == "failed")
    assert bridge.follow_path.call_count == 0


def indexed_map_failed_closed():
    response = success_path()
    response.status = Response.STALE_REVISION
    response.reason = "a different snapshot failed indexed publication"
    response.path = []
    return response


def test_indexed_map_that_failed_closed_is_waited_out(monkeypatch):
    bridge, planner, client = rig(monkeypatch, indexed_map_failed_closed())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 5.0
    planner.replan_max_attempts = 2
    monkeypatch.setattr(objective_planning, "PLANNER_INPUT_RETRY_S", 0.0)
    authority = correction_authority()
    planner.authority_reader = NS(current=lambda: authority)
    client.call_async.side_effect = [
        completed(indexed_map_failed_closed()) for _ in range(4)
    ] + [completed(success_path())]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.follow_path.call_count == 1)
    assert client.call_async.call_count == 5
    assert bridge.nav_status == "active"


def index_query_over_budget():
    response = success_path()
    response.status = Response.BLOCKED
    response.reason = "query time budget exceeded"
    response.path = []
    return response


def test_index_query_budget_is_a_missing_answer_not_a_blocked_route(monkeypatch):
    # The index server gave up one validation under load (benchbot,
    # 2026-09-19: a 31 m return leg failed at once). The next answer is in
    # time and the objective proceeds.
    bridge, planner, client = rig(monkeypatch, index_query_over_budget())
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 5.0
    planner.replan_max_attempts = 2
    monkeypatch.setattr(objective_planning, "PLANNER_INPUT_RETRY_S", 0.0)
    planner.authority_reader = NS(current=lambda: correction_authority())
    client.call_async.side_effect = [
        completed(index_query_over_budget()) for _ in range(3)
    ] + [completed(success_path())]

    assert planner.navigate({"x": 2, "y": 1})
    assert wait_until(lambda: bridge.follow_path.call_count == 1)
    assert client.call_async.call_count == 4
    assert bridge.nav_status == "active"


def test_other_stale_revisions_still_fail(monkeypatch):
    response = indexed_map_failed_closed()
    response.reason = "component-frame objective uses a stale frame revision"
    bridge, planner, client = rig(monkeypatch, response)
    planner.authority_reader = NS(current=lambda: correction_authority())

    assert not planner.navigate({"x": 2, "y": 1})
    assert client.call_async.call_count == 1
    assert bridge.follow_path.call_count == 0


def test_recovery_pauses_are_bounded_and_never_negative():
    """A planning call that outlives the deadline leaves a negative remainder;
    the paced retry clamps it rather than raising from time.sleep."""
    assert objective_planning.bounded_pause(0.5, 3.0) == 0.5
    assert objective_planning.bounded_pause(0.5, 0.2) == pytest.approx(0.2)
    assert objective_planning.bounded_pause(0.5, 0.0) == 0.0
    assert objective_planning.bounded_pause(0.5, -0.2) == 0.0
