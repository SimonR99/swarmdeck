"""Whole-route MGG objective planning boundary without a ROS installation."""

from concurrent.futures import Future
from copy import deepcopy
import math
import sys
import threading
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from adapters.objective_planning import MggObjectivePlanning


class Request:
    NAVIGATE = 1
    RETURN_HOME = 2

    def __init__(self):
        self.goal = pose(0.0, 0.0)
        self.mission_id = ""
        self.objective = 0
        self.component_id = ""
        self.map_epoch = 0
        self.mapping_graph_revision = 0
        self.geometry_revision = ""
        self.map_source_stamp = NS(sec=0, nanosec=0)


class Response:
    SUCCEEDED = 0
    UNREACHABLE = 1
    STALE_REVISION = 2
    UNSUPPORTED_OBJECTIVE = 3
    BLOCKED = 4


class Service:
    Request = Request
    Response = Response


def pose(x, y, z=0.0, yaw=0.0):
    return NS(
        position=NS(x=x, y=y, z=z),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    )


def authority(revision=3, component="component-a"):
    return {
        "mission_id": "mission-1",
        "component_id": component,
        "navigation_frame": "robot_1/navigation_frame",
        "map_epoch": 7,
        "mapping_graph_revision": revision,
        "geometry_revision": "a" * 64,
        "map_source_stamp": {"sec": 4, "nanosec": 5},
        "T_component_navigation": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }


def response_for(request, path=None, status=Response.SUCCEEDED, reason=""):
    path = path or [pose(0, 0), pose(request.goal.position.x, request.goal.position.y)]
    return NS(
        status=status,
        component_id=request.component_id,
        map_epoch=request.map_epoch,
        mapping_graph_revision=request.mapping_graph_revision,
        geometry_revision=request.geometry_revision,
        map_source_stamp=deepcopy(request.map_source_stamp),
        path=path,
        reason=reason,
    )


def completed(value):
    future = Future()
    future.set_result(value)
    return future


def rig(monkeypatch, responder):
    package = ModuleType("mgg_msgs")
    services = ModuleType("mgg_msgs.srv")
    services.PlanObjective = Service
    monkeypatch.setitem(sys.modules, "mgg_msgs", package)
    monkeypatch.setitem(sys.modules, "mgg_msgs.srv", services)

    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.side_effect = lambda request: completed(responder(request))
    bridge = Mock()
    bridge.id = "robot_1"
    bridge.navigation_frame = "robot_1/navigation_frame"
    bridge.cfg = {}
    bridge.map_pose.return_value = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    bridge._mapping_authority = NS(current=lambda: authority())
    bridge._goal_generation = 4
    bridge._goal_lock = threading.RLock()
    bridge.node.create_client.return_value = client
    bridge.node.get_clock().now().to_msg.return_value = NS(sec=12, nanosec=34)
    bridge.node.create_timer.return_value = Mock()

    def cancel_goal():
        with bridge._goal_lock:
            bridge._goal_generation += 1
            bridge.nav_status = "cancelled"
            return bridge._goal_generation

    def set_status(expected, status):
        with bridge._goal_lock:
            if expected != bridge._goal_generation:
                return False
            bridge.nav_status = status
            return True

    def follow_path(plan, *, expected_generation=None):
        with bridge._goal_lock:
            if (
                expected_generation is not None
                and expected_generation != bridge._goal_generation
            ):
                return None
            bridge._goal_generation += 1
            bridge.nav_status = "active"
            return bridge._goal_generation

    bridge.cancel_goal.side_effect = cancel_goal
    bridge.set_goal_pending_if_current.side_effect = lambda generation: set_status(
        generation, "active"
    )
    bridge.set_nav_status_if_current.side_effect = set_status
    bridge.follow_path.side_effect = follow_path
    planner = MggObjectivePlanning(
        bridge,
        {
            "component_id": "component-a",
            "objective_timeout_s": 0.1,
            "authority_replan_backoff_s": 0.0,
            "controller_replan_backoff_s": 0.0,
        },
    )
    return bridge, planner, client


def test_one_request_carries_authority_and_submits_whole_path(monkeypatch):
    seen = []

    def respond(request):
        seen.append(request)
        return response_for(request, [pose(0, 0), pose(1, 0.5), pose(2, 1)])

    bridge, planner, client = rig(monkeypatch, respond)
    assert planner.navigate({"x": 2, "y": 1, "yaw": 0.3})
    assert len(seen) == 1
    request = seen[0]
    assert request.objective == Request.NAVIGATE
    assert request.mission_id == "mission-1"
    assert request.component_id == "component-a"
    assert request.map_epoch == 7
    assert request.mapping_graph_revision == 3
    assert request.geometry_revision == "a" * 64
    assert request.map_source_stamp.sec == 4
    assert request.goal.position.x == 2
    assert request.goal.orientation.z == pytest.approx(math.sin(0.15))
    plan = bridge.follow_path.call_args.args[0]
    assert plan.revision_ns == 12_000_000_034
    assert [(p.x, p.y, p.z) for p in plan.poses] == [
        (0, 0, 0.0),
        (1, 0.5, 0.0),
        (2, 1, 0.0),
    ]
    assert client.call_async.call_count == 1


def test_return_home_uses_authority_home_and_request_kind(monkeypatch):
    current = authority()
    current["home"] = {
        "keyframe_id": "home",
        "T_navigation_home": [
            [1.0, 0.0, 0.0, -3.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.4],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }
    seen = []
    bridge, planner, _ = rig(
        monkeypatch, lambda request: (seen.append(request) or response_for(request))
    )
    planner.authority_reader = NS(current=lambda: current)
    assert planner.return_home()
    assert seen[0].objective == Request.RETURN_HOME
    assert seen[0].goal.position.x == -3.0
    assert seen[0].goal.position.y == 2.0


def test_stale_identity_replans_with_latest_authority(monkeypatch):
    calls = []
    current = [authority(revision=3)]

    def respond(request):
        calls.append(request)
        if len(calls) == 1:
            current[0] = authority(revision=4)
            stale = response_for(request)
            stale.status = Response.STALE_REVISION
            return stale
        return response_for(request)

    bridge, planner, client = rig(monkeypatch, respond)
    planner.authority_reader = NS(current=lambda: current[0])
    assert planner.navigate({"x": 1, "y": 1})
    assert client.call_async.call_count == 2
    assert calls[0].mapping_graph_revision == 3
    assert calls[1].mapping_graph_revision == 4
    bridge.follow_path.assert_called_once()


def test_identity_mismatch_is_bounded_and_published(monkeypatch):
    def respond(request):
        result = response_for(request)
        result.component_id = "other-component"
        return result

    bridge, planner, client = rig(monkeypatch, respond)
    planner.authority_replan_max_attempts = 2
    assert not planner.navigate({"x": 1, "y": 1})
    assert client.call_async.call_count == 3
    assert bridge.nav_status == "failed"
    state = planner.decorate_state({"nav_status": "failed"})
    assert "stale" in state["nav_failure_reason"]
    bridge.follow_path.assert_not_called()


def test_controller_success_keeps_arrival_and_clears_reason(monkeypatch):
    bridge, planner, _ = rig(monkeypatch, lambda request: response_for(request))
    assert planner.navigate({"x": 1, "y": 1})
    bridge.nav_status = "succeeded"
    planner.tick()
    state = planner.decorate_state({"nav_status": "succeeded", "goal": None})
    assert state["nav_status"] == "succeeded"
    assert "nav_failure_reason" not in state
    assert planner.global_display_plan() is not None


def test_nonprogress_controller_failure_replans_whole_path(monkeypatch):
    calls = []

    def respond(request):
        calls.append(request)
        return response_for(request)

    bridge, planner, _ = rig(monkeypatch, respond)
    planner.controller_replan_backoff_s = 0.0
    planner.controller_replan_deadline_s = 0.5
    planner.controller_replan_max_attempts = 1
    assert planner.navigate({"x": 1, "y": 1})
    bridge._nav_failure_reason = "failed to make progress along the route"
    bridge.nav_status = "failed"
    planner.tick()
    planner._recovery_thread.join(timeout=1.0)
    assert len(calls) == 2
    assert bridge.follow_path.call_count == 2


def test_other_controller_failure_is_blocked_without_replan(monkeypatch):
    bridge, planner, client = rig(
        monkeypatch,
        lambda request: response_for(request),
    )
    assert planner.navigate({"x": 1, "y": 1})
    bridge._nav_failure_reason = "controller rejected malformed trajectory"
    bridge.nav_status = "failed"
    planner.tick()
    assert client.call_async.call_count == 1
    assert bridge.nav_status == "failed"
    state = planner.decorate_state({"nav_status": "failed"})
    assert state["nav_failure_reason"] == "controller rejected malformed trajectory"


def test_manual_generation_change_owns_planning(monkeypatch):
    pending = Future()
    bridge, planner, client = rig(monkeypatch, lambda request: pending)
    claim = planner.claim_objective("navigate", {"x": 1, "y": 1})
    bridge.cancel_goal()
    pending.set_result(response_for(Request()))
    assert not planner.execute_claimed(claim)
    assert client.call_async.call_count == 0
    bridge.follow_path.assert_not_called()


def test_global_display_plan_is_whole_route(monkeypatch):
    whole = [pose(0, 0), pose(1, 0), pose(2, 2)]
    bridge, planner, _ = rig(monkeypatch, lambda request: response_for(request, whole))
    assert planner.navigate({"x": 2, "y": 2})
    displayed = planner.global_display_plan()
    assert displayed is not None
    assert len(displayed.poses) == 3
    assert displayed.poses[-1].x == 2


NO_ROUTE = (
    "goal cannot be linked to the global graph; goal lattice: 1 vertices in 1 "
    "sweep(s), 1 reached from the goal, 0 bridge checks, none onto the "
    "robot's roadmap"
)


def test_goal_without_a_known_route_is_explored_toward_when_asked(monkeypatch):
    bridge, planner, _client = rig(
        monkeypatch,
        lambda request: response_for(
            request, status=Response.UNREACHABLE, reason=NO_ROUTE
        ),
    )
    bridge.goal_exploration = Mock()
    bridge.goal_exploration.begin.return_value = True
    claim = planner.claim_objective(
        "navigate", {"x": 40.0, "y": 3.0}, explore_if_unknown=True
    )
    assert claim.explore_if_unknown
    assert planner.execute_claimed(claim) is False
    requested, in_frame = bridge.goal_exploration.begin.call_args.args
    assert requested == {"x": 40.0, "y": 3.0}
    assert (in_frame["x"], in_frame["y"]) == (40.0, 3.0)
    assert bridge.nav_status != "failed"
    assert planner._nav_failure_reason is None


@pytest.mark.parametrize(
    "explore, reason",
    [(False, NO_ROUTE), (True, "odometry is stale")],
)
def test_other_refusals_still_fail_the_goal(monkeypatch, explore, reason):
    bridge, planner, _client = rig(
        monkeypatch,
        lambda request: response_for(
            request, status=Response.UNREACHABLE, reason=reason
        ),
    )
    bridge.goal_exploration = Mock()
    claim = planner.claim_objective(
        "navigate", {"x": 40.0, "y": 3.0}, explore_if_unknown=explore
    )
    assert planner.execute_claimed(claim) is False
    bridge.goal_exploration.begin.assert_not_called()
    assert bridge.nav_status == "failed"
    assert planner._nav_failure_reason == reason


def test_route_probe_is_not_superseded_by_exploration_goals(monkeypatch):
    bridge, planner, _client = rig(monkeypatch, response_for)
    outcome, _reason, plan = planner._call_once("navigate", {"x": 2.0, "y": 0.0}, None)
    assert outcome == "ready" and plan is not None
    bridge.follow_path.assert_not_called()


def test_a_momentarily_unavailable_map_is_waited_out(monkeypatch):
    answers = []

    def respond(request):
        answers.append(request)
        if len(answers) < 3:
            return response_for(
                request, status=Response.BLOCKED, reason="map unavailable"
            )
        return response_for(request)

    bridge, planner, _client = rig(monkeypatch, respond)
    planner.blocked_retry_s = 0.01
    assert planner.navigate({"x": 2.0, "y": 1.0})
    assert len(answers) == 3
    assert bridge.nav_status == "active"


def test_a_planner_that_stays_unavailable_fails_with_its_reason(monkeypatch):
    bridge, planner, _client = rig(
        monkeypatch,
        lambda request: response_for(
            request, status=Response.BLOCKED, reason="map unavailable"
        ),
    )
    planner.blocked_retry_s = 0.01
    assert not planner.navigate({"x": 2.0, "y": 1.0})
    assert bridge.nav_status == "failed"
    assert planner._nav_failure_reason == "map unavailable"


def test_the_waypoint_explored_toward_is_shown_as_the_goal(monkeypatch):
    bridge, planner, _client = rig(monkeypatch, response_for)
    bridge.goal_exploration = NS(display_goal={"x": 40.0, "y": 3.0})
    state = planner.decorate_state({"nav_status": "active", "goal": {"x": 1.0}})
    assert state["goal"] == {"x": 40.0, "y": 3.0}
    bridge.goal_exploration = NS(display_goal=None)
    state = planner.decorate_state({"nav_status": "active", "goal": {"x": 1.0}})
    assert state["goal"] == {"x": 1.0}
