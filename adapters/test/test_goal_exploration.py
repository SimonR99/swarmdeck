"""Exploring toward a goal with no known route, without ROS."""

import sys
import threading
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

from adapters.goal_exploration import GoalExploration, is_no_known_route


class TargetRequest:
    def __init__(self):
        self.active = False
        self.target = NS(x=0.0, y=0.0, z=0.0)


class Exploration:
    def __init__(self):
        self.active = False
        self.status = "idle"
        self.reason = None
        self.stopped = 0

    def start(self):
        self.active = True
        self.status = "exploring"

    def stop(self, **_):
        self.active = False
        self.status = "stopped"
        self.stopped += 1


def rig(monkeypatch, probe_outcome="failed"):
    package = ModuleType("mgg_msgs")
    services = ModuleType("mgg_msgs.srv")
    services.PlannerSetExplorationTarget = NS(Request=TargetRequest)
    monkeypatch.setitem(sys.modules, "mgg_msgs", package)
    monkeypatch.setitem(sys.modules, "mgg_msgs.srv", services)
    targets = []
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda request: targets.append(
        (request.active, request.target.x, request.target.y)
    )
    bridge = Mock()
    bridge.id = "robot_1"
    bridge._goal_generation = 9
    bridge._goal_lock = threading.RLock()
    bridge.node.create_client.return_value = client
    planner = Mock()
    planner._call_once.return_value = (probe_outcome, "", None)
    planner.claim_objective.return_value = "claim"
    exploration = Exploration()
    explorer = GoalExploration(
        bridge, planner, exploration, {"explore_to_goal_probe_period_s": 0.5}
    )
    explorer._next_probe = 0.0
    return explorer, planner, exploration, targets


def settle(explorer):
    probe = explorer._probe
    if probe is not None:
        probe.join(2.0)


def test_only_map_coverage_refusals_count_as_no_known_route():
    assert is_no_known_route("goal cannot be linked to the global graph; ...")
    assert is_no_known_route("no mapped ground under the goal")
    assert is_no_known_route("no route over the global graph reaches the goal")
    assert not is_no_known_route("odometry is stale")
    assert not is_no_known_route(None)


def test_explores_toward_the_goal_then_navigates_once_a_route_exists(monkeypatch):
    explorer, planner, exploration, targets = rig(monkeypatch, "ready")
    requested = {"x": 40.0, "y": 3.0}
    assert explorer.begin(requested, {"x": 38.0, "y": 2.0})
    assert exploration.active
    assert targets == [(True, 38.0, 2.0)]
    assert explorer.display_goal == requested
    explorer._next_probe = 0.0
    explorer.tick()
    settle(explorer)
    # Probed without a goal generation, so exploration's own paths cannot
    # supersede it.
    assert planner._call_once.call_args.args == (
        "navigate",
        {"x": 38.0, "y": 2.0},
        None,
    )
    explorer.tick()
    assert not exploration.active and exploration.stopped == 1
    assert targets[-1][0] is False
    planner.claim_objective.assert_called_once_with("navigate", requested)
    assert explorer.display_goal is None


def test_exploration_running_out_fails_the_goal_with_that_reason(monkeypatch):
    explorer, planner, exploration, targets = rig(monkeypatch, "failed")
    explorer.begin({"x": 40.0, "y": 3.0}, {"x": 38.0, "y": 2.0})
    exploration.active = False
    exploration.status = "complete"
    exploration.reason = "no frontier left"
    explorer.tick()
    message, generation = planner._fail_if_current.call_args.args
    assert "no known route to the goal" in message
    assert "complete: no frontier left" in message
    assert generation == 9
    assert targets[-1][0] is False and not explorer.active


def test_operator_command_ends_it_quietly(monkeypatch):
    explorer, planner, exploration, targets = rig(monkeypatch, "failed")
    explorer.begin({"x": 40.0, "y": 3.0}, {"x": 38.0, "y": 2.0})
    explorer.cancel()
    assert not explorer.active and targets[-1][0] is False
    explorer.tick()
    planner._fail_if_current.assert_not_called()
    planner.claim_objective.assert_not_called()


def test_no_handoff_when_exploration_cannot_start(monkeypatch):
    explorer, _planner, exploration, targets = rig(monkeypatch)
    exploration.start = lambda: None
    assert not explorer.begin({"x": 40.0, "y": 3.0}, {"x": 38.0, "y": 2.0})
    assert not explorer.active and targets[-1][0] is False
