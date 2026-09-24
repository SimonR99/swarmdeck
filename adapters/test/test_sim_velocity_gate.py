"""Simulated Nav2 output must pass the same ownership gate as hardware."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from test_adapter_sim_goals import _bridge, _plan, _ImmediateFuture


def _twist():
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0)
    )


def _capture(bridge):
    messages = []
    bridge.pub_cmd = SimpleNamespace(publish=messages.append)
    return messages


def _accept(bridge, generation):
    handle = MagicMock(accepted=True)
    bridge._goal_response(_ImmediateFuture(handle), generation)
    return handle


def test_sim_velocity_waits_for_acceptance_and_stops_on_terminal(sim_module):
    bridge = _bridge(sim_module)
    messages = _capture(bridge)
    command = _twist()
    command.linear.x = 0.3
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        generation = bridge.follow_path(_plan(), expected_generation=0)
    bridge._on_nav_cmd_vel(command)
    assert messages == []

    handle = _accept(bridge, generation)
    bridge._on_nav_cmd_vel(command)
    assert messages == [command]
    outcome = SimpleNamespace(status=sim_module.GoalStatus.STATUS_SUCCEEDED)
    bridge._goal_result(_ImmediateFuture(outcome), generation, handle)
    messages.clear()
    bridge._on_nav_cmd_vel(command)
    assert messages == []


@pytest.mark.parametrize("operation", ["cancel_goal", "stop", "drive", "pending"])
def test_sim_preemption_holds_nav_velocity_but_preserves_manual_drive(
    sim_module, operation
):
    bridge = _bridge(sim_module)
    bridge.nav_status = "active"
    _accept(bridge, 0)
    messages = _capture(bridge)
    command = _twist()
    command.linear.x = 0.3
    bridge._on_nav_cmd_vel(command)
    assert messages == [command]
    messages.clear()

    with patch.object(sim_module, "Twist", side_effect=_twist):
        if operation == "drive":
            bridge.drive(0.2, -0.1)
        elif operation == "pending":
            bridge.set_goal_pending_if_current(0)
        else:
            getattr(bridge, operation)()
    if operation == "drive":
        assert messages[-1].linear.x == 0.2
        assert messages[-1].angular.z == -0.1
        assert bridge.mode == "teleop"
    elif operation != "pending":
        assert messages[-1].linear.x == 0.0
    messages.clear()
    bridge._on_nav_cmd_vel(command)
    assert messages == []


@pytest.mark.parametrize("blocked", ["link", "route", "inactive"])
def test_sim_velocity_gate_honours_shared_link_route_and_status_checks(
    sim_module, blocked
):
    bridge = _bridge(sim_module)
    bridge.nav_status = "active"
    _accept(bridge, 0)
    messages = _capture(bridge)
    if blocked == "link":
        bridge._last_link_at = 0.0
    elif blocked == "route":
        bridge._nav_route_blocked = True
    else:
        bridge.nav_status = "idle"
    bridge._on_nav_cmd_vel(_twist())
    assert messages == []


def test_sim_late_acceptance_cannot_reopen_velocity_gate(sim_module):
    bridge = _bridge(sim_module)
    bridge.cancel_goal()
    _accept(bridge, 0)
    bridge.nav_status = "active"  # A replacement objective is still planning.
    messages = _capture(bridge)
    bridge._on_nav_cmd_vel(_twist())
    assert messages == []


def test_sim_result_monitor_failure_keeps_velocity_gate_closed(sim_module):
    bridge = _bridge(sim_module)
    bridge.nav_status = "active"
    handle = MagicMock(accepted=True)
    handle.get_result_async.side_effect = RuntimeError("result channel lost")
    bridge._goal_response(_ImmediateFuture(handle), 0)
    messages = _capture(bridge)
    bridge._on_nav_cmd_vel(_twist())
    assert messages == []
