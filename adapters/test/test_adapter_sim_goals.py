"""Generation and readiness races at the simulation navigation boundary."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest


def _bridge(sim_module):
    """Build only the state used by RobotBridge's navigation methods.

    RobotBridge.__init__ starts ROS clients and timers, so these tests exercise
    the ownership protocol through a deliberately small, lock-complete bridge.
    """
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "r0"
    bridge.map_frame = "r0/map_frame"
    bridge.node = MagicMock()
    bridge.pub_cmd = MagicMock()
    bridge.goal = None
    bridge.planned_path = []
    bridge.nav_status = "idle"
    bridge.mode = "idle"
    bridge._goal_generation = 0
    bridge._goal_handle = None
    bridge._goal_request_future = None
    bridge._goal_request_generation = None
    bridge._cancel_events = {}
    bridge._nav_quiet_unknown = False
    bridge._goal_lock = threading.RLock()
    bridge.path_client = MagicMock()
    bridge.path_client.server_is_ready.return_value = True
    bridge._clear_escape = MagicMock()
    bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    bridge._escape_from = None
    bridge._escape_started_at = 0.0
    bridge._escape_progress_at = 0.0
    bridge.exploration = None
    return bridge


def _plan(frame="r0/map_frame"):
    from adapters.exploration import PlannerPath, PlannerPose

    return PlannerPath(
        frame,
        1,
        (
            PlannerPose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            PlannerPose(2.0, 2.5, 0.0, 0.0, 0.0, 0.1, 0.995),
        ),
    )


class _ImmediateFuture:
    def __init__(self, value):
        self._value = value

    def result(self):
        if isinstance(self._value, BaseException):
            raise self._value
        return self._value

    def add_done_callback(self, callback):
        callback(self)


def test_conditional_follow_path_submission_failure_keeps_reserved_generation(
    sim_module,
):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 7
    bridge.path_client.send_goal_async.side_effect = RuntimeError("send failed")

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_plan(), expected_generation=7)

    assert result is False
    assert bridge._goal_generation == 7
    assert bridge.nav_status == "idle"
    assert bridge.mode == "idle"
    assert bridge.goal is None
    assert bridge.planned_path == []


def test_immediate_follow_path_rejection_leaves_terminal_failure(sim_module):
    bridge = _bridge(sim_module)
    rejected = MagicMock()
    rejected.accepted = False
    bridge.path_client.send_goal_async.return_value = _ImmediateFuture(rejected)

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_plan())

    assert type(result) is int
    assert result == bridge._goal_generation
    assert bridge.nav_status == "failed"
    assert bridge.mode == "recover"
    assert bridge.goal is None
    assert bridge.planned_path == []


def _active_goal_cancel_state(sim_module):
    """Return a live goal plus callbacks needed to drive cancellation races."""
    bridge = _bridge(sim_module)
    bridge._goal_generation = 5
    handle = MagicMock()
    handle.accepted = True
    result_future = MagicMock()
    result_callbacks = []
    result_future.add_done_callback.side_effect = result_callbacks.append
    handle.get_result_async.return_value = result_future
    response_future = MagicMock()
    response_future.result.return_value = handle

    bridge._goal_response(response_future, generation=5)
    cancel_future = MagicMock()
    cancel_callbacks = []
    cancel_future.add_done_callback.side_effect = cancel_callbacks.append
    handle.cancel_goal_async.return_value = cancel_future
    bridge._cancel_nav()
    quiet = bridge._cancel_events[5]
    assert len(result_callbacks) == 1
    return bridge, quiet, cancel_future, cancel_callbacks, result_callbacks[0]


def test_sim_cancel_response_completion_alone_does_not_make_route_quiet(sim_module):
    bridge, quiet, cancel_future, cancel_callbacks, result_done = (
        _active_goal_cancel_state(sim_module)
    )
    cancel_future.result.return_value = SimpleNamespace(return_code=0)
    for callback in cancel_callbacks:
        callback(cancel_future)

    assert not quiet.is_set()
    assert bridge.wait_goal_quiet(6, time.monotonic()) is False

    old_result = MagicMock()
    old_result.result.return_value = SimpleNamespace(
        status=sim_module.GoalStatus.STATUS_CANCELED
    )
    result_done(old_result)

    assert quiet.is_set()
    assert bridge.wait_goal_quiet(6, time.monotonic() + 1.0) is True


@pytest.mark.parametrize("cancel_failure", ["rejected", "exception"])
def test_sim_failed_cancel_response_does_not_make_route_quiet(
    sim_module, cancel_failure
):
    bridge, quiet, cancel_future, cancel_callbacks, _result_done = (
        _active_goal_cancel_state(sim_module)
    )
    if cancel_failure == "rejected":
        cancel_future.result.return_value = SimpleNamespace(return_code=1)
    else:
        cancel_future.result.side_effect = RuntimeError("cancel failed")
    for callback in cancel_callbacks:
        callback(cancel_future)

    assert not quiet.is_set()
    assert bridge.wait_goal_quiet(6, time.monotonic()) is False


def test_sim_late_accepted_goal_is_canceled_and_waits_for_terminal_result(sim_module):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 5
    bridge._goal_request_future = MagicMock()
    bridge._goal_request_generation = 5
    late_handle = MagicMock()
    late_handle.accepted = True
    cancel_future = MagicMock()
    cancel_callbacks = []
    cancel_future.add_done_callback.side_effect = cancel_callbacks.append
    late_handle.cancel_goal_async.return_value = cancel_future
    terminal_future = MagicMock()
    terminal_callbacks = []
    terminal_future.add_done_callback.side_effect = terminal_callbacks.append
    late_handle.get_result_async.return_value = terminal_future
    response_future = MagicMock()
    response_future.result.return_value = late_handle

    bridge._cancel_nav()
    assert bridge._goal_generation == 6
    quiet = bridge._cancel_events[5]
    bridge._goal_response(response_future, generation=5)

    late_handle.cancel_goal_async.assert_called_once_with()
    late_handle.get_result_async.assert_called_once_with()
    for callback in cancel_callbacks:
        callback(cancel_future)
    assert len(terminal_callbacks) == 1
    assert not quiet.is_set()
    assert bridge.wait_goal_quiet(6, time.monotonic()) is False

    terminal = MagicMock()
    terminal.result.return_value = SimpleNamespace(
        status=sim_module.GoalStatus.STATUS_CANCELED
    )
    terminal_callbacks[0](terminal)
    assert quiet.is_set()
    assert bridge.wait_goal_quiet(6, time.monotonic() + 1.0) is True


def test_unmonitored_accepted_goal_latches_unknown_without_escape(sim_module):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 5
    bridge._arm_escape = MagicMock()
    handle = MagicMock()
    handle.accepted = True
    handle.get_result_async.side_effect = RuntimeError("result channel lost")
    response = MagicMock()
    response.result.return_value = handle

    bridge._goal_response(response, generation=5)

    assert bridge._nav_quiet_unknown is True
    handle.cancel_goal_async.assert_called_once_with()
    bridge._arm_escape.assert_not_called()
    assert bridge.nav_status == "failed"


def test_sim_double_cancel_waits_for_the_original_terminal_result(sim_module):
    """A newer cancel must not hide an older action still settling in Nav2."""
    bridge, original_quiet, _cancel_future, _cancel_callbacks, result_done = (
        _active_goal_cancel_state(sim_module)
    )

    # First cancel owns generation 5 and remains unresolved. A second cancel
    # creates generation 7 and has no handle of its own, so its event is quiet
    # immediately; wait_goal_quiet must still account for generation 5.
    bridge._cancel_nav()
    assert bridge._goal_generation == 7
    assert not original_quiet.is_set()
    assert bridge.wait_goal_quiet(7, time.monotonic()) is False

    old_result = MagicMock()
    old_result.result.return_value = SimpleNamespace(
        status=sim_module.GoalStatus.STATUS_CANCELED
    )
    result_done(old_result)

    assert original_quiet.is_set()
    assert bridge.wait_goal_quiet(7, time.monotonic() + 1.0) is True


def test_stale_conditional_follow_path_does_not_preempt_newer_goal(sim_module):
    bridge = _bridge(sim_module)
    newer_handle = MagicMock()
    bridge._goal_generation = 9
    bridge._goal_handle = newer_handle
    bridge.goal = {"x": 8.0, "y": 9.0, "yaw": 0.2}
    bridge.planned_path = [{"x": 8.0, "y": 9.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        result = bridge.follow_path(_plan(), expected_generation=8)

    assert result is None
    bridge.path_client.send_goal_async.assert_not_called()
    newer_handle.cancel_goal_async.assert_not_called()
    assert bridge._goal_generation == 9
    assert bridge._goal_handle is newer_handle
    assert bridge.goal == {"x": 8.0, "y": 9.0, "yaw": 0.2}
    assert bridge.planned_path == [{"x": 8.0, "y": 9.0}]
    assert bridge.nav_status == "active"


def test_follow_path_returns_the_accepted_owned_generation(sim_module):
    bridge = _bridge(sim_module)

    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        generation = bridge.follow_path(_plan())

    assert type(generation) is int
    assert generation == bridge._goal_generation
    assert generation > 0
    bridge.path_client.send_goal_async.assert_called_once()
    assert bridge.nav_status == "active"
    assert bridge.mode == "nav"


def test_stop_while_follow_path_readiness_is_pending_wins_without_blocking(sim_module):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 4
    bridge.nav_status = "active"
    bridge.mode = "nav"
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    stop_done = threading.Event()
    result = []

    def server_is_ready():
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        return True

    bridge.path_client.server_is_ready.side_effect = server_is_ready
    worker = threading.Thread(
        target=lambda: result.append(bridge.follow_path(_plan(), expected_generation=4))
    )
    stopper = threading.Thread(target=lambda: (bridge.stop(), stop_done.set()))
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
            stopper.start()
            assert stop_done.wait(timeout=1.0), "stop waited on readiness"
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)
            if stopper.ident is not None:
                stopper.join(timeout=2.0)

    assert not worker.is_alive()
    assert not stopper.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "idle"
    assert bridge.mode == "estop"


def test_follow_path_expiry_during_readiness_prevents_send(sim_module, monkeypatch):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 6
    readiness_entered = threading.Event()
    release_readiness = threading.Event()
    result = []
    clock = {"now": 100.0}
    monkeypatch.setattr(sim_module.time, "monotonic", lambda: clock["now"])

    def server_is_ready():
        readiness_entered.set()
        release_readiness.wait(timeout=2.0)
        clock["now"] = 106.0
        return True

    bridge.path_client.server_is_ready.side_effect = server_is_ready
    worker = threading.Thread(
        target=lambda: result.append(
            bridge.follow_path(_plan(), expected_generation=6, not_after=105.0)
        )
    )
    with patch("adapters.exploration.follow_path_goal", return_value=MagicMock()):
        worker.start()
        try:
            assert readiness_entered.wait(timeout=2.0)
        finally:
            release_readiness.set()
            worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert result == [None]
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge._goal_generation == 6


def test_stale_readiness_failure_preserves_estop(sim_module):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 10
    bridge.nav_status = "estop"
    bridge.mode = "idle"
    bridge.path_client.server_is_ready.return_value = False

    result = bridge.follow_path(_plan(), expected_generation=9)

    assert result is None
    bridge.path_client.send_goal_async.assert_not_called()
    assert bridge._goal_generation == 10
    assert bridge.nav_status == "estop"
    assert bridge.mode == "idle"


def test_accepted_callback_after_stop_is_canceled_without_resurrecting_state(
    sim_module,
):
    bridge = _bridge(sim_module)
    bridge._goal_generation = 12
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.goal = {"x": 1.0, "y": 2.0, "yaw": 0.1}
    bridge.planned_path = [{"x": 1.0, "y": 2.0}]
    bridge.stop()
    accepted_handle = MagicMock()
    accepted_handle.accepted = True
    future = MagicMock()
    future.result.return_value = accepted_handle

    bridge._goal_response(future, generation=12)

    accepted_handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None
    assert bridge.nav_status == "idle"
    assert bridge.mode == "estop"
    assert bridge.goal is None
    assert bridge.planned_path == []


def test_stale_conditional_cancel_and_status_cannot_change_newer_state(sim_module):
    bridge = _bridge(sim_module)
    newer_handle = MagicMock()
    bridge._goal_generation = 15
    bridge._goal_handle = newer_handle
    bridge.goal = {"x": 3.0, "y": 4.0, "yaw": 0.3}
    bridge.planned_path = [{"x": 3.0, "y": 4.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"

    assert bridge.cancel_goal_if_current(14) is None
    assert bridge.set_nav_status_if_current(14, "failed") is False
    newer_handle.cancel_goal_async.assert_not_called()
    assert bridge._goal_generation == 15
    assert bridge._goal_handle is newer_handle
    assert bridge.goal == {"x": 3.0, "y": 4.0, "yaw": 0.3}
    assert bridge.planned_path == [{"x": 3.0, "y": 4.0}]
    assert bridge.nav_status == "active"
    assert bridge.mode == "nav"
