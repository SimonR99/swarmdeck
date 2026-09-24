"""ROS watchdog timers must remain effective while telemetry is stalled."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from test_adapter_sim_goals import _ImmediateFuture


def _twist():
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0)
    )


def _timed_bridge(sim_module, monkeypatch):
    clock = {"now": 100.0}
    monkeypatch.setattr(sim_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(sim_module, "Twist", _twist)
    monkeypatch.delenv("SWARMDECK_DETECTOR_URL", raising=False)
    monkeypatch.setattr(
        "adapters.exploration.configure_exploration", lambda bridge: None
    )
    monkeypatch.setattr(
        "adapters.objective_planning.configure_objective_planning", lambda bridge: None
    )
    node = MagicMock()
    timers = []

    def create_timer(period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        timers.append(timer)
        return timer

    node.create_timer.side_effect = create_timer
    bridge = sim_module.RobotBridge(node, "robot_0", "http://backend")
    velocities = []
    bridge.pub_cmd = SimpleNamespace(publish=velocities.append)
    # Select the actual registered callback; simply calling _watchdogs directly
    # would miss the bug where sim never schedules it independently of telemetry.
    assert len(timers) == 1
    assert timers[0].period == 0.05
    return bridge, timers[0].callback, clock, velocities


@pytest.mark.parametrize("elapsed, stale", [(0.5, False), (2.0, True)])
def test_sim_timer_cancels_stale_link_without_a_state_tick(
    sim_module, monkeypatch, elapsed, stale
):
    bridge, tick, clock, velocities = _timed_bridge(sim_module, monkeypatch)
    bridge.nav_status, bridge.mode = "active", "nav"
    bridge.goal = {"x": 1.0, "y": 2.0}
    handle = MagicMock(accepted=True)
    bridge._goal_response(_ImmediateFuture(handle), 0)
    bridge.note_link_activity()
    command = _twist()
    command.linear.x = 0.3
    bridge._on_nav_cmd_vel(command)
    assert velocities == [command]
    velocities.clear()

    clock["now"] += elapsed
    tick()  # session_state_tick and websocket processing remain stalled.

    if stale:
        assert bridge.nav_status == "cancelled"
        assert bridge._goal_handle is None
        assert bridge.goal is None
        assert bridge._goal_generation > 0
        assert velocities
        assert all(msg.linear.x == msg.angular.z == 0.0 for msg in velocities)
        velocities.clear()
        bridge._on_nav_cmd_vel(command)
        assert velocities == []
    else:
        assert bridge.nav_status == "active"
        assert bridge._goal_handle is handle
        assert bridge._goal_generation == 0
        assert velocities == []
        bridge._on_nav_cmd_vel(command)
        assert velocities == [command]


def test_sim_timer_consumes_pending_drive_and_applies_deadman(sim_module, monkeypatch):
    bridge, tick, clock, velocities = _timed_bridge(sim_module, monkeypatch)
    bridge.note_drive_command(0.2, -0.1)

    tick()

    assert bridge._pending_drive is None
    assert bridge.mode == "teleop"
    assert len(velocities) == 1
    assert velocities[0].linear.x == 0.2
    assert velocities[0].angular.z == -0.1
    tick()
    assert len(velocities) == 1
    clock["now"] += 0.5
    tick()
    assert bridge.mode == "idle"
    assert len(velocities) == 2
    assert velocities[-1].linear.x == velocities[-1].angular.z == 0.0


def test_sim_timer_skips_a_held_goal_lock_and_cancels_on_the_next_tick(
    sim_module, monkeypatch
):
    """C1: the timer runs on the executor thread; it must never block on the lock."""
    import threading
    import time as real_time

    bridge, tick, clock, velocities = _timed_bridge(sim_module, monkeypatch)
    bridge.nav_status, bridge.mode = "active", "nav"
    bridge._goal_response(_ImmediateFuture(MagicMock(accepted=True)), 0)
    bridge.note_link_activity()
    velocities.clear()
    clock["now"] += 2.0
    taken, release = threading.Event(), threading.Event()

    def hold():
        with bridge._goal_lock:
            taken.set()
            release.wait(5.0)

    holder = threading.Thread(target=hold, daemon=True)
    holder.start()
    assert taken.wait(1.0)
    try:
        started = real_time.perf_counter()
        tick()
        assert real_time.perf_counter() - started < 0.1
        assert bridge.nav_status == "active"
        assert velocities == []
    finally:
        release.set()
        holder.join(1.0)

    tick()
    assert bridge.nav_status == "cancelled"
    assert velocities and velocities[-1].linear.x == 0.0
