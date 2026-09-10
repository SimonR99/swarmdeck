from __future__ import annotations

import asyncio
import sys
import threading
from pathlib import Path

SIM_ADAPTER = Path(__file__).resolve().parents[1] / "adapter_sim"
sys.path.insert(0, str(SIM_ADAPTER))
try:
    from slam_startup import (
        STATE_ACTIVE,
        STATE_INACTIVE,
        STATE_UNCONFIGURED,
        TRANSITION_ACTIVATE,
        TRANSITION_CONFIGURE,
        SlamToolboxStartup,
        uses_slam_toolbox,
    )
finally:
    sys.path.remove(str(SIM_ADAPTER))


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


def recovery(states, changes, failures, *, clock=None, deadline_s=2.0):
    clock = clock or Clock()
    states = iter(states)
    return SlamToolboxStartup(
        lambda deadline: next(states),
        lambda transition, deadline: changes.append(transition) is None or True,
        failures.append,
        deadline_s=deadline_s,
        backoff_s=0.1,
        clock=clock,
        sleep=clock.sleep,
    )


def test_active_toolbox_is_observed_without_a_transition():
    changes, failures = [], []

    result = recovery([STATE_ACTIVE], changes, failures).run_once()

    assert result.ready is True
    assert changes == []
    assert failures == []


def test_only_the_toolbox_launch_backend_enables_this_recovery():
    assert uses_slam_toolbox(None) is True
    assert uses_slam_toolbox(" toolbox ") is True
    assert uses_slam_toolbox("rtabmap") is False


def test_inactive_toolbox_is_activated_then_requeried():
    changes, failures = [], []

    result = recovery([STATE_INACTIVE, STATE_ACTIVE], changes, failures).run_once()

    assert result.ready is True
    assert changes == [TRANSITION_ACTIVATE]
    assert failures == []


def test_unconfigured_toolbox_is_configured_requeried_and_activated():
    changes, failures = [], []

    result = recovery(
        [STATE_UNCONFIGURED, STATE_INACTIVE, STATE_ACTIVE], changes, failures
    ).run_once()

    assert result.ready is True
    assert changes == [TRANSITION_CONFIGURE, TRANSITION_ACTIVATE]


def test_transitional_and_unknown_states_are_only_observed():
    changes, failures = [], []

    result = recovery(
        [0, 11, STATE_INACTIVE, STATE_ACTIVE], changes, failures
    ).run_once()

    assert result.ready is True
    assert changes == [TRANSITION_ACTIVATE]


def test_timeout_is_bounded_and_reported_once():
    clock = Clock()
    failures = []

    def unavailable(deadline):
        clock.now += 0.12
        raise TimeoutError("get_state unavailable")

    startup = SlamToolboxStartup(
        unavailable,
        lambda transition, deadline: True,
        failures.append,
        deadline_s=0.25,
        backoff_s=0.05,
        clock=clock,
        sleep=clock.sleep,
    )

    result = startup.run_once()

    assert result.ready is False
    assert clock.now <= 100.30
    assert failures == ["state query failed: get_state unavailable"]
    assert startup.run_once() is None
    assert len(failures) == 1


def test_a_hung_state_query_is_cut_off_per_call_and_retried():
    clock = Clock()
    deadlines = []
    failures = []

    def hung_query(deadline):
        deadlines.append(deadline)
        clock.now = deadline
        raise TimeoutError("get_state timed out")

    startup = SlamToolboxStartup(
        hung_query,
        lambda transition, deadline: True,
        failures.append,
        deadline_s=5.0,
        backoff_s=0.5,
        query_timeout_s=2.0,
        clock=clock,
        sleep=clock.sleep,
    )

    result = startup.run_once()

    assert result.ready is False
    assert deadlines == [102.0, 104.5]
    assert failures == ["state query failed: get_state timed out"]


def test_late_active_response_cannot_turn_an_expired_episode_into_success():
    clock = Clock()
    failures = []

    def late_active(deadline):
        clock.now = deadline
        return STATE_ACTIVE

    startup = SlamToolboxStartup(
        late_active,
        lambda transition, deadline: True,
        failures.append,
        deadline_s=0.2,
        clock=clock,
        sleep=clock.sleep,
    )

    result = startup.run_once()

    assert result.ready is False
    assert failures == ["state query exceeded the startup deadline"]


def test_concurrent_callers_share_one_startup_episode():
    entered = threading.Event()
    release = threading.Event()
    calls = []

    def query(deadline):
        calls.append("query")
        entered.set()
        assert release.wait(1.0)
        return STATE_ACTIVE

    startup = SlamToolboxStartup(query, lambda *args: True, lambda detail: None)
    worker = threading.Thread(target=startup.run_once)
    worker.start()
    assert entered.wait(1.0)

    assert startup.run_once() is None
    release.set()
    worker.join(1.0)

    assert not worker.is_alive()
    assert calls == ["query"]


def test_map_tick_submits_the_startup_episode_only_once(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "fixture_robot"
    bridge.t0 = 0.0
    bridge.cfg = {"rates": {"cloud_period_s": 100.0}}
    calls = []

    class Startup:
        def run_once(self):
            calls.append("run")

    bridge._slam_startup = Startup()
    sim_module.SLAM_GRAPHS.pop(bridge.id, None)

    async def exercise():
        class Loop:
            def run_in_executor(self, executor, function):
                async def completed():
                    return function()

                return completed()

        loop = Loop()

        async def send(message):
            raise AssertionError(f"unexpected message: {message}")

        await bridge.session_maps_tick(1.0, send, loop)
        await bridge.session_maps_tick(2.0, send, loop)

    asyncio.run(exercise())

    assert calls == ["run"]
    assert bridge._slam_startup is None
