"""Shared WebSocket session: command dispatch without a live socket."""

from __future__ import annotations

import asyncio
import json
import threading
from types import SimpleNamespace
import pytest

from adapters.session import dispatch_command, _rx, _tx_maps


class _Ros2Nav:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def navigate_to(self, goal):
        self.calls.append(("nav", goal))


class _Ros1Nav:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def navigate_to(self, goal, path=None):
        self.calls.append(("nav", goal, path))


def test_navigate_to_passes_path_only_when_the_bridge_accepts_it():
    ros2 = _Ros2Nav()
    ros1 = _Ros1Nav()
    goal = {"x": 1.0, "y": 2.0}
    path = [{"x": 0.0, "y": 0.0}]

    async def run():
        loop = asyncio.get_running_loop()
        await dispatch_command(
            ros2, {"type": "navigate_to", "goal": goal, "path": path}, loop
        )
        await dispatch_command(
            ros1, {"type": "navigate_to", "goal": goal, "path": path}, loop
        )

    asyncio.run(run())
    assert ros2.calls == [("nav", goal)]
    assert ros1.calls == [("nav", goal, path)]


def test_unknown_command_types_are_ignored():
    bridge = SimpleNamespace()

    async def run():
        await dispatch_command(
            bridge, {"type": "not_a_command"}, asyncio.get_running_loop()
        )

    asyncio.run(run())


def test_explore_command_binds_common_run_before_start():
    calls = []
    coordinator = SimpleNamespace(begin_run=lambda *args: calls.append(("run", *args)))
    explorer = SimpleNamespace(
        coordinator=coordinator, active=False, start=lambda: calls.append(("start",))
    )
    bridge = SimpleNamespace(exploration=explorer)

    async def run():
        await dispatch_command(
            bridge,
            {
                "type": "explore",
                "enabled": True,
                "run_id": "shared",
                "participants": ["r0", "r1"],
            },
            asyncio.get_running_loop(),
        )

    asyncio.run(run())
    assert calls == [("run", "shared", ["r0", "r1"]), ("start",)]
    calls.clear()
    explorer.active = True
    coordinator.run_id = "previous"
    asyncio.run(run())
    assert calls == [("run", "shared", ["r0", "r1"]), ("start",)]


@pytest.mark.parametrize("onboard", [False, True])
def test_onboard_session_does_not_use_central_optimizer_or_nav_map(
    monkeypatch, onboard
):
    from adapters import session

    calls = []

    async def offload(loop, bridge, name):
        calls.append(name)

    async def stop_after_tick(period):
        raise asyncio.CancelledError

    monkeypatch.setattr(session, "_offload", offload)
    monkeypatch.setattr(session.asyncio, "sleep", stop_after_tick)
    bridge = SimpleNamespace(onboard_mapping=onboard)

    async def run():
        with pytest.raises(asyncio.CancelledError):
            await _tx_maps(
                bridge,
                None,
                {
                    "rates": {
                        "state_hz": 5,
                        "map_period_s": 0,
                        "cloud_period_s": 0,
                    }
                },
            )

    asyncio.run(run())
    assert ("upload_keyframe" in calls) is not onboard
    assert ("pull_nav_map" in calls) is not onboard
    assert "upload_map" in calls  # Optional operator display remains available.


def test_home_objective_uses_onboard_home_after_stopping_exploration():
    calls = []
    bridge = SimpleNamespace(
        exploration=SimpleNamespace(stop=lambda: calls.append("stop_exploration")),
        return_home=lambda: calls.append("onboard_home"),
    )

    async def run():
        await dispatch_command(
            bridge,
            {
                "type": "plan_objective",
                "objective": "return_home",
                "goal": {"x": 999, "y": 999},
            },
            asyncio.get_running_loop(),
        )

    asyncio.run(run())
    assert calls == ["stop_exploration", "onboard_home"]


def test_receive_loop_processes_stop_while_claimed_objective_is_planning():
    started = threading.Event()
    release = threading.Event()
    finished = threading.Event()
    submitted = []

    class Planner:
        def __init__(self, bridge):
            self.bridge = bridge

        def claim_objective(self, objective, goal):
            self.bridge._goal_generation += 1
            return objective, goal, self.bridge._goal_generation

        def execute_claimed(self, claim):
            started.set()
            release.wait(2.0)
            if claim[2] == self.bridge._goal_generation:
                submitted.append(claim)
            finished.set()

    class Bridge:
        id = "robot_1"

        def __init__(self):
            self._goal_generation = 0
            self.objective_planner = Planner(self)
            self.stopped = False

        def stop(self):
            self._goal_generation += 1
            self.stopped = True

        def note_link_activity(self):
            pass

    class Socket:
        def __init__(self):
            self.index = 0

        def __aiter__(self):
            return self

        async def __anext__(self):
            if self.index == 0:
                self.index += 1
                return json.dumps(
                    {
                        "type": "plan_objective",
                        "objective": "navigate",
                        "goal": {"x": 10.0, "y": 0.0},
                    }
                )
            if self.index == 1:
                self.index += 1
                while not started.is_set():
                    await asyncio.sleep(0.001)
                return json.dumps({"type": "stop"})
            raise StopAsyncIteration

    bridge = Bridge()

    async def run():
        await _rx(bridge, Socket())
        assert bridge.stopped
        assert bridge._goal_generation == 2
        assert not release.is_set()
        release.set()
        while not finished.is_set():
            await asyncio.sleep(0.001)

    asyncio.run(run())
    assert submitted == []
