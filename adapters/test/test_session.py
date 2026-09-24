"""Shared WebSocket session contracts without a live socket."""

from __future__ import annotations

import asyncio
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest

from adapters.session import _rx, dispatch_command


def test_unknown_command_types_are_ignored():
    bridge = SimpleNamespace()

    async def run():
        await dispatch_command(
            bridge, {"type": "not_a_command"}, asyncio.get_running_loop()
        )

    asyncio.run(run())


def test_a_legacy_reset_command_is_ignored():
    """Resets go through the epoch supervisor; the adapter ignores `reset`."""
    calls = []
    bridge = SimpleNamespace(
        reset=lambda: calls.append("reset"),
        exploration=SimpleNamespace(stop=lambda: calls.append("stop_exploration")),
        goal_exploration=SimpleNamespace(cancel=lambda: calls.append("cancel")),
    )

    async def run():
        await dispatch_command(bridge, {"type": "reset"}, asyncio.get_running_loop())
        await asyncio.sleep(0)

    asyncio.run(run())
    assert calls == []


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


def test_home_objective_stops_exploration_before_dispatching_home():
    calls = []
    bridge = SimpleNamespace(
        exploration=SimpleNamespace(stop=lambda: calls.append("stop_exploration")),
        return_home=lambda: calls.append("home"),
    )

    async def run():
        await dispatch_command(
            bridge,
            {"type": "plan_objective", "objective": "return_home"},
            asyncio.get_running_loop(),
        )

    asyncio.run(run())
    assert calls == ["stop_exploration", "home"]


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


def test_command_queued_before_reset_cannot_move_after_new_epoch_is_ready():
    from autonomy.map_epochs import robot_run_id

    mission = "00000000-0000-0000-0000-000000000001"
    state = {"epoch": 0, "ready": True, "goal": None}
    release = threading.Event()

    def authority():
        if not state["ready"]:
            return None
        return {
            "mission_id": mission,
            "robot_map_epoch": state["epoch"],
            "run_id": robot_run_id(mission, "r0", state["epoch"]),
        }

    def command(epoch):
        return {
            "type": "plan_objective",
            "objective": "navigate",
            "goal": {"x": 2, "y": 0},
            "mission_id": mission,
            "robot_map_epoch": epoch,
            "map_run_id": robot_run_id(mission, "r0", epoch),
        }

    bridge = SimpleNamespace(
        _goal_lock=threading.RLock(),
        _mapping_authority=SimpleNamespace(current=authority),
        plan_objective=lambda objective, goal: state.update(goal=goal),
        stop=lambda: state.update(goal=None),
    )

    async def run():
        loop = asyncio.get_running_loop()
        loop.set_default_executor(ThreadPoolExecutor(max_workers=1))
        blocked = loop.run_in_executor(None, release.wait)
        pending = asyncio.create_task(dispatch_command(bridge, command(0), loop))
        await asyncio.sleep(0)
        state["epoch"] = 1
        release.set()
        await blocked
        await pending
        assert state["goal"] is None

        await dispatch_command(bridge, command(1), loop)
        assert state["goal"] == {"x": 2, "y": 0}
        state["ready"] = False
        await dispatch_command(bridge, {"type": "stop"}, loop)
        assert state["goal"] is None

    asyncio.run(run())
