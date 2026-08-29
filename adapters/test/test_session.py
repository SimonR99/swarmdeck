"""Shared WebSocket session: command dispatch without a live socket."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

from adapters.session import dispatch_command


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
        await dispatch_command(ros2, {"type": "navigate_to", "goal": goal, "path": path}, loop)
        await dispatch_command(ros1, {"type": "navigate_to", "goal": goal, "path": path}, loop)

    asyncio.run(run())
    assert ros2.calls == [("nav", goal)]
    assert ros1.calls == [("nav", goal, path)]


def test_unknown_command_types_are_ignored():
    bridge = SimpleNamespace()

    async def run():
        await dispatch_command(bridge, {"type": "not_a_command"}, asyncio.get_running_loop())

    asyncio.run(run())
