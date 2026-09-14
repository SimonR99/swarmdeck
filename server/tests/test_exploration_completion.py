from swarmdeck_server.fleet.registry import Registry
import asyncio
from uuid import UUID


def test_registry_preserves_local_exhaustion_and_expires_offline_completion():
    registry = Registry()
    robot = registry.hello({"robot_id": "r0"}, None)
    registry.update_state(
        {
            "robot_id": "r0",
            "exploration_status": "locally_exhausted",
            "fleet_exploration_status": "complete",
        }
    )
    state = robot.to_state()
    assert state["exploration_status"] == "locally_exhausted"
    assert state["fleet_exploration_status"] == "complete"
    robot.last_seen -= 1000
    assert robot.to_state()["fleet_exploration_status"] == "unknown"


def test_fleet_start_shares_run_and_participants(monkeypatch):
    from swarmdeck_server.api import app as module

    registry = Registry()
    for name in ("r0", "r1"):
        registry.hello({"robot_id": name, "capabilities": ["explore"]}, None)
    sent = []

    async def send(robot, message):
        sent.append((robot, message))
        return True

    monkeypatch.setattr(registry, "send", send)
    monkeypatch.setattr(module, "registry", registry)
    monkeypatch.setattr(module.events, "log", lambda *args, **kwargs: None)
    asyncio.run(module.handle_gui_message({"type": "start_explore"}))
    assert len(sent) == 2
    first, second = (message for _, message in sent)
    assert str(UUID(first["run_id"])) == first["run_id"] == second["run_id"]
    assert first["participants"] == second["participants"] == ["r0", "r1"]


def test_registry_bounds_exploration_reason_and_clears_it_on_progress():
    registry = Registry()
    robot = registry.hello({"robot_id": "r0"}, None)
    registry.update_state(
        {
            "robot_id": "r0",
            "exploration_status": "waiting",
            "exploration_reason": "x" * 800,
        }
    )
    assert robot.to_state()["exploration_reason"] == "x" * 512
    registry.update_state({"robot_id": "r0", "exploration_status": "exploring"})
    assert robot.to_state()["exploration_reason"] is None
