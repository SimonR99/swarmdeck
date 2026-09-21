import asyncio

from swarmdeck_server.fleet.registry import Registry


def test_return_home_uses_onboard_objective_without_server_planner(monkeypatch):
    from swarmdeck_server.api import app as module

    registry = Registry()
    robot = registry.hello(
        {
            "robot_id": "r0",
            "coordinate_frame": "merged",
            "capabilities": ["navigate", "plan_objective"],
        },
        None,
    )
    assert robot.home_pose is None
    sent = []

    async def send(robot_id, message):
        sent.append(message)
        return True

    monkeypatch.setattr(registry, "send", send)
    monkeypatch.setattr(module, "registry", registry)
    monkeypatch.setattr(module.events, "log", lambda *args, **kw: None)
    asyncio.run(module.handle_gui_message({"type": "return_home", "robot_id": "r0"}))
    assert sent[0]["type"] == "plan_objective"
    assert sent[0]["objective"] == "return_home"
    assert "goal" not in sent[0]
