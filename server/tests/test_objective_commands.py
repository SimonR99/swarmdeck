import asyncio

from swarmdeck_server.fleet.registry import Registry


def test_onboard_goals_bypass_central_path_and_home_resolution(monkeypatch):
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

    async def send(robot, message):
        sent.append(message)
        return True

    def no_central_plan(*args):
        raise AssertionError("Onboard objectives cannot depend on a server map")

    monkeypatch.setattr(registry, "send", send)
    monkeypatch.setattr(module, "registry", registry)
    monkeypatch.setattr(module, "goal_taken", lambda *args, **kw: None)
    monkeypatch.setattr(module, "is_robot_enabled", lambda *args: True)
    monkeypatch.setattr(module.events, "log", lambda *args, **kw: None)
    monkeypatch.setattr(module.map_service, "plan_path", no_central_plan)
    asyncio.run(module.handle_gui_message({"type": "return_home", "robot_id": "r0"}))
    asyncio.run(
        module.handle_gui_message(
            {
                "type": "set_goal",
                "robot_id": "r0",
                "payload": {"x": 4, "y": 2},
            }
        )
    )
    assert sent[0]["type"] == "plan_objective"
    assert sent[0]["objective"] == "return_home"
    assert "goal" not in sent[0]
    assert sent[1]["objective"] == "navigate"
    assert sent[1]["goal"] == {"x": 4, "y": 2}
