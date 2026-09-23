import asyncio
from types import SimpleNamespace

from swarmdeck_server.api import app
from swarmdeck_server.api.navigation_alerts import explain_navigation_failure


def test_planner_reasons_become_actionable_explanations():
    assert "No known route" in explain_navigation_failure(
        "goal cannot be linked to the global graph; goal lattice: 812 vertices "
        "in 3 sweep(s), 640 reached from the goal, 0 bridge checks, none onto "
        "the robot's roadmap"
    )
    assert "not on mapped ground" in explain_navigation_failure(
        "no mapped ground under the goal"
    )
    assert "blocked or stuck" in explain_navigation_failure(
        "Failed to make progress; error_code=105"
    )
    assert "nothing left to explore" in explain_navigation_failure(
        "no known route to the goal, and exploring toward it ended (complete)"
    )
    assert "see the planner's reason" in explain_navigation_failure("odd new reason")
    assert "without a reported reason" in explain_navigation_failure(None)


def test_one_alert_per_failure_retired_when_the_robot_moves_on(monkeypatch):
    sent = []

    async def publish(message):
        sent.append(message)

    monkeypatch.setattr(app, "broadcast", publish)
    monkeypatch.setattr(app, "_alerts", {})
    monkeypatch.setattr(app, "_nav_failure_alerts", {})
    monkeypatch.setattr(app, "_alert_suppress_until", {})
    robot = SimpleNamespace(
        robot_id="robot_0", nav_status="failed", nav_failure_reason=None
    )

    async def scenario():
        await app.sync_navigation_alert(robot)
        (first,) = app._alerts.values()
        assert first["kind"] == "nav_failure" and first["detail"] is None
        # The planner's reason arrives with the next state message.
        robot.nav_failure_reason = "no mapped ground under the goal"
        await app.sync_navigation_alert(robot)
        (updated,) = app._alerts.values()
        assert updated["id"] == first["id"]
        assert updated["detail"] == "no mapped ground under the goal"
        assert "not on mapped ground" in updated["message"]
        # Repeated state messages do not re-raise it.
        count = len(sent)
        await app.sync_navigation_alert(robot)
        assert len(sent) == count
        # A new goal retires it; the next failure is a new alert.
        robot.nav_status, robot.nav_failure_reason = "active", None
        await app.sync_navigation_alert(robot)
        assert app._alerts == {}
        robot.nav_status = "failed"
        robot.nav_failure_reason = "controller recovery timed out"
        await app.sync_navigation_alert(robot)
        (second,) = app._alerts.values()
        assert second["id"] != first["id"]

    asyncio.run(scenario())
