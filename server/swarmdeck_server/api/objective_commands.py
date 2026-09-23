"""Dispatch operator intent to robots that advertise an onboard objective planner."""

from ..bus import stamps


async def send_objective(
    registry, robot_id, objective, goal=None, explore_if_unknown=False
):
    command = {"type": "plan_objective", "objective": objective, **stamps()}
    if goal is not None:
        command["goal"] = goal
    if explore_if_unknown:
        # Explore toward a goal with no known route instead of failing it.
        command["explore_if_unknown"] = True
    if await registry.send(robot_id, command):
        robot = registry.robots[robot_id]
        robot.goal = goal
        robot.nav_status, robot.mode = "active", "nav"
        robot.nav_failure_reason = None
        robot.global_planned_path = []
        robot.local_planned_path = []
        robot.planned_path = []
        return True
    return False
