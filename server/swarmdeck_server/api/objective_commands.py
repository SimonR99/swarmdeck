"""Dispatch operator intent to robots that advertise an onboard objective planner."""

from ..bus import stamps


async def send_objective(registry, robot_id, objective, goal=None):
    command = {"type": "plan_objective", "objective": objective, **stamps()}
    if goal is not None:
        command["goal"] = goal
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
