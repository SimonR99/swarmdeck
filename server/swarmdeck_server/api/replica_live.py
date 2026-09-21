"""Live overlays and frame-qualified goals for onboard component maps."""

from __future__ import annotations

import asyncio
import os
import time

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict, field_validator

from autonomy.live_mapping import (
    LIVE_MAPPING_MAX_AGE_S,
    navigation_goal,
    point,
    solution_order,
)
from ..fleet.registry import registry
from .objective_commands import send_objective

router = APIRouter()


class ComponentGoal(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class LiveGoal(BaseModel):
    robot_id: str
    component_id: str
    solution_order: tuple[int, int]
    goal: ComponentGoal

    @field_validator("solution_order", mode="before")
    @classmethod
    def valid_solution_order(cls, value):
        return tuple(solution_order(value))


async def component(session_id, component_id):
    from .replica_views import (
        current_catalogue,
        deployment_component_id,
        deployment_placements,
        deployment_view,
        is_deployment_component,
    )

    active = os.environ.get("SWARMDECK_MISSION_ID")
    if active and session_id != active:
        raise HTTPException(409, "Component is not in the live mission")
    try:
        if is_deployment_component(component_id):
            if component_id != deployment_component_id(session_id):
                raise KeyError("Replica component not found")
            placements = deployment_placements(session_id)
            view = await asyncio.to_thread(
                lambda: deployment_view(
                    current_catalogue(session_id), session_id, placements
                )
            )
            if view is None:
                raise KeyError("Replica component not found")
        else:
            view = await asyncio.to_thread(
                lambda: current_catalogue(session_id).view(session_id, component_id)
            )
        if not isinstance(view.get("snapshot_id"), str) or not view["snapshot_id"]:
            raise ValueError("component view has no publication identity")
        if view.get("solution_order_known") is not True or "solution_order" not in view:
            raise ValueError("component view has no frame revision")
        if view.get("selected") is None:
            raise ValueError("component view has no selected component")
        return view
    except (KeyError, LookupError, TypeError, ValueError, OverflowError) as exc:
        raise HTTPException(409, "Component publication is not ready") from exc


def robot_frame_revision(view, robot_id):
    """The solution order this robot's own publication of the view carries.

    A live merge's publishers lag one another by a few solver reports; a
    robot's telemetry is fenced against its own accepted solution, not the
    newest one in the view.
    """
    orders = view.get("solution_orders") or {}
    if robot_id in orders and orders[robot_id] is not None:
        return tuple(solution_order(orders[robot_id]))
    return view_solution_order(view)


def view_solution_order(view):
    """Catalogue uses None for the valid pre-optimizer (0, -1) sentinel.

    The deployment composite has no solver solution at all and reports the
    same sentinel; each member's own frame revision is fenced separately by
    ``composite_live_robot`` and by the robot when a goal is dispatched.
    """
    return (
        (0, -1)
        if view["solution_order"] is None
        else tuple(solution_order(view["solution_order"]))
    )


def compatible_solution_orders(first, second):
    """Whether two solution orders name one component frame.

    Equal orders do. So do two real solutions of the same optimizer robot
    (``order[1]``) that differ only in their clock: they are the same frame a
    few solver reports apart (the rule ``replica_components._assemble``
    applies to the publishers of a merged view). The pre-optimizer sentinel
    ``(0, -1)`` on one side only, or two optimizers, are not one frame.

    The live overlay needs the tolerance because a robot's mapping authority
    is product-gated: it names the frame of the last MOLA product its worker
    published, which lags the solver by tens of reports while the fleet
    explores under inter-robot closures (benchbot 2026-09-19, mission
    1a8cc114: authorities at [413, 0] and [403, 0] against replicas at
    [443, 0]), and an equality fence hid exactly the lagging robots from the
    3D view. The displayed pose is placed with the authority's own
    ``T_component_navigation``, so it sits off the drawn map by the solver's
    step between the two orders. Goal dispatch still converts a click with
    the robot's own authority and sends the displayed order; the robot fences
    the goal on its own frame revision and refuses one it does not hold.
    """
    first, second = tuple(first), tuple(second)
    if first == second:
        return True
    if first[1] == -1 or second[1] == -1:
        return False
    return first[1] == second[1]


def composite_member(view, robot_id):
    """A composite member's placement record, or None for a non-member."""
    if not view.get("composite"):
        return None
    for member in view.get("members", ()):
        if member["robot_id"] == robot_id:
            return member
    return None


def composite_live_robot(robot, view, session_id, now):
    """A member's live state placed in the deployment frame.

    The pose, goal and paths stay in the robot's navigation frame exactly as it
    reported them; ``T_component_navigation`` becomes the deployment frame's
    ``T_world_navigation`` so the same browser projection lands them where the
    2D fleet map draws them. The robot's own component and frame revision are
    kept under ``member`` and remain the fence for its telemetry.
    """
    member = composite_member(view, robot.robot_id)
    if member is None:
        return None
    value = live_robot(
        robot,
        session_id,
        member["component_id"],
        tuple(member["solution_order"]),
        now,
    )
    if value is None:
        return None
    return {
        **value,
        "component_id": view["component_id"],
        "solution_order": list(view_solution_order(view)),
        "T_component_navigation": member["T_world_navigation"],
        "member": {
            "component_id": member["component_id"],
            "solution_order": list(member["solution_order"]),
        },
    }


def live_robot(robot, session_id, component_id, expected_solution_order, now):
    """The robot's fresh live state in this component, or None.

    ``expected_solution_order`` is the order of the robot's own publication
    of the displayed view; the authority may trail it by solver reports
    (``compatible_solution_orders``).
    """
    value = robot.live_mapping
    if (
        value is None
        or not robot.online
        or not registry.has_sink(robot.robot_id)
        or value["mission_id"] != session_id
        or value["component_id"] != component_id
        or not compatible_solution_orders(
            value["solution_order"], expected_solution_order
        )
    ):
        return None
    age = value["authority_age_s"] + max(0.0, now - robot.live_mapping_received_at)
    if age > LIVE_MAPPING_MAX_AGE_S:
        return None
    return {
        **{key: val for key, val in value.items() if key != "authority_age_s"},
        "robot_type": robot.robot_type,
        "nav_status": robot.nav_status,
        "mode": robot.mode,
        "freshness": {
            "pose_s": age,
            "goal_s": age if value["goal"] else None,
            "path_s": (
                age
                if any(
                    value[name]
                    for name in (
                        "planned_path",
                        "global_planned_path",
                        "local_planned_path",
                    )
                )
                else None
            ),
        },
    }


@router.get("/components/live/{session_id}")
async def live_component(session_id: str, component_id: str):
    view = await component(session_id, component_id)
    selected = view["selected"]
    frame_revision = view_solution_order(view)
    now = time.monotonic()
    if view.get("composite"):
        robots = [
            value
            for robot in registry.robots.values()
            if (value := composite_live_robot(robot, view, session_id, now)) is not None
        ]
    else:
        robots = [
            value
            for robot in registry.robots.values()
            if (
                value := live_robot(
                    robot,
                    session_id,
                    component_id,
                    robot_frame_revision(view, robot.robot_id),
                    now,
                )
            )
            is not None
        ]
    if not robots:
        raise HTTPException(404, "No fresh robot telemetry for this component")
    return {
        "version": 1,
        "mission_id": session_id,
        "session_id": session_id,
        "component_id": component_id,
        "frame_id": selected["frame_id"],
        "solution_order": frame_revision,
        "robots": robots,
    }


@router.post("/components/live/{session_id}/goal")
async def set_live_goal(session_id: str, command: LiveGoal):
    view = await component(session_id, command.component_id)
    frame_revision = view_solution_order(view)
    if command.solution_order != frame_revision:
        raise HTTPException(409, "Displayed component frame is stale")
    robot = registry.robots.get(command.robot_id)
    if robot is None:
        raise HTTPException(404, "Unknown robot")
    error = registry.command_guard(robot.robot_id) if registry.command_guard else None
    if error:
        raise HTTPException(409, error)
    if not all(
        registry.can(robot.robot_id, cap) for cap in ("navigate", "plan_objective")
    ):
        raise HTTPException(409, "Robot does not have an onboard objective planner")
    member = composite_member(view, robot.robot_id)
    if view.get("composite") and member is None:
        raise HTTPException(409, "Robot is not placed in the deployment composite")
    live = (
        composite_live_robot(robot, view, session_id, time.monotonic())
        if member is not None
        else live_robot(
            robot,
            session_id,
            command.component_id,
            robot_frame_revision(view, robot.robot_id),
            time.monotonic(),
        )
    )
    if live is None:
        raise HTTPException(
            409, "Robot mapping authority is stale or in another component"
        )
    original = point(command.goal.model_dump(), heading=True)
    try:
        # One inversion of the displayed frame's T_component_navigation; for
        # the composite that is T_world_navigation(robot).
        goal = navigation_goal(original, live["T_component_navigation"])
        if member is not None:
            # The robot re-resolves ``component_goal`` through its own fresh
            # T_component_navigation and fences it with its own component id
            # and frame revision, so hand it the goal in ITS component frame:
            # inv(T_world_component) applied to the deployment-frame click.
            component_goal = navigation_goal(original, member["T_world_component"])
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    if member is not None:
        goal.update(
            frame_id=live["navigation_frame"],
            mission_id=session_id,
            component_id=member["component_id"],
            solution_order=list(member["solution_order"]),
            component_goal=component_goal,
            deployment_goal=original,
        )
    else:
        # The click was converted with the robot's own authority transform,
        # so it is a goal in the frame of the authority's solution order (a
        # few solver reports behind the displayed view while the product
        # lags); the robot fences the goal on exactly that order.
        goal.update(
            frame_id=live["navigation_frame"],
            mission_id=session_id,
            component_id=command.component_id,
            solution_order=list(live["solution_order"]),
            component_goal=original,
        )
    registry.attend(robot.robot_id)
    if not await send_objective(registry, robot.robot_id, "navigate", goal):
        raise HTTPException(409, "Robot command connection is unavailable")
    return {"ok": True}
