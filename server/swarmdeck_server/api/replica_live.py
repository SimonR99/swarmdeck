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
    from .replica_views import current_catalogue

    active = os.environ.get("SWARMDECK_MISSION_ID")
    if active and session_id != active:
        raise HTTPException(409, "Component is not in the live mission")
    try:
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


def view_solution_order(view):
    """Catalogue uses None for the valid pre-optimizer (0, -1) sentinel."""
    return (
        (0, -1)
        if view["solution_order"] is None
        else tuple(solution_order(view["solution_order"]))
    )


def live_robot(robot, session_id, component_id, expected_solution_order, now):
    value = robot.live_mapping
    if (
        value is None
        or not robot.online
        or not registry.has_sink(robot.robot_id)
        or value["mission_id"] != session_id
        or value["component_id"] != component_id
        or tuple(value["solution_order"]) != expected_solution_order
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
    robots = [
        value
        for robot in registry.robots.values()
        if (value := live_robot(robot, session_id, component_id, frame_revision, now))
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
    if not all(
        registry.can(robot.robot_id, cap) for cap in ("navigate", "plan_objective")
    ):
        raise HTTPException(409, "Robot does not have an onboard objective planner")
    live = live_robot(
        robot,
        session_id,
        command.component_id,
        frame_revision,
        time.monotonic(),
    )
    if live is None:
        raise HTTPException(
            409, "Robot mapping authority is stale or in another component"
        )
    original = point(command.goal.model_dump(), heading=True)
    try:
        goal = navigation_goal(original, live["T_component_navigation"])
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    goal.update(
        frame_id=live["navigation_frame"],
        mission_id=session_id,
        component_id=command.component_id,
        solution_order=list(command.solution_order),
        component_goal=original,
    )
    registry.attend(robot.robot_id)
    if not await send_objective(registry, robot.robot_id, "navigate", goal):
        raise HTTPException(409, "Robot command connection is unavailable")
    return {"ok": True}
