"""Robot teleoperation, navigation commands, body controls & perception inspection routes.

Provides clean REST endpoints for:
- Direct velocity driving (/api/robot/{id}/drive)
- Navigation goals and cancelation (/api/robot/{id}/goal, /cancel, /stop)
- Quadruped body / posture commands (/api/robot/{id}/body)
- Camera & vision inspection (/api/robot/{id}/vision)
- Object detections and review snapshot (/api/detections)
"""

from __future__ import annotations

import asyncio
import time
from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse


def _app():
    from . import app
    return app


async def post_robot_drive(robot_id: str, request: Request) -> Any:
    app = _app()
    try:
        body = await request.json()
    except Exception:
        body = {}

    linear = max(-0.8, min(0.8, float(body.get("linear", 0.0))))
    angular = max(-1.5, min(1.5, float(body.get("angular", 0.0))))
    duration = max(0.0, min(60.0, float(body.get("duration", 0.0))))

    robot = app.registry.robots.get(robot_id)
    if not robot:
        return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)

    sent = await app.registry.send(
        robot_id,
        {"type": "drive", "linear": linear, "angular": angular, **app.stamps()},
    )
    if not sent:
        return JSONResponse({"error": f"Failed to send drive to '{robot_id}'"}, status_code=502)

    robot.goal = None
    app.events.log("agent_drive", {"robot_id": robot_id, "linear": linear, "angular": angular, "duration": duration})

    if duration > 0.0 and (linear != 0.0 or angular != 0.0):
        async def _auto_stop(rid: str, delay: float):
            await asyncio.sleep(delay)
            await app.registry.send(
                rid,
                {"type": "drive", "linear": 0.0, "angular": 0.0, **app.stamps()},
            )
            app.events.log("agent_drive_stop", {"robot_id": rid})

        asyncio.create_task(_auto_stop(robot_id, duration))

    return {
        "ok": True,
        "robot_id": robot_id,
        "linear": linear,
        "angular": angular,
        "duration": duration,
    }


async def post_robot_goal(robot_id: str, request: Request) -> Any:
    app = _app()
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON body"}, status_code=400)

    if "x" not in body or "y" not in body:
        return JSONResponse({"error": "'x' and 'y' required"}, status_code=400)

    robot = app.registry.robots.get(robot_id)
    if not robot:
        return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)

    if not app.registry.can(robot_id, "navigate"):
        return JSONResponse({"error": f"Robot '{robot_id}' does not support navigation"}, status_code=400)

    goal = {
        "x": float(body["x"]),
        "y": float(body["y"]),
        "yaw": float(body.get("yaw", 0.0)),
    }

    taken_by = app.goal_taken(goal, exclude=robot_id)
    if taken_by:
        return JSONResponse({"error": f"Goal position is already occupied/assigned to {taken_by}"}, status_code=409)

    local_goal = (
        goal
        if robot.coordinate_frame == "merged"
        else app.map_service.world_to_robot(robot_id, goal)
    )

    start_pose = (
        robot.pose
        if robot.coordinate_frame == "merged"
        else app.map_service.robot_to_world(robot_id, robot.pose)
    )
    world_goal = (
        goal
        if robot.coordinate_frame == "merged"
        else app.map_service.robot_to_world(robot_id, local_goal)
    )
    planned_world = app.map_service.plan_path(robot_id, start_pose, world_goal)
    local_planned = (
        (
            planned_world
            if robot.coordinate_frame == "merged"
            else [app.map_service.world_to_robot(robot_id, pt) for pt in planned_world]
        )
        if planned_world
        else []
    )

    sent = await app.registry.send(
        robot_id,
        {
            "type": "navigate_to",
            "goal": local_goal,
            "path": local_planned,
            **app.stamps(),
        },
    )
    if not sent:
        return JSONResponse({"error": f"Failed to send navigation goal to {robot_id}"}, status_code=502)

    robot.goal = local_goal
    robot.nav_status = "active"
    robot.mode = "nav"
    if local_planned:
        robot.global_planned_path = local_planned
        robot.planned_path = local_planned

    app.events.log("agent_goal", {"robot_id": robot_id, "goal": goal})
    return {"ok": True, "robot_id": robot_id, "goal": goal, "path_length": len(local_planned)}


async def post_robot_cancel(robot_id: str) -> Any:
    app = _app()
    robot = app.registry.robots.get(robot_id)
    if not robot:
        return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)

    sent = await app.registry.send(robot_id, {"type": "cancel_goal", **app.stamps()})
    if sent:
        robot.goal = None
        robot.global_planned_path = []
        robot.local_planned_path = []
        robot.planned_path = []
        if robot.nav_status == "active":
            robot.nav_status = "cancelled"

    app.events.log("agent_cancel", {"robot_id": robot_id})
    return {"ok": True, "robot_id": robot_id}


async def post_robot_stop(robot_id: str) -> Any:
    app = _app()
    if robot_id == "all":
        targets = list(app.registry.robots.keys())
    else:
        if robot_id not in app.registry.robots:
            return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)
        targets = [robot_id]

    for rid in targets:
        await app.registry.send(rid, {"type": "cancel_goal", **app.stamps()})
        await app.registry.send(
            rid, {"type": "drive", "linear": 0.0, "angular": 0.0, **app.stamps()}
        )
        r = app.registry.robots.get(rid)
        if r:
            r.goal = None
            r.planned_path = []
            if r.nav_status == "active":
                r.nav_status = "cancelled"

    app.events.log("agent_stop", {"targets": targets})
    return {"ok": True, "stopped": targets}


async def post_robot_body(robot_id: str, request: Request) -> Any:
    app = _app()
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON body"}, status_code=400)

    action = str(body.get("action", ""))
    if action not in (
        "claim", "release", "sit", "stand", "damping", "lie_to_stand",
        "lock_stand", "walk_mode", "run_mode", "wave", "set_height"
    ):
        return JSONResponse({"error": f"Unknown body action '{action}'"}, status_code=400)

    robot = app.registry.robots.get(robot_id)
    if not robot:
        return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)

    msg: dict[str, Any] = {"type": "body_command", "action": action, **app.stamps()}
    if "height" in body:
        try:
            msg["height"] = float(body["height"])
        except (ValueError, TypeError):
            pass

    sent = await app.registry.send(robot_id, msg)
    app.events.log("agent_body", {"robot_id": robot_id, "action": action})
    return {"ok": sent, "robot_id": robot_id, "action": action}


async def get_robot_vision(robot_id: str) -> Any:
    app = _app()
    robot = app.registry.robots.get(robot_id)
    if not robot:
        return JSONResponse({"error": f"Robot '{robot_id}' not found"}, status_code=404)

    # Ensure interest is requested so adapter streams frames if available
    await app.push_camera_interest({robot_id})

    frame_tuple = app._camera_frames.get(robot_id)
    has_frame = frame_tuple is not None
    frame_age_ms = int((time.monotonic() - frame_tuple[1]) * 1000) if frame_tuple else None
    seq = frame_tuple[2] if frame_tuple else None

    # Retrieve live tracks for this robot
    tracks = [
        d for d in app._detections.values()
        if d.get("robot_id") == robot_id
    ]

    return {
        "robot_id": robot_id,
        "robot_type": robot.robot_type,
        "pose": robot.pose,
        "camera_streaming": has_frame and (frame_age_ms is not None and frame_age_ms < 5000),
        "frame_age_ms": frame_age_ms,
        "frame_seq": seq,
        "tracks": tracks,
    }


async def get_all_detections() -> Any:
    app = _app()
    snap = app.review_store.snapshot()
    return {
        "tracks": list(app._detections.values()),
        "proposals": snap.get("proposals", []),
        "entities": snap.get("entities", []),
        "ignored": snap.get("ignored", []),
    }
