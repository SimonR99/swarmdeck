"""Cortex AI Agent, Vision & Robot Teleoperation REST & Streaming Routes.

Enables the SwarmDeck UI and Cortex assistant to:
- Control robots via REST (drive, navigate, cancel, stop, body postures)
- Query live vision, camera streams, and detections
- Capture live camera snapshots and upload images for multimodal understanding
- Stream interactive Cortex conversations with live tool execution, codebase edits, and vision analysis
"""

from __future__ import annotations

import asyncio
import base64
import json
import mimetypes
import os
import re
import shutil
import time
from pathlib import Path
from typing import Any, AsyncGenerator, Optional

from fastapi import Request, Response
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse


def _app():
    from . import app
    return app


# ----------------------------------------------------------------- Robot Controls


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


# ----------------------------------------------------------------- Cortex Images & Attachments


CAPTURES_DIR = Path("/app/agent/captures/chat_attachments")
CAPTURES_DIR.mkdir(parents=True, exist_ok=True)


async def post_agent_upload(request: Request) -> Any:
    """Accept an uploaded image (base64 payload or raw) and save for Cortex analysis."""
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON payload"}, status_code=400)

    data_b64 = body.get("data")
    if not data_b64:
        return JSONResponse({"error": "'data' base64 string required"}, status_code=400)

    # Strip header if present (e.g. data:image/png;base64,...)
    if "," in data_b64:
        header, data_b64 = data_b64.split(",", 1)

    try:
        raw_bytes = base64.b64decode(data_b64)
    except Exception as exc:
        return JSONResponse({"error": f"Base64 decode failed: {exc}"}, status_code=400)

    raw_filename = body.get("filename") or f"upload_{int(time.time() * 1000)}.png"
    safe_filename = re.sub(r"[^a-zA-Z0-9_.-]", "_", raw_filename)
    target_path = CAPTURES_DIR / safe_filename

    target_path.write_bytes(raw_bytes)

    return {
        "ok": True,
        "filename": safe_filename,
        "path": str(target_path),
        "url": f"/api/agent/captures/{safe_filename}",
        "size_kb": round(len(raw_bytes) / 1024, 1),
    }


async def get_agent_capture(filename: str) -> Any:
    """Serve a saved capture / attached image."""
    safe_filename = Path(filename).name
    target = CAPTURES_DIR / safe_filename
    if not target.is_file():
        # Check parent captures directory
        alt = Path("/app/agent/captures") / safe_filename
        if alt.is_file():
            target = alt
        else:
            return JSONResponse({"error": "File not found"}, status_code=404)

    mime, _ = mimetypes.guess_type(str(target))
    return FileResponse(target, media_type=mime or "image/jpeg")


async def post_agent_snapshot(robot_id: str) -> Any:
    """Capture the current camera frame from robot_id and save as a chat-ready attachment."""
    app = _app()
    frame_tuple = app._camera_frames.get(robot_id)
    if not frame_tuple or not frame_tuple[0]:
        return JSONResponse({"error": f"No active camera frame available for robot '{robot_id}'"}, status_code=404)

    raw_jpeg = frame_tuple[0]
    filename = f"snapshot_{robot_id}_{int(time.time())}.jpg"
    target_path = CAPTURES_DIR / filename
    target_path.write_bytes(raw_jpeg)

    return {
        "ok": True,
        "robot_id": robot_id,
        "filename": filename,
        "path": str(target_path),
        "url": f"/api/agent/captures/{filename}",
        "size_kb": round(len(raw_jpeg) / 1024, 1),
    }


# ----------------------------------------------------------------- Cortex Chat & Streaming


def _find_agy_binary() -> Optional[str]:
    candidates = [
        os.environ.get("ANTIGRAVITY_AGENTAPI_EXE"),
        "/usr/local/bin/agy",
        "/home/sroy/.local/bin/agy",
        shutil.which("agy"),
    ]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def _get_workspace_dir() -> str:
    app = _app()
    if os.path.isdir("/workspace"):
        return "/workspace"
    if os.path.isdir("/app/server"):
        return "/app"
    return str(app.REPO)


async def get_agent_status() -> dict[str, Any]:
    agy_bin = _find_agy_binary()
    ws_dir = _get_workspace_dir()
    return {
        "name": "Cortex",
        "available": agy_bin is not None,
        "binary": agy_bin,
        "workspace": ws_dir,
        "agent_dir": "/app/agent",
        "model": "Gemini 3.7 Flash",
        "capabilities": [
            "multimodal_image_understanding",
            "robot_mentions",
            "slash_commands",
            "live_camera_inspection",
            "autonomous_navigation",
            "fleet_teleoperation",
            "live_code_editing",
        ],
        "skills": [
            "image-understanding",
            "navigation",
            "fleet-control",
            "code-ops",
        ],
    }


async def post_agent_chat(request: Request) -> Response:
    """Stream Cortex chat completions, multimodal image understanding, and tool calls via SSE."""
    app = _app()
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"error": "Invalid JSON body"}, status_code=400)

    user_prompt = str(body.get("prompt", "")).strip()
    if not user_prompt:
        return JSONResponse({"error": "'prompt' field is required"}, status_code=400)

    conversation_id = body.get("conversation_id")
    selected_robot = body.get("selected_robot")
    tagged_robot = body.get("tagged_robot")
    images_payload = body.get("images", [])

    agy_bin = _find_agy_binary()
    if not agy_bin:
        return JSONResponse({
            "error": "Antigravity CLI ('agy') binary was not found in environment."
        }, status_code=500)

    ws_dir = _get_workspace_dir()

    # Process and save any attached images
    attached_image_paths: list[str] = []
    if isinstance(images_payload, list):
        for idx, item in enumerate(images_payload):
            if isinstance(item, dict):
                d = item.get("data")
                name = item.get("filename") or item.get("name") or f"chat_img_{int(time.time())}_{idx}.png"
                if d:
                    if "," in d:
                        _, d = d.split(",", 1)
                    try:
                        b = base64.b64decode(d)
                        safe_name = re.sub(r"[^a-zA-Z0-9_.-]", "_", name)
                        p = CAPTURES_DIR / safe_name
                        p.write_bytes(b)
                        attached_image_paths.append(str(p))
                    except Exception:
                        pass
                elif item.get("path"):
                    attached_image_paths.append(str(item["path"]))
            elif isinstance(item, str):
                if os.path.exists(item):
                    attached_image_paths.append(item)

    # Check for @robot_id mentions in prompt
    robot_mentions = re.findall(r"@([a-zA-Z0-9_-]+)", user_prompt)
    effective_target = tagged_robot or (robot_mentions[0] if robot_mentions else selected_robot)

    # Build fleet snapshot
    fleet_snap = app.fleet_snapshot()
    fleet_summary = ", ".join([
        f"{r.get('robot_id')} ({r.get('robot_type')}, battery {int(r.get('battery', 0)*100)}%, pose: ({round(r.get('pose',{}).get('x',0),2)}, {round(r.get('pose',{}).get('y',0),2)}))"
        for r in fleet_snap
    ]) or "None"

    # Assemble system prefix
    images_info = ""
    if attached_image_paths:
        images_info = f"- Attached User Images:\n" + "\n".join([f"  * {p}" for p in attached_image_paths]) + "\n  (You can directly inspect these images visually with your `view_file` tool or run `python /app/agent/tools/vision.py inspect <path>`!)\n"

    system_prefix = (
        f"[SYSTEM CONTEXT: You are Cortex, the AI Fleet Intelligence, Perception Engine & Autonomous Operator embedded directly in SwarmDeck.\n"
        f"- Active Workspace Directory: {ws_dir}\n"
        f"- Agent Home Directory: /app/agent\n"
        f"- Connected Robots: {fleet_summary}\n"
        f"- Selected / Mentioned Robot: {effective_target or 'None'}\n"
        f"- SwarmDeck REST API: http://127.0.0.1:8080\n"
        f"{images_info}"
        f"- You have full multimodal vision understanding capabilities. When user attaches an image or asks about visual details, use `view_file` on the image path or run `python /app/agent/tools/vision.py inspect <path>`.\n"
        f"- You can control robots, inspect vision, and capture snapshots via:\n"
        f"  * `python /app/agent/tools/cortex_cli.py <subcommand>` (list, drive, navigate, cancel, stop, body, snap, inspect, see, detections)\n"
        f"  * `python /app/agent/tools/vision.py <subcommand>` (inspect, snapshot, see, detections)\n"
        f"  * `python /app/scripts/robot_tool.py <subcommand>`\n"
        f"- When the user asks to move a robot (e.g. 'move forward', 'turn left', '@aslan_0 drive'), immediately run the appropriate drive command with a duration.\n"
        f"- When the user asks 'what are you seeing on this robot' or asks for a photo/snapshot, run `python /app/agent/tools/vision.py snapshot <robot_id>` and inspect it with `view_file`.\n"
        f"- When the user asks to modify the UI or backend, edit the files directly.]\n\n"
    )

    full_prompt = f"{system_prefix}User Request: {user_prompt}"

    cmd = [
        agy_bin,
        "--add-dir", ws_dir,
        "--add-dir", "/app/agent",
        "-p", full_prompt,
        "--output-format", "stream-json",
        "--dangerously-skip-permissions",
    ]
    if conversation_id:
        cmd.extend(["--conversation", str(conversation_id)])

    async def sse_event_stream() -> AsyncGenerator[str, None]:
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                cwd=ws_dir,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except Exception as exc:
            yield f"data: {json.dumps({'type': 'error', 'error': str(exc)})}\n\n"
            return

        current_conv_id = conversation_id

        try:
            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                line_str = line.decode(errors="replace").strip()
                if not line_str:
                    continue

                try:
                    event_data = json.loads(line_str)
                except Exception:
                    continue

                event_type = event_data.get("event")

                if event_type == "init":
                    current_conv_id = event_data.get("conversation_id")
                    yield f"data: {json.dumps({'type': 'init', 'conversation_id': current_conv_id})}\n\n"

                elif event_type == "step_update":
                    su = event_data.get("step_update", {})
                    step_type = su.get("step_type")
                    state = su.get("state")

                    if step_type == "agent_response":
                        delta = su.get("text_delta")
                        if delta:
                            yield f"data: {json.dumps({'type': 'token', 'delta': delta})}\n\n"

                    elif step_type == "tool":
                        tool_name = su.get("tool_name")
                        tool_info = su.get("tool_info", {})
                        if state == "ACTIVE":
                            yield f"data: {json.dumps({'type': 'tool_call', 'tool': tool_name, 'params': tool_info.get('parameters', {})})}\n\n"
                        elif state == "DONE":
                            out = tool_info.get("output", "")
                            yield f"data: {json.dumps({'type': 'tool_output', 'tool': tool_name, 'output': out})}\n\n"

                elif event_type == "result":
                    res = event_data.get("result", {})
                    yield f"data: {json.dumps({'type': 'done', 'response': res.get('response', ''), 'status': res.get('status'), 'usage': res.get('usage', {})})}\n\n"

            await proc.wait()
        except asyncio.CancelledError:
            try:
                proc.kill()
            except Exception:
                pass
            raise
        except Exception as exc:
            yield f"data: {json.dumps({'type': 'error', 'error': str(exc)})}\n\n"

    return StreamingResponse(
        sse_event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )
