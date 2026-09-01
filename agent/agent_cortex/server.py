"""Cortex Agent FastAPI Server (Port 8085).

Provides:
- Streaming SSE Chat endpoint with codebase modification and fleet tools
- Image upload and multimodal attachment endpoint
- Skills & slash command registry
- Dedicated agent status & diagnostics
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import shutil
import time
import uuid
from pathlib import Path
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import FastAPI, File, Request, Response, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from .fleet import query_detections, query_fleet
from .skills import get_skills_dict
from .vision import ensure_upload_dir, save_image_bytes

app = FastAPI(title="SwarmDeck Cortex AI Service", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

WORKSPACE_DIR = os.environ.get("CORTEX_WORKSPACE", "/home/sroy/workspaces/swarmdeck")
SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "http://server:8080").rstrip("/")


def find_agy_binary() -> Optional[str]:
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


def get_actual_workspace() -> str:
    if os.path.isdir("/home/sroy/workspaces/swarmdeck"):
        return "/home/sroy/workspaces/swarmdeck"
    if os.path.isdir("/workspace"):
        return "/workspace"
    if os.path.isdir("/app/server"):
        return "/app"
    return WORKSPACE_DIR


HISTORY_DIR = Path(os.environ.get("CORTEX_HISTORY_DIR", "/app/sessions/cortex_history"))


def ensure_history_dir() -> Path:
    HISTORY_DIR.mkdir(parents=True, exist_ok=True)
    return HISTORY_DIR


@app.get("/health")
@app.get("/api/agent/status")
async def get_status() -> Dict[str, Any]:
    agy_bin = find_agy_binary()
    ws = get_actual_workspace()
    robots = query_fleet(SERVER_URL)
    return {
        "status": "healthy",
        "name": "Cortex",
        "version": "0.1.0",
        "agy_available": agy_bin is not None,
        "agy_binary": agy_bin,
        "workspace": ws,
        "server_url": SERVER_URL,
        "fleet_count": len(robots),
        "model": "Gemini 3.7 Flash",
    }


@app.get("/api/agent/skills")
async def get_skills() -> Dict[str, Any]:
    return {"skills": get_skills_dict()}


@app.get("/api/agent/threads")
async def list_threads() -> Dict[str, Any]:
    h_dir = ensure_history_dir()
    threads = []
    for f in h_dir.glob("*.json"):
        try:
            data = json.loads(f.read_text())
            threads.append({
                "id": data.get("id", f.stem),
                "title": data.get("title", "New Conversation"),
                "createdAt": data.get("createdAt", 0),
                "updatedAt": data.get("updatedAt", 0),
                "messageCount": len(data.get("messages", [])),
                "conversationId": data.get("conversationId"),
            })
        except Exception:
            continue
    threads.sort(key=lambda x: x.get("updatedAt", 0), reverse=True)
    return {"threads": threads}


@app.get("/api/agent/threads/{thread_id}")
async def get_thread(thread_id: str) -> Response:
    h_dir = ensure_history_dir()
    f = h_dir / f"{thread_id}.json"
    if not f.is_file():
        return JSONResponse({"error": "Thread not found"}, status_code=404)
    try:
        data = json.loads(f.read_text())
        return JSONResponse(data)
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=500)


@app.post("/api/agent/threads")
async def save_thread(req: Request) -> Dict[str, Any]:
    data = await req.json()
    thread_id = data.get("id")
    if not thread_id:
        thread_id = f"thread_{int(time.time())}_{uuid.uuid4().hex[:6]}"
        data["id"] = thread_id
    if "updatedAt" not in data:
        data["updatedAt"] = int(time.time() * 1000)

    h_dir = ensure_history_dir()
    f = h_dir / f"{thread_id}.json"
    f.write_text(json.dumps(data, indent=2))
    return {"ok": True, "thread": data}


@app.delete("/api/agent/threads/{thread_id}")
async def delete_thread(thread_id: str) -> Dict[str, Any]:
    h_dir = ensure_history_dir()
    f = h_dir / f"{thread_id}.json"
    if f.is_file():
        f.unlink()
    return {"ok": True, "deleted": thread_id}


@app.post("/api/agent/upload")
async def upload_image(file: UploadFile = File(...)) -> Dict[str, Any]:
    data = await file.read()
    if not data:
        return JSONResponse({"error": "Empty file uploaded"}, status_code=400)

    image_id, path, meta = save_image_bytes(data, file.filename or "upload.png")
    return {
        "ok": True,
        "image_id": image_id,
        "filename": meta.get("filename"),
        "path": str(path),
        "width": meta.get("width"),
        "height": meta.get("height"),
        "size_bytes": meta.get("size_bytes"),
        "url": f"/api/agent/image/{meta.get('filename')}",
    }


@app.get("/api/agent/image/{filename}")
async def get_image(filename: str) -> Response:
    upload_dir = ensure_upload_dir()
    file_path = upload_dir / filename
    if not file_path.is_file():
        return Response(status_code=404)
    ext = file_path.suffix.lower()
    media_type = "image/jpeg" if ext in (".jpg", ".jpeg") else "image/png"
    return FileResponse(file_path, media_type=media_type)


class ChatRequest(BaseModel):
    prompt: str
    conversation_id: Optional[str] = None
    selected_robot: Optional[str] = None
    attachments: Optional[List[Dict[str, Any]]] = None


@app.post("/api/agent/chat")
async def post_chat(req: ChatRequest) -> Response:
    user_prompt = req.prompt.strip()
    if not user_prompt:
        return JSONResponse({"error": "Prompt cannot be empty"}, status_code=400)

    agy_bin = find_agy_binary()
    if not agy_bin:
        return JSONResponse({
            "error": "Antigravity CLI ('agy') binary was not found in environment."
        }, status_code=500)

    ws_dir = get_actual_workspace()

    # Parse @robot mentions (e.g., @aslan_0, @botman_0, @tars_0)
    mentions = re.findall(r"@([a-zA-Z0-9_-]+)", user_prompt)
    target_robot = mentions[0] if mentions else req.selected_robot

    # Fleet telemetry context
    fleet_robots = query_fleet(SERVER_URL)
    fleet_summary_list = []
    for r in fleet_robots:
        rid = r.get("robot_id")
        rtype = r.get("robot_type")
        batt = f"{int(r.get('battery', 0.0) * 100)}%" if r.get("battery") is not None else "N/A"
        pose = r.get("pose") or {}
        px, py = pose.get("x", 0.0), pose.get("y", 0.0)
        fleet_summary_list.append(f"{rid} ({rtype}, battery {batt}, pose ({px:.2f}, {py:.2f}))")
    fleet_summary = "; ".join(fleet_summary_list) or "No robots connected"

    # Attachments context
    attachment_text = ""
    if req.attachments:
        attachment_lines = []
        for att in req.attachments:
            p = att.get("path") or att.get("filename")
            attachment_lines.append(f"- Attached image file: {p}")
        attachment_text = "\nAttached Media/Images:\n" + "\n".join(attachment_lines) + "\n(You can use view_file to inspect image details or pixel contents.)\n"

    system_prefix = (
        f"[SYSTEM CONTEXT: You are CORTEX, the AI Fleet Intelligence & Operator Assistant for SwarmDeck.\n"
        f"- Active Workspace Directory: {ws_dir}\n"
        f"- Connected Fleet: {fleet_summary}\n"
        f"- Target / Mentioned Robot: {target_robot or 'Fleet-wide'}\n"
        f"- SwarmDeck Server URL: {SERVER_URL}\n"
        f"{attachment_text}"
        f"\nSWARMDECK OVERVIEW (Scalable Multi-Robot Platform):\n"
        f"- SwarmDeck orchestrates autonomous ground and legged robots with real-time mapping (CSLAM), live video, and telemetry.\n"
        f"- Microservices: `server:8080` (core fleet supervisor), `slam:8090` (CSLAM optimizer), `agent:8085` (Cortex), `ui:5173` (dashboard), `mediamtx:8554` (video), `zenoh:7447` (robot communications).\n"
        f"- Supported Hardware Profiles (`deploy/robots/`): `spot` (Boston Dynamics Spot quadruped), `aslan` (AgileX Bunker UGV), `botman` (AgileX Bunker UGV), `tars` (AgileX Scout Mini UGV), or `all`.\n"
        f"\nCORE CAPABILITIES & TOOL RUNNERS:\n"
        f"1. ROBOT LIFECYCLE & DEPLOYMENT (Start / Restart / Deploy):\n"
        f"   - When the user asks to 'start spot', 'restart aslan', 'deploy botman', 'start all', etc.:\n"
        f"     Execute `make deploy ROBOT=<robot_name>` (e.g. `spot`, `aslan`, `botman`, `tars`, `all`) or `python scripts/robot_tool.py deploy <robot_name>`.\n"
        f"2. ROBOT FLEET COMMANDS & NAVIGATION GOALS:\n"
        f"   - Navigation Goals: When asked to navigate or set a goal to coordinates, objects, or relative distances:\n"
        f"     * To Coordinates: Run `python scripts/robot_tool.py navigate <robot_id> --x <x> --y <y>`.\n"
        f"     * To Objects/Landmarks (e.g., rubber duck, wooden block): Run `python scripts/robot_tool.py see <robot_id>` to get its (x, y), then run `python scripts/robot_tool.py navigate <robot_id> --x <x> --y <y>`.\n"
        f"     * Relative: Compute target from robot's pose (px + dist*cos(yaw), py + dist*sin(yaw)) and dispatch `navigate`.\n"
        f"     * Setting a navigation goal automatically places the target pin and planned path on the SwarmDeck map.\n"
        f"   - Velocity Driving: Run `python scripts/robot_tool.py drive <robot_id> --linear <m/s> --angular <rad/s> --duration <sec>`.\n"
        f"   - Vision Inspection: Run `python scripts/robot_tool.py see <robot_id>`.\n"
        f"   - Emergency Stop: Run `python scripts/robot_tool.py stop <robot_id | all>`.\n"
        f"3. CODEBASE MODIFICATION:\n"
        f"   - Read, edit, and test files in {ws_dir} using file tools.\n"
        f"\nOPERATOR COMMUNICATION RULES:\n"
        f"- The user/operator is NON-TECHNICAL. Keep every answer SIMPLE, BRIEF, and DIRECT (1-3 sentences maximum).\n"
        f"- Avoid technical jargon, long dissertations, or code dumps unless explicitly asked.\n"
        f"- Always confirm actions with clear, friendly status confirmations (e.g. '✓ Spot is restarting.', '✓ Navigation target set for Aslan.').]\n\n"
    )

    full_prompt = f"{system_prefix}User Request: {user_prompt}"

    cmd = [
        agy_bin,
        "--add-dir", ws_dir,
        "-p", full_prompt,
        "--output-format", "stream-json",
        "--dangerously-skip-permissions",
    ]
    if req.conversation_id:
        cmd.extend(["--conversation", str(req.conversation_id)])

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

        current_conv_id = req.conversation_id

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
