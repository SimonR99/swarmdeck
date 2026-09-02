"""Cortex Agent FastAPI Server (Port 8085).

Provides:
- Streaming SSE Chat endpoint with codebase modification and fleet tools
- Image upload and multimodal attachment endpoint
- Skills & slash command registry
- Dedicated agent status & diagnostics
"""

from __future__ import annotations

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

from .events import encode_sse
from .fleet import query_fleet
from .providers import ProviderRequest, find_agy_binary, get_provider
from .skills import get_skills_dict
from .supervisor import SupervisorRequest, build_supervisor
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


def get_actual_workspace() -> str:
    if os.path.isdir("/home/sroy/workspaces/swarmdeck"):
        return "/home/sroy/workspaces/swarmdeck"
    if os.path.isdir("/workspace"):
        return "/workspace"
    if os.path.isdir("/app/server"):
        return "/app"
    return WORKSPACE_DIR


HISTORY_DIR = Path(os.environ.get("CORTEX_HISTORY_DIR", "/app/sessions/cortex_history"))
SUPERVISOR = build_supervisor(HISTORY_DIR)


def init_ssh_environment() -> None:
    """Ensure /root/.ssh has correct ownership and 0600/0700 permissions."""
    ssh_host = Path("/root/.ssh_host")
    ssh_root = Path("/root/.ssh")
    try:
        if ssh_host.is_dir():
            ssh_root.mkdir(mode=0o700, parents=True, exist_ok=True)
            for item in ssh_host.iterdir():
                if item.is_file():
                    dest = ssh_root / item.name
                    shutil.copy2(item, dest)
                    dest.chmod(0o600)
            print("[Cortex] Initialized /root/.ssh from /root/.ssh_host.")
        elif ssh_root.is_dir():
            ssh_root.chmod(0o700)
            for item in ssh_root.iterdir():
                if item.is_file():
                    item.chmod(0o600)
    except Exception as exc:
        print(f"[Cortex] Notice: Could not initialize SSH files: {exc}")


init_ssh_environment()


def ensure_history_dir() -> Path:
    HISTORY_DIR.mkdir(parents=True, exist_ok=True)
    return HISTORY_DIR


@app.get("/health")
@app.get("/api/agent/status")
async def get_status() -> Dict[str, Any]:
    try:
        provider = get_provider()
        provider_status = provider.status()
        provider_error = None
    except Exception as exc:
        provider_status = {
            "name": os.environ.get("CORTEX_PROVIDER", "agy"),
            "available": False,
        }
        provider_error = str(exc)
    ws = get_actual_workspace()
    robots = query_fleet(SERVER_URL)
    agy_binary = find_agy_binary()
    supervisor_status = SUPERVISOR.status()
    return {
        "status": "healthy" if provider_status.get("available") else "degraded",
        "name": "Cortex",
        "version": "0.1.0",
        "provider": provider_status,
        "provider_error": provider_error,
        # Backward-compatible fields for older dashboards.
        "agy_available": agy_binary is not None,
        "agy_binary": agy_binary,
        "workspace": ws,
        "server_url": SERVER_URL,
        "fleet_count": len(robots),
        "model": provider_status.get("model", "unknown"),
        "supervisor": supervisor_status,
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


def build_system_prompt(
    *,
    workspace: str,
    fleet_summary: str,
    target_robot: Optional[str],
    attachment_text: str = "",
) -> str:
    """Build the stable operator contract shared by every agent provider."""
    return (
        "[SYSTEM CONTEXT: You are CORTEX, the AI Fleet Intelligence & Operator "
        "Assistant for SwarmDeck.\n"
        f"- Active Workspace Directory: {workspace}\n"
        f"- Connected Fleet Snapshot: {fleet_summary}\n"
        f"- Target / Mentioned Robot: {target_robot or 'Fleet-wide'}\n"
        f"- SwarmDeck Server URL: {SERVER_URL}\n"
        f"{attachment_text}"
        "\nSWARMDECK OVERVIEW:\n"
        "- SwarmDeck coordinates ground and legged robots, mapping, video, and telemetry.\n"
        "- Services: server:8080 (fleet), slam:8090, agent:8085, ui:5173, "
        "mediamtx:8554 (RTSP), and zenoh:7447.\n"
        "- Deployment profiles: spot, aslan, botman, scout (robot id tars_0), "
        "asimov, and all.\n"
        "\nUSE THE PURPOSE-BUILT ROBOT TOOL FIRST:\n"
        "- Fleet/service health: `python scripts/robot_tool.py doctor all --services`.\n"
        "- One robot or missing video: `python scripts/robot_tool.py doctor <robot> "
        "--services`. This checks telemetry, camera frames, RTSP publication and "
        "progressing media packets, "
        "SSH, and the profile's required containers in one command.\n"
        "- Telemetry only: `python scripts/robot_tool.py list`.\n"
        "- Vision/detections: `python scripts/robot_tool.py see <robot_id>`.\n"
        "- Deploy/restart only when explicitly requested: `python "
        "scripts/robot_tool.py deploy <profile>`. Use profile `scout` for tars_0.\n"
        "- Navigation: `python scripts/robot_tool.py navigate <robot_id> --x <x> --y <y>`.\n"
        "- Timed drive: `python scripts/robot_tool.py drive <robot_id> --linear "
        "<m/s> --angular <rad/s> --duration <sec>`.\n"
        "- Emergency stop: `python scripts/robot_tool.py stop <robot_id|all>`.\n"
        f"- Code work: inspect, edit, and test files under {workspace}.\n"
        "\nEVIDENCE AND OPERATION RULES:\n"
        "- Treat the fleet snapshot above as context, not proof of current health; run "
        "the relevant command for the user's request.\n"
        "- Never claim all services are healthy from telemetry alone. A robot is online "
        "only when the tool says online; a stored pose does not prove liveness.\n"
        "- Never claim dashboard video is live from an RTSP DESCRIBE/200 response alone. "
        "That proves publication metadata. Require a successful progressing media-packet "
        "probe, and state that browser/WebRTC decoding remains unverified.\n"
        "- Deployment output distinguishes network and SSH authentication failures. Do "
        "not relabel an authentication failure as an unreachable robot.\n"
        "- Prefer one diagnostic command over chains of ad-hoc `ssh`, `find`, `ps`, or "
        "socket probes. Escalate to lower-level inspection only for the failing layer.\n"
        "- Diagnose before editing. After a repair, rerun the same check and base the "
        "success statement on its result. Do not change global Git/SSH configuration.\n"
        "\nOPERATOR COMMUNICATION:\n"
        "- Keep the final answer simple, brief, and direct (normally 1-3 sentences).\n"
        "- Separate confirmed facts from remaining uncertainty.\n"
        "- Confirm requested actions clearly, but never announce success before the "
        "verification command succeeds.]\n\n"
    )


@app.post("/api/agent/chat")
async def post_chat(req: ChatRequest) -> Response:
    user_prompt = req.prompt.strip()
    if not user_prompt:
        return JSONResponse({"error": "Prompt cannot be empty"}, status_code=400)

    try:
        provider = get_provider()
        provider_status = provider.status()
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=500)
    if not provider_status.get("available"):
        return JSONResponse(
            {"error": f"Cortex provider '{provider_status.get('name')}' is unavailable"},
            status_code=503,
        )

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
        state = "online" if r.get("online") is True else "OFFLINE/stale"
        batt = f"{int(r.get('battery', 0.0) * 100)}%" if r.get("battery") is not None else "N/A"
        pose = r.get("pose") or {}
        px, py = pose.get("x", 0.0), pose.get("y", 0.0)
        fleet_summary_list.append(
            f"{rid} ({state}, {rtype}, battery {batt}, pose ({px:.2f}, {py:.2f}))"
        )
    fleet_summary = "; ".join(fleet_summary_list) or "No robots connected"

    # Attachments context
    attachment_text = ""
    if req.attachments:
        attachment_lines = []
        for att in req.attachments:
            p = att.get("path") or att.get("filename")
            attachment_lines.append(f"- Attached image file: {p}")
        attachment_text = "\nAttached Media/Images:\n" + "\n".join(attachment_lines) + "\n(You can use view_file to inspect image details or pixel contents.)\n"

    system_prefix = build_system_prompt(
        workspace=ws_dir,
        fleet_summary=fleet_summary,
        target_robot=target_robot,
        attachment_text=attachment_text,
    )

    full_prompt = f"{system_prefix}User Request: {user_prompt}"

    async def sse_event_stream() -> AsyncGenerator[str, None]:
        process_request = ProviderRequest(
            prompt=full_prompt,
            workspace=ws_dir,
            conversation_id=req.conversation_id,
        )
        supervisor_request = SupervisorRequest(
            provider_request=process_request,
            operator_prompt=user_prompt,
            provider_name=str(provider_status.get("name") or provider.name),
            selected_robot=target_robot,
        )
        async for event in SUPERVISOR.run(provider, supervisor_request):
            yield encode_sse(event)

    return StreamingResponse(
        sse_event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )
