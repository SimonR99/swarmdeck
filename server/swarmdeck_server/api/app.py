"""FastAPI app: GUI websocket, adapter websocket, map endpoints.

The backend has no ROS import anywhere — acceptance criterion 12.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import time
from contextlib import asynccontextmanager
from typing import Any
from uuid import UUID, uuid4

from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from .deployment_raster import deployment_raster_loop
from . import state


@asynccontextmanager
async def lifespan(_: FastAPI):
    if not state.CONFIG:
        state.load_config()
    state.settings_store.load()
    state.apply_review_radii(state.settings_store.value)
    state.load_review()
    tasks = [
        asyncio.create_task(state.state_loop()),
        asyncio.create_task(state.network_loop()),
        asyncio.create_task(state.session_loop()),
        asyncio.create_task(deployment_raster_loop()),
    ]
    yield
    for task in tasks:
        task.cancel()


from .map_routes import CachedEpochStore, command_guard

state.registry.epoch_store = CachedEpochStore
state.registry.command_guard = command_guard

app = FastAPI(title="SwarmDeck", lifespan=lifespan)

from .autonomy_routes import router as autonomy_router
from .replica_views import router as replica_views_router

from .gui_socket import router as gui_router

app.include_router(gui_router)
from .adapter_socket import router as adapter_router

app.include_router(adapter_router)
app.include_router(autonomy_router)
app.include_router(replica_views_router)

# ----------------------------------------------------------------- REST


@app.get("/api/config")
async def get_config() -> dict[str, Any]:
    from .control_routes import get_config as handler

    return await handler()


@app.get("/api/settings")
async def get_settings() -> dict[str, Any]:
    from .control_routes import get_settings as handler

    return await handler()


@app.get("/api/detection/classes")
async def get_detection_classes() -> dict[str, Any]:
    from .control_routes import get_detection_classes as handler

    return await handler()


@app.put("/api/settings")
async def put_settings(request: Request) -> dict[str, Any]:
    from .control_routes import put_settings as handler

    return await handler(request)


@app.get("/api/fleet")
async def get_fleet() -> dict[str, Any]:
    from .control_routes import get_fleet as handler

    return await handler()


@app.delete("/api/fleet/{robot_id}")
async def delete_fleet_robot(robot_id: str) -> dict[str, Any]:
    from .control_routes import delete_fleet_robot as handler

    return await handler(robot_id)


@app.post("/api/fleet/{robot_id}/discard")
async def post_discard_fleet_robot(robot_id: str) -> dict[str, Any]:
    from .control_routes import delete_fleet_robot as handler

    return await handler(robot_id)


@app.get("/api/agent/status")
async def get_agent_status() -> dict[str, Any]:
    from .agent_routes import get_agent_status as handler

    return await handler()


@app.post("/api/agent/chat")
async def post_agent_chat(request: Request) -> Response:
    from .agent_routes import post_agent_chat as handler

    return await handler(request)


@app.post("/api/agent/upload")
async def post_agent_upload(request: Request) -> Any:
    from .agent_routes import post_agent_upload as handler

    return await handler(request)


@app.get("/api/agent/captures/{filename}")
async def get_agent_capture(filename: str) -> Any:
    from .agent_routes import get_agent_capture as handler

    return await handler(filename)


@app.post("/api/agent/snapshot/{robot_id}")
async def post_agent_snapshot(robot_id: str) -> Any:
    from .agent_routes import post_agent_snapshot as handler

    return await handler(robot_id)


@app.post("/api/robot/{robot_id}/drive")
async def post_robot_drive(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_drive as handler

    return await handler(robot_id, request)


@app.post("/api/robot/{robot_id}/goal")
async def post_robot_goal(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_goal as handler

    return await handler(robot_id, request)


@app.post("/api/robot/{robot_id}/cancel")
async def post_robot_cancel(robot_id: str) -> Any:
    from .teleop_routes import post_robot_cancel as handler

    return await handler(robot_id)


@app.post("/api/robot/{robot_id}/stop")
async def post_robot_stop(robot_id: str) -> Any:
    from .teleop_routes import post_robot_stop as handler

    return await handler(robot_id)


@app.post("/api/robot/{robot_id}/body")
async def post_robot_body(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_body as handler

    return await handler(robot_id, request)


@app.get("/api/robot/{robot_id}/vision")
async def get_robot_vision(robot_id: str) -> Any:
    from .teleop_routes import get_robot_vision as handler

    return await handler(robot_id)


@app.get("/api/detections")
async def get_all_detections() -> Any:
    from .teleop_routes import get_all_detections as handler

    return await handler()


@app.get("/api/session")
async def get_session() -> dict[str, Any]:
    from .control_routes import get_session as handler

    return await handler()


@app.post("/api/session/start")
async def start_session() -> dict[str, Any]:
    from .control_routes import start_session as handler

    return await handler()


@app.post("/api/session/stop")
async def stop_session() -> dict[str, Any]:
    from .control_routes import stop_session as handler

    return await handler()


@app.post("/api/sim/reset")
async def post_sim_reset(request_id: UUID | None = None) -> dict[str, Any]:
    from .control_routes import post_sim_reset as handler

    return await handler(str(request_id) if request_id is not None else None)


@app.get("/api/sim/reset")
async def get_sim_reset() -> dict[str, Any]:
    from .control_routes import get_sim_reset as handler

    return await handler()


@app.get("/api/map/status")
async def get_map_status() -> dict[str, Any]:
    from .map_routes import get_map_status as handler

    return await handler()


@app.post("/api/map/reset/{robot_id}")
async def reset_robot_map(robot_id: str, request_id: str | None = None) -> Response:
    from .map_routes import reset_robot_map as handler

    return await handler(robot_id, request_id)


@app.get("/api/map/reset/{robot_id}")
async def get_robot_map_reset(robot_id: str, request_id: str | None = None) -> Response:
    from .map_routes import get_robot_map_reset as handler

    return await handler(robot_id, request_id)


@app.post("/api/map/reset")
async def reset_all_maps() -> Response:
    from .map_routes import reset_all_maps as handler

    return await handler()


@app.get("/api/map/optimized")
async def get_optimized_index() -> dict[str, Any]:
    from .map_routes import get_optimized_index as handler

    return await handler()


@app.get("/api/map/optimized/{scope}")
async def get_optimized_map(scope: str, request: Request) -> Response:
    from .map_routes import get_optimized_map as handler

    return await handler(scope, request.headers.get("if-none-match"))


@app.get("/api/map/gaussians")
async def get_gaussians(request: Request) -> Response:
    from .reconstruction_routes import get_gaussians as handler

    return await handler(request)


@app.post("/api/adapter/camera")
async def post_camera(request: Request) -> Any:
    """Accept a throttled JPEG preview from an adapter.

    This is the ROS-free fallback when the low-latency WHEP pipeline is not
    installed. Adapters remain responsible for converting their native camera
    format into a browser-ready JPEG.
    """

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    if request.headers.get("content-type", "").split(";", 1)[0] != "image/jpeg":
        return JSONResponse({"error": "image/jpeg required"}, status_code=415)
    frame = await request.body()
    if not frame or len(frame) > 2_000_000 or not frame.startswith(b"\xff\xd8"):
        return JSONResponse({"error": "invalid JPEG frame"}, status_code=400)

    state._camera_seq += 1
    state._camera_frames[rid] = (frame, time.monotonic(), state._camera_seq)
    return {"ok": True, "bytes": len(frame), "seq": state._camera_seq}


@app.get("/api/camera/{robot_id}")
async def get_camera(robot_id: str) -> Response:
    current = state._camera_frames.get(robot_id)
    if current is None:
        return Response(status_code=404, headers={"Cache-Control": "no-store"})
    frame, received_at, seq = current
    return Response(
        content=frame,
        media_type="image/jpeg",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "X-Camera-Seq": str(seq),
            "X-Frame-Age-Ms": str(int((time.monotonic() - received_at) * 1000)),
        },
    )
