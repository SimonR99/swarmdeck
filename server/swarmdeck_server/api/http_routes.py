"""HTTP route bindings and request adaptation for the server services."""

from __future__ import annotations

from typing import Any
from uuid import UUID

from fastapi import APIRouter, Request, Response

router = APIRouter()

# ----------------------------------------------------------------- REST


@router.get("/api/config")
async def get_config() -> dict[str, Any]:
    from .control_routes import get_config as handler

    return await handler()


@router.get("/api/settings")
async def get_settings() -> dict[str, Any]:
    from .control_routes import get_settings as handler

    return await handler()


@router.get("/api/detection/classes")
async def get_detection_classes() -> dict[str, Any]:
    from .control_routes import get_detection_classes as handler

    return await handler()


@router.put("/api/settings")
async def put_settings(request: Request) -> dict[str, Any]:
    from .control_routes import put_settings as handler

    return await handler(request)


@router.get("/api/fleet")
async def get_fleet() -> dict[str, Any]:
    from .control_routes import get_fleet as handler

    return await handler()


@router.delete("/api/fleet/{robot_id}")
async def delete_fleet_robot(robot_id: str) -> dict[str, Any]:
    from .control_routes import delete_fleet_robot as handler

    return await handler(robot_id)


@router.post("/api/robot/{robot_id}/drive")
async def post_robot_drive(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_drive as handler

    return await handler(robot_id, request)


@router.post("/api/robot/{robot_id}/goal")
async def post_robot_goal(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_goal as handler

    return await handler(robot_id, request)


@router.post("/api/robot/{robot_id}/cancel")
async def post_robot_cancel(robot_id: str) -> Any:
    from .teleop_routes import post_robot_cancel as handler

    return await handler(robot_id)


@router.post("/api/robot/{robot_id}/stop")
async def post_robot_stop(robot_id: str) -> Any:
    from .teleop_routes import post_robot_stop as handler

    return await handler(robot_id)


@router.post("/api/robot/{robot_id}/body")
async def post_robot_body(robot_id: str, request: Request) -> Any:
    from .teleop_routes import post_robot_body as handler

    return await handler(robot_id, request)


@router.get("/api/robot/{robot_id}/vision")
async def get_robot_vision(robot_id: str) -> Any:
    from .teleop_routes import get_robot_vision as handler

    return await handler(robot_id)


@router.get("/api/detections")
async def get_all_detections() -> Any:
    from .teleop_routes import get_all_detections as handler

    return await handler()


@router.get("/api/session")
async def get_session() -> dict[str, Any]:
    from .control_routes import get_session as handler

    return await handler()


@router.post("/api/session/start")
async def start_session() -> dict[str, Any]:
    from .control_routes import start_session as handler

    return await handler()


@router.post("/api/session/stop")
async def stop_session() -> dict[str, Any]:
    from .control_routes import stop_session as handler

    return await handler()


@router.post("/api/sim/reset")
async def post_sim_reset(request_id: UUID | None = None) -> dict[str, Any]:
    from .control_routes import post_sim_reset as handler

    return await handler(str(request_id) if request_id is not None else None)


@router.get("/api/sim/reset")
async def get_sim_reset() -> dict[str, Any]:
    from .control_routes import get_sim_reset as handler

    return await handler()


@router.get("/api/map/status")
async def get_map_status() -> dict[str, Any]:
    from .map_routes import get_map_status as handler

    return await handler()


@router.post("/api/map/reset/{robot_id}")
async def reset_robot_map(robot_id: str, request_id: str | None = None) -> Response:
    from .map_routes import reset_robot_map as handler

    return await handler(robot_id, request_id)


@router.get("/api/map/reset/{robot_id}")
async def get_robot_map_reset(robot_id: str, request_id: str | None = None) -> Response:
    from .map_routes import get_robot_map_reset as handler

    return await handler(robot_id, request_id)


@router.post("/api/map/reset")
async def reset_all_maps() -> Response:
    from .map_routes import reset_all_maps as handler

    return await handler()


@router.get("/api/map/optimized")
async def get_optimized_index() -> dict[str, Any]:
    from .map_routes import get_optimized_index as handler

    return await handler()


@router.get("/api/map/optimized/{scope}")
async def get_optimized_map(scope: str, request: Request) -> Response:
    from .map_routes import get_optimized_map as handler

    return await handler(scope, request.headers.get("if-none-match"))


@router.get("/api/map/gaussians")
async def get_gaussians(request: Request) -> Response:
    from .reconstruction_routes import get_gaussians as handler

    return await handler(request)
