"""REST handlers for configuration, settings, fleet, and sessions."""

from __future__ import annotations

import json
import time
from datetime import datetime
from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse


from . import state


async def get_config() -> dict[str, Any]:
    return {
        "config": state.CONFIG,
        "settings": state.settings_store.value,
        "protocol": state.PROTOCOL_VERSION,
        "supported_protocols": list(state.SUPPORTED_PROTOCOLS),
    }


async def get_settings() -> dict[str, Any]:
    return {"type": "settings_state", "settings": state.settings_store.value}


async def get_detection_classes() -> dict[str, Any]:
    return {"classes": state.DETECTION_CLASSES}


async def put_settings(request: Request) -> dict[str, Any]:
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    settings = state.settings_store.save(payload)
    state.discard_disabled_detections(settings)
    state.review_store.drop_classes(
        set(settings.get("detection_classes") or [])
        if settings.get("detection_enabled", True)
        else set()
    )
    state.apply_review_radii(settings)
    state.save_review(force=True)
    revised = state.reapply_detection_floors(settings)
    for rid in state.disabled_robot_ids(settings):
        await state.registry.send(rid, {"type": "cancel_goal", **state.stamps()})
        await state.registry.send(
            rid, {"type": "drive", "linear": 0.0, "angular": 0.0, **state.stamps()}
        )
        robot = state.registry.robots.get(rid)
        if robot is not None:
            robot.goal = None
            if robot.nav_status == "active":
                robot.nav_status = "idle"
    state.events.log("settings_update", {"settings": settings})
    message = {"type": "settings_state", "settings": settings}
    await state.broadcast(message)
    for detection in revised:
        await state.broadcast({"type": "detection", "detection": detection})
    await state.broadcast_review()
    return message


async def get_fleet() -> dict[str, Any]:
    return {"robots": state.fleet_snapshot()}


async def delete_fleet_robot(robot_id: str) -> dict[str, Any]:
    robot = state.registry.robots.get(robot_id)
    if robot is None:
        return JSONResponse({"error": f"Robot {robot_id} not found"}, status_code=404)
    state.registry.disconnect(robot_id)
    state.registry.remove(robot_id)
    if hasattr(state.map_service, "reset_robot_async"):
        await state.map_service.reset_robot_async(robot_id)
    state.events.log("robot_discarded", {"robot_id": robot_id})
    await state.broadcast({"type": "fleet_change", "robots": state.fleet_snapshot()})
    return {"ok": True, "robot_id": robot_id}


async def get_session() -> dict[str, Any]:
    return state.session_state()


async def start_session() -> dict[str, Any]:
    name = f"S_{state.CONFIG.get('name', 'session')}_{datetime.now():%Y%m%dT%H%M%S}"
    out = state.REPO / "sessions" / name
    state.events.open(out)
    (out / "manifest.json").write_text(
        json.dumps(
            {"name": name, "config": state.CONFIG, "started": time.time()}, indent=2
        )
    )
    state.mark_session_start()
    state.SESSION.update(
        running=True, name=name, started_at=time.time(), recording=True
    )
    state.events.log("session_start", {"name": name})
    await state.broadcast(state.session_state())
    return state.session_state()


async def stop_session() -> dict[str, Any]:
    state.events.log("session_stop", {"name": state.SESSION["name"]})
    state.events.close()
    state.SESSION.update(running=False, recording=False)
    await state.broadcast(state.session_state())
    return state.session_state()


async def post_sim_reset(request_id: str | None = None) -> dict[str, Any]:
    # reset_fleet owns both implementations: the legacy adapter handshake and
    # the host-supervisor request plus its fleet-wide progress broadcast. Keep
    # REST and websocket reset commands on that single observable path.
    return await state.reset_fleet(request_id)


async def get_sim_reset() -> dict[str, Any]:
    from .simulation_reset import reset_root, reset_status

    root = reset_root()
    if root is None:
        return {"version": 1, "phase": "legacy", "ok": None}
    return reset_status(root)
