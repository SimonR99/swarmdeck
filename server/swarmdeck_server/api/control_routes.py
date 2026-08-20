"""REST handlers for configuration, settings, fleet, and sessions.

These handlers deliberately resolve the application collaborators at call time.
The FastAPI app remains the composition root, while this module keeps ordinary
HTTP policy out of the websocket/adapter protocol implementation and preserves
the existing ``api.app`` endpoint names through compatibility wrappers.
"""

from __future__ import annotations

import asyncio
import json
import time
from datetime import datetime
from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse


def _app():
    from . import app

    return app


async def get_config() -> dict[str, Any]:
    app = _app()
    return {
        "config": app.CONFIG,
        "settings": app.settings_store.value,
        "protocol": app.PROTOCOL_VERSION,
        "supported_protocols": list(app.SUPPORTED_PROTOCOLS),
    }


async def get_settings() -> dict[str, Any]:
    app = _app()
    return {"type": "settings_state", "settings": app.settings_store.value}


async def get_detection_classes() -> dict[str, Any]:
    app = _app()
    return {"classes": app.DETECTION_CLASSES}


async def put_settings(request: Request) -> dict[str, Any]:
    app = _app()
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    settings = app.settings_store.save(payload)
    app.discard_disabled_detections(settings)
    app.review_store.drop_classes(
        set(settings.get("detection_classes") or [])
        if settings.get("detection_enabled", True)
        else set()
    )
    app.apply_review_radii(settings)
    app.save_review(force=True)
    revised = app.reapply_detection_floors(settings)
    await asyncio.to_thread(
        app.map_service.set_excluded, app.disabled_robot_ids(settings)
    )
    for rid in app.disabled_robot_ids(settings):
        await app.registry.send(rid, {"type": "cancel_goal", **app.stamps()})
        await app.registry.send(
            rid, {"type": "drive", "linear": 0.0, "angular": 0.0, **app.stamps()}
        )
        robot = app.registry.robots.get(rid)
        if robot is not None:
            robot.goal = None
            if robot.nav_status == "active":
                robot.nav_status = "idle"
    app.events.log("settings_update", {"settings": settings})
    message = {"type": "settings_state", "settings": settings}
    await app.broadcast(message)
    for detection in revised:
        await app.broadcast({"type": "detection", "detection": detection})
    await app.broadcast_review()
    return message


async def get_fleet() -> dict[str, Any]:
    return {"robots": _app().fleet_snapshot()}


async def get_session() -> dict[str, Any]:
    return _app().session_state()


async def start_session() -> dict[str, Any]:
    app = _app()
    name = f"S_{app.CONFIG.get('name', 'session')}_{datetime.now():%Y%m%dT%H%M%S}"
    out = app.REPO / "sessions" / name
    app.events.open(out)
    (out / "manifest.json").write_text(
        json.dumps({"name": name, "config": app.CONFIG, "started": time.time()}, indent=2)
    )
    app.mark_session_start()
    app.SESSION.update(running=True, name=name, started_at=time.time(), recording=True)
    app.events.log("session_start", {"name": name})
    await app.broadcast(app.session_state())
    return app.session_state()


async def stop_session() -> dict[str, Any]:
    app = _app()
    app.events.log("session_stop", {"name": app.SESSION["name"]})
    app.events.close()
    app.SESSION.update(running=False, recording=False)
    await app.broadcast(app.session_state())
    return app.session_state()


async def post_sim_reset() -> dict[str, Any]:
    return await _app().reset_fleet()
