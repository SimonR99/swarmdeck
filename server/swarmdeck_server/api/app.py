"""FastAPI app: GUI websocket, adapter websocket, map endpoints.

The backend has no ROS import anywhere — acceptance criterion 12.
"""

from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from ..bus import bus, mark_session_start, session_elapsed, stamps
from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, MapService, map_service

@asynccontextmanager
async def lifespan(_: FastAPI):
    if not CONFIG:
        load_config()
    tasks = [
        asyncio.create_task(state_loop()),
        asyncio.create_task(map_loop()),
        asyncio.create_task(session_loop()),
    ]
    yield
    for t in tasks:
        t.cancel()


app = FastAPI(title="SwarmDeck", lifespan=lifespan)

REPO = Path(__file__).resolve().parents[3]
CONFIG: dict[str, Any] = {}
SESSION: dict[str, Any] = {"running": False, "name": None, "started_at": None, "recording": False}

_gui_clients: set[WebSocket] = set()
_alerts: dict[str, dict[str, Any]] = {}


# ----------------------------------------------------------------- config


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    global CONFIG, map_service
    p = Path(path) if path else REPO / "study" / "4robot.yaml"
    CONFIG = yaml.safe_load(p.read_text()) if p.exists() else {}

    mcfg = CONFIG.get("map", {}) or {}
    # Rebuild the service so resolution/extent follow the config.
    new_service = MapService(
        resolution=float(mcfg.get("resolution", 0.05)),
        size_m=float(mcfg.get("size_m", 30.0)),
    )
    new_service.set_mode(mcfg.get("merge_mode", "static"))
    for rid, pose in (mcfg.get("start_poses") or {}).items():
        new_service.set_transform(rid, pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0))

    map_service.__dict__.update(new_service.__dict__)
    return CONFIG


# ----------------------------------------------------------------- broadcast


async def broadcast(msg: dict[str, Any]) -> None:
    dead = []
    for ws in _gui_clients:
        try:
            await ws.send_json(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _gui_clients.discard(ws)


async def raise_alert(
    alert_id: str, level: str, kind: str, message: str, robot_id: str | None = None
) -> None:
    if alert_id in _alerts:
        return
    alert = {
        "id": alert_id,
        "level": level,
        "kind": kind,
        "robot_id": robot_id,
        "message": message,
        "t_wall": time.time(),
        "acknowledged": False,
    }
    _alerts[alert_id] = alert
    events.log("alert", {"alert": alert})
    await broadcast({"type": "alert", "alert": alert})


async def clear_alert(alert_id: str) -> None:
    if _alerts.pop(alert_id, None) is not None:
        await broadcast({"type": "alert_clear", "id": alert_id})


# ----------------------------------------------------------------- loops


async def state_loop() -> None:
    """5 Hz robot_state fan-out (FR-R3)."""
    threshold = float((CONFIG.get("alerts") or {}).get("unattended_threshold_s", 45))
    while True:
        await asyncio.sleep(0.2)
        for r in list(registry.robots.values()):
            await broadcast(r.to_state())

            aid = f"unattended_{r.robot_id}"
            if r.online and r.unattended_s > threshold:
                await raise_alert(
                    aid, "warn", "unattended",
                    f"{r.robot_id} unattended for {int(r.unattended_s)} s", r.robot_id,
                )
            elif r.unattended_s <= threshold:
                await clear_alert(aid)

            did = f"disconnect_{r.robot_id}"
            if not r.online:
                await raise_alert(
                    did, "critical", "adapter_disconnect",
                    f"{r.robot_id} adapter disconnected", r.robot_id,
                )
            else:
                await clear_alert(did)


async def map_loop() -> None:
    """2 Hz patch emission — never re-sends the whole grid (NFR-6)."""
    while True:
        await asyncio.sleep(0.5)
        patch = map_service.take_patch()
        if patch:
            await broadcast(patch)


async def session_loop() -> None:
    while True:
        await asyncio.sleep(1.0)
        await broadcast(session_state())


def session_state() -> dict[str, Any]:
    return {
        "type": "session_state",
        "running": SESSION["running"],
        "name": SESSION["name"],
        "started_at": SESSION["started_at"],
        "elapsed_s": round(session_elapsed(), 1) if SESSION["running"] else 0.0,
        "recording": SESSION["recording"],
    }


# ----------------------------------------------------------------- REST


@app.get("/api/config")
async def get_config() -> dict[str, Any]:
    return {"config": CONFIG, "protocol": 1}


@app.get("/api/fleet")
async def get_fleet() -> dict[str, Any]:
    return {"robots": registry.snapshot()}


@app.get("/api/session")
async def get_session() -> dict[str, Any]:
    return session_state()


@app.post("/api/session/start")
async def start_session() -> dict[str, Any]:
    name = f"S_{CONFIG.get('name', 'session')}_{datetime.now():%Y%m%dT%H%M%S}"
    out = REPO / "sessions" / name
    events.open(out)
    (out / "manifest.json").write_text(
        json.dumps({"name": name, "config": CONFIG, "started": time.time()}, indent=2)
    )
    mark_session_start()
    SESSION.update(running=True, name=name, started_at=time.time(), recording=True)
    events.log("session_start", {"name": name})
    await broadcast(session_state())
    return session_state()


@app.post("/api/session/stop")
async def stop_session() -> dict[str, Any]:
    events.log("session_stop", {"name": SESSION["name"]})
    events.close()
    SESSION.update(running=False, recording=False)
    await broadcast(session_state())
    return session_state()


@app.get("/api/map")
async def get_map() -> Response:
    return Response(
        content=map_service.as_png(),
        media_type="image/png",
        headers={"Cache-Control": "no-cache", "X-Map-Seq": str(map_service.seq)},
    )


@app.get("/api/map/status")
async def get_map_status() -> dict[str, Any]:
    """Merge mode, per-robot transforms, and registration quality (FR-M6)."""
    return map_service.status()


@app.get("/api/map/info")
async def get_map_info() -> dict[str, Any]:
    return {"type": "map_info", "info": map_service.meta.as_dict(map_service.seq)}


@app.post("/api/adapter/map")
async def post_map(request: Request) -> dict[str, Any]:
    """Adapter uploads its occupancy grid (zlib int8, row-major)."""
    import zlib

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    meta = GridMeta(
        resolution=float(request.query_params.get("resolution", 0.05)),
        width=int(request.query_params.get("width", 0)),
        height=int(request.query_params.get("height", 0)),
        origin_x=float(request.query_params.get("origin_x", 0.0)),
        origin_y=float(request.query_params.get("origin_y", 0.0)),
    )
    raw = zlib.decompress(await request.body())
    cells = np.frombuffer(raw, dtype=np.int8).reshape(meta.height, meta.width)
    map_service.ingest(rid, meta, cells)
    return {"ok": True, "cells": int(cells.size)}


# ----------------------------------------------------------------- GUI socket


@app.websocket("/ws")
async def gui_socket(ws: WebSocket) -> None:
    await ws.accept()
    _gui_clients.add(ws)
    try:
        await ws.send_json({"type": "map_info", "info": map_service.meta.as_dict(map_service.seq)})
        await ws.send_json({"type": "fleet_change", "robots": registry.snapshot()})
        await ws.send_json(session_state())
        for a in _alerts.values():
            await ws.send_json({"type": "alert", "alert": a})

        while True:
            msg = json.loads(await ws.receive_text())
            await handle_gui_message(msg)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"[gui] socket error: {exc}")
    finally:
        _gui_clients.discard(ws)


async def handle_gui_message(msg: dict[str, Any]) -> None:
    kind = msg.get("type", "")
    rid = msg.get("robot_id", "")

    # Every operator action is logged before it takes effect.
    events.log(kind, {k: v for k, v in msg.items() if k != "type"})
    if rid:
        registry.attend(rid)

    if kind == "set_goal":
        goal = msg.get("payload") or {}
        if not registry.can(rid, "navigate"):
            return
        taken_by = registry.goal_taken(goal, exclude=rid)
        if taken_by:
            await raise_alert(
                f"dupgoal_{rid}", "warn", "fault",
                f"Goal already assigned to {taken_by}", rid,
            )
            return
        await registry.send(rid, {"type": "navigate_to", "goal": goal, **stamps()})

    elif kind == "cancel_goal":
        await registry.send(rid, {"type": "cancel_goal", **stamps()})

    elif kind == "stop_all":
        for robot_id in list(registry.robots):
            await registry.send(robot_id, {"type": "stop", **stamps()})
        await raise_alert("stop_all", "critical", "fault", "STOP ALL issued by operator")

    elif kind == "acknowledge_alert":
        aid = msg.get("id", "")
        if aid in _alerts:
            _alerts[aid]["acknowledged"] = True
        await clear_alert(aid)

    elif kind in ("select_robots", "switch_camera", "report_target"):
        pass  # logged above; no robot-side effect


# ----------------------------------------------------------------- adapter socket


@app.websocket("/adapter")
async def adapter_socket(ws: WebSocket) -> None:
    await ws.accept()
    robot_id: str | None = None
    try:
        while True:
            msg = json.loads(await ws.receive_text())
            kind = msg.get("type", "")

            if kind == "hello":
                if msg.get("protocol") != 1:
                    await ws.close(code=4400)
                    return
                r = registry.hello(msg, ws)
                robot_id = r.robot_id
                await ws.send_json({"type": "hello_ack", "robot_id": robot_id})
                await broadcast({"type": "fleet_change", "robots": registry.snapshot()})
                events.log("adapter_connect", {"robot_id": robot_id, "adapter": r.adapter})

            elif kind == "robot_state":
                registry.update_state(msg)

            elif kind == "detections":
                for item in msg.get("items", []):
                    det = {
                        "id": f"{msg['robot_id']}_{item.get('class')}_{int(time.time()*10)}",
                        "class": item.get("class", "object"),
                        "score": item.get("score", 0.0),
                        "robot_id": msg["robot_id"],
                        "camera": msg.get("camera", "front"),
                        "bbox": item.get("bbox"),
                        "map_position": item.get("map_position"),
                        "first_seen": time.time(),
                        "last_seen": time.time(),
                        "observations": 1,
                    }
                    await broadcast({"type": "detection", "detection": det})

            elif kind == "map_meta":
                pass  # metadata accompanies the HTTP upload

            # Unknown types are ignored, not fatal (protocol rule 3).
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"[adapter] socket error: {exc}")
    finally:
        if robot_id:
            registry.disconnect(robot_id)
            events.log("adapter_disconnect", {"robot_id": robot_id})
