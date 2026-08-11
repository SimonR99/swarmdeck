"""FastAPI app: GUI websocket, adapter websocket, map endpoints.

The backend has no ROS import anywhere — acceptance criterion 12.
"""

from __future__ import annotations

import asyncio
import json
import math
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
from ..config.detection import DETECTION_CLASSES
from ..config.settings import SettingsStore
from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, MapService, map_service

# 2 adds one optional adapter message, `slam_graph`, carrying a robot's view of a
# collaborative pose graph. Purely additive: a protocol-1 adapter never sends it
# and is fully supported, which is the whole point of versioning it rather than
# redefining `hello`.
PROTOCOL_VERSION = 2
SUPPORTED_PROTOCOLS = (1, 2)


@asynccontextmanager
async def lifespan(_: FastAPI):
    if not CONFIG:
        load_config()
    settings_store.load()
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
settings_store = SettingsStore(REPO / "sessions" / "settings.json")
CONFIG: dict[str, Any] = {}
SESSION: dict[str, Any] = {"running": False, "name": None, "started_at": None, "recording": False}

_gui_clients: set[WebSocket] = set()
_alerts: dict[str, dict[str, Any]] = {}
# alert_id → wall-clock time until which raise_alert is a no-op (after acknowledge)
_alert_suppress_until: dict[str, float] = {}
_camera_frames: dict[str, tuple[bytes, float, int]] = {}
_detections: dict[str, dict[str, Any]] = {}
_camera_seq = 0

# How long to wait for adapters to report `reset_done` before clearing anyway. A
# reset restarts SLAM and re-zeroes an odometry filter on every robot; measured
# on the four-robot Gazebo stack the slow step is the Gazebo world reset itself.
# Generous, because the failure mode of waiting too little (clearing the map
# while adapters still hold the old one) is worse than the failure mode of
# waiting too long (a spinner stays up).
RESET_TIMEOUT_S = 25.0

# Robots that have been sent `reset` and have not yet answered. Mutated from the
# adapter socket, awaited by reset_fleet(); _reset_done fires when it empties.
_reset_pending: set[str] = set()
# robot_id → the `steps` map from a reset_done that reported ok: false. Held
# until reset_fleet() has finished clearing, so the alert survives that clear.
_reset_failures: dict[str, dict[str, Any]] = {}
# Created per reset, not at import. An asyncio.Event binds to the first loop
# that awaits it and then refuses every other one, so a module-level Event
# survives exactly one event loop — which is one more than a test suite gets,
# and a landmine for anything that ever runs this app under a second loop.
# `_reset_running` is a plain bool for the same reason: an asyncio.Lock would
# reintroduce the binding this avoids.
_reset_done: asyncio.Event | None = None
_reset_running = False


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
    # How members disagreeing about a cell is resolved. Default `majority`
    # so a stale obstacle one robot recorded is erased once others have
    # driven through it; `occupied` restores the old any-vote-wins rule.
    new_service.set_conflict_mode(mcfg.get("merge_conflict", "majority"))
    for rid, pose in (mcfg.get("start_poses") or {}).items():
        new_service.set_transform(rid, pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0))

    map_service.__dict__.update(new_service.__dict__)
    _camera_frames.clear()
    _detections.clear()
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
    until = _alert_suppress_until.get(alert_id)
    if until is not None:
        if time.time() < until:
            return
        _alert_suppress_until.pop(alert_id, None)
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


def suppress_alert(alert_id: str) -> None:
    """Prevent the same alert id from reappearing for alert_suppress_s seconds."""
    seconds = float(settings_store.value.get("alert_suppress_s", 30))
    if seconds <= 0 or not alert_id:
        return
    _alert_suppress_until[alert_id] = time.time() + seconds


# ----------------------------------------------------------------- loops


async def state_loop() -> None:
    """5 Hz robot_state fan-out (FR-R3)."""
    while True:
        await asyncio.sleep(0.2)
        threshold = float(settings_store.value["unattended_threshold_s"])
        for r in list(registry.robots.values()):
            await broadcast(robot_state(r))

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


def robot_state(robot: Any) -> dict[str, Any]:
    """Expose adapter-local state in the GUI's shared merged-map frame."""
    state = robot.to_state()
    if robot.coordinate_frame == "merged":
        return state
    # In `cslam` mode the collaborative back end already knows where this robot
    # is in the frame the map is drawn in, so use that answer rather than
    # transforming one out of the robot's own SLAM frame. Composing across two
    # independently optimised trajectories is precisely what put robots 8-18 m
    # from ground truth.
    direct = map_service.common_pose(robot.robot_id)
    if direct is not None:
        state["pose"] = dict(direct)
        if state["goal"]:
            state["goal"] = map_service.robot_to_world(robot.robot_id, state["goal"])
        state["planned_path"] = []
        return state
    state["pose"] = map_service.robot_to_world(robot.robot_id, state["pose"])
    if state["goal"]:
        state["goal"] = map_service.robot_to_world(robot.robot_id, state["goal"])
    state["planned_path"] = [
        map_service.robot_to_world(robot.robot_id, point)
        for point in state.get("planned_path", [])
    ]
    return state


def fleet_snapshot() -> list[dict[str, Any]]:
    return [robot_state(robot) for robot in registry.robots.values()]


def detection_position(robot_id: str, position: Any) -> dict[str, float] | None:
    """Normalize an adapter-local detection point into the merged-map frame."""
    if not isinstance(position, dict) or "x" not in position or "y" not in position:
        return None
    try:
        x, y = float(position["x"]), float(position["y"])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    normalized = {"x": x, "y": y}
    robot = registry.robots.get(robot_id)
    if robot is None or robot.coordinate_frame == "merged":
        return normalized
    return map_service.robot_to_world(robot_id, normalized)


# ----------------------------------------------------------------- reset


async def reset_fleet() -> dict[str, Any]:
    """Put the simulation back to its start state.

    Two halves that must happen in this order. The adapters reset the things only
    they can reach — the simulator's model poses, each robot's SLAM map, its
    odometry filter, its costmaps — and report `reset_done`. Only then does the
    backend drop what it derived from those robots. Clearing first would race: the
    server holds each robot's last uploaded grid, and an upload already in flight
    would restore the map a moment after it was cleared.

    Backend state is cleared even when an adapter never answers. A stuck adapter
    must not leave the operator staring at a map with a spinner over it forever;
    the robots that failed to confirm are named in the result instead, because a
    robot whose SLAM did not actually reset will push its old map back within a
    couple of seconds and the operator needs to know why.
    """
    global _reset_done, _reset_running

    if _reset_running:
        return {"ok": False, "error": "a reset is already running"}
    _reset_running = True
    _reset_done = asyncio.Event()
    try:
        # Capability-gated, and this is a safety boundary rather than a
        # nicety: `reset` means "teleport to spawn and forget the map", which a
        # physical robot cannot do and must never be asked to do. adapter_ros2
        # does not advertise it. See adapters/protocol/README.md.
        targets = {rid for rid, r in registry.robots.items() if "reset" in r.capabilities}
        skipped = sorted(set(registry.robots) - targets)

        events.log("reset_start", {"robots": sorted(targets), "skipped": skipped})
        await broadcast(
            {"type": "sim_reset", "phase": "start",
             "robots": sorted(targets), "skipped": skipped}
        )

        _reset_pending.clear()
        _reset_failures.clear()
        _reset_done.clear()
        # Wait only on robots the command actually reached. A robot whose socket
        # died between the capability check and the send would otherwise hold the
        # whole reset until the timeout.
        for rid in sorted(targets):
            if await registry.send(rid, {"type": "reset", **stamps()}):
                _reset_pending.add(rid)
        unreachable = sorted(targets - _reset_pending)
        if not _reset_pending:
            _reset_done.set()

        timed_out = False
        try:
            await asyncio.wait_for(_reset_done.wait(), RESET_TIMEOUT_S)
        except asyncio.TimeoutError:
            timed_out = True
        silent = sorted(_reset_pending)
        _reset_pending.clear()

        map_service.reset()
        _detections.clear()
        _camera_frames.clear()
        # Alerts describe a world that no longer exists — an `unattended` warning
        # for a robot now back at its spawn pose is stale by construction. The
        # suppression window goes too, so a condition that genuinely returns
        # after the reset is reported again rather than swallowed.
        for alert_id in list(_alerts):
            await clear_alert(alert_id)
        _alert_suppress_until.clear()
        for robot in registry.robots.values():
            robot.goal = None
            robot.planned_path = []
            robot.nav_status = "idle"
            robot.mode = "idle"

        # Three distinct ways to not be reset, kept apart because they mean
        # different things to an operator: the command never arrived, it arrived
        # and was never answered, or it was answered with a failure.
        partial = dict(_reset_failures)
        _reset_failures.clear()
        failed = sorted(set(silent) | set(unreachable) | set(partial))
        result = {
            "type": "sim_reset",
            "phase": "done",
            "ok": not failed,
            "reset": sorted(targets - set(failed)),
            "skipped": skipped,
            "unreachable": unreachable,
            "no_response": silent,
            "partial": {rid: steps for rid, steps in partial.items()},
            "failed": failed,
            "timed_out": timed_out,
        }
        events.log("reset_done", {k: v for k, v in result.items() if k != "type"})
        await broadcast(result)
        await broadcast({"type": "fleet_change", "robots": fleet_snapshot()})
        if failed:
            await raise_alert(
                "reset_incomplete", "warn", "fault",
                f"Reset not confirmed by {', '.join(failed)} — their map may return",
            )
        return result
    finally:
        _reset_running = False
        _reset_done = None


def goal_taken(goal: dict[str, float], exclude: str, tol: float = 0.5) -> str | None:
    """Find a duplicate goal after normalizing every robot to the shared frame."""
    for rid, robot in registry.robots.items():
        if rid == exclude or not robot.goal:
            continue
        current = (
            robot.goal
            if robot.coordinate_frame == "merged"
            else map_service.robot_to_world(rid, robot.goal)
        )
        if abs(current["x"] - goal["x"]) < tol and abs(current["y"] - goal["y"]) < tol:
            return rid
    return None


# ----------------------------------------------------------------- REST


@app.get("/api/config")
async def get_config() -> dict[str, Any]:
    return {
        "config": CONFIG,
        "settings": settings_store.value,
        "protocol": PROTOCOL_VERSION,
        "supported_protocols": list(SUPPORTED_PROTOCOLS),
    }


@app.get("/api/settings")
async def get_settings() -> dict[str, Any]:
    return {"type": "settings_state", "settings": settings_store.value}


@app.get("/api/detection/classes")
async def get_detection_classes() -> dict[str, Any]:
    """What the detector can be asked to look for, for the settings dialog."""
    return {"classes": DETECTION_CLASSES}


@app.put("/api/settings")
async def put_settings(request: Request) -> dict[str, Any]:
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    settings = settings_store.save(payload)
    events.log("settings_update", {"settings": settings})
    message = {"type": "settings_state", "settings": settings}
    await broadcast(message)
    return message


@app.get("/api/fleet")
async def get_fleet() -> dict[str, Any]:
    return {"robots": fleet_snapshot()}


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


@app.post("/api/sim/reset")
async def post_sim_reset() -> dict[str, Any]:
    """Same reset the GUI button issues, awaited rather than broadcast.

    Exists so the reset can be exercised from a shell or a test without driving a
    browser — `curl -X POST .../api/sim/reset` returns the outcome, including
    which robots failed to confirm, instead of only announcing it on a socket.
    """
    return await reset_fleet()


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


def _robots_blocking_map_reset(robot_id: str | None = None) -> list[str]:
    """Robots whose current command state makes a map reset unsafe."""
    robots = (
        [registry.robots[robot_id]]
        if robot_id in registry.robots
        else list(registry.robots.values()) if robot_id is None else []
    )
    return sorted(
        robot.robot_id
        for robot in robots
        if robot.nav_status == "active" or robot.goal is not None
    )


async def _publish_map_reset(scope: str, robot_id: str | None = None) -> Response:
    blocked = _robots_blocking_map_reset(robot_id)
    if blocked:
        return JSONResponse(
            {
                "error": "map reset refused while navigation is active",
                "robots": blocked,
            },
            status_code=409,
        )

    # A collaborative backend supplies one already-fused grid, so this service
    # has no per-robot cells it could subtract from it. Clearing one robot would
    # falsely imply that it had been removed. The explicit fleet reset remains
    # available and clears that fused product wholesale.
    if robot_id is not None and map_service.merge_mode == "cslam" and map_service.global_grid is not None:
        return JSONResponse(
            {"error": "targeted reset unavailable for a fused collaborative map; reset all maps"},
            status_code=409,
        )

    reset = await map_service.reset_robot_async(robot_id)
    patch = map_service.take_patch()
    if patch is not None:
        await broadcast(patch)
    events.log(
        "map_reset",
        {"scope": scope, "robot_id": robot_id, "robots": reset},
    )
    return JSONResponse(
        {
            "ok": True,
            "scope": scope,
            "robots": reset,
            "info": map_service.meta.as_dict(map_service.seq),
        }
    )


@app.post("/api/map/reset/{robot_id}")
async def reset_robot_map(robot_id: str) -> Response:
    """Clear one robot's backend map products without restarting its SLAM."""
    return await _publish_map_reset("robot", robot_id)


@app.post("/api/map/reset")
async def reset_all_maps() -> Response:
    """Clear every backend map product, provided the fleet is stationary."""
    return await _publish_map_reset("all")


@app.get("/api/map/local/{robot_id}")
async def get_local_map(robot_id: str) -> Response:
    """The robot's unregistered SLAM grid, expressed in its own map frame."""
    content = map_service.local_png(robot_id)
    info = map_service.local_info(robot_id)
    if content is None or info is None:
        return JSONResponse({"error": "local map not available"}, status_code=404)
    return Response(
        content=content,
        media_type="image/png",
        headers={"Cache-Control": "no-cache", "X-Map-Seq": str(info["seq"])},
    )


@app.get("/api/map/local/{robot_id}/info")
async def get_local_map_info(robot_id: str) -> Response:
    info = map_service.local_info(robot_id)
    if info is None:
        return JSONResponse({"error": "local map not available"}, status_code=404)
    return JSONResponse(info)


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
    await map_service.ingest_async(rid, meta, cells)
    return {"ok": True, "cells": int(cells.size)}


# 1 cm, which is finer than the 5 cm occupancy grid and keeps a cloud inside
# int16 out to +/-327 m. Quantising at the transport rather than sending float32
# halves the bytes for a view whose points are one pixel on screen.
CLOUD_SCALE = 0.01


@app.post("/api/adapter/global_map")
async def post_global_map(request: Request) -> Any:
    """A collaborative back end's already-merged grid, in its own common frame.

    Distinct from `/api/adapter/map`, which takes ONE robot's grid to be merged
    here. This is the whole map, produced by the back end from its own keyframes
    at its own optimised poses, and in `cslam` mode it is used as-is. Merging it
    again against per-robot grids from a different SLAM system is exactly the
    mistake that put robots 11-16 m from ground truth.
    """
    import zlib

    meta = GridMeta(
        resolution=float(request.query_params.get("resolution", 0.05)),
        width=int(request.query_params.get("width", 0)),
        height=int(request.query_params.get("height", 0)),
        origin_x=float(request.query_params.get("origin_x", 0.0)),
        origin_y=float(request.query_params.get("origin_y", 0.0)),
    )
    if meta.width <= 0 or meta.height <= 0:
        return JSONResponse({"error": "width and height required"}, status_code=400)
    try:
        raw = zlib.decompress(await request.body())
        cells = np.frombuffer(raw, dtype=np.int8)
    except (zlib.error, ValueError):
        return JSONResponse({"error": "malformed grid"}, status_code=400)
    if cells.size != meta.width * meta.height:
        return JSONResponse({"error": "size mismatch"}, status_code=400)
    map_service.set_global_grid(meta, cells.reshape(meta.height, meta.width))
    return {"ok": True, "cells": int(cells.size)}


@app.post("/api/adapter/cloud")
async def post_cloud(request: Request) -> Any:
    """Adapter uploads its 3D map cloud (zlib int16 xyz triples, 1 cm units).

    Optional, and deliberately so: a robot on 2D SLAM has no such cloud, never
    calls this, and the GUI's 3D view simply stays empty for it. Throttle to
    0.5 Hz — this is a view of an accumulated map, not a sensor stream.
    """
    import zlib

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    scale = float(request.query_params.get("scale", CLOUD_SCALE))
    try:
        raw = zlib.decompress(await request.body())
        quantised = np.frombuffer(raw, dtype=np.int16)
    except (zlib.error, ValueError):
        return JSONResponse({"error": "malformed cloud"}, status_code=400)
    if quantised.size % 3:
        return JSONResponse({"error": "xyz triples expected"}, status_code=400)
    points = quantised.reshape(-1, 3).astype(np.float32) * scale
    await map_service.set_cloud_async(rid, points)
    return {"ok": True, "points": int(len(points))}


@app.post("/api/adapter/scan")
async def post_scan(request: Request) -> Any:
    """One lidar scan for a robot with no `OccupancyGrid` publisher of its own.

    Body: zlib int16 xy pairs (1 cm units, already deduplicated onto a voxel
    lattice by the adapter — see adapter_ros1.py). `origin_x`/`origin_y` are
    the sensor's position in the SAME frame as the points, at capture time,
    used to raytrace free space back to it — see `mapsvc/scan_grid.py`. Feeds
    the same `ingest()` a robot that publishes its own grid uses, so
    registration and merging are unaffected either way.
    """
    import zlib

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    try:
        origin_x = float(request.query_params["origin_x"])
        origin_y = float(request.query_params["origin_y"])
    except (KeyError, ValueError):
        return JSONResponse({"error": "origin_x/origin_y required"}, status_code=400)
    scale = float(request.query_params.get("scale", CLOUD_SCALE))
    try:
        raw = zlib.decompress(await request.body())
        quantised = np.frombuffer(raw, dtype=np.int16)
    except (zlib.error, ValueError):
        return JSONResponse({"error": "malformed scan"}, status_code=400)
    if quantised.size % 2:
        return JSONResponse({"error": "xy pairs expected"}, status_code=400)
    points = quantised.reshape(-1, 2).astype(np.float32) * scale
    await map_service.ingest_scan_async(rid, origin_x, origin_y, points)
    return {"ok": True, "points": int(len(points))}


@app.get("/api/map/cloud")
async def get_cloud() -> Response:
    """The merged 3D cloud for the GUI's optional 3D view.

    Same shape as the 2D map's transport: one HTTP fetch of everything, polled
    slowly, rather than a stream. A cloud is large and changes slowly, and the
    3D view is not on the operator's critical path.
    """
    import zlib

    points, indices, names = map_service.merged_cloud()
    quantised = np.round(points / CLOUD_SCALE).astype(np.int16)
    body = zlib.compress(quantised.tobytes() + indices.tobytes(), 1)
    return Response(
        content=body,
        media_type="application/octet-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Cloud-Points": str(len(points)),
            "X-Cloud-Scale": str(CLOUD_SCALE),
            "X-Cloud-Robots": ",".join(names),
        },
    )


@app.post("/api/adapter/camera")
async def post_camera(request: Request) -> Any:
    """Accept a throttled JPEG preview from an adapter.

    This is the ROS-free fallback when the low-latency WHEP pipeline is not
    installed. Adapters remain responsible for converting their native camera
    format into a browser-ready JPEG.
    """
    global _camera_seq

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    if request.headers.get("content-type", "").split(";", 1)[0] != "image/jpeg":
        return JSONResponse({"error": "image/jpeg required"}, status_code=415)
    frame = await request.body()
    if not frame or len(frame) > 2_000_000 or not frame.startswith(b"\xff\xd8"):
        return JSONResponse({"error": "invalid JPEG frame"}, status_code=400)

    _camera_seq += 1
    _camera_frames[rid] = (frame, time.monotonic(), _camera_seq)
    return {"ok": True, "bytes": len(frame), "seq": _camera_seq}


@app.get("/api/camera/{robot_id}")
async def get_camera(robot_id: str) -> Response:
    current = _camera_frames.get(robot_id)
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


# ----------------------------------------------------------------- GUI socket


@app.websocket("/ws")
async def gui_socket(ws: WebSocket) -> None:
    await ws.accept()
    _gui_clients.add(ws)
    try:
        await ws.send_json({"type": "map_info", "info": map_service.meta.as_dict(map_service.seq)})
        await ws.send_json({"type": "fleet_change", "robots": fleet_snapshot()})
        await ws.send_json(session_state())
        await ws.send_json({"type": "settings_state", "settings": settings_store.value})
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
        taken_by = goal_taken(goal, exclude=rid)
        if taken_by:
            await raise_alert(
                f"dupgoal_{rid}", "warn", "fault",
                f"Goal already assigned to {taken_by}", rid,
            )
            return
        robot = registry.robots[rid]
        local_goal = (
            goal
            if robot.coordinate_frame == "merged"
            else map_service.world_to_robot(rid, goal)
        )
        sent = await registry.send(
            rid, {"type": "navigate_to", "goal": local_goal, **stamps()}
        )
        if sent:
            # Reserve immediately. Waiting for the adapter's next 5 Hz state
            # packet leaves a race where back-to-back UI messages can assign
            # the same destination to multiple robots.
            robot.goal = local_goal
            robot.nav_status = "active"
            robot.mode = "nav"

    elif kind == "cancel_goal":
        sent = await registry.send(rid, {"type": "cancel_goal", **stamps()})
        if sent and rid in registry.robots:
            registry.robots[rid].goal = None

    elif kind == "drive":
        if not registry.can(rid, "navigate"):
            return
        payload = msg.get("payload") or {}
        try:
            linear = max(-0.45, min(0.45, float(payload.get("linear", 0.0))))
            angular = max(-1.2, min(1.2, float(payload.get("angular", 0.0))))
        except (TypeError, ValueError):
            return
        sent = await registry.send(
            rid,
            {"type": "drive", "linear": linear, "angular": angular, **stamps()},
        )
        if sent:
            registry.robots[rid].goal = None

    elif kind == "stop_all":
        for robot_id in list(registry.robots):
            await registry.send(robot_id, {"type": "stop", **stamps()})
        await raise_alert("stop_all", "critical", "fault", "STOP ALL issued by operator")

    elif kind == "reset_sim":
        # Fire-and-forget: reset_fleet() waits on every adapter, and awaiting it
        # here would stall this socket's receive loop for as long as that takes,
        # so the operator's own GUI would stop updating during the one operation
        # they most want to watch. Progress reaches every client by broadcast.
        asyncio.create_task(reset_fleet())

    elif kind == "acknowledge_alert":
        aid = msg.get("id", "")
        if aid in _alerts:
            _alerts[aid]["acknowledged"] = True
        suppress_alert(aid)
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
                # 2 adds the optional `slam_graph` message and nothing else, so a
                # protocol-1 adapter stays valid forever. Rejecting it would
                # break exactly the mixed-fleet property the contract exists for.
                if msg.get("protocol") not in SUPPORTED_PROTOCOLS:
                    await ws.close(code=4400)
                    return
                r = registry.hello(msg, ws)
                robot_id = r.robot_id
                if r.coordinate_frame == "merged":
                    map_service.set_transform(robot_id, 0.0, 0.0, 0.0)
                await ws.send_json({"type": "hello_ack", "robot_id": robot_id})
                await broadcast({"type": "fleet_change", "robots": fleet_snapshot()})
                events.log("adapter_connect", {"robot_id": robot_id, "adapter": r.adapter})

            elif kind == "robot_state":
                registry.update_state(msg)

            elif kind == "detections":
                for item in msg.get("items", []):
                    detection_id = f"{msg['robot_id']}:{item.get('id', item.get('class', 'object'))}"
                    previous = _detections.get(detection_id)
                    now = time.time()
                    det = {
                        "id": detection_id,
                        "class": item.get("class", "object"),
                        "score": item.get("score", 0.0),
                        "robot_id": msg["robot_id"],
                        "camera": msg.get("camera", "front"),
                        "bbox": item.get("bbox"),
                        "polygon": item.get("polygon"),
                        "map_position": detection_position(
                            msg["robot_id"], item.get("map_position")
                        ),
                        "first_seen": previous["first_seen"] if previous else now,
                        "last_seen": now,
                        "observations": (previous["observations"] + 1) if previous else 1,
                    }
                    _detections[detection_id] = det
                    await broadcast({"type": "detection", "detection": det})

            elif kind == "map_meta":
                pass  # metadata accompanies the HTTP upload

            elif kind == "reset_done":
                # The adapter has finished resetting and has dropped its cached
                # grid, so the backend may now clear without the old map coming
                # straight back. See reset_fleet().
                #
                # A partial failure is recorded rather than alerted on here:
                # reset_fleet() clears every alert once the fleet has answered,
                # which would wipe an alert raised from inside this branch.
                rid = msg.get("robot_id", "")
                if rid in _reset_pending:
                    if not msg.get("ok", True):
                        _reset_failures[rid] = msg.get("steps") or {}
                    _reset_pending.discard(rid)
                    if not _reset_pending and _reset_done is not None:
                        _reset_done.set()

            elif kind == "slam_graph":
                # Optional (protocol 2). A robot running a collaborative back end
                # reports its own view of the shared pose graph; adapters that do
                # not run one simply never send this and nothing downstream
                # changes.
                rid = msg.get("robot_id")
                if rid:
                    graph = {
                        "keyframes": int(msg.get("keyframes", 0)),
                        "in_common_frame": bool(msg.get("in_common_frame", False)),
                        "residual": msg.get("residual"),
                        "inter_robot": msg.get("inter_robot", []),
                        "t_mono": msg.get("t_mono"),
                    }
                    map_service.set_slam_graph(rid, graph)
                    # `origin` is this robot's SLAM frame expressed in the
                    # collaborative back end's common frame. In `cslam` mode it
                    # REPLACES grid registration as the source of the merge
                    # transform, which is the whole point of running a joint
                    # pose graph: the transform falls out of the loop closures
                    # instead of being re-estimated from finished maps.
                    common_pose = msg.get("common_pose")
                    if isinstance(common_pose, dict):
                        map_service.set_common_pose(rid, common_pose)
                    origin = msg.get("origin")
                    if isinstance(origin, dict):
                        map_service.set_cslam_origin(
                            rid,
                            float(origin.get("x", 0.0)),
                            float(origin.get("y", 0.0)),
                            float(origin.get("yaw", 0.0)),
                            str(origin.get("frame") or ""),
                        )
                    await broadcast({"type": "slam_graph", "robot_id": rid, "graph": graph})

            # Unknown types are ignored, not fatal (protocol rule 3).
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"[adapter] socket error: {exc}")
    finally:
        if robot_id:
            registry.disconnect(robot_id)
            events.log("adapter_disconnect", {"robot_id": robot_id})
