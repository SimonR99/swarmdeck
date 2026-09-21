"""Map endpoints for deployment-frame raster products and local costmaps.

Replica keyframes are the only source of optimized maps.  This module contains
no occupancy-map merge, SLAM ingress, or server-side planning code.
"""

from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Any

import numpy as np
from fastapi import Request, Response
from fastapi.responses import JSONResponse

from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, map_service

MAX_UPLOAD_BYTES = 64 * 1024 * 1024
DEPLOYMENT_SCOPE_PREFIX = "deployment:"
_optimized: dict[str, tuple[GridMeta, np.ndarray, tuple[str, ...], dict[str, dict[str, float]] | None]] = {}
_optimized_lock = threading.Lock()
_server_scopes: set[str] = set()
_optimized_seq: dict[str, int] = {}
_raster_generation = 0
_robot_epoch_locks: dict[str, asyncio.Lock] = {}


def robot_epoch_lock(robot_id: str) -> asyncio.Lock:
    return _robot_epoch_locks.setdefault(robot_id, asyncio.Lock())


def raster_generation() -> int:
    with _optimized_lock:
        return _raster_generation


def is_deployment_scope(scope: str) -> bool:
    return scope.startswith(DEPLOYMENT_SCOPE_PREFIX)


def is_server_scope(scope: str) -> bool:
    return is_deployment_scope(scope) or scope in _server_scopes


def publish_optimized_map(
    scope: str,
    meta: GridMeta,
    cells: np.ndarray,
    robots: tuple[str, ...],
    transforms: dict[str, dict[str, float]] | None,
    *,
    expected_generation: int | None = None,
) -> bool:
    cells = np.asarray(cells, dtype=np.int8)
    if cells.shape != (meta.height, meta.width):
        raise ValueError("grid cells shape does not match metadata")
    if transforms is not None and not set(transforms).issubset(robots):
        raise ValueError("transform robot outside map scope")
    with _optimized_lock:
        if expected_generation is not None and expected_generation != _raster_generation:
            return False
        _optimized[scope] = (meta, cells, tuple(robots), transforms)
        _server_scopes.add(scope)
        _optimized_seq[scope] = _optimized_seq.get(scope, 0) + 1
    return True


def retire_server_scopes(keep: str | None = None) -> list[str]:
    with _optimized_lock:
        dead = sorted(scope for scope in _optimized if is_server_scope(scope) and scope != keep)
        for scope in dead:
            _optimized.pop(scope, None)
            _server_scopes.discard(scope)
            _optimized_seq.pop(scope, None)
    return dead


def retire_deployment_scopes(keep: str | None = None) -> list[str]:
    return retire_server_scopes(keep)


def has_optimized_map(scope: str) -> bool:
    with _optimized_lock:
        return scope in _optimized


def _map_headers(info: dict[str, Any]) -> dict[str, str]:
    return {
        "Cache-Control": "no-cache",
        "X-Map-Resolution": str(info["resolution"]),
        "X-Map-Width": str(info["width"]),
        "X-Map-Height": str(info["height"]),
        "X-Map-Origin-X": str(info["origin"]["x"]),
        "X-Map-Origin-Y": str(info["origin"]["y"]),
        **({"X-Map-Seq": str(info["seq"])} if "seq" in info else {}),
        **({"X-Map-Transforms": json.dumps(info["transforms"], separators=(",", ":"))} if "transforms" in info else {}),
    }


async def get_map_status() -> dict[str, Any]:
    return map_service.status()


def robot_command_error(robot_id: str) -> str | None:
    from autonomy.live_mapping import LIVE_MAPPING_MAX_AGE_S
    from .autonomy_routes import store
    from .simulation_reset import reset_root, robot_reset_status

    mission = os.environ.get("SWARMDECK_MISSION_ID")
    if not mission:
        return None
    root = reset_root()
    if root is not None:
        status = robot_reset_status(root, robot_id)
        if status.get("mission_id") == mission and status.get("phase") in {"accepted", "stopping", "starting", "verifying", "failed"}:
            return status.get("error") or "robot map reset is in progress"
    floor = store().map_epoch(robot_id, mission)
    if floor is None:
        return "robot mapping authority is missing or stale"
    robot = registry.robots.get(robot_id)
    live = robot.live_mapping if robot else None
    if not live or live.get("mission_id") != mission or live.get("robot_map_epoch") != floor or not robot.online or live["authority_age_s"] + max(0.0, time.monotonic() - robot.live_mapping_received_at) > LIVE_MAPPING_MAX_AGE_S:
        return "robot mapping authority is missing or stale"
    return None


async def retire_robot_epoch(robot_id: str, mission_id: str, map_epoch: int) -> None:
    from .app import CONFIG, broadcast
    from ..bus import stamps

    global _raster_generation
    robot = registry.robots.get(robot_id)
    if robot is not None:
        robot.command_generation += 1
        robot.live_mapping = None
        robot.peer_slam = None
        robot.navigation_ready = False
        robot.objective_continuation = None
        robot.goal = None
        robot.global_planned_path = []
        robot.local_planned_path = []
        robot.planned_path = []
        robot.nav_status = "cancelled"
        robot.mode = "idle"
        robot.exploration_status = "stopped"
        if robot_id not in ((CONFIG.get("map") or {}).get("start_poses") or {}):
            robot.home_pose = None
    with _optimized_lock:
        _raster_generation += 1
        dead = [scope for scope, (_, _, robots, _) in _optimized.items() if is_server_scope(scope) or robot_id in robots]
        for scope in dead:
            _optimized.pop(scope, None)
            _optimized_seq.pop(scope, None)
            _server_scopes.discard(scope)
    await registry.send(robot_id, {"type": "stop", **stamps()})
    await map_service.reset_robot_async(robot_id)
    reset_costmaps(robot_id)
    await broadcast({"type": "costmap_clear", "robot_id": robot_id})
    await broadcast({"type": "network_clear", "robot_id": robot_id})
    await broadcast({"type": "robot_map_reset", "robot_id": robot_id, "mission_id": mission_id, "map_epoch": map_epoch})


async def reset_robot_map(robot_id: str, request_id: str | None = None) -> Response:
    from .autonomy_routes import store
    from .simulation_reset import request_robot_reset, reset_root

    root = reset_root()
    mission = os.environ.get("SWARMDECK_MISSION_ID")
    if root is None or not mission:
        return JSONResponse({"phase": "failed", "ok": False, "error": "robot map reset supervisor is unavailable"}, status_code=503)
    if robot_id not in registry.robots:
        return JSONResponse({"error": "Unknown robot"}, status_code=404)
    if request_id is None:
        return JSONResponse({"error": "request_id is required"}, status_code=400)
    try:
        async with robot_epoch_lock(robot_id):
            advanced: list[int] = []
            def reserve():
                current = store().map_epoch(robot_id, mission)
                peer = registry.robots[robot_id].peer_slam or {}
                epoch = max(current if current is not None else 0, peer.get("robot_map_epoch", 0)) + 1
                store().reserve_map_epoch(robot_id, mission, epoch)
                advanced.append(epoch)
                return epoch
            result = await asyncio.to_thread(request_robot_reset, root, robot_id, mission, request_id, reserve)
            if advanced:
                await retire_robot_epoch(robot_id, mission, advanced[0])
        result["status_url"] = f"/api/map/reset/{robot_id}?request_id={result['request_id']}"
        code = 200 if result.get("phase") == "done" else 503 if result.get("phase") == "failed" else 202
        return JSONResponse(result, status_code=code, headers={"Cache-Control": "no-store"})
    except (ValueError, KeyError, TypeError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
    except OSError as exc:
        return JSONResponse({"phase": "failed", "ok": False, "error": str(exc)}, status_code=503)


async def get_robot_map_reset(robot_id: str, request_id: str | None = None) -> Response:
    from .simulation_reset import reset_root, robot_reset_status
    root = reset_root()
    if root is None:
        return JSONResponse({"phase": "failed", "ok": False, "error": "robot map reset supervisor is unavailable"}, status_code=503)
    try:
        return JSONResponse(robot_reset_status(root, robot_id, request_id), headers={"Cache-Control": "no-store"})
    except (ValueError, KeyError, TypeError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


async def reset_all_maps() -> Response:
    if os.environ.get("SWARMDECK_MISSION_ID"):
        return JSONResponse({"ok": False, "error": "reset all maps is unsupported for peer mapping; use the full mission reset"}, status_code=409)
    from .app import broadcast
    blocked = sorted(robot.robot_id for robot in registry.robots.values() if robot.nav_status == "active" or robot.goal is not None)
    if blocked:
        return JSONResponse({"error": "map reset refused while navigation is active", "robots": blocked}, status_code=409)
    reset = await map_service.reset_robot_async()
    reset_optimized_maps()
    reset_costmaps()
    await broadcast({"type": "costmap_clear", "robot_id": None})
    await broadcast({"type": "network_clear", "robot_id": None})
    events.log("map_reset", {"scope": "all", "robots": reset})
    return JSONResponse({"ok": True, "scope": "all", "robots": reset})


@dataclass
class CostmapEntry:
    robot_id: str
    kind: str
    meta: GridMeta
    cells: np.ndarray
    frame_id: str
    seq: int = 0
    updated_at: float = 0.0
    dirty: bool = True


_costmaps: dict[tuple[str, str], CostmapEntry] = {}
_costmap_lock = threading.Lock()


def _costmap_payload(entry: CostmapEntry) -> dict[str, Any]:
    top_down = np.flipud(entry.cells)
    return {"type": "costmap", "robot_id": entry.robot_id, "kind": entry.kind, "seq": entry.seq, "resolution": entry.meta.resolution, "origin": {"x": entry.meta.origin_x, "y": entry.meta.origin_y}, "width": entry.meta.width, "height": entry.meta.height, "frame_id": entry.frame_id, "updated_at": entry.updated_at, "data": base64.b64encode(zlib.compress(np.ascontiguousarray(top_down, dtype=np.int8).tobytes(), 1)).decode("ascii")}


def costmap_snapshots() -> list[dict[str, Any]]:
    with _costmap_lock:
        return [_costmap_payload(entry) for entry in _costmaps.values()]


def take_costmap_patches() -> list[dict[str, Any]]:
    with _costmap_lock:
        entries = [entry for entry in _costmaps.values() if entry.dirty]
        for entry in entries:
            entry.dirty = False
        return [_costmap_payload(entry) for entry in entries]


def reset_costmaps(robot_id: str | None = None) -> None:
    with _costmap_lock:
        if robot_id is None:
            _costmaps.clear()
        else:
            for key in list(_costmaps):
                if key[0] == robot_id:
                    del _costmaps[key]


async def get_costmap(robot_id: str, kind: str) -> Response:
    if kind != "local":
        return JSONResponse({"error": "only local costmaps are supported"}, status_code=400)
    with _costmap_lock:
        entry = _costmaps.get((robot_id, kind))
        if entry is None:
            return JSONResponse({"error": "costmap not available"}, status_code=404)
        payload = _costmap_payload(entry)
    return JSONResponse(payload, headers={"Cache-Control": "no-store"})


def _inflate(body: bytes) -> bytes:
    decompressor = zlib.decompressobj()
    raw = decompressor.decompress(body, MAX_UPLOAD_BYTES)
    if decompressor.unconsumed_tail:
        raise ValueError("upload exceeds the maximum decompressed size")
    return raw


async def post_costmap(request: Request) -> Any:
    rid = request.query_params.get("robot_id", "")
    kind = request.query_params.get("kind", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    if kind != "local":
        return JSONResponse({"error": "only local costmaps are supported"}, status_code=400)
    try:
        resolution = float(request.query_params.get("resolution", 0.0))
        width = int(request.query_params.get("width", 0))
        height = int(request.query_params.get("height", 0))
        origin_x = float(request.query_params.get("origin_x", 0.0))
        origin_y = float(request.query_params.get("origin_y", 0.0))
    except (TypeError, ValueError):
        return JSONResponse({"error": "malformed costmap metadata"}, status_code=400)
    if not math.isfinite(resolution) or resolution <= 0.0 or width <= 0 or height <= 0 or width * height > MAX_UPLOAD_BYTES or not math.isfinite(origin_x) or not math.isfinite(origin_y):
        return JSONResponse({"error": "invalid costmap dimensions or geometry"}, status_code=400)
    try:
        cells = np.frombuffer(_inflate(await request.body()), dtype=np.int8)
    except (zlib.error, ValueError) as exc:
        return JSONResponse({"error": f"malformed costmap: {exc}"}, status_code=400)
    if cells.size != width * height:
        return JSONResponse({"error": "costmap size mismatch"}, status_code=400)
    normalized = np.where(cells < 0, -1, np.clip(cells, 0, 100)).astype(np.int8)
    key = (rid, kind)
    with _costmap_lock:
        previous = _costmaps.get(key)
        entry = CostmapEntry(rid, kind, GridMeta(resolution, width, height, origin_x, origin_y), np.ascontiguousarray(normalized.reshape(height, width)).copy(), str(request.query_params.get("frame_id", "") or "").lstrip("/"), (previous.seq + 1) if previous else 1, time.time(), True)
        _costmaps[key] = entry
    return {"ok": True, "kind": kind, "seq": entry.seq, "cells": int(cells.size)}


def _prune_optimized_maps(scopes: Any) -> list[str]:
    if not isinstance(scopes, list) or not all(isinstance(scope, str) for scope in scopes):
        return []
    live = set(scopes)
    with _optimized_lock:
        dead = sorted(scope for scope in set(_optimized) - live if not is_server_scope(scope))
        for scope in dead:
            _optimized.pop(scope, None)
            _optimized_seq.pop(scope, None)
    return dead


async def get_optimized_index() -> dict[str, Any]:
    with _optimized_lock:
        items = [{"scope": scope, "robots": list(robots), "resolution": meta.resolution, "width": meta.width, "height": meta.height, "origin": {"x": meta.origin_x, "y": meta.origin_y}, "seq": _optimized_seq.get(scope, 0)} for scope, (meta, _cells, robots, _transforms) in sorted(_optimized.items())]
    return {"type": "optimized_maps", "maps": items}


async def get_optimized_map(scope: str) -> Response:
    with _optimized_lock:
        entry = _optimized.get(scope)
        seq = _optimized_seq.get(scope, 0)
    if entry is None:
        return JSONResponse({"error": f"no optimized map for {scope!r}"}, status_code=404)
    meta, cells, _robots, transforms = entry
    from ..mapsvc.output import grid_png
    return Response(content=grid_png(meta, cells), media_type="image/png", headers=_map_headers({**meta.as_dict(), "seq": seq, **({"transforms": transforms} if transforms is not None else {})}))


def reset_optimized_maps() -> None:
    global _raster_generation
    with _optimized_lock:
        _optimized.clear()
        _server_scopes.clear()
        _optimized_seq.clear()
        _raster_generation += 1
