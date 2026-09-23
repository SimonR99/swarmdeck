"""Map endpoints for deployment-frame raster products.

Replica keyframes are the only source of optimized maps.  This module contains
no occupancy-map merge, SLAM ingress, or server-side planning code.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import os
import secrets
import threading
import time
from collections import OrderedDict
from collections.abc import Iterable
from typing import Any

import numpy as np
from fastapi import Response
from fastapi.responses import JSONResponse

from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, map_service

MAX_UPLOAD_BYTES = 64 * 1024 * 1024
DEPLOYMENT_SCOPE_PREFIX = "deployment:"
_optimized: dict[
    str,
    tuple[GridMeta, np.ndarray, tuple[str, ...], dict[str, dict[str, float]] | None],
] = {}
_optimized_lock = threading.Lock()
_server_scopes: set[str] = set()
_optimized_seq: dict[str, int] = {}
# ``seq`` restarts at 1 after a retire, prune, reset or server restart, so it
# cannot name a published raster on its own. Each publication also takes the
# next value of a process-wide counter, and the ETag and PNG cache key use it
# with a per-process nonce: no two publications ever share either.
_OPTIMIZED_PROCESS_NONCE = secrets.token_hex(8)
_optimized_publication_counter = 0
_optimized_publication: dict[str, int] = {}
_raster_generation = 0
_robot_epoch_locks: dict[str, asyncio.Lock] = {}
_OPTIMIZED_PNG_CACHE_MAX_ENTRIES = 16
_OPTIMIZED_PNG_CACHE_MAX_BYTES = 16 * 1024 * 1024
_optimized_png_cache: OrderedDict[tuple[str, int], tuple[str, bytes]] = OrderedDict()
_optimized_png_cache_bytes = 0
_MAP_EPOCH_CACHE_MAX_ENTRIES = 256
_map_epoch_cache: OrderedDict[tuple[int, str, str], int | None] = OrderedDict()
_map_epoch_cache_lock = threading.Lock()


def robot_epoch_lock(robot_id: str) -> asyncio.Lock:
    return _robot_epoch_locks.setdefault(robot_id, asyncio.Lock())


def raster_generation() -> int:
    with _optimized_lock:
        return _raster_generation


def _map_epoch_cache_key(robot_id: str, session_id: str) -> tuple[int, str, str]:
    from .autonomy_routes import store

    return (id(store()), robot_id, session_id)


def _cached_map_epoch(key: tuple[int, str, str]) -> int | None:
    with _map_epoch_cache_lock:
        if key not in _map_epoch_cache:
            raise KeyError
        value = _map_epoch_cache[key]
        _map_epoch_cache.move_to_end(key)
        return value


def _remember_map_epoch_key(key: tuple[int, str, str], epoch: int | None) -> None:
    with _map_epoch_cache_lock:
        _map_epoch_cache[key] = epoch
        _map_epoch_cache.move_to_end(key)
        while len(_map_epoch_cache) > _MAP_EPOCH_CACHE_MAX_ENTRIES:
            _map_epoch_cache.popitem(last=False)


def cached_map_epoch(robot_id: str, session_id: str) -> int | None:
    from .autonomy_routes import store

    key = _map_epoch_cache_key(robot_id, session_id)
    try:
        return _cached_map_epoch(key)
    except KeyError:
        pass
    value = store().map_epoch(robot_id, session_id)
    _remember_map_epoch_key(key, value)
    return value


async def cached_map_epoch_async(robot_id: str, session_id: str) -> int | None:
    from .autonomy_routes import store

    replica_store = store()
    key = (id(replica_store), robot_id, session_id)
    try:
        return _cached_map_epoch(key)
    except KeyError:
        pass
    value = await asyncio.to_thread(replica_store.map_epoch, robot_id, session_id)
    _remember_map_epoch_key(key, value)
    return value


def remember_map_epoch(robot_id: str, session_id: str, epoch: int | None) -> None:
    _remember_map_epoch_key(_map_epoch_cache_key(robot_id, session_id), epoch)


def clear_map_epoch_cache() -> None:
    with _map_epoch_cache_lock:
        _map_epoch_cache.clear()


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
    global _optimized_publication_counter
    cells = np.asarray(cells, dtype=np.int8)
    if cells.shape != (meta.height, meta.width):
        raise ValueError("grid cells shape does not match metadata")
    if transforms is not None and not set(transforms).issubset(robots):
        raise ValueError("transform robot outside map scope")
    with _optimized_lock:
        if (
            expected_generation is not None
            and expected_generation != _raster_generation
        ):
            return False
        _optimized[scope] = (meta, cells, tuple(robots), transforms)
        _server_scopes.add(scope)
        _optimized_seq[scope] = _optimized_seq.get(scope, 0) + 1
        _optimized_publication_counter += 1
        _optimized_publication[scope] = _optimized_publication_counter
    return True


def retire_server_scopes(keep: str | Iterable[str] | None = None) -> list[str]:
    kept = {keep} if isinstance(keep, str) else set(keep or ())
    with _optimized_lock:
        dead = sorted(
            scope
            for scope in _optimized
            if is_server_scope(scope) and scope not in kept
        )
        for scope in dead:
            _optimized.pop(scope, None)
            _server_scopes.discard(scope)
            _optimized_seq.pop(scope, None)
            _optimized_publication.pop(scope, None)
        _drop_optimized_png_cache(dead)
    return dead


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
        **(
            {"X-Map-Transforms": json.dumps(info["transforms"], separators=(",", ":"))}
            if "transforms" in info
            else {}
        ),
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
        if status.get("mission_id") == mission and status.get("phase") in {
            "accepted",
            "stopping",
            "starting",
            "verifying",
            "failed",
        }:
            return status.get("error") or "robot map reset is in progress"
    floor = cached_map_epoch(robot_id, mission)
    if floor is None:
        return "robot mapping authority is missing or stale"
    robot = registry.robots.get(robot_id)
    live = robot.live_mapping if robot else None
    if (
        not live
        or live.get("mission_id") != mission
        or live.get("robot_map_epoch") != floor
        or not robot.online
        or live["authority_age_s"]
        + max(0.0, time.monotonic() - robot.live_mapping_received_at)
        > LIVE_MAPPING_MAX_AGE_S
    ):
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
        dead = [
            scope
            for scope, (_, _, robots, _) in _optimized.items()
            if is_server_scope(scope) or robot_id in robots
        ]
        for scope in dead:
            _optimized.pop(scope, None)
            _optimized_seq.pop(scope, None)
            _optimized_publication.pop(scope, None)
            _server_scopes.discard(scope)
        _drop_optimized_png_cache(dead)
    await registry.send(robot_id, {"type": "stop", **stamps()})
    await map_service.reset_robot_async(robot_id)
    await broadcast({"type": "network_clear", "robot_id": robot_id})
    await broadcast(
        {
            "type": "robot_map_reset",
            "robot_id": robot_id,
            "mission_id": mission_id,
            "map_epoch": map_epoch,
        }
    )


async def reset_robot_map(robot_id: str, request_id: str | None = None) -> Response:
    from .autonomy_routes import store
    from .simulation_reset import request_robot_reset, reset_root

    root = reset_root()
    mission = os.environ.get("SWARMDECK_MISSION_ID")
    if root is None or not mission:
        return JSONResponse(
            {
                "phase": "failed",
                "ok": False,
                "error": "robot map reset supervisor is unavailable",
            },
            status_code=503,
        )
    if robot_id not in registry.robots:
        return JSONResponse({"error": "Unknown robot"}, status_code=404)
    if request_id is None:
        return JSONResponse({"error": "request_id is required"}, status_code=400)
    try:
        async with robot_epoch_lock(robot_id):
            advanced: list[int] = []

            def reserve():
                current = cached_map_epoch(robot_id, mission)
                peer = registry.robots[robot_id].peer_slam or {}
                epoch = (
                    max(
                        current if current is not None else 0,
                        peer.get("robot_map_epoch", 0),
                    )
                    + 1
                )
                store().reserve_map_epoch(robot_id, mission, epoch)
                remember_map_epoch(robot_id, mission, epoch)
                advanced.append(epoch)
                return epoch

            result = await asyncio.to_thread(
                request_robot_reset, root, robot_id, mission, request_id, reserve
            )
            if advanced:
                await retire_robot_epoch(robot_id, mission, advanced[0])
        result["status_url"] = (
            f"/api/map/reset/{robot_id}?request_id={result['request_id']}"
        )
        code = (
            200
            if result.get("phase") == "done"
            else 503 if result.get("phase") == "failed" else 202
        )
        return JSONResponse(
            result, status_code=code, headers={"Cache-Control": "no-store"}
        )
    except (ValueError, KeyError, TypeError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
    except OSError as exc:
        return JSONResponse(
            {"phase": "failed", "ok": False, "error": str(exc)}, status_code=503
        )


async def get_robot_map_reset(robot_id: str, request_id: str | None = None) -> Response:
    from .simulation_reset import reset_root, robot_reset_status

    root = reset_root()
    if root is None:
        return JSONResponse(
            {
                "phase": "failed",
                "ok": False,
                "error": "robot map reset supervisor is unavailable",
            },
            status_code=503,
        )
    try:
        return JSONResponse(
            robot_reset_status(root, robot_id, request_id),
            headers={"Cache-Control": "no-store"},
        )
    except (ValueError, KeyError, TypeError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


async def reset_all_maps() -> Response:
    if os.environ.get("SWARMDECK_MISSION_ID"):
        return JSONResponse(
            {
                "ok": False,
                "error": "reset all maps is unsupported for peer mapping; use the full mission reset",
            },
            status_code=409,
        )
    from .app import broadcast

    blocked = sorted(
        robot.robot_id
        for robot in registry.robots.values()
        if robot.nav_status == "active" or robot.goal is not None
    )
    if blocked:
        return JSONResponse(
            {
                "error": "map reset refused while navigation is active",
                "robots": blocked,
            },
            status_code=409,
        )
    reset = await map_service.reset_robot_async()
    reset_optimized_maps()
    await broadcast({"type": "network_clear", "robot_id": None})
    events.log("map_reset", {"scope": "all", "robots": reset})
    return JSONResponse({"ok": True, "scope": "all", "robots": reset})


def _prune_optimized_maps(scopes: Any) -> list[str]:
    if not isinstance(scopes, list) or not all(
        isinstance(scope, str) for scope in scopes
    ):
        return []
    live = set(scopes)
    with _optimized_lock:
        dead = sorted(
            scope for scope in set(_optimized) - live if not is_server_scope(scope)
        )
        for scope in dead:
            _optimized.pop(scope, None)
            _optimized_seq.pop(scope, None)
            _optimized_publication.pop(scope, None)
        _drop_optimized_png_cache(dead)
    return dead


async def get_optimized_index() -> dict[str, Any]:
    with _optimized_lock:
        items = [
            {
                "scope": scope,
                "robots": list(robots),
                "resolution": meta.resolution,
                "width": meta.width,
                "height": meta.height,
                "origin": {"x": meta.origin_x, "y": meta.origin_y},
                "seq": _optimized_seq.get(scope, 0),
            }
            for scope, (meta, _cells, robots, _transforms) in sorted(_optimized.items())
        ]
    return {"type": "optimized_maps", "maps": items}


def _etag_for_optimized_map(scope: str, publication: int) -> str:
    token = hashlib.sha256(
        f"{_OPTIMIZED_PROCESS_NONCE}\0{scope}\0{publication}".encode()
    ).hexdigest()[:24]
    return f'"optimized-map-{token}"'


def _client_has_etag(header: str | None, etag: str) -> bool:
    if not header:
        return False
    return any(item.strip() == etag for item in header.split(","))


def _drop_optimized_png_cache(scopes: Iterable[str] | None = None) -> None:
    global _optimized_png_cache_bytes
    if scopes is None:
        _optimized_png_cache.clear()
        _optimized_png_cache_bytes = 0
        return
    dead = set(scopes)
    for key in list(_optimized_png_cache):
        if key[0] in dead:
            _optimized_png_cache_bytes -= len(_optimized_png_cache.pop(key)[1])


def _png_for_optimized_map(
    scope: str, publication: int, meta: GridMeta, cells: np.ndarray
) -> tuple[str, bytes]:
    global _optimized_png_cache_bytes
    cache_key = (scope, publication)
    with _optimized_lock:
        cached = _optimized_png_cache.get(cache_key)
        if cached is not None:
            _optimized_png_cache.move_to_end(cache_key)
            return cached
    from ..mapsvc.output import grid_png

    encoded = (_etag_for_optimized_map(scope, publication), grid_png(meta, cells))
    with _optimized_lock:
        cached = _optimized_png_cache.get(cache_key)
        if cached is not None:
            _optimized_png_cache.move_to_end(cache_key)
            return cached
        _optimized_png_cache[cache_key] = encoded
        _optimized_png_cache_bytes += len(encoded[1])
        while (
            len(_optimized_png_cache) > _OPTIMIZED_PNG_CACHE_MAX_ENTRIES
            or _optimized_png_cache_bytes > _OPTIMIZED_PNG_CACHE_MAX_BYTES
        ):
            _old_key, (_old_etag, body) = _optimized_png_cache.popitem(last=False)
            _optimized_png_cache_bytes -= len(body)
    return encoded


async def get_optimized_map(scope: str, if_none_match: str | None = None) -> Response:
    with _optimized_lock:
        entry = _optimized.get(scope)
        seq = _optimized_seq.get(scope, 0)
        publication = _optimized_publication.get(scope, 0)
    if entry is None:
        return JSONResponse(
            {"error": f"no optimized map for {scope!r}"}, status_code=404
        )
    meta, cells, _robots, transforms = entry
    etag, body = _png_for_optimized_map(scope, publication, meta, cells)
    headers = {
        **_map_headers(
            {
                **meta.as_dict(),
                "seq": seq,
                **({"transforms": transforms} if transforms is not None else {}),
            }
        ),
        "ETag": etag,
    }
    if _client_has_etag(if_none_match, etag):
        return Response(status_code=304, headers=headers)
    return Response(content=body, media_type="image/png", headers=headers)


def reset_optimized_maps() -> None:
    global _raster_generation
    with _optimized_lock:
        _optimized.clear()
        _server_scopes.clear()
        _optimized_seq.clear()
        _optimized_publication.clear()
        _drop_optimized_png_cache()
        _raster_generation += 1
