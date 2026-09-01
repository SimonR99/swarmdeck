"""HTTP handlers for merged, local, and adapter-uploaded map products.

The FastAPI application keeps compatibility wrappers for these handlers in
``api.app``.  Keeping map transport here prevents the websocket/control module
from becoming the owner of binary upload validation and map rendering policy.
"""

from __future__ import annotations

import asyncio
import base64
from dataclasses import dataclass
import math
import threading
import time
import zlib
from typing import Any

import numpy as np
from fastapi import Request, Response
from fastapi.responses import JSONResponse

from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, map_service

MAX_UPLOAD_BYTES = 64 * 1024 * 1024
# 1 cm, which is finer than the 5 cm occupancy grid and keeps a cloud inside
# int16 out to +/-327 m. This is part of the adapter HTTP wire format.
CLOUD_SCALE = 0.01

# Optimized grids the SLAM back-end renders per robot and per component.
#
# Held here rather than in map_service because they are a VIEW, never an input
# to navigation or merging: the merged map keeps its rule that two components
# are never overlaid, and these exist so an operator can look at a robot that
# rule makes invisible -- one that has merged with nobody. Storing them
# alongside the authoritative merged grid would invite exactly the confusion
# the rule prevents.
#
# Bounded by fleet size (one entry per robot plus one per component). Each scope
# is overwritten in place on every publish, and scopes the back-end has stopped
# publishing are dropped by _prune_optimized_maps -- without that, a retired
# component id would be served from here forever, because a scope only ever
# arrives and nothing else could tell a live one from a dead one.
_optimized: dict[str, tuple[GridMeta, np.ndarray, tuple[str, ...]]] = {}
_optimized_lock = threading.Lock()


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


# Costmaps are navigation products, not map inputs. They stay in this separate
# store so a local rolling window can never affect collaborative map merging.
_costmaps: dict[tuple[str, str], CostmapEntry] = {}
_costmap_lock = threading.Lock()


async def get_map() -> Response:
    content, seq = map_service.map_png()
    return Response(
        content=content,
        media_type="image/png",
        headers={"Cache-Control": "no-cache", "X-Map-Seq": str(seq)},
    )


async def get_map_status() -> dict[str, Any]:
    """Merge mode, per-robot transforms, and registration quality (FR-M6)."""
    return map_service.status()


async def get_map_info() -> dict[str, Any]:
    return {"type": "map_info", "info": map_service.map_info()}


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
    """Reset backend map products and publish the matching patch."""
    from .app import broadcast

    blocked = _robots_blocking_map_reset(robot_id)
    if blocked:
        return JSONResponse(
            {
                "error": "map reset refused while navigation is active",
                "robots": blocked,
            },
            status_code=409,
        )

    graph_delete: dict[str, Any] | None = None
    # Graph mode owns the keyframes that produced the fused grid, so ask it to
    # remove the selected robot before clearing the server-side projections.
    # The old implementation rejected this request because the server cannot
    # subtract pixels from an already-fused grid; deleting graph inputs and
    # re-rendering the survivors is the exact operation the UI means.
    if (
        robot_id is not None
        and map_service.merge_mode == "graph"
        and (
            robot_id in map_service.slam_graphs
            or robot_id in map_service.common_poses
            or robot_id in map_service.cslam_frames
        )
    ):
        from ..mapsvc import graph_bridge

        code, graph_delete = await asyncio.to_thread(graph_bridge.delete_robot, robot_id)
        if code < 200 or code >= 300:
            return JSONResponse(
                {
                    "error": graph_delete.get(
                        "error", "pose-graph keyframe deletion failed"
                    )
                },
                status_code=code,
            )

    # The legacy collaborative backend supplies one already-fused grid and has
    # no scoped graph-deletion endpoint. Its explicit fleet reset remains the
    # only truthful operation.
    if (
        robot_id is not None
        and map_service.merge_mode == "cslam"
        and map_service.global_grid is not None
    ):
        return JSONResponse(
            {
                "error": "targeted reset unavailable for a fused collaborative map; reset all maps"
            },
            status_code=409,
        )

    reset = await map_service.reset_robot_async(robot_id)
    if robot_id is None:
        from ..mapsvc import graph_bridge

        # The scoped grids are rendered from the pose graph, so they die with
        # it. Without this they outlived every reset -- reset_optimized_maps
        # existed for exactly this and was never called from anywhere.
        reset_optimized_maps()
        await asyncio.to_thread(graph_bridge.post_reset)
    elif graph_delete is not None:
        # Every scoped raster and the fused global raster describes the old
        # graph until its queued re-solve publishes replacement products.
        reset_optimized_maps()
        with map_service._state_lock:
            map_service.global_grid = None
            map_service.global_map_seq += 1
        map_service._remerge()
    reset_costmaps(robot_id)
    await broadcast({"type": "costmap_clear", "robot_id": robot_id})
    await broadcast({"type": "network_clear", "robot_id": robot_id})
    patch = map_service.take_patch()
    if patch is not None:
        await broadcast(patch)
    events.log("map_reset", {"scope": scope, "robot_id": robot_id, "robots": reset})
    return JSONResponse(
        {
            "ok": True,
            "scope": scope,
            "robots": reset,
            "keyframes": graph_delete,
            "info": map_service.map_info(),
        }
    )


async def reset_robot_map(robot_id: str) -> Response:
    """Clear one robot's backend map products without restarting its SLAM."""
    return await _publish_map_reset("robot", robot_id)


async def reset_all_maps() -> Response:
    """Clear every backend map product, provided the fleet is stationary."""
    return await _publish_map_reset("all")


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


async def get_local_map_info(robot_id: str) -> Response:
    info = map_service.local_info(robot_id)
    if info is None:
        return JSONResponse({"error": "local map not available"}, status_code=404)
    return JSONResponse(info)


async def get_local_network(robot_id: str) -> Response:
    """Full robot-local Wi-Fi heatmap for reconnects and view switches."""
    snapshot = map_service.network_snapshot(robot_id)
    if snapshot is None:
        return JSONResponse({"error": "network heatmap not available"}, status_code=404)
    return JSONResponse(snapshot, headers={"Cache-Control": "no-store"})


def _costmap_payload(entry: CostmapEntry) -> dict[str, Any]:
    """Make the full browser snapshot from a bottom-up ROS grid."""
    # Canvas images are top-down while ROS OccupancyGrid data is bottom-up.
    top_down = np.flipud(entry.cells)
    return {
        "type": "costmap",
        "robot_id": entry.robot_id,
        "kind": entry.kind,
        "seq": entry.seq,
        "resolution": entry.meta.resolution,
        "origin": {
            "x": entry.meta.origin_x,
            "y": entry.meta.origin_y,
        },
        "width": entry.meta.width,
        "height": entry.meta.height,
        "frame_id": entry.frame_id,
        "updated_at": entry.updated_at,
        "data": base64.b64encode(
            zlib.compress(np.ascontiguousarray(top_down, dtype=np.int8).tobytes(), 1)
        ).decode("ascii"),
    }


def costmap_snapshots() -> list[dict[str, Any]]:
    """Return current costmaps for a newly connected GUI websocket."""
    with _costmap_lock:
        return [_costmap_payload(entry) for entry in _costmaps.values()]


def take_costmap_patches() -> list[dict[str, Any]]:
    """Take newly uploaded full snapshots for the websocket fan-out."""
    with _costmap_lock:
        entries = [entry for entry in _costmaps.values() if entry.dirty]
        for entry in entries:
            entry.dirty = False
        return [_costmap_payload(entry) for entry in entries]


def reset_costmaps(robot_id: str | None = None) -> None:
    """Forget visualization snapshots after a map/world reset."""
    with _costmap_lock:
        if robot_id is None:
            _costmaps.clear()
        else:
            for key in list(_costmaps):
                if key[0] == robot_id:
                    del _costmaps[key]


async def get_costmap(robot_id: str, kind: str) -> Response:
    """Restore one full costmap for a browser that missed the websocket frame."""
    if kind not in {"global", "local"}:
        return JSONResponse({"error": "kind must be global or local"}, status_code=400)
    with _costmap_lock:
        entry = _costmaps.get((robot_id, kind))
        if entry is None:
            return JSONResponse({"error": "costmap not available"}, status_code=404)
        payload = _costmap_payload(entry)
    return JSONResponse(payload, headers={"Cache-Control": "no-store"})


def _inflate(body: bytes) -> bytes:
    """zlib-decompress an upload, refusing anything past MAX_UPLOAD_BYTES."""
    decompressor = zlib.decompressobj()
    raw = decompressor.decompress(body, MAX_UPLOAD_BYTES)
    if decompressor.unconsumed_tail:
        raise ValueError("upload exceeds the maximum decompressed size")
    return raw


async def post_map(request: Request) -> Any:
    """Adapter occupancy grid upload (zlib int8, row-major)."""
    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    try:
        meta = GridMeta(
            resolution=float(request.query_params.get("resolution", 0.05)),
            width=int(request.query_params.get("width", 0)),
            height=int(request.query_params.get("height", 0)),
            origin_x=float(request.query_params.get("origin_x", 0.0)),
            origin_y=float(request.query_params.get("origin_y", 0.0)),
        )
    except (TypeError, ValueError):
        return JSONResponse({"error": "malformed grid metadata"}, status_code=400)
    if meta.width <= 0 or meta.height <= 0:
        return JSONResponse({"error": "width and height required"}, status_code=400)
    try:
        raw = _inflate(await request.body())
        cells = np.frombuffer(raw, dtype=np.int8)
    except (zlib.error, ValueError) as exc:
        return JSONResponse({"error": f"malformed grid: {exc}"}, status_code=400)
    if cells.size != meta.width * meta.height:
        return JSONResponse({"error": "size mismatch"}, status_code=400)
    await map_service.ingest_async(rid, meta, cells.reshape(meta.height, meta.width))
    return {"ok": True, "cells": int(cells.size)}


async def post_costmap(request: Request) -> Any:
    """Store one normalized Nav2 costmap for the dashboard overlay.

    The body is zlib-compressed int8 data in ROS's bottom-up row order. It is
    deliberately not passed to ``MapService``: costmaps describe the planner's
    current obstacle view and must not become collaborative map evidence.
    """
    rid = request.query_params.get("robot_id", "")
    kind = request.query_params.get("kind", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    if kind not in {"global", "local"}:
        return JSONResponse(
            {"error": "kind must be global or local"}, status_code=400
        )
    try:
        resolution = float(request.query_params.get("resolution", 0.0))
        width = int(request.query_params.get("width", 0))
        height = int(request.query_params.get("height", 0))
        origin_x = float(request.query_params.get("origin_x", 0.0))
        origin_y = float(request.query_params.get("origin_y", 0.0))
    except (TypeError, ValueError):
        return JSONResponse({"error": "malformed costmap metadata"}, status_code=400)
    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
        or width <= 0
        or height <= 0
        or width * height > MAX_UPLOAD_BYTES
        or not math.isfinite(origin_x)
        or not math.isfinite(origin_y)
    ):
        return JSONResponse({"error": "invalid costmap dimensions or geometry"}, status_code=400)

    try:
        raw = _inflate(await request.body())
        cells = np.frombuffer(raw, dtype=np.int8)
    except (zlib.error, ValueError) as exc:
        return JSONResponse({"error": f"malformed costmap: {exc}"}, status_code=400)
    if cells.size != width * height:
        return JSONResponse({"error": "costmap size mismatch"}, status_code=400)

    # Be defensive about adapters that hand us a value outside Nav2's range.
    normalized = np.where(cells < 0, -1, np.clip(cells, 0, 100)).astype(np.int8)
    frame_id = str(request.query_params.get("frame_id", "") or "").lstrip("/")
    now = time.time()
    key = (rid, kind)
    with _costmap_lock:
        previous = _costmaps.get(key)
        entry = CostmapEntry(
            robot_id=rid,
            kind=kind,
            meta=GridMeta(resolution, width, height, origin_x, origin_y),
            cells=np.ascontiguousarray(normalized.reshape(height, width)).copy(),
            frame_id=frame_id,
            seq=(previous.seq + 1) if previous else 1,
            updated_at=now,
            dirty=True,
        )
        _costmaps[key] = entry
    return {"ok": True, "kind": kind, "seq": entry.seq, "cells": int(cells.size)}


async def post_global_map(request: Request) -> Any:
    """Adopt a collaborative backend's already-merged common-frame grid."""
    try:
        meta = GridMeta(
            resolution=float(request.query_params.get("resolution", 0.05)),
            width=int(request.query_params.get("width", 0)),
            height=int(request.query_params.get("height", 0)),
            origin_x=float(request.query_params.get("origin_x", 0.0)),
            origin_y=float(request.query_params.get("origin_y", 0.0)),
        )
    except (TypeError, ValueError):
        return JSONResponse({"error": "malformed grid metadata"}, status_code=400)
    if meta.width <= 0 or meta.height <= 0:
        return JSONResponse({"error": "width and height required"}, status_code=400)
    try:
        raw = _inflate(await request.body())
        cells = np.frombuffer(raw, dtype=np.int8)
    except (zlib.error, ValueError):
        return JSONResponse({"error": "malformed grid"}, status_code=400)
    if cells.size != meta.width * meta.height:
        return JSONResponse({"error": "size mismatch"}, status_code=400)
    map_service.set_global_grid(meta, cells.reshape(meta.height, meta.width))
    return {"ok": True, "cells": int(cells.size)}


async def post_cloud(request: Request) -> Any:
    """Adapter 3D map cloud upload (zlib int16 xyz triples)."""
    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    scale = float(request.query_params.get("scale", CLOUD_SCALE))
    try:
        raw = _inflate(await request.body())
        quantised = np.frombuffer(raw, dtype=np.int16)
    except (zlib.error, ValueError) as exc:
        return JSONResponse({"error": f"malformed cloud: {exc}"}, status_code=400)
    if quantised.size % 3:
        return JSONResponse({"error": "xyz triples expected"}, status_code=400)
    points = quantised.reshape(-1, 3).astype(np.float32) * scale
    await map_service.set_cloud_async(rid, points)
    return {"ok": True, "points": int(len(points))}


async def post_scan(request: Request) -> Any:
    """Adapter lidar scan upload for robots without an OccupancyGrid."""
    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    try:
        origin_x = float(request.query_params["origin_x"])
        origin_y = float(request.query_params["origin_y"])
    except (KeyError, ValueError):
        return JSONResponse({"error": "origin_x/origin_y required"}, status_code=400)
    scale = float(request.query_params.get("scale", CLOUD_SCALE))
    retain_free_space = request.query_params.get(
        "retain_free_space", "0"
    ).strip().lower() in {"1", "true", "yes", "on"}
    try:
        raw = _inflate(await request.body())
        quantised = np.frombuffer(raw, dtype=np.int16)
    except (zlib.error, ValueError) as exc:
        return JSONResponse({"error": f"malformed scan: {exc}"}, status_code=400)
    if quantised.size % 2:
        return JSONResponse({"error": "xy pairs expected"}, status_code=400)
    points = quantised.reshape(-1, 2).astype(np.float32) * scale
    await map_service.ingest_scan_async(
        rid,
        origin_x,
        origin_y,
        points,
        retain_free_space=retain_free_space,
    )
    return {"ok": True, "points": int(len(points))}


async def post_keyframe(request: Request) -> Any:
    """Adapter keyframe upload. Validates identity, forwards the opaque body.

    The server does not decode the cloud and does not run the optimizer. A
    mismatch between the query-string robot_id and the blob's own robot_id is
    rejected so one robot cannot inject another robot's trajectory.
    """
    from swarmdeck_protocol import (
        MAX_KEYFRAME_BYTES,
        ProtocolError,
        peek_keyframe_header,
    )

    from ..mapsvc import graph_bridge

    rid = request.query_params.get("robot_id", "")
    if not rid:
        return JSONResponse({"error": "robot_id required"}, status_code=400)
    body = await request.body()
    if not body:
        return JSONResponse({"error": "empty body"}, status_code=400)
    if len(body) > MAX_KEYFRAME_BYTES:
        return JSONResponse({"error": "keyframe too large"}, status_code=413)
    try:
        header = peek_keyframe_header(body)
    except ProtocolError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
    blob_id = str(header.get("robot_id", ""))
    if blob_id != rid:
        return JSONResponse(
            {"error": f"robot_id mismatch: query {rid!r} vs blob {blob_id!r}"},
            status_code=400,
        )
    result = graph_bridge.enqueue(body)
    return {"ok": True, "seq": header.get("seq"), **result}


async def post_slam_update(request: Request) -> Any:
    """Pose-graph back-end snapshot: membership, T_world_map, common poses."""
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    if not isinstance(payload, dict):
        return JSONResponse({"error": "JSON object required"}, status_code=400)
    try:
        map_service.apply_slam_update(payload)
    except (TypeError, ValueError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
    dropped = _prune_optimized_maps(payload.get("scopes"))
    return {
        "ok": True,
        "robots": sorted((payload.get("graphs") or {}).keys()),
        "dropped_scopes": dropped,
    }


def _prune_optimized_maps(scopes: Any) -> list[str]:
    """Forget every scoped grid the back-end no longer publishes.

    Scoped grids only ever arrive; nothing here could tell a live scope from a
    dead one on its own. ``component:<n>`` ids are positional over the
    back-end's sorted union-find roots, so merging two robots drops the
    component count and permanently retires the highest id -- and that grid,
    a snapshot of a merge that no longer describes the fleet, would otherwise
    be served from ``/api/map/optimized`` forever.

    A back-end that sends no ``scopes`` key (an older one) prunes nothing, so
    this cannot empty the store by accident.
    """
    if not isinstance(scopes, list) or not all(isinstance(s, str) for s in scopes):
        return []
    live = set(scopes)
    with _optimized_lock:
        dead = sorted(set(_optimized) - live)
        for scope in dead:
            del _optimized[scope]
    return dead


async def get_nav_map(request: Request, robot_id: str) -> Response:
    """Occupancy in this robot's map frame, for Nav2's global costmap.

    The merged grid warped into that frame once the robot is in a multi-robot
    component, and the robot's OWN raytraced grid before then -- see
    ``MapService.nav_grid``, which falls back deliberately so Nav2's static
    layer initializes without waiting for a merge. 404 only when the robot has
    no map at all. 304 if the caller already has the current seq.

    Local costmaps must not subscribe to the OccupancyGrid this becomes.
    """
    wanted = request.headers.get("if-none-match")
    product = map_service.nav_grid(robot_id)
    if product is None:
        return JSONResponse({"error": "nav map not available"}, status_code=404)
    meta, cells, seq = product
    if wanted is not None and wanted.strip() == str(seq):
        return Response(status_code=304)
    body = zlib.compress(np.ascontiguousarray(cells).tobytes())
    return Response(
        content=body,
        media_type="application/octet-stream",
        headers={
            "Cache-Control": "no-store",
            "ETag": str(seq),
            "X-Map-Seq": str(seq),
            "X-Map-Resolution": str(meta.resolution),
            "X-Map-Width": str(meta.width),
            "X-Map-Height": str(meta.height),
            "X-Map-Origin-X": str(meta.origin_x),
            "X-Map-Origin-Y": str(meta.origin_y),
        },
    )


async def get_cloud(request: Request | None = None) -> Response:
    """Merged 3D cloud or single robot 3D cloud for the GUI's 3D view."""
    robot_id = str(request.query_params.get("robot_id", "") or "") if request else ""
    points, indices, names = map_service.merged_cloud(robot_id=robot_id or None)
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


async def post_optimized_map(request: Request) -> Any:
    """Accept one scoped optimized grid from the SLAM back-end.

    ``scope`` is opaque here: the back-end names them ``robot:<id>`` and
    ``component:<n>``, and this endpoint deliberately does not parse or validate
    that shape, so adding a scope later needs no server change.
    """
    scope = request.query_params.get("scope", "")
    if not scope:
        return JSONResponse({"error": "scope required"}, status_code=400)
    try:
        meta = GridMeta(
            resolution=float(request.query_params.get("resolution", 0.05)),
            width=int(request.query_params.get("width", 0)),
            height=int(request.query_params.get("height", 0)),
            origin_x=float(request.query_params.get("origin_x", 0.0)),
            origin_y=float(request.query_params.get("origin_y", 0.0)),
        )
    except (TypeError, ValueError):
        return JSONResponse({"error": "malformed grid metadata"}, status_code=400)
    if meta.width <= 0 or meta.height <= 0:
        return JSONResponse({"error": "width and height required"}, status_code=400)
    body = await request.body()
    if len(body) > MAX_UPLOAD_BYTES:
        return JSONResponse({"error": "grid too large"}, status_code=413)
    try:
        cells = np.frombuffer(_inflate(body), dtype=np.int8)
    except (zlib.error, ValueError):
        return JSONResponse({"error": "malformed grid"}, status_code=400)
    if cells.size != meta.width * meta.height:
        return JSONResponse({"error": "size mismatch"}, status_code=400)
    robots = tuple(r for r in request.query_params.get("robots", "").split(",") if r)
    with _optimized_lock:
        _optimized[scope] = (meta, cells.reshape(meta.height, meta.width), robots)
    return {"ok": True, "scope": scope, "cells": int(cells.size)}


async def get_optimized_index() -> dict[str, Any]:
    """Which optimized scopes exist, so the UI can offer them without guessing."""
    with _optimized_lock:
        items = [
            {
                "scope": scope,
                "robots": list(robots),
                "resolution": meta.resolution,
                "width": meta.width,
                "height": meta.height,
                "origin": {"x": meta.origin_x, "y": meta.origin_y},
            }
            for scope, (meta, _cells, robots) in sorted(_optimized.items())
        ]
    return {"type": "optimized_maps", "maps": items}


async def get_optimized_map(scope: str) -> Response:
    with _optimized_lock:
        entry = _optimized.get(scope)
    if entry is None:
        return JSONResponse(
            {"error": f"no optimized map for {scope!r}"}, status_code=404
        )
    meta, cells, _robots = entry
    from ..mapsvc.output import grid_png

    return Response(
        content=grid_png(meta, cells),
        media_type="image/png",
        headers={"Cache-Control": "no-cache"},
    )


async def get_slam_backend() -> Any:
    """Operator view of the pose-graph process: status, merge knobs, reachability."""
    from ..mapsvc import graph_bridge

    status_code, status_body = await asyncio.to_thread(graph_bridge.fetch_json, "/status")
    config_code, config_body = (503, {})
    if status_code == 200:
        config_code, config_body = await asyncio.to_thread(
            graph_bridge.fetch_json, "/config"
        )
    reachable = status_code == 200
    payload = {
        "ok": reachable,
        "reachable": reachable,
        "status": status_body if status_code == 200 else None,
        "settings": config_body.get("settings") if config_code == 200 else None,
        "defaults": config_body.get("defaults") if config_code == 200 else None,
        "error": None if reachable else status_body.get("error", "slam unreachable"),
    }
    return JSONResponse(payload, status_code=200 if reachable else 503)


async def put_slam_config(request: Request) -> Any:
    """Forward merge knobs to the slam process. Never runs the solver here."""
    from ..mapsvc import graph_bridge

    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    if not isinstance(payload, dict):
        return JSONResponse({"error": "JSON object required"}, status_code=400)
    code, body = await asyncio.to_thread(graph_bridge.put_json, "/config", payload)
    return JSONResponse(body, status_code=code)


def reset_optimized_maps() -> None:
    """Drop every scoped grid. Used when the session or the graph resets."""
    with _optimized_lock:
        _optimized.clear()
