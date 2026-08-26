"""HTTP process wrapping :class:`CollaborativeBackend`.

Two reasons this is not a module inside the FastAPI server:

1. gtsam 4.2.2 segfaults under numpy 2.x, which is what the server runs.
2. Optimize + GICP must never run on the server's asyncio loop -- that is
   exactly the stall that previously knocked the live fleet offline.

The SwarmDeck server is a dumb pipe: it validates identity and POSTs the
opaque blob here. This process decodes, optimizes, renders, and pushes the
result back with ``POST /api/slam/update`` and ``POST /api/adapter/global_map``.
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib
from collections import deque
from contextlib import asynccontextmanager
from typing import Any

import numpy as np
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from swarmdeck_protocol import (
    MAX_KEYFRAME_BYTES,
    ProtocolError,
    decode_keyframe,
)
from swarmdeck_slam.backend import (
    BackendSnapshot,
    CollaborativeBackend,
    majority_component,
    scoped_grids,
    snapshot_update,
)
from swarmdeck_slam.render import RenderConfig

# Occupancy is a PROJECTION of the keyframe clouds over a height band, never a
# dump of every point. With a single-ring lidar the distinction is invisible --
# every return sits at one height, so an open band and a tight one render the
# same grid. With 33 rings it is the difference between a floor plan and a solid
# rectangle: the downward rings see floor and the upward rings see ceiling, and
# both rasterize as walls if nothing filters them out. Leaving RenderConfig at
# its -inf/+inf default was harmless only while the fleet was planar.
#
# Bounds are read in the KEYFRAME frame, which is base_link -- keyframe_producer
# transforms map->base at capture. floor_z is therefore minus the platform's
# base_link height above the floor (scout_mini: 0.1225, see spawn_fleet.py's
# ROBOT_PROFILES), which puts the band at 0.0275..0.2725 in base_link. The band
# itself reuses the calibrated hardware vocabulary from
# adapters/adapter_ros1/config/scout_mini.yaml: 15 cm above the floor through
# chassis height plus 15 cm.
#
# One band serves the whole fleet, which is correct while every robot shares a
# chassis and wrong the moment a bunker (base_height 0.200) maps alongside a
# scout_mini. Move it onto the keyframe rather than widening it if that happens:
# a band wide enough for both admits the taller robot's floor returns.
RENDER = RenderConfig(
    floor_z=float(os.environ.get("SWARMDECK_SLAM_FLOOR_Z", "0.0")),
    min_z=float(os.environ.get("SWARMDECK_SLAM_MIN_Z", "0.08")),
    max_z=float(os.environ.get("SWARMDECK_SLAM_MAX_Z", "2.20")),
)

backend = CollaborativeBackend(render=RENDER)

_queue: deque[bytes] = deque()
_queue_lock = threading.Lock()
_queue_cap = 64
_dropped = 0
_ingested = 0
_last_error = ""
_last_snapshot: BackendSnapshot | None = None
_stop = threading.Event()
_worker: threading.Thread | None = None

# Offline-replay capture. Set SWARMDECK_SLAM_CAPTURE_DIR to record every blob
# this service accepts, so a Gazebo run can be turned into a dataset once and
# replayed against the backend in seconds instead of re-flown for every
# parameter change.
#
# Captured at ACCEPT time, deliberately before the bounded queue: a dataset
# wants everything the fleet actually sent, including blobs a busy optimizer
# would have dropped. Filenames are the arrival index and nothing else --
# replay order defines the odometry chain (backend._last_of), so preserving
# arrival order is what makes a replay reproduce the live graph rather than
# merely resemble it.
#
# Never let capture break ingestion: a full disk must cost a dataset, not the
# live map.
CAPTURE_DIR = os.environ.get("SWARMDECK_SLAM_CAPTURE_DIR", "")
_capture_seq = 0
_capture_lock = threading.Lock()
_capture_failed = False


def _capture(blob: bytes) -> None:
    global _capture_seq, _capture_failed, _last_error
    if not CAPTURE_DIR or _capture_failed:
        return
    with _capture_lock:
        index = _capture_seq
        _capture_seq += 1
    try:
        directory = os.path.join(CAPTURE_DIR, "keyframes")
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, f"{index:06d}.kf"), "wb") as handle:
            handle.write(blob)
    except OSError as exc:
        _capture_failed = True
        _last_error = f"keyframe capture disabled: {exc}"


SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "").rstrip("/")
OPTIMIZE_EVERY_N = int(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_EVERY", "1"))
OPTIMIZE_EVERY_S = float(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_S", "1.0"))
PUBLISH_TIMEOUT_S = 5.0


def _publish_snapshot(snapshot: BackendSnapshot) -> None:
    """Push origins + the majority-component grid to the SwarmDeck server."""
    global _last_snapshot, _last_error
    _last_snapshot = snapshot
    if not SERVER_URL:
        return
    body = snapshot_update(snapshot)
    try:
        req = urllib.request.Request(
            f"{SERVER_URL}/api/slam/update",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=PUBLISH_TIMEOUT_S).read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _last_error = f"slam update failed: {exc}"
        return

    # Per-robot and per-component grids, then the merged one.
    #
    # The merged map deliberately shows only the majority component, because
    # overlaying two components that share no verified transform is a confident
    # lie. That is the right call for the fleet view and it leaves the operator
    # unable to see a robot that merged with nobody -- exactly the case worth
    # looking at. These scoped grids fill that gap without weakening the rule:
    # each is a separate, separately-labelled map, never overlaid.
    for scope, grid in scoped_grids(snapshot):
        _publish_grid(scope, grid)

    grid = majority_component(snapshot)
    if grid is None:
        return
    cells = np.ascontiguousarray(grid.cells)
    payload = zlib.compress(cells.tobytes())
    url = (
        f"{SERVER_URL}/api/adapter/global_map"
        f"?resolution={grid.resolution}&width={grid.width}&height={grid.height}"
        f"&origin_x={grid.origin_x}&origin_y={grid.origin_y}"
    )
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                url,
                data=payload,
                headers={"Content-Type": "application/octet-stream"},
                method="POST",
            ),
            timeout=PUBLISH_TIMEOUT_S,
        ).read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _last_error = f"global map publish failed: {exc}"


def _publish_grid(scope: str, grid: Any) -> None:
    """POST one scoped optimized grid. Failure is logged, never raised.

    A scoped grid is a view, not the map Nav2 drives on, so losing one must not
    disturb the merged publish that follows it.
    """
    global _last_error
    if not SERVER_URL:
        return
    payload = zlib.compress(np.ascontiguousarray(grid.cells).tobytes())
    url = (
        f"{SERVER_URL}/api/slam/optimized_map"
        f"?scope={urllib.parse.quote(scope)}"
        f"&resolution={grid.resolution}&width={grid.width}&height={grid.height}"
        f"&origin_x={grid.origin_x}&origin_y={grid.origin_y}"
        f"&robots={urllib.parse.quote(','.join(sorted(grid.robots)))}"
    )
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                url,
                data=payload,
                headers={"Content-Type": "application/octet-stream"},
                method="POST",
            ),
            timeout=PUBLISH_TIMEOUT_S,
        ).read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _last_error = f"optimized grid publish failed ({scope}): {exc}"


def _worker_loop() -> None:
    """Drain everything queued, then optimize once over the lot.

    Ingest is milliseconds; optimize+render is seconds and grows with graph
    size (a 240-keyframe two-robot fleet measured ~7 s per cycle before the
    render was deduplicated). Optimizing after *each* blob meant a worker that
    had fallen behind spent its way further behind -- 64 queued keyframes cost
    64 full re-solves to produce 64 snapshots, 63 of which nobody would ever
    see, while the queue kept overflowing and dropping real keyframes.

    Draining first costs nothing when keeping up (the queue holds one blob, so
    this is still optimize-per-keyframe at ``OPTIMIZE_EVERY_N=1``) and degrades
    into "solve the newest state, once" exactly when that is the only thing
    that can catch up.
    """
    global _ingested, _last_error
    last_optimize = 0.0
    while not _stop.is_set():
        blobs: list[bytes] = []
        with _queue_lock:
            while _queue:
                blobs.append(_queue.popleft())
        if not blobs:
            _stop.wait(0.05)
            continue
        for blob in blobs:
            try:
                backend.ingest_packet(decode_keyframe(blob))
            except (ProtocolError, ValueError) as exc:
                _last_error = str(exc)
                continue
            _ingested += 1
        now = time.monotonic()
        due = (
            backend.new_since_optimize >= OPTIMIZE_EVERY_N
            or (now - last_optimize) >= OPTIMIZE_EVERY_S
        )
        if backend.dirty and due:
            snapshot = backend.optimize_and_render()
            last_optimize = now
            if snapshot is not None:
                _publish_snapshot(snapshot)


@asynccontextmanager
async def lifespan(_: FastAPI):
    global _worker
    _stop.clear()
    _worker = threading.Thread(target=_worker_loop, name="slam-worker", daemon=True)
    _worker.start()
    try:
        yield
    finally:
        _stop.set()
        if _worker is not None:
            _worker.join(timeout=2.0)


app = FastAPI(title="SwarmDeck SLAM", lifespan=lifespan)


@app.get("/health")
def health() -> dict[str, Any]:
    return {"ok": True, "keyframes": len(backend), "queued": _queued()}


@app.get("/status")
def status() -> dict[str, Any]:
    snapshot = _last_snapshot
    return {
        "ok": True,
        "keyframes": len(backend),
        "queued": _queued(),
        "dropped": _dropped,
        "ingested": _ingested,
        "dirty": backend.dirty,
        "last_error": _last_error,
        "has_snapshot": snapshot is not None,
        "components": (
            [
                {"id": c.component_id, "robots": sorted(c.robots)}
                for c in snapshot.optimized.components
            ]
            if snapshot is not None
            else []
        ),
        "keyframe_counts": snapshot.keyframe_counts if snapshot is not None else {},
        "accepted_closures": snapshot.accepted_closures if snapshot is not None else 0,
        "inter_robot_closures": (
            snapshot.inter_robot_closures if snapshot is not None else 0
        ),
        "server_url": SERVER_URL,
    }


@app.post("/keyframe")
async def post_keyframe(request: Request) -> Any:
    """Accept one opaque keyframe blob. Never runs the optimizer here."""
    global _dropped
    body = await request.body()
    if not body:
        return JSONResponse({"error": "empty body"}, status_code=400)
    if len(body) > MAX_KEYFRAME_BYTES:
        return JSONResponse({"error": "keyframe too large"}, status_code=413)
    _capture(body)
    with _queue_lock:
        if len(_queue) >= _queue_cap:
            _queue.popleft()
            _dropped += 1
        _queue.append(body)
    return {"ok": True, "queued": _queued(), "dropped": _dropped}


@app.post("/reset")
def post_reset() -> dict[str, Any]:
    with _queue_lock:
        _queue.clear()
    backend.reset()
    global _last_snapshot, _ingested, _dropped, _last_error
    _last_snapshot = None
    _ingested = 0
    _dropped = 0
    _last_error = ""
    return {"ok": True}


def _queued() -> int:
    with _queue_lock:
        return len(_queue)


def main() -> None:
    parser = argparse.ArgumentParser(prog="swarmdeck-slam")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument(
        "--server-url",
        default=os.environ.get("SWARMDECK_SERVER_URL", ""),
        help="SwarmDeck server to push merged maps back to",
    )
    args = parser.parse_args()
    global SERVER_URL
    SERVER_URL = args.server_url.rstrip("/")
    print(f"[slam] http://{args.host}:{args.port}  server={SERVER_URL or '(none)'}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
