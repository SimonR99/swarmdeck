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
import urllib.request
import zlib
from collections import deque
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
    snapshot_update,
)

app = FastAPI(title="SwarmDeck SLAM")
backend = CollaborativeBackend()

_queue: deque[bytes] = deque()
_queue_lock = threading.Lock()
_queue_cap = 64
_dropped = 0
_ingested = 0
_last_error = ""
_last_snapshot: BackendSnapshot | None = None
_stop = threading.Event()
_worker: threading.Thread | None = None

SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "").rstrip("/")
OPTIMIZE_EVERY_N = int(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_EVERY", "5"))
OPTIMIZE_EVERY_S = float(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_S", "2.0"))
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


def _worker_loop() -> None:
    last_optimize = 0.0
    while not _stop.is_set():
        blob: bytes | None = None
        with _queue_lock:
            if _queue:
                blob = _queue.popleft()
        if blob is None:
            _stop.wait(0.05)
            continue
        try:
            packet = decode_keyframe(blob)
            backend.ingest_packet(packet)
        except (ProtocolError, ValueError) as exc:
            global _last_error
            _last_error = str(exc)
            continue
        global _ingested
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


@app.on_event("startup")
def _startup() -> None:
    global _worker
    _stop.clear()
    _worker = threading.Thread(target=_worker_loop, name="slam-worker", daemon=True)
    _worker.start()


@app.on_event("shutdown")
def _shutdown() -> None:
    _stop.set()
    if _worker is not None:
        _worker.join(timeout=2.0)


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
