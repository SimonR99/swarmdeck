"""Forward adapter keyframes to the SLAM process without blocking ingest.

The server is a dumb pipe: it checks identity, queues the opaque blob, and a
background task POSTs it to the SLAM service. If that service is down the
queue drops rather than blocking -- adapters must never wait on optimization.
"""

from __future__ import annotations

import asyncio
import os
from collections import deque
from typing import Any

import urllib.error
import urllib.request

SLAM_URL = os.environ.get("SWARMDECK_SLAM_URL", "").rstrip("/")
FORWARD_TIMEOUT_S = 2.0
QUEUE_CAP = 32

_queue: deque[bytes] = deque()
_dropped = 0
_forwarded = 0
_last_error = ""
_task: asyncio.Task[None] | None = None


def configure(url: str | None = None) -> None:
    """Tests and config loaders set the slam URL without touching the environ."""
    global SLAM_URL
    if url is not None:
        SLAM_URL = url.rstrip("/")
    else:
        SLAM_URL = os.environ.get("SWARMDECK_SLAM_URL", "").rstrip("/")


def status() -> dict[str, Any]:
    return {
        "url": SLAM_URL,
        "queued": len(_queue),
        "dropped": _dropped,
        "forwarded": _forwarded,
        "last_error": _last_error,
    }


def enqueue(blob: bytes) -> dict[str, Any]:
    """Non-blocking. Drops the oldest blob when the queue is full."""
    global _dropped
    if len(_queue) >= QUEUE_CAP:
        _queue.popleft()
        _dropped += 1
    _queue.append(blob)
    return {"ok": True, "queued": len(_queue), "dropped": _dropped, "forwarded": False}


async def start_worker() -> None:
    global _task
    if _task is None or _task.done():
        _task = asyncio.create_task(_forward_loop())


async def stop_worker() -> None:
    global _task
    if _task is not None:
        _task.cancel()
        try:
            await _task
        except asyncio.CancelledError:
            pass
        _task = None


def reset() -> None:
    global _dropped, _forwarded, _last_error
    _queue.clear()
    _dropped = 0
    _forwarded = 0
    _last_error = ""


async def _forward_loop() -> None:
    global _forwarded, _last_error
    while True:
        if not _queue or not SLAM_URL:
            await asyncio.sleep(0.05)
            continue
        blob = _queue.popleft()
        try:
            await asyncio.to_thread(_post_keyframe, blob)
            _forwarded += 1
            _last_error = ""
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            _last_error = str(exc)
            # Do not put the blob back: a down slam service would then pin the
            # queue forever. The next keyframe continues the trajectory via
            # relative odometry even with a gap.
            await asyncio.sleep(0.2)


def _post_keyframe(blob: bytes) -> None:
    urllib.request.urlopen(
        urllib.request.Request(
            f"{SLAM_URL}/keyframe",
            data=blob,
            headers={"Content-Type": "application/octet-stream"},
            method="POST",
        ),
        timeout=FORWARD_TIMEOUT_S,
    ).read()


def post_reset() -> None:
    """Best-effort: tell the slam process to forget the session."""
    reset()
    if not SLAM_URL:
        return
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                f"{SLAM_URL}/reset",
                data=b"",
                method="POST",
            ),
            timeout=FORWARD_TIMEOUT_S,
        ).read()
    except (urllib.error.URLError, TimeoutError, OSError):
        pass
