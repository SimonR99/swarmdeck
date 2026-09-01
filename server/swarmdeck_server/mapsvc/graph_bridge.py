"""Forward adapter keyframes to the SLAM process without blocking ingest.

The server is a dumb pipe: it checks identity, queues the opaque blob, and a
background task POSTs it to the SLAM service. If that service is down the
queue drops rather than blocking -- adapters must never wait on optimization.
"""

from __future__ import annotations

import asyncio
import json
import os
from collections import deque
from typing import Any

import urllib.error
import urllib.parse
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


def delete_robot(robot_id: str) -> tuple[int, dict[str, Any]]:
    """Permanently remove one robot's keyframes from the graph service."""
    if not SLAM_URL:
        return 503, {"error": "slam service is not configured"}
    path = urllib.parse.quote(robot_id, safe="")
    try:
        request = urllib.request.Request(
            f"{SLAM_URL}/robots/{path}/keyframes",
            data=b"",
            method="DELETE",
        )
        with urllib.request.urlopen(request, timeout=FORWARD_TIMEOUT_S) as response:
            body = json.loads(response.read().decode())
            if not isinstance(body, dict):
                return 502, {"error": "slam service returned a non-object"}
            return int(response.status), body
    except urllib.error.HTTPError as exc:
        try:
            payload = json.loads(exc.read().decode())
        except Exception:
            payload = {"error": str(exc)}
        return int(exc.code), payload if isinstance(payload, dict) else {"error": str(exc)}
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return 503, {"error": str(exc)}


def fetch_json(path: str, timeout: float = 2.0) -> tuple[int, dict[str, Any]]:
    """GET a JSON path on the slam process. 503 when it is not configured or down."""
    if not SLAM_URL:
        return 503, {"error": "slam service is not configured"}
    try:
        with urllib.request.urlopen(f"{SLAM_URL}{path}", timeout=timeout) as response:
            body = json.loads(response.read().decode())
            if not isinstance(body, dict):
                return 502, {"error": "slam service returned a non-object"}
            return 200, body
    except urllib.error.HTTPError as exc:
        try:
            payload = json.loads(exc.read().decode())
        except Exception:
            payload = {"error": str(exc)}
        if not isinstance(payload, dict):
            payload = {"error": str(exc)}
        return int(exc.code), payload
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return 503, {"error": str(exc)}


def put_json(path: str, payload: dict[str, Any], timeout: float = 2.0) -> tuple[int, dict[str, Any]]:
    """PUT JSON to the slam process."""
    if not SLAM_URL:
        return 503, {"error": "slam service is not configured"}
    data = json.dumps(payload).encode()
    try:
        request = urllib.request.Request(
            f"{SLAM_URL}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="PUT",
        )
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode())
            if not isinstance(body, dict):
                return 502, {"error": "slam service returned a non-object"}
            return 200, body
    except urllib.error.HTTPError as exc:
        try:
            parsed = json.loads(exc.read().decode())
        except Exception:
            parsed = {"error": str(exc)}
        if not isinstance(parsed, dict):
            parsed = {"error": str(exc)}
        return int(exc.code), parsed
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return 503, {"error": str(exc)}
