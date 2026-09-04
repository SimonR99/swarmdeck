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
import uuid
import zlib
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import numpy as np
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from swarmdeck_protocol import (
    MAX_KEYFRAME_BYTES,
    ProtocolError,
    decode_keyframe,
    peek_keyframe_header,
)
from swarmdeck_slam.backend import (
    BackendSnapshot,
    CollaborativeBackend,
    operator_setting_defaults,
    majority_component,
    scoped_grids,
    snapshot_update,
)
from swarmdeck_slam.render import RenderConfig
from swarmdeck_slam.types import TrajectoryId, se3_from_quat_xyz

# Occupancy is a PROJECTION of the keyframe clouds over a height band, never a
# dump of every point. With a single-ring lidar the distinction is invisible --
# every return sits at one height, so an open band and a tight one render the
# same grid. With 33 rings it is the difference between a floor plan and a solid
# rectangle: the downward rings see floor and the upward rings see ceiling, and
# both rasterize as walls if nothing filters them out. Leaving RenderConfig at
# its -inf/+inf default was harmless only while the fleet was planar.
#
# Bounds are read in the KEYFRAME frame, which is base_link -- keyframe_producer
# transforms map->base at capture. Current producers carry their own ground
# plane, lidar height, and physical 0.15..1.80 m band in the keyframe header;
# the values below are only the compatibility fallback for old packets or a
# profile without floor calibration. The environment variables remain useful
# when replaying such legacy captures.
REGISTRATION_MODE = os.environ.get("SWARMDECK_SLAM_REGISTRATION_MODE", "graph")
ANCHOR_ROBOT = os.environ.get("SWARMDECK_SLAM_ANCHOR_ROBOT", "").strip() or None
RENDER = RenderConfig(
    floor_z=float(os.environ.get("SWARMDECK_SLAM_FLOOR_Z", "0.0")),
    min_z=float(os.environ.get("SWARMDECK_SLAM_MIN_Z", "0.15")),
    max_z=float(os.environ.get("SWARMDECK_SLAM_MAX_Z", "1.80")),
    odometry_as_pose=REGISTRATION_MODE != "odom_free",
    close_occupied=1,
    hit_weight=8,
    peer_exclusion_radius_m=float(
        os.environ.get("SWARMDECK_SLAM_PEER_EXCLUSION_RADIUS_M", "0.80")
    ),
    peer_exclusion_max_dt_s=float(
        os.environ.get("SWARMDECK_SLAM_PEER_EXCLUSION_MAX_DT_S", "2.0")
    ),
    peer_exclusion_max_interp_gap_s=float(
        os.environ.get("SWARMDECK_SLAM_PEER_EXCLUSION_MAX_INTERP_GAP_S", "15.0")
    ),
)

# gtsam tangent order: rx, ry, rz, tx, ty, tz. Tight enough that a working
# onboard SLAM trajectory stays rigid; loose enough that a genuine residual
# of a few centimetres can still be absorbed.
_POSE_PRIOR_SIGMAS = np.array([0.05, 0.05, 0.05, 0.10, 0.10, 0.15])


def _start_pose_hints(path: str) -> dict[str, np.ndarray] | None:
    """Read ``map.start_poses`` from the session yaml, if present.

    These are Gazebo spawn poses or surveyed first-observation poses. Graph
    mode uses them as ``T_world_map`` so onboard maps overlay in the world
    frame. Odometry-free mode uses them only to choose between already-valid
    symmetric geometric modes and to rigidly gauge each connected component;
    they never pin individual keyframes. The field may be empty when hardware
    starts are unknown.
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        text = Path(path).read_text()
    except OSError:
        return None
    import math
    import re

    matches = re.findall(
        r"(robot_\w+):\s*\{x:\s*([-\d.e]+),\s*y:\s*([-\d.e]+),\s*yaw:\s*([-\d.e]+)\}",
        text,
    )
    if not matches:
        return None
    hints = {}
    for robot_id, x, y, yaw in matches:
        yaw_f = float(yaw)
        hints[robot_id] = se3_from_quat_xyz(
            [
                float(x),
                float(y),
                0.0,
                0.0,
                0.0,
                math.sin(yaw_f / 2.0),
                math.cos(yaw_f / 2.0),
            ]
        )
    return hints or None


backend = CollaborativeBackend(
    render=RENDER,
    registration_mode=REGISTRATION_MODE,
    pose_prior_sigmas=_POSE_PRIOR_SIGMAS if REGISTRATION_MODE == "graph" else None,
    anchor_robot_id=ANCHOR_ROBOT,
    t_world_map_hint=_start_pose_hints(os.environ.get("SWARMDECK_CONFIG", "")),
)

_queue: deque[bytes] = deque()
_queue_lock = threading.Lock()
# Makes capture persistence and admission to the in-memory queue one operation
# with respect to reset/delete. A blob is therefore unambiguously before the
# reset (archived + dropped) or after it (kept + queued), never one of each.
_ingress_lock = threading.Lock()
_queue_cap = int(os.environ.get("SWARMDECK_SLAM_QUEUE_CAP", "2048"))
_controls: deque[tuple[str, str | None]] = deque()
_control_lock = threading.Lock()
_generation = 0
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
RESTORE_CAPTURE = os.environ.get("SWARMDECK_SLAM_RESTORE_CAPTURE", "false").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
#: -1 until the first capture, when it resumes past whatever is already on disk.
_capture_seq = -1
_capture_lock = threading.Lock()
_capture_failed = False
_restored = 0


def _resume_capture_index(directory: str) -> int:
    """Next free index in ``directory``, so a restart appends instead of
    overwriting.

    Filenames are the arrival index alone, and the counter lives in module
    state, so a restarted service would begin again at 000000 and silently
    overwrite an existing dataset from the beginning -- losing exactly the
    capture someone had been collecting, with no error anywhere. Resuming past
    the highest file on disk keeps a restarted session appending to the same
    dataset, which also preserves the arrival order that makes a replay
    reproduce the live graph.
    """
    try:
        existing = [
            int(name[:-3])
            for name in os.listdir(directory)
            if name.endswith(".kf") and name[:-3].isdigit()
        ]
    except OSError:
        return 0
    return max(existing) + 1 if existing else 0


def _capture(blob: bytes) -> None:
    global _capture_seq, _capture_failed, _last_error
    if not CAPTURE_DIR or _capture_failed:
        return
    with _capture_lock:
        if _capture_seq < 0:
            _capture_seq = _resume_capture_index(os.path.join(CAPTURE_DIR, "keyframes"))
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


def _archive_capture(robot_id: str | None = None) -> int:
    """Move captured keyframes out of the restore path, recoverably.

    A map reset that clears only RAM is undone by the next container restart
    when ``RESTORE_CAPTURE`` is enabled.  Archived blobs live under
    ``discarded/`` for forensic recovery, but startup only replays
    ``keyframes/*.kf``.
    """
    if not CAPTURE_DIR:
        return 0
    source = Path(CAPTURE_DIR) / "keyframes"
    try:
        paths = sorted(source.glob("*.kf"))
    except OSError:
        return 0

    selected: list[Path] = []
    for path in paths:
        if robot_id is None:
            selected.append(path)
            continue
        try:
            header = peek_keyframe_header(path.read_bytes())
        except (OSError, ProtocolError):
            continue
        if str(header.get("robot_id", "")) == robot_id:
            selected.append(path)
    if not selected:
        return 0

    label = "fleet" if robot_id is None else robot_id
    destination = (
        Path(CAPTURE_DIR)
        / "discarded"
        / f"{int(time.time())}-{label}-{uuid.uuid4().hex[:8]}"
    )
    try:
        destination.mkdir(parents=True, exist_ok=False)
        moved = 0
        for path in selected:
            path.replace(destination / path.name)
            moved += 1
        return moved
    except OSError as exc:
        global _last_error
        _last_error = f"keyframe archive incomplete: {exc}"
        return 0


def _enqueue_control(kind: str, robot_id: str | None = None) -> int:
    """Invalidate an in-flight solve and queue a worker-owned graph mutation."""
    global _generation, _last_snapshot
    with _control_lock:
        _generation += 1
        generation = _generation
        _controls.append((kind, robot_id))
    _last_snapshot = None
    return generation


def _take_controls() -> tuple[list[tuple[str, str | None]], int]:
    with _control_lock:
        commands = list(_controls)
        _controls.clear()
        return commands, _generation


def _current_generation() -> int:
    with _control_lock:
        return _generation


def _restore_capture() -> int:
    """Rebuild the in-memory graph from an explicitly selected capture.

    Keyframes are captured before entering the bounded live queue, so this is
    the durable source of truth after a planned container replacement.  Replay
    directly into the backend rather than POSTing the blobs: POST would append
    a second copy of every file to the same capture and grow it on each boot.

    Restoration is opt-in.  Capture directories are also used for offline
    experiments, and silently adopting yesterday's dataset into a fresh
    simulation would be much worse than starting empty.
    """
    global _ingested, _last_error, _restored
    if not RESTORE_CAPTURE or not CAPTURE_DIR:
        return 0
    directory = Path(CAPTURE_DIR) / "keyframes"
    try:
        paths = sorted(directory.glob("*.kf"))
    except OSError as exc:
        _last_error = f"capture restore disabled: {exc}"
        return 0

    restored = 0
    failures: list[str] = []
    for path in paths:
        try:
            accepted = backend.ingest_packet(decode_keyframe(path.read_bytes()))
        except (OSError, ProtocolError, ValueError) as exc:
            failures.append(f"{path.name}: {exc}")
            continue
        if accepted:
            restored += 1
    _restored += restored
    _ingested += restored
    if failures:
        preview = "; ".join(failures[:3])
        suffix = f" (+{len(failures) - 3} more)" if len(failures) > 3 else ""
        _last_error = f"capture restore skipped {len(failures)} file(s): {preview}{suffix}"
    return restored


SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "").rstrip("/")
OPTIMIZE_EVERY_N = int(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_EVERY", "1"))
OPTIMIZE_EVERY_S = float(os.environ.get("SWARMDECK_SLAM_OPTIMIZE_S", "1.0"))
PUBLISH_TIMEOUT_S = 15.0


def _publish_snapshot(snapshot: BackendSnapshot, generation: int | None = None) -> None:
    """Push origins + the majority-component grid to the SwarmDeck server."""
    global _last_snapshot, _last_error
    if generation is not None and generation != _current_generation():
        return
    _last_snapshot = snapshot
    if not SERVER_URL:
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
        if generation is not None and generation != _current_generation():
            return
        _publish_grid(scope, grid)

    grid = majority_component(snapshot)
    if grid is not None:
        if generation is not None and generation != _current_generation():
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

    if generation is not None and generation != _current_generation():
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
        commands, generation = _take_controls()
        for kind, robot_id in commands:
            if kind == "reset":
                backend.reset()
            elif kind == "delete_robot" and robot_id:
                backend.delete_robot(robot_id)
        blobs: list[bytes] = []
        with _queue_lock:
            while _queue:
                blobs.append(_queue.popleft())
        for blob in blobs:
            try:
                backend.ingest_packet(decode_keyframe(blob))
            except (ProtocolError, ValueError) as exc:
                _last_error = str(exc)
                continue
            _ingested += 1
        if not blobs:
            # An idle pass still falls through to the due check below, because
            # a new keyframe is no longer the only thing that can dirty the
            # graph: changing which trajectories are included does too, and
            # returning here meant that selection sat unapplied until the next
            # blob happened to arrive -- forever, on a fleet that has stopped.
            _stop.wait(0.05)
        now = time.monotonic()
        due = (
            backend.new_since_optimize >= OPTIMIZE_EVERY_N
            or (now - last_optimize) >= OPTIMIZE_EVERY_S
        )
        if backend.dirty and due:
            snapshot = backend.optimize_and_render()
            last_optimize = now
            # A reset/delete can arrive while the CPU-heavy solve is running.
            # Its generation changes immediately; never let that obsolete solve
            # republish the map the operator just discarded.
            if snapshot is not None and generation == _current_generation():
                _publish_snapshot(snapshot, generation)


@asynccontextmanager
async def lifespan(_: FastAPI):
    global _worker
    _stop.clear()
    _restore_capture()
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
        "restored": _restored,
        "dirty": backend.dirty,
        "registration_mode": backend.registration_mode,
        "anchor_robot": backend.anchor_robot_id,
        "capture_dir": CAPTURE_DIR,
        "restore_capture": RESTORE_CAPTURE,
        "generation": _current_generation(),
        "pending_controls": len(_controls),
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
        # Read off the back-end, not off the snapshot, so a trajectory that
        # arrived (or was excluded) since the last solve is already listed. The
        # component column is the only part that needs a solve, and it is None
        # until there is one.
        "trajectories": [
            t.to_dict()
            for t in backend.trajectory_summaries(
                snapshot.optimized if snapshot is not None else None
            )
        ],
        "server_url": SERVER_URL,
    }


@app.get("/config")
def get_config() -> dict[str, Any]:
    """Operator merge knobs. Safe to poll from the dashboard."""
    return {
        "ok": True,
        "settings": backend.operator_settings(),
        "defaults": operator_setting_defaults(),
    }


@app.put("/config")
async def put_config(request: Request) -> Any:
    """Apply merge knobs. The worker re-solves on its next pass."""
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    if not isinstance(payload, dict):
        return JSONResponse({"error": "JSON object required"}, status_code=400)
    settings = backend.apply_operator_settings(payload)
    return {"ok": True, "settings": settings, "defaults": operator_setting_defaults()}


@app.post("/trajectories/select")
async def post_trajectories_select(request: Request) -> Any:
    """Choose which trajectories the next optimization is built from.

    Three shapes, all naming trajectories as ``robot_id`` or
    ``robot_id@session``::

        {"only":    ["botman_0@1787715679-a1b2c3d4"]}   # exactly these
        {"exclude": ["botman_0"]}                        # drop these
        {"include": ["botman_0"]}                        # put these back

    ``exclude`` and ``include`` may be sent together and are applied in that
    order. An excluded trajectory is still stored and still listed by
    ``/status``; it contributes no keyframe and no edge to the solve, so it
    gets no pose and appears in no grid, and including it again restores every
    edge it had. That is what makes rebuilding a map from a chosen subset a
    selection rather than a destructive filter.

    Returns immediately with the new selection. The worker picks the
    re-optimization up on its next pass, because the selection change marks the
    back-end dirty -- this endpoint never runs the solver on the HTTP thread,
    for the same reason ``/keyframe`` does not.
    """
    try:
        payload = await request.json()
    except Exception:
        return JSONResponse({"error": "JSON body required"}, status_code=400)
    if not isinstance(payload, dict):
        return JSONResponse({"error": "JSON object required"}, status_code=400)

    try:
        only = _parse_trajectories(payload.get("only"))
        exclude = _parse_trajectories(payload.get("exclude"))
        include = _parse_trajectories(payload.get("include"))
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)

    if only is None and exclude is None and include is None:
        return JSONResponse(
            {"error": "one of 'only', 'exclude', 'include' is required"},
            status_code=400,
        )

    changed = 0
    if only is not None:
        before = {t for t in backend.trajectory_ids() if backend.is_included(t)}
        backend.include_only(only)
        after = {t for t in backend.trajectory_ids() if backend.is_included(t)}
        changed = len(before ^ after)
    for trajectory in exclude or []:
        changed += int(backend.set_included(trajectory, False))
    for trajectory in include or []:
        changed += int(backend.set_included(trajectory, True))

    return {
        "ok": True,
        "changed": changed,
        "trajectories": [t.to_dict() for t in backend.trajectory_summaries()],
    }


def _parse_trajectories(value: Any) -> list[TrajectoryId] | None:
    """``["robot@session", ...]`` to trajectory ids. None when the key is absent.

    An unknown trajectory is accepted rather than rejected: the operator is
    naming a selection, and refusing the whole request because one segment has
    not arrived yet would make the endpoint depend on ingest timing.
    """
    if value is None:
        return None
    if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
        raise ValueError("trajectory selection must be a list of strings")
    try:
        return [TrajectoryId.parse(v) for v in value]
    except ValueError as exc:
        raise ValueError(str(exc)) from exc


@app.post("/keyframe")
async def post_keyframe(request: Request) -> Any:
    """Accept one opaque keyframe blob. Never runs the optimizer here."""
    global _dropped
    body = await request.body()
    if not body:
        return JSONResponse({"error": "empty body"}, status_code=400)
    if len(body) > MAX_KEYFRAME_BYTES:
        return JSONResponse({"error": "keyframe too large"}, status_code=413)
    with _ingress_lock:
        _capture(body)
        with _queue_lock:
            if len(_queue) >= _queue_cap:
                _queue.popleft()
                _dropped += 1
            _queue.append(body)
    return {"ok": True, "queued": _queued(), "dropped": _dropped}


@app.post("/reset")
def post_reset() -> dict[str, Any]:
    with _ingress_lock:
        with _queue_lock:
            _queue.clear()
        archived = _archive_capture()
    generation = _enqueue_control("reset")
    global _last_snapshot, _ingested, _dropped, _last_error, _restored
    _ingested = 0
    _restored = 0
    _dropped = 0
    _last_error = ""
    return {"ok": True, "archived_keyframes": archived, "generation": generation}


@app.delete("/robots/{robot_id}/keyframes")
def delete_robot_keyframes(robot_id: str) -> Any:
    """Delete one robot's live and restorable pose-graph contribution."""
    robot_id = robot_id.strip()
    if not robot_id:
        return JSONResponse({"error": "robot_id required"}, status_code=400)

    with _ingress_lock:
        with _queue_lock:
            kept: deque[bytes] = deque()
            dropped_queued = 0
            while _queue:
                blob = _queue.popleft()
                try:
                    belongs = (
                        str(peek_keyframe_header(blob).get("robot_id", ""))
                        == robot_id
                    )
                except ProtocolError:
                    belongs = False
                if belongs:
                    dropped_queued += 1
                else:
                    kept.append(blob)
            _queue.extend(kept)
        archived = _archive_capture(robot_id)
    generation = _enqueue_control("delete_robot", robot_id)
    return {
        "ok": True,
        "robot_id": robot_id,
        "archived_keyframes": archived,
        "dropped_queued": dropped_queued,
        "generation": generation,
    }


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
