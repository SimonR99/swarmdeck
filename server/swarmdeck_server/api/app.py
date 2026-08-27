"""FastAPI app: GUI websocket, adapter websocket, map endpoints.

The backend has no ROS import anywhere — acceptance criterion 12.
"""

from __future__ import annotations

import asyncio
import json
import math
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import yaml
from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from ..bus import bus, mark_session_start, session_elapsed, stamps
from ..config.detection import DETECTION_CLASSES, floor_for
from ..config.settings import SettingsStore, disabled_robot_ids, is_robot_enabled
from ..detect.review import ReviewStore
from ..events.logger import events
from ..fleet.registry import registry
from ..mapsvc.service import GridMeta, MapService, map_service

# Compatibility exports for callers that historically imported map transport
# constants/helpers from ``api.app``. The route implementation now lives in
# ``map_routes``; keeping these names here avoids a needless API break while
# the FastAPI handlers remain thin composition-root wrappers.
from .map_routes import (
    CLOUD_SCALE,
    MAX_UPLOAD_BYTES,
    _inflate,
    costmap_snapshots,
    reset_costmaps,
    take_costmap_patches,
)

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
    apply_review_radii(settings_store.value)
    load_review()
    map_service.set_excluded(disabled_robot_ids(settings_store.value))
    tasks = [
        asyncio.create_task(state_loop()),
        asyncio.create_task(map_loop()),
        asyncio.create_task(network_loop()),
        asyncio.create_task(costmap_loop()),
        asyncio.create_task(session_loop()),
        # Registration runs here rather than inside each upload — see
        # MapService.registration_worker. Without this task the transforms in
        # `auto` mode would never be recomputed at all.
        asyncio.create_task(map_service.registration_worker()),
    ]
    from ..mapsvc import graph_bridge

    graph_bridge.configure()
    await graph_bridge.start_worker()
    yield
    await graph_bridge.stop_worker()
    for t in tasks:
        t.cancel()


app = FastAPI(title="SwarmDeck", lifespan=lifespan)

REPO = Path(__file__).resolve().parents[3]
settings_store = SettingsStore(REPO / "sessions" / "settings.json")
CONFIG: dict[str, Any] = {}
SESSION: dict[str, Any] = {
    "running": False,
    "name": None,
    "started_at": None,
    "recording": False,
}

_gui_clients: set[WebSocket] = set()
_alerts: dict[str, dict[str, Any]] = {}
# alert_id → wall-clock time until which raise_alert is a no-op (after acknowledge)
_alert_suppress_until: dict[str, float] = {}
_camera_frames: dict[str, tuple[bytes, float, int]] = {}
_detections: dict[str, dict[str, Any]] = {}
# Live camera tracks above; operator-validated map objects below. They are
# deliberately separate stores: a track is "what a camera can see right now",
# an entity is "what the fleet agreed is there". See detect/review.py.
review_store = ReviewStore()
# Validated objects outlive the process. They are the operator's decisions, not
# derived data: a restart or a crash used to lose every accepted object and
# every ignore zone with no trace, which is the one kind of state a dashboard
# must not quietly forget.
REVIEW_PATH = REPO / "sessions" / "detections.json"
_review_pushed_at = 0.0
_review_dirty = False
_review_saved_at = 0.0
_camera_seq = 0

# A robot still reporting telemetry can stop delivering frames entirely — a
# congested link starves the camera POST long before it starves the 5 Hz
# websocket. `GET /api/camera` answers 200 with the last frame regardless of
# age, so nothing else in the system distinguishes that from live video.
# Measured on a healthy link: p95 frame age 0.63 s. Well clear of it.
CAMERA_STALE_S = 3.0

# gui socket -> the robot whose camera that dashboard is currently showing.
# Keyed by socket rather than a single global, because two operators watching
# two different robots both need their frames.
_camera_watchers: dict[Any, str] = {}

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
    if path:
        p = Path(path)
    elif (REPO / "configs" / "4robot.yaml").exists():
        p = REPO / "configs" / "4robot.yaml"
    else:
        p = REPO / "study" / "4robot.yaml"
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
        new_service.set_transform(
            rid, pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0)
        )

    map_service.__dict__.update(new_service.__dict__)
    reset_costmaps()
    _camera_frames.clear()
    _detections.clear()
    # Deliberately does NOT persist. This runs at startup, before load_review(),
    # so writing here would truncate the saved objects on every boot and then
    # read the file it had just emptied. Config selection clears the in-memory
    # store; only an operator action or an explicit reset writes to disk.
    review_store.reset()
    return CONFIG


# ----------------------------------------------------------------- broadcast


async def broadcast(msg: dict[str, Any]) -> None:
    dead = []
    # Snapshot, never the live set. The send below yields, and a dashboard
    # connecting or closing during that yield mutates `_gui_clients` while it is
    # being iterated -- which raises RuntimeError out of broadcast() and into
    # whichever caller was unlucky. Observed 2026-08-13 as a fleet-wide outage:
    # the `hello` handler broadcasts `fleet_change`, so the exception surfaced
    # as "dropped a malformed hello" and robots could not register AT ALL, on a
    # network that was working. A GUI reload was enough to trigger it.
    for ws in list(_gui_clients):
        try:
            await ws.send_json(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _gui_clients.discard(ws)


def detection_class_enabled(
    class_name: Any, settings: dict[str, Any] | None = None
) -> bool:
    """Whether a detection class belongs in the current operator view.

    Adapters poll settings, so one can still submit a batch made with the old
    class selection for a few seconds after a save.  Enforcing the selection at
    the backend closes that window and keeps a removed map entity from being
    recreated immediately after it was cleared.

    An empty class list means the detector catalog was unavailable when the
    settings were validated.  In that compatibility mode the adapter owns the
    class list, so enabled detection continues to accept its classes.
    """
    value = settings if settings is not None else settings_store.value
    if not value.get("detection_enabled", True):
        return False
    selected = value.get("detection_classes")
    return not selected or class_name in selected


def discard_disabled_detections(settings: dict[str, Any]) -> list[str]:
    """Delete cached entities whose classes were removed by a settings save."""
    stale = [
        detection_id
        for detection_id, detection in _detections.items()
        if not detection_class_enabled(detection.get("class"), settings)
    ]
    for detection_id in stale:
        _detections.pop(detection_id, None)
    return stale


def camera_is_watched(robot_id: str) -> bool:
    return robot_id in set(_camera_watchers.values())


async def push_camera_interest(robot_ids: Any) -> None:
    """Tell each robot whether any dashboard is currently showing its camera.

    Camera frames are by far the largest thing a robot sends -- measured at
    73-78 KB per frame against 0.4 KB of telemetry -- and until now every robot
    uploaded them continuously while at most one was ever displayed. On a
    contended link that crowded out the traffic that actually matters.

    Best-effort by design: `registry.send` returns False for a robot that is not
    connected, and adapters default to uploading. Losing this message costs
    bandwidth, never video.
    """
    for robot_id in robot_ids:
        if not robot_id:
            continue
        await registry.send(
            robot_id,
            {
                "type": "camera_interest",
                "watched": camera_is_watched(robot_id),
                **stamps(),
            },
        )


async def set_camera_watch(source: Any, robot_id: str) -> None:
    """Record which robot a dashboard is showing and notify both robots.

    Both: the one gaining a viewer has to start uploading at full rate, and the
    one losing its last viewer has to stop.
    """
    if source is None:
        return
    previous = _camera_watchers.get(source)
    if previous == robot_id:
        return
    if robot_id:
        _camera_watchers[source] = robot_id
    else:
        _camera_watchers.pop(source, None)
    await push_camera_interest({previous, robot_id})


def frozen_camera_message(robot: Any) -> str | None:
    """Alert text for a frozen camera on an otherwise healthy robot, else None.

    Only while the robot is online: an offline robot has a frozen camera by
    definition, and `adapter_disconnect` already reports that — two alerts for
    one cause is how an alert stack gets ignored. A robot with no camera at all
    never qualifies either, because it has no frame that could have gone stale.
    """
    frame = _camera_frames.get(robot.robot_id)
    if frame is None or not robot.online:
        return None
    frozen_s = time.monotonic() - frame[1]
    if frozen_s <= CAMERA_STALE_S:
        return None
    return f"{robot.robot_id} camera frozen for {int(frozen_s)} s"


def review_state() -> dict[str, Any]:
    return {"type": "detection_review", **review_store.snapshot()}


def load_review() -> None:
    try:
        review_store.load_dict(json.loads(REVIEW_PATH.read_text()))
    except (FileNotFoundError, json.JSONDecodeError, OSError) as exc:
        if not isinstance(exc, FileNotFoundError):
            print(f"[review] ignoring unreadable {REVIEW_PATH.name}: {exc}")
        return
    snap = review_store.snapshot()
    print(
        f"[review] restored {len(snap['entities'])} confirmed object(s), "
        f"{len(snap['proposals'])} pending, {snap['ignored']} ignored zone(s)"
    )


def save_review(force: bool = False) -> None:
    """Write validated objects out, atomically and not on the hot path.

    Folding a sighting nudges a centroid at frame rate, so the flush is
    coalesced: operator decisions force it, ordinary drift waits for the tick.
    """
    global _review_dirty, _review_saved_at
    if not force and not _review_dirty:
        return
    _review_dirty = False
    _review_saved_at = time.monotonic()
    try:
        REVIEW_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary = REVIEW_PATH.with_suffix(".tmp")
        temporary.write_text(json.dumps(review_store.to_dict(), indent=2) + "\n")
        temporary.replace(REVIEW_PATH)
    except OSError as exc:
        print(f"[review] could not persist to {REVIEW_PATH.name}: {exc}")


async def broadcast_review(urgency: str = "now") -> None:
    """Push review state, rate-limiting the merely-incremental updates."""
    global _review_pushed_at
    now = time.monotonic()
    if urgency != "now" and now - _review_pushed_at < 1.0:
        return
    _review_pushed_at = now
    await broadcast(review_state())


def apply_review_radii(settings: dict[str, Any]) -> None:
    review_store.same_radius = float(settings.get("detection_same_radius_m", 0.5))
    review_store.ask_radius = float(settings.get("detection_ask_radius_m", 1.5))
    review_store.cross_class_merge = bool(
        settings.get("detection_single_mode", False)
        or settings.get("detection_cross_class_merge", False)
    )


def detection_hidden(detection: dict[str, Any], settings: dict[str, Any]) -> bool:
    """Whether this stored entity currently sits below its operator floor.

    Judged on `best_score`, not the newest one.  A model's confidence in a
    stationary object wanders by a few points frame to frame, so filtering on
    the live score makes a marker sitting near its floor blink on and off; the
    best evidence we ever had for the object does not oscillate.
    """
    return float(detection.get("best_score", detection.get("score", 0.0))) < floor_for(
        settings, detection.get("robot_id", ""), detection.get("class", "")
    )


def reapply_detection_floors(settings: dict[str, Any]) -> list[dict[str, Any]]:
    """Re-judge every stored entity against the floors that were just saved.

    This is what makes an operator's threshold retroactive and immediate.  The
    floor is a question about stored evidence -- "is this a real detection?" --
    and stored evidence is right here, so answering it needs no robot, no round
    trip, and no wait for the next frame.  Raising a floor buries markers that
    are already on the map; lowering it back exhumes them, because the entity
    was hidden rather than deleted.

    Returns the entities whose visibility actually flipped, for broadcast.
    """
    changed: list[dict[str, Any]] = []
    for detection_id, detection in _detections.items():
        hidden = detection_hidden(detection, settings)
        if hidden == bool(detection.get("hidden", False)):
            continue
        updated = {**detection, "hidden": hidden}
        _detections[detection_id] = updated
        changed.append(updated)
    return changed


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
                    aid,
                    "warn",
                    "unattended",
                    f"{r.robot_id} unattended for {int(r.unattended_s)} s",
                    r.robot_id,
                )
            elif r.unattended_s <= threshold:
                await clear_alert(aid)

            did = f"disconnect_{r.robot_id}"
            if not r.online:
                await raise_alert(
                    did,
                    "critical",
                    "adapter_disconnect",
                    f"{r.robot_id} adapter disconnected",
                    r.robot_id,
                )
            else:
                await clear_alert(did)

            sid = f"stream_{r.robot_id}"
            frozen = frozen_camera_message(r)
            if frozen is not None:
                await raise_alert(sid, "warn", "stream_loss", frozen, r.robot_id)
            else:
                await clear_alert(sid)


async def map_loop() -> None:
    """2 Hz patch emission — never re-sends the whole grid (NFR-6)."""
    while True:
        await asyncio.sleep(0.5)
        patch = map_service.take_patch()
        if patch:
            await broadcast(patch)


async def network_loop() -> None:
    """1 Hz per-robot Wi-Fi heatmap patches."""
    while True:
        await asyncio.sleep(1.0)
        for robot_id in map_service.network_robot_ids():
            patch = map_service.take_network_patch(robot_id)
            if patch:
                await broadcast(patch)


async def costmap_loop() -> None:
    """Fan out the latest global/local Nav2 costmap snapshots."""
    while True:
        await asyncio.sleep(0.5)
        for patch in take_costmap_patches():
            await broadcast(patch)


async def session_loop() -> None:
    ticks = 0
    while True:
        await asyncio.sleep(1.0)
        await broadcast(session_state())
        # Centroid drift is worth persisting, but not at frame rate. Operator
        # decisions are written immediately and do not wait for this.
        ticks += 1
        if ticks % 5 == 0:
            await asyncio.to_thread(save_review)


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
    state["pose"] = map_service.robot_to_world(robot.robot_id, state["pose"])
    if state["goal"]:
        state["goal"] = map_service.robot_to_world(robot.robot_id, state["goal"])
    for path_name in ("planned_path", "global_planned_path", "local_planned_path"):
        state[path_name] = [
            map_service.robot_to_world(robot.robot_id, point)
            for point in state.get(path_name, [])
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
        targets = {
            rid for rid, r in registry.robots.items() if "reset" in r.capabilities
        }
        skipped = sorted(set(registry.robots) - targets)

        events.log("reset_start", {"robots": sorted(targets), "skipped": skipped})
        await broadcast(
            {
                "type": "sim_reset",
                "phase": "start",
                "robots": sorted(targets),
                "skipped": skipped,
            }
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

        await map_service.reset_async()
        from ..mapsvc import graph_bridge

        await asyncio.to_thread(graph_bridge.post_reset)
        reset_costmaps()
        await broadcast({"type": "costmap_clear", "robot_id": None})
        await broadcast({"type": "network_clear", "robot_id": None})
        _detections.clear()
        # Validated objects describe the world before the reset. Keeping them
        # would leave confirmed markers floating over a map that no longer has
        # the geometry they were placed against.
        review_store.reset()
        save_review(force=True)
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
            robot.global_planned_path = []
            robot.local_planned_path = []
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
                "reset_incomplete",
                "warn",
                "fault",
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
    from .control_routes import get_config as handler

    return await handler()


@app.get("/api/settings")
async def get_settings() -> dict[str, Any]:
    from .control_routes import get_settings as handler

    return await handler()


@app.get("/api/detection/classes")
async def get_detection_classes() -> dict[str, Any]:
    from .control_routes import get_detection_classes as handler

    return await handler()


@app.put("/api/settings")
async def put_settings(request: Request) -> dict[str, Any]:
    from .control_routes import put_settings as handler

    return await handler(request)


@app.get("/api/fleet")
async def get_fleet() -> dict[str, Any]:
    from .control_routes import get_fleet as handler

    return await handler()


@app.get("/api/session")
async def get_session() -> dict[str, Any]:
    from .control_routes import get_session as handler

    return await handler()


@app.post("/api/session/start")
async def start_session() -> dict[str, Any]:
    from .control_routes import start_session as handler

    return await handler()


@app.post("/api/session/stop")
async def stop_session() -> dict[str, Any]:
    from .control_routes import stop_session as handler

    return await handler()


@app.post("/api/sim/reset")
async def post_sim_reset() -> dict[str, Any]:
    from .control_routes import post_sim_reset as handler

    return await handler()


@app.get("/api/map")
async def get_map() -> Response:
    from .map_routes import get_map as handler

    return await handler()


@app.get("/api/map/status")
async def get_map_status() -> dict[str, Any]:
    from .map_routes import get_map_status as handler

    return await handler()


@app.get("/api/map/info")
async def get_map_info() -> dict[str, Any]:
    from .map_routes import get_map_info as handler

    return await handler()


def _robots_blocking_map_reset(robot_id: str | None = None) -> list[str]:
    from .map_routes import _robots_blocking_map_reset as handler

    return handler(robot_id)


async def _publish_map_reset(scope: str, robot_id: str | None = None) -> Response:
    from .map_routes import _publish_map_reset as handler

    return await handler(scope, robot_id)


@app.post("/api/map/reset/{robot_id}")
async def reset_robot_map(robot_id: str) -> Response:
    from .map_routes import reset_robot_map as handler

    return await handler(robot_id)


@app.post("/api/map/reset")
async def reset_all_maps() -> Response:
    from .map_routes import reset_all_maps as handler

    return await handler()


@app.get("/api/map/local/{robot_id}")
async def get_local_map(robot_id: str) -> Response:
    from .map_routes import get_local_map as handler

    return await handler(robot_id)


@app.get("/api/map/local/{robot_id}/info")
async def get_local_map_info(robot_id: str) -> Response:
    from .map_routes import get_local_map_info as handler

    return await handler(robot_id)


@app.get("/api/map/local/{robot_id}/network")
async def get_local_network(robot_id: str) -> Response:
    from .map_routes import get_local_network as handler

    return await handler(robot_id)


@app.get("/api/map/costmap/{robot_id}/{kind}")
async def get_costmap(robot_id: str, kind: str) -> Response:
    from .map_routes import get_costmap as handler

    return await handler(robot_id, kind)


@app.post("/api/adapter/map")
async def post_map(request: Request) -> Any:
    from .map_routes import post_map as handler

    return await handler(request)


@app.post("/api/adapter/costmap")
async def post_costmap(request: Request) -> Any:
    from .map_routes import post_costmap as handler

    return await handler(request)


@app.post("/api/adapter/global_map")
async def post_global_map(request: Request) -> Any:
    from .map_routes import post_global_map as handler

    return await handler(request)


@app.post("/api/adapter/cloud")
async def post_cloud(request: Request) -> Any:
    from .map_routes import post_cloud as handler

    return await handler(request)


@app.post("/api/adapter/scan")
async def post_scan(request: Request) -> Any:
    from .map_routes import post_scan as handler

    return await handler(request)


@app.post("/api/adapter/keyframe")
async def post_keyframe(request: Request) -> Any:
    from .map_routes import post_keyframe as handler

    return await handler(request)


@app.post("/api/slam/optimized_map")
async def post_optimized_map(request: Request) -> Any:
    from .map_routes import post_optimized_map as handler

    return await handler(request)


@app.get("/api/map/optimized")
async def get_optimized_index() -> dict[str, Any]:
    from .map_routes import get_optimized_index as handler

    return await handler()


@app.get("/api/map/optimized/{scope}")
async def get_optimized_map(scope: str) -> Response:
    from .map_routes import get_optimized_map as handler

    return await handler(scope)


@app.post("/api/slam/update")
async def post_slam_update(request: Request) -> Any:
    from .map_routes import post_slam_update as handler

    return await handler(request)


@app.get("/api/map/nav/{robot_id}")
async def get_nav_map(request: Request, robot_id: str) -> Response:
    from .map_routes import get_nav_map as handler

    return await handler(request, robot_id)


@app.get("/api/map/cloud")
async def get_cloud(request: Request) -> Response:
    from .map_routes import get_cloud as handler

    return await handler(request)


@app.get("/api/map/local/{robot_id}/cloud")
async def get_local_cloud(robot_id: str) -> Response:
    from .map_routes import get_cloud as handler

    return await handler(
        Request(scope={"type": "http", "query_string": f"robot_id={robot_id}".encode()})
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
        await ws.send_json({"type": "map_info", "info": map_service.map_info()})
        await ws.send_json({"type": "fleet_change", "robots": fleet_snapshot()})
        await ws.send_json(session_state())
        await ws.send_json({"type": "settings_state", "settings": settings_store.value})
        await ws.send_json(review_state())
        for costmap in costmap_snapshots():
            await ws.send_json(costmap)
        for a in _alerts.values():
            await ws.send_json({"type": "alert", "alert": a})

        while True:
            raw = await ws.receive_text()
            # Per message, so one malformed frame costs that frame and not the
            # connection. Only a transport failure should end the loop.
            try:
                await handle_gui_message(json.loads(raw), source=ws)
            except (WebSocketDisconnect, asyncio.CancelledError):
                raise
            except Exception as exc:
                print(f"[gui] dropped a malformed message: {exc}")
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"[gui] socket error: {exc}")
    finally:
        _gui_clients.discard(ws)
        # A closed dashboard is not watching anything. Without this the last
        # robot it looked at would upload full-rate video forever. Only that
        # robot can have changed, and it may still be watched by someone else --
        # push_camera_interest re-reads the remaining watchers to decide.
        departed = _camera_watchers.pop(ws, None)
        if departed:
            await push_camera_interest({departed})


async def handle_gui_message(msg: dict[str, Any], source: Any = None) -> None:
    kind = msg.get("type", "")
    rid = msg.get("robot_id", "")

    # Every operator action is logged before it takes effect.
    events.log(kind, {k: v for k, v in msg.items() if k != "type"})
    if rid:
        registry.attend(rid)

    if (
        rid
        and kind in {"set_goal", "drive", "body_command"}
        and not is_robot_enabled(settings_store.value, rid)
    ):
        return

    if kind in {
        "detection_accept",
        "detection_ignore",
        "detection_merge",
        "detection_forget",
        "detection_forget_all",
        "detection_clear_proposals",
        "detection_delete_all",
        "detection_unignore",
    }:
        pid = str(msg.get("proposal_id", ""))
        eid = str(msg.get("entity_id", ""))
        if kind == "detection_accept":
            changed = review_store.accept(pid) is not None
        elif kind == "detection_ignore":
            changed = review_store.ignore(pid)
        elif kind == "detection_merge":
            changed = review_store.merge(pid, eid) is not None
        elif kind == "detection_unignore":
            changed = review_store.clear_ignored() > 0
        elif kind == "detection_forget_all":
            include_props = bool(msg.get("include_proposals", False))
            changed = review_store.forget_all(include_proposals=include_props) > 0
        elif kind == "detection_clear_proposals":
            changed = review_store.clear_proposals() > 0
        elif kind == "detection_delete_all":
            changed = review_store.delete_all() > 0
        else:
            if pid and not eid:
                changed = review_store.forget_proposal(pid)
            elif eid:
                changed = review_store.forget(eid) or review_store.forget_proposal(eid)
            else:
                changed = False
        # Silence on an unknown id is deliberate: two operators can answer the
        # same proposal, and the loser of that race must not get an error for
        # having agreed.
        if changed:
            # Persist before broadcasting: an operator who sees the dashboard
            # confirm a deletion and then loses the process must not find the
            # object back on the map.
            save_review(force=True)
            await broadcast_review()
        return

    if kind == "set_goal":
        goal = msg.get("payload") or {}
        if not registry.can(rid, "navigate"):
            return
        taken_by = goal_taken(goal, exclude=rid)
        if taken_by:
            await raise_alert(
                f"dupgoal_{rid}",
                "warn",
                "fault",
                f"Goal already assigned to {taken_by}",
                rid,
            )
            return
        robot = registry.robots[rid]
        local_goal = (
            goal
            if robot.coordinate_frame == "merged"
            else map_service.world_to_robot(rid, goal)
        )
        # Pre-compute collision-free global A* path for robots (like Scout)
        # that lack an onboard global grid planner.
        start_pose = (
            robot.pose
            if robot.coordinate_frame == "merged"
            else map_service.robot_to_world(rid, robot.pose)
        )
        world_goal = (
            goal
            if robot.coordinate_frame == "merged"
            else map_service.robot_to_world(rid, local_goal)
        )
        planned_world = map_service.plan_path(rid, start_pose, world_goal)
        local_planned = (
            (
                planned_world
                if robot.coordinate_frame == "merged"
                else [map_service.world_to_robot(rid, pt) for pt in planned_world]
            )
            if planned_world
            else []
        )

        sent = await registry.send(
            rid,
            {
                "type": "navigate_to",
                "goal": local_goal,
                "path": local_planned,
                **stamps(),
            },
        )
        if sent:
            # Reserve immediately. Waiting for the adapter's next 5 Hz state
            # packet leaves a race where back-to-back UI messages can assign
            # the same destination to multiple robots.
            robot.goal = local_goal
            robot.nav_status = "active"
            robot.mode = "nav"
            if local_planned:
                robot.global_planned_path = local_planned
                robot.planned_path = local_planned

    elif kind == "cancel_goal":
        sent = await registry.send(rid, {"type": "cancel_goal", **stamps()})
        if sent and rid in registry.robots:
            registry.robots[rid].goal = None
            registry.robots[rid].global_planned_path = []
            registry.robots[rid].local_planned_path = []
            registry.robots[rid].planned_path = []

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

    elif kind == "body_command":
        action = str(msg.get("action") or "")
        if action not in ("claim", "release", "sit", "stand"):
            return
        if not registry.can(rid, "body"):
            return
        await registry.send(rid, {"type": "body_command", "action": action, **stamps()})

    elif kind == "stop_all":
        # Whether each robot was actually REACHED, not merely addressed. A stop
        # that went nowhere is the one command an operator must never be allowed
        # to believe succeeded, so the robots it failed to reach are named.
        undelivered = [
            robot_id
            for robot_id in list(registry.robots)
            if not await registry.send(robot_id, {"type": "stop", **stamps()})
        ]
        await raise_alert(
            "stop_all", "critical", "fault", "STOP ALL issued by operator"
        )
        if undelivered:
            events.log("stop_all_undelivered", {"robots": sorted(undelivered)})
            await raise_alert(
                "stop_all_undelivered",
                "critical",
                "fault",
                f"STOP did not reach {', '.join(sorted(undelivered))} — "
                f"they may still be moving",
            )

    elif kind in ("start_explore", "stop_explore"):
        # Fleet-wide, but sent per robot because that is the only channel the
        # protocol has. Only robots that advertise `explore` are addressed:
        # exploration starts a process that drives the whole fleet reactively,
        # and a hardware adapter must never be asked to do that. `registry.can`
        # is the same gate `navigate` and `reset` use.
        enabled = kind == "start_explore"
        targets = [
            robot_id
            for robot_id in list(registry.robots)
            if registry.can(robot_id, "explore")
        ]
        if not targets:
            await raise_alert(
                "explore_unsupported",
                "warn",
                "fault",
                "No connected robot supports exploration",
            )
        else:
            undelivered = [
                robot_id
                for robot_id in targets
                if not await registry.send(
                    robot_id, {"type": "explore", "enabled": enabled, **stamps()}
                )
            ]
            events.log(
                kind, {"robots": sorted(targets), "undelivered": sorted(undelivered)}
            )
            # Stop is the direction worth alerting on. A start that did not
            # arrive shows up immediately as robots that do not move; a stop
            # that did not arrive leaves them driving, which is the same class
            # of problem stop_all names its failures for.
            if undelivered and not enabled:
                await raise_alert(
                    "stop_explore_undelivered",
                    "warn",
                    "fault",
                    f"Stop exploration did not reach {', '.join(sorted(undelivered))}",
                )

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

    elif kind == "switch_camera":
        await set_camera_watch(source, rid)

    elif kind in ("select_robots", "report_target"):
        pass  # logged above; no robot-side effect


# ----------------------------------------------------------------- adapter socket


@app.websocket("/adapter")
async def adapter_socket(ws: WebSocket) -> None:
    await ws.accept()
    robot_id: str | None = None
    try:
        while True:
            raw = await ws.receive_text()
            # Rule 3 makes an unknown message type non-fatal. A MALFORMED one
            # deserves the same: a truncated frame, a `detections` batch with no
            # `robot_id`, a `hello` whose `footprint_radius` will not parse —
            # each of those used to raise out of this loop and disconnect the
            # robot, which on hardware means losing telemetry and control over a
            # single bad packet.
            try:
                msg = json.loads(raw)
            except ValueError as exc:
                print(f"[adapter] dropped an unparseable message: {exc}")
                continue
            try:
                closed = await handle_adapter_message(msg, ws)
            except (WebSocketDisconnect, asyncio.CancelledError):
                raise
            except Exception as exc:
                print(f"[adapter] dropped a malformed {msg.get('type', '?')}: {exc}")
                continue
            if closed:
                return
            if robot_id is None and msg.get("type") == "hello":
                robot_id = registry_id_of(msg)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"[adapter] socket error: {exc}")
    finally:
        if robot_id:
            # Pass the socket: a robot that reconnected while this one was dying
            # already owns the entry, and unbinding it here would leave the robot
            # visibly online with no way to reach it. See Registry.disconnect.
            registry.disconnect(robot_id, ws)
            events.log("adapter_disconnect", {"robot_id": robot_id})
            # Drop the stale preview, but only if nothing took this robot's
            # place: `GET /api/camera/<id>` otherwise serves a departed robot's
            # last frame indefinitely, with only X-Frame-Age-Ms to say so.
            if not registry.has_sink(robot_id):
                _camera_frames.pop(robot_id, None)


def registry_id_of(hello: dict[str, Any]) -> str | None:
    """The robot_id a `hello` claimed, once the registry has accepted it."""
    rid = hello.get("robot_id")
    return rid if isinstance(rid, str) and rid in registry.robots else None


async def handle_adapter_message(msg: dict[str, Any], ws: WebSocket) -> bool:
    global _review_dirty
    """Dispatch one adapter message. Returns True if the socket was closed."""
    kind = msg.get("type", "")

    if kind == "hello":
        # 2 adds the optional `slam_graph` message and nothing else, so a
        # protocol-1 adapter stays valid forever. Rejecting it would
        # break exactly the mixed-fleet property the contract exists for.
        if msg.get("protocol") not in SUPPORTED_PROTOCOLS:
            await ws.close(code=4400)
            return True
        client = getattr(ws, "client", None)
        r = registry.hello(msg, ws, peer=client.host if client else "")
        robot_id = r.robot_id
        if r.coordinate_frame == "merged":
            map_service.set_transform(robot_id, 0.0, 0.0, 0.0)
        await ws.send_json({"type": "hello_ack", "robot_id": robot_id})
        # Adapters come up assuming they are watched, so a reconnect during a
        # session would otherwise resume full-rate video for a robot nobody has
        # on screen, and stay that way until the operator happened to switch.
        await push_camera_interest({robot_id})
        await broadcast({"type": "fleet_change", "robots": fleet_snapshot()})
        events.log("adapter_connect", {"robot_id": robot_id, "adapter": r.adapter})

    elif kind == "robot_state":
        robot = registry.update_state(msg)
        network = msg.get("network")
        pose = msg.get("pose")
        if robot is not None and isinstance(network, dict) and isinstance(pose, dict):
            try:
                map_service.ingest_network_sample(
                    robot.robot_id,
                    float(pose["x"]),
                    float(pose["y"]),
                    float(network["quality_pct"]),
                )
            except (KeyError, TypeError, ValueError):
                # Optional telemetry must never cost the robot its control link.
                pass

    elif kind == "detections":
        rid = msg["robot_id"]
        camera = msg.get("camera", "front")
        visible: set[str] = set()
        review_push: str | None = None
        for item in msg.get("items", []):
            # Adapters refresh settings every few seconds.  Ignore proposals
            # from an in-flight batch built with the previous class selection,
            # so a just-deleted map marker cannot flash back into existence.
            if not detection_class_enabled(item.get("class", "object")):
                continue
            detection_id = f"{rid}:{item.get('id', item.get('class', 'object'))}"
            visible.add(detection_id)
            previous = _detections.get(detection_id)
            now = time.time()
            score = float(item.get("score", 0.0) or 0.0)
            det = {
                "id": detection_id,
                "class": item.get("class", "object"),
                "score": score,
                # The strongest evidence this entity has ever produced, which is
                # what the operator floor is judged against.  See
                # detection_hidden().
                "best_score": (
                    max(score, float(previous["best_score"])) if previous else score
                ),
                "robot_id": rid,
                "camera": camera,
                "bbox": item.get("bbox"),
                "polygon": item.get("polygon"),
                "image": item.get("image"),
                "map_position": detection_position(rid, item.get("map_position")),
                "first_seen": previous["first_seen"] if previous else now,
                "last_seen": now,
                "observations": (previous["observations"] + 1) if previous else 1,
            }
            det["hidden"] = detection_hidden(det, settings_store.value)
            _detections[detection_id] = det

            # Route the located sighting to the operator's review queue. Only
            # evidence that clears the display floor is worth asking about: a
            # detection the operator has already decided is noise should not
            # come back as a question.
            if det["map_position"] and not det["hidden"]:
                # Where the reporting robot is standing, so a sighting from a
                # pose we have already averaged does not get averaged again.
                # Without it a parked robot's depth bias becomes the object's
                # position. See MIN_VIEWPOINT_MOVE_M in detect/review.py.
                observer = registry.robots.get(rid)
                outcome, _target = review_store.observe(
                    rid,
                    det["class"],
                    det["map_position"]["x"],
                    det["map_position"]["y"],
                    score,
                    observer=(
                        (observer.pose["x"], observer.pose["y"])
                        if observer is not None
                        else None
                    ),
                    image=det.get("image"),
                )
                # A new question goes out at once — the operator is waiting on
                # it. Folds and updates only shift a centroid, arrive at frame
                # rate, and are coalesced onto a 1 Hz tick so the queue stays
                # live without putting a broadcast on the hot path.
                if outcome == "proposed":
                    review_push = "now"
                elif outcome in ("folded", "updated") and review_push is None:
                    review_push = "soon"
                if outcome in ("proposed", "folded", "updated"):
                    _review_dirty = True
            # Robots capture below the operator's floor on purpose, so that
            # lowering it later can be answered from this cache instead of from
            # frames that no longer exist.  Until then the entity is stored and
            # not sent: an operator who raised a floor should see no trace of it.
            if not det["hidden"]:
                await broadcast({"type": "detection", "detection": det})

        if review_push:
            await broadcast_review(review_push)

        # An object that has left the frame is reported by its ABSENCE
        # from the batch; the protocol carries no per-object "lost"
        # message. So the batch itself has to retract what it no longer
        # contains, or the operator keeps a stale rectangle painted over
        # live video for as long as the adapter stays connected.
        #
        # Only the box is retracted. `map_position` is somewhere we went
        # and found something, not something we can currently see, so it
        # outlives the sighting and stays on the map.
        retracted = [
            key
            for key, det in _detections.items()
            if det["robot_id"] == rid
            and det["camera"] == camera
            and key not in visible
            and det["bbox"] is not None
        ]
        for key in retracted:
            # Re-read: the broadcast below yields, so another adapter's
            # batch can land between building this list and using it.
            current = _detections.get(key)
            if current is None or current["bbox"] is None:
                continue
            det = {**current, "bbox": None, "polygon": None}
            _detections[key] = det
            # A hidden entity was never sent, so there is no box out there to
            # retract; the cache still has to drop it or un-hiding this entity
            # later would restore a rectangle from an old frame.
            if not det.get("hidden", False):
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
    return False
