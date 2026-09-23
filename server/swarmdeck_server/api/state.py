"""Shared server state and operations, independent of application registration."""

from __future__ import annotations

import asyncio
import itertools
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import yaml
from fastapi import WebSocket

from ..bus import bus, mark_session_start, session_elapsed, stamps
from ..config.detection import DETECTION_CLASSES, floor_for
from ..config.settings import (
    SettingsStore,
    default_color,
    disabled_robot_ids,
    is_robot_enabled,
)
from ..detect.review import ReviewStore
from ..events.logger import events
from ..fleet.departure import departure_order, release_in_turn
from ..fleet.registry import registry
from ..mapsvc.service import map_service
from .broadcast import JsonBroadcaster
from .navigation_alerts import explain_navigation_failure

# Protocol 2 optionally carries peer SLAM status; protocol 1 remains accepted.
PROTOCOL_VERSION = 2
SUPPORTED_PROTOCOLS = (1, 2)


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
_gui_broadcaster = JsonBroadcaster(_gui_clients)
_alerts: dict[str, dict[str, Any]] = {}
# The one fleet-wide Explore still releasing its robots in turn, if any.
_departure_task: asyncio.Task | None = None
# alert_id → wall-clock time until which raise_alert is a no-op (after acknowledge)
_alert_suppress_until: dict[str, float] = {}
# robot_id → its current navigation-failure alert. One id per failure, so an
# acknowledged failure's suppression cannot silence the next one.
_nav_failure_alerts: dict[str, str] = {}
_nav_failure_serial = itertools.count(1)
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

# How long to wait for adapters to report `reset_done` before clearing server
# state. The adapters reset simulator poses, odometry, and navigation state.
# Generous, because waiting too little can clear state while an adapter still
# holds old navigation products.
RESET_TIMEOUT_S = 25.0

# Robots that have been sent `reset` and have not yet answered. Mutated from the
# adapter socket, awaited by reset_fleet(); _reset_done fires when it empties.
_reset_pending: set[str] = set()

STATE_LOOP_INTERVAL_S = 0.2
STATE_KEEPALIVE_S = 1.0
# Compare floats at 1e-6 absolute resolution to ignore recomputation noise;
# outgoing messages retain their original precision.
STATE_SIGNATURE_FLOAT_DIGITS = 6
_state_loop_cache: dict[str, tuple[str, float]] = {}
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
    new_service = type(map_service)()
    for rid, pose in (mcfg.get("start_poses") or {}).items():
        new_service.set_transform(
            rid,
            pose.get("x", 0.0),
            pose.get("y", 0.0),
            pose.get("yaw", 0.0),
            pose.get("z", 0.0),
        )
    map_service.__dict__.update(new_service.__dict__)
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
    await _gui_broadcaster.publish(msg)


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
    alert_id: str,
    level: str,
    kind: str,
    message: str,
    robot_id: str | None = None,
    detail: str | None = None,
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
        "detail": detail,
        "t_wall": time.time(),
        "acknowledged": False,
    }
    _alerts[alert_id] = alert
    events.log("alert", {"alert": alert})
    await broadcast({"type": "alert", "alert": alert})


async def clear_alert(alert_id: str) -> None:
    if _alerts.pop(alert_id, None) is not None:
        await broadcast({"type": "alert_clear", "id": alert_id})


async def sync_navigation_alert(robot) -> None:
    """One alert per navigation failure, saying why; retired once the robot
    navigates again, succeeds or stops."""
    rid = robot.robot_id
    current = _nav_failure_alerts.get(rid)
    if robot.nav_status != "failed":
        if current is not None:
            del _nav_failure_alerts[rid]
            await clear_alert(current)
        return
    reason = robot.nav_failure_reason
    if current is None:
        current = f"nav_failed_{rid}_{next(_nav_failure_serial)}"
        _nav_failure_alerts[rid] = current
    else:
        alert = _alerts.get(current)
        # Acknowledged, or already saying this. The reason can arrive a state
        # message after the status, so a reason that appears late replaces it.
        if alert is None or reason is None or alert.get("detail") == reason:
            return
        await clear_alert(current)
    await raise_alert(
        current,
        "warn",
        "nav_failure",
        explain_navigation_failure(reason),
        rid,
        detail=reason,
    )


def suppress_alert(alert_id: str) -> None:
    """Prevent the same alert id from reappearing for alert_suppress_s seconds."""
    seconds = float(settings_store.value.get("alert_suppress_s", 30))
    if seconds <= 0 or not alert_id:
        return
    _alert_suppress_until[alert_id] = time.time() + seconds


# ----------------------------------------------------------------- loops


async def state_loop() -> None:
    """Change-only robot_state fan-out with a 1 Hz per-robot keep-alive."""
    while True:
        await asyncio.sleep(STATE_LOOP_INTERVAL_S)
        await state_loop_tick()


def _robot_state_signature(message: dict[str, Any]) -> str:
    stable = {
        key: value
        for key, value in message.items()
        if key not in {"t_mono", "t_wall", "t_sess", "unattended_s"}
    }
    if isinstance(stable.get("live_mapping"), dict):
        # The 1 Hz keep-alive refreshes this clock without triggering fan-out.
        stable["live_mapping"] = {
            key: value
            for key, value in stable["live_mapping"].items()
            if key != "authority_age_s"
        }

    def rounded(value: Any) -> Any:
        if isinstance(value, float):
            # Normalize signed zero too: JSON distinguishes -0.0 from 0.0.
            return round(value, STATE_SIGNATURE_FLOAT_DIGITS) or 0.0
        if isinstance(value, dict):
            return {key: rounded(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [rounded(item) for item in value]
        return value

    return json.dumps(
        rounded(stable), sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )


async def state_loop_tick(now: float | None = None) -> None:
    """Run one state-loop iteration; exposed for focused scheduling tests."""
    now = time.monotonic() if now is None else now
    threshold = float(settings_store.value["unattended_threshold_s"])
    has_gui_clients = bool(_gui_clients)
    if not has_gui_clients:
        _state_loop_cache.clear()
    seen: set[str] = set()
    for r in list(registry.robots.values()):
        if has_gui_clients:
            state = robot_state(r)
            signature = _robot_state_signature(state)
            previous = _state_loop_cache.get(r.robot_id)
            if (
                previous is None
                or previous[0] != signature
                or now - previous[1] >= STATE_KEEPALIVE_S
            ):
                await broadcast(state)
                _state_loop_cache[r.robot_id] = (signature, now)
            seen.add(r.robot_id)

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

    for robot_id in set(_state_loop_cache) - seen:
        _state_loop_cache.pop(robot_id, None)


async def network_loop() -> None:
    """1 Hz per-robot Wi-Fi heatmap patches."""
    while True:
        await asyncio.sleep(1.0)
        for robot_id in map_service.network_robot_ids():
            patch = map_service.take_network_patch(robot_id)
            if patch:
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
    with map_service._state_lock:
        values = map_service.transforms.get(robot.robot_id, (0.0, 0.0, 0.0, 0.0))
    tx, ty, tz, yaw = map_service._placement(values)
    c, s = math.cos(yaw), math.sin(yaw)

    def to_world(point: dict[str, float]) -> dict[str, float]:
        value = dict(point)
        x, y = float(point["x"]), float(point["y"])
        value["x"], value["y"] = tx + x * c - y * s, ty + x * s + y * c
        if "z" in point:
            value["z"] = tz + float(point["z"])
        if "yaw" in point:
            value["yaw"] = map_service._wrap_yaw(float(point["yaw"]) + yaw)
        return value

    state["navigation_transform"] = {"x": tx, "y": ty, "yaw": yaw}
    state["pose"] = to_world(state["pose"])
    if state["goal"]:
        state["goal"] = to_world(state["goal"])
    for path_name in ("planned_path", "global_planned_path", "local_planned_path"):
        state[path_name] = [to_world(point) for point in state.get(path_name, [])]
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


def cancel_departures() -> None:
    """Stop releasing robots of an earlier fleet-wide Explore."""
    global _departure_task
    if _departure_task is not None and not _departure_task.done():
        _departure_task.cancel()
    _departure_task = None


async def reset_fleet(request_id: str | None = None) -> dict[str, Any]:
    """Put the simulation back to its start state.

    Adapters reset the simulator's model poses, odometry filter, and navigation
    state, then report `reset_done`. Only after that acknowledgement does the
    backend clear deployment raster and telemetry state. Clearing first would
    race with an in-flight navigation product.

    Backend state is cleared even when an adapter never answers. A stuck adapter
    must not leave the operator staring at stale state forever; robots that
    failed to confirm are named in the result.
    """
    cancel_departures()
    global _reset_done, _reset_running

    from .simulation_reset import request_reset, reset_root

    supervisor_root = reset_root()
    if supervisor_root is not None:
        result = request_reset(supervisor_root, request_id)
        await broadcast(
            {
                "type": "sim_reset",
                "phase": (
                    "start"
                    if result.get("phase")
                    in {"accepted", "stopping", "starting", "verifying"}
                    else "done"
                ),
                "request_id": result.get("request_id"),
                "ok": result.get("ok"),
                "error": result.get("error"),
                "skipped": [],
            }
        )
        return result

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
