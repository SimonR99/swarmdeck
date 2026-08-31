"""One WebSocket session for every adapter.

Connect, send `hello`, pump telemetry/maps/camera on separate coroutines,
reconnect with backoff. Bridges supply ROS-specific work through methods
this loop looks up by name; missing methods are skipped.
"""

from __future__ import annotations

import asyncio
import inspect
import json
import time
from collections.abc import Callable
from typing import Any

from adapters.runtime import (
    RECONNECT_BACKOFF_S,
    TRANSPORT_DEFAULTS,
    detections_message,
    next_backoff,
    run_until_first_failure,
    websocket_connect_kwargs,
)


def _cfg(bridge: Any) -> dict[str, Any]:
    return getattr(bridge, "cfg", None) or TRANSPORT_DEFAULTS


def _emit(bridge: Any, level: str, message: str) -> None:
    node = getattr(bridge, "node", None)
    if node is not None:
        try:
            getattr(node.get_logger(), level)(message)
            return
        except Exception:
            pass
    try:
        import rospy

        getattr(rospy, f"log{level}")(message)
        return
    except Exception:
        pass
    print(message)


def _call(bridge: Any, name: str, *args: Any) -> Any:
    fn = getattr(bridge, name, None)
    return fn(*args) if callable(fn) else None


async def _offload(loop: asyncio.AbstractEventLoop, bridge: Any, name: str) -> Any:
    fn = getattr(bridge, name, None)
    if not callable(fn):
        return None
    return await loop.run_in_executor(None, fn)


async def dispatch_command(
    bridge: Any, msg: dict[str, Any], loop: asyncio.AbstractEventLoop
) -> None:
    """Map one adapter-protocol command onto the bridge. Unknown types: no-op."""
    kind = msg.get("type")
    if kind == "navigate_to":
        fn = bridge.navigate_to
        goal = msg.get("goal", {})
        try:
            takes_path = len(inspect.signature(fn).parameters) > 1
        except (TypeError, ValueError):
            takes_path = False
        if takes_path:
            await loop.run_in_executor(None, fn, goal, msg.get("path"))
        else:
            await loop.run_in_executor(None, fn, goal)
    elif kind == "cancel_goal":
        (getattr(bridge, "cancel_goal", None) or bridge.cancel)()
    elif kind == "drive":
        lin, ang = msg.get("linear", 0.0), msg.get("angular", 0.0)
        latch = getattr(bridge, "note_drive_command", None)
        if callable(latch):
            latch(lin, ang)
        else:
            bridge.drive(lin, ang)
    elif kind == "stop":
        bridge.stop()
    elif kind == "set_mode":
        bridge.mode = msg.get("mode", bridge.mode)
    elif kind == "body_command":
        fn = getattr(bridge, "body_command", None)
        if callable(fn):
            await loop.run_in_executor(None, fn, msg.get("action", ""))
    elif kind == "reset":
        fn = getattr(bridge, "reset", None)
        if callable(fn):
            loop.run_in_executor(None, fn)


async def _rx(bridge: Any, ws: Any) -> None:
    loop = asyncio.get_running_loop()
    async for raw in ws:
        _call(bridge, "note_link_activity")
        try:
            msg = json.loads(raw)
        except ValueError:
            continue
        await dispatch_command(bridge, msg, loop)


async def _tx_state(bridge: Any, send: Callable, cfg: dict[str, Any]) -> None:
    period = 1.0 / float(cfg["rates"]["state_hz"])
    while True:
        await send(bridge.state())
        extra = _call(bridge, "session_state_tick")
        if extra is not None:
            await send(extra)
        _call(bridge, "note_link_activity")
        await asyncio.sleep(period)


async def _tx_maps(bridge: Any, send: Callable, cfg: dict[str, Any]) -> None:
    rates = cfg["rates"]
    tick = 1.0 / float(rates["state_hz"])
    loop = asyncio.get_running_loop()
    last_map = 0.0
    last_cloud = 0.0
    last_settings = 0.0
    while True:
        now = time.monotonic()
        if now - last_map > float(rates["map_period_s"]):
            meta = await _offload(loop, bridge, "upload_map")
            if meta:
                await send(meta)
            await _offload(loop, bridge, "upload_scan")
            last_map = time.monotonic()
        await _offload(loop, bridge, "upload_costmaps")
        if now - last_cloud > float(rates["cloud_period_s"]):
            await _offload(loop, bridge, "upload_cloud")
            last_cloud = time.monotonic()
        await _offload(loop, bridge, "upload_keyframe")
        await _offload(loop, bridge, "pull_nav_map")
        extra = getattr(bridge, "session_maps_tick", None)
        if callable(extra):
            await extra(now, send, loop)
        if now - last_settings > float(rates.get("settings_period_s", 5.0)):
            last_settings = now
            await _offload(loop, bridge, "refresh_settings")
        await asyncio.sleep(tick)


async def _tx_camera(bridge: Any, send: Callable, cfg: dict[str, Any]) -> None:
    rates = cfg["rates"]
    tick = 1.0 / float(rates["state_hz"])
    loop = asyncio.get_running_loop()
    detect = getattr(bridge, "run_detection", None) or getattr(
        bridge, "process_camera", None
    )
    last_detect = 0.0
    while True:
        now = time.monotonic()
        if detect is not None and now - last_detect > float(rates["camera_period_s"]):
            await loop.run_in_executor(None, detect)
            detections = _call(bridge, "take_detections")
            if detections is not None:
                await send(
                    detections_message(bridge.id, bridge.t0, detections, now=now)
                )
            last_detect = time.monotonic()
        await asyncio.sleep(tick)


def _stop_on_disconnect(bridge: Any) -> None:
    # Cancel first: while nav is active, a zero Twist is overwritten by the
    # next autonomy sample and the robot never actually stops.
    try:
        (getattr(bridge, "cancel_goal", None) or getattr(bridge, "cancel"))()
        bridge.drive(0.0, 0.0)
    except Exception:
        pass


async def run_adapter_session(
    bridge: Any,
    ws_url: str,
    *,
    connect: Callable[..., Any] | None = None,
) -> None:
    """Connect, announce, pump, reconnect. `connect` is injectable for tests."""
    cfg = _cfg(bridge)
    if connect is None:
        import websockets

        open_ws = websockets.connect
    else:
        open_ws = connect
    backoff = RECONNECT_BACKOFF_S
    while True:
        try:
            async with open_ws(ws_url, **websocket_connect_kwargs(cfg)) as ws:
                await ws.send(json.dumps(bridge.hello()))
                _emit(
                    bridge,
                    "info",
                    f"[{bridge.id}] connected; capabilities={bridge.capabilities()}",
                )
                backoff = RECONNECT_BACKOFF_S
                _call(bridge, "note_link_activity")
                send_lock = asyncio.Lock()

                async def send(payload: dict[str, Any]) -> None:
                    async with send_lock:
                        await ws.send(json.dumps(payload))

                await run_until_first_failure(
                    _rx(bridge, ws),
                    _tx_state(bridge, send, cfg),
                    _tx_maps(bridge, send, cfg),
                    _tx_camera(bridge, send, cfg),
                )
        except Exception as exc:
            _emit(
                bridge,
                "warn",
                f"[{bridge.id}] disconnected ({exc}); retrying in {backoff:.0f}s",
            )
            _stop_on_disconnect(bridge)
            await asyncio.sleep(backoff)
            backoff = next_backoff(backoff)
