"""Adapter WebSocket connection, telemetry and reset acknowledgements."""

from __future__ import annotations

import asyncio
import json
import os
import time
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import state

router = APIRouter()

# ----------------------------------------------------------------- adapter socket


@router.websocket("/adapter")
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
            state.registry.disconnect(robot_id, ws)
            state.events.log("adapter_disconnect", {"robot_id": robot_id})


def registry_id_of(hello: dict[str, Any]) -> str | None:
    """The robot_id a `hello` claimed, once the registry has accepted it."""
    rid = hello.get("robot_id")
    return rid if isinstance(rid, str) and rid in state.registry.robots else None


async def handle_adapter_message(msg: dict[str, Any], ws: WebSocket) -> bool:
    """Dispatch one adapter message. Returns True if the socket was closed."""
    kind = msg.get("type", "")

    if kind == "hello":
        # 2 adds the optional `slam_graph` message and nothing else, so a
        # protocol-1 adapter stays valid forever. Rejecting it would
        # break exactly the mixed-fleet property the contract exists for.
        if msg.get("protocol") not in state.SUPPORTED_PROTOCOLS:
            await ws.close(code=4400)
            return True
        client = getattr(ws, "client", None)
        r = state.registry.hello(msg, ws, peer=client.host if client else "")
        robot_id = r.robot_id
        if r.coordinate_frame == "merged":
            state.map_service.set_transform(robot_id, 0.0, 0.0, 0.0)
        await ws.send_json({"type": "hello_ack", "robot_id": robot_id})
        known_ids = {
            item.get("id")
            for item in state.settings_store.value.get("robots", [])
            if isinstance(item, dict)
        }
        if robot_id not in known_ids:
            state.settings_store.value.setdefault("robots", []).append(
                {
                    "id": robot_id,
                    "enabled": True,
                    "color": state.default_color(
                        robot_id, len(state.settings_store.value.get("robots", []))
                    ),
                }
            )
            state.settings_store.save(state.settings_store.value)
            await state.broadcast(
                {"type": "settings_state", "settings": state.settings_store.value}
            )
        await state.broadcast(
            {"type": "fleet_change", "robots": state.fleet_snapshot()}
        )
        state.events.log(
            "adapter_connect", {"robot_id": robot_id, "adapter": r.adapter}
        )

    elif kind == "robot_state":
        if state.registry._sinks.get(msg.get("robot_id")) is not ws:
            return False
        mission = os.environ.get("SWARMDECK_MISSION_ID")
        robot_id = msg.get("robot_id")
        if mission and isinstance(robot_id, str):
            from .map_routes import cached_map_epoch_async

            await cached_map_epoch_async(robot_id, mission)
        robot = state.registry.update_state(msg)
        if robot is not None:
            await state.sync_navigation_alert(robot)
        network = msg.get("network")
        pose = msg.get("pose")
        if robot is not None and isinstance(network, dict) and isinstance(pose, dict):
            try:
                state.map_service.ingest_network_sample(
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
            if not state.detection_class_enabled(item.get("class", "object")):
                continue
            detection_id = f"{rid}:{item.get('id', item.get('class', 'object'))}"
            visible.add(detection_id)
            previous = state._detections.get(detection_id)
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
                "map_position": state.detection_position(rid, item.get("map_position")),
                "first_seen": previous["first_seen"] if previous else now,
                "last_seen": now,
                "observations": (previous["observations"] + 1) if previous else 1,
            }
            det["hidden"] = state.detection_hidden(det, state.settings_store.value)
            state._detections[detection_id] = det

            # Route the located sighting to the operator's review queue. Only
            # evidence that clears the display floor is worth asking about: a
            # detection the operator has already decided is noise should not
            # come back as a question.
            if det["map_position"] and not det["hidden"]:
                # Where the reporting robot is standing, so a sighting from a
                # pose we have already averaged does not get averaged again.
                # Without it a parked robot's depth bias becomes the object's
                # position. See MIN_VIEWPOINT_MOVE_M in detect/review.py.
                observer = state.registry.robots.get(rid)
                outcome, _target = state.review_store.observe(
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
                    state._review_dirty = True
            # Robots capture below the operator's floor on purpose, so that
            # lowering it later can be answered from this cache instead of from
            # frames that no longer exist.  Until then the entity is stored and
            # not sent: an operator who raised a floor should see no trace of it.
            if not det["hidden"]:
                await state.broadcast({"type": "detection", "detection": det})

        if review_push:
            await state.broadcast_review(review_push)

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
            for key, det in state._detections.items()
            if det["robot_id"] == rid
            and det["camera"] == camera
            and key not in visible
            and det["bbox"] is not None
        ]
        for key in retracted:
            # Re-read: the broadcast below yields, so another adapter's
            # batch can land between building this list and using it.
            current = state._detections.get(key)
            if current is None or current["bbox"] is None:
                continue
            det = {**current, "bbox": None, "polygon": None}
            state._detections[key] = det
            # A hidden entity was never sent, so there is no box out there to
            # retract; the cache still has to drop it or un-hiding this entity
            # later would restore a rectangle from an old frame.
            if not det.get("hidden", False):
                await state.broadcast({"type": "detection", "detection": det})

    elif kind == "reset_done":
        # The adapter has finished resetting and has dropped its cached
        # grid, so the backend may now clear without the old map coming
        # straight back. See reset_fleet().
        #
        # A partial failure is recorded rather than alerted on here:
        # reset_fleet() clears every alert once the fleet has answered,
        # which would wipe an alert raised from inside this branch.
        rid = msg.get("robot_id", "")
        if rid in state._reset_pending:
            if not msg.get("ok", True):
                state._reset_failures[rid] = msg.get("steps") or {}
            state._reset_pending.discard(rid)
            if not state._reset_pending and state._reset_done is not None:
                state._reset_done.set()

    # Unknown types are ignored, not fatal (protocol rule 3).
    return False
