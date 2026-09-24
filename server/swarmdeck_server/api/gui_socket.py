"""Dashboard WebSocket connection and inbound command dispatcher."""

from __future__ import annotations

import asyncio
import json
from typing import Any
from uuid import uuid4

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import state

router = APIRouter()

# ----------------------------------------------------------------- GUI socket


@router.websocket("/ws")
async def gui_socket(ws: WebSocket) -> None:
    await ws.accept()
    try:
        await state._gui_broadcaster.subscribe(ws, gui_snapshot)

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
        state._gui_clients.discard(ws)


def gui_snapshot() -> list[dict[str, Any]]:
    """Capture initial GUI state atomically with respect to publications."""
    return [
        {"type": "fleet_change", "robots": state.fleet_snapshot()},
        state.session_state(),
        {"type": "settings_state", "settings": state.settings_store.value},
        state.review_state(),
        *({"type": "alert", "alert": alert} for alert in state._alerts.values()),
    ]


async def handle_gui_message(msg: dict[str, Any], source: Any = None) -> None:
    kind = msg.get("type", "")
    rid = msg.get("robot_id", "")

    # Every operator action is logged before it takes effect.
    state.events.log(kind, {k: v for k, v in msg.items() if k != "type"})
    if rid:
        state.registry.attend(rid)
    if rid and kind in {"return_home", "body_command", "start_explore"}:
        from .map_routes import robot_command_error

        error = robot_command_error(rid)
        if error:
            await state.raise_alert(f"map_reset_{rid}", "warn", "fault", error, rid)
            return

    if (
        rid
        and kind in {"return_home", "drive", "body_command"}
        and not state.is_robot_enabled(state.settings_store.value, rid)
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
            changed = state.review_store.accept(pid) is not None
        elif kind == "detection_ignore":
            changed = state.review_store.ignore(pid)
        elif kind == "detection_merge":
            changed = state.review_store.merge(pid, eid) is not None
        elif kind == "detection_unignore":
            changed = state.review_store.clear_ignored() > 0
        elif kind == "detection_forget_all":
            include_props = bool(msg.get("include_proposals", False))
            changed = state.review_store.forget_all(include_proposals=include_props) > 0
        elif kind == "detection_clear_proposals":
            changed = state.review_store.clear_proposals() > 0
        elif kind == "detection_delete_all":
            changed = state.review_store.delete_all() > 0
        else:
            if pid and not eid:
                changed = state.review_store.forget_proposal(pid)
            elif eid:
                changed = state.review_store.forget(
                    eid
                ) or state.review_store.forget_proposal(eid)
            else:
                changed = False
        # Silence on an unknown id is deliberate: two operators can answer the
        # same proposal, and the loser of that race must not get an error for
        # having agreed.
        if changed:
            # Persist before broadcasting: an operator who sees the dashboard
            # confirm a deletion and then loses the process must not find the
            # object back on the map.
            state.save_review(force=True)
            await state.broadcast_review()
        return

    if kind == "return_home":
        if not state.registry.can(rid, "plan_objective"):
            await state.raise_alert(
                f"objective_unavailable_{rid}",
                "warn",
                "fault",
                "Robot does not support plan_objective",
                rid,
            )
            return
        from .objective_commands import send_objective

        await send_objective(state.registry, rid, "return_home")
        return
    elif kind == "cancel_goal":
        sent = await state.registry.send(rid, {"type": "cancel_goal", **state.stamps()})
        if sent and rid in state.registry.robots:
            state.registry.robots[rid].goal = None
            state.registry.robots[rid].global_planned_path = []
            state.registry.robots[rid].local_planned_path = []
            state.registry.robots[rid].planned_path = []

    elif kind == "drive":
        if not (
            state.registry.can(rid, "navigate") or state.registry.can(rid, "estop")
        ):
            return
        payload = msg.get("payload") if isinstance(msg.get("payload"), dict) else msg
        try:
            linear = max(-0.8, min(0.8, float(payload.get("linear", 0.0))))
            angular = max(-1.5, min(1.5, float(payload.get("angular", 0.0))))
        except (TypeError, ValueError):
            return
        sent = await state.registry.send(
            rid,
            {"type": "drive", "linear": linear, "angular": angular, **state.stamps()},
        )
        if sent:
            state.registry.robots[rid].goal = None

    elif kind == "body_command":
        action = str(msg.get("action") or "")
        if action not in (
            "claim",
            "release",
            "sit",
            "stand",
            "damping",
            "lie_to_stand",
            "lock_stand",
            "walk_mode",
            "run_mode",
            "wave",
            "set_height",
        ):
            return
        if not state.registry.can(rid, "body"):
            return
        body_msg = {"type": "body_command", "action": action, **state.stamps()}
        if "height" in msg:
            try:
                body_msg["height"] = float(msg["height"])
            except (ValueError, TypeError):
                pass
        await state.registry.send(rid, body_msg)

    elif kind == "stop_all":
        state.cancel_departures()
        # Whether each robot was actually REACHED, not merely addressed. A stop
        # that went nowhere is the one command an operator must never be allowed
        # to believe succeeded, so the robots it failed to reach are named.
        undelivered = [
            robot_id
            for robot_id in list(state.registry.robots)
            if not await state.registry.send(
                robot_id, {"type": "stop", **state.stamps()}
            )
        ]
        await state.raise_alert(
            "stop_all", "critical", "fault", "STOP ALL issued by operator"
        )
        if undelivered:
            state.events.log("stop_all_undelivered", {"robots": sorted(undelivered)})
            await state.raise_alert(
                "stop_all_undelivered",
                "critical",
                "fault",
                f"STOP did not reach {', '.join(sorted(undelivered))} — "
                f"they may still be moving",
            )

    elif kind in ("start_explore", "stop_explore"):
        # A robot card addresses one configured explorer. Legacy fleet-wide
        # commands remain supported; capability gating also applies to hardware.
        enabled = kind == "start_explore"
        run_id = str(uuid4())
        targets = [
            robot_id
            for robot_id in list(state.registry.robots)
            if state.registry.can(robot_id, "explore") and (not rid or robot_id == rid)
        ]
        if not targets:
            await state.raise_alert(
                "explore_unsupported",
                "warn",
                "fault",
                "No connected robot supports exploration",
            )
        else:
            state.cancel_departures()
            generations = {
                robot_id: state.registry.robots[robot_id].command_generation
                for robot_id in targets
            }

            async def send_explore(robot_id: str) -> bool:
                robot = state.registry.robots.get(robot_id)
                if robot is None or robot.command_generation != generations[robot_id]:
                    return False
                return await state.registry.send(
                    robot_id,
                    {
                        "type": "explore",
                        "enabled": enabled,
                        "run_id": run_id,
                        "participants": sorted(targets),
                        **state.stamps(),
                    },
                )

            departure = (state.CONFIG.get("fleet") or {}).get("departure") or {}
            clearance_m = float(departure.get("clearance_m", 0.0))
            if enabled and len(targets) > 1 and clearance_m > 0.0:
                # Robots parked together are in nobody's map, so they leave
                # one at a time, head of the group first. Stop, Stop All, a
                # reset or another Explore ends the sequence.
                order = state.departure_order(
                    {
                        robot_id: state.registry.robots[robot_id].pose
                        for robot_id in targets
                    }
                )

                async def depart() -> None:
                    undelivered = await state.release_in_turn(
                        order,
                        send_explore,
                        lambda robot_id: (
                            state.registry.robots[robot_id].pose
                            if robot_id in state.registry.robots
                            else None
                        ),
                        clearance_m=clearance_m,
                        timeout_s=float(departure.get("timeout_s", 30.0)),
                    )
                    state.events.log(
                        kind, {"robots": order, "undelivered": sorted(undelivered)}
                    )

                state._departure_task = asyncio.create_task(depart())
                undelivered = []
            else:
                undelivered = [
                    robot_id for robot_id in targets if not await send_explore(robot_id)
                ]
                state.events.log(
                    kind,
                    {"robots": sorted(targets), "undelivered": sorted(undelivered)},
                )
            # Stop is the direction worth alerting on. A start that did not
            # arrive shows up immediately as robots that do not move; a stop
            # that did not arrive leaves them driving, which is the same class
            # of problem stop_all names its failures for.
            if undelivered and not enabled:
                await state.raise_alert(
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
        asyncio.create_task(state.reset_fleet())

    elif kind == "acknowledge_alert":
        aid = msg.get("id", "")
        if aid in state._alerts:
            state._alerts[aid]["acknowledged"] = True
        state.suppress_alert(aid)
        await state.clear_alert(aid)

    elif kind in ("discard_robot", "remove_robot"):
        if rid and rid in state.registry.robots:
            state.registry.disconnect(rid)
            state.registry.remove(rid)
            if hasattr(state.map_service, "reset_robot_async"):
                await state.map_service.reset_robot_async(rid)
            state.events.log("robot_discarded", {"robot_id": rid})
            await state.broadcast(
                {"type": "fleet_change", "robots": state.fleet_snapshot()}
            )

    elif kind in ("select_robots", "report_target", "switch_camera"):
        pass  # logged above; no robot-side effect
