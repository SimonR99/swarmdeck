#!/usr/bin/env python3
"""Observe one bounded fleet Explore run through an isolated SwarmDeck API.

The script records server-visible state and replica metadata. Robot displacement
is evidence that commands executed, not evidence of coverage or loop closure.
Stop All is attempted from ``finally`` on every run that reaches the event loop.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
from typing import Any
from urllib import error, parse, request


DEFAULT_BASE_URL = "http://127.0.0.1:18080"
MAX_JSON_BYTES = 8 * 1024 * 1024


class ObservationError(RuntimeError):
    pass


def validate_base_url(value: str) -> str:
    """Accept only an explicit loopback test API and never production port 8080."""

    parsed = parse.urlsplit(str(value).strip())
    if parsed.scheme != "http" or parsed.hostname not in {
        "127.0.0.1",
        "localhost",
        "::1",
    }:
        raise ValueError("observation API must be loopback HTTP")
    try:
        port = parsed.port
    except ValueError as exc:
        raise ValueError("observation API port is invalid") from exc
    if port is None:
        raise ValueError("observation API requires an explicit isolated port")
    if port == 8080:
        raise ValueError("production port 8080 is forbidden")
    path = parsed.path.rstrip("/")
    if path:
        raise ValueError("observation API URL must not contain a path")
    host = f"[{parsed.hostname}]" if ":" in parsed.hostname else parsed.hostname
    return f"http://{host}:{port}"


def websocket_url(base_url: str) -> str:
    parsed = parse.urlsplit(base_url)
    return parse.urlunsplit(("ws", parsed.netloc, "/ws", "", ""))


def get_json(base_url: str, path: str, timeout_s: float) -> Any:
    req = request.Request(
        base_url + path,
        method="GET",
        headers={"Accept": "application/json"},
    )
    try:
        with request.urlopen(req, timeout=timeout_s) as response:
            body = response.read(MAX_JSON_BYTES + 1)
    except error.HTTPError as exc:
        detail = exc.read(4097).decode("utf-8", "replace")
        raise ObservationError(f"GET {path} returned {exc.code}: {detail[:4096]}") from exc
    except (error.URLError, TimeoutError, OSError) as exc:
        raise ObservationError(f"GET {path} failed: {exc}") from exc
    if len(body) > MAX_JSON_BYTES:
        raise ObservationError(f"GET {path} exceeded {MAX_JSON_BYTES} bytes")
    try:
        return json.loads(body)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ObservationError(f"GET {path} returned invalid JSON") from exc


async def endpoint(base_url: str, path: str, timeout_s: float) -> dict[str, Any]:
    try:
        data = await asyncio.wait_for(
            asyncio.to_thread(get_json, base_url, path, timeout_s),
            timeout=timeout_s + 0.5,
        )
        return {"ok": True, "data": data}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _exception_detail(exc: BaseException) -> str:
    message = str(exc).strip()
    return f"{type(exc).__name__}: {message}" if message else type(exc).__name__


async def _drain_websocket(socket: Any) -> str | None:
    """Consume dashboard broadcasts so they cannot fill the client receive queue."""

    try:
        async for _message in socket:
            pass
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        return _exception_detail(exc)
    return None


async def _close_websocket(
    socket: Any, drain_task: asyncio.Task[str | None], timeout_s: float
) -> list[str]:
    errors = []
    try:
        async with asyncio.timeout(timeout_s):
            await socket.close()
            wait_closed = getattr(socket, "wait_closed", None)
            if callable(wait_closed):
                await wait_closed()
    except Exception as exc:
        errors.append(f"close: {_exception_detail(exc)}")
        transport = getattr(socket, "transport", None)
        abort = getattr(transport, "abort", None)
        if callable(abort):
            abort()

    if not drain_task.done():
        drain_task.cancel()
    try:
        async with asyncio.timeout(timeout_s):
            drain_error = await drain_task
        if drain_error:
            errors.append(f"receive drain: {drain_error}")
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        errors.append(f"receive drain cleanup: {_exception_detail(exc)}")
    return errors


async def send_gui_command(
    base_url: str, payload: dict[str, Any], timeout_s: float
) -> dict[str, Any]:
    """Write one GUI command while reporting close cleanup independently."""

    socket = None
    drain_task = None
    frame_written = False
    command_error = None
    try:
        import websockets

        socket = await asyncio.wait_for(
            websockets.connect(
                websocket_url(base_url),
                open_timeout=timeout_s,
                close_timeout=timeout_s,
                max_size=MAX_JSON_BYTES,
            ),
            timeout=timeout_s,
        )
        drain_task = asyncio.create_task(_drain_websocket(socket))
        await asyncio.wait_for(
            socket.send(json.dumps(payload, separators=(",", ":"))),
            timeout=timeout_s,
        )
        frame_written = True
    except Exception as exc:
        command_error = _exception_detail(exc)

    cleanup_errors = []
    if socket is not None:
        if drain_task is None:
            drain_task = asyncio.create_task(_drain_websocket(socket))
        cleanup_errors = await _close_websocket(socket, drain_task, timeout_s)

    result = {
        "sent": frame_written,
        "frame_written": frame_written,
        "delivery_confirmed": False,
        "payload": payload,
        "cleanup_complete": not cleanup_errors,
    }
    if command_error:
        result["error"] = command_error
    if cleanup_errors:
        result["cleanup_errors"] = cleanup_errors
    return result


def _robots(payload: Any) -> list[dict[str, Any]]:
    robots = payload.get("robots") if isinstance(payload, dict) else None
    return [robot for robot in robots or [] if isinstance(robot, dict)]


def _finite_pose(robot: dict[str, Any]) -> tuple[float, float] | None:
    pose = robot.get("pose")
    try:
        xy = float(pose["x"]), float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return xy if all(math.isfinite(value) for value in xy) else None


def summarize_post_stop(
    snapshot: dict[str, Any], eligible_robots: list[str]
) -> dict[str, Any]:
    """Describe the server's post-command state without calling it an acknowledgement."""

    fleet = snapshot.get("fleet", {})
    by_id = {
        robot["robot_id"]: robot
        for robot in _robots(fleet.get("data")) if fleet.get("ok")
        if isinstance(robot.get("robot_id"), str)
    }
    states = {}
    for robot_id in eligible_robots:
        robot = by_id.get(robot_id)
        if robot is None:
            states[robot_id] = {"present": False}
            continue
        global_points = len(robot.get("global_planned_path") or [])
        local_points = len(robot.get("local_planned_path") or [])
        nav_status = str(robot.get("nav_status", "unknown"))
        states[robot_id] = {
            "present": True,
            "online": bool(robot.get("online")),
            "mode": str(robot.get("mode", "unknown")),
            "exploration_status": str(robot.get("exploration_status", "unknown")),
            "navigation_status": nav_status,
            "global_path_points": global_points,
            "local_path_points": local_points,
            "reported_stopped": robot.get("exploration_status") == "stopped",
            "navigation_inactive": nav_status not in {"active", "nav"}
            and not bool(robot.get("goal")),
            "no_active_paths": global_points == 0 and local_points == 0,
        }

    complete = bool(eligible_robots) and len(states) == len(eligible_robots)
    return {
        "source": "server fleet snapshot after Stop All transport write",
        "protocol_acknowledgement": False,
        "fleet_snapshot_available": bool(fleet.get("ok")),
        "eligible_robot_states": states,
        "all_eligible_reported_stopped_and_inactive": complete
        and all(
            state.get("present")
            and state.get("online")
            and state.get("reported_stopped")
            and state.get("navigation_inactive")
            and state.get("no_active_paths")
            for state in states.values()
        ),
    }


def summarize(
    baseline: dict[str, Any],
    samples: list[dict[str, Any]],
    replica_manifests: list[dict[str, Any]],
    mission_id: str | None = None,
) -> dict[str, Any]:
    """Reduce raw evidence without inferring coverage or verified closures."""

    histories: dict[str, list[dict[str, Any]]] = {}
    snapshots = [baseline, *samples]
    for snapshot in snapshots:
        fleet = snapshot.get("fleet", {})
        if not fleet.get("ok"):
            continue
        for robot in _robots(fleet.get("data")):
            robot_id = robot.get("robot_id")
            if isinstance(robot_id, str) and robot_id:
                histories.setdefault(robot_id, []).append(robot)

    robot_summary = {}
    for robot_id, history in sorted(histories.items()):
        poses = [pose for robot in history if (pose := _finite_pose(robot)) is not None]
        displacement = None
        if len(poses) >= 2:
            displacement = round(math.dist(poses[0], poses[-1]), 3)
        exploration = [str(robot.get("exploration_status", "unknown")) for robot in history]
        navigation = [str(robot.get("nav_status", "unknown")) for robot in history]
        modes = [str(robot.get("mode", "unknown")) for robot in history]
        robot_summary[robot_id] = {
            "first_pose": (
                {"x": poses[0][0], "y": poses[0][1]} if poses else None
            ),
            "last_pose": (
                {"x": poses[-1][0], "y": poses[-1][1]} if poses else None
            ),
            "server_frame_displacement_m": displacement,
            "exploration_status_counts": dict(sorted(Counter(exploration).items())),
            "navigation_status_counts": dict(sorted(Counter(navigation).items())),
            "final_exploration_status": exploration[-1],
            "final_fleet_exploration_status": str(
                history[-1].get("fleet_exploration_status", "unknown")
            ),
            "final_navigation_status": navigation[-1],
            "ever_reported_blocked": "blocked" in exploration,
            "ever_reported_exploring": "exploring" in exploration,
            "ever_reported_executing": any(
                explore == "exploring" or nav in {"active", "nav"} or mode == "explore"
                for explore, nav, mode in zip(exploration, navigation, modes)
            ),
            "online_samples": sum(bool(robot.get("online")) for robot in history),
            "sample_count": len(history),
            "max_global_path_points": max(
                (len(robot.get("global_planned_path") or []) for robot in history),
                default=0,
            ),
            "max_local_path_points": max(
                (len(robot.get("local_planned_path") or []) for robot in history),
                default=0,
            ),
        }

    replica_revisions: dict[str, list[int]] = {}
    for snapshot in snapshots:
        index = snapshot.get("replica_index", {})
        if not index.get("ok"):
            continue
        rows = index.get("data", {}).get("replicas", [])
        for row in rows if isinstance(rows, list) else []:
            if not isinstance(row, dict):
                continue
            robot, session_id, revision = (
                row.get("robot_id"),
                row.get("session_id"),
                row.get("revision"),
            )
            if (
                isinstance(robot, str)
                and isinstance(session_id, str)
                and type(revision) is int
                and (mission_id is None or session_id == mission_id)
            ):
                replica_revisions.setdefault(f"{robot}/{session_id}", []).append(
                    revision
                )

    replicas = {
        key: {
            "first_observed_revision": revisions[0],
            "last_observed_revision": revisions[-1],
            "observed_revision_delta": revisions[-1] - revisions[0],
        }
        for key, revisions in sorted(replica_revisions.items())
    }
    components = []
    for item in replica_manifests:
        if not item.get("ok"):
            continue
        envelope = item.get("data")
        if not isinstance(envelope, dict):
            continue
        if mission_id is not None and envelope.get("session_id") != mission_id:
            continue
        key = f"{envelope.get('robot_id', '')}/{envelope.get('session_id', '')}"
        replicas.setdefault(key, {}).update(
            {
                "final_manifest_revision": envelope.get("revision"),
                "solution_order": envelope.get("solution_order"),
                "snapshot_id": (envelope.get("snapshot") or {}).get("snapshot_id"),
            }
        )
        for manifest in (envelope.get("snapshot") or {}).get("manifests", []):
            if not isinstance(manifest, dict):
                continue
            graph = manifest.get("graph_revision")
            if isinstance(graph, dict) and graph.get("component_id"):
                components.append(
                    {
                        "robot_id": envelope.get("robot_id"),
                        "session_id": envelope.get("session_id"),
                        "component_id": graph.get("component_id"),
                        "epoch": graph.get("epoch"),
                        "graph_revision": graph.get("revision"),
                        "geometry_revision": manifest.get("geometry_revision"),
                        # Current replica manifests do not carry a closure-
                        # verification boolean. Do not infer one from sharing
                        # a component ID or from robot motion.
                        "verified": (
                            graph.get("verified")
                            if isinstance(graph.get("verified"), bool)
                            else None
                        ),
                    }
                )
    return {
        "robots": robot_summary,
        "replicas": dict(sorted(replicas.items())),
        "reported_components": components,
        "verified_components": [item for item in components if item["verified"] is True],
        "interpretation_limits": [
            "displacement is measured in the server display frame and does not prove coverage",
            "a shared component identifier does not by itself prove a verified inter-robot closure",
            "command transport success does not confirm delivery to every robot",
        ],
    }


async def collect_sample(base_url: str, timeout_s: float, elapsed_s: float) -> dict[str, Any]:
    fleet, map_status, replicas = await asyncio.gather(
        endpoint(base_url, "/api/fleet", timeout_s),
        endpoint(base_url, "/api/map/status", timeout_s),
        endpoint(base_url, "/api/autonomy/replicas", timeout_s),
    )
    return {
        "elapsed_s": round(elapsed_s, 3),
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "fleet": fleet,
        "map_status": map_status,
        "replica_index": replicas,
    }


async def final_manifests(
    base_url: str,
    replica_index: dict[str, Any],
    timeout_s: float,
    mission_id: str | None,
    max_manifests: int,
) -> list[dict[str, Any]]:
    if not replica_index.get("ok"):
        return []
    rows = replica_index.get("data", {}).get("replicas", [])
    paths = []
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        robot = row.get("robot_id")
        session_id = row.get("session_id")
        if (
            isinstance(robot, str)
            and isinstance(session_id, str)
            and (mission_id is None or session_id == mission_id)
        ):
            path = "/api/autonomy/replicas/{}/{}".format(
                parse.quote(robot, safe=""), parse.quote(session_id, safe="")
            )
            paths.append(path)
    paths = sorted(set(paths))[:max_manifests]
    semaphore = asyncio.Semaphore(4)

    async def fetch(path: str) -> dict[str, Any]:
        async with semaphore:
            return await endpoint(base_url, path, timeout_s)

    return list(await asyncio.gather(*(fetch(path) for path in paths))) if paths else []


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


async def observe(args: argparse.Namespace) -> tuple[dict[str, Any], bool]:
    report: dict[str, Any] = {
        "schema_version": 1,
        "base_url": args.base_url,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "requested_duration_s": args.duration,
        "sample_interval_s": args.interval,
        "mission_id_filter": args.mission_id,
        "commands": {},
        "samples": [],
        "errors": [],
    }
    fatal = False
    stop_requested = asyncio.Event()
    loop = asyncio.get_running_loop()
    for name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, name, None)
        if sig is not None:
            try:
                loop.add_signal_handler(sig, stop_requested.set)
            except (NotImplementedError, RuntimeError):
                pass

    baseline = {
        "fleet": {"ok": False, "error": "not sampled"},
        "map_status": {"ok": False, "error": "not sampled"},
        "replica_index": {"ok": False, "error": "not sampled"},
    }
    report["baseline"] = baseline
    report["eligible_robots"] = []
    started = time.monotonic()
    try:
        baseline = await collect_sample(args.base_url, args.timeout, 0.0)
        report["baseline"] = baseline
        robots = _robots(baseline.get("fleet", {}).get("data"))
        eligible = sorted(
            robot["robot_id"]
            for robot in robots
            if robot.get("online")
            and "explore" in (robot.get("capabilities") or [])
            and isinstance(robot.get("robot_id"), str)
        )
        report["eligible_robots"] = eligible
        if not baseline.get("fleet", {}).get("ok"):
            raise ObservationError("initial fleet query failed")
        if not eligible:
            raise ObservationError("no online exploration-capable robots")
        report["commands"]["start_explore"] = await send_gui_command(
            args.base_url, {"type": "start_explore"}, args.timeout
        )
        if not report["commands"]["start_explore"]["sent"]:
            raise ObservationError("fleet Explore command was not sent")

        deadline = started + args.duration
        while True:
            now = time.monotonic()
            report["samples"].append(
                await collect_sample(args.base_url, args.timeout, now - started)
            )
            if now >= deadline or stop_requested.is_set():
                break
            try:
                await asyncio.wait_for(
                    stop_requested.wait(),
                    timeout=min(args.interval, max(0.0, deadline - now)),
                )
            except TimeoutError:
                pass
    except Exception as exc:
        fatal = True
        report["errors"].append(str(exc))
    finally:
        report["commands"]["stop_all"] = await send_gui_command(
            args.base_url, {"type": "stop_all"}, args.timeout
        )
        if not report["commands"]["stop_all"]["sent"]:
            fatal = True
            report["errors"].append("Stop All command was not sent")
        # Give adapters one bounded state period to report the stop without
        # turning this observation into an acknowledgement claim.
        await asyncio.sleep(min(1.0, args.interval))
        report["post_stop"] = await collect_sample(
            args.base_url, args.timeout, time.monotonic() - started
        )
        latest_index = report["post_stop"]["replica_index"]
        report["replica_manifests"] = await final_manifests(
            args.base_url,
            latest_index,
            args.timeout,
            args.mission_id,
            args.max_manifests,
        )
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        report["actual_duration_s"] = round(time.monotonic() - started, 3)
        report["summary"] = summarize(
            report["baseline"],
            report["samples"],
            report["replica_manifests"],
            args.mission_id,
        )
        report["summary"]["post_stop_state"] = summarize_post_stop(
            report["post_stop"], report["eligible_robots"]
        )
        write_report(args.output, report)
    return report, fatal


def parser() -> argparse.ArgumentParser:
    default_name = datetime.now(timezone.utc).strftime(
        "/tmp/swarmdeck-onboard-explore-%Y%m%dT%H%M%SZ.json"
    )
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--base-url", default=DEFAULT_BASE_URL)
    result.add_argument("--duration", type=float, default=60.0)
    result.add_argument("--interval", type=float, default=1.0)
    result.add_argument("--timeout", type=float, default=2.0)
    result.add_argument(
        "--mission-id",
        help="only fetch and summarize replicas whose session_id matches this mission",
    )
    result.add_argument("--max-manifests", type=int, default=16)
    result.add_argument("--output", type=Path, default=Path(default_name))
    return result


def main() -> None:
    args = parser().parse_args()
    try:
        args.base_url = validate_base_url(args.base_url)
        if not math.isfinite(args.duration) or not 1.0 <= args.duration <= 900.0:
            raise ValueError("duration must be between 1 and 900 seconds")
        if not math.isfinite(args.interval) or not 0.2 <= args.interval <= 30.0:
            raise ValueError("interval must be between 0.2 and 30 seconds")
        if not math.isfinite(args.timeout) or not 0.2 <= args.timeout <= 10.0:
            raise ValueError("timeout must be between 0.2 and 10 seconds")
        if not 1 <= args.max_manifests <= 64:
            raise ValueError("max-manifests must be between 1 and 64")
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    report, fatal = asyncio.run(observe(args))
    print(json.dumps(report["summary"], indent=2, sort_keys=True))
    print(f"evidence: {args.output}")
    raise SystemExit(1 if fatal else 0)


if __name__ == "__main__":
    main()
