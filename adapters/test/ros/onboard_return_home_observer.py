#!/usr/bin/env python3
"""Observe one bounded onboard-MGG Return Home run through an isolated API."""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
from typing import Any

from adapters.test.ros.onboard_exploration_observer import (
    ObservationError,
    _robots,
    collect_sample,
    final_manifests,
    send_gui_command,
    summarize_post_stop,
    validate_base_url,
    write_report,
)


def load_authority(path: Path, max_age_s: float) -> tuple[dict[str, Any], float]:
    """Read a fresh plain-JSON map-authority fixture."""

    age_s = max(0.0, time.time() - path.stat().st_mtime)
    if age_s > max_age_s:
        raise ValueError(
            f"authority fixture is {age_s:.1f}s old; limit is {max_age_s:.1f}s"
        )
    try:
        value = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(
            "authority fixture must be plain JSON from ros2 topic echo --field data"
        ) from exc
    if not isinstance(value, dict):
        raise ValueError("authority fixture does not contain a JSON object")
    return value, age_s


def authority_home(
    authority: dict[str, Any], robot_id: str, mission_id: str | None
) -> dict[str, Any]:
    """Validate the authority binding and extract its corrected navigation-frame home."""

    if authority.get("robot_id") != robot_id:
        raise ValueError("authority robot_id does not match the requested robot")
    if mission_id is not None and authority.get("mission_id") != mission_id:
        raise ValueError("authority mission_id does not match the requested mission")
    if (
        authority.get("navigation_frame", "").lstrip("/")
        != f"{robot_id}/navigation_frame"
    ):
        raise ValueError(
            "authority navigation_frame does not match the robot map frame"
        )
    if (
        not isinstance(authority.get("component_id"), str)
        or not authority["component_id"]
    ):
        raise ValueError("authority component_id is missing")
    home = authority.get("home")
    if not isinstance(home, dict) or not isinstance(home.get("keyframe_id"), str):
        raise ValueError("authority corrected home landmark is missing")
    try:
        matrix = [[float(value) for value in row] for row in home["T_navigation_home"]]
        correction_revision = int(authority["correction_revision"])
        map_epoch = int(authority["map_epoch"])
        robot_map_epoch = int(authority["robot_map_epoch"])
        graph_revision = int(authority["mapping_graph_revision"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("authority revision or home transform is invalid") from exc
    if (
        len(matrix) != 4
        or any(len(row) != 4 for row in matrix)
        or not all(math.isfinite(value) for row in matrix for value in row)
        or min(correction_revision, map_epoch, robot_map_epoch, graph_revision) < 0
    ):
        raise ValueError("authority revision or home transform is invalid")
    run_id = authority.get("run_id")
    if not isinstance(run_id, str) or home["keyframe_id"] != f"{robot_id}/{run_id}/0":
        raise ValueError("authority home does not belong to this robot map run")
    return {
        "x": matrix[0][3],
        "y": matrix[1][3],
        "z": matrix[2][3],
        "yaw": math.atan2(matrix[1][0], matrix[0][0]),
        "landmark_id": home["keyframe_id"],
        "mission_id": authority.get("mission_id"),
        "robot_map_epoch": robot_map_epoch,
        "run_id": run_id,
        "component_id": authority["component_id"],
        "correction_revision": correction_revision,
        "map_epoch": map_epoch,
        "mapping_graph_revision": graph_revision,
        "geometry_revision": authority.get("geometry_revision"),
    }


def _robot(snapshot: dict[str, Any], robot_id: str) -> dict[str, Any] | None:
    fleet = snapshot.get("fleet", {})
    if not fleet.get("ok"):
        return None
    return next(
        (
            robot
            for robot in _robots(fleet.get("data"))
            if robot.get("robot_id") == robot_id
        ),
        None,
    )


def _xy(value: Any) -> tuple[float, float] | None:
    try:
        result = float(value["x"]), float(value["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def display_home(
    snapshot: dict[str, Any], robot_id: str, home: dict[str, Any]
) -> dict[str, float]:
    """Project a navigation-frame home into this sample's server display frame."""

    status = snapshot.get("map_status", {})
    transforms = (
        status.get("data", {}).get("transforms", []) if status.get("ok") else {}
    )
    transform = transforms.get(robot_id) if isinstance(transforms, dict) else None
    try:
        tx, ty, yaw = (
            float(transform["x"]),
            float(transform["y"]),
            float(transform["yaw"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ObservationError(
            f"{robot_id} has no valid navigation-to-display transform"
        ) from exc
    if not all(math.isfinite(value) for value in (tx, ty, yaw)):
        raise ObservationError(
            f"{robot_id} navigation-to-display transform is nonfinite"
        )
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = float(home["x"]), float(home["y"])
    return {
        "x": tx + x * c - y * s,
        "y": ty + x * s + y * c,
        "yaw": (float(home["yaw"]) + yaw + math.pi) % (2 * math.pi) - math.pi,
        "transform_x": tx,
        "transform_y": ty,
        "transform_yaw": yaw,
    }


def summarize_return_home(
    robot_id: str,
    authority: dict[str, Any],
    baseline: dict[str, Any],
    samples: list[dict[str, Any]],
    arrival_tolerance_m: float,
) -> dict[str, Any]:
    observations = []
    for snapshot in [baseline, *samples]:
        robot = _robot(snapshot, robot_id)
        if robot is None:
            continue
        target = display_home(snapshot, robot_id, authority)
        observations.append((robot, target))
    history = [robot for robot, _target in observations]
    distances = [
        math.dist(pose, (target["x"], target["y"]))
        for robot, target in observations
        if (pose := _xy(robot.get("pose"))) is not None
    ]
    nav_statuses = [str(robot.get("nav_status", "unknown")) for robot in history]
    exploration_statuses = [
        str(robot.get("exploration_status", "unknown")) for robot in history
    ]
    route, route_target = max(
        (
            (robot.get("global_planned_path") or [], target)
            for robot, target in observations
            if isinstance(robot.get("global_planned_path") or [], list)
        ),
        key=lambda item: len(item[0]),
        default=([], None),
    )
    endpoint = _xy(route[-1]) if route else None
    endpoint_error = (
        math.dist(endpoint, (route_target["x"], route_target["y"]))
        if endpoint is not None and route_target is not None
        else None
    )
    terminal = next(
        (
            status
            for status in reversed(nav_statuses)
            if status in {"succeeded", "failed", "cancelled"}
        ),
        None,
    )
    blocked = "blocked" in exploration_statuses
    display_arrival = bool(distances) and distances[-1] <= arrival_tolerance_m
    route_matches = endpoint_error is not None and endpoint_error <= arrival_tolerance_m
    multi_waypoint = len(route) > 1
    succeeded = terminal == "succeeded"
    return {
        "robot_id": robot_id,
        "authority_home_navigation_frame": authority,
        "initial_authority_home_display_frame": (
            observations[0][1] if observations else None
        ),
        "last_authority_home_display_frame": (
            observations[-1][1] if observations else None
        ),
        "initial_server_frame_distance_to_authority_home_m": (
            round(distances[0], 3) if distances else None
        ),
        "closest_server_frame_distance_to_authority_home_m": (
            round(min(distances), 3) if distances else None
        ),
        "last_server_frame_distance_to_authority_home_m": (
            round(distances[-1], 3) if distances else None
        ),
        "terminal_navigation_status_before_final_stop": terminal,
        "navigation_statuses_observed": sorted(set(nav_statuses)),
        "exploration_statuses_observed": sorted(set(exploration_statuses)),
        "blocked_reported": blocked,
        "max_global_path_points": len(route),
        "multi_waypoint_route_observed": multi_waypoint,
        "observed_route_endpoint": (
            {"x": endpoint[0], "y": endpoint[1]} if endpoint is not None else None
        ),
        "route_endpoint_error_to_authority_home_m": (
            round(endpoint_error, 3) if endpoint_error is not None else None
        ),
        "route_endpoint_matches_authority_home": route_matches,
        "server_frame_arrival_consistent": display_arrival,
        "server_evidence_consistent_with_objective_execution": (
            succeeded
            and display_arrival
            and multi_waypoint
            and route_matches
            and not blocked
        ),
        "interpretation": (
            "consistent server evidence requires a multi-waypoint route to the fresh "
            "authority home, a succeeded status, and a matching displayed pose; it is "
            "not independent physical-arrival truth, and blocked/failed motion is not success"
        ),
    }


def _safe_baseline_robot(snapshot: dict[str, Any], robot_id: str) -> dict[str, Any]:
    robot = _robot(snapshot, robot_id)
    if robot is None or not robot.get("online"):
        raise ObservationError(
            f"{robot_id} is not online in the initial fleet snapshot"
        )
    capabilities = set(robot.get("capabilities") or [])
    if not {"navigate", "plan_objective"}.issubset(capabilities):
        raise ObservationError(
            f"{robot_id} does not advertise onboard objective planning"
        )
    if robot.get("exploration_status") != "stopped":
        raise ObservationError(f"{robot_id} is not initially stopped")
    if robot.get("nav_status") != "idle" or robot.get("goal"):
        raise ObservationError(f"{robot_id} does not report idle navigation")
    if robot.get("global_planned_path") or robot.get("local_planned_path"):
        raise ObservationError(f"{robot_id} already reports an active path")
    return robot


async def observe(args: argparse.Namespace) -> tuple[dict[str, Any], bool]:
    raw_authority, authority_age_s = load_authority(
        args.authority_file, args.max_authority_age
    )
    home = authority_home(raw_authority, args.robot_id, args.mission_id)
    report: dict[str, Any] = {
        "schema_version": 1,
        "base_url": args.base_url,
        "robot_id": args.robot_id,
        "mission_id_filter": args.mission_id,
        "requested_duration_s": args.duration,
        "sample_interval_s": args.interval,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "authority_fixture": {
            "path": str(args.authority_file),
            "age_at_start_s": round(authority_age_s, 3),
            "max_age_s": args.max_authority_age,
            "raw": raw_authority,
        },
        "commands": {},
        "objective_activity_observed": False,
        "samples": [],
        "errors": [],
    }
    baseline = {
        "fleet": {"ok": False, "error": "not sampled"},
        "map_status": {"ok": False, "error": "not sampled"},
        "replica_index": {"ok": False, "error": "not sampled"},
    }
    report["baseline"] = baseline
    fatal = False

    def checkpoint(phase: str) -> None:
        report["checkpoint"] = {
            "phase": phase,
            "written_at": datetime.now(timezone.utc).isoformat(),
        }
        write_report(args.output, report)

    stop_requested = asyncio.Event()
    loop = asyncio.get_running_loop()
    for name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, name, None)
        if sig is not None:
            try:
                loop.add_signal_handler(sig, stop_requested.set)
            except (NotImplementedError, RuntimeError):
                pass

    started = time.monotonic()
    try:
        baseline = await collect_sample(args.base_url, args.timeout, 0.0)
        report["baseline"] = baseline
        checkpoint("baseline_sampled")
        _safe_baseline_robot(baseline, args.robot_id)
        display_home(baseline, args.robot_id, home)
        report["commands"]["return_home"] = await send_gui_command(
            args.base_url,
            {"type": "return_home", "robot_id": args.robot_id},
            args.timeout,
        )
        if not report["commands"]["return_home"]["sent"]:
            raise ObservationError("Return Home command frame was not written")

        deadline = started + args.duration
        while True:
            now = time.monotonic()
            sample = await collect_sample(args.base_url, args.timeout, now - started)
            report["samples"].append(sample)
            checkpoint("objective_sampled")
            display_home(sample, args.robot_id, home)
            robot = _robot(sample, args.robot_id)
            if robot is not None:
                active = (
                    robot.get("nav_status") in {"active", "nav"}
                    or bool(robot.get("goal"))
                    or bool(robot.get("global_planned_path"))
                    or bool(robot.get("local_planned_path"))
                )
                if active:
                    report["objective_activity_observed"] = True
                status = robot.get("nav_status")
                if status in {"succeeded", "failed"} or (
                    status == "cancelled" and report["objective_activity_observed"]
                ):
                    break
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
        checkpoint("stop_all_written")
        if not report["commands"]["stop_all"]["sent"]:
            fatal = True
            report["errors"].append("Stop All command frame was not written")
        await asyncio.sleep(min(1.0, args.interval))
        report["post_stop"] = await collect_sample(
            args.base_url, args.timeout, time.monotonic() - started
        )
        checkpoint("post_stop_sampled")
        report["replica_manifests"] = await final_manifests(
            args.base_url,
            report["post_stop"]["replica_index"],
            args.timeout,
            args.mission_id or home["mission_id"],
            args.max_manifests,
        )
        report["summary"] = summarize_return_home(
            args.robot_id,
            home,
            report["baseline"],
            report["samples"],
            args.arrival_tolerance,
        )
        report["summary"]["post_stop_state"] = summarize_post_stop(
            report["post_stop"], [args.robot_id]
        )
        report["actual_duration_s"] = round(time.monotonic() - started, 3)
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        checkpoint("complete")
    return report, fatal


def parser() -> argparse.ArgumentParser:
    default_name = datetime.now(timezone.utc).strftime(
        "/tmp/swarmdeck-return-home-%Y%m%dT%H%M%SZ.json"
    )
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--base-url", default="http://127.0.0.1:18080")
    result.add_argument("--robot-id", required=True)
    result.add_argument("--mission-id")
    result.add_argument("--authority-file", required=True, type=Path)
    result.add_argument("--max-authority-age", type=float, default=30.0)
    result.add_argument("--duration", type=float, default=60.0)
    result.add_argument("--interval", type=float, default=0.5)
    result.add_argument("--timeout", type=float, default=2.0)
    result.add_argument("--arrival-tolerance", type=float, default=0.5)
    result.add_argument("--max-manifests", type=int, default=8)
    result.add_argument("--output", type=Path, default=Path(default_name))
    return result


def main() -> None:
    args = parser().parse_args()
    try:
        args.base_url = validate_base_url(args.base_url)
        if not args.robot_id or "/" in args.robot_id:
            raise ValueError("robot-id must be a nonempty ROS namespace segment")
        bounds = (
            ("duration", args.duration, 1.0, 300.0),
            ("interval", args.interval, 0.2, 10.0),
            ("timeout", args.timeout, 0.2, 10.0),
            ("arrival-tolerance", args.arrival_tolerance, 0.05, 2.0),
            ("max-authority-age", args.max_authority_age, 1.0, 120.0),
        )
        for name, value, low, high in bounds:
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be between {low:g} and {high:g}")
        if not 1 <= args.max_manifests <= 16:
            raise ValueError("max-manifests must be between 1 and 16")
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    report, fatal = asyncio.run(observe(args))
    print(json.dumps(report["summary"], indent=2, sort_keys=True))
    print(f"evidence: {args.output}")
    raise SystemExit(1 if fatal else 0)


if __name__ == "__main__":
    main()
