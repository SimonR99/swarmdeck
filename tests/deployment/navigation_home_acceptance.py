#!/usr/bin/env python3
"""Bounded simulation acceptance for one Navigate or Return Home command."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import websockets


def json_request(base_url, path, body=None, timeout=5.0):
    request = Request(
        base_url + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=timeout) as response:
        return json.load(response)


def transform_navigation_goal(pose, distance, lateral, transform):
    """Create one component-frame goal from the displayed navigation pose."""
    yaw = float(pose["yaw"])
    navigation = (
        float(pose["x"]) + distance * math.cos(yaw) - lateral * math.sin(yaw),
        float(pose["y"]) + distance * math.sin(yaw) + lateral * math.cos(yaw),
        float(pose.get("z", 0.0)),
    )
    component = navigation_to_component(navigation, transform)
    heading = (
        float(transform[0][0]) * math.cos(yaw) + float(transform[0][1]) * math.sin(yaw),
        float(transform[1][0]) * math.cos(yaw) + float(transform[1][1]) * math.sin(yaw),
    )
    if math.hypot(*heading) < 1e-9:
        raise RuntimeError("component transform collapses the navigation heading")
    return navigation, {
        "x": component[0],
        "y": component[1],
        "z": component[2],
        "yaw": math.atan2(heading[1], heading[0]),
    }


def navigation_to_component(point, transform):
    return tuple(
        sum(float(transform[row][column]) * point[column] for column in range(3))
        + float(transform[row][3])
        for row in range(3)
    )


def authority_identity(mission, component, robot, *, include_home):
    home = robot.get("home") or {}
    return (
        mission,
        component,
        robot.get("navigation_frame"),
        home.get("keyframe_id") if include_home else None,
    )


def navigation_from_component(goal, transform):
    delta = [
        float(goal[key]) - float(transform[row][3])
        for row, key in enumerate(("x", "y", "z"))
    ]
    return tuple(
        sum(float(transform[row][column]) * delta[row] for row in range(3))
        for column in range(3)
    )


def path_endpoint_error(robot, target, field):
    path = robot.get(field) or []
    if not path:
        return None
    endpoint = path[-1]
    return math.hypot(
        float(endpoint["x"]) - target[0], float(endpoint["y"]) - target[1]
    )


def home_target(robot):
    home = (robot.get("home") or {}).get("T_navigation_home")
    if (
        not isinstance(home, list)
        or len(home) != 4
        or any(not isinstance(row, list) or len(row) != 4 for row in home)
        or not all(math.isfinite(float(value)) for row in home for value in row)
    ):
        raise RuntimeError("qualified live authority has no Home transform")
    return (float(home[0][3]), float(home[1][3]), float(home[2][3]))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation", action="store_true", required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--robot", default="robot_0")
    parser.add_argument("--distance", type=float, default=12.0)
    parser.add_argument("--lateral-offset", type=float, default=0.0)
    parser.add_argument("--mode", choices=("navigate", "home"), required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--poll", type=float, default=1.0)
    args = parser.parse_args()
    if not args.robot.startswith("robot_"):
        parser.error("--robot must use the simulation robot_ prefix")
    if not math.isfinite(args.distance) or not 0.1 <= args.distance <= 100.0:
        parser.error("--distance must be finite and between 0.1 and 100 metres")
    if not math.isfinite(args.lateral_offset) or abs(args.lateral_offset) > 100.0:
        parser.error("--lateral-offset must be finite and at most 100 metres")
    if not 5.0 <= args.duration <= 600.0 or not 0.1 <= args.poll <= 5.0:
        parser.error("duration must be 5..600 seconds and poll 0.1..5 seconds")
    args.base_url = args.base_url.rstrip("/")
    return args


def simulation_fleet(args):
    reset = json_request(args.base_url, "/api/sim/reset")
    if reset.get("version") != 1 or reset.get("phase") in {None, "legacy", "failed"}:
        raise RuntimeError("simulation reset supervisor is unavailable")
    fleet = json_request(args.base_url, "/api/fleet").get("robots", [])
    if not fleet or any(
        not robot.get("robot_id", "").startswith("robot_") for robot in fleet
    ):
        raise RuntimeError("fleet is not an all-simulation robot_ fleet")
    if args.robot not in {robot["robot_id"] for robot in fleet}:
        raise RuntimeError(f"simulation fleet has no {args.robot}")
    return fleet


def current_live(args, deadline):
    last_error = "qualified live component is unavailable"
    while time.monotonic() < deadline:
        catalogue = json_request(args.base_url, "/api/autonomy/replicas/components")
        mission = catalogue.get("active_session_id")
        for component in catalogue.get("components", []):
            if (
                component.get("session_id") != mission
                or component.get("available") is not True
                or args.robot not in component.get("robot_ids", [])
            ):
                continue
            query = urlencode({"component_id": component["component_id"]})
            try:
                live = json_request(
                    args.base_url,
                    f"/api/autonomy/replicas/components/live/{mission}?{query}",
                )
            except HTTPError as error:
                if error.code == 404:
                    last_error = "component has no fresh robot telemetry"
                    continue
                raise
            robot = next(
                (
                    item
                    for item in live.get("robots", [])
                    if item.get("robot_id") == args.robot
                ),
                None,
            )
            if robot is not None:
                return mission, component["component_id"], live["solution_order"], robot
        time.sleep(0.2)
    raise RuntimeError(last_error)


async def verify_stopped(args, robot_ids):
    for _ in range(20):
        await asyncio.sleep(0.5)
        fleet = (
            await asyncio.to_thread(json_request, args.base_url, "/api/fleet")
        ).get("robots", [])
        if {robot["robot_id"] for robot in fleet} == robot_ids and all(
            robot.get("online") is True
            and robot.get("nav_status") not in {"active", "nav"}
            and robot.get("exploration_status") not in {"exploring", "waiting"}
            for robot in fleet
        ):
            return True
    return False


async def run(args):
    fleet = simulation_fleet(args)
    robot_ids = {robot["robot_id"] for robot in fleet}
    ws_url = (
        args.base_url.replace("http://", "ws://", 1).replace("https://", "wss://", 1)
        + "/ws"
    )
    summary = {
        "mode": args.mode,
        "robot": args.robot,
        "outcome": "timeout",
        "active_samples": 0,
        "max_displacement_m": 0.0,
        "authority_changed": False,
        "solution_order_changes": 0,
        "local_endpoint_changes": 0,
        "observed_phases": [],
        "local_endpoint_error_m": None,
        "global_endpoint_error_m": None,
        "remaining_error_m": None,
    }
    async with websockets.connect(ws_url, max_size=16 * 1024 * 1024) as socket:
        reader = asyncio.create_task(_drain(socket))
        try:
            await socket.send(json.dumps({"type": "stop_all"}))
            if not await verify_stopped(args, robot_ids):
                raise RuntimeError("fleet was not idle before the trial")
            fleet = (
                await asyncio.to_thread(json_request, args.base_url, "/api/fleet")
            ).get("robots", [])
            mission, component, order, live_robot = await asyncio.to_thread(
                current_live, args, time.monotonic() + 10.0
            )
            initial_identity = authority_identity(
                mission,
                component,
                live_robot,
                include_home=args.mode == "home",
            )
            last_order = tuple(order)
            local_endpoint = None
            initial_pose = live_robot["pose"]
            initial_component_pose = navigation_to_component(
                (
                    float(initial_pose["x"]),
                    float(initial_pose["y"]),
                    float(initial_pose.get("z", 0.0)),
                ),
                live_robot["T_component_navigation"],
            )
            if args.mode == "navigate":
                target, component_goal = transform_navigation_goal(
                    initial_pose,
                    args.distance,
                    args.lateral_offset,
                    live_robot["T_component_navigation"],
                )
                await asyncio.to_thread(
                    json_request,
                    args.base_url,
                    f"/api/autonomy/replicas/components/live/{mission}/goal",
                    {
                        "robot_id": args.robot,
                        "component_id": component,
                        "solution_order": order,
                        "goal": component_goal,
                    },
                )
            else:
                target = home_target(live_robot)
                await socket.send(
                    json.dumps({"type": "return_home", "robot_id": args.robot})
                )

            baseline = next(robot for robot in fleet if robot["robot_id"] == args.robot)
            baseline_status = (
                baseline.get("nav_status"),
                baseline.get("nav_failure_reason"),
            )
            started = time.monotonic()
            trial_deadline = started + args.duration
            status_robot = baseline
            while time.monotonic() < trial_deadline:
                await asyncio.sleep(args.poll)
                fleet_now = (
                    await asyncio.to_thread(json_request, args.base_url, "/api/fleet")
                ).get("robots", [])
                status_robot = next(
                    (
                        robot
                        for robot in fleet_now
                        if robot.get("robot_id") == args.robot
                    ),
                    {},
                )
                try:
                    mission_now, component_now, order_now, robot_now = (
                        await asyncio.to_thread(
                            current_live,
                            args,
                            min(trial_deadline, time.monotonic() + 1.0),
                        )
                    )
                except (HTTPError, RuntimeError):
                    robot_now = None
                if robot_now is not None:
                    identity = authority_identity(
                        mission_now,
                        component_now,
                        robot_now,
                        include_home=args.mode == "home",
                    )
                    if identity != initial_identity:
                        summary["authority_changed"] = True
                    else:
                        current_order = tuple(order_now)
                        if current_order != last_order:
                            summary["solution_order_changes"] += 1
                            last_order = current_order
                        if args.mode == "navigate":
                            target = navigation_from_component(
                                component_goal, robot_now["T_component_navigation"]
                            )
                        else:
                            target = home_target(robot_now)
                        current_component_pose = navigation_to_component(
                            (
                                float(robot_now["pose"]["x"]),
                                float(robot_now["pose"]["y"]),
                                float(robot_now["pose"].get("z", 0.0)),
                            ),
                            robot_now["T_component_navigation"],
                        )
                        displacement = math.hypot(
                            current_component_pose[0] - initial_component_pose[0],
                            current_component_pose[1] - initial_component_pose[1],
                        )
                        summary["max_displacement_m"] = max(
                            summary["max_displacement_m"], displacement
                        )
                        for field, output in (
                            ("local_planned_path", "local_endpoint_error_m"),
                            ("global_planned_path", "global_endpoint_error_m"),
                        ):
                            endpoint_error = path_endpoint_error(
                                robot_now, target, field
                            )
                            if endpoint_error is not None:
                                summary[output] = endpoint_error
                        local_path = robot_now.get("local_planned_path") or []
                        if local_path:
                            latest_endpoint = (
                                round(float(local_path[-1]["x"]), 3),
                                round(float(local_path[-1]["y"]), 3),
                            )
                            if (
                                local_endpoint is not None
                                and latest_endpoint != local_endpoint
                            ):
                                summary["local_endpoint_changes"] += 1
                            local_endpoint = latest_endpoint
                        summary["remaining_error_m"] = math.hypot(
                            float(robot_now["pose"]["x"]) - target[0],
                            float(robot_now["pose"]["y"]) - target[1],
                        )
                status = status_robot.get("nav_status")
                phase = (status_robot.get("objective_continuation") or {}).get("phase")
                if phase in {"planning", "following_local", "following_final"}:
                    if phase not in summary["observed_phases"]:
                        summary["observed_phases"].append(phase)
                summary["active_samples"] += int(status == "active")
                failure = status == "failed" and (
                    summary["active_samples"] > 0
                    or (status, status_robot.get("nav_failure_reason"))
                    != baseline_status
                )
                if status == "succeeded" and summary["active_samples"] > 0:
                    summary["outcome"] = "succeeded"
                    break
                if failure:
                    summary["outcome"] = "failed"
                    summary["failure_reason"] = str(
                        status_robot.get("nav_failure_reason") or "navigation failed"
                    )[:512]
                    break
            summary["final_status"] = status_robot.get("nav_status")
            summary["elapsed_s"] = round(time.monotonic() - started, 1)
        finally:
            await socket.send(json.dumps({"type": "stop_all"}))
            summary["stop_all_verified"] = await verify_stopped(args, robot_ids)
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
    for key in (
        "max_displacement_m",
        "local_endpoint_error_m",
        "global_endpoint_error_m",
        "remaining_error_m",
    ):
        if summary.get(key) is not None:
            summary[key] = round(float(summary[key]), 3)
    print(json.dumps(summary, sort_keys=True))
    if not summary["stop_all_verified"]:
        raise RuntimeError("Stop All did not settle the simulation fleet")
    return summary["outcome"] == "succeeded" and not summary["authority_changed"]


async def _drain(socket):
    async for _ in socket:
        pass


if __name__ == "__main__":
    raise SystemExit(0 if asyncio.run(run(parse_args())) else 1)
