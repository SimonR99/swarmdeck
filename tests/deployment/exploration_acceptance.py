#!/usr/bin/env python3
"""Bounded, simulation-only end-to-end acceptance for fleet Explore.

The harness mirrors the dashboard's fleet Explore action, samples the
component-qualified navigation telemetry, and requires every simulated robot to
receive a path and move in its stable navigation frame.  It always sends Stop
All and verifies the fleet is idle before returning.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import websockets

ROBOT_IDS = frozenset(f"robot_{index}" for index in range(4))
ACTIVE_EXPLORATION = frozenset({"starting", "exploring", "waiting"})
PATH_FIELDS = ("local_planned_path", "global_planned_path", "planned_path")


class AuthorityChanged(RuntimeError):
    """The active mission, component, or navigation frame was replaced."""


def json_request(base_url, path, body=None, timeout=5.0):
    request = Request(
        base_url + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={"Accept": "application/json", "Content-Type": "application/json"},
    )
    with urlopen(request, timeout=timeout) as response:
        return json.load(response)


def websocket_url(base_url):
    return (
        base_url.replace("http://", "ws://", 1)
        .replace("https://", "wss://", 1)
        .rstrip("/")
        + "/ws"
    )


def simulation_fleet(args):
    """Require exactly the four simulation robots and an available reset supervisor."""
    reset = json_request(args.base_url, "/api/sim/reset")
    if (
        reset.get("supervisor_available") is not True
        or reset.get("version") != 1
        or reset.get("phase")
        in {
            None,
            "legacy",
            "failed",
            "accepted",
            "stopping",
            "starting",
            "verifying",
        }
    ):
        raise RuntimeError("simulation reset supervisor is unavailable")
    fleet = json_request(args.base_url, "/api/fleet").get("robots", [])
    ids = {robot.get("robot_id") for robot in fleet}
    if (
        len(fleet) != len(ROBOT_IDS)
        or ids != ROBOT_IDS
        or any(
            not isinstance(robot.get("robot_id"), str)
            or not robot["robot_id"].startswith("robot_")
            or robot.get("online") is not True
            or "explore" not in (robot.get("capabilities") or [])
            for robot in fleet
        )
    ):
        raise RuntimeError(
            "acceptance requires four online simulation robots robot_0..robot_3 "
            "with Explore capability"
        )
    return fleet


def idle_robot(robot):
    """Return true only when the server reports no active command or path."""
    return (
        robot.get("online") is True
        and robot.get("mode") not in {"explore", "nav", "recover", "teleop"}
        and robot.get("nav_status") not in {"active", "nav"}
        and robot.get("exploration_status") not in ACTIVE_EXPLORATION
        and robot.get("goal") is None
        and not any(robot.get(field) for field in PATH_FIELDS)
    )


async def verify_idle(args, robot_ids, *, attempts=20, delay=0.5):
    for _ in range(attempts):
        try:
            fleet = await asyncio.to_thread(
                json_request, args.base_url, "/api/fleet", None, 1.0
            )
        except (HTTPError, URLError, TimeoutError, OSError):
            await asyncio.sleep(delay)
            continue
        robots = {robot.get("robot_id"): robot for robot in fleet.get("robots", [])}
        if set(robots) == set(robot_ids) and all(
            idle_robot(robots[robot_id]) for robot_id in robot_ids
        ):
            return True
        await asyncio.sleep(delay)
    return False


def authority_identity(mission, component, robot):
    """Identity whose replacement invalidates motion evidence.

    ``solution_order`` and graph revisions are deliberately absent: those can
    advance while the same mission/component/navigation frame remains active.
    """
    return (
        mission,
        component,
        robot.get("navigation_frame"),
    )


def navigation_pose(robot):
    """Read the pose expressed in the declared stable navigation frame."""
    pose = robot.get("pose")
    if not isinstance(pose, dict):
        raise ValueError("qualified live robot has no navigation pose")
    try:
        x, y = float(pose["x"]), float(pose["y"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "qualified live robot has an invalid navigation pose"
        ) from error
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("qualified live robot has a non-finite navigation pose")
    return x, y


def path_points(robot):
    """Validate the first currently published route and return its point count."""
    selected = None
    for field in PATH_FIELDS:
        value = robot.get(field) or []
        if value:
            selected = value
            break
    if selected is None:
        return 0
    if not isinstance(selected, (list, tuple)):
        raise ValueError("qualified live robot has an invalid path")
    for point in selected:
        if not isinstance(point, dict):
            raise ValueError("qualified live robot has an invalid path point")
        try:
            x, y = float(point["x"]), float(point["y"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "qualified live robot has an invalid path point"
            ) from error
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("qualified live robot has a non-finite path point")
    return len(selected)


def new_robot_evidence(robot_id):
    return {
        "robot_id": robot_id,
        "authority": None,
        "authority_changed": False,
        "samples": 0,
        "exploration_statuses": [],
        "last_exploration_status": None,
        "last_navigation_status": None,
        "last_navigation_failure": None,
        "ever_executing": False,
        "path_samples": 0,
        "max_path_points": 0,
        "max_displacement_m": 0.0,
        "initial_navigation_pose": None,
    }


def record_robot_sample(evidence, live, status_robot, mission, component):
    """Record one qualified sample, returning a reason if it is unusable."""
    if (
        not isinstance(live.get("navigation_frame"), str)
        or not live["navigation_frame"].strip()
    ):
        return "qualified live robot has no stable navigation frame"
    identity = authority_identity(mission, component, live)
    if evidence["authority"] is None:
        evidence["authority"] = identity
    elif evidence["authority"] != identity:
        evidence["authority_changed"] = True
        return "mission/component/navigation authority changed"

    try:
        pose = navigation_pose(live)
        point_count = path_points(live)
    except ValueError as error:
        return str(error)

    evidence["samples"] += 1
    if evidence["initial_navigation_pose"] is None:
        evidence["initial_navigation_pose"] = pose
    evidence["max_displacement_m"] = max(
        evidence["max_displacement_m"],
        math.hypot(
            pose[0] - evidence["initial_navigation_pose"][0],
            pose[1] - evidence["initial_navigation_pose"][1],
        ),
    )
    status = status_robot.get("exploration_status")
    evidence["last_exploration_status"] = status
    evidence["last_navigation_status"] = status_robot.get("nav_status")
    reason = status_robot.get("nav_failure_reason")
    if isinstance(reason, str) and reason.strip():
        evidence["last_navigation_failure"] = reason[:512]
    if status and status not in evidence["exploration_statuses"]:
        evidence["exploration_statuses"].append(status)
    evidence["ever_executing"] |= status == "exploring" or (
        status in ACTIVE_EXPLORATION and status_robot.get("nav_status") == "active"
    )
    if point_count:
        evidence["path_samples"] += 1
        evidence["max_path_points"] = max(evidence["max_path_points"], point_count)
    return None


def assess_robot(evidence, min_displacement_m):
    """Assess one robot without treating a pose-authority correction as motion."""
    reasons = []
    if evidence["authority_changed"]:
        reasons.append("mission/component/navigation authority changed")
    if evidence["samples"] == 0:
        reasons.append("no qualified navigation samples")
    if evidence["path_samples"] == 0:
        reasons.append("no waypoint/path observation")
    if evidence["max_displacement_m"] < min_displacement_m:
        reasons.append(f"no XY motion (max {evidence['max_displacement_m']:.3f} m)")
    if not evidence["ever_executing"]:
        reasons.append("Explore never reported an executing state")
    return {
        "robot_id": evidence["robot_id"],
        "passed": not reasons,
        "reasons": reasons,
        **{key: value for key, value in evidence.items() if not key.startswith("_")},
    }


def _catalogue_live(args, robot_ids, deadline):
    """Fetch one coherent component-qualified live snapshot."""
    catalogue_path = "/api/autonomy/replicas/components"
    catalogue = json_request(
        args.base_url,
        catalogue_path,
        timeout=max(0.2, min(2.0, deadline - time.monotonic())),
    )
    mission = catalogue.get("active_session_id")
    if not isinstance(mission, str) or not mission:
        raise RuntimeError("no active simulation mission")
    components = sorted(
        (
            component
            for component in (catalogue.get("components") or [])
            if component.get("session_id") == mission
            and component.get("available") is True
        ),
        key=lambda component: str(component.get("component_id", "")),
    )
    component_by_robot = {}
    by_id = {}
    for component_entry in components:
        component = component_entry.get("component_id")
        members = set(component_entry.get("robot_ids") or []) & set(robot_ids)
        members -= set(component_by_robot)
        if not members:
            continue
        live = _fetch_live_component(args, mission, component, deadline)
        live_by_id = {robot.get("robot_id"): robot for robot in live.get("robots", [])}
        for robot_id in members & set(live_by_id):
            component_by_robot[robot_id] = component
            by_id[robot_id] = live_by_id[robot_id]
    if set(by_id) != set(robot_ids):
        raise RuntimeError("qualified live authority is missing a simulation robot")
    return mission, component_by_robot, by_id


def _fetch_live_component(args, mission, component, deadline):
    live = json_request(
        args.base_url,
        f"/api/autonomy/replicas/components/live/{mission}?"
        + urlencode({"component_id": component}),
        timeout=max(0.2, min(2.0, deadline - time.monotonic())),
    )
    if live.get("mission_id") != mission or live.get("component_id") != component:
        raise AuthorityChanged("qualified live authority envelope changed")
    for robot in live.get("robots", []):
        if robot.get("mission_id") != mission or robot.get("component_id") != component:
            raise AuthorityChanged("qualified robot authority envelope changed")
    return live


def _live_component(args, robot_ids, mission, components, deadline):
    """Read the already-selected component without rebuilding the catalogue."""
    by_id = {}
    for component in sorted(set(components.values())):
        live = _fetch_live_component(args, mission, component, deadline)
        by_id.update(
            {
                robot.get("robot_id"): robot
                for robot in live.get("robots", [])
                if components.get(robot.get("robot_id")) == component
            }
        )
    if set(by_id) != set(robot_ids):
        raise RuntimeError("qualified live authority is missing a simulation robot")
    return mission, components, by_id


async def qualified_live(args, robot_ids, deadline, expected=None):
    last_error = "qualified live authority is unavailable"
    while time.monotonic() < deadline:
        try:
            if expected is None:
                return await asyncio.to_thread(
                    _catalogue_live, args, robot_ids, deadline
                )
            return await asyncio.to_thread(
                _live_component, args, robot_ids, expected[0], expected[1], deadline
            )
        except AuthorityChanged:
            raise
        except HTTPError as error:
            if expected is not None and error.code == 409:
                raise AuthorityChanged("active mission/component authority changed")
            last_error = f"HTTP {error.code}"
            await asyncio.sleep(0.2)
        except (URLError, TimeoutError, OSError, RuntimeError) as error:
            last_error = str(error) or type(error).__name__
            await asyncio.sleep(0.2)
    raise RuntimeError(last_error)


async def _drain(socket):
    async for _ in socket:
        pass


async def run(args):
    summary = {
        "outcome": "error",
        "requested_duration_s": args.duration,
        "min_displacement_m": args.min_displacement,
        "robot_ids": sorted(ROBOT_IDS),
        "robots": {},
        "authority_changed": False,
        "observation_errors": 0,
        "stop_all_verified": False,
    }
    socket = None
    reader = None
    robot_ids = set(ROBOT_IDS)
    try:
        fleet = simulation_fleet(args)
        robot_ids = {robot["robot_id"] for robot in fleet}
        summary["robots"] = {
            robot_id: new_robot_evidence(robot_id) for robot_id in sorted(robot_ids)
        }
        socket = await websockets.connect(
            websocket_url(args.base_url), max_size=16 * 1024 * 1024
        )
        reader = asyncio.create_task(_drain(socket))
        await socket.send(json.dumps({"type": "stop_all"}))
        if not await verify_idle(args, robot_ids):
            raise RuntimeError("fleet was not idle before the Explore trial")

        baseline_fleet = await asyncio.to_thread(
            json_request, args.base_url, "/api/fleet", None, 1.0
        )
        baseline_by_id = {
            robot["robot_id"]: robot for robot in baseline_fleet.get("robots", [])
        }
        if set(baseline_by_id) != robot_ids or not all(
            idle_robot(baseline_by_id[robot_id]) for robot_id in robot_ids
        ):
            raise RuntimeError("fleet became active before the Explore trial")
        mission, components, live = await qualified_live(
            args, robot_ids, time.monotonic() + args.authority_timeout
        )
        summary["mission_id"] = mission
        summary["components"] = components
        expected = (mission, components)
        baseline_errors = []
        for robot_id in sorted(robot_ids):
            error = record_robot_sample(
                summary["robots"][robot_id],
                live[robot_id],
                baseline_by_id[robot_id],
                mission,
                components[robot_id],
            )
            if error:
                baseline_errors.append(f"{robot_id}: {error}")
            elif summary["robots"][robot_id]["path_samples"]:
                baseline_errors.append(f"{robot_id}: baseline already has a live path")
        if baseline_errors:
            raise RuntimeError(
                "invalid qualified baseline before Explore: "
                + "; ".join(baseline_errors)
            )

        # This is the exact sequence emitted by ui/src/lib/api/connection.ts.
        for robot_id in sorted(robot_ids):
            await socket.send(
                json.dumps({"type": "start_explore", "robot_id": robot_id})
            )

        started = time.monotonic()
        deadline = started + args.duration
        while time.monotonic() < deadline:
            await asyncio.sleep(min(args.poll, max(0.0, deadline - time.monotonic())))
            if time.monotonic() >= deadline:
                break
            try:
                fleet_now = await asyncio.to_thread(
                    json_request, args.base_url, "/api/fleet", None, 1.0
                )
                status_by_id = {
                    robot.get("robot_id"): robot
                    for robot in fleet_now.get("robots", [])
                }
                if set(status_by_id) != robot_ids:
                    raise RuntimeError("simulation fleet changed during the trial")
                mission_now, components_now, live_now = await qualified_live(
                    args,
                    robot_ids,
                    min(deadline, time.monotonic() + args.sample_timeout),
                    expected,
                )
                for robot_id in sorted(robot_ids):
                    error = record_robot_sample(
                        summary["robots"][robot_id],
                        live_now[robot_id],
                        status_by_id[robot_id],
                        mission_now,
                        components_now[robot_id],
                    )
                    if error:
                        summary["observation_errors"] += 1
            except AuthorityChanged:
                summary["authority_changed"] = True
                raise
            except (HTTPError, URLError, TimeoutError, OSError, RuntimeError) as error:
                summary["observation_errors"] += 1
                summary["last_observation_error"] = str(error)[:512]
        summary["elapsed_s"] = round(time.monotonic() - started, 1)
        summary["outcome"] = "succeeded"
    except Exception as error:
        summary["outcome"] = "failed"
        summary["failure_reason"] = f"{type(error).__name__}: {error}"[:512]
    finally:
        if socket is not None:
            try:
                await socket.send(json.dumps({"type": "stop_all"}))
                summary["stop_all_verified"] = await verify_idle(args, robot_ids)
            except Exception as error:
                summary["stop_all_verified"] = False
                summary.setdefault(
                    "failure_reason", f"Stop All verification: {error}"[:512]
                )
        if reader is not None:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
        if socket is not None:
            try:
                await socket.close()
            except Exception as error:
                summary.setdefault("failure_reason", f"WebSocket close: {error}"[:512])

    assessments = {
        robot_id: assess_robot(value, args.min_displacement)
        for robot_id, value in summary["robots"].items()
    }
    summary["robots"] = assessments
    summary["authority_changed"] |= any(
        value["authority_changed"] for value in assessments.values()
    )
    failed = [
        robot_id for robot_id, value in assessments.items() if not value["passed"]
    ]
    if summary["outcome"] == "succeeded" and failed:
        summary["outcome"] = "failed"
        summary["failure_reason"] = "; ".join(
            f"{robot_id}: {', '.join(assessments[robot_id]['reasons'])}"
            for robot_id in failed
        )[:512]
    if not summary["stop_all_verified"]:
        summary["outcome"] = "failed"
        summary.setdefault("failure_reason", "Stop All did not verify an idle fleet")
    print(json.dumps(summary, sort_keys=True))
    return summary["outcome"] == "succeeded"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation", action="store_true", required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--poll", type=float, default=1.0)
    parser.add_argument("--authority-timeout", type=float, default=20.0)
    parser.add_argument("--sample-timeout", type=float, default=3.0)
    parser.add_argument("--min-displacement", type=float, default=5.0)
    args = parser.parse_args()
    if not args.simulation:
        parser.error("--simulation is required")
    if not 5.0 <= args.duration <= 600.0:
        parser.error("--duration must be between 5 and 600 seconds")
    if not 0.1 <= args.poll <= 5.0:
        parser.error("--poll must be between 0.1 and 5 seconds")
    if not 2.0 <= args.authority_timeout <= 60.0:
        parser.error("--authority-timeout must be between 2 and 60 seconds")
    if not 0.5 <= args.sample_timeout <= 10.0:
        parser.error("--sample-timeout must be between 0.5 and 10 seconds")
    if (
        not math.isfinite(args.min_displacement)
        or not 0.01 <= args.min_displacement <= 5.0
    ):
        parser.error("--min-displacement must be finite and between 0.01 and 5 metres")
    args.base_url = args.base_url.rstrip("/")
    return args


def main():
    args = parse_args()
    if not args.simulation:
        raise SystemExit(2)
    try:
        passed = asyncio.run(run(args))
    except Exception as error:
        print(
            json.dumps(
                {
                    "outcome": "failed",
                    "failure_reason": f"{type(error).__name__}: {error}"[:512],
                    "stop_all_verified": False,
                },
                sort_keys=True,
            )
        )
        passed = False
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
