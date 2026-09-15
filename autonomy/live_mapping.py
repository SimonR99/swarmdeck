"""Bounded navigation-frame telemetry accompanying durable map replicas.

Geometry and live state have different lifetimes: chunk revisions never prove
that a robot pose is current. Ages here are monotonic durations, not ROS time.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from .contracts import validate_se3

LIVE_MAPPING_MAX_AGE_S = 3.0
PATH_FIELDS = ("planned_path", "global_planned_path", "local_planned_path")
MAX_DISPLAY_PATH_POINTS = 200


def display_path(path):
    """Bound telemetry across the whole route, retaining both endpoints.

    This is display geometry only; the controller must receive the full path.
    """
    if not isinstance(path, (list, tuple)):
        raise ValueError("display path must be a sequence")
    if len(path) <= MAX_DISPLAY_PATH_POINTS:
        return list(path)
    last = len(path) - 1
    return [
        path[i * last // (MAX_DISPLAY_PATH_POINTS - 1)]
        for i in range(MAX_DISPLAY_PATH_POINTS)
    ]


def point(value, *, heading=False):
    if not isinstance(value, Mapping):
        raise ValueError("point must be an object")
    result = {key: float(value[key]) for key in ("x", "y")}
    result["z"] = float(value.get("z", 0.0))
    if heading:
        result["yaw"] = float(value.get("yaw", 0.0))
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError("point must be finite")
    return result


def solution_order(value):
    """Validate the shared component-frame revision, including its initial sentinel."""
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
        or any(type(item) is not int for item in value)
    ):
        raise ValueError("invalid mapping solution order")
    clock, optimizer = value
    if (
        clock < 0
        or optimizer < -1
        or clock > 2**53 - 1
        or optimizer > 2**53 - 1
        or (optimizer == -1 and clock != 0)
    ):
        raise ValueError("invalid mapping solution order")
    return [clock, optimizer]


def validate_live_mapping(value, robot_id):
    """Copy only the protocol fields; reject malformed or already stale data."""
    if not isinstance(value, Mapping) or value.get("robot_id") != robot_id:
        raise ValueError("live robot identity mismatch")
    result = {"robot_id": robot_id}
    for field in ("mission_id", "component_id", "navigation_frame"):
        item = value.get(field)
        if not isinstance(item, str) or not item or len(item) > 512:
            raise ValueError(f"invalid {field}")
        result[field] = item
    result["solution_order"] = solution_order(value["solution_order"])
    result["T_component_navigation"] = validate_se3(value["T_component_navigation"])
    home = value.get("home")
    if home is not None:
        if not isinstance(home, Mapping):
            raise ValueError("invalid Home authority")
        keyframe_id = home.get("keyframe_id")
        if (
            not isinstance(keyframe_id, str)
            or not keyframe_id
            or len(keyframe_id) > 512
        ):
            raise ValueError("invalid Home keyframe identity")
        prefix = f"{robot_id}/{result['mission_id']}/"
        sequence = keyframe_id.removeprefix(prefix)
        if (
            not keyframe_id.startswith(prefix)
            or not sequence.isdecimal()
            or str(int(sequence)) != sequence
        ):
            raise ValueError("Home keyframe identity differs from live authority")
        result["home"] = {
            "keyframe_id": keyframe_id,
            "T_navigation_home": validate_se3(home.get("T_navigation_home")),
        }
    age = float(value["authority_age_s"])
    if not math.isfinite(age) or not 0 <= age <= LIVE_MAPPING_MAX_AGE_S:
        raise ValueError("stale mapping authority")
    result["authority_age_s"] = age
    result["pose"] = point(value["pose"], heading=True)
    result["goal"] = point(value["goal"], heading=True) if value.get("goal") else None
    for field in PATH_FIELDS:
        path = value.get(field, [])
        if not isinstance(path, (list, tuple)) or len(path) > MAX_DISPLAY_PATH_POINTS:
            raise ValueError("invalid or oversized live path")
        result[field] = [point(p) for p in path]
    return result


def navigation_goal(component_goal, transform):
    """Invert a qualified SE(3) once, including the projected heading."""
    goal = point(component_goal, heading=True)
    matrix = validate_se3(transform)
    delta = [goal[k] - matrix[i][3] for i, k in enumerate(("x", "y", "z"))]
    xyz = [sum(matrix[j][i] * delta[j] for j in range(3)) for i in range(3)]
    direction = (math.cos(goal["yaw"]), math.sin(goal["yaw"]), 0.0)
    heading = [sum(matrix[j][i] * direction[j] for j in range(3)) for i in range(2)]
    if math.hypot(*heading) < 1e-6:
        raise ValueError("heading has no navigation-plane projection")
    return dict(zip(("x", "y", "z"), xyz), yaw=math.atan2(heading[1], heading[0]))
