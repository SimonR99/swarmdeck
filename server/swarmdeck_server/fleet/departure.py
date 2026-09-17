"""Release a fleet-wide Explore one robot at a time.

Robots parked together cannot plan around each other: a neighbour that has not
moved is in nobody's map, so every first route may run through it. The server
knows every pose, so it lets the robot at the head of the group leave first
and releases the next once the previous one has actually cleared the start,
rather than after a fixed time that says nothing about where it got to.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Awaitable, Callable, Mapping

Pose = Mapping[str, float]


def departure_order(poses: Mapping[str, Pose]) -> list[str]:
    """Robots ranked from the head of the group to its tail.

    The head is whoever stands farthest along the group's mean heading: that
    robot has nobody in front of it. Robots without a usable pose leave last,
    in name order.
    """
    known = {
        robot: pose
        for robot, pose in poses.items()
        if all(
            isinstance(pose.get(key), (int, float)) and math.isfinite(pose[key])
            for key in ("x", "y", "yaw")
        )
    }
    ahead_x = sum(math.cos(pose["yaw"]) for pose in known.values())
    ahead_y = sum(math.sin(pose["yaw"]) for pose in known.values())
    norm = math.hypot(ahead_x, ahead_y)
    if norm < 1e-6:
        ahead_x, ahead_y, norm = 1.0, 0.0, 1.0
    ranked = sorted(
        known,
        key=lambda robot: (
            # Half-metre rows: spawn jitter must not reorder robots abreast.
            -round(
                2.0 * (known[robot]["x"] * ahead_x + known[robot]["y"] * ahead_y) / norm
            ),
            robot,
        ),
    )
    return ranked + sorted(robot for robot in poses if robot not in known)


def displacement_m(start: Pose | None, current: Pose | None) -> float:
    if not start or not current:
        return 0.0
    try:
        value = math.hypot(current["x"] - start["x"], current["y"] - start["y"])
    except (KeyError, TypeError):
        return 0.0
    return value if math.isfinite(value) else 0.0


async def release_in_turn(
    order: list[str],
    release: Callable[[str], Awaitable[bool]],
    pose_of: Callable[[str], Pose | None],
    *,
    clearance_m: float,
    timeout_s: float,
    poll_s: float = 0.25,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> list[str]:
    """Release each robot once the one before it has cleared the start.

    Returns the robots the release did not reach. A robot that was not reached,
    or that does not move, holds the next one back only until the timeout.
    Cancelling the task stops the sequence: robots not yet released stay put.
    """
    undelivered: list[str] = []
    for index, robot in enumerate(order):
        origin = dict(pose_of(robot) or {})
        delivered = await release(robot)
        if not delivered:
            undelivered.append(robot)
        if index + 1 == len(order) or not delivered:
            continue
        deadline = clock() + timeout_s
        while clock() < deadline:
            if displacement_m(origin, pose_of(robot)) >= clearance_m:
                break
            await sleep(poll_s)
    return undelivered
