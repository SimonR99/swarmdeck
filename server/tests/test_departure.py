"""A fleet-wide Explore releases parked robots one at a time, head first."""

import asyncio
import math

from swarmdeck_server.fleet.departure import (
    departure_order,
    displacement_m,
    release_in_turn,
)

SOUTH = -math.pi / 2
BISTRO = {
    "robot_0": {"x": -11.2, "y": 4.0, "yaw": SOUTH},
    "robot_1": {"x": -11.2, "y": 6.0, "yaw": SOUTH},
    "robot_2": {"x": -13.2, "y": 4.0, "yaw": SOUTH},
    "robot_3": {"x": -13.2, "y": 6.0, "yaw": SOUTH},
}


def test_the_front_row_leaves_before_the_back_row():
    # Everyone faces -y: the row at y = 4 stands in front of the row at y = 6.
    assert departure_order(BISTRO) == ["robot_0", "robot_2", "robot_1", "robot_3"]
    # Small spawn disturbances do not reshuffle a row.
    nudged = {**BISTRO, "robot_2": {"x": -13.2, "y": 4.004, "yaw": SOUTH - 0.1}}
    assert departure_order(nudged)[:2] == ["robot_0", "robot_2"]


def test_robots_without_a_pose_leave_last():
    poses = {**BISTRO, "robot_9": {}, "robot_8": {"x": float("nan"), "y": 0, "yaw": 0}}
    assert departure_order(poses)[-2:] == ["robot_8", "robot_9"]
    assert departure_order({}) == []


def test_displacement_tolerates_missing_poses():
    assert displacement_m(None, {"x": 1, "y": 1}) == 0.0
    assert displacement_m({"x": 0, "y": 0}, {"x": 3, "y": 4}) == 5.0


def run_release(order, moves, *, reached=lambda robot: True, timeout_s=30.0):
    """Drive release_in_turn on a fake clock. `moves` is metres per second."""
    now = [0.0]
    released: list[tuple[str, float]] = []
    start: dict[str, float] = {}

    async def release(robot):
        if reached(robot):
            released.append((robot, now[0]))
            start[robot] = now[0]
            return True
        return False

    def pose_of(robot):
        travelled = (
            (now[0] - start[robot]) * moves.get(robot, 0.0) if robot in start else 0.0
        )
        return {"x": travelled, "y": 0.0}

    async def sleep(seconds):
        now[0] += seconds

    undelivered = asyncio.run(
        release_in_turn(
            order,
            release,
            pose_of,
            clearance_m=5.0,
            timeout_s=timeout_s,
            poll_s=0.5,
            clock=lambda: now[0],
            sleep=sleep,
        )
    )
    return released, undelivered


def test_the_next_robot_waits_until_the_previous_one_has_cleared():
    released, undelivered = run_release(["a", "b", "c"], {"a": 0.5, "b": 1.0, "c": 1.0})
    assert undelivered == []
    assert released == [("a", 0.0), ("b", 10.0), ("c", 15.0)]


def test_a_robot_that_does_not_move_holds_the_rest_only_until_the_timeout():
    released, _ = run_release(["a", "b"], {"a": 0.0}, timeout_s=30.0)
    assert released == [("a", 0.0), ("b", 30.0)]


def test_an_unreached_robot_is_reported_and_does_not_delay_the_next():
    released, undelivered = run_release(
        ["a", "b"], {"b": 1.0}, reached=lambda robot: robot != "a"
    )
    assert undelivered == ["a"]
    assert released == [("b", 0.0)]


def test_cancelling_the_sequence_leaves_later_robots_parked():
    released: list[str] = []

    async def scenario():
        async def release(robot):
            released.append(robot)
            return True

        task = asyncio.create_task(
            release_in_turn(
                ["a", "b"],
                release,
                lambda robot: {"x": 0.0, "y": 0.0},
                clearance_m=5.0,
                timeout_s=30.0,
                poll_s=0.01,
            )
        )
        await asyncio.sleep(0.05)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(scenario())
    assert released == ["a"]
