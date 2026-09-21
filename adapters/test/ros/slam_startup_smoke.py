#!/usr/bin/env python3
"""Native ROS smoke test for simulation SLAM startup recovery.

Run inside the simulation image after sourcing ROS:

    ROS_DOMAIN_ID=222 python3 /app/adapters/test/ros/slam_startup_smoke.py

The fixture creates two lifecycle services only under ``/swarmdeck_fixture``.
It does not publish, call a real robot namespace, or issue a motion command.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import MultiThreadedExecutor

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "adapters" / "adapter_sim"))
import adapter_sim  # noqa: E402
from slam_startup import SlamToolboxStartup  # noqa: E402


def main() -> int:
    rclpy.init()
    server = rclpy.create_node("swarmdeck_slam_startup_fixture_server")
    client_node = rclpy.create_node("swarmdeck_slam_startup_fixture_client")
    namespace = "/swarmdeck_fixture/peer_slam"
    state = [State.PRIMARY_STATE_UNCONFIGURED]
    transitions: list[int] = []

    def get_state(request, response):
        response.current_state.id = state[0]
        return response

    def change_state(request, response):
        transitions.append(request.transition.id)
        if (
            state[0] == State.PRIMARY_STATE_UNCONFIGURED
            and request.transition.id == Transition.TRANSITION_CONFIGURE
        ):
            state[0] = State.PRIMARY_STATE_INACTIVE
            response.success = True
        elif (
            state[0] == State.PRIMARY_STATE_INACTIVE
            and request.transition.id == Transition.TRANSITION_ACTIVATE
        ):
            state[0] = State.PRIMARY_STATE_ACTIVE
            response.success = True
        else:
            response.success = False
        return response

    server.create_service(GetState, f"{namespace}/get_state", get_state)
    server.create_service(ChangeState, f"{namespace}/change_state", change_state)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(client_node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    bridge = adapter_sim.RobotBridge.__new__(adapter_sim.RobotBridge)
    bridge.node = client_node
    bridge.id = "swarmdeck_fixture"
    bridge._service_clients = {}
    failures: list[str] = []
    recovery = SlamToolboxStartup(
        bridge._peer_slam_state,
        bridge._change_peer_slam_state,
        failures.append,
        deadline_s=8.0,
        backoff_s=0.05,
    )

    try:
        result = recovery.run_once()
        assert result is not None and result.ready, result
        assert transitions == [
            Transition.TRANSITION_CONFIGURE,
            Transition.TRANSITION_ACTIVATE,
        ], transitions
        assert state[0] == State.PRIMARY_STATE_ACTIVE
        assert failures == []

        transitions.clear()
        active_check = SlamToolboxStartup(
            bridge._peer_slam_state,
            bridge._change_peer_slam_state,
            failures.append,
            deadline_s=3.0,
        ).run_once()
        assert active_check is not None and active_check.ready
        assert transitions == [], "active state must not be mutated"

        state[0] = State.PRIMARY_STATE_INACTIVE
        inactive_check = SlamToolboxStartup(
            bridge._peer_slam_state,
            bridge._change_peer_slam_state,
            failures.append,
            deadline_s=3.0,
            backoff_s=0.05,
        ).run_once()
        assert inactive_check is not None and inactive_check.ready
        assert transitions == [Transition.TRANSITION_ACTIVATE], transitions
        print("PASS: unconfigured -> configure -> inactive -> activate -> active")
        print("PASS: active state produced no lifecycle mutation")
        print("PASS: inactive -> activate -> active")
        return 0
    finally:
        executor.shutdown(timeout_sec=2.0)
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
