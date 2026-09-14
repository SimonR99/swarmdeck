#!/usr/bin/env python3
"""Exercise bounded startup against real ROS services with a lost response."""
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import GetState, ChangeState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()
    node = rclpy.create_node("navigation_startup_fixture", namespace="startup_fixture")
    group = ReentrantCallbackGroup()
    states = {"controller": 1, "planner": 1}
    changes = []
    for name in states:
        def query(request, response, name=name):
            response.current_state.id = states[name]
            return response

        def change(request, response, name=name):
            transition = request.transition.id
            changes.append((name, transition))
            assert states[name] == (1 if transition == 1 else 2)
            states[name] = 2 if transition == 1 else 3
            if (name, transition) == ("controller", 1):
                # Side effect completes, but response exceeds client's deadline.
                time.sleep(11)
            response.success = True
            return response

        node.create_service(GetState, name + "/get_state", query, callback_group=group)
        node.create_service(ChangeState, name + "/change_state", change, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        root = Path(__file__).resolve().parents[3]
        script = root / "swarmdeck_ros/src/swarmdeck_nav/scripts/lifecycle_startup.py"
        result = subprocess.run([
            sys.executable, str(script), "--ros-args", "-r", "__ns:=/startup_fixture",
            "-p", "node_names:=[controller,planner]", "-p", "startup_timeout_s:=30.0",
        ], capture_output=True, text=True, timeout=40)
        assert result.returncode == 0, result.stdout + result.stderr
        assert list(states.values()) == [3, 3], states
        assert changes == [(name, t) for t in (1, 3) for name in states], changes
        print("PASS: lost response recovered through observed lifecycle state; no repeated transition")
    finally:
        executor.shutdown(timeout_sec=15)
        node.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=5)


if __name__ == "__main__":
    main()
