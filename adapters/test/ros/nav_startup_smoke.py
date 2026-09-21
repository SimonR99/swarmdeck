#!/usr/bin/env python3
"""Live lifecycle recovery, lost responses, and prompt shutdown while waiting."""

import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import GetState, ChangeState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


def launch_owner(script, namespace, names):
    return subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            f"__ns:=/{namespace}",
            "-p",
            f"node_names:=[{','.join(names)}]",
            "-p",
            "startup_timeout_s:=300.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_owner(process):
    if process.poll() is not None:
        output, _ = process.communicate()
        raise AssertionError(f"lifecycle owner exited before interruption: {output}")
    process.send_signal(signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
        raise AssertionError(f"lifecycle owner required SIGKILL after SIGINT: {output}")
    print(output, end="")
    assert process.returncode == 0, f"unclean lifecycle shutdown: {process.returncode}"


def main():
    rclpy.init()
    node = rclpy.create_node("navigation_startup_fixture", namespace="startup_fixture")
    group = ReentrantCallbackGroup()
    states = {"controller": 1, "planner": 1}
    changes = []
    services = {}
    hold_activation = False
    activation_entered = threading.Event()
    release_activation = threading.Event()
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
            if name == "controller" and transition == 3 and hold_activation:
                activation_entered.set()
                assert release_activation.wait(
                    10
                ), "fixture activation was not released"
            response.success = True
            return response

        services[name] = node.create_service(
            GetState, name + "/get_state", query, callback_group=group
        )
        node.create_service(
            ChangeState, name + "/change_state", change, callback_group=group
        )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    process = None
    try:
        root = Path(__file__).resolve().parents[3]
        script = root / "swarmdeck_ros/src/swarmdeck_nav/scripts/lifecycle_startup.py"
        process = launch_owner(script, "startup_fixture", states)
        client = node.create_client(Trigger, "navigation_startup/recover")
        assert client.wait_for_service(timeout_sec=10), "recovery owner did not start"

        def recover():
            future = client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            assert completed.wait(40), "bounded recovery did not answer"
            response = future.result()
            assert response.success, response.message

        # This may queue behind the initial startup. The owner must not race
        # that startup or repeat any transition whose response arrived late.
        recover()
        assert list(states.values()) == [3, 3], states
        assert changes == [(name, t) for t in (1, 3) for name in states], changes
        states["controller"] = 2
        recover()
        assert list(states.values()) == [3, 3], states
        assert changes == [
            ("controller", 1),
            ("planner", 1),
            ("controller", 3),
            ("planner", 3),
            ("controller", 3),
        ], changes

        # Interrupt while a transition side effect has happened but the reply
        # is pending. Shutdown must cancel the wait, not wait for its 10 s deadline.
        hold_activation = True
        states["controller"] = 2
        pending = client.call_async(Trigger.Request())
        assert activation_entered.wait(10), "recovery did not request activation"
        try:
            stop_owner(process)
        finally:
            release_activation.set()
        process = None
        client.remove_pending_request(pending)
        pending.cancel()
        hold_activation = False

        # A missing GetState service during a later recovery must not keep
        # re-entering bringup's ordinary timeout/retry loop after ROS shutdown.
        # Use a new service identity: discovery may still advertise the stopped
        # owner's endpoint, and a request sent to it before rediscovery is lost.
        process = launch_owner(
            script,
            "recovery_missing",
            ["/startup_fixture/controller", "/startup_fixture/planner"],
        )
        client = node.create_client(
            Trigger, "/recovery_missing/navigation_startup/recover"
        )
        assert client.wait_for_service(timeout_sec=10), "recovery owner did not restart"
        recover()
        node.destroy_service(services["controller"])
        pending = client.call_async(Trigger.Request())
        time.sleep(0.5)
        assert not pending.done(), "recovery unexpectedly completed without GetState"
        stop_owner(process)
        process = None
        client.remove_pending_request(pending)
        pending.cancel()

        # The same interruption rule applies during the initial startup.
        process = launch_owner(script, "startup_missing", ["missing_controller"])
        missing_client = node.create_client(
            Trigger, "/startup_missing/navigation_startup/recover"
        )
        assert missing_client.wait_for_service(
            timeout_sec=10
        ), "missing-node owner did not start"
        time.sleep(0.5)
        stop_owner(process)
        process = None
        print(
            "PASS: lost reply and inactive-node recovery; prompt SIGINT during reply, discovery and startup waits"
        )
    finally:
        release_activation.set()
        try:
            if process is not None and process.poll() is None:
                stop_owner(process)
        finally:
            executor.shutdown(timeout_sec=15)
            thread.join(timeout=5)
            assert not thread.is_alive(), "fixture executor did not stop"
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
