#!/usr/bin/env python3
"""One bounded lifecycle owner for simulation Nav2 startup and recovery.

A lost transition response is resolved by observing the node's actual state.
The same owner remains available at ~/recover after initial startup; never
retry an assumed transition or reset an already active node.
"""

from __future__ import annotations

import signal
import time
import threading


class BringupCancelled(Exception):
    """The lifecycle owner is stopping; no more transitions may be issued."""


def bringup(
    nodes,
    query,
    change,
    *,
    deadline_s=60.0,
    clock=time.monotonic,
    sleep=time.sleep,
    cancelled=None,
):
    """Configure every node, then activate in dependency order; fail boundedly."""

    def check_cancelled():
        if cancelled is not None and cancelled():
            raise BringupCancelled()

    deadline = clock() + deadline_s
    detail = "no lifecycle state received"
    for target, transition in ((2, 1), (3, 3)):
        for name in nodes:
            while clock() < deadline:
                check_cancelled()
                try:
                    state = query(name, min(deadline, clock() + 2.0))
                    check_cancelled()
                    if clock() >= deadline:
                        break
                    if state == 3 or (target == 2 and state == 2):
                        break
                    if state == (1 if target == 2 else 2):
                        accepted = change(
                            name, transition, min(deadline, clock() + 10.0)
                        )
                        check_cancelled()
                        detail = f"{name}: transition {transition} {'sent' if accepted else 'rejected'}"
                    else:
                        detail = (
                            f"{name}: waiting for lifecycle state {state} to settle"
                        )
                except (TimeoutError, RuntimeError) as exc:
                    detail = f"{name}: {exc}"
                check_cancelled()
                remaining = deadline - clock()
                if remaining > 0:
                    sleep(min(0.25, remaining))
            else:
                raise TimeoutError(detail)
            if clock() >= deadline:
                raise TimeoutError(detail)
    # Activation can take time. Confirm the entire group before reporting ready.
    for name in nodes:
        check_cancelled()
        if (
            clock() >= deadline
            or query(name, min(deadline, clock() + 2.0)) != 3
            or clock() >= deadline
        ):
            raise RuntimeError(f"{name}: active state not confirmed")
        check_cancelled()


def main():
    import rclpy
    from lifecycle_msgs.srv import GetState, ChangeState
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.task import Future
    from collections import deque
    from std_srvs.srv import Trigger

    # Stop callbacks before destroying their ROS handles. rclpy's default
    # signal handler shuts the context down first, racing an in-flight recovery
    # response with destruction even when its worker observes cancellation.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = rclpy.create_node("navigation_startup")
    stopping = threading.Event()
    node.context.on_shutdown(stopping.set)
    names = node.declare_parameter("node_names", ["controller_server"]).value
    deadline_s = float(node.declare_parameter("startup_timeout_s", 60.0).value)
    if not names or not 0 < deadline_s <= 300:
        node.destroy_node()
        rclpy.try_shutdown()
        raise ValueError("invalid navigation startup parameters")
    clients = {}
    # A service coroutine queues work, rather than blocking its executor.
    # The main thread alone runs bringup and spins client responses; there is
    # no worker pool whose callbacks can outlive node destruction.
    client_group = MutuallyExclusiveCallbackGroup()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    queued = deque()
    requests = {}

    def check_running():
        if stopping.is_set() or not node.context.ok():
            raise BringupCancelled()

    def call(name, kind, request, deadline):
        key = (name, kind)
        if key not in clients:
            clients[key] = node.create_client(kind, name, callback_group=client_group)
        client = clients[key]
        while not client.service_is_ready():
            check_running()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"service unavailable: {name}")
            executor.spin_once(timeout_sec=min(0.1, remaining))
        check_running()
        if time.monotonic() >= deadline:
            raise TimeoutError(f"service discovery exceeded deadline: {name}")
        future = client.call_async(request)
        try:
            while not future.done():
                check_running()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"service response timed out: {name}")
                executor.spin_once(timeout_sec=min(0.1, remaining))
            check_running()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"service response timed out: {name}")
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()
        if future.exception() is not None:
            raise RuntimeError(f"service failed: {name}: {future.exception()}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service returned no response: {name}")
        return result

    def query(name, deadline):
        return call(
            name + "/get_state", GetState, GetState.Request(), deadline
        ).current_state.id

    def change(name, transition, deadline):
        request = ChangeState.Request()
        request.transition.id = transition
        return call(name + "/change_state", ChangeState, request, deadline).success

    def run_bringup(response):
        try:
            bringup(
                names,
                query,
                change,
                deadline_s=deadline_s,
                cancelled=stopping.is_set,
            )
            response.success = True
            response.message = "Navigation lifecycle nodes confirmed active"
            node.get_logger().info(response.message)
        except BringupCancelled:
            response.success = False
            response.message = "Navigation lifecycle owner is shutting down"
        except (TimeoutError, RuntimeError) as exc:
            response.success = False
            response.message = f"Navigation startup failed: {exc}"
            node.get_logger().error(response.message)
        return response

    async def recover(_request, response):
        if stopping.is_set():
            response.success = False
            response.message = "Navigation lifecycle owner is shutting down"
            return response
        reply = Future(executor=executor)
        requests[reply] = response
        queued.append(reply)
        try:
            return await reply
        finally:
            del requests[reply]

    node.create_service(Trigger, "~/recover", recover)
    try:
        run_bringup(Trigger.Response())
        while not stopping.is_set():
            if queued:
                reply = queued[0]
                response = run_bringup(requests[reply])
                queued.popleft()
                reply.set_result(response)
            else:
                # Finite native waits let Python handle SIGINT/SIGTERM before
                # shutting down the ROS context.
                executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopping.set()
        # Complete accepted service callbacks while their handles remain live.
        for reply, response in list(requests.items()):
            response.success = False
            response.message = "Navigation lifecycle owner is shutting down"
            reply.set_result(response)
        while requests and node.context.ok():
            executor.spin_once(timeout_sec=0.1)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
