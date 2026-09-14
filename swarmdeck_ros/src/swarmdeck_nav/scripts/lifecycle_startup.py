#!/usr/bin/env python3
"""One bounded lifecycle owner for simulation Nav2 startup.

A lost transition response is resolved by observing the node's actual state.
Never retry an assumed transition or reset an already active node.
"""
from __future__ import annotations

import time


def bringup(nodes, query, change, *, deadline_s=60.0, clock=time.monotonic,
            sleep=time.sleep):
    """Configure every node, then activate in dependency order; fail boundedly."""
    deadline = clock() + deadline_s
    detail = "no lifecycle state received"
    for target, transition in ((2, 1), (3, 3)):
        for name in nodes:
            while clock() < deadline:
                try:
                    state = query(name, min(deadline, clock() + 2.0))
                    if clock() >= deadline:
                        break
                    if state == 3 or (target == 2 and state == 2):
                        break
                    if state == (1 if target == 2 else 2):
                        accepted = change(name, transition, min(deadline, clock() + 10.0))
                        detail = f"{name}: transition {transition} {'sent' if accepted else 'rejected'}"
                    else:
                        detail = f"{name}: waiting for lifecycle state {state} to settle"
                except (TimeoutError, RuntimeError) as exc:
                    detail = f"{name}: {exc}"
                remaining = deadline - clock()
                if remaining > 0:
                    sleep(min(.25, remaining))
            else:
                raise TimeoutError(detail)
            if clock() >= deadline:
                raise TimeoutError(detail)
    # Activation can take time. Confirm the entire group before reporting ready.
    for name in nodes:
        if (clock() >= deadline
                or query(name, min(deadline, clock() + 2.0)) != 3
                or clock() >= deadline):
            raise RuntimeError(f"{name}: active state not confirmed")


def main():
    import rclpy
    from lifecycle_msgs.srv import GetState, ChangeState

    rclpy.init()
    node = rclpy.create_node("navigation_startup")
    names = node.declare_parameter("node_names", ["controller_server"]).value
    deadline_s = float(node.declare_parameter("startup_timeout_s", 60.0).value)
    clients = {}

    def call(name, kind, request, deadline):
        key = (name, kind)
        if key not in clients:
            clients[key] = node.create_client(kind, name)
        client = clients[key]
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError(f"service unavailable: {name}")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"service discovery exceeded deadline: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        if not future.done() or time.monotonic() >= deadline:
            client.remove_pending_request(future)
            future.cancel()
            raise TimeoutError(f"service response timed out: {name}")
        if future.exception() is not None:
            raise RuntimeError(f"service failed: {name}: {future.exception()}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service returned no response: {name}")
        return result

    def query(name, deadline):
        return call(name + "/get_state", GetState, GetState.Request(), deadline).current_state.id

    def change(name, transition, deadline):
        request = ChangeState.Request()
        request.transition.id = transition
        return call(name + "/change_state", ChangeState, request, deadline).success

    try:
        if not names or not 0 < deadline_s <= 300:
            raise ValueError("invalid navigation startup parameters")
        bringup(names, query, change, deadline_s=deadline_s)
        node.get_logger().info("Navigation lifecycle nodes confirmed active")
    except Exception as exc:
        node.get_logger().error(f"Navigation startup failed: {exc}")
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
