#!/usr/bin/env python3
"""Target-only ROS quiescence and product/readiness probe for map reset."""

from __future__ import annotations

from contextlib import ExitStack
import argparse
import json
import os
from pathlib import Path
import re
import time


def call(node, client, request, deadline, *, idempotent=False):
    import rclpy

    while (remaining := deadline - time.monotonic()) > 0:
        if client.wait_for_service(timeout_sec=min(0.2, remaining)):
            break
    else:
        raise TimeoutError(f"service unavailable: {client.srv_name}")
    while time.monotonic() < deadline:
        future = client.call_async(request)
        remaining = max(0.0, deadline - time.monotonic())
        # Fast DDS can discover the request writer before the response reader.
        # The caller retains this client throughout the operation; only replay
        # requests whose side effects are explicitly idempotent.
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=min(2.0, remaining) if idempotent else remaining
        )
        if future.done():
            return future.result()
        client.remove_pending_request(future)
        future.cancel()
        if not idempotent:
            break
    raise TimeoutError(f"service timed out: {client.srv_name}")


def quiesce(node, robot, deadline):
    import rclpy
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist

    required = {f"/{robot}/follow_path/_action/cancel_goal"}
    while time.monotonic() < deadline:
        services = dict(node.get_service_names_and_types())
        if all(
            "action_msgs/srv/CancelGoal" in services.get(name, []) for name in required
        ):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise TimeoutError(f"trajectory action server unavailable for {robot}")
    with ExitStack() as resources:
        publisher = node.create_publisher(Twist, f"/{robot}/cmd_vel", 1)
        resources.callback(node.destroy_publisher, publisher)
        clients = []
        for name, types in services.items():
            if name.startswith(f"/{robot}/") and "action_msgs/srv/CancelGoal" in types:
                client = node.create_client(CancelGoal, name)
                resources.callback(node.destroy_client, client)
                clients.append(client)
        for client in clients:
            reply = call(node, client, CancelGoal.Request(), deadline, idempotent=True)
            if reply.return_code != CancelGoal.Response.ERROR_NONE:
                raise RuntimeError(
                    f"action cancellation failed: {client.srv_name}: {reply.return_code}"
                )
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise TimeoutError(f"velocity receiver unavailable for {robot}")
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.1)


class SourceResetter:
    """Quiesce navigation and clear the live local costmap for a new epoch."""

    def __init__(self, node, robot):
        from nav2_msgs.srv import ClearEntireCostmap

        self.node, self.robot = node, robot
        self.clear = node.create_client(
            ClearEntireCostmap,
            f"/{robot}/local_costmap/clear_entirely_local_costmap",
        )

    def reset(self, deadline, *, quiesced=False):
        import rclpy
        from nav2_msgs.srv import ClearEntireCostmap

        node, robot = self.node, self.robot
        if not quiesced:
            quiesce(node, robot, deadline)
        call(node, self.clear, ClearEntireCostmap.Request(), deadline, idempotent=True)
        rclpy.spin_once(node, timeout_sec=0.1)
        stamp = node.get_clock().now().nanoseconds
        if stamp <= 0:
            raise RuntimeError("simulation clock unavailable after local costmap reset")
        return {
            "source_reset_stamp": {
                "sec": stamp // 1_000_000_000,
                "nanosec": stamp % 1_000_000_000,
            }
        }


def verify(node, robot, mission, minimum, deadline):
    import rclpy
    from geometry_msgs.msg import Point
    from mgg_msgs.srv import PlanObjective, QueryMapBatch
    from std_msgs.msg import String

    authority = {}

    def receive(message):
        nonlocal authority
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict):
            authority = value

    subscription = node.create_subscription(
        String, f"/{robot}/map_authority", receive, 5
    )
    planner = node.create_client(PlanObjective, f"/{robot}/mgg/plan_objective")
    query = node.create_client(QueryMapBatch, f"/{robot}/mapping/query_batch")
    last_error = "waiting for a fresh product-backed authority and planner"
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                epoch = json.loads(
                    (
                        Path(os.environ.get("SWARMDECK_MAPS_ROOT", "/maps"))
                        / mission
                        / robot
                        / "map-epoch.json"
                    ).read_text()
                )
                if not (
                    epoch.get("mission_id") == mission
                    and epoch.get("robot_id") == robot
                    and type(epoch.get("map_epoch")) is int
                    and epoch["map_epoch"] >= minimum
                    and authority.get("mission_id") == mission
                    and authority.get("robot_id") == robot
                    and authority.get("robot_map_epoch") == epoch["map_epoch"]
                    and authority.get("run_id") == epoch.get("run_id")
                    and authority.get("mapping_graph_revision", 0) > 0
                    and planner.service_is_ready()
                ):
                    continue
                request = QueryMapBatch.Request()
                request.component_id = authority["component_id"]
                request.epoch = authority["map_epoch"]
                request.graph_revision = authority["mapping_graph_revision"]
                request.geometry_revision = authority["geometry_revision"]
                stamp = authority["map_source_stamp"]
                request.source_stamp.sec = stamp["sec"]
                request.source_stamp.nanosec = stamp["nanosec"]
                transform = authority["T_component_navigation"]
                request.samples = [
                    Point(
                        x=float(transform[0][3]),
                        y=float(transform[1][3]),
                        z=float(transform[2][3]),
                    )
                ]
                request.body_size.x = request.body_size.y = request.body_size.z = 0.1
                request.max_step_m = request.max_drop_m = 0.15
                request.stop_at_unknown = False
                reply = call(node, query, request, min(deadline, time.monotonic() + 2))
                if reply.status != QueryMapBatch.Response.OK:
                    last_error = (
                        f"fresh indexed product is not queryable: {reply.status}"
                    )
                    continue
                return {"map_epoch": epoch["map_epoch"], "run_id": epoch["run_id"]}
            except (OSError, ValueError, KeyError, TypeError) as exc:
                last_error = str(exc)
        raise TimeoutError(last_error)
    finally:
        node.destroy_client(query)
        node.destroy_client(planner)
        node.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("quiesce", "verify"))
    parser.add_argument("--robot", required=True)
    parser.add_argument("--mission", required=True)
    parser.add_argument("--minimum", type=int, default=0)
    parser.add_argument("--timeout", type=float, required=True)
    args = parser.parse_args()
    deadline = time.monotonic() + args.timeout
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.robot):
        parser.error("invalid robot namespace")
    import rclpy

    rclpy.init()
    from rclpy.parameter import Parameter

    node = rclpy.create_node(
        f"map_reset_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    try:
        if args.operation == "quiesce":
            result = quiesce(node, args.robot, deadline)
        else:
            result = verify(node, args.robot, args.mission, args.minimum, deadline)
        print(json.dumps(result or {"ok": True}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
