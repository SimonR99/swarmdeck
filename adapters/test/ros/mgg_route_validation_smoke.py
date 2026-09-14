#!/usr/bin/env python3
"""Plan and validate one full route against a running MGG instance; no motion.

Run inside the simulation's ROS environment after all robots are online:
    python3 adapters/test/ros/mgg_route_validation_smoke.py robot_0 --distance 3
The native planner and adapter must use the same ValidateObjectiveRoute type.
"""

import argparse
import json
import math
import time

import rclpy
from mgg_msgs.srv import PlanObjective, ValidateObjectiveRoute
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot", nargs="?", default="robot_0")
    parser.add_argument("--distance", type=float, default=3.0)
    args = parser.parse_args()
    if not math.isfinite(args.distance) or not 0.5 <= args.distance <= 50:
        parser.error("distance must be between 0.5 and 50 meters")
    rclpy.init()
    node = rclpy.create_node("mgg_route_validation_smoke")
    latest = {}

    def authority(message):
        try:
            latest["authority"] = json.loads(message.data)
        except ValueError:
            pass

    def odometry(message):
        latest["odometry"] = message

    subscriptions = [
        node.create_subscription(String, f"/{args.robot}/map_authority", authority, 10),
        node.create_subscription(Odometry, f"/{args.robot}/mgg/map_odometry", odometry, 10),
    ]
    plan = node.create_client(PlanObjective, f"/{args.robot}/mgg/plan_objective")
    validate = node.create_client(
        ValidateObjectiveRoute, f"/{args.robot}/mgg/validate_objective_route"
    )

    def call(client, request):
        start = time.monotonic()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            future.cancel()
            client.remove_pending_request(future)
            raise RuntimeError(f"{client.srv_name} timed out")
        return future.result(), round((time.monotonic() - start) * 1000, 1)

    try:
        if not plan.wait_for_service(timeout_sec=10) or not validate.wait_for_service(timeout_sec=10):
            raise RuntimeError("native planning or validation service unavailable")
        deadline = time.monotonic() + 15
        while len(latest) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(latest) != 2:
            raise RuntimeError("navigation authority or planning odometry unavailable")
        authority_value = latest["authority"]
        odom = latest["odometry"]
        pose = odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        request = PlanObjective.Request()
        request.objective = request.NAVIGATE
        for field in ("mission_id", "component_id", "map_epoch", "mapping_graph_revision", "geometry_revision"):
            setattr(request, field, authority_value[field])
        request.map_source_stamp.sec = authority_value["map_source_stamp"]["sec"]
        request.map_source_stamp.nanosec = authority_value["map_source_stamp"]["nanosec"]
        request.goal.position.x = pose.position.x + args.distance * math.cos(yaw)
        request.goal.position.y = pose.position.y + args.distance * math.sin(yaw)
        request.goal.position.z = pose.position.z
        request.goal.orientation = q
        result, planning_ms = call(plan, request)
        if result.status != result.SUCCEEDED or len(result.path) < 2:
            raise RuntimeError(f"full route rejected: {result.reason}")
        endpoint = result.path[-1].position
        if math.hypot(endpoint.x - request.goal.position.x, endpoint.y - request.goal.position.y) > 1e-5:
            raise RuntimeError("planner replaced the selected destination")
        validation_request = ValidateObjectiveRoute.Request()
        validation_request.mission_id = request.mission_id
        validation_request.component_id = request.component_id
        validation_request.frame_id = odom.header.frame_id
        validation_request.path = result.path
        validation_request.lookahead_m = 3.0
        checked, validation_ms = call(validate, validation_request)
        print(json.dumps({
            "robot": args.robot, "distance_m": args.distance,
            "path_poses": len(result.path), "planning_ms": planning_ms,
            "validation_ms": validation_ms, "validation_status": checked.status,
            "reason": checked.reason,
        }), flush=True)
        if checked.status != checked.VALID:
            raise RuntimeError("accepted route did not pass live validation")
        validation_request.mission_id = "obsolete-mission-smoke-test"
        stale, _ = call(validate, validation_request)
        if stale.status != stale.UNAVAILABLE:
            raise RuntimeError("validator accepted a route from another mission")
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
