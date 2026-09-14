#!/usr/bin/env python3
"""Native two-peer loop-closure smoke with a known relative SE(3) pose.

Run in swarmdeck-cslam:planning on an unused ROS domain. This checks descriptor
exchange, TEASER verification, and recipient-specific optimization without any
server. The default case uses a 0.4 m translation and 6 degree yaw; pass
``--identity`` for the historical identity case. It remains an integration
fixture rather than a general alignment benchmark.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from cslam_common_interfaces.msg import InterRobotLoopClosure, OptimizationResult


def _rotation_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pose_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = _rotation_matrix(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    )
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _transform_matrix(transform):
    matrix = np.eye(4)
    matrix[:3, :3] = _rotation_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def _yaw_transform(yaw, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1],
    ]
    matrix[:3, 3] = translation
    return matrix


def _relative_result_pose(result, recipient):
    anchors = [
        value
        for value in result.anchor_estimates
        if value.key.robot_id == result.origin_robot_id
    ]
    estimates = [value for value in result.estimates if value.key.robot_id == recipient]
    if not anchors or not estimates:
        raise AssertionError("optimization result omitted anchor or recipient estimate")
    return np.linalg.inv(_pose_matrix(anchors[0].pose)) @ _pose_matrix(
        estimates[0].pose
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--identity",
        action="store_true",
        help="retain the historical identity relative-pose fixture",
    )
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("peer_fixture")
    mission = str(uuid.uuid4())
    robots = ["fixture_0", "fixture_1"]
    directory = Path(tempfile.mkdtemp(prefix="cslam-peer-smoke-"))
    rng = np.random.default_rng(831)
    # Asymmetric surfaces provide nondegenerate local registration features.
    points = np.vstack(
        [
            np.column_stack(
                [rng.uniform(-9, 13, 4000), np.full(4000, 7), rng.uniform(-1, 5, 4000)]
            ),
            np.column_stack(
                [np.full(3000, -6), rng.uniform(-11, 7, 3000), rng.uniform(-1, 3, 3000)]
            ),
            np.column_stack(
                [
                    rng.uniform(-9, 13, 5000),
                    rng.uniform(-11, 7, 5000),
                    np.full(5000, -1),
                ]
            ),
            rng.normal([3, 2, 1], [1, 1, 1], (2000, 3)),
        ]
    ).astype(np.float32)
    expected_relative = (
        np.eye(4) if args.identity else _yaw_transform(math.radians(6), [0.4, 0.0, 0.0])
    )
    # If robot 1 has world pose T, its local coordinates are T^-1 * p_world.
    # Registration(src=robot0, dst=robot1) estimates T_robot1_robot0.
    # compute_transform must invert it for the BetweenFactor(robot0, robot1)
    # measurement, whose expected relation is T_robot0_robot1.
    point_sets = [
        points,
        ((points - expected_relative[:3, 3]) @ expected_relative[:3, :3]).astype(
            np.float32
        ),
    ]
    closures, solution_groups = [], {}
    node.create_subscription(
        InterRobotLoopClosure,
        "/cslam/inter_robot_loop_closure",
        lambda msg: closures.append(msg) if msg.success else None,
        100,
    )
    for index in range(2):

        def solution(msg, recipient=index):
            if (
                msg.success
                and msg.mission_id == mission
                and msg.estimates
                and msg.anchor_estimates
            ):
                group_key = (
                    int(msg.solution_clock),
                    int(msg.optimizer_robot_id),
                    int(msg.origin_robot_id),
                )
                group = solution_groups.setdefault(group_key, {})
                group[recipient] = msg
                while len(solution_groups) > 16:
                    solution_groups.pop(next(iter(solution_groups)))

        node.create_subscription(
            OptimizationResult, f"/r{index}/cslam/optimized_estimates", solution, 100
        )
    cloud_pubs = [
        node.create_publisher(PointCloud2, f"/{robot}/scan/points", 5)
        for robot in robots
    ]
    tf_pubs = [
        node.create_publisher(TFMessage, f"/{robot}/tf", 100) for robot in robots
    ]
    processes, streams = [], []
    try:
        for index, robot in enumerate(robots):
            env = {
                **os.environ,
                "SWARMDECK_MISSION_ID": mission,
                "SWARMDECK_PEER_NAMES": json.dumps(robots),
                "SWARMDECK_PEER_INDEX": str(index),
                "SWARMDECK_USE_SIM_TIME": "false",
                "SWARMDECK_SERVER_URL": "",
                "SWARMDECK_MAP_STORE": str(directory / "maps"),
                "SWARMDECK_NAVIGATION_FRAME": f"{robot}/odom",
            }
            stream = (directory / f"peer{index}.log").open("w")
            streams.append(stream)
            processes.append(
                subprocess.Popen(
                    ["ros2", "launch", "/app/deploy/autonomy/peer.launch.py"],
                    env=env,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        start, published, reported = time.monotonic(), 0.0, 0.0
        while time.monotonic() - start < 150:
            now = time.monotonic()
            if any(process.poll() is not None for process in processes):
                raise RuntimeError(f"peer exited; inspect {directory}")
            if now - published >= 0.2:
                stamp = node.get_clock().now().to_msg()
                for index, robot in enumerate(robots):
                    tf = TransformStamped()
                    tf.header = Header(stamp=stamp, frame_id=f"{robot}/odom")
                    tf.child_frame_id = f"{robot}/base_link"
                    yaw = expected_relative if index else np.eye(4)
                    yaw_angle = math.atan2(yaw[1, 0], yaw[0, 0])
                    tf.transform.rotation.w = math.cos(yaw_angle / 2)
                    tf.transform.rotation.z = math.sin(yaw_angle / 2)
                    tf.transform.translation.x = float(yaw[0, 3])
                    tf.transform.translation.y = float(yaw[1, 3])
                    tf.transform.translation.z = float(yaw[2, 3])
                    tf_pubs[index].publish(TFMessage(transforms=[tf]))
                    cloud_pubs[index].publish(
                        create_cloud_xyz32(
                            Header(stamp=stamp, frame_id=f"{robot}/base_link"),
                            point_sets[index],
                        )
                    )
                published = now
            rclpy.spin_once(node, timeout_sec=0.05)
            if now - reported > 10:
                print(
                    json.dumps(
                        {
                            "elapsed_s": round(now - start),
                            "verified_closures": len(closures),
                            "joint_solution_groups": sum(
                                set(group) == {0, 1}
                                for group in solution_groups.values()
                            ),
                        }
                    ),
                    flush=True,
                )
                reported = now
            joint_groups = [
                group for group in solution_groups.values() if set(group) == {0, 1}
            ]
            if closures and joint_groups:
                solutions = joint_groups[-1]
                anchor_poses = []
                for recipient, result in solutions.items():
                    if result.origin_robot_id not in (0, 1):
                        raise AssertionError(
                            f"unexpected graph anchor robot: {result.origin_robot_id}"
                        )
                    if any(v.key.robot_id != recipient for v in result.estimates):
                        raise AssertionError(
                            "optimizer mixed recipient poses or used the wrong estimate stream"
                        )
                    anchors = [
                        value
                        for value in result.anchor_estimates
                        if value.key.robot_id == result.origin_robot_id
                    ]
                    if not anchors:
                        raise AssertionError(
                            "joint solution omitted its declared anchor"
                        )
                    anchor_poses.append(_pose_matrix(anchors[0].pose))
                if not np.allclose(anchor_poses[0], anchor_poses[1], atol=0.2):
                    raise AssertionError(
                        "joint solution recipients used different anchor poses"
                    )
                closure = closures[-1]
                if (closure.robot0_id, closure.robot1_id) == (0, 1):
                    expected_closure = expected_relative
                elif (closure.robot0_id, closure.robot1_id) == (1, 0):
                    expected_closure = np.linalg.inv(expected_relative)
                else:
                    raise AssertionError(
                        f"unexpected closure IDs: {closure.robot0_id}->{closure.robot1_id}"
                    )
                actual_closure = _transform_matrix(closure.transform)
                if not np.allclose(actual_closure, expected_closure, atol=0.12):
                    raise AssertionError(
                        f"closure transform mismatch: expected {expected_closure}, "
                        f"got {actual_closure}"
                    )
                for recipient, result in solutions.items():
                    origin = result.origin_robot_id
                    if recipient == origin:
                        expected_pose = np.eye(4)
                    elif origin == 0:
                        expected_pose = expected_relative
                    else:
                        expected_pose = np.linalg.inv(expected_relative)
                    actual_pose = _relative_result_pose(result, recipient)
                    if not np.allclose(actual_pose, expected_pose, atol=0.2):
                        raise AssertionError(
                            f"optimized recipient {recipient} pose mismatch: "
                            f"expected {expected_pose}, got {actual_pose}"
                        )
                print(
                    f"PASS: verified native peer closure and both optimized recipients; logs {directory}",
                    flush=True,
                )
                return
        raise TimeoutError(f"no verified joint solution; inspect {directory}")
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
