#!/usr/bin/env python3
"""Inter-robot transforms for MGG's roadmap merge.

MGG merges another robot's roadmap only with the transform from that robot's
planning frame into its own (``neighbour_pose_source: topic``). This node
publishes those transforms for one robot on ``/<robot>/mgg/neighbour_transforms``
as ``tf2_msgs/TFMessage``: ``header.frame_id`` is the robot's planning frame,
``child_frame_id`` the neighbour's, and the transform is T_ours_theirs.

Two sources, chosen with ``--robot-poses`` (``SWARMDECK_ROBOT_POSES``):

``cslam`` (the default, and the only one on hardware)
    C-SLAM's inter-robot estimate. Each peer's map authority
    (``/<robot>/map_authority``) carries ``T_component_planning``, its planning
    frame in its map component. Two robots in the same component give
    T_ours_theirs = inv(T_component_ours) @ T_component_theirs. Robots in
    different components have no transform, and share nothing, until C-SLAM
    links them.

``ground_truth`` (simulation only)
    The simulator's pose. The ARGoS bridge publishes ``/<robot>/ground_truth``
    (world <- base) beside ``/<robot>/odom`` (odom <- base) at the same tick,
    so world <- odom = ground_truth @ inv(odom) for every robot, from the start.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re

import numpy as np

SOURCES = ("cslam", "ground_truth")
PUBLISH_PERIOD_S = 1.0
# Ground-truth samples kept per robot to pair with an odometry stamp.
MAX_TRUTH_SAMPLES = 400
# Odometry without a ground-truth sample of its own tick pairs with one this
# close; a robot at 0.5 m/s moves 2.5 cm in it.
MAX_PAIRING_GAP_NS = 50_000_000
MAX_AUTHORITY_BYTES = 32_768


def pose_matrix(position, orientation):
    """4x4 transform of a position (x, y, z) and a quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in orientation)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not norm > 1e-9 or not all(math.isfinite(float(v)) for v in position):
        raise ValueError("invalid pose")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = [float(v) for v in position]
    return result


def quaternion(rotation):
    """(x, y, z, w) of a proper rotation matrix."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


def rigid_transform(value):
    """A finite 4x4 rigid transform, or None."""
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        return None
    rotation = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6)
        or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4)
        or np.linalg.det(rotation) <= 0.0
    ):
        return None
    return matrix


class CslamPoses:
    """Planning frames placed in C-SLAM map components, from map authorities."""

    def __init__(self):
        self.placements = {}

    def update(self, robot, authority):
        """Record `robot`'s map authority (a parsed JSON dict)."""
        placement = None
        if (
            isinstance(authority, dict)
            and authority.get("robot_id") == robot
            and authority.get("state") != "resetting"
        ):
            component = authority.get("component_id")
            frame = str(authority.get("planning_frame") or "").lstrip("/")
            transform = rigid_transform(authority.get("T_component_planning"))
            if isinstance(component, str) and component and frame:
                if transform is not None:
                    placement = (component, frame, transform)
        if placement is None:
            self.placements.pop(robot, None)
        else:
            self.placements[robot] = placement

    def transforms(self, robot):
        """(own frame, {peer: (peer frame, T_ours_theirs)}) for peers in
        `robot`'s component."""
        own = self.placements.get(robot)
        if own is None:
            return None, {}
        component, frame, t_component_ours = own
        ours_component = np.linalg.inv(t_component_ours)
        result = {}
        for peer, (peer_component, peer_frame, t_component_theirs) in sorted(
            self.placements.items()
        ):
            if peer != robot and peer_component == component:
                result[peer] = (peer_frame, ours_component @ t_component_theirs)
        return frame, result


class GroundTruthPoses:
    """Odometry frames placed in the simulator's world frame."""

    def __init__(self):
        self.truth = {}
        self.world_odom = {}

    def add_truth(self, robot, stamp_ns, t_world_base):
        samples = self.truth.setdefault(robot, {})
        samples[stamp_ns] = t_world_base
        while len(samples) > MAX_TRUTH_SAMPLES:
            del samples[next(iter(samples))]

    def add_odometry(self, robot, stamp_ns, frame, t_odom_base):
        """Pair odometry with the ground truth of its tick (the bridge stamps
        both from the tick counter), or the nearest within
        MAX_PAIRING_GAP_NS, and place the odometry frame in the world."""
        samples = self.truth.get(robot, {})
        t_world_base = samples.get(stamp_ns)
        if t_world_base is None and samples:
            nearest = min(samples, key=lambda stamp: abs(stamp - stamp_ns))
            if abs(nearest - stamp_ns) <= MAX_PAIRING_GAP_NS:
                t_world_base = samples[nearest]
        frame = str(frame).lstrip("/")
        if t_world_base is None or not frame:
            return False
        self.world_odom[robot] = (frame, t_world_base @ np.linalg.inv(t_odom_base))
        return True

    def transforms(self, robot):
        own = self.world_odom.get(robot)
        if own is None:
            return None, {}
        frame, t_world_ours = own
        ours_world = np.linalg.inv(t_world_ours)
        return frame, {
            peer: (peer_frame, ours_world @ t_world_theirs)
            for peer, (peer_frame, t_world_theirs) in sorted(self.world_odom.items())
            if peer != robot
        }


def selected_source(value=None):
    source = (value or os.environ.get("SWARMDECK_ROBOT_POSES") or "cslam").lower()
    if source not in SOURCES:
        raise ValueError(f"SWARMDECK_ROBOT_POSES must be one of {', '.join(SOURCES)}")
    return source


class Announcements:
    """Logs once when a neighbour has no transform and once when it gains one."""

    def __init__(self, robot, source, log):
        self.robot, self.source, self.log = robot, source, log
        self.missing = set()
        self.present = set()

    def update(self, peers, placed):
        for peer in peers:
            if peer in placed and peer not in self.present:
                self.present.add(peer)
                self.missing.discard(peer)
                self.log(f"{self.robot}: sharing roadmaps with {peer} ({self.source})")
            elif peer not in placed and peer not in self.missing:
                self.missing.add(peer)
                self.present.discard(peer)
                reason = (
                    "not in the same C-SLAM map component yet"
                    if self.source == "cslam"
                    else "no ground truth and odometry received yet"
                )
                self.log(
                    f"{self.robot}: no transform to {peer} ({reason}); its "
                    "roadmap is not merged"
                )


def transform_message(frame, peer_frame, transform, stamp):
    from geometry_msgs.msg import TransformStamped

    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame
    message.child_frame_id = peer_frame
    t = message.transform
    t.translation.x, t.translation.y, t.translation.z = (
        float(v) for v in transform[:3, 3]
    )
    q = quaternion(transform[:3, :3])
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = (float(v) for v in q)
    return message


def run(robot, peers, source):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("robot_poses", namespace=f"/{robot}/mgg")
    publisher = node.create_publisher(
        TFMessage, f"/{robot}/mgg/neighbour_transforms", 10
    )
    announcements = Announcements(robot, source, node.get_logger().info)
    if source == "cslam":
        poses = CslamPoses()

        def on_authority(peer):
            def handle(message):
                if len(message.data) > MAX_AUTHORITY_BYTES:
                    return
                try:
                    poses.update(peer, json.loads(message.data))
                except ValueError:
                    poses.update(peer, None)

            return handle

        for peer in [robot, *peers]:
            node.create_subscription(
                String, f"/{peer}/map_authority", on_authority(peer), 5
            )
    else:
        poses = GroundTruthPoses()

        def stamp_ns(message):
            return (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )

        def matrix(message):
            pose = message.pose.pose
            p, q = pose.position, pose.orientation
            return pose_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

        def on_truth(peer):
            return lambda m: poses.add_truth(peer, stamp_ns(m), matrix(m))

        def on_odometry(peer):
            return lambda m: poses.add_odometry(
                peer, stamp_ns(m), m.header.frame_id, matrix(m)
            )

        for peer in [robot, *peers]:
            node.create_subscription(
                Odometry, f"/{peer}/ground_truth", on_truth(peer), 10
            )
            node.create_subscription(Odometry, f"/{peer}/odom", on_odometry(peer), 10)

    def publish():
        frame, placed = poses.transforms(robot)
        announcements.update(peers, placed if frame else {})
        if not frame or not placed:
            return
        stamp = node.get_clock().now().to_msg()
        publisher.publish(
            TFMessage(
                transforms=[
                    transform_message(frame, peer_frame, transform, stamp)
                    for peer_frame, transform in placed.values()
                ]
            )
        )

    node.create_timer(PUBLISH_PERIOD_S, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 launch stops the group with a signal, which shuts the context
        # down under a spin that may be building its wait set.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", required=True)
    parser.add_argument("--peers", required=True, help="JSON list of robot names")
    parser.add_argument("--robot-poses", choices=SOURCES, default=None)
    args, _ = parser.parse_known_args(argv)
    peers = json.loads(args.peers)
    names = [args.robot, *peers]
    if not all(
        isinstance(n, str) and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n) for n in names
    ):
        raise ValueError("robot names must be ROS namespace components")
    run(
        args.robot,
        [p for p in peers if p != args.robot],
        selected_source(args.robot_poses),
    )


if __name__ == "__main__":
    main()
