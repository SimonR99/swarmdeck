#!/usr/bin/env python3
"""Coordinated frontier exploration for the simulated fleet.

The controller deliberately separates two objectives:

* one short, ordered rendezvous gives the collaborative SLAM back-end common
  observations with which to connect all robot trajectories;
* after that, frontier goals are allocated jointly so robot sensing footprints
  do not chase the same unknown area.

All navigation goals are expressed in each robot's own SLAM ``map_frame``.  A
configured spawn transform is used only to put maps and candidate goals in a
common planning frame.  Robot motion comes from the complete SLAM TF chain
(``map -> odom -> base_link``), not raw wheel odometry.  Gazebo ground truth is
never subscribed to or used by this node.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

from frontier_planner import (
    Assignment,
    CommonGrid,
    Frontier,
    GridSnapshot,
    RobotState,
    allocate_frontiers,
    extract_frontiers,
    frontier_near,
    inverse_transform_point,
    merge_grids,
    rendezvous_slots,
    transform_point,
)

PLAN_PERIOD_S = 5.0
GOAL_TIMEOUT_S = 150.0
RENDEZVOUS_TIMEOUT_S = 210.0
RENDEZVOUS_DWELL_S = 20.0
RENDEZVOUS_RETRY_DELAY_S = 5.0
MERGE_SCAN_ATTEMPTS = 3
NO_FRONTIER_DONE_S = 25.0
FAILED_GOAL_COOLDOWN_S = 90.0
FAILED_GOAL_RADIUS_M = 1.5
STALE_GOAL_RADIUS_M = 1.0
STALE_GOAL_MIN_AGE_S = 15.0
PATH_SAMPLE_MAX_M = 1.5


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _compose(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> tuple[float, float, float]:
    cosine, sine = math.cos(first[2]), math.sin(first[2])
    return (
        first[0] + second[0] * cosine - second[1] * sine,
        first[1] + second[0] * sine + second[1] * cosine,
        (first[2] + second[2] + math.pi) % (2.0 * math.pi) - math.pi,
    )


@dataclass(slots=True)
class ActiveGoal:
    token: int
    purpose: str
    world_x: float
    world_y: float
    sent_at: float
    handle: object | None = None


class CoordinatedExplorer(Node):
    def __init__(
        self,
        robot_ids: list[str],
        *,
        starts: dict[str, tuple[float, float, float]],
        radii: dict[str, float],
        navigation_clearances: dict[str, float],
        map_size_m: float,
        resolution: float,
        metrics_path: str = "",
        slam_status_url: str = "",
    ) -> None:
        super().__init__("swarmdeck_coordinated_explorer")
        self.robot_ids = robot_ids
        self.starts = starts
        self.radii = radii
        self.navigation_clearances = navigation_clearances
        self.map_size_m = map_size_m
        self.resolution = resolution
        self.metrics_path = metrics_path
        self.slam_status_url = slam_status_url

        self.local_maps: dict[str, GridSnapshot] = {}
        self.global_maps: dict[str, GridSnapshot] = {}
        self.map_received_at: dict[tuple[str, str], float] = {}
        self.map_to_odom: dict[str, tuple[float, float, float]] = {}
        self.odom_to_base: dict[str, tuple[float, float, float]] = {}
        self.nav_clients: dict[str, ActionClient] = {}
        self.spin_clients: dict[str, ActionClient] = {}
        self.stop_publishers: dict[str, object] = {}

        self.active: dict[str, ActiveGoal] = {}
        self.goal_tokens = {rid: 0 for rid in robot_ids}
        self.completed_goals = {rid: 0 for rid in robot_ids}
        self.failed_goals = {rid: 0 for rid in robot_ids}
        self.cancelled_goals = {rid: 0 for rid in robot_ids}
        self.rendezvous_finished: set[str] = set()
        self.rendezvous_retry_after = {rid: 0.0 for rid in robot_ids}
        self.merge_scan_finished: set[str] = set()
        self.merge_scan_attempts = 0
        self.rendezvous_targets: dict[str, tuple[float, float]] = {}
        self.merge_verified: bool | None = None
        self.merge_components: list[list[str]] = []
        self.failed_points: list[tuple[float, float, float]] = []

        self.path_m = {rid: 0.0 for rid in robot_ids}
        self.last_world_pose: dict[str, tuple[float, float]] = {}
        self.phase = "waiting"
        self.phase_since = time.monotonic()
        self.started_at = self.phase_since
        self.last_plan_at = 0.0
        self.no_frontier_since = 0.0
        self.latest_common: CommonGrid | None = None
        self.latest_frontiers: list[Frontier] = []
        self.last_metrics_at = 0.0
        self.done = threading.Event()
        self.running = True
        self._waiting_reported_at = 0.0

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        for robot_id in robot_ids:
            self.create_subscription(
                OccupancyGrid,
                f"/{robot_id}/map",
                lambda msg, rid=robot_id: self._on_map(rid, "local", msg),
                latched,
            )
            self.create_subscription(
                OccupancyGrid,
                f"/{robot_id}/global_map",
                lambda msg, rid=robot_id: self._on_map(rid, "global", msg),
                latched,
            )
            self.create_subscription(
                TFMessage,
                f"/{robot_id}/tf",
                lambda msg, rid=robot_id: self._on_tf(rid, msg),
                20,
            )
            self.nav_clients[robot_id] = ActionClient(
                self, NavigateToPose, f"/{robot_id}/navigate_to_pose"
            )
            self.spin_clients[robot_id] = ActionClient(
                self, Spin, f"/{robot_id}/spin"
            )
            self.stop_publishers[robot_id] = self.create_publisher(
                Twist, f"/{robot_id}/cmd_vel", 10
            )

        self.create_timer(1.0, self.tick)

    def _on_map(self, robot_id: str, kind: str, msg: OccupancyGrid) -> None:
        width, height = int(msg.info.width), int(msg.info.height)
        values = np.asarray(msg.data, dtype=np.int8)
        if width <= 0 or height <= 0 or values.size != width * height:
            return
        snapshot = GridSnapshot(
            values.reshape(height, width).copy(),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            _yaw(msg.info.origin.orientation),
        )
        if kind == "global":
            self.global_maps[robot_id] = snapshot
        else:
            self.local_maps[robot_id] = snapshot
        self.map_received_at[(robot_id, kind)] = time.monotonic()

    def _on_tf(self, robot_id: str, msg: TFMessage) -> None:
        map_frame = f"{robot_id}/map_frame"
        odom_frame = f"{robot_id}/odom"
        base_frame = f"{robot_id}/base_link"
        for stamped in msg.transforms:
            value = (
                float(stamped.transform.translation.x),
                float(stamped.transform.translation.y),
                _yaw(stamped.transform.rotation),
            )
            parent = stamped.header.frame_id.lstrip("/")
            child = stamped.child_frame_id.lstrip("/")
            if parent == map_frame and child == odom_frame:
                self.map_to_odom[robot_id] = value
            elif parent == odom_frame and child == base_frame:
                self.odom_to_base[robot_id] = value

    def _local_pose(self, robot_id: str) -> tuple[float, float, float] | None:
        odom_base = self.odom_to_base.get(robot_id)
        if odom_base is None:
            return None
        return _compose(self.map_to_odom.get(robot_id, (0.0, 0.0, 0.0)), odom_base)

    def _world_pose(self, robot_id: str) -> tuple[float, float, float] | None:
        pose = self._local_pose(robot_id)
        start = self.starts.get(robot_id)
        if pose is None or start is None:
            return None
        x, y = transform_point((pose[0], pose[1]), start)
        return x, y, (pose[2] + start[2] + math.pi) % (2.0 * math.pi) - math.pi

    def _update_paths(self) -> None:
        for robot_id in self.robot_ids:
            pose = self._world_pose(robot_id)
            if pose is None:
                continue
            previous = self.last_world_pose.get(robot_id)
            self.last_world_pose[robot_id] = (pose[0], pose[1])
            if previous is None:
                continue
            distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
            # A larger jump is a SLAM graph correction, not driven distance.
            if 0.002 < distance <= PATH_SAMPLE_MAX_M:
                self.path_m[robot_id] += distance

    def _ready(self) -> bool:
        return (
            all(robot_id in self.local_maps for robot_id in self.robot_ids)
            and all(robot_id in self.global_maps for robot_id in self.robot_ids)
            and all(
                self._local_pose(robot_id) is not None for robot_id in self.robot_ids
            )
            and all(client.server_is_ready() for client in self.nav_clients.values())
            and all(client.server_is_ready() for client in self.spin_clients.values())
        )

    def _map_source(self) -> dict[str, GridSnapshot]:
        # The optimized downlink is preferable once it exists.  Before the
        # common graph connects, the server deliberately returns each robot's
        # own raytraced grid, so this remains valid during bootstrap too.
        return {
            robot_id: self.global_maps.get(robot_id, self.local_maps[robot_id])
            for robot_id in self.robot_ids
            if robot_id in self.local_maps
        }

    def _common_grid(self, *, local_only: bool = False) -> CommonGrid | None:
        source = self.local_maps if local_only else self._map_source()
        if not source:
            return None
        return merge_grids(
            source,
            self.starts,
            size_m=self.map_size_m,
            resolution=self.resolution,
        )

    def _send_goal(
        self,
        robot_id: str,
        world: tuple[float, float],
        purpose: str,
        *,
        world_yaw: float | None = None,
    ) -> bool:
        if robot_id in self.active or robot_id not in self.starts:
            return False
        client = self.nav_clients[robot_id]
        if not client.server_is_ready():
            return False
        local_x, local_y = inverse_transform_point(world, self.starts[robot_id])
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = f"{robot_id}/map_frame"
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = float(local_x)
        request.pose.pose.position.y = float(local_y)
        local_yaw = 0.0 if world_yaw is None else world_yaw - self.starts[robot_id][2]
        request.pose.pose.orientation.z = math.sin(local_yaw / 2.0)
        request.pose.pose.orientation.w = math.cos(local_yaw / 2.0)

        self.goal_tokens[robot_id] += 1
        token = self.goal_tokens[robot_id]
        self.active[robot_id] = ActiveGoal(
            token, purpose, float(world[0]), float(world[1]), time.monotonic()
        )
        future = client.send_goal_async(request)
        future.add_done_callback(
            lambda result, rid=robot_id, expected=token: self._goal_response(
                rid, expected, result
            )
        )
        self.get_logger().info(
            f"{robot_id}: {purpose} goal world=({world[0]:.2f}, {world[1]:.2f}) "
            f"local=({local_x:.2f}, {local_y:.2f})"
        )
        return True

    def _goal_response(self, robot_id: str, token: int, future) -> None:
        record = self.active.get(robot_id)
        if record is None or record.token != token:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{robot_id}: goal request failed: {exc}")
            self._finish_goal(robot_id, token, GoalStatus.STATUS_ABORTED)
            return
        if not handle.accepted:
            self.get_logger().warn(f"{robot_id}: Nav2 rejected {record.purpose} goal")
            self._finish_goal(robot_id, token, GoalStatus.STATUS_ABORTED)
            return
        record.handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed, rid=robot_id, expected=token: self._goal_result(
                rid, expected, completed
            )
        )

    def _send_spin(self, robot_id: str, angle: float) -> bool:
        """Use Nav2's collision-checked behavior; pose goals ignore yaw here."""
        if robot_id in self.active:
            return False
        client = self.spin_clients[robot_id]
        if not client.server_is_ready():
            return False
        pose = self._world_pose(robot_id)
        world_x, world_y = (pose[0], pose[1]) if pose is not None else (0.0, 0.0)
        request = Spin.Goal()
        request.target_yaw = float(angle)
        request.time_allowance.sec = 30

        self.goal_tokens[robot_id] += 1
        token = self.goal_tokens[robot_id]
        self.active[robot_id] = ActiveGoal(
            token, "merge_scan", world_x, world_y, time.monotonic()
        )
        future = client.send_goal_async(request)
        future.add_done_callback(
            lambda result, rid=robot_id, expected=token: self._goal_response(
                rid, expected, result
            )
        )
        self.get_logger().info(
            f"{robot_id}: collision-checked merge scan {math.degrees(angle):.0f}deg"
        )
        return True

    def _goal_result(self, robot_id: str, token: int, future) -> None:
        try:
            status = int(future.result().status)
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        self._finish_goal(robot_id, token, status)

    def _finish_goal(self, robot_id: str, token: int, status: int) -> None:
        record = self.active.get(robot_id)
        if record is None or record.token != token:
            return
        del self.active[robot_id]
        if (
            record.purpose == "rendezvous"
            and status == GoalStatus.STATUS_SUCCEEDED
        ):
            self.rendezvous_finished.add(robot_id)
        elif record.purpose == "rendezvous":
            # Action discovery becomes ready before a lifecycle action server
            # necessarily becomes active. A goal rejected in that small race
            # must not permanently strand one robot outside the common graph.
            self.rendezvous_retry_after[robot_id] = (
                time.monotonic() + RENDEZVOUS_RETRY_DELAY_S
            )
        elif record.purpose == "merge_scan":
            self.merge_scan_finished.add(robot_id)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.completed_goals[robot_id] += 1
            label = "reached"
        elif status == GoalStatus.STATUS_CANCELED:
            self.cancelled_goals[robot_id] += 1
            label = "cancelled"
        else:
            self.failed_goals[robot_id] += 1
            label = f"failed status={status}"
            if record.purpose == "frontier":
                self.failed_points.append(
                    (record.world_x, record.world_y, time.monotonic())
                )
        self.get_logger().info(f"{robot_id}: {record.purpose} {label}")

    def _cancel(self, robot_id: str, reason: str) -> None:
        record = self.active.pop(robot_id, None)
        if record is None:
            return
        self.goal_tokens[robot_id] += 1
        if record.handle is not None:
            record.handle.cancel_goal_async()
        self.cancelled_goals[robot_id] += 1
        self.get_logger().info(f"{robot_id}: cancelled {record.purpose}: {reason}")

    def _start_rendezvous(self) -> None:
        if len(self.robot_ids) <= 1:
            self.phase = "explore"
            self.phase_since = time.monotonic()
            return
        spacing = 2.0 * max(self.radii.values(), default=0.3) + 0.30
        slots = rendezvous_slots(self.starts, spacing)
        self.rendezvous_targets = slots
        sent = sum(
            self._send_goal(robot_id, slots[robot_id], "rendezvous")
            for robot_id in self.robot_ids
            if robot_id in slots
        )
        if sent:
            self.phase = "rendezvous"
            self.phase_since = time.monotonic()
            self.get_logger().info(
                f"one-time merge rendezvous started: {sent}/{len(self.robot_ids)} goals"
            )

    def _tick_rendezvous(self, now: float) -> None:
        elapsed = now - self.phase_since
        if elapsed <= RENDEZVOUS_TIMEOUT_S:
            for robot_id in self.robot_ids:
                if (
                    robot_id in self.rendezvous_finished
                    or robot_id in self.active
                    or now < self.rendezvous_retry_after[robot_id]
                ):
                    continue
                target = self.rendezvous_targets.get(robot_id)
                if target is not None and self._send_goal(
                    robot_id, target, "rendezvous"
                ):
                    self.get_logger().info(
                        f"{robot_id}: retrying rendezvous after Nav2 rejection"
                    )
                else:
                    self.rendezvous_retry_after[robot_id] = (
                        now + RENDEZVOUS_RETRY_DELAY_S
                    )
        if elapsed > RENDEZVOUS_TIMEOUT_S:
            for robot_id, record in list(self.active.items()):
                if record.purpose == "rendezvous":
                    self._cancel(robot_id, "rendezvous timeout")
                    self.rendezvous_finished.add(robot_id)
        if len(self.rendezvous_finished) == len(self.robot_ids) or (
            elapsed > RENDEZVOUS_TIMEOUT_S and not self.active
        ):
            self._start_merge_scan(now)

    def _start_merge_scan(self, now: float) -> None:
        """Collect explicit common keyframes with bounded merge-only motion.

        A scan rotation is distance-free, but the producer drops in-turn clouds
        to avoid motion skew and therefore emits only one useful final pose. If
        that is not enough for the two-closure PCM gate, the fleet expands its
        ordered line by 1.5 m and, at most, returns once. The order is preserved,
        so recovery adds no crossing conflicts and at most 3 m per robot.
        """
        attempt = self.merge_scan_attempts
        self.merge_scan_attempts += 1
        self.merge_scan_finished.clear()
        if attempt == 0:
            label = "90deg scan rotation"
            sent = sum(
                self._send_spin(robot_id, math.pi / 2.0)
                for robot_id in self.robot_ids
            )
        else:
            centre_x = sum(point[0] for point in self.rendezvous_targets.values()) / len(
                self.rendezvous_targets
            )
            centre_y = sum(point[1] for point in self.rendezvous_targets.values()) / len(
                self.rendezvous_targets
            )
            if attempt == 1:
                targets = {}
                for robot_id, point in self.rendezvous_targets.items():
                    dx, dy = point[0] - centre_x, point[1] - centre_y
                    length = max(math.hypot(dx, dy), 1e-9)
                    targets[robot_id] = (
                        point[0] + 1.5 * dx / length,
                        point[1] + 1.5 * dy / length,
                    )
                label = "ordered 1.5m outward sweep"
            else:
                targets = self.rendezvous_targets
                label = "ordered return sweep"
            sent = sum(
                self._send_goal(robot_id, targets[robot_id], "merge_scan")
                for robot_id in self.robot_ids
                if robot_id in targets
            )
        self.phase = "merge_scan"
        self.phase_since = now
        self.get_logger().info(
            f"merge observation {self.merge_scan_attempts}/{MERGE_SCAN_ATTEMPTS} "
            f"({label}) started: {sent}/{len(self.robot_ids)} goals"
        )

    def _tick_merge_scan(self, now: float) -> None:
        elapsed = now - self.phase_since
        if elapsed > RENDEZVOUS_TIMEOUT_S:
            for robot_id, record in list(self.active.items()):
                if record.purpose == "merge_scan":
                    self._cancel(robot_id, "merge scan timeout")
                    self.merge_scan_finished.add(robot_id)
        if len(self.merge_scan_finished) == len(self.robot_ids) or (
            elapsed > RENDEZVOUS_TIMEOUT_S and not self.active
        ):
            self.phase = "dwell"
            self.phase_since = now
            self.get_logger().info(
                "common-site scan complete; holding still for pose-graph optimization"
            )

    def _merge_status(self) -> tuple[bool | None, list[list[str]]]:
        """Ask the collaborative back-end whether all robot graphs connect."""
        if not self.slam_status_url:
            return None, []
        try:
            with urllib.request.urlopen(self.slam_status_url, timeout=2.0) as response:
                payload = json.loads(response.read())
        except (OSError, ValueError, urllib.error.URLError) as exc:
            self.get_logger().warn(f"could not verify collaborative merge: {exc}")
            return None, []
        components = [
            [str(robot_id) for robot_id in component.get("robots", [])]
            for component in payload.get("components", [])
            if isinstance(component, dict)
        ]
        expected = set(self.robot_ids)
        return any(expected <= set(component) for component in components), components

    def _finish_merge_dwell(self, now: float) -> None:
        merged, components = self._merge_status()
        self.merge_verified = merged
        self.merge_components = components
        if merged is False and self.merge_scan_attempts < MERGE_SCAN_ATTEMPTS:
            self.get_logger().warn(
                f"pose graph is not fully merged yet: {components}; repeating scan"
            )
            self._start_merge_scan(now)
            return
        if merged is False:
            self.get_logger().error(
                f"pose graph still split after merge scans: {components}"
            )
        elif merged is True:
            self.get_logger().info(
                f"verified one collaborative pose-graph component: {components}"
            )
        else:
            self.get_logger().warn(
                "merge status is unavailable; continuing from common observations"
            )
        self.phase = "explore"
        self.phase_since = now
        self.get_logger().info("starting joint frontier allocation")

    def _valid_frontiers(self, common: CommonGrid) -> list[Frontier]:
        frontiers = extract_frontiers(
            common,
            clearance_m=max(self.navigation_clearances.values(), default=0.42),
            gain_radius_m=4.0,
            min_cluster_cells=max(4, int(round(0.35 / common.resolution))),
            min_separation_m=1.75,
        )
        now = time.monotonic()
        self.failed_points = [
            item
            for item in self.failed_points
            if now - item[2] < FAILED_GOAL_COOLDOWN_S
        ]
        return [
            frontier
            for frontier in frontiers
            if not any(
                math.hypot(frontier.x - x, frontier.y - y) < FAILED_GOAL_RADIUS_M
                for x, y, _ in self.failed_points
            )
        ]

    def _tick_explore(self, now: float) -> None:
        if now - self.last_plan_at < PLAN_PERIOD_S:
            return
        self.last_plan_at = now
        # Plan on independent local SLAM maps.  A global-map downlink already
        # contains teammates once their graph connects; merging four copies of
        # that map exaggerates overlap and makes frontier changes self-referential.
        # Nav2 still consumes the optimized global map as its collision map.
        common = self._common_grid(local_only=True)
        if common is None:
            return
        # Small residual differences between four independently rasterized
        # maps can turn one physical wall into parallel occupied/free cells and
        # seal a traversable corridor in the conservative union. Use a second,
        # free-precedence union only for auction reachability. Goals still lie
        # on conservative safe frontiers and Nav2 validates every path against
        # its optimized full-resolution costmap before the robot moves.
        navigation_common = merge_grids(
            self.local_maps,
            self.starts,
            size_m=self.map_size_m,
            resolution=self.resolution,
            occupied_wins=False,
        )
        frontiers = self._valid_frontiers(common)
        self.latest_common, self.latest_frontiers = common, frontiers

        # A teammate may have observed an assigned frontier while this robot was
        # driving. Cancel that now-redundant leg and re-auction the robot.
        for robot_id, record in list(self.active.items()):
            if record.purpose != "frontier":
                continue
            age = now - record.sent_at
            if age > GOAL_TIMEOUT_S:
                self.failed_points.append((record.world_x, record.world_y, now))
                self._cancel(robot_id, "goal timeout")
            elif age > STALE_GOAL_MIN_AGE_S and not frontier_near(
                frontiers,
                (record.world_x, record.world_y),
                STALE_GOAL_RADIUS_M,
            ):
                self._cancel(robot_id, "frontier already observed by teammate")

        idle: list[RobotState] = []
        for robot_id in self.robot_ids:
            if robot_id in self.active:
                continue
            pose = self._world_pose(robot_id)
            if pose is None:
                continue
            idle.append(
                RobotState(
                    robot_id,
                    pose[0],
                    pose[1],
                    self.radii.get(robot_id, 0.3),
                    self.path_m[robot_id],
                    self.navigation_clearances.get(robot_id),
                )
            )
        reserved = [
            (record.world_x, record.world_y)
            for record in self.active.values()
            if record.purpose == "frontier"
        ]
        assignments = allocate_frontiers(
            common,
            idle,
            frontiers,
            reserved=reserved,
            goal_separation_m=2.75,
            navigation_grid=navigation_common,
        )
        for assignment in assignments:
            self._dispatch_assignment(assignment)

        known = int(np.count_nonzero(common.cells >= 0))
        redundant = int(np.count_nonzero(common.observations > 1))
        redundant_fraction = redundant / max(known, 1)
        reconciled_conflicts = int(
            np.count_nonzero(
                (common.cells >= 65)
                & (navigation_common.cells >= 0)
                & (navigation_common.cells <= 20)
            )
        )
        self.get_logger().info(
            f"frontiers={len(frontiers)} active={len(self.active)} "
            f"known={known * common.resolution**2:.1f}m2 "
            f"redundant_cells={redundant_fraction:.1%} "
            f"reachability_conflicts={reconciled_conflicts}"
        )

        if assignments or self.active:
            self.no_frontier_since = 0.0
        elif self.no_frontier_since == 0.0:
            self.no_frontier_since = now
        elif now - self.no_frontier_since >= NO_FRONTIER_DONE_S:
            self.get_logger().info(
                "no reachable frontier remains; exploration complete"
            )
            self.done.set()

        if now - self.last_metrics_at >= 15.0:
            self.last_metrics_at = now
            if self.merge_verified is not True:
                merged, components = self._merge_status()
                self.merge_verified = merged
                self.merge_components = components
                if merged:
                    self.get_logger().info(
                        "verified collaborative merge during exploration"
                    )
            self._persist_metrics()

    def _dispatch_assignment(self, assignment: Assignment) -> None:
        self.get_logger().info(
            f"allocate {assignment.robot_id} -> frontier "
            f"({assignment.frontier.x:.2f}, {assignment.frontier.y:.2f}), "
            f"gain={assignment.frontier.information_m2:.1f}m2, "
            f"path={assignment.path_cost_m:.1f}m"
        )
        self._send_goal(
            assignment.robot_id,
            (assignment.frontier.x, assignment.frontier.y),
            "frontier",
        )

    def tick(self) -> None:
        if not self.running:
            return
        now = time.monotonic()
        self._update_paths()
        if self.phase == "waiting":
            if self._ready():
                self._start_rendezvous()
            elif now - self._waiting_reported_at > 10.0:
                self._waiting_reported_at = now
                missing_maps = [
                    rid
                    for rid in self.robot_ids
                    if rid not in self.local_maps or rid not in self.global_maps
                ]
                missing_tf = [
                    rid for rid in self.robot_ids if self._local_pose(rid) is None
                ]
                missing_nav = [
                    rid
                    for rid, client in self.nav_clients.items()
                    if not client.server_is_ready()
                    or not self.spin_clients[rid].server_is_ready()
                ]
                self.get_logger().info(
                    f"waiting: maps={missing_maps} slam_tf={missing_tf} "
                    f"nav2={missing_nav}"
                )
        elif self.phase == "rendezvous":
            self._tick_rendezvous(now)
        elif self.phase == "merge_scan":
            self._tick_merge_scan(now)
        elif self.phase == "dwell":
            if now - self.phase_since >= RENDEZVOUS_DWELL_S:
                self._finish_merge_dwell(now)
        elif self.phase == "explore":
            self._tick_explore(now)

    def _metrics(self) -> dict:
        local_common = self._common_grid(local_only=True)
        known_area = 0.0
        redundant_fraction = 0.0
        per_robot_known_area: dict[str, float] = {}
        per_robot_unique_area: dict[str, float] = {}
        if local_common is not None:
            known = int(np.count_nonzero(local_common.cells >= 0))
            redundant = int(np.count_nonzero(local_common.observations > 1))
            known_area = known * local_common.resolution**2
            redundant_fraction = redundant / max(known, 1)
            for robot_id in self.robot_ids:
                if robot_id not in self.local_maps:
                    continue
                alone = merge_grids(
                    {robot_id: self.local_maps[robot_id]},
                    self.starts,
                    size_m=self.map_size_m,
                    resolution=self.resolution,
                )
                per_robot_known_area[robot_id] = round(
                    float(np.count_nonzero(alone.cells >= 0) * alone.resolution**2), 3
                )
                per_robot_unique_area[robot_id] = round(
                    float(
                        np.count_nonzero(
                            (alone.cells >= 0) & (local_common.observations == 1)
                        )
                        * alone.resolution**2
                    ),
                    3,
                )
        aggregate_path = sum(self.path_m.values())
        return {
            "strategy": "coordinated_frontier",
            "phase": self.phase,
            "wall_time_s": round(time.monotonic() - self.started_at, 3),
            "merge_verified": self.merge_verified,
            "merge_components": self.merge_components,
            "robots": {
                robot_id: {
                    "path_m": round(self.path_m[robot_id], 3),
                    "known_area_m2": per_robot_known_area.get(robot_id, 0.0),
                    "unique_contribution_m2": per_robot_unique_area.get(robot_id, 0.0),
                    "goals_completed": self.completed_goals[robot_id],
                    "goals_failed": self.failed_goals[robot_id],
                    "goals_cancelled": self.cancelled_goals[robot_id],
                }
                for robot_id in self.robot_ids
            },
            "aggregate_path_m": round(aggregate_path, 3),
            "known_union_m2": round(known_area, 3),
            "coverage_efficiency_m2_per_m": round(
                known_area / max(aggregate_path, 1e-9), 3
            ),
            "redundant_known_fraction": round(redundant_fraction, 6),
            "frontiers_remaining": len(self.latest_frontiers),
        }

    def halt(self) -> None:
        if not self.running:
            return
        self.running = False
        for robot_id in list(self.active):
            self._cancel(robot_id, "exploration stopping")
        for publisher in self.stop_publishers.values():
            publisher.publish(Twist())
        metrics = self._metrics()
        self.get_logger().info(
            f"exploration metrics: {json.dumps(metrics, sort_keys=True)}"
        )
        self._persist_metrics(metrics)

    def _persist_metrics(self, metrics: dict | None = None) -> None:
        if not self.metrics_path:
            return
        try:
            path = Path(self.metrics_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(metrics or self._metrics(), indent=2, sort_keys=True) + "\n"
            )
            os.replace(temporary, path)
        except OSError as exc:
            self.get_logger().warn(f"could not write metrics: {exc}")


def _pose_dict(value: str) -> dict[str, tuple[float, float, float]]:
    result: dict[str, tuple[float, float, float]] = {}
    if not value:
        return result
    try:
        for robot_id, pose in json.loads(value).items():
            result[robot_id] = (
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("yaw", 0.0)),
            )
    except (ValueError, TypeError, AttributeError):
        return {}
    return result


def _float_dict(value: str) -> dict[str, float]:
    if not value:
        return {}
    try:
        return {robot_id: float(item) for robot_id, item in json.loads(value).items()}
    except (ValueError, TypeError, AttributeError):
        return {}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, default=4)
    parser.add_argument("--prefix", default="robot_")
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--start-poses", default="")
    parser.add_argument("--radii", default="")
    parser.add_argument("--navigation-clearances", default="")
    parser.add_argument("--map-size-m", type=float, default=30.0)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--metrics", default="")
    parser.add_argument("--slam-status-url", default="http://slam:8090/status")
    args = parser.parse_args()

    robot_ids = [f"{args.prefix}{index}" for index in range(args.robots)]
    starts = {
        robot_id: pose
        for robot_id, pose in _pose_dict(args.start_poses).items()
        if robot_id in robot_ids
    }
    if len(starts) != len(robot_ids):
        missing = [robot_id for robot_id in robot_ids if robot_id not in starts]
        raise SystemExit(
            "coordinated exploration requires configured start poses; "
            f"missing {missing}"
        )
    radii = {
        robot_id: radius
        for robot_id, radius in _float_dict(args.radii).items()
        if robot_id in robot_ids
    }
    for robot_id in robot_ids:
        radii.setdefault(robot_id, 0.30)
    navigation_clearances = {
        robot_id: clearance
        for robot_id, clearance in _float_dict(args.navigation_clearances).items()
        if robot_id in robot_ids
    }
    for robot_id in robot_ids:
        navigation_clearances.setdefault(robot_id, radii[robot_id] + 0.12)

    rclpy.init()
    node = CoordinatedExplorer(
        robot_ids,
        starts=starts,
        radii=radii,
        navigation_clearances=navigation_clearances,
        map_size_m=args.map_size_m,
        resolution=args.resolution,
        metrics_path=args.metrics,
        slam_status_url=args.slam_status_url,
    )
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    deadline = time.monotonic() + max(0.0, args.seconds)
    try:
        while time.monotonic() < deadline and not node.done.wait(timeout=0.5):
            pass
    except KeyboardInterrupt:
        pass
    node.halt()
    time.sleep(0.5)
    rclpy.shutdown()
    thread.join(timeout=2.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
