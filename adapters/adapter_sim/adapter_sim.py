#!/usr/bin/env python3
"""Simulation adapter: ROS 2 fleet -> SwarmDeck adapter protocol.

One process bridges every simulated robot. This is the ONLY place in the
simulation path that knows about both ROS and the SwarmDeck protocol — the
backend stays ROS-free.

    ros2 run swarmdeck_sim adapter_sim --robots 4
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import importlib
import json
import math
import os
import sys
import threading
from contextlib import nullcontext
import time
import urllib.parse
import urllib.request
import zlib
from pathlib import Path

import rclpy
import websockets
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Keep perception reusable by real adapters without packaging it into ROS.
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from adapters.perception.depth_projection import point_for_depth_image
from adapters.perception.object_detector import ObjectDetector, track_ids
from adapters.runtime import (
    AdapterDetectionMixin,
    AdapterGoalOwnershipMixin,
    AdapterHelloMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
    TRANSPORT_DEFAULTS,
    deep_merge,
    stamp_seconds,
    yaw_of,
)
from adapters.session import run_adapter_session
from adapters.navigation_result import navigation_failure_reason
from adapters.route_progress import route_progress_tick

from sim_cslam import (
    SLAM_GRAPHS,
    on_slam_graph as _on_slam_graph,
    slam_graph_payload,
)

# The platform table, imported from the spawner rather than restated here.
#
# This adapter has to know its own footprint (it reports it at `hello`, and the
# GUI draws robots at that size). That is a fact about the model the simulator
# was handed, and a second copy of it would be wrong the first time a chassis
# changed — the same argument session.launch.py already makes by importing
# lidar_spec from here instead of re-reading the YAML.
sys.path.insert(0, str(REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim" / "scenario"))
from spawn_fleet import (  # noqa: E402
    DEFAULT_ROBOT_PROFILE,
    lidar_spec,
    robot_spec,
    robot_types,
)

# Depth acceptance band for placing a detection on the map, metres. The same
# figures the hardware adapters default to (`perception.depth_min_m` /
# `depth_max_m`): below the near limit the depth camera sees the robot's own
# body, and beyond the far one a box a few pixels wide covers metres of room.
DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 8.0
# How far apart the colour frame and the depth frame may be stamped before the
# pair is refused, seconds. Both come from one `rgbd_camera` at one rate, so
# this is not a synchronisation tolerance — it is what stops a frozen depth
# stream projecting a live detection onto stale geometry.
DEPTH_MAX_AGE_S = 0.35

# Voxel edge for downsampling the 3D map before upload, metres. Coarser than the
# 5 cm occupancy grid on purpose: this feeds a view whose points are one pixel.
CLOUD_VOXEL = 0.10

# Transport quantisation. 1 cm keeps a cloud well inside int16 and is far finer
# than the voxel above, so it costs nothing in fidelity.
CLOUD_SCALE = 0.01


def resolve_sim_robot_count(
    cli_robots: int | None,
    env_robots: str = "",
    config_count: int | None = None,
    default: int = 4,
) -> int:
    """How many simulated robots this process should bridge.

    Dashboard ``robot_count`` is a hardware-session setting (tars/botman/aslan)
    and must not inflate a 2-robot Gazebo fleet. Prefer the CLI, then
    ``SWARMDECK_ROBOT_COUNT``, then the YAML that spawned the world.
    """
    if cli_robots is not None:
        count = int(cli_robots)
    else:
        env = (env_robots or "").strip()
        if env:
            count = int(env)
        elif config_count is not None:
            count = int(config_count)
        else:
            count = int(default)
    return max(1, min(count, 5))


def camera_point_to_map(
    point, pose: dict[str, float], camera_x: float, camera_z: float
) -> dict[str, float] | None:
    """Place one camera-frame XYZ sample in the robot's map frame.

    Two rigid steps, composed here rather than looked up through tf2, for the
    same reason `map_pose()` composes its own chain: this adapter is reading a
    tree we built. The camera mount comes from the ROBOT_PROFILES entry that
    generated the model's SDF, so the geometry used here and the geometry
    Gazebo rendered from cannot drift apart the way a hand-copied extrinsic
    would. On a real robot both steps are a tf2 lookup instead — see
    adapter_ros2._depth_map_position, and the warning in its map_pose().

    1. Optical -> base_link. `point_for_depth_image` returns the ROS camera
       optical convention (x right, y down, z forward along the boresight); the
       robot's own frame is x forward, y left, z up, and the camera is bolted to
       it looking straight ahead from `(camera_x, 0, camera_z)`.
    2. base_link -> map, by the robot's SE(2) pose.

    Height is then dropped, because the protocol's `map_position` is a point on
    a 2D map. It is deliberately not used to filter: a duck on a table is still
    a duck at that (x, y), and this is not the place to decide what a detection
    is allowed to be sitting on.
    """
    try:
        right, down, forward = (float(value) for value in point)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (right, down, forward)):
        return None

    base_x = camera_x + forward
    base_y = -right
    cos_yaw, sin_yaw = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    return {
        "x": round(pose["x"] + base_x * cos_yaw - base_y * sin_yaw, 3),
        "y": round(pose["y"] + base_x * sin_yaw + base_y * cos_yaw, 3),
    }


# --------------------------------------------------------------- wedge escape
#
# Nav2's controller can wedge when the footprint overlaps a local obstacle.
# A bounded reverse leaves the robot able to accept the next command without
# making a second planning or mapping authority.
#
# So the escape lives here, outside Nav2. It reverses, and ONLY reverses,
# because the ground immediately behind is the ground the robot just drove over
# — the one direction known to be clear without trusting the map that is wrong.
# Rotating in place would not do: turning sweeps the footprint out to the
# circumscribed radius (0.422 m on the Scout), which is further than the jamb it
# is trying to escape.
#
# Deliberately bounded and deliberately not a retry loop. It backs off once per
# failed goal, and the goal still reports `failed` to the operator — recovering
# the robot's ability to accept the NEXT command is the whole objective, not
# quietly re-attempting a command that already failed.

ESCAPE_SPEED = -0.15  # m/s, reverse slowly during recovery.
ESCAPE_DISTANCE = 0.45  # m of retreat before the escape is considered done
# Wall-clock, and the retreat happens in simulation time, so this has to allow
# for the real-time factor as well as for a robot that creeps. Measured on the
# Scout Mini: 0.15 m/s commanded came out as ~0.046 m/s of ground covered, so 8 s
# ended the escape on the timeout at 0.376 m instead of on the distance it was
# asked for. The timeout is meant to be the backstop, not the usual exit.
ESCAPE_TIMEOUT_S = 15.0
# Below this the robot is not moving despite being commanded, so it is pinned on
# real geometry rather than on a mapping error, and reversing is not the answer.
ESCAPE_STALL_S = 2.5
ESCAPE_STALL_DISTANCE = 0.05

# ------------------------------------------------------- Nav2 bringup recovery
#
# Simulation launches the bounded, state-aware navigation_startup owner, not
# Nav2's lifecycle manager. Recovery must reach that same owner: a manager
# service request here would never answer. It confirms actual lifecycle state
# before every transition, preserving nodes already active after a lost reply.

# Long enough that a healthy stack finishes its own bringup untouched — measured
# at 8-12 s from process start on this machine — with margin for a loaded one.
NAV_READY_GRACE_S = 30.0
# A re-bringup that did not take is nearly always a node still starting, so
# leave real time between attempts rather than hammering the service.
NAV_RECOVER_INTERVAL_S = 30.0
# The owner has a 60 s shared bringup deadline; allow its initial startup to
# finish before a queued recovery, plus the recovery's own bounded attempt.
NAV_RECOVER_TIMEOUT_S = 125.0
# Per-service discovery wait inside a bounded recovery: short, so one absent
# service cannot consume the whole shared deadline.
LIFECYCLE_SERVICE_TIMEOUT_S = 2.0

SERVICE_TIMEOUT_S = 8.0


def _srv_type(module: str, name: str):
    try:
        return getattr(importlib.import_module(module), name)
    except (ImportError, AttributeError):
        return None


class RobotBridge(
    AdapterHelloMixin,
    AdapterDetectionMixin,
    AdapterGoalOwnershipMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
):
    """Per-robot ROS subscriptions and command publisher."""

    adapter_name = "adapter_sim/0.1.0"
    coordinate_frame = "local"
    # Nav2 controls the simulated robot directly, so do not claim that a
    # replacement route is executing while cancellation settles.
    goal_pending_status = "idle"
    _TRACK_IDS = staticmethod(track_ids)

    def __init__(
        self,
        node: Node,
        robot_id: str,
        http_url: str,
        platform: str | None = None,
        *,
        instantaneous_planar_scan: bool = False,
        exploration_config: dict | None = None,
        planning_config: dict | None = None,
    ) -> None:
        self.node = node
        self.id = robot_id
        self.navigation_frame = f"{robot_id}/odom"
        self.http_url = http_url
        self.t0 = time.monotonic()
        # What this robot IS. `footprint_radius` is not decoration: the GUI draws
        # the robot at that size and the operator judges clearances by it, so a
        # 1.02 m Bunker reporting the old hardcoded 0.3 m would be drawn at less
        # than half its width.
        self.platform = platform or DEFAULT_ROBOT_PROFILE
        spec = robot_spec(self.platform)
        self.cfg = deep_merge(
            TRANSPORT_DEFAULTS,
            {
                "robot_type": spec.robot_type,
                "ros_distro": "jazzy",
                "footprint_radius": round(spec.footprint_radius, 3),
                "footprint": json.loads(spec.footprint),
                "network_iface": "",
                "exploration": exploration_config or {},
                "planning": planning_config or {},
            },
        )
        self.robot_type = self.cfg["robot_type"]
        self.footprint_radius = self.cfg["footprint_radius"]
        self.footprint = self.cfg["footprint"]
        # Where this platform's RGBD camera is bolted, from the same table the
        # SDF was rendered from. Turning a duck detection into a map marker is
        # the only thing that reads it. See camera_point_to_map().
        self.camera_x = spec.camera_x
        self.camera_z = spec.camera_z
        self.lidar_x = spec.lidar_x
        self.lidar_z = spec.lidar_z
        self._scan_cloud_at = 0.0
        self.battery = None

        self._odom_to_base: dict[str, float] | None = None
        self._odom_to_base_log: deque[tuple[float, dict[str, float]]] = deque(
            maxlen=128
        )
        self._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._warned_no_tf_base = False
        self.goal: dict | None = None
        self.planned_path: list[dict[str, float]] = []
        self.nav_status = "idle"
        self._nav_failure_reason: str | None = None
        self.mode = "idle"
        self._camera_frame: Image | None = None
        self._camera_dirty = False
        self._camera_encoding_warned = False
        # The depth half of the same sensor, plus its intrinsics. Kept as the
        # newest message rather than queued: detection runs off the colour frame
        # and only ever wants the depth taken with it.
        self._camera_depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._last_depth_warning_at = 0.0
        # The simulation runs without a detector unless `sim-up --detector`
        # started one and set its URL; nothing then posts frames anywhere.
        self._detector = (
            ObjectDetector() if os.environ.get("SWARMDECK_DETECTOR_URL") else None
        )
        self._detection_enabled = True
        default_period = float(os.environ.get("SWARMDECK_DETECTION_PERIOD_S", "1.0"))
        self._detection_period_s = max(
            0.05,
            float((self.cfg.get("rates") or {}).get("camera_period_s", default_period)),
        )
        self._last_detection_at = 0.0
        self._detections: list[dict] | None = None
        self._goal_handle = None
        self._goal_generation = 0
        self._goal_lock = threading.RLock()
        self._goal_request_future = None
        self._goal_request_generation = None
        self._cancel_events: dict[int, threading.Event] = {}
        self._nav_quiet_unknown = False
        # The FollowPath handle the route progress watchdog cancelled. Its late
        # CANCELED result must not overwrite the failure recorded for it.
        self._route_stalled_handle = None
        self._last_drive_at = 0.0
        # Wedge escape — see ESCAPE_SPEED. `_escape_from` is the pose the failed
        # goal ended at, which is what the retreat is measured against.
        self._escape_from: tuple[float, float] | None = None
        self._escape_started_at = 0.0
        self._escape_progress_at = 0.0
        # Nav2 bringup recovery — see NAV_READY_GRACE_S.
        self._nav_down_since = 0.0
        self._nav_recovered_at = 0.0
        # Held for the whole of reset(), and tried without blocking by every
        # upload. That ordering is what stops a grid captured before the reset
        # reaching the backend after it: an upload already running finishes
        # first, and one that starts during the reset is skipped.
        self._upload_lock = threading.Lock()
        self._service_clients: dict = {}
        self._reset_report: dict | None = None

        node.create_subscription(Odometry, f"/{robot_id}/odom", self._on_odom, 10)
        # Depth chosen for the executor, not the publisher. TF arrives at 10 Hz,
        # but this node does the cloud work in the same executor, so under load
        # the callback is starved and a shallow queue silently drops the samples
        # a pose lookup then cannot find. Measured with a depth of 20: lookups
        # reaching 100 to 500 ms past the stamp they asked for, and the rotated
        # copy in the merged map tracked how far they reached.
        node.create_subscription(TFMessage, f"/{robot_id}/tf", self._on_tf, 200)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(NavPath, f"/{robot_id}/plan", self._on_plan, 10)
        # The independent media process subscribes to RGB for dashboard video.
        # Only perception needs these three streams in the adapter itself.
        if self._detector is not None:
            node.create_subscription(
                Image,
                f"/{robot_id}/camera/image",
                self._on_camera,
                qos_profile_sensor_data,
            )
            node.create_subscription(
                Image,
                f"/{robot_id}/camera/depth_image",
                self._on_camera_depth,
                qos_profile_sensor_data,
            )
            node.create_subscription(
                CameraInfo,
                f"/{robot_id}/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )

        self.path_client = ActionClient(node, FollowPath, f"/{robot_id}/follow_path")
        self.pub_cmd = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
        from adapters.exploration import configure_exploration
        from adapters.objective_planning import configure_objective_planning

        configure_exploration(self)
        configure_objective_planning(self)

    def _on_odom(self, msg: Odometry) -> None:
        """Wheel odometry — a FALLBACK only. See map_pose() for why."""
        p = msg.pose.pose
        self._odom_topic_pose = {
            "x": p.position.x,
            "y": p.position.y,
            "z": p.position.z,
            "yaw": yaw_of(p.orientation),
        }

    def _on_tf(self, msg: TFMessage) -> None:
        """Track only the continuous odometry-to-base transform."""
        odom_frame = f"{self.id}/odom"
        base_frame = f"{self.id}/base_link"
        for stamped in msg.transforms:
            if (
                stamped.header.frame_id != odom_frame
                or stamped.child_frame_id != base_frame
            ):
                continue
            t = stamped.transform
            self._odom_to_base = {
                "x": t.translation.x,
                "y": t.translation.y,
                "z": t.translation.z,
                "yaw": yaw_of(t.rotation),
            }

    def map_pose(self) -> dict[str, float]:
        pose = self._odom_to_base or self._odom_topic_pose
        if self._odom_to_base is None and not self._warned_no_tf_base:
            self._warned_no_tf_base = True
            self.node.get_logger().warn(
                f"[{self.id}] no {self.id}/odom -> {self.id}/base_link TF; using odometry topic"
            )
        return dict(pose)

    def _on_camera(self, msg: Image) -> None:
        self._camera_frame = msg
        self._camera_dirty = True

    def _on_camera_depth(self, msg: Image) -> None:
        self._camera_depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _depth_map_position(
        self, bbox, image_header=None, polygon=None
    ) -> dict[str, float] | None:
        """Where a detection box is in this robot's map frame — or nothing.

        Fail-closed at every step. An absent `map_position` costs a marker on
        the map; an invented one puts a duck where there is none, and the
        operator has no way to tell those two apart. The hardware adapters make
        the same choice — see adapters/adapter_ros2/adapter_ros2.py.
        """
        depth, info = self._camera_depth, self._camera_info
        if depth is None or info is None:
            self._warn_depth("no depth image or intrinsics yet")
            return None

        # Both streams come off one sensor, so a gap here means one of them has
        # stopped rather than that the two need aligning.
        image_time = stamp_seconds(image_header)
        depth_time = stamp_seconds(getattr(depth, "header", None))
        if (
            image_time is not None
            and depth_time is not None
            and abs(image_time - depth_time) > DEPTH_MAX_AGE_S
        ):
            self._warn_depth(
                f"depth is {abs(image_time - depth_time):.2f}s from the colour frame"
            )
            return None

        camera_point = point_for_depth_image(
            depth,
            info,
            bbox,
            polygon=polygon,
            min_range_m=DEPTH_MIN_M,
            max_range_m=DEPTH_MAX_M,
        )
        if camera_point is None:
            return None
        return camera_point_to_map(
            camera_point, self.map_pose(), self.camera_x, self.camera_z
        )

    def _warn_depth(self, reason: str) -> None:
        """Say why detections are not reaching the map, at most every 10 s."""
        now = time.monotonic()
        if now - self._last_depth_warning_at < 10.0:
            return
        self._last_depth_warning_at = now
        self.node.get_logger().warn(
            f"[{self.id}] detections cannot be placed on the map: {reason}"
        )

    def _on_plan(self, msg: NavPath) -> None:
        """Keep a bounded representation of Nav2's latest global plan."""
        poses = msg.poses
        if not poses:
            self.planned_path = []
            return
        stride = max(1, math.ceil(len(poses) / 120))
        sampled = poses[::stride]
        if sampled[-1] is not poses[-1]:
            sampled = [*sampled, poses[-1]]
        self.planned_path = [
            {"x": float(item.pose.position.x), "y": float(item.pose.position.y)}
            for item in sampled
        ]

    # -- protocol side -------------------------------------------------

    def capabilities(self) -> list[str]:
        """Advertise only what this process honours. `reset` is simulation-only."""
        caps = ["navigate", "map", "camera", "estop", "reset"]
        if getattr(self, "exploration", None) is not None:
            caps.append("explore")
        if getattr(self, "objective_planner", None) is not None:
            caps.append("plan_objective")
        return caps

    def _cfg_timeout(self, key: str) -> float:
        cfg = getattr(self, "cfg", None) or TRANSPORT_DEFAULTS
        return float(cfg[key])

    def plan_objective(self, objective: str, goal: dict | None = None) -> bool:
        planner = getattr(self, "objective_planner", None)
        if planner is None:
            return False
        return planner.plan(objective, goal or {})

    def return_home(self, goal: dict | None = None) -> bool:
        planner = getattr(self, "objective_planner", None)
        return bool(planner and planner.return_home(goal))

    def follow_path(
        self,
        plan,
        expected_generation: int | None = None,
        not_after: float | None = None,
        pre_submit=None,
    ) -> int | bool | None:
        """Execute every MGG waypoint through Nav2's controller server."""
        if not plan.poses:
            return False
        if expected_generation is not None:
            with self._goal_lock:
                if expected_generation != self._goal_generation:
                    return None
                if not_after is not None and time.monotonic() >= not_after:
                    return None
        if not self.path_client.server_is_ready():
            self.node.get_logger().error(
                f"[{self.id}] Nav2 FollowPath action server is not ready"
            )
            if expected_generation is None:
                self.goal = None
                self.nav_status, self.mode = "failed", "idle"
            return False

        from adapters.exploration import follow_path_goal

        request = follow_path_goal(plan)
        final = plan.poses[-1]
        yaw = math.atan2(
            2.0 * (final.qw * final.qz + final.qx * final.qy),
            1.0 - 2.0 * (final.qy * final.qy + final.qz * final.qz),
        )
        with self._goal_lock:
            if not_after is not None and time.monotonic() >= not_after:
                return None
            if (
                expected_generation is not None
                and expected_generation != self._goal_generation
            ):
                return None
            if pre_submit is not None and not pre_submit():
                return None
            if expected_generation is None:
                self._cancel_nav()
            generation = self._goal_generation
            try:
                future = self.path_client.send_goal_async(request)
            except Exception as exc:
                self.node.get_logger().error(
                    f"[{self.id}] path submission failed: {exc}"
                )
                if expected_generation is None:
                    self.nav_status, self.mode = "failed", "idle"
                else:
                    self.goal = None
                    self.planned_path = []
                    self.nav_status, self.mode = "idle", "idle"
                return False
            self._goal_request_future = future
            self._goal_request_generation = generation
            self.goal = {
                "x": final.x,
                "y": final.y,
                "z": final.z,
                "yaw": yaw,
                "frame_id": plan.frame_id,
            }
            self.planned_path = [{"x": pose.x, "y": pose.y} for pose in plan.poses]
            self._follow_path_display = (generation, plan)
            self._reset_route_watchdog()
            self.nav_status, self.mode = "active", "nav"
            future.add_done_callback(lambda done: self._goal_response(done, generation))
            return generation

    def _goal_response(self, future, generation: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            with self._goal_lock:
                if self._goal_request_generation == generation:
                    self._goal_request_future = None
                    self._goal_request_generation = None
                self._nav_quiet_unknown = True
                if generation != self._goal_generation:
                    return
                self.node.get_logger().error(f"[{self.id}] goal request failed: {exc}")
                self._finish_goal("failed", generation)
            return
        with self._goal_lock:
            if self._goal_request_generation == generation:
                self._goal_request_future = None
                self._goal_request_generation = None
            if generation != self._goal_generation:
                # A cancel can arrive before Nav2 accepts the request. Cancel
                # the resulting handle instead of ignoring its callbacks.
                if handle.accepted:
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                    try:
                        result = handle.get_result_async()
                        result.add_done_callback(
                            lambda done: self._goal_result(done, generation, handle)
                        )
                    except Exception:
                        self._nav_quiet_unknown = True
                else:
                    event = self._cancel_events.get(generation)
                    if event is not None:
                        event.set()
                return
            if not handle.accepted:
                self.node.get_logger().warn(f"[{self.id}] navigation goal rejected")
                self._finish_goal("failed", generation)
                return

            self._goal_handle = handle
            exploration = getattr(self, "exploration", None)
            if exploration is not None:
                exploration.controller_accepted(generation)
            try:
                result = handle.get_result_async()
                result.add_done_callback(
                    lambda done: self._goal_result(done, generation, handle)
                )
            except Exception:
                self._nav_quiet_unknown = True
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
                self._finish_goal("failed", generation)

    def _goal_result(self, future, generation: int, handle=None) -> None:
        with self._goal_lock:
            if handle is not None and handle is getattr(
                self, "_route_stalled_handle", None
            ):
                # The route progress watchdog already recorded this goal's
                # failure and cancelled it. Its generation is still current,
                # possibly with a replacement route in flight, so only settle
                # the cancellation rather than relabel the outcome.
                self._route_stalled_handle = None
                try:
                    status = future.result().status
                except Exception:
                    self._nav_quiet_unknown = True
                    return
                if status not in {
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                }:
                    self._nav_quiet_unknown = True
                return
            if generation != self._goal_generation:
                try:
                    status = future.result().status
                except Exception:
                    self._nav_quiet_unknown = True
                    return
                if status in {
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                }:
                    event = self._cancel_events.get(generation)
                    if event is not None:
                        event.set()
                else:
                    self._nav_quiet_unknown = True
                return
            try:
                outcome = future.result()
                status = outcome.status
            except Exception as exc:
                self._nav_quiet_unknown = True
                self.node.get_logger().error(
                    f"[{self.id}] navigation result failed: {exc}"
                )
                self._finish_goal("failed", generation)
                return

            terminal = {
                GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_CANCELED: "cancelled",
                GoalStatus.STATUS_ABORTED: "failed",
            }.get(status, "failed")
            result = getattr(outcome, "result", None)
            if status not in {
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            }:
                self._nav_quiet_unknown = True
            self._finish_goal(
                terminal,
                generation,
                reason=navigation_failure_reason(result),
            )

    def _finish_goal(
        self, status: str, generation: int, *, reason: str | None = None
    ) -> None:
        if generation != self._goal_generation:
            return
        self._goal_handle = None
        self.goal = None
        self.planned_path = []
        self.nav_status, self.mode = status, "idle"
        self._nav_failure_reason = reason if status == "failed" else None
        self._reset_route_watchdog()
        if status == "failed" and not self._nav_quiet_unknown:
            self._arm_escape()
        exploration = getattr(self, "exploration", None)
        if exploration is not None:
            exploration.controller_finished()

    # -- route progress watchdog ------------------------------------------
    #
    # Nav2's SimpleProgressChecker measures displacement, and a robot rocking
    # 0.3 m back and forth on a 7 cm ridge satisfies it forever (measured in
    # the Bistro world, 2026-09-17: three minutes with nav_status active and
    # no failure raised, so no recovery ever ran). The shared watchdog in
    # adapters/route_progress.py measures progress along the route instead.

    def _reset_route_watchdog(self) -> None:
        watchdog = getattr(self, "_route_watchdog", None)
        if watchdog is not None:
            watchdog.reset()

    def _route_progress_pose(self, frame) -> dict[str, float] | None:
        """The robot's pose in the route's frame.

        Routes are planned in the odometry frame (the MGG planning frame) or
        in the map frame. This bridge composes its pose from named TF links
        rather than a tf2 buffer: the odometry frame is the ``odom ->
        base_link`` link itself, with the wheel topic as the same fallback
        ``map_pose`` uses, and the map frame is that link under the map
        correction. A route in any other frame is left unsupervised.
        """
        name = str(frame or "").lstrip("/")
        if name == f"{self.id}/odom":
            base = self._odom_to_base
            return base if base is not None else self._odom_topic_pose
        if name == self.navigation_frame.lstrip("/"):
            return self.map_pose()
        return None

    def _fail_route_progress(self, generation: int, reason: str) -> bool:
        """Retire the route as the controller no-progress failure it is.

        The goal generation stays current, exactly as after Nav2's own
        ``Failed to make progress`` abort, so the exploration and objective
        recovery paths keep their ownership of it and can submit a replacement
        route. The cancelled handle is remembered so its late result cannot
        relabel this failure.
        """
        with self._goal_lock:
            if generation != self._goal_generation or self.nav_status != "active":
                return False
            handle = self._goal_handle
            if handle is None:
                return False
            try:
                handle.cancel_goal_async()
            except Exception:
                self._nav_quiet_unknown = True
            self._route_stalled_handle = handle
            self.node.get_logger().warn(
                f"[{self.id}] {reason}; cancelling the route as a controller "
                "no-progress failure"
            )
            self.pub_cmd.publish(Twist())
            self._finish_goal("failed", generation, reason=reason)
            return True

    # -- Nav2 bringup recovery ------------------------------------------

    def navigation_ready(self) -> bool:
        """Whether the action servers and configured objective planner are live."""
        actions_ready = bool(self.path_client.server_is_ready())
        planner = getattr(self, "objective_planner", None)
        planner_client = getattr(planner, "client", None)
        return actions_ready and (
            planner_client is None or planner_client.service_is_ready()
        )

    def recover_nav_if_down(self) -> bool:
        """Ask the launched lifecycle owner to recover missing action servers.

        Runs on a worker thread while the ROS thread completes service futures.
        A successful response means every managed node was observed active;
        action discovery is checked again on the next health cycle.
        """
        now = time.monotonic()
        if self.path_client.server_is_ready():
            self._nav_down_since = 0.0
            return False
        if self._nav_down_since == 0.0:
            # First cycle that noticed. Start the clock rather than acting: at
            # this point a perfectly healthy stack is simply still starting.
            self._nav_down_since = now
            return False
        if now - self._nav_down_since < NAV_READY_GRACE_S:
            return False
        if (
            self._nav_recovered_at
            and now - self._nav_recovered_at < NAV_RECOVER_INTERVAL_S
        ):
            return False

        self._nav_recovered_at = now
        self.node.get_logger().warn(
            f"[{self.id}] Nav2 action server absent for "
            f"{now - self._nav_down_since:.0f} s; re-running lifecycle bringup"
        )
        trigger = _srv_type("std_srvs.srv", "Trigger")
        try:
            if trigger is None:
                raise RuntimeError("std_srvs.srv.Trigger is not installed")
            response = self._lifecycle_service_response(
                f"/{self.id}/navigation_startup/recover",
                trigger,
                trigger.Request(),
                now + NAV_RECOVER_TIMEOUT_S,
            )
            if not response.success:
                raise RuntimeError(response.message)
        except (TimeoutError, RuntimeError) as exc:
            self.node.get_logger().error(
                f"[{self.id}] navigation recovery failed: {exc}"
            )
            return False
        return True

    # -- wedge escape --------------------------------------------------

    def _arm_escape(self) -> None:
        """Begin reversing out of a pose Nav2 could not plan from.

        for the failures this does not apply to, and it is the only thing that
        helps for the one it does.
        """
        pose = self.map_pose()
        self._escape_from = (pose["x"], pose["y"])
        self._escape_started_at = self._escape_progress_at = time.monotonic()
        self.mode = "recover"

    def _clear_escape(self) -> None:
        if self._escape_from is not None:
            self._escape_from = None
            self.pub_cmd.publish(Twist())
            if self.mode == "recover":
                self.mode = "idle"

    def escape_tick(self) -> None:
        """Drive one cycle of the escape. Called from the adapter's main loop.

        Ends on any of: enough retreat, the timeout, or the robot failing to
        make progress — the last meaning it is pinned on real geometry, where
        reversing harder is the wrong answer and an operator needs to see a
        stopped robot rather than one grinding against a wall.
        """
        with self._goal_lock:
            escape_from = self._escape_from
            generation = self._goal_generation
            if escape_from is None:
                return
        pose = self.map_pose()
        moved = math.hypot(pose["x"] - escape_from[0], pose["y"] - escape_from[1])
        now = time.monotonic()

        if moved >= ESCAPE_DISTANCE or now - self._escape_started_at > ESCAPE_TIMEOUT_S:
            self._clear_escape()
            return
        if moved > ESCAPE_STALL_DISTANCE:
            self._escape_progress_at = now
        elif now - self._escape_progress_at > ESCAPE_STALL_S:
            self.node.get_logger().warn(
                f"[{self.id}] wedge escape made no progress in "
                f"{ESCAPE_STALL_S:.1f} s; the robot is against real geometry"
            )
            self._clear_escape()
            return

        command = Twist()
        command.linear.x = ESCAPE_SPEED
        with self._goal_lock:
            if generation != self._goal_generation or self._escape_from != escape_from:
                return
            self.pub_cmd.publish(command)

    def _cancel_nav(self) -> int:
        with self._goal_lock:
            canceled_generation = self._goal_generation
            self._goal_generation += 1  # Ignore callbacks from superseded goals.
            quiet = threading.Event()
            pending_acceptance = (
                self._goal_request_future is not None
                and self._goal_request_generation == canceled_generation
            )
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    self._nav_quiet_unknown = True
            elif not pending_acceptance:
                quiet.set()
            self._goal_handle = None
            self._nav_failure_reason = None
            self._reset_route_watchdog()
            for old, event in list(self._cancel_events.items()):
                if event.is_set():
                    self._cancel_events.pop(old, None)
            if not quiet.is_set():
                if len(self._cancel_events) >= 8:
                    self._nav_quiet_unknown = True
                else:
                    self._cancel_events[canceled_generation] = quiet
            # Every operator command that supersedes what the robot is doing
            # comes through here, so an escape can never outlive operator input.
            self._clear_escape()
            return self._goal_generation

    def stop(self) -> None:
        exploration = getattr(self, "exploration", None)
        if exploration is not None:
            exploration.stop()
        with self._goal_lock:
            self._cancel_nav()
            self.pub_cmd.publish(Twist())
            self.goal = None
            self.planned_path = []
            self.nav_status, self.mode = "idle", "estop"

    def drive(self, linear: float, angular: float) -> None:
        """Publish a bounded teleop command; the watchdog stops stale input."""
        with self._goal_lock:
            self._cancel_nav()
            command = Twist()
            command.linear.x = max(-0.45, min(0.45, float(linear)))
            command.angular.z = max(-1.2, min(1.2, float(angular)))
            self.pub_cmd.publish(command)
            moving = abs(command.linear.x) > 1e-3 or abs(command.angular.z) > 1e-3
            self._last_drive_at = time.monotonic() if moving else 0.0
            self.goal = None
            self.planned_path = []
            self.nav_status, self.mode = "idle", "teleop" if moving else "idle"

    def drive_watchdog(self) -> None:
        timeout = self._cfg_timeout("drive_timeout_s")
        if self.mode == "teleop" and time.monotonic() - self._last_drive_at > timeout:
            self.pub_cmd.publish(Twist())
            self._last_drive_at = 0.0
            self.mode = "idle"

    def cancel(self) -> int:
        with self._goal_lock:
            self._cancel_nav()
            self.pub_cmd.publish(Twist())
            self.goal = None
            self.planned_path = []
            self.nav_status, self.mode = "cancelled", "idle"
            return self._goal_generation

    def cancel_goal(self) -> int:
        return self.cancel()

    def _goal_status_writable(self) -> bool:
        return not self._nav_quiet_unknown

    def wait_goal_quiet(self, expected_generation: int, not_after: float) -> bool:
        """Wait off the ROS timer until the canceled simulated route settles."""
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return False
            if self._nav_quiet_unknown:
                return False
            events = [
                event
                for generation, event in sorted(self._cancel_events.items())
                if generation < expected_generation and not event.is_set()
            ]
        for event in events:
            while not event.is_set():
                remaining = max(0.0, not_after - time.monotonic())
                if remaining <= 0.0:
                    return False
                event.wait(min(0.02, remaining))
                with self._goal_lock:
                    if (
                        expected_generation != self._goal_generation
                        or self._nav_quiet_unknown
                    ):
                        return False
        with self._goal_lock:
            if expected_generation != self._goal_generation or self._nav_quiet_unknown:
                return False
            for old, event in list(self._cancel_events.items()):
                if event.is_set():
                    self._cancel_events.pop(old, None)
            return True

    # -- reset ---------------------------------------------------------

    def _call(
        self, name: str, srv_type, request, timeout_s: float = SERVICE_TIMEOUT_S
    ) -> bool:
        """Call a ROS service from a worker thread and say whether it answered.

        Polls the future instead of using spin_until_future_complete: rclpy.spin()
        already owns this node on the ROS thread, and adding a second executor
        over the same node is how a service call becomes a permanent hang. The
        spin thread completes the future; this one only watches for it.
        """
        client = self._service_clients.get(name)
        if client is None:
            client = self.node.create_client(srv_type, name)
            self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.node.get_logger().warn(f"[{self.id}] service unavailable: {name}")
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self.node.get_logger().warn(f"[{self.id}] service timed out: {name}")
            return False
        error = future.exception()
        if error is not None:
            self.node.get_logger().warn(f"[{self.id}] service failed: {name}: {error}")
            return False
        return True

    def _lifecycle_service_response(
        self, name: str, srv_type, request, not_after: float
    ):
        """Call one lifecycle service within a shared startup deadline.

        This intentionally does not log per-attempt failures. The bounded
        recovery reports one final readiness diagnostic with its last reason.
        """
        remaining = not_after - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("startup deadline expired")

        client = self._service_clients.get(name)
        if client is None:
            client = self.node.create_client(srv_type, name)
            self._service_clients[name] = client
        wait_s = min(LIFECYCLE_SERVICE_TIMEOUT_S, remaining)
        if not client.wait_for_service(timeout_sec=wait_s):
            raise TimeoutError(f"service unavailable: {name}")

        if time.monotonic() >= not_after:
            raise TimeoutError("startup deadline expired")
        try:
            future = client.call_async(request)
        except Exception as exc:
            raise RuntimeError(f"service submission failed: {name}: {exc}") from exc
        while not future.done():
            remaining = not_after - time.monotonic()
            if remaining <= 0.0:
                future.cancel()
                raise TimeoutError(f"service timed out: {name}")
            time.sleep(min(0.02, remaining))
        if time.monotonic() >= not_after:
            raise TimeoutError(f"service response exceeded deadline: {name}")
        error = future.exception()
        if error is not None:
            raise RuntimeError(f"service failed: {name}: {error}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"service returned no response: {name}")
        return response

    def reset(self) -> dict[str, bool]:
        """Refuse the legacy per-robot reset. Runs on a worker thread.

        ARGoS owns the physical world through its socket bridge and onboard
        mapping requires a fresh frontend mission, so a reset is a
        composition-wide lifecycle operation of the host reset supervisor.
        The negative acknowledgement is queued for the session transmitter,
        which owns the websocket, so a misconfigured legacy server fails
        promptly instead of waiting the full fleet reset timeout for silence.
        """
        self.node.get_logger().warn(
            f"[{self.id}] refusing legacy per-robot reset; "
            "use the epoch-safe simulation reset supervisor"
        )
        steps = {"supervisor_required": False}
        self._reset_report = {
            "type": "reset_done",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "ok": False,
            "steps": steps,
        }
        return steps

    def take_reset_report(self) -> dict | None:
        """Hand the reset verdict to the tx loop, which owns the socket.

        Sending it from the reset's own thread would mean two coroutines writing
        to one websocket concurrently. The tx loop already runs at 5 Hz, so this
        costs at most 200 ms.
        """
        report = self._reset_report
        self._reset_report = None
        return report

    def session_state_tick(self) -> dict | None:
        self.drive_watchdog()
        self.escape_tick()
        self.route_progress_watchdog()
        return self.take_reset_report()

    def route_progress_watchdog(self) -> bool:
        """Cancel a FollowPath goal whose progress along the route stalled."""
        try:
            return route_progress_tick(self)
        except Exception as exc:
            # A supervisor must never take the state loop, and with it the
            # operator link, down with it.
            self.node.get_logger().warn(
                f"[{self.id}] route progress watchdog failed: {exc}"
            )
            return False

    async def session_maps_tick(self, now: float, send, loop) -> None:
        graph = SLAM_GRAPHS.get(self.id)
        last_graph = getattr(self, "_session_last_graph", 0.0)
        if graph is not None and now - last_graph > 3.0:
            self._session_last_graph = now
            await send(slam_graph_payload(self.id, self.t0, graph, None, now))
        last_nav = getattr(self, "_session_last_nav_health", 0.0)
        if now - last_nav > 5.0:
            self._session_last_nav_health = now
            await loop.run_in_executor(None, self.recover_nav_if_down)

    def process_camera(self) -> None:
        """Run perception on the newest local camera image.

        Simulation has no media publisher in the Gazebo container. Keep the
        image local for detection instead of sending a JPEG preview over the
        adapter connection; hardware robots use their dedicated H.264 RTSP
        publisher for operator video.
        """
        if not self._camera_dirty or self._camera_frame is None:
            return
        if not self._upload_lock.acquire(blocking=False):
            return  # a reset is running
        try:
            self._process_camera_locked()
        finally:
            self._upload_lock.release()

    def _process_camera_locked(self) -> None:
        if not self._camera_dirty or self._camera_frame is None:
            return
        self._camera_dirty = False
        msg = self._camera_frame

        try:
            image = self._image_to_bgr(msg)
            if image is None:
                if not self._camera_encoding_warned:
                    self.node.get_logger().warn(
                        f"[{self.id}] cannot decode camera encoding "
                        f"{msg.encoding!r}; detection has no frames"
                    )
                    self._camera_encoding_warned = True
                return

            self._detect_bgr(image, image_header=msg.header)
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] camera processing failed: {exc}")


async def run_robot(bridge: RobotBridge, ws_url: str) -> None:
    """Connect, announce, pump state. Same session shape as hardware."""
    await run_adapter_session(bridge, ws_url, connect=websockets.connect)


async def main_async(bridges: list[RobotBridge], ws_url: str) -> None:
    await asyncio.gather(*(run_robot(b, ws_url) for b in bridges))


def create_adapter_node():
    """The adapter's ROS node, on sim time like every other node in this stack.

    `use_sim_time` is not a formality here. This node stamps exactly two
    outgoing messages, and both are read by nodes living in simulation time: the
    `set_pose` that re-zeroes each EKF during a reset, and the goal handed to
    Nav2.

    On the default wall clock a reset stamped its `set_pose` about 1.78e9
    seconds after every measurement that filter would ever see again.
    `robot_localization` keeps the reset stamp as the filter's last measurement
    time and discards anything older, so each EKF stopped integrating at the
    instant of the first reset and published `odom -> base_link` as identity
    from then on. That transform is half of the pose the GUI draws, so all four
    robots sat frozen on their spawn points while driving around the building —
    and peer_slam, which uses the same TF as its motion prior, lost its
    odometry at the same moment.
    """
    return rclpy.create_node(
        "swarmdeck_adapter_sim",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )


# Every simulated robot's intentions travel through this one process, so a
# peer's claim arrives within milliseconds; the 0.5 s default covers radio links
# between real robots. A grant that a later claim revokes still stops the path.
SIM_RESERVATION_SETTLE_S = 0.15


def spin_events(node) -> None:
    from rclpy.experimental import EventsExecutor

    executor = EventsExecutor()
    executor.add_node(node)
    executor.spin()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--robots",
        type=int,
        default=None,
        help="override YAML/SWARMDECK_ROBOT_COUNT fleet size",
    )
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = create_adapter_node()
    http_url = f"http://{args.host}:{args.port}"
    fleet_cfg: dict = {}
    start_poses: dict = {}
    try:
        with urllib.request.urlopen(f"{http_url}/api/config", timeout=5) as response:
            deployment_cfg = json.loads(response.read()).get("config") or {}
        fleet_cfg = deployment_cfg.get("fleet", {}) or {}
        start_poses = (deployment_cfg.get("map") or {}).get("start_poses") or {}
    except Exception as exc:
        print(f"[adapter_sim] fleet config unavailable ({exc})")
    config_count = fleet_cfg.get("robot_count")
    try:
        config_count_int = int(config_count) if config_count is not None else None
    except (TypeError, ValueError):
        config_count_int = None
    robot_count = resolve_sim_robot_count(
        args.robots,
        os.environ.get("SWARMDECK_ROBOT_COUNT", ""),
        config_count_int,
    )
    node.create_subscription(String, "/swarmdeck/slam_graph", _on_slam_graph, 10)
    # Platforms from the SAME config the fleet was spawned from, so the
    # adapter cannot describe a different fleet than the one Gazebo built.
    if fleet_cfg:
        platforms = robot_types(fleet_cfg, robot_count, args.prefix)
    else:
        print(f"[adapter_sim] assuming every robot is a {DEFAULT_ROBOT_PROFILE}")
        platforms = [DEFAULT_ROBOT_PROFILE] * robot_count

    instantaneous_planar_scan = bool(fleet_cfg) and lidar_spec(fleet_cfg).rings == 1
    bridges = [
        RobotBridge(
            node,
            f"{args.prefix}{i}",
            http_url,
            platforms[i],
            instantaneous_planar_scan=instantaneous_planar_scan,
            exploration_config={
                "enabled": os.environ.get("SWARMDECK_MGG_ENABLED", "0").lower()
                in ("1", "true", "yes"),
                "peer_coordination": os.environ.get(
                    "SWARMDECK_PEER_COORDINATION", "0"
                ).lower()
                in ("1", "true", "yes"),
                "planar_tolerance_m": robot_spec(platforms[i]).max_step_height,
                "max_inclination_rad": math.radians(30.0),
                "reservation_settle_s": SIM_RESERVATION_SETTLE_S,
                # The simulation knows where it spawned every robot, so
                # frontier reservations can be arbitrated in that frame while
                # the robots' maps stay separate components.
                "deployment_start_pose": (
                    start_poses.get(f"{args.prefix}{i}")
                    if os.environ.get("SWARMDECK_COORDINATION_FRAME", "").lower()
                    == "deployment"
                    else None
                ),
            },
            planning_config={
                "backend": os.environ.get("SWARMDECK_PLANNING_BACKEND", ""),
                "planar_tolerance_m": robot_spec(platforms[i]).max_step_height,
                "max_inclination_rad": math.radians(30.0),
            },
        )
        for i in range(robot_count)
    ]
    for bridge in bridges:
        print(
            f"[adapter_sim] {bridge.id}: {bridge.platform} "
            f"({bridge.robot_type}, r={bridge.footprint_radius} m)"
        )

    # ROS spins in its own thread; asyncio owns the protocol side. The events
    # executor dispatches each message as it arrives; the default executor
    # rebuilds a wait set over every subscription, timer and client of all
    # four robots per message, which was the adapter's whole parked CPU.
    threading.Thread(target=lambda: spin_events(node), daemon=True).start()
    try:
        asyncio.run(main_async(bridges, f"ws://{args.host}:{args.port}/adapter"))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
