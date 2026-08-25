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
import importlib
import json
import math
import subprocess
import sys
import threading
import time
import urllib.request
import zlib
from pathlib import Path

import cv2
import numpy as np
import rclpy
import websockets
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from robot_localization.srv import SetPose
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Keep perception reusable by real adapters without packaging it into ROS.
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from adapters.perception.depth_projection import point_for_depth_image
from adapters.perception.object_detector import ObjectDetector, track_ids
from adapters.runtime import AdapterSensorMixin, cloud_xyz, stamp_seconds, yaw_of
from adapters.keyframe_producer import (
    KeyframeUploader,
    laser_scan_to_map_points,
    points_lidar_to_map,
    pose7_from_xy_yaw,
)
from adapters.map_downlink import NavMapClient, apply_to_occupancy_grid

# The platform table, imported from the spawner rather than restated here.
#
# This adapter has to know its own footprint (it reports it at `hello`, and the
# GUI draws robots at that size) and its spawn height (the reset teleports it
# back there). Both are facts about the model Gazebo was handed, and a second
# copy of them would be wrong the first time a chassis changed — the same
# argument session.launch.py already makes by importing lidar_spec from here
# instead of re-reading the YAML.
sys.path.insert(
    0, str(REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim" / "scenario")
)
from spawn_fleet import DEFAULT_ROBOT_PROFILE, robot_spec, robot_types  # noqa: E402


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


# Newest collaborative pose graph per robot, as reported by
# swarmdeck_cslam's graph_reporter. Empty unless Swarm-SLAM is running, in
# which case the GUI's swarm panel appears on its own.
SLAM_GRAPHS: dict[str, dict] = {}


def _on_slam_graph(msg) -> None:
    """cslam pose-graph summary, arriving as JSON on a std_msgs/String.

    Deliberately not a cslam message type: `cslam_common_interfaces` is built in
    the cslam image and absent here, and a subscriber cannot deserialise a type
    it does not have — it would simply never fire. See graph_reporter.py.
    """
    try:
        graph = json.loads(msg.data)
    except (ValueError, TypeError):
        return
    rid = graph.get("robot_id")
    if isinstance(rid, str):
        SLAM_GRAPHS[rid] = graph


# The collaborative back end's own merged grid, if one is running. Fleet-wide
# rather than per robot: cslam produces a single map in its common frame from
# every robot's keyframes, so there is nothing to merge afterwards.
CSLAM_GRID: dict[str, object] = {}


def _on_cslam_grid(msg) -> None:
    CSLAM_GRID["grid"] = msg
    CSLAM_GRID["dirty"] = True


def upload_cslam_grid(http_url: str) -> None:
    """Push the back end's merged grid straight to the backend, unmerged.

    Uploading it as a per-robot map instead would send it back through the
    merge, which would transform a grid that is already in the common frame and
    reintroduce exactly the mismatch cslam_grid.py exists to remove.
    """
    if not CSLAM_GRID.get("dirty"):
        return
    CSLAM_GRID["dirty"] = False
    g = CSLAM_GRID.get("grid")
    if g is None:
        return
    cells = np.array(g.data, dtype=np.int8)
    body = zlib.compress(np.ascontiguousarray(cells).tobytes())
    url = (
        f"{http_url}/api/adapter/global_map?resolution={g.info.resolution}"
        f"&width={g.info.width}&height={g.info.height}"
        f"&origin_x={g.info.origin.position.x}&origin_y={g.info.origin.position.y}"
    )
    try:
        urllib.request.urlopen(
            urllib.request.Request(
                url, data=body, headers={"Content-Type": "application/octet-stream"}
            ),
            timeout=5,
        ).read()
    except Exception as exc:
        print(f"[adapter_sim] cslam grid upload failed: {exc}")


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
# Nav2 cannot plan out of a pose whose footprint overlaps a mapped obstacle, and
# it cannot recover from one either. Measured on the Scout Mini at a corridor
# doorway: its own map placed the door jamb 0.255 m from its centre, inside the
# 0.290 m inscribed radius derived from its 0.580 m width, so
#
#   planner_server   "Failed to create plan with tolerance of: 0.500000"
#   behavior_server  "Collision Ahead - Exiting DriveOnHeading" / "backup failed"
#
# repeated forever. The robot was NOT actually touching anything — ground truth
# put the jamb 0.379 m away, 0.089 m clear — it was ~0.26 m of SLAM drift, which
# the wider Bunkers (0.389 m inscribed) absorb and the Scout does not. Clearing
# the costmap does not help: the obstacle comes from the static layer, which
# slam_toolbox repopulates every `map_update_interval`.
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

ESCAPE_SPEED = -0.15        # m/s, reverse. Slow: this runs without a costmap.
ESCAPE_DISTANCE = 0.45      # m of retreat before the escape is considered done
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
# Nav2's lifecycle manager brings its nodes up in order and ABORTS THE WHOLE
# SEQUENCE if any one of them fails to answer. The confirmation call has a
# 2 s timeout compiled into `nav2_util::ServiceClient` — there is no parameter
# for it — and starting four Nav2 stacks, four SLAM instances, Gazebo and the
# detector at once is enough to miss it. Measured on robot_2:
#
#   Configuring planner_server
#   ERROR  Failed to change state for node: planner_server. Exception:
#          planner_server/get_state service client: async_send_request failed.
#   ERROR  Failed to bring up all requested nodes. Aborting bringup.
#
# 2.07 s after the configure began. The transition itself had SUCCEEDED —
# planner_server sat at `inactive` — only the manager's confirmation timed out,
# and it then abandoned a robot that was fine. Everything after it in the list
# stayed `unconfigured`, including `bt_navigator`, so the NavigateToPose action
# server never existed and every goal failed instantly with "Nav2 action server
# is not ready" until someone re-ran the bringup by hand.
#
# Which robot loses the race is down to scheduling, so this cannot be fixed by
# ordering or by waiting longer at launch. Re-issuing STARTUP is idempotent and
# is exactly what recovered robot_2 live, so the adapter does it itself.

# Long enough that a healthy stack finishes its own bringup untouched — measured
# at 8-12 s from process start on this machine — with margin for a loaded one.
NAV_READY_GRACE_S = 30.0
# A re-bringup that did not take is nearly always a node still starting, so
# leave real time between attempts rather than hammering the service.
NAV_RECOVER_INTERVAL_S = 30.0

# ------------------------------------------------------------------- reset
#
# A reset is fleet-wide in Gazebo and per-robot in ROS, while the protocol
# addresses one robot at a time. The world reset is therefore coalesced here.

# Seconds within which a second robot's reset reuses the first one's world reset.
# Longer than a fleet-wide reset takes to fan out, shorter than any interval an
# operator could press the button twice deliberately.
WORLD_RESET_COALESCE_S = 5.0
# Spawn height is per platform and comes from ROBOT_PROFILES via RobotBridge —
# a Spot's hidden drive wheels sit 0.50 m below its body, so the single constant
# that used to live here would bury it in the floor on reset.
# Let the teleport's velocity transient pass before re-zeroing the filters.
WORLD_SETTLE_S = 0.5
SERVICE_TIMEOUT_S = 8.0

_WORLD_RESET_LOCK = threading.Lock()
_world_reset_at = 0.0
_world_name: str | None = None

# The SLAM back ends this stack can run, newest request first. Discovered rather
# than assumed, because `slam_backend:=rtabmap` swaps the SLAM node out entirely
# and a call to a service that is not there is indistinguishable from a call to
# one that is merely slow.
SLAM_RESET_SERVICES = (
    ("slam_toolbox/reset", "slam_toolbox.srv", "Reset"),
    ("rtabmap/reset", "std_srvs.srv", "Empty"),
)


def _srv_type(module: str, name: str):
    try:
        return getattr(importlib.import_module(module), name)
    except (ImportError, AttributeError):
        return None


def _gz_world_name(logger) -> str | None:
    """Find the running world's name instead of hardcoding it.

    generate_world.py writes the world, so its name is a property of the
    scenario rather than of this adapter, and a stale hardcoded name fails as a
    service timeout that looks exactly like a busy simulator.
    """
    global _world_name
    if _world_name:
        return _world_name
    try:
        listing = subprocess.run(
            ["gz", "service", "-l"], capture_output=True, text=True, timeout=10
        ).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        logger.warn(f"[adapter_sim] cannot list gz services: {exc}")
        return None
    for line in listing.splitlines():
        line = line.strip()
        # Exactly /world/<name>/control. /world/<name>/playback/control also ends
        # in /control and controls log playback, not the world.
        if line.startswith("/world/") and line.endswith("/control"):
            name = line[len("/world/") : -len("/control")]
            if name and "/" not in name:
                _world_name = name
                return name
    logger.warn("[adapter_sim] no /world/<name>/control service found")
    return None


def reset_world(logger) -> bool:
    """Put the *scenery* back where it started. Once per fleet-wide reset.

    This restores the chairs, tables, plants and rubber ducks that make up the
    generated world, which robots push around as they drive.

    It does NOT move the robots, and that is not a bug to be fixed here — it is
    a property of Gazebo. A world reset restores entities declared in the world
    SDF, and the fleet is spawned into a running world by spawn_fleet.py through
    `/world/<name>/create`. Measured against the live stack: the service answers
    `data: true` and robot_0 stays exactly where it was. `RobotBridge._reset_pose`
    puts the robots back, one explicit `set_pose` each.

    `model_only`, and deliberately not `all`: `all` also sets the simulation
    clock back to zero, and every node in this stack runs on `use_sim_time`. A
    /clock that jumps backwards invalidates the tf2 buffers, the Nav2 lifecycle
    timers and SLAM's scan queue at the same instant, which is a far larger
    change than the one being asked for.

    The lock coalesces the per-robot resets into one world reset, and orders
    them: the scene is restored before any robot starts mapping it again.
    """
    global _world_reset_at
    with _WORLD_RESET_LOCK:
        if time.monotonic() - _world_reset_at < WORLD_RESET_COALESCE_S:
            return True
        world = _gz_world_name(logger)
        if world is None:
            return False
        try:
            done = subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/control",
                 "--reqtype", "gz.msgs.WorldControl",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", "reset: {model_only: true}"],
                capture_output=True, text=True, timeout=15,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            logger.warn(f"[adapter_sim] world reset failed: {exc}")
            return False
        # gz exits 0 whether the world answered or the call timed out, so the
        # reply body is the only evidence that anything happened.
        if "true" not in done.stdout.lower():
            logger.warn(
                f"[adapter_sim] world reset rejected: "
                f"{done.stdout.strip()} {done.stderr.strip()}"
            )
            return False
        _world_reset_at = time.monotonic()
        # The collaborative back end's merged grid describes the world that was
        # just reset. Holding it would re-upload the old map to the backend
        # moments after the backend cleared it.
        CSLAM_GRID.clear()
        SLAM_GRAPHS.clear()
        return True


class RobotBridge(AdapterSensorMixin):
    """Per-robot ROS subscriptions and command publisher."""

    def __init__(
        self, node: Node, robot_id: str, http_url: str, platform: str | None = None
    ) -> None:
        self.node = node
        self.id = robot_id
        self.http_url = http_url
        self.t0 = time.monotonic()
        # What this robot IS. `footprint_radius` is not decoration: the GUI draws
        # the robot at that size and the operator judges clearances by it, so a
        # 1.02 m Bunker reporting the old hardcoded 0.3 m would be drawn at less
        # than half its width.
        self.platform = platform or DEFAULT_ROBOT_PROFILE
        spec = robot_spec(self.platform)
        self.robot_type = spec.robot_type
        self.footprint_radius = round(spec.footprint_radius, 3)
        self.footprint = json.loads(spec.footprint)
        self.spawn_z = spec.spawn_z
        # Where this platform's RGBD camera is bolted, from the same table the
        # SDF was rendered from. Turning a duck detection into a map marker is
        # the only thing that reads it. See camera_point_to_map().
        self.camera_x = spec.camera_x
        self.camera_z = spec.camera_z
        self.lidar_x = spec.lidar_x
        self.lidar_z = spec.lidar_z
        self._keyframes = KeyframeUploader(robot_id, http_url)
        self._nav_map = NavMapClient(http_url, robot_id)
        self._scan_cloud_at = 0.0

        # Two links of the same TF chain: map_frame -> odom -> base_link. Both
        # come off the robot's namespaced /tf, which is the only place they are
        # guaranteed to be consistent with each other. See map_pose().
        self._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._odom_to_base: dict[str, float] | None = None
        self._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._warned_no_tf_base = False
        self.goal: dict | None = None
        self.planned_path: list[dict[str, float]] = []
        self.nav_status = "idle"
        self.mode = "idle"
        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False
        self._cloud: PointCloud2 | None = None
        self._cloud_dirty = False
        self._camera_frame: Image | None = None
        self._camera_dirty = False
        self._camera_encoding_warned = False
        # The depth half of the same sensor, plus its intrinsics. Kept as the
        # newest message rather than queued: detection runs off the colour frame
        # and only ever wants the depth taken with it.
        self._camera_depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._last_depth_warning_at = 0.0
        self._detector = ObjectDetector()
        self._detection_enabled = True
        self._detections: list[dict] | None = None
        self._goal_handle = None
        self._goal_generation = 0
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
        self._start_pose: dict | None = None

        node.create_subscription(Odometry, f"/{robot_id}/odom", self._on_odom, 10)
        node.create_subscription(TFMessage, f"/{robot_id}/tf", self._on_tf, 20)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, f"/{robot_id}/map", self._on_map, latched)
        self.pub_global_map = node.create_publisher(
            OccupancyGrid, f"/{robot_id}/global_map", latched
        )
        node.create_subscription(
            LaserScan, f"/{robot_id}/scan", self._on_scan, qos_profile_sensor_data
        )
        node.create_subscription(
            PointCloud2,
            f"/{robot_id}/scan/points",
            self._on_scan_cloud,
            qos_profile_sensor_data,
        )
        # RTAB-Map's accumulated 3D map, for the GUI's optional 3D view. This is
        # the assembled map in the robot's own map frame, NOT the raw sensor
        # cloud on scan/points: the view wants what the robot has built, and the
        # sensor stream would be both wrong (lidar frame) and far too fast.
        # SLAM Toolbox publishes nothing here, so the 2D fleet simply has no 3D
        # view rather than a misleading one.
        node.create_subscription(
            PointCloud2, f"/{robot_id}/cloud_map", self._on_cloud, latched
        )
        node.create_subscription(NavPath, f"/{robot_id}/plan", self._on_plan, 10)
        # Three streams off one `rgbd_camera`, bridged by session.launch.py. The
        # colour frame is what the operator sees and what the detector runs on;
        # the depth image and the intrinsics are what turn a detection box into a
        # point on the map. A robot whose depth is missing still detects and
        # still streams video — it just reports no `map_position`.
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

        self.nav_client = ActionClient(
            node, NavigateToPose, f"/{robot_id}/navigate_to_pose"
        )
        self.pub_cmd = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)

    def _on_odom(self, msg: Odometry) -> None:
        """Wheel odometry — a FALLBACK only. See map_pose() for why."""
        p = msg.pose.pose
        self._odom_topic_pose = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": yaw_of(p.orientation),
        }

    def _on_tf(self, msg: TFMessage) -> None:
        """Track both links of map_frame -> odom -> base_link from robot /tf."""
        map_frame = f"{self.id}/map_frame"
        odom_frame = f"{self.id}/odom"
        base_frame = f"{self.id}/base_link"
        for stamped in msg.transforms:
            t = stamped.transform
            value = {
                "x": t.translation.x,
                "y": t.translation.y,
                "yaw": yaw_of(t.rotation),
            }
            if stamped.header.frame_id == map_frame and stamped.child_frame_id == odom_frame:
                self._map_to_odom = value
            elif stamped.header.frame_id == odom_frame and stamped.child_frame_id == base_frame:
                self._odom_to_base = value

    @staticmethod
    def _compose(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
        """SE(2) composition: the pose of `b` expressed in `a`'s parent frame."""
        c, s = math.cos(a["yaw"]), math.sin(a["yaw"])
        return {
            "x": a["x"] + b["x"] * c - b["y"] * s,
            "y": a["y"] + b["x"] * s + b["y"] * c,
            "yaw": (a["yaw"] + b["yaw"] + math.pi) % (2 * math.pi) - math.pi,
        }

    def map_pose(self) -> dict[str, float]:
        """Where this robot is in its own SLAM map frame.

        Both links come from TF, and that is the whole point. `odom -> base_link`
        is owned by the EKF (or, with `fuse_imu:=false`, by the drive plugin's
        bridged TF), while `/<ns>/odom` carries the drive plugin's *raw wheel*
        integration regardless. Composing SLAM's correction with the wheel topic
        mixes two different chains: SLAM computed `map_frame -> odom` against the
        EKF's `base_link`, so the result is off by exactly however far wheel
        odometry has diverged from the filter.

        That was not theoretical. Measured live on a four-robot run, the wheel
        topic differed from the EKF by 0.18-0.48 m per robot, and the pose the
        GUI drew was wrong against Gazebo ground truth by 0.16-0.47 m — the same
        numbers, robot for robot. Composing from TF instead brings it to ~0.07 m,
        which is SLAM's own residual. Worse, wheel odometry is the channel that
        breaks catastrophically when a differential drive jams and spins its
        wheels (8.8-30.5 m of error measured in docs/operations/known-issues.md), which is
        why robot markers would occasionally jump right off the building.

        The wheel topic remains a fallback for a robot whose TF carries no
        `odom -> base_link` at all, because reporting the map origin forever is a
        worse failure than reporting a drifting pose — but it says so out loud.
        """
        base = self._odom_to_base
        if base is None:
            if not self._warned_no_tf_base:
                self._warned_no_tf_base = True
                self.node.get_logger().warn(
                    f"[{self.id}] no {self.id}/odom -> {self.id}/base_link on TF; "
                    f"falling back to raw wheel odometry for the reported pose, "
                    f"which will disagree with the map whenever the wheels slip."
                )
            base = self._odom_topic_pose
        return self._compose(self._map_to_odom, base)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        self._grid_dirty = True

    def _pose7(self) -> np.ndarray:
        pose = self.map_pose()
        return pose7_from_xy_yaw(pose["x"], pose["y"], pose["yaw"])

    def _enqueue_keyframe(self, points_map: np.ndarray, stamp: float) -> None:
        uploader = getattr(self, "_keyframes", None)
        if uploader is None or points_map.shape[0] == 0:
            return
        try:
            uploader.consider(points_map, self._pose7(), stamp)
        except Exception:
            # Keyframe production must never starve the scan map or Nav2.
            pass

    def _on_scan(self, msg: LaserScan) -> None:
        """2D fallback. Ignored while the 3D ``scan/points`` path is alive."""
        if time.monotonic() - getattr(self, "_scan_cloud_at", 0.0) < 1.0:
            return
        pose = self.map_pose()
        points = laser_scan_to_map_points(
            np.asarray(msg.ranges, dtype=np.float64),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            pose_xy_yaw=(pose["x"], pose["y"], pose["yaw"]),
            lidar_x=float(getattr(self, "lidar_x", 0.0)),
            lidar_z=float(getattr(self, "lidar_z", 0.0)),
        )
        self._enqueue_keyframe(points, stamp_seconds(msg.header) or time.time())

    def _on_scan_cloud(self, msg: PointCloud2) -> None:
        self._scan_cloud_at = time.monotonic()
        points = cloud_xyz(msg)
        if not len(points):
            return
        pose = self.map_pose()
        mapped = points_lidar_to_map(
            points,
            (pose["x"], pose["y"], pose["yaw"]),
            lidar_x=float(getattr(self, "lidar_x", 0.0)),
            lidar_z=float(getattr(self, "lidar_z", 0.0)),
        )
        self._enqueue_keyframe(mapped, stamp_seconds(msg.header) or time.time())

    def upload_keyframe(self) -> None:
        uploader = getattr(self, "_keyframes", None)
        if uploader is None:
            return
        if not self._upload_lock.acquire(blocking=False):
            return
        try:
            uploader.upload_one()
        finally:
            self._upload_lock.release()

    def pull_nav_map(self) -> None:
        """Publish the collaborative map in this robot's frame for Nav2."""
        client = getattr(self, "_nav_map", None)
        pub = getattr(self, "pub_global_map", None)
        if client is None or pub is None:
            return
        if not self._upload_lock.acquire(blocking=False):
            return
        try:
            downloaded = client.poll()
        finally:
            self._upload_lock.release()
        if downloaded is None:
            if client.last_error:
                self.node.get_logger().warn(
                    f"[{self.id}] nav map download failed: {client.last_error}"
                )
            return
        grid = OccupancyGrid()
        apply_to_occupancy_grid(grid, downloaded, f"{self.id}/map_frame")
        grid.header.stamp = self.node.get_clock().now().to_msg()
        pub.publish(grid)

    def cslam_origin(self, graph: dict) -> dict | None:
        """This robot's SLAM map frame expressed in cslam's common frame.

        cslam reports where the robot IS in the common frame; the adapter knows
        where the same robot is in its own map frame. The transform between the
        two frames is therefore

            T_map->common  =  pose_common  o  pose_own^-1

        which is exactly what the map service needs to place this robot's grid.
        Returns None until cslam has actually merged this robot with someone:
        before that it reports its own frame as the common one, and publishing
        an identity transform would claim a merge that has not happened.
        """
        common = graph.get("common")
        if not isinstance(common, dict) or not graph.get("in_common_frame"):
            return None
        own = self.map_pose()
        cyaw = float(common.get("yaw", 0.0))
        dyaw = (cyaw - own["yaw"] + math.pi) % (2 * math.pi) - math.pi
        c, s = math.cos(dyaw), math.sin(dyaw)
        return {
            "x": float(common.get("x", 0.0)) - (own["x"] * c - own["y"] * s),
            "y": float(common.get("y", 0.0)) - (own["x"] * s + own["y"] * c),
            "yaw": dyaw,
            "frame": common.get("frame"),
        }

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._cloud = msg
        self._cloud_dirty = True

    def upload_cloud(self) -> None:
        """Voxel-downsample the 3D map and push it, quantised to 1 cm.

        Downsampling happens here rather than in the backend because this is the
        expensive end of a link that should stay cheap: a full RTAB-Map cloud is
        millions of points, and the view draws each one as a single pixel.
        """
        if not self._cloud_dirty or self._cloud is None:
            return
        if not self._upload_lock.acquire(blocking=False):
            return  # a reset is running; see upload_map
        try:
            self._upload_cloud_locked()
        finally:
            self._upload_lock.release()

    def _upload_cloud_locked(self) -> None:
        if not self._cloud_dirty or self._cloud is None:
            return
        self._cloud_dirty = False
        points = self._cloud_xyz(self._cloud)
        if not len(points):
            return
        # Deduplicate onto a voxel lattice: one point per occupied cell.
        keys = np.round(points / CLOUD_VOXEL).astype(np.int32)
        _, keep = np.unique(keys, axis=0, return_index=True)
        quantised = np.round(points[keep] / CLOUD_SCALE).astype(np.int16)
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    f"{self.http_url}/api/adapter/cloud?robot_id={self.id}"
                    f"&scale={CLOUD_SCALE}",
                    data=zlib.compress(quantised.tobytes(), 1),
                    headers={"Content-Type": "application/octet-stream"},
                ),
                timeout=5,
            ).read()
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] cloud upload failed: {exc}")

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

    def hello(self) -> dict:
        return {
            "type": "hello",
            # 2: adds the optional slam_graph message, emitted only when
            # Swarm-SLAM is running and graph_reporter is publishing.
            "protocol": 2,
            "robot_id": self.id,
            "robot_type": self.robot_type,
            "adapter": "adapter_sim/0.1.0",
            "ros": "jazzy",
            # `reset` is simulation-only and must stay that way — it means
            # teleport to spawn and forget the map. See the protocol README.
            "capabilities": ["navigate", "map", "camera", "battery", "estop", "reset"],
            "footprint_radius": self.footprint_radius,
            "footprint": self.footprint,
        }

    def state(self) -> dict:
        return {
            "type": "robot_state",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "pose": self.map_pose(),
            "battery": None,  # simulation has no battery model
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
            "planned_path": self.planned_path,
        }

    def navigate_to(self, goal: dict) -> None:
        """Map a planner-agnostic command onto Nav2's NavigateToPose action."""
        self._cancel_nav()
        generation = self._goal_generation

        if not self.nav_client.server_is_ready():
            self.node.get_logger().error(f"[{self.id}] Nav2 action server is not ready")
            self.goal = None
            self.nav_status, self.mode = "failed", "idle"
            return

        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = f"{self.id}/map_frame"
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.position.x = float(goal["x"])
        request.pose.pose.position.y = float(goal["y"])
        yaw = float(goal.get("yaw", 0.0))
        request.pose.pose.orientation.z = math.sin(yaw / 2)
        request.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self.nav_client.send_goal_async(request)
        future.add_done_callback(lambda done: self._goal_response(done, generation))
        self.goal = {"x": float(goal["x"]), "y": float(goal["y"]), "yaw": yaw}
        self.nav_status, self.mode = "active", "nav"

    def _goal_response(self, future, generation: int) -> None:
        if generation != self._goal_generation:
            # A cancel can arrive before Nav2 accepts the request. Cancel the
            # resulting handle instead of merely ignoring its callbacks.
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"[{self.id}] goal request failed: {exc}")
            self._finish_goal("failed", generation)
            return
        if not handle.accepted:
            self.node.get_logger().warn(f"[{self.id}] navigation goal rejected")
            self._finish_goal("failed", generation)
            return

        self._goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda done: self._goal_result(done, generation))

    def _goal_result(self, future, generation: int) -> None:
        if generation != self._goal_generation:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.node.get_logger().error(f"[{self.id}] navigation result failed: {exc}")
            self._finish_goal("failed", generation)
            return

        terminal = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "cancelled",
            GoalStatus.STATUS_ABORTED: "failed",
        }.get(status, "failed")
        self._finish_goal(terminal, generation)

    def _finish_goal(self, status: str, generation: int) -> None:
        if generation != self._goal_generation:
            return
        self._goal_handle = None
        self.goal = None
        self.planned_path = []
        self.nav_status, self.mode = status, "idle"
        if status == "failed":
            self._arm_escape()

    # -- Nav2 bringup recovery ------------------------------------------

    def recover_nav_if_down(self) -> bool:
        """Re-run Nav2's bringup if its action server never appeared.

        Runs on a worker thread — `_call` polls a future that only the spin
        thread can complete. Returns whether a bringup was issued, not whether
        Nav2 came up: the manager reports success as soon as it has walked the
        list, and the next cycle's readiness check is the real verdict.

        Deliberately driven by the action server's absence rather than by
        querying lifecycle states. That is the thing the adapter actually needs
        in order to navigate, it costs nothing to check, and it stays true if a
        future Nav2 reorganises which nodes exist.
        """
        now = time.monotonic()
        if self.nav_client.server_is_ready():
            self._nav_down_since = 0.0
            return False
        if self._nav_down_since == 0.0:
            # First cycle that noticed. Start the clock rather than acting: at
            # this point a perfectly healthy stack is simply still starting.
            self._nav_down_since = now
            return False
        if now - self._nav_down_since < NAV_READY_GRACE_S:
            return False
        if self._nav_recovered_at and now - self._nav_recovered_at < NAV_RECOVER_INTERVAL_S:
            return False

        self._nav_recovered_at = now
        self.node.get_logger().warn(
            f"[{self.id}] Nav2 action server absent for "
            f"{now - self._nav_down_since:.0f} s; re-running lifecycle bringup"
        )
        # STARTUP, and only STARTUP. It walks the node list issuing CONFIGURE
        # then ACTIVATE, which is exactly right for the state the race leaves —
        # a prefix the manager got to before it aborted, and an unconfigured
        # suffix it never reached. Verified live on robot_2: five nodes went to
        # `active` and it drove its next goal.
        #
        # RESET first was tried and removed. Nav2's manager is all-or-nothing in
        # BOTH directions — RESET deactivates in reverse and aborts on the first
        # node already below `inactive`, just as STARTUP aborts on the first
        # already above it — so a stack in mixed state (one node down among
        # healthy ones) cannot be repaired through this service at all, and
        # adding the call bought nothing for the state that can. A robot in that
        # condition needs its launch restarted, and the repeated warning below is
        # how an operator finds out.
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        answered = self._call(
            f"/{self.id}/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes,
            request,
        )
        if not answered:
            self.node.get_logger().error(
                f"[{self.id}] lifecycle bringup did not answer; "
                f"navigation stays unavailable"
            )
        return answered

    # -- wedge escape --------------------------------------------------

    def _arm_escape(self) -> None:
        """Begin reversing out of a pose Nav2 could not plan from.

        Armed on any aborted goal rather than on a costmap test, because the
        adapter deliberately holds no costmap: Nav2 owns that, and duplicating
        its inflation maths here to second-guess it would be a second source of
        truth to keep in step. Reversing 0.45 m is harmless for the failures
        this does not apply to, and it is the only thing that helps for the one
        it does.
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
        if self._escape_from is None:
            return
        pose = self.map_pose()
        moved = math.hypot(pose["x"] - self._escape_from[0], pose["y"] - self._escape_from[1])
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
        self.pub_cmd.publish(command)

    def _cancel_nav(self) -> None:
        self._goal_generation += 1  # Ignore callbacks from the superseded goal.
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        # Every operator command that supersedes what the robot is doing comes
        # through here — navigate_to, drive, stop, cancel and reset — so this is
        # the one place that guarantees an escape can never drive a robot the
        # operator has just taken control of.
        self._clear_escape()

    def stop(self) -> None:
        self._cancel_nav()
        self.pub_cmd.publish(Twist())
        self.goal = None
        self.planned_path = []
        self.nav_status, self.mode = "idle", "estop"

    def drive(self, linear: float, angular: float) -> None:
        """Publish a bounded teleop command; the watchdog stops stale input."""
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
        if self.mode == "teleop" and time.monotonic() - self._last_drive_at > 0.45:
            self.pub_cmd.publish(Twist())
            self._last_drive_at = 0.0
            self.mode = "idle"

    def cancel(self) -> None:
        self._cancel_nav()
        self.pub_cmd.publish(Twist())
        self.goal = None
        self.planned_path = []
        self.nav_status, self.mode = "cancelled", "idle"

    # -- reset ---------------------------------------------------------

    def _call(self, name: str, srv_type, request, timeout_s: float = SERVICE_TIMEOUT_S) -> bool:
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

    def _configured_start_pose(self) -> dict | None:
        """Where this robot was spawned, according to the backend's config.

        Read from `/api/config` rather than from the study YAML on disk, so the
        pose a reset returns the robot to and the prior the backend registers it
        against cannot drift apart — they are then the same number from the same
        file, which is the argument docker-compose.yml already makes for handing
        the backend the same config the fleet runs.

        Cached after the first success: the config does not change under a
        running stack, and a reset should not depend on an HTTP round trip
        succeeding at exactly the wrong moment.
        """
        if self._start_pose is not None:
            return self._start_pose
        try:
            with urllib.request.urlopen(
                f"{self.http_url}/api/config", timeout=5
            ) as response:
                config = json.loads(response.read()).get("config", {})
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] cannot read start pose: {exc}")
            return None
        pose = ((config.get("map") or {}).get("start_poses") or {}).get(self.id)
        if not isinstance(pose, dict):
            self.node.get_logger().warn(
                f"[{self.id}] no start pose configured; leaving it where it stands "
                f"rather than guessing one"
            )
            return None
        self._start_pose = pose
        return pose

    def _reset_pose(self) -> bool:
        """Teleport this robot back to its spawn pose.

        Explicit, because a Gazebo world reset does not do it: the fleet is
        created in a running world, and a reset only restores what the world SDF
        declared. See reset_world().
        """
        pose = self._configured_start_pose()
        if pose is None:
            return False
        world = _gz_world_name(self.node.get_logger())
        if world is None:
            return False
        yaw = float(pose.get("yaw", 0.0))
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        request = (
            f'name: "{self.id}", '
            f'position: {{x: {float(pose["x"])}, y: {float(pose["y"])}, z: {self.spawn_z}}}, '
            f"orientation: {{z: {qz:.9f}, w: {qw:.9f}}}"
        )
        try:
            done = subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/set_pose",
                 "--reqtype", "gz.msgs.Pose",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", request],
                capture_output=True, text=True, timeout=15,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.node.get_logger().warn(f"[{self.id}] set_pose failed: {exc}")
            return False
        if "true" not in done.stdout.lower():
            self.node.get_logger().warn(
                f"[{self.id}] set_pose rejected: "
                f"{done.stdout.strip()} {done.stderr.strip()}"
            )
            return False
        return True

    def _reset_odometry(self) -> bool:
        """Re-zero the EKF's integrated pose.

        Blunt, and safe to be blunt about, because of what ekf.yaml actually
        fuses: linear velocity from the wheels and yaw RATE from the gyro, never
        an absolute pose. Nothing will answer the new origin by re-asserting the
        old one, and Gazebo returning the model to spawn injects no pose
        measurement for the filter to argue with. Called after the world reset
        has settled so the teleport's velocity transient is not what gets
        integrated into the fresh estimate.
        """
        request = SetPose.Request()
        request.pose.header.frame_id = f"{self.id}/odom"
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.pose.orientation.w = 1.0
        return self._call(f"/{self.id}/set_pose", SetPose, request)

    def _reset_slam(self) -> bool:
        """Drop the pose graph and the map, whichever back end is running."""
        available = dict(self.node.get_service_names_and_types())
        for suffix, module, name in SLAM_RESET_SERVICES:
            service = f"/{self.id}/{suffix}"
            if service not in available:
                continue
            srv_type = _srv_type(module, name)
            if srv_type is None:
                self.node.get_logger().warn(
                    f"[{self.id}] {service} exists but {module}.{name} is not installed"
                )
                continue
            return self._call(service, srv_type, srv_type.Request())
        self.node.get_logger().warn(
            f"[{self.id}] no SLAM reset service; tried "
            f"{[s for s, _, _ in SLAM_RESET_SERVICES]}"
        )
        return False

    def _clear_costmaps(self) -> bool:
        """Forget obstacles recorded against a map that no longer exists."""
        ok = True
        for layer in (
            "global_costmap/clear_entirely_global_costmap",
            "local_costmap/clear_entirely_local_costmap",
        ):
            ok = self._call(
                f"/{self.id}/{layer}", ClearEntireCostmap, ClearEntireCostmap.Request()
            ) and ok
        return ok

    def reset(self) -> dict[str, bool]:
        """Return this robot to its start state. Runs on a worker thread.

        Ordering is the whole design. Stop the robot before moving it; move it
        before re-zeroing the filter that measures its movement; re-zero the
        filter before restarting the SLAM that uses it as a motion prior; and
        only drop the cached uploads once all of it has happened, so that what
        the backend clears afterwards stays cleared.

        Note what a reset does NOT do: restart the exploration bootstrap. If
        `explore_seconds` has elapsed the fleet comes back stationary, waiting
        for goals, which is the same state it would have been left in.

        Returns a step-by-step verdict rather than a single boolean, because a
        reset where the world moved but SLAM did not is a specific and
        recognisable failure — the operator sees robots back at the start
        drawing their old map — and naming it beats reporting `false`.
        """
        steps: dict[str, bool] = {}
        with self._upload_lock:
            self._cancel_nav()
            self.pub_cmd.publish(Twist())
            self.goal = None
            self.planned_path = []
            self.nav_status, self.mode = "idle", "idle"
            self._last_drive_at = 0.0

            # Scenery first, then this robot. Two calls because a world reset
            # restores only what the world SDF declared, and the fleet was
            # spawned into a world that was already running.
            steps["world"] = reset_world(self.node.get_logger())
            steps["pose"] = self._reset_pose()
            time.sleep(WORLD_SETTLE_S)
            steps["odometry"] = self._reset_odometry()
            steps["slam"] = self._reset_slam()
            steps["costmaps"] = self._clear_costmaps()

            # Last, and inside the lock: anything still cached here describes the
            # world before the reset, and uploading it after the backend clears
            # would put the old map straight back.
            self.grid = None
            self._grid_dirty = False
            self._cloud = None
            self._cloud_dirty = False
            self._camera_frame = None
            self._camera_dirty = False
            # Not `_camera_info`: intrinsics are a fact about the lens, not about
            # the world that was just reset, and dropping them would only delay
            # the first detection that can be placed on the new map.
            self._camera_depth = None
            self._detections = None

        self._reset_report = {
            "type": "reset_done",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "ok": all(steps.values()),
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

    def upload_map(self) -> None:
        if not self._grid_dirty or self.grid is None:
            return
        # Skip rather than wait. reset() holds this lock for several seconds, and
        # blocking here would stall the tx loop behind it — the 5 Hz state stream
        # would stop, and the backend reads four seconds of silence as the robot
        # going offline. A skipped upload costs one 2 s cycle.
        if not self._upload_lock.acquire(blocking=False):
            return
        try:
            self._upload_map_locked()
        finally:
            self._upload_lock.release()

    def _upload_map_locked(self) -> None:
        if not self._grid_dirty or self.grid is None:
            return
        self._grid_dirty = False
        g = self.grid
        cells = np.array(g.data, dtype=np.int8).reshape(g.info.height, g.info.width)
        body = zlib.compress(np.ascontiguousarray(cells).tobytes())
        url = (
            f"{self.http_url}/api/adapter/map?robot_id={self.id}"
            f"&resolution={g.info.resolution}&width={g.info.width}&height={g.info.height}"
            f"&origin_x={g.info.origin.position.x}&origin_y={g.info.origin.position.y}"
        )
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    url, data=body, headers={"Content-Type": "application/octet-stream"}
                ),
                timeout=5,
            ).read()
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] map upload failed: {exc}")

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
            return  # a reset is running; see upload_map
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
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            encoding = msg.encoding.lower()
            if encoding in ("rgb8", "8uc3"):
                rgb = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif encoding == "bgr8":
                image = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
            elif encoding == "rgba8":
                rgba = rows[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
                image = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            elif encoding == "bgra8":
                bgra = rows[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
                image = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            elif encoding == "mono8":
                image = rows[:, : msg.width].reshape(msg.height, msg.width)
            else:
                if not self._camera_encoding_warned:
                    self.node.get_logger().warn(
                        f"[{self.id}] unsupported camera encoding: {msg.encoding}"
                    )
                    self._camera_encoding_warned = True
                return

            detections = []
            if self._detection_enabled:
                for detection, track_id in track_ids(self._detector.detect_bgr(image)):
                    item = detection.as_protocol(track_id)
                    # The detector is RGB-only; the depth half of the same frame
                    # is what makes the box a place rather than a rectangle.
                    item["map_position"] = self._depth_map_position(
                        detection.bbox, msg.header, detection.polygon
                    )
                    detections.append(item)
            self._detections = detections
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] camera processing failed: {exc}")

    def take_detections(self) -> list[dict] | None:
        current = self._detections
        self._detections = None
        return current

    def refresh_settings(self) -> None:
        """Apply persisted perception settings without coupling to ROS/Gazebo."""
        try:
            with urllib.request.urlopen(f"{self.http_url}/api/settings", timeout=2) as response:
                payload = json.loads(response.read())
            value = payload.get("settings", {})
            self._detection_enabled = bool(value.get("detection_enabled", True))
            self._detector.sensitivity = max(
                0.1, min(1.0, float(value.get("detection_sensitivity", 0.55)))
            )
            self._detector.classes = value.get("detection_classes")
            # The CAPTURE floor, not the operator's display floor; the backend
            # enforces the latter against stored detections. See
            # capture_floors() in the server's config/detection.py. The
            # fallback keeps a new adapter working against an old backend.
            self._detector.class_floors = value.get(
                "detection_capture_floors"
            ) or value.get("detection_class_floors")
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] settings refresh failed: {exc}")


async def run_robot(bridge: RobotBridge, ws_url: str) -> None:
    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                await ws.send(json.dumps(bridge.hello()))
                print(f"[adapter_sim] {bridge.id} connected")

                async def rx() -> None:
                    loop = asyncio.get_running_loop()
                    async for raw in ws:
                        msg = json.loads(raw)
                        t = msg.get("type")
                        if t == "navigate_to":
                            bridge.navigate_to(msg["goal"])
                        elif t == "cancel_goal":
                            bridge.cancel()
                        elif t == "stop":
                            bridge.stop()
                        elif t == "drive":
                            bridge.drive(msg.get("linear", 0.0), msg.get("angular", 0.0))
                        elif t == "reset":
                            # Several seconds of blocking service calls. Run it
                            # off the loop and do not await it here, or this
                            # socket stops reading — including the `stop` an
                            # operator might send while it runs.
                            loop.run_in_executor(None, bridge.reset)

                async def tx() -> None:
                    last_map = 0.0
                    last_cloud = 0.0
                    last_graph = 0.0
                    last_cslam_grid = 0.0
                    last_frame = 0.0
                    last_settings = 0.0
                    last_nav_health = 0.0
                    loop = asyncio.get_running_loop()
                    while True:
                        await ws.send(json.dumps(bridge.state()))
                        report = bridge.take_reset_report()
                        if report is not None:
                            await ws.send(json.dumps(report))
                        now = time.monotonic()
                        bridge.drive_watchdog()
                        bridge.escape_tick()
                        if now - last_map > 2.0:
                            last_map = now
                            await loop.run_in_executor(None, bridge.upload_map)
                            await loop.run_in_executor(None, bridge.upload_keyframe)
                            await loop.run_in_executor(None, bridge.pull_nav_map)
                        # Slower than the grid: a 3D map changes gradually and
                        # is an order of magnitude more bytes.
                        graph = SLAM_GRAPHS.get(bridge.id)
                        if graph is not None and now - last_graph > 3.0:
                            last_graph = now
                            payload = {
                                "type": "slam_graph",
                                "robot_id": bridge.id,
                                "t_mono": round(now - bridge.t0, 4),
                                "keyframes": graph.get("keyframes", 0),
                                "in_common_frame": graph.get(
                                    "in_common_frame", False
                                ),
                                "residual": graph.get("residual"),
                                "inter_robot": graph.get("inter_robot", []),
                            }
                            origin = bridge.cslam_origin(graph)
                            if origin is not None:
                                payload["origin"] = origin
                            # The robot's pose AS THE BACK END SEES IT, in the
                            # common frame. Sent verbatim so the backend can use
                            # it directly instead of transforming a pose out of
                            # RTAB-Map's frame: composing across two independent
                            # SLAM estimates is what produced 8-18 m of error,
                            # and the composition is unnecessary once the grid is
                            # cslam's too.
                            common = graph.get("common")
                            if isinstance(common, dict) and graph.get("in_common_frame"):
                                payload["common_pose"] = {
                                    "x": float(common.get("x", 0.0)),
                                    "y": float(common.get("y", 0.0)),
                                    "yaw": float(common.get("yaw", 0.0)),
                                }
                            await ws.send(json.dumps(payload))
                        if now - last_cslam_grid > 4.0:
                            last_cslam_grid = now
                            await loop.run_in_executor(
                                None, upload_cslam_grid, bridge.http_url
                            )
                        if now - last_cloud > 4.0:
                            last_cloud = now
                            await loop.run_in_executor(None, bridge.upload_cloud)
                        if now - last_frame > 0.2:
                            last_frame = now
                            # Detection runs on every frame regardless of which
                            # camera is on screen. No image is sent over the
                            # adapter connection.
                            await loop.run_in_executor(None, bridge.process_camera)
                            detections = bridge.take_detections()
                            if detections is not None:
                                await ws.send(json.dumps({
                                    "type": "detections",
                                    "robot_id": bridge.id,
                                    "camera": "front",
                                    "items": detections,
                                }))
                        if now - last_settings > 5.0:
                            last_settings = now
                            await loop.run_in_executor(None, bridge.refresh_settings)
                        # Proactive, not on demand: an operator should not have
                        # to issue a doomed goal to discover that this robot's
                        # Nav2 lost the startup race, and the recovery takes
                        # seconds during which nothing else needs to happen.
                        if now - last_nav_health > 5.0:
                            last_nav_health = now
                            await loop.run_in_executor(None, bridge.recover_nav_if_down)
                        await asyncio.sleep(0.2)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            print(f"[adapter_sim] {bridge.id} disconnected ({exc}); retry in 2s")
            await asyncio.sleep(2)


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
    and slam_toolbox, which uses the same TF as its motion prior, lost its
    odometry at the same moment.
    """
    return rclpy.create_node(
        "swarmdeck_adapter_sim",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=None,
                    help="override the persisted fleet count")
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = create_adapter_node()
    http_url = f"http://{args.host}:{args.port}"
    robot_count = args.robots
    if robot_count is None:
        try:
            with urllib.request.urlopen(f"{http_url}/api/settings", timeout=2) as response:
                runtime = json.loads(response.read()).get("settings", {})
            robot_count = int(runtime.get("robot_count", 4))
        except Exception as exc:
            print(f"[adapter_sim] settings unavailable ({exc}); using 4 robots")
            robot_count = 4
    robot_count = max(1, min(robot_count, 5))
    node.create_subscription(String, "/swarmdeck/slam_graph", _on_slam_graph, 10)
    node.create_subscription(
        OccupancyGrid, "/cslam/map", _on_cslam_grid,
        QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
    )
    # Which platform each robot is, from the SAME config the fleet was spawned
    # from. Read through the backend rather than off disk so the adapter cannot
    # end up describing a different fleet than the one Gazebo built.
    try:
        with urllib.request.urlopen(f"{http_url}/api/config", timeout=5) as response:
            fleet_cfg = (json.loads(response.read()).get("config") or {}).get("fleet", {})
        platforms = robot_types(fleet_cfg, robot_count, args.prefix)
    except Exception as exc:
        print(
            f"[adapter_sim] fleet config unavailable ({exc}); "
            f"assuming every robot is a {DEFAULT_ROBOT_PROFILE}"
        )
        platforms = [DEFAULT_ROBOT_PROFILE] * robot_count

    bridges = [
        RobotBridge(node, f"{args.prefix}{i}", http_url, platforms[i])
        for i in range(robot_count)
    ]
    for bridge in bridges:
        print(
            f"[adapter_sim] {bridge.id}: {bridge.platform} "
            f"({bridge.robot_type}, r={bridge.footprint_radius} m)"
        )

    # ROS spins in its own thread; asyncio owns the protocol side.
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    try:
        asyncio.run(main_async(bridges, f"ws://{args.host}:{args.port}/adapter"))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
