#!/usr/bin/env python3
"""Hardware adapter: a real ROS 1 robot -> SwarmDeck adapter protocol.

    python3 adapter_ros1.py --robot-id tars_0 --config robot.yaml

One process per robot, running ON the robot (or on a machine that shares its ROS
graph). This is `adapter_ros2` ported to `rospy`/`actionlib`, not a different
design: same protocol, same config schema, same capability/deadman rules. It
exists because some robots in this fleet run a real ROS 1 stack today — see
docs/robots/fleet.md for the platform matrix.

WHY THIS IS A SEPARATE FILE, NOT AN IF/ELSE IN adapter_ros2.py
----------------------------------------------------------------
`rclpy` and `rospy` are different libraries with different node models: ROS 2
has QoS profiles and per-node subscriptions; ROS 1 has none of that (durability
is a publisher-side `latch` flag, transparent to subscribers) and no `Node`
object to hang callbacks off. `nav2_msgs/NavigateToPose` and `actionlib`'s
`move_base_msgs/MoveBaseAction` are different action types with different
client APIs (futures vs. callback-style `done_cb`). Branching all of that
inside one file would obscure exactly the differences a maintainer needs to
see. The protocol layer (capabilities, deadman, config schema) is identical by
construction — see `adapter_ros2.py` for the twin.

WHAT IS DELIBERATELY NOT DONE
-----------------------------
This has never run against physical hardware (import-checked against a real
`rospy`/Noetic install on `scout`, nothing more — see
`adapters/adapter_ros1/config/scout_mini.yaml` for what was read out of that
robot's actual stack). Every topic name, frame and timeout below is a
hypothesis until a robot proves it. See docs/operations/hardware-bringup.md.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import signal
import threading
import time
import urllib.request
import zlib
from pathlib import Path
from typing import Any

import numpy as np
import rospy
import websockets
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Joy, PointCloud2
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener

# Keep perception independent of ROS packaging, as adapter_sim does.  Hardware
# containers run this file directly, so the repository root is not otherwise on
# sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adapters.perception.depth_projection import (
    point_for_bbox,
    point_for_depth_image,
    transform_point,
    transform_points,
)
from adapters.network_quality import read_link_quality
from adapters.runtime import (
    AdapterDetectionMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
    deep_merge,
    map_cloud_height_limits,
    stamp_seconds,
    yaw_of,
)

# Transport quantisation for `map_cloud` uploads, matching adapter_sim's
# `/api/adapter/cloud` convention: 1 cm keeps a scan well inside int16.
MAP_CLOUD_SCALE = 0.01
# Dedup edge for scan points before upload, metres. Matches the backend's
# default occupancy grid resolution (mapsvc.service.MapService) — one
# candidate point per grid cell is exactly what the raytracer needs, and
# finer than that would just be bandwidth spent on points that land in the
# same cell anyway.
MAP_CLOUD_VOXEL = 0.05
# Voxel edge for the optional 3D viewer, metres. This is deliberately coarser
# than the 2D raytracing lattice: points render as tiny sprites, so sending
# several returns inside one 10 cm cube only spends robot/network/backend time.
MAP_CLOUD_3D_VOXEL = 0.10

# The detector needs OpenCV and the inference sidecar's client; a robot image
# built without them still runs, just without perception.
try:
    from adapters.perception.object_detector import ObjectDetector, track_ids
except ImportError as exc:  # pragma: no cover - depends on the robot's install
    ObjectDetector = None
    track_ids = None
    OBJECT_DETECTOR_IMPORT_ERROR = exc
else:
    OBJECT_DETECTOR_IMPORT_ERROR = None

# move_base is the common case but not the only one; see `navigate_to` below.
try:
    import actionlib
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
except ImportError:  # pragma: no cover - depends on the robot's install
    actionlib = None
    MoveBaseAction = None
    MoveBaseGoal = None

DEFAULTS: dict[str, Any] = {
    "robot_type": "generic",
    "footprint_radius": 0.35,
    # Optional polygon in base_frame coordinates, x forward / y left. The
    # radius remains the conservative fallback for map filtering and older
    # protocol peers.
    "footprint": [],
    # Linux wireless interface to sample into the per-robot network heatmap.
    # "auto" uses the first row in /proc/net/wireless; empty disables it.
    "network_iface": "",
    # Frames. `map` and `base_link` are REP-105 names and the only two that must
    # exist; the pose is a tf2 lookup between them rather than a composition of
    # transforms we recognise, because a real TF tree has links we do not know
    # about (base_footprint, odom_combined, per-vendor intermediates).
    "map_frame": "map",
    "base_frame": "base_link",
    "topics": {
        "odom": "odom",
        "map": "map",
        # PointCloud2, for a robot whose SLAM stack registers a cloud but never
        # projects one to a 2D OccupancyGrid (LVI-SAM, LOAM-family stacks in
        # general). Mutually additive with `map`, not a fallback for it: the
        # backend raytraces this into a grid itself (mapsvc/scan_grid.py)
        # rather than the adapter needing its own occupancy-grid mapper.
        "map_cloud": "",
        "plan": "plan",
        "cmd_vel": "cmd_vel",
        "battery": "",       # empty disables the capability
        "camera": "",
        "camera_compressed": "",
        # RGB-aligned depth image and its CameraInfo. Preferred over a point
        # cloud because many depth drivers publish unordered clouds by default.
        "camera_depth": "",
        "camera_info": "",
        # Colour CameraInfo. Set this when depth is *not* RGB-aligned so a
        # detection box in the operator image can be joined to a slower,
        # independently-published depth stream. Leave empty when `camera_info`
        # already describes aligned depth (the usual RGB-D case).
        "camera_color_info": "",
        # Organised PointCloud2 whose pixels are aligned with the RGB image.
        # When present, detections gain a map_position; otherwise bbox-only
        # detection continues unchanged.
        "camera_depth_points": "",
        # `move_base_simple/goal`-style single-goal topic (geometry_msgs/PoseStamped) —
        # the interface stacks like CMU's local_planner use instead of an
        # actionlib server. Takes priority over `actions.navigate_to_pose`
        # when both are configured: a robot only ever has one real navigation
        # stack, and this is the more specific of the two settings.
        "nav_goal": "",
        # std_msgs/Int8 safety-stop local_planner's pathFollower listens on
        # (1 = halt, 0 = resume). Optional even when `nav_goal` is set — a
        # stack without one just can't be preempted as cleanly by teleop or
        # cancel_goal, which then fall back to zeroing cmd_vel once.
        "nav_stop": "",
        # Where a topic-based nav stack's OWN cmd_vel output lands when it has
        # been remapped away from the real `/cmd_vel` (see
        # launch/local_planner_muxed.launch). If set, this adapter relays it
        # to the real `cmd_vel` topic ONLY while nav_status is "active" — the
        # adapter becomes the sole arbiter of the real topic, so a nav stack
        # that publishes continuously even when idle (confirmed true of
        # local_planner's pathFollower) never gets to race teleop for it.
        # Leave empty for a nav stack that already respects nav_stop/cancel
        # cleanly on its own.
        "nav_cmd_vel": "",
        # sensor_msgs/Joy, for a nav stack whose speed AND path direction both
        # come from a joystick with no software override (confirmed true of
        # local_planner: pathFollower.joystickHandler sets joySpeed = |axes[1]|
        # and localPlanner.joystickHandler sets joyDir = atan2(axes[2], axes[1])
        # — neither has an autonomyMode gate, unlike every other speed/direction
        # source either node has). If set, this adapter fakes both axes every
        # tick from the real bearing to the goal (see _pump_nav_joy) while
        # nav_status is "active", zero otherwise. Real safety still comes from
        # nav_cmd_vel only being relayed while nav_status is "active"
        # (_on_nav_cmd_vel) — this does not need to be itself trusted as a
        # safety mechanism, only a plausible one.
        "nav_joy": "",
    },
    # Magnitude of the fake nav_joy vector, 0..1 — NOT a speed by itself, see
    # _pump_nav_joy: axes[1]/axes[2] together encode the bearing to the goal,
    # and this is that vector's length. The stack rescales its own output to
    # maxSpeed regardless (confirmed live), so this mostly just needs to stay
    # comfortably nonzero after being split into cos/sin components.
    "nav_joy_throttle": 0.5,
    # Height band for `map_cloud` points, metres, in the map_frame (NOT
    # relative to the robot — this stack has no live z estimate to be relative
    # to, since the EKF publishing map_frame runs `two_d_mode: true`). Points
    # outside are dropped before upload: ground and ceiling returns would
    # otherwise flood the grid with false occupied cells. The default is a
    # starting guess, not a calibration — verify against the actual map that
    # comes out, per docs/operations/hardware-bringup.md.
    # min_z/max_z are map-frame limits by default. A profile may add floor_z;
    # then they mean physical heights above the floor and the adapter adds that
    # map-frame floor reference before filtering.
    "map_cloud_height_band": {"min_z": -0.3, "max_z": 0.5},
    # How close counts as "arrived" for a `nav_goal`-topic navigation stack,
    # metres. Unlike actionlib there is no explicit success signal to wait
    # for, so this adapter watches its own tf2 pose against the goal.
    "nav_goal_tolerance_m": 0.5,
    # `move_base`'s actionlib namespace — the ROS 1 convention, the way
    # `navigate_to_pose` is the ROS 2/Nav2 one.
    "actions": {"navigate_to_pose": "move_base"},
    "rates": {
        "state_hz": 5.0,
        "map_period_s": 2.0,
        "cloud_period_s": 4.0,
        "camera_period_s": 0.2,   # detection poll; hardware video is WebRTC
    },
    "perception": {
        "enabled": True,
        "period_s": 0.2,
        "sensitivity": 0.55,
        # Catalog classes this robot looks for (adapters/perception/catalog.py).
        # Empty means all of them; the dashboard's own selection overrides this
        # on the next settings refresh either way.
        "classes": [],
        # Empty uses SWARMDECK_DETECTOR_URL, then localhost:8091.
        "detector_url": "",
        "depth_min_m": 0.15,
        "depth_max_m": 8.0,
        "depth_max_age_s": 0.35,
    },
    # A drive command older than this stops the robot. Teleop over a network is
    # only safe with a deadman: if the link drops mid-command the robot must not
    # keep executing the last velocity it heard.
    "drive_timeout_s": 0.45,
    # The same deadman for autonomy — see the matching note in adapter_ros2.py.
    # `drive_timeout_s` works because held teleop repeats; an active goal does
    # not, so `_on_nav_cmd_vel` would relay pathFollower's ~27 Hz stream for the
    # whole goal with no operator attached. Cost of getting this wrong was
    # demonstrated on Botman, 2026-08-12.
    "link_timeout_s": 1.5,
    # Websocket keepalive — see the matching note in adapter_ros2.py. The
    # library defaults (20 s / 20 s) leave ~40 s in which `link_ok()` reads true
    # off our own completing sends while nothing reaches the operator, and an
    # active goal keeps running for all of it.
    "ping_interval_s": 2.0,
    "ping_timeout_s": 4.0,
    # How long a map/scan/cloud upload may take before the adapter gives up.
    #
    # This was hardcoded at 5 s, chosen when the uploads shared a coroutine with
    # the state pump: a longer wait there meant a longer telemetry blackout, so
    # the timeout had to stay under the backend's 4 s OFFLINE_AFTER_S. Since
    # `tx_maps` was split out, a slow upload costs nothing but its own latency,
    # and the ceiling is free to reflect what the BACKEND actually needs.
    #
    # It needs much more than 5 s. Every robot's scan queues behind one
    # registration lock on the server, so the wait scales with fleet size: on the
    # live four-robot fleet essentially every scan upload was hitting the 5 s
    # timeout and being DISCARDED. That is not a cosmetic loss — every robot here
    # runs `topics.map: ""` (no OccupancyGrid publisher), so the scan endpoint is
    # the ONLY source its map has, and the maps were starving.
    #
    # A discarded scan is worse than a late one: the points carry the pose they
    # were captured at, so arriving late costs nothing but freshness.
    "upload_timeout_s": 25.0,
}


class HardwareBridge(
    AdapterDetectionMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
):
    """One real robot's ROS interface, expressed as the SwarmDeck protocol."""

    _TRACK_IDS = staticmethod(track_ids) if track_ids is not None else None

    # Deliberately stale — see the matching default in adapter_ros2.py. A link
    # nobody has been heard on is not a link that may drive the robot.
    _last_link_at: float = 0.0

    def __init__(self, robot_id: str, cfg: dict, http_url: str) -> None:
        # No `Node` object in rospy — subscriptions/publishers/logging are all
        # module-level, so unlike `adapter_ros2.HardwareBridge` this takes no
        # node argument.
        self.id = robot_id
        self.cfg = cfg
        self.http_url = http_url
        self.t0 = time.monotonic()

        self.map_frame = cfg["map_frame"]
        self.base_frame = cfg["base_frame"]
        topics = cfg["topics"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False
        self._scan_points: np.ndarray | None = None
        # Sensor pose when `_scan_points` was captured; see _on_map_cloud.
        self._scan_origin: dict[str, float] | None = None
        self._scan_dirty = False
        self._cloud_points: np.ndarray | None = None
        self._cloud_dirty = False
        self._last_cloud_prepare_at = 0.0
        self.planned_path: list[dict[str, float]] = []
        self.battery: float | None = None
        self.nav_status = "idle"
        self.mode = "idle"
        self.goal: dict[str, float] | None = None
        self._goal_generation = 0
        self._last_drive_at = 0.0
        self._camera_encoding_warned = False
        # Newest frame awaiting detection, as (jpeg, header). The ROS callback
        # only ever assigns it; run_detection() consumes it from a worker thread.
        self._detect_pending: tuple[bytes, Any] | None = None
        self._camera_depth_image: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._camera_color_info: CameraInfo | None = None
        self._camera_depth_cloud: PointCloud2 | None = None
        perception = cfg.get("perception", {})
        self._detector = None
        self._detection_enabled = bool(perception.get("enabled", True))
        if self._detection_enabled and (topics.get("camera") or topics.get("camera_compressed")):
            if ObjectDetector is None:
                self._detection_enabled = False
                rospy.logwarn(
                    f"[{self.id}] camera detection disabled: "
                    f"{OBJECT_DETECTOR_IMPORT_ERROR}"
                )
            else:
                self._detector = ObjectDetector(
                    perception.get("sensitivity", 0.55),
                    perception.get("detector_url") or None,
                    classes=perception.get("classes"),
                )
        self._detection_period_s = max(0.05, float(perception.get("period_s", 0.2)))
        self._last_detection_at = 0.0
        self._detections: list[dict] | None = None
        self._pose_warned = False
        self._plan_frame_warned = False

        # ROS 1 has no subscriber-side durability setting: a latched publisher
        # (the ROS 1 equivalent of ROS 2's TRANSIENT_LOCAL) delivers its last
        # message to any new subscriber automatically. Nothing to configure
        # here, unlike `adapter_ros2`, where the subscriber's QoS must
        # independently declare TRANSIENT_LOCAL or it silently gets nothing.
        if topics.get("odom"):
            rospy.Subscriber(topics["odom"], Odometry, self._on_odom, queue_size=10)
        if topics.get("map"):
            rospy.Subscriber(topics["map"], OccupancyGrid, self._on_map, queue_size=1)
        if topics.get("map_cloud"):
            rospy.Subscriber(
                topics["map_cloud"], PointCloud2, self._on_map_cloud, queue_size=1
            )
        if topics.get("plan"):
            rospy.Subscriber(topics["plan"], NavPath, self._on_plan, queue_size=10)
        if topics.get("battery"):
            rospy.Subscriber(topics["battery"], BatteryState, self._on_battery, queue_size=10)
        # Prefer compressed: a raw camera stream at full rate is the single most
        # expensive thing an adapter can subscribe to over a robot's network.
        # Frames stay on-robot for detection; the operator picture is WebRTC.
        if topics.get("camera_compressed"):
            rospy.Subscriber(
                topics["camera_compressed"], CompressedImage,
                self._on_camera_compressed, queue_size=1,
            )
        elif topics.get("camera"):
            rospy.Subscriber(topics["camera"], Image, self._on_camera_raw, queue_size=1)
        if topics.get("camera_depth_points"):
            rospy.Subscriber(
                topics["camera_depth_points"],
                PointCloud2,
                self._on_camera_depth_cloud,
                queue_size=1,
            )
        if topics.get("camera_depth"):
            rospy.Subscriber(
                topics["camera_depth"], Image, self._on_camera_depth, queue_size=1
            )
        if topics.get("camera_info"):
            rospy.Subscriber(
                topics["camera_info"], CameraInfo, self._on_camera_info, queue_size=1
            )
        if topics.get("camera_color_info"):
            rospy.Subscriber(
                topics["camera_color_info"],
                CameraInfo,
                self._on_camera_color_info,
                queue_size=1,
            )
        if topics.get("nav_cmd_vel"):
            rospy.Subscriber(
                topics["nav_cmd_vel"], Twist, self._on_nav_cmd_vel, queue_size=10
            )

        self.pub_cmd = (
            rospy.Publisher(topics["cmd_vel"], Twist, queue_size=10)
            if topics.get("cmd_vel") else None
        )
        self.pub_nav_goal = (
            rospy.Publisher(topics["nav_goal"], PoseStamped, queue_size=1)
            if topics.get("nav_goal") else None
        )
        self.pub_nav_stop = (
            rospy.Publisher(topics["nav_stop"], Int8, queue_size=1)
            if topics.get("nav_stop") else None
        )
        self.pub_nav_joy = (
            rospy.Publisher(topics["nav_joy"], Joy, queue_size=1)
            if topics.get("nav_joy") else None
        )
        self._nav_joy_throttle = float(cfg.get("nav_joy_throttle", 0.5))

        # `nav_goal` (topic-based, e.g. local_planner) takes priority over
        # `actions.navigate_to_pose` (actionlib, e.g. move_base) when both are
        # configured — see the DEFAULTS comment on `topics.nav_goal`.
        action_name = cfg.get("actions", {}).get("navigate_to_pose")
        self.nav_client = None
        if self.pub_nav_goal is None and action_name and actionlib is not None:
            self.nav_client = actionlib.SimpleActionClient(action_name, MoveBaseAction)

        # The deadman runs off the ROBOT's clock, not the operator link — see
        # the identical timer in `adapter_ros2.HardwareBridge.__init__` for why
        # driving it from a websocket send loop cannot be trusted: the one
        # failure it exists to cover (a wedged link) is the one that stops that
        # loop from running it.
        self._last_link_at = time.monotonic()
        # Newest operator drive intent, applied by the timer — see
        # `note_drive_command`.
        self._pending_drive: tuple[float, float] | None = None
        self._watchdog_timer = rospy.Timer(
            rospy.Duration(0.05), lambda _event: self._watchdogs()
        )

    # ------------------------------------------------------------- capabilities

    def _network_quality(self, iface: str):
        # Keep the legacy module-level seam available to offline callers/tests.
        return read_link_quality(iface)

    def capabilities(self) -> list[str]:
        """Only what this robot can actually honour (protocol rule 4)."""
        caps: list[str] = []
        if self.nav_client is not None or self.pub_nav_goal is not None:
            caps.append("navigate")
        if self.cfg["topics"].get("map") or self.cfg["topics"].get("map_cloud"):
            caps.append("map")
        if self.cfg["topics"].get("camera") or self.cfg["topics"].get("camera_compressed"):
            caps.append("camera")
        if self.cfg["topics"].get("battery"):
            caps.append("battery")
        if self.cfg.get("network_iface"):
            caps.append("network")
        if self.pub_cmd is not None:
            caps.append("estop")
        return caps

    # ------------------------------------------------------------- ROS inputs

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Prepare one registered XYZ cloud for both map consumers.

        The 3D viewer keeps XYZ and downsamples in 10 cm voxels. The 2D map
        independently height-filters the same input, drops Z, and deduplicates
        at the occupancy-grid resolution before raytracing. Keeping these as
        two products matters: a display cloud must not inherit the obstacle
        height band and lose the floor, ceiling, or upper wall structure.

        Both reductions happen here rather than server-side because this is the
        expensive end of the link. A registered lidar scan can contain tens of
        thousands of points that are visually indistinguishable after upload.
        """
        points = self._cloud_xyz(msg)
        if not len(points):
            return

        now = time.monotonic()
        cloud_period = max(
            0.1, float(self.cfg.get("rates", {}).get("cloud_period_s", 4.0))
        )
        # The source runs at lidar rate (~5 Hz on TARS), but the viewer polls
        # slowly. A full 3D np.unique() on every scan spent roughly a third of a
        # CPU core preparing clouds that could never be uploaded. Reduce only
        # when the next display sample is due; the 2D scan below remains live.
        if now - self._last_cloud_prepare_at >= cloud_period:
            self._last_cloud_prepare_at = now
            cloud_keys = np.round(points / MAP_CLOUD_3D_VOXEL).astype(np.int32)
            _, cloud_keep = np.unique(cloud_keys, axis=0, return_index=True)
            self._cloud_points = points[cloud_keep]
            self._cloud_dirty = True

        min_z, max_z = map_cloud_height_limits(
            self.cfg.get("map_cloud_height_band")
        )
        xy = points[(points[:, 2] >= min_z) & (points[:, 2] <= max_z)][:, :2]
        if not len(xy):
            self._scan_points = np.zeros((0, 2), dtype=np.float32)
            self._scan_dirty = True
            return
        keys = np.round(xy / MAP_CLOUD_VOXEL).astype(np.int32)
        _, keep = np.unique(keys, axis=0, return_index=True)
        self._scan_points = xy[keep]
        # Pair the points with the pose they were captured AT. upload_scan used
        # to read the pose at upload time, up to map_period_s later, and the
        # backend raytraces free space from that origin — so at 0.5 m/s with a
        # 2 s period every ray was traced from a point up to a metre from where
        # the beam actually left the sensor, carving free space through geometry
        # it never crossed. That corrupts precisely the free/occupied contrast
        # registration.py relies on to break rotational symmetry.
        self._scan_origin = self.map_pose()
        self._scan_dirty = True

    def _on_plan(self, msg: NavPath) -> None:
        """Publish the planner's intended route, in `map_frame`.

        The protocol says a planned path is in the robot's navigation-map frame,
        and Nav2's global plan already is — which is why this used to copy the
        poses straight through. A reactive local planner does not: TARS's
        `local_planner` publishes `/path` in `chassis_link`, a vehicle frame, and
        copying those numbers verbatim draws the route as though the robot were
        parked at the map origin facing +x. It looks plausible exactly once, at
        startup, and is wrong everywhere else.

        Transform once per message rather than once per pose: a local path is
        ~100 poses at 10 Hz, and they all share a frame and a stamp.
        """
        if not msg.poses:
            self.planned_path = []
            return

        frame = msg.header.frame_id.lstrip("/")
        if not frame or frame == self.map_frame:
            self.planned_path = [
                {"x": ps.pose.position.x, "y": ps.pose.position.y} for ps in msg.poses
            ]
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, frame, msg.header.stamp, rospy.Duration(0.1)
            )
        except Exception:
            # Drop the path rather than draw it in the wrong frame: an operator
            # reading a route that is confidently somewhere the robot is not is
            # worse off than one reading no route at all.
            if not self._plan_frame_warned:
                self._plan_frame_warned = True
                rospy.logwarn(
                    f"[{self.id}] no {self.map_frame} -> {frame} transform for the "
                    f"planned path; not publishing it. The plan topic is in a frame "
                    f"this robot's TF tree does not connect to {self.map_frame}."
                )
            self.planned_path = []
            return

        points = np.array(
            [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
             for ps in msg.poses],
            dtype=np.float64,
        )
        mapped = transform_points(points, tf.transform)
        if mapped is None:
            self.planned_path = []
            return
        self.planned_path = [
            {"x": round(float(x), 3), "y": round(float(y), 3)} for x, y in mapped[:, :2]
        ]

    def _on_camera_compressed(self, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower():
            return
        # Queue for detection; do NOT run inference here. See run_detection().
        self._detect_pending = (bytes(msg.data), getattr(msg, "header", None))

    def _on_camera_raw(self, msg: Image) -> None:
        # Detection takes JPEG (the sidecar posts it). Imported lazily so a
        # robot with no camera needs no OpenCV. This does not go to the backend;
        # hardware video is WebRTC.
        try:
            import cv2
        except ImportError:
            return
        frame = self._image_to_bgr(msg)
        if frame is None:
            if not self._camera_encoding_warned:
                self._camera_encoding_warned = True
                rospy.logwarn(
                    f"[{self.id}] cannot decode camera encoding "
                    f"{getattr(msg, 'encoding', '?')!r}; detection has no frames"
                )
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        self._detect_pending = (buf.tobytes(), getattr(msg, "header", None))

    def run_detection(self) -> None:
        """Detect on the newest queued frame. Runs OFF rospy's callback threads.

        Inference is a blocking HTTP round trip to the sidecar (up to
        `timeout_s`), and `_depth_map_position` adds a tf2 lookup per detection.
        Called straight from the subscription callback, as it used to be, that
        occupied a rospy dispatch thread several times a second and, on a
        single-threaded subscriber queue, delayed everything behind it.
        `adapter_sim` has always run detection off the ROS thread; this is the
        same arrangement, driven from `tx_camera`'s executor.
        """
        pending = self._detect_pending
        self._detect_pending = None
        if pending is None or not self._detection_due():
            return
        jpeg, image_header = pending
        try:
            import cv2

            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self._detect_bgr(frame, due_checked=True, image_header=image_header)
        except (ValueError, TypeError):
            return

    def _depth_image_kwargs(self, depth_header) -> dict | None:
        """Extra args for `point_for_depth_image`, or None to skip this frame.

        `camera_color_info` means the operator image and the depth image are
        not the same grid.  Sampling depth as if it were RGB-aligned would
        put the marker in the wrong place, so a missing optical TF is a skip
        rather than a fallback.
        """
        color_info = self._camera_color_info
        if color_info is None:
            return {}
        color_frame = getattr(getattr(color_info, "header", None), "frame_id", "") or ""
        depth_frame = getattr(depth_header, "frame_id", "") or ""
        if not color_frame or not depth_frame or color_frame == depth_frame:
            return {"color_camera_info": color_info}
        try:
            stamp = getattr(depth_header, "stamp", rospy.Time(0))
            tf = self.tf_buffer.lookup_transform(
                color_frame, depth_frame, stamp, rospy.Duration(0.1)
            )
            return {
                "color_camera_info": color_info,
                "depth_to_color": tf.transform,
            }
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0,
                f"[{self.id}] cannot join colour detection to depth: {exc}",
            )
            return None

    def _depth_map_position(
        self, bbox, image_header=None, polygon=None
    ) -> dict[str, float] | None:
        perception = self.cfg.get("perception", {})
        image_time = self._stamp_seconds(image_header)
        max_age = float(perception.get("depth_max_age_s", 0.35))
        min_range = float(perception.get("depth_min_m", 0.15))
        max_range = float(perception.get("depth_max_m", 8.0))
        camera_point = None
        source_header = None

        depth_image = self._camera_depth_image
        camera_info = self._camera_info
        if depth_image is not None and camera_info is not None:
            depth_header = getattr(depth_image, "header", None)
            depth_time = self._stamp_seconds(depth_header)
            if image_time is None or depth_time is None or abs(image_time - depth_time) <= max_age:
                extra = self._depth_image_kwargs(depth_header)
                if extra is not None:
                    configured_scale = perception.get("depth_scale")
                    camera_point = point_for_depth_image(
                        depth_image,
                        camera_info,
                        bbox,
                        polygon=polygon,
                        min_range_m=min_range,
                        max_range_m=max_range,
                        depth_scale=None if configured_scale is None else float(configured_scale),
                        **extra,
                    )
                    source_header = depth_header

        cloud = self._camera_depth_cloud
        if camera_point is None and cloud is not None:
            cloud_header = getattr(cloud, "header", None)
            cloud_time = self._stamp_seconds(cloud_header)
            if image_time is None or cloud_time is None or abs(image_time - cloud_time) <= max_age:
                camera_point = point_for_bbox(
                    cloud,
                    bbox,
                    polygon=polygon,
                    min_range_m=min_range,
                    max_range_m=max_range,
                )
                source_header = cloud_header
        if camera_point is None or source_header is None:
            return None
        frame_id = getattr(source_header, "frame_id", "")
        if not frame_id:
            return None
        try:
            if frame_id == self.map_frame:
                map_point = camera_point
            else:
                stamp = getattr(source_header, "stamp", rospy.Time(0))
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, frame_id, stamp, rospy.Duration(0.1)
                )
                map_point = transform_point(camera_point, tf.transform)
            if map_point is None:
                return None
            return {"x": round(float(map_point[0]), 3), "y": round(float(map_point[1]), 3)}
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0,
                f"[{self.id}] cannot place camera detection in {self.map_frame}: {exc}",
            )
            return None

    # ------------------------------------------------------------- pose

    def map_pose(self) -> dict[str, float]:
        """The robot's pose in its navigation-map frame, via tf2.

        A tf2 lookup rather than composing transforms we recognise by name —
        same reasoning as `adapter_ros2.map_pose`: a real robot's TF tree has
        links we do not know about, and hardcoding a chain through them is how
        an adapter ends up reporting a pose that is subtly wrong.

        Falls back to raw odometry only if TF is unavailable, and says so once —
        reporting the map origin forever would look like a stationary robot.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0)
            )
            t = tf.transform
            return {
                "x": t.translation.x,
                "y": t.translation.y,
                "yaw": yaw_of(t.rotation),
            }
        except Exception:
            fallback = getattr(self, "_odom_pose", None)
            if fallback is None:
                return {"x": 0.0, "y": 0.0, "yaw": 0.0}
            if not self._pose_warned:
                self._pose_warned = True
                rospy.logwarn(
                    f"[{self.id}] no {self.map_frame} -> {self.base_frame} transform; "
                    f"falling back to raw odometry, which DRIFTS. Check that SLAM or "
                    f"localisation is running and publishing TF."
                )
            return dict(fallback)

    # ------------------------------------------------------------- commands

    def drive(self, linear: float, angular: float) -> None:
        if self.pub_cmd is None:
            return
        moving = abs(linear) > 1e-3 or abs(angular) > 1e-3
        # Operator motion always preempts autonomy — the same rule
        # `adapter_ros2.drive` follows, and for the same reason.
        #
        # This used to be nested inside the `pub_nav_stop` branch below, which
        # meant it only ever ran on a `local_planner`-style stack. On a robot
        # driving `move_base` — every default ROS 1 config, where `nav_stop` is
        # empty because it is not a move_base concept — teleop left the action
        # goal running, and move_base publishes straight to the real cmd_vel, so
        # the operator and the planner fought over the topic.
        if moving and self.nav_status == "active":
            self.cancel_goal()
        # Belt-and-suspenders even with nav_cmd_vel relaying: also tell a nav
        # stack that respects nav_stop to actually stop trying.
        if moving and self.pub_nav_stop is not None:
            self.pub_nav_stop.publish(Int8(data=1))
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub_cmd.publish(twist)
        self.mode = "teleop" if moving else self.mode
        self._last_drive_at = time.monotonic() if moving else 0.0

    def navigate_to(self, goal: dict[str, float]) -> None:
        if self.pub_nav_goal is not None:
            self._navigate_to_topic(goal)
            return
        if self.nav_client is None:
            return
        if not self.nav_client.wait_for_server(rospy.Duration(2.0)):
            rospy.logwarn(
                f"[{self.id}] navigation action server not available; goal dropped"
            )
            self.nav_status = "failed"
            return
        self._goal_generation += 1
        generation = self._goal_generation

        msg = MoveBaseGoal()
        msg.target_pose.header.frame_id = self.map_frame
        msg.target_pose.header.stamp = rospy.Time.now()
        msg.target_pose.pose.position.x = float(goal["x"])
        msg.target_pose.pose.position.y = float(goal["y"])
        yaw = float(goal.get("yaw", 0.0))
        msg.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal = {"x": float(goal["x"]), "y": float(goal["y"])}
        self.nav_status = "active"
        self.mode = "nav"

        # actionlib is callback-style, not futures: `done_cb` fires from rospy's
        # own callback machinery, same as any subscriber. `SimpleActionClient`
        # tracks only its most recent goal, but a stale server response for an
        # already-superseded goal can still arrive — the generation guard below
        # is what `adapter_ros2` does for the same reason with action futures.
        self.nav_client.send_goal(
            msg, done_cb=lambda status, result, g=generation: self._on_goal_done(status, g)
        )

    def _on_goal_done(self, status: int, generation: int) -> None:
        if generation != self._goal_generation:
            return
        from actionlib_msgs.msg import GoalStatus

        self.nav_status = {
            GoalStatus.SUCCEEDED: "succeeded",
            GoalStatus.PREEMPTED: "cancelled",
        }.get(status, "failed")
        self.mode = "idle"
        self.goal = None

    def _navigate_to_topic(self, goal: dict[str, float]) -> None:
        """`move_base_simple/goal`-style: a plain publish, not an action.

        No actionlib means no "accepted"/"succeeded" callback — a stack like
        local_planner just starts driving. Progress is instead polled each
        state tick by `_check_topic_nav_progress` against this adapter's own
        tf2 pose, the same source `state()` already reports from.
        """
        self._goal_generation += 1
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(goal["x"])
        msg.pose.position.y = float(goal["y"])
        yaw = float(goal.get("yaw", 0.0))
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal = {"x": float(goal["x"]), "y": float(goal["y"])}
        self.nav_status = "active"
        self.mode = "nav"
        if self.pub_nav_stop is not None:
            self.pub_nav_stop.publish(Int8(data=0))  # release any prior safety stop
        self.pub_nav_goal.publish(msg)

    def _check_topic_nav_progress(self) -> None:
        """Declare arrival once close enough — the only "done" signal a
        topic-based nav stack gives this adapter."""
        if self.pub_nav_goal is None or self.nav_status != "active" or self.goal is None:
            return
        pose = self.map_pose()
        dist = math.hypot(pose["x"] - self.goal["x"], pose["y"] - self.goal["y"])
        if dist <= float(self.cfg.get("nav_goal_tolerance_m", 0.5)):
            self.nav_status = "succeeded"
            self.mode = "idle"
            self.goal = None

    def _pump_nav_joy(self) -> None:
        """Fake the joystick pathFollower's speed AND localPlanner's path
        DIRECTION both come from, unconditionally, with `autonomyMode: false`.

        Two independent things read this, in two different executables:

          * pathFollower.joystickHandler: `joySpeed = |axes[1]|` — the speed
            gate (see the DEFAULTS comment on `topics.nav_joy`).
          * localPlanner.joystickHandler: `joyDir = atan2(axes[2], axes[1])`
            — which CANDIDATE PATH direction gets selected from the path
            library. This is NOT derived from goalX/Y at all when
            `autonomyMode` is false; only the joystick's own axes drive it.

        Publishing a constant axes[1]=throttle, axes[2]=0 (an earlier version
        of this method) therefore always signalled "goal straight ahead" —
        confirmed live: a goal placed behind the robot still drove forward,
        because joyDir never moved off zero. Fixed by computing the real
        bearing from the robot's current pose to the goal, every tick (the
        bearing changes as the robot moves/turns, same reason the
        RelativeSetPoint scheme needs continuous updates too), and encoding
        it into both axes so `atan2(axes[2], axes[1])` equals that bearing.
        `axes[1]` can go negative for a goal behind the robot — pathFollower
        takes `|axes[1]|` for speed, so sign only ever affects direction,
        never zeroes the speed gate.

        This build has no joystick staleness timeout (the check exists in
        source but is commented out) — publishing once would latch a stale
        direction/speed forever internally. Publish a fresh value every tick
        instead, the way a real joystick would, and always zero when not
        actively navigating. None of this needs to be trusted as the safety
        mechanism — `_on_nav_cmd_vel` only relays to the real cmd_vel while
        nav_status is "active" regardless of what pathFollower thinks
        internally, and that's what actually stops the robot.
        """
        if self.pub_nav_joy is None:
            return
        msg = Joy()
        if self.nav_status == "active" and self.goal is not None:
            pose = self.map_pose()
            dx = self.goal["x"] - pose["x"]
            dy = self.goal["y"] - pose["y"]
            c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
            forward = dx * c + dy * s
            left = -dx * s + dy * c
            bearing = math.atan2(left, forward)
            t = self._nav_joy_throttle
            msg.axes = [0.0, math.cos(bearing) * t, math.sin(bearing) * t]
        else:
            msg.axes = [0.0, 0.0, 0.0]
        msg.buttons = [0, 0, 0, 0]
        self.pub_nav_joy.publish(msg)

    def cancel_goal(self) -> None:
        self._goal_generation += 1
        if self.nav_client is not None:
            try:
                self.nav_client.cancel_goal()
            except Exception:
                pass
        if self.pub_nav_stop is not None:
            self.pub_nav_stop.publish(Int8(data=1))
        self.goal = None
        self.nav_status = "cancelled"
        self.mode = "idle"

    # ------------------------------------------------------------- uploads

    def upload_map(self) -> dict[str, Any] | None:
        """Push the occupancy grid; return the `map_meta` to send afterwards."""
        if not self._grid_dirty or self.grid is None:
            return None
        self._grid_dirty = False
        g = self.grid
        cells = np.array(g.data, dtype=np.int8)
        body = zlib.compress(np.ascontiguousarray(cells).tobytes())
        url = (
            f"{self.http_url}/api/adapter/map?robot_id={self.id}"
            f"&resolution={g.info.resolution}&width={g.info.width}"
            f"&height={g.info.height}"
            f"&origin_x={g.info.origin.position.x}"
            f"&origin_y={g.info.origin.position.y}"
        )
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    url, data=body, headers={"Content-Type": "application/octet-stream"}
                ),
                timeout=float(self.cfg["upload_timeout_s"]),
            ).read()
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] map upload failed: {exc}")
            return None
        return {
            "type": "map_meta",
            "robot_id": self.id,
            "resolution": g.info.resolution,
            "width": g.info.width,
            "height": g.info.height,
            "origin": {
                "x": g.info.origin.position.x,
                "y": g.info.origin.position.y,
            },
        }

    def upload_scan(self) -> None:
        """Push the latest deduplicated scan; the backend raytraces it.

        No `map_meta` to send afterwards, unlike `upload_map` — the merged
        grid's diff reaches the GUI through the existing 2 Hz patch broadcast
        regardless of which upload path fed it (`app.py`'s `map_loop`), so
        there is nothing extra to announce here.
        """
        if not self._scan_dirty or self._scan_points is None:
            return
        self._scan_dirty = False
        if not len(self._scan_points):
            return
        # The pose these points were captured at, not the pose now. See _on_map_cloud.
        origin = self._scan_origin or self.map_pose()
        quantised = np.round(self._scan_points / MAP_CLOUD_SCALE).astype(np.int16)
        url = (
            f"{self.http_url}/api/adapter/scan?robot_id={self.id}"
            f"&origin_x={origin['x']}&origin_y={origin['y']}&scale={MAP_CLOUD_SCALE}"
        )
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    url, data=zlib.compress(quantised.tobytes()),
                    headers={"Content-Type": "application/octet-stream"},
                ),
                timeout=float(self.cfg["upload_timeout_s"]),
            ).read()
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] scan upload failed: {exc}")

    def upload_cloud(self) -> None:
        """Push the latest registered XYZ scan to the optional 3D viewer.

        Unlike ``upload_scan``, this intentionally keeps Z and does no height
        filtering. The backend already knows how to place each robot's local
        cloud into the merged frame; this is the same transport used by the
        simulation adapter and is not tied to a particular SLAM package.
        """
        if not self._cloud_dirty or self._cloud_points is None:
            return
        self._cloud_dirty = False
        if not len(self._cloud_points):
            return
        quantised = np.round(self._cloud_points / MAP_CLOUD_SCALE).astype(np.int16)
        url = (
            f"{self.http_url}/api/adapter/cloud?robot_id={self.id}"
            f"&scale={MAP_CLOUD_SCALE}"
        )
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    url, data=zlib.compress(quantised.tobytes(), 1),
                    headers={"Content-Type": "application/octet-stream"},
                ),
                timeout=float(self.cfg["upload_timeout_s"]),
            ).read()
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] 3D cloud upload failed: {exc}")


async def run_until_first_failure(*coros: Any) -> None:
    """Run coroutines together; the first to fail cancels the rest and raises.

    The link's coroutines share one socket, so any one of them dying makes the
    others meaningless — they would go on writing to a closing connection, and
    their exceptions would surface much later as "Task exception was never
    retrieved" rather than as the reconnect this is supposed to trigger.
    """
    tasks = [asyncio.ensure_future(c) for c in coros]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for task in done:
            task.result()  # re-raise into run_robot's reconnect handler
    finally:
        for task in tasks:
            task.cancel()
        # Cancelling a task blocked in run_in_executor does not stop the worker
        # thread; the in-flight urllib call still runs to its own timeout. It
        # writes nothing but the upload it was already making, so letting it
        # finish unobserved is safe.
        await asyncio.gather(*tasks, return_exceptions=True)


async def run_robot(bridge: HardwareBridge, ws_url: str) -> None:
    """Connect, announce, then pump state until the link drops. Repeat.

    Protocol rule 2: reconnect with backoff and re-send `hello` every time. The
    backoff matters more on hardware than in simulation — a robot that hammers a
    downed backend over Wi-Fi wastes the airtime its own teleop needs.
    """
    backoff = 1.0
    while True:
        try:
            async with websockets.connect(
                ws_url,
                ping_interval=float(bridge.cfg["ping_interval_s"]),
                ping_timeout=float(bridge.cfg["ping_timeout_s"]),
            ) as ws:
                await ws.send(json.dumps({
                    "type": "hello",
                    "protocol": 1,
                    "robot_id": bridge.id,
                    "robot_type": bridge.cfg["robot_type"],
                    "adapter": "adapter_ros1/0.1.0",
                    "ros": "noetic",
                    # `local`: a real robot's pose and grid are in its own
                    # navigation-map frame. The backend does the merging.
                    "coordinate_frame": "local",
                    "capabilities": bridge.capabilities(),
                    "footprint_radius": bridge.cfg["footprint_radius"],
                    "footprint": bridge.cfg.get("footprint") or None,
                }))
                rospy.loginfo(
                    f"[{bridge.id}] connected; capabilities="
                    f"{bridge.capabilities()}"
                )
                backoff = 1.0
                bridge.note_link_activity()

                async def rx() -> None:
                    async for raw in ws:
                        bridge.note_link_activity()
                        try:
                            msg = json.loads(raw)
                        except ValueError:
                            continue
                        kind = msg.get("type")
                        if kind == "navigate_to":
                            # Off-thread: sending a goal talks to an action
                            # server that may be slow or absent, and blocking
                            # this coroutine stalls every other one on this link.
                            await asyncio.get_running_loop().run_in_executor(
                                None, bridge.navigate_to, msg.get("goal", {})
                            )
                        elif kind == "cancel_goal":
                            bridge.cancel_goal()
                        elif kind == "drive":
                            # Latched, not executed here — see note_drive_command.
                            bridge.note_drive_command(
                                msg.get("linear", 0.0), msg.get("angular", 0.0)
                            )
                        elif kind == "stop":
                            bridge.stop()
                        elif kind == "set_mode":
                            bridge.mode = msg.get("mode", bridge.mode)
                        # Rule 3: unknown types are ignored, not fatal.

                # Telemetry and the bulk uploads run as separate coroutines, so
                # one socket now has several writers. `websockets` does not
                # serialise overlapping `send()` calls, and two coroutines
                # writing at once can interleave frames on the wire.
                send_lock = asyncio.Lock()

                async def send(payload: dict[str, Any]) -> None:
                    async with send_lock:
                        await ws.send(json.dumps(payload))

                async def tx_state() -> None:
                    """Telemetry and nav upkeep. Nothing slow may run in here.

                    This used to share one loop with the uploads below, which
                    tied liveness to backend latency: `upload_map` blocks for up
                    to its 5 s timeout, and no state frame went out while it did.
                    The server calls a robot offline after 4 s (OFFLINE_AFTER_S)
                    and `link_watchdog` cancels an active goal after
                    `link_timeout_s`, so a single slow upload took the robot off
                    the dashboard AND stopped its mission. Worse here than on
                    ROS 2: `_pump_nav_joy` is what feeds pathFollower its speed
                    and localPlanner its direction, so a stalled loop did not
                    merely stop REPORTING the robot, it stopped STEERING it.

                    That made the failure a property of FLEET SIZE rather than of
                    this robot: every map upload queues behind one lock on the
                    server, so a fourth robot connecting is what makes the first
                    one drop out.
                    """
                    period = 1.0 / float(bridge.cfg["rates"]["state_hz"])
                    while True:
                        await send(bridge.state())
                        # A completed send is the autonomy deadman's freshness
                        # signal; on a wedged link this is where it stops
                        # completing, which is the point.
                        bridge.note_link_activity()
                        # No drive_watchdog() here: it runs on a ROS timer so it
                        # keeps ticking when this loop cannot. See __init__.
                        bridge._check_topic_nav_progress()
                        bridge._pump_nav_joy()
                        await asyncio.sleep(period)

                async def tx_maps() -> None:
                    """Occupancy, scans and clouds — the backend-bound path."""
                    rates = bridge.cfg["rates"]
                    tick = 1.0 / float(rates["state_hz"])
                    loop = asyncio.get_running_loop()
                    last_map = 0.0
                    last_cloud = 0.0
                    last_settings = 0.0
                    while True:
                        now = time.monotonic()
                        if now - last_map > float(rates["map_period_s"]):
                            meta = await loop.run_in_executor(None, bridge.upload_map)
                            if meta:
                                await send(meta)
                            await loop.run_in_executor(None, bridge.upload_scan)
                            # Measured from COMPLETION, not from the start of the
                            # attempt. Against a backend slow enough that an
                            # upload outlasts its own period — which is the state
                            # a four-robot fleet puts the server in — starting the
                            # clock at the top means the next upload is already
                            # due the moment this one returns, and the adapter
                            # spins, piling work onto a server that is already
                            # behind. Maps now arrive as fast as the backend can
                            # accept them and no faster.
                            last_map = time.monotonic()
                        if now - last_cloud > float(rates["cloud_period_s"]):
                            await loop.run_in_executor(None, bridge.upload_cloud)
                            last_cloud = time.monotonic()
                        if now - last_settings > 5.0:
                            last_settings = now
                            await loop.run_in_executor(None, bridge.refresh_settings)
                        await asyncio.sleep(tick)

                async def tx_camera() -> None:
                    """Detections, on their own clock.

                    Separate from `tx_maps` because the two block for very
                    different reasons: a map upload waits on the server's
                    registration lock for seconds at a time, and sharing a loop
                    with it stalled detection for exactly as long.
                    """
                    rates = bridge.cfg["rates"]
                    tick = 1.0 / float(rates["state_hz"])
                    loop = asyncio.get_running_loop()
                    last_cam = 0.0
                    while True:
                        now = time.monotonic()
                        if now - last_cam > float(rates["camera_period_s"]):
                            await loop.run_in_executor(None, bridge.run_detection)
                            detections = bridge.take_detections()
                            if detections is not None:
                                await send({
                                    "type": "detections",
                                    "robot_id": bridge.id,
                                    "t_mono": round(now - bridge.t0, 4),
                                    "camera": "front",
                                    "items": detections,
                                })
                            last_cam = time.monotonic()
                        await asyncio.sleep(tick)

                # Not `gather`: it leaves the siblings of a failed coroutine
                # running against a socket that is already closing, and their
                # exceptions surface later as "never retrieved". The first one to
                # fail is the reconnect trigger, and the rest must be torn down
                # with it.
                await run_until_first_failure(rx(), tx_state(), tx_maps(), tx_camera())
        except Exception as exc:
            rospy.logwarn(f"[{bridge.id}] disconnected ({exc}); retrying in {backoff:.0f}s")
            # Stop the robot on link loss. A robot that keeps driving after
            # losing its operator is the failure that hurts someone.
            #
            # cancel_goal() FIRST: while nav_status is still "active" the relay
            # in `_on_nav_cmd_vel` overwrites a zero Twist with pathFollower's
            # next sample, so zeroing alone stops nothing.
            try:
                bridge.cancel_goal()
                bridge.drive(0.0, 0.0)
            except Exception:
                pass
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, 30.0)


def load_config(path: str | None) -> dict:
    cfg = dict(DEFAULTS)
    if path:
        loaded = yaml.safe_load(Path(path).read_text()) or {}
        cfg = deep_merge(cfg, loaded)
    return cfg


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robot-id", required=True,
                    help="Stable identity, used as the key everywhere (rule 5)")
    ap.add_argument("--config", default="",
                    help="YAML of topics/frames/rates for this robot type")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    cfg = load_config(args.config or None)
    ws_url = f"ws://{args.host}:{args.port}/adapter"
    http_url = f"http://{args.host}:{args.port}"

    # disable_signals: we drive our own asyncio.run() as the process's main
    # loop and handle KeyboardInterrupt ourselves, same shape as adapter_ros2's
    # rclpy.init()/rclpy.shutdown() bracketing.
    rospy.init_node(f"swarmdeck_adapter_{args.robot_id}", anonymous=False, disable_signals=True)
    bridge = HardwareBridge(args.robot_id, cfg, http_url)

    # Unlike rclpy, rospy dispatches subscriber/action callbacks on its own
    # threads regardless of spin() — this thread exists to hold the process
    # open on ROS's shutdown machinery, not to pump callbacks.
    spin = threading.Thread(target=rospy.spin, daemon=True)
    spin.start()

    # SIGTERM, not just Ctrl-C — see the matching handler in adapter_ros2.py.
    # Its default action kills the interpreter outright, so the `finally` below
    # never runs and nothing zeroes the driver on a container restart.
    def _on_signal(_signum, _frame):
        bridge.stop_for_exit()
        raise KeyboardInterrupt

    for _sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(_sig, _on_signal)

    try:
        asyncio.run(run_robot(bridge, ws_url))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.stop_for_exit()
        except Exception:
            pass
        rospy.signal_shutdown("adapter exiting")


if __name__ == "__main__":
    main()
