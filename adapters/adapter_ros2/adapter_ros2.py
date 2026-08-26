#!/usr/bin/env python3
"""Hardware adapter: a real ROS 2 robot -> SwarmDeck adapter protocol.

    python3 adapter_ros2.py --robot-id spot_0 --config robot.yaml

One process per robot, running ON the robot (or on a machine that shares its ROS
graph). This and `adapter_sim` are the only components that know about both ROS
and the SwarmDeck protocol; the backend stays ROS-free.

WHY THIS IS NOT `adapter_sim` WITH DIFFERENT TOPIC NAMES
--------------------------------------------------------
`adapter_sim` bridges four simulated robots from one process and may assume the
simulator's conventions: every topic under `/<ns>/`, a `<ns>/tf` that carries the
whole chain, `map_frame` named by us, and a lidar whose extrinsics we chose. None
of that survives contact with a real robot, where the driver names topics, the
URDF names frames, and TF is global rather than namespaced.

So everything here is CONFIGURED, not assumed:

  * topic names come from a YAML file, one per robot type
  * frames come from that file too, and the pose is resolved through tf2 rather
    than by composing transforms we happened to recognise
  * capabilities are declared from what is actually configured, so a robot with
    no camera advertises none rather than advertising one and timing out

The protocol rule "never send a capability you cannot honour" is enforced here by
construction: a capability is only advertised if its topic or action is present
in the configuration AND resolves at runtime.

WHAT IS DELIBERATELY NOT DONE
-----------------------------
This has never run against physical hardware. It is written against the protocol
spec (adapters/protocol/README.md) and the working `adapter_sim` reference, and
its message construction is unit-tested, but every timeout, QoS choice and frame
name below is a hypothesis until a robot proves it. See docs/operations/hardware-bringup.md
for the order to validate them in.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import sys
import threading
import time
import urllib.parse
import urllib.request
import zlib
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import websockets
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

# Hardware containers run this file directly, so make the repository's shared
# perception helpers importable without requiring a Python package install.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "protocol"))

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
from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw
from adapters.map_downlink import NavMapClient, apply_to_occupancy_grid

# Transport quantisation for registered-cloud uploads. One centimetre keeps a
# normal building-scale map inside int16 while remaining finer than either
# consumer's display/grid resolution.
MAP_CLOUD_SCALE = 0.01
MAP_CLOUD_VOXEL = 0.05
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

# Nav2 is the common case but not the only one; see `navigate_to` below.
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on the robot's install
    NavigateToPose = None

try:
    from std_srvs.srv import Trigger
except ImportError:  # pragma: no cover - depends on the robot's install
    Trigger = None

try:
    from spot_msgs.action import Trajectory
except ImportError:  # pragma: no cover - depends on the robot's install
    Trajectory = None

try:
    from spot_msgs.srv import SetVelocity
except ImportError:  # pragma: no cover - depends on the robot's install
    SetVelocity = None

# Spot (and similar) body services. `stand` may also call `power_on` first.
BODY_ACTIONS = ("claim", "release", "sit", "stand")
# Trigger names that are not GUI body actions: motors, software e-stop allow,
# tablet keepalive clear, and the SDK stop used because Clearpath's ROS 2
# Trajectory server does not honour cancel/preempt.
BODY_SERVICE_NAMES = (
    *BODY_ACTIONS, "power_on", "stop", "estop_release", "clear_keepalive",
)
# Missing these is normal on non-Spot robots; do not warn.
OPTIONAL_BODY_SERVICES = frozenset(
    {"power_on", "estop_release", "clear_keepalive"}
)

DEFAULTS: dict[str, Any] = {
    "robot_type": "generic",
    "ros_distro": "jazzy",
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
        # OccupancyGrid the collaborative back-end warps into this robot's
        # map frame. Nav2's global static layer subscribes here. Local
        # costmaps must not.
        "nav_map": "/global_map",
        # Registered PointCloud2 for a 3D SLAM stack that does not publish an
        # OccupancyGrid. The backend raytraces a height-filtered XY view into
        # a grid and keeps a coarser XYZ view for the optional 3D panel.
        "map_cloud": "",
        "plan": "plan",
        "cmd_vel": "cmd_vel",
        "battery": "",       # empty disables the capability
        "camera": "",
        "camera_compressed": "",
        "camera_depth": "",
        "camera_info": "",
        # Colour CameraInfo. Set this when depth is *not* RGB-aligned so a
        # detection box in the operator image can be joined to a slower,
        # independently-published depth stream. Leave empty when `camera_info`
        # already describes aligned depth (the usual RGB-D case).
        "camera_color_info": "",
        # Organised PointCloud2 aligned pixel-for-pixel with the RGB image.
        # Optional: bbox-only detection still works when it is absent.
        "camera_depth_points": "",
        # Isolated output from a navigation stack. If set, the adapter relays
        # it to the real driver only while an action goal is active, keeping
        # teleop, cancellation and e-stop authoritative.
        "nav_cmd_vel": "",
        # Nav2's DWB controller trajectory. This is preferred over the global
        # planner path for the dashboard because it is the route selected for
        # the next control cycle. Empty disables the optional local route.
        "local_plan": "",
    },
    # min_z/max_z are map-frame limits by default. A profile may add floor_z;
    # then they mean physical heights above the floor and the adapter adds that
    # map-frame floor reference before filtering.
    "map_cloud_height_band": {"min_z": -0.3, "max_z": 0.5},
    # Keep ray-traced known-free cells white after the lidar moves on. Unknown
    # cells remain unknown; this only controls retention of observed free space.
    "retain_free_space": False,
    "actions": {
        "navigate_to_pose": "navigate_to_pose",
        # Spot: Clearpath `spot_msgs/Trajectory`. Empty on every other robot.
        "trajectory": "",
    },
    # Spot Trajectory goals are in `body`. duration_s must be > 0 or the
    # driver aborts. The high-level controller uses 30 s.
    "trajectory": {
        "frame": "body",
        "duration_s": 30.0,
        "precise_positioning": True,
        "disable_obstacle_avoidance": False,
        # Optional Spot SDK mobility limit, applied through /max_velocity
        # immediately before each trajectory. `duration_s` is only a timeout;
        # it does not control how quickly Spot walks to the target.
        "velocity_limit": {},
    },
    # Empty disables the `body` capability. Spot's Clearpath driver exposes
    # these as std_srvs/Trigger; a robot without them leaves them blank.
    "services": {
        "claim": "",
        "release": "",
        "sit": "",
        "stand": "",
        "power_on": "",
        "stop": "",
        "estop_release": "",
        "clear_keepalive": "",
        # Spot SDK mobility limit service (spot_msgs/SetVelocity).
        "max_velocity": "",
    },
    "rates": {
        "state_hz": 5.0,
        "map_period_s": 2.0,
        "cloud_period_s": 4.0,
        "camera_period_s": 0.2,   # detection poll; hardware video is H.264/WebRTC
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
    # The same deadman, for autonomy. `drive_timeout_s` protects teleop because
    # the GUI repeats `drive` while a button is held, so silence is detectable.
    # An active navigation goal sends nothing repeatedly — Nav2 drives the robot
    # from on board, and `_on_nav_cmd_vel` relays it purely on nav_status, which
    # nothing revokes when the operator link wedges. Without this an active goal
    # runs to completion with nobody watching, which is what happened to Botman
    # on 2026-08-12: the link dropped mid-goal and the robot kept driving and
    # rotating until it was powered down by hand.
    #
    # Freshness is "either direction heard from": `tx_state` sends state at
    # `state_hz`, and any received command also counts.
    #
    # A completed `await ws.send()` is weaker evidence than it looks — it means
    # the frame reached the kernel's write buffer, not the operator. State
    # frames are ~400 B at 5 Hz, so filling a ~200 KB socket buffer after a hard
    # cut takes on the order of a hundred seconds; backpressure is NOT the thing
    # that catches this. What catches it is the websocket ping below closing the
    # connection, after which sends raise and this stops advancing.
    "link_timeout_s": 1.5,
    # Websocket keepalive. The library defaults (20 s interval, 20 s timeout)
    # leave up to ~40 s between the network dying and the socket closing, and
    # for all of it `link_ok()` reads true off our own completing sends — so an
    # active goal keeps running with nobody watching. Detection now costs at
    # most interval + timeout = 6 s, after which the `except` in `run_robot`
    # cancels the goal and zeroes cmd_vel.
    #
    # Not tighter: a robot on a degraded-but-usable link (Botman measured 60%
    # packet loss at 333 ms RTT) would drop the socket constantly, and each
    # reconnect cancels the active goal. 4 s matches the backend's own
    # OFFLINE_AFTER_S, so both ends give up at about the same point.
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

    # Deliberately stale: a link that has never been heard from is not a link
    # that may drive the robot. Subscriptions exist before `__init__` finishes,
    # so `_on_nav_cmd_vel` can fire against a half-built bridge — and the safe
    # answer to "is the operator there?" before anyone has connected is no.
    _last_link_at: float = 0.0

    def __init__(self, node: Node, robot_id: str, cfg: dict, http_url: str) -> None:
        self.node = node
        self.id = robot_id
        self.cfg = cfg
        self.http_url = http_url
        self.t0 = time.monotonic()
        rates = cfg.get("rates") or {}
        self._keyframes = KeyframeUploader(
            robot_id,
            http_url,
            min_period_s=float(rates.get("keyframe_period_s", 2.0)),
        )
        self._nav_map = NavMapClient(http_url, robot_id)

        self.map_frame = cfg["map_frame"]
        self.base_frame = cfg["base_frame"]
        topics = cfg["topics"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

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
        self._global_planned_path: list[dict[str, float]] = []
        self._local_planned_path: list[dict[str, float]] = []
        self._plan_frame_warned = False
        self.battery: float | None = None
        self.nav_status = "idle"
        self.mode = "idle"
        self.goal: dict[str, float] | None = None
        self._goal_handle = None
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
                node.get_logger().warn(
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
        self._last_depth_warning_at = 0.0
        self._pose_warned = False

        # A map is latched: published once and expected to be available to any
        # later subscriber. Matching that QoS is not optional — a VOLATILE
        # subscriber on a TRANSIENT_LOCAL publisher receives nothing and the
        # robot silently contributes no map.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        if topics.get("odom"):
            # Sensor-data QoS (BEST_EFFORT). LIO-SAM and several lidar odometry
            # publishers use it; a RELIABLE subscriber never sees those samples.
            # A BEST_EFFORT subscriber still matches a RELIABLE publisher, so
            # this does not break SuperOdometry on the Bunkers.
            node.create_subscription(
                Odometry, topics["odom"], self._on_odom, qos_profile_sensor_data
            )
        if topics.get("map"):
            node.create_subscription(OccupancyGrid, topics["map"], self._on_map, latched)
        if topics.get("map_cloud"):
            node.create_subscription(
                PointCloud2, topics["map_cloud"], self._on_map_cloud,
                qos_profile_sensor_data,
            )
        if topics.get("plan"):
            node.create_subscription(NavPath, topics["plan"], self._on_plan, 10)
        if topics.get("local_plan"):
            node.create_subscription(
                NavPath, topics["local_plan"], self._on_local_plan, 10
            )
        if topics.get("battery"):
            node.create_subscription(
                BatteryState, topics["battery"], self._on_battery, qos_profile_sensor_data
            )
        # Prefer compressed: a raw camera stream at full rate is the single most
        # expensive thing an adapter can subscribe to over a robot's network.
        # Frames stay on-robot for detection; the operator picture is WebRTC.
        if topics.get("camera_compressed"):
            node.create_subscription(
                CompressedImage, topics["camera_compressed"],
                self._on_camera_compressed, qos_profile_sensor_data,
            )
        elif topics.get("camera"):
            node.create_subscription(
                Image, topics["camera"], self._on_camera_raw, qos_profile_sensor_data
            )
        if topics.get("camera_depth_points"):
            node.create_subscription(
                PointCloud2,
                topics["camera_depth_points"],
                self._on_camera_depth_cloud,
                qos_profile_sensor_data,
            )
        if topics.get("camera_depth"):
            node.create_subscription(
                Image,
                topics["camera_depth"],
                self._on_camera_depth,
                qos_profile_sensor_data,
            )
        if topics.get("camera_info"):
            node.create_subscription(
                CameraInfo,
                topics["camera_info"],
                self._on_camera_info,
                qos_profile_sensor_data,
            )
        if topics.get("camera_color_info"):
            node.create_subscription(
                CameraInfo,
                topics["camera_color_info"],
                self._on_camera_color_info,
                qos_profile_sensor_data,
            )
        if topics.get("nav_cmd_vel"):
            node.create_subscription(
                Twist, topics["nav_cmd_vel"], self._on_nav_cmd_vel, 10
            )

        self.pub_cmd = (
            node.create_publisher(Twist, topics["cmd_vel"], 10)
            if topics.get("cmd_vel") else None
        )
        nav_map_topic = topics.get("nav_map") or "/global_map"
        self.pub_global_map = node.create_publisher(
            OccupancyGrid, nav_map_topic, latched
        )
        action_name = cfg.get("actions", {}).get("navigate_to_pose")
        self.nav_client = None
        if action_name and NavigateToPose is not None:
            self.nav_client = ActionClient(node, NavigateToPose, action_name)

        traj_name = cfg.get("actions", {}).get("trajectory")
        self.traj_client = None
        if traj_name and Trajectory is not None:
            self.traj_client = ActionClient(node, Trajectory, traj_name)

        self._body_clients: dict[str, Any] = {}
        if Trigger is not None:
            for name, topic in (cfg.get("services") or {}).items():
                if name in BODY_SERVICE_NAMES and topic:
                    self._body_clients[name] = node.create_client(Trigger, topic)

        max_velocity_name = (cfg.get("services") or {}).get("max_velocity")
        self._velocity_client = None
        if max_velocity_name and SetVelocity is not None:
            self._velocity_client = node.create_client(
                SetVelocity, max_velocity_name
            )

        # The deadman runs off the ROBOT's clock, not the operator link.
        #
        # It used to be called from the websocket tx loop, which is the one place
        # it cannot be trusted: a wedged TCP connection — the normal shape of a
        # Wi-Fi link at the edge of coverage, where it stops delivering without
        # ever resetting — blocks `await ws.send()` on the write buffer, and the
        # watchdog stopped with it. websockets only gives up after
        # ping_interval + ping_timeout, so the robot would keep executing its
        # last commanded velocity for up to ~40 s with nobody watching.
        #
        # `link_watchdog` is that same deadman applied to autonomy, and it shares
        # this timer because it depends on the same property: it runs off the
        # robot's clock, not off the link it is watching.
        self._last_link_at = time.monotonic()
        # Newest operator drive intent, applied by the timer rather than inline.
        # See `note_drive_command`.
        self._pending_drive: tuple[float, float] | None = None
        # 20 Hz: fast enough that latching adds at most 50 ms to teleop response
        # (the GUI only repeats every 120 ms), and finer than `drive_timeout_s`.
        self._watchdog_timer = node.create_timer(0.05, self._watchdogs)

    # ------------------------------------------------------------- capabilities

    def _network_quality(self, iface: str):
        # Keep the legacy module-level seam available to offline callers/tests.
        host = getattr(self, "_server_host", None)
        port = getattr(self, "_server_port", None)
        if not host and getattr(self, "http_url", None):
            try:
                parsed = urllib.parse.urlparse(self.http_url)
                host = parsed.hostname
                port = parsed.port
            except Exception:
                pass
        try:
            return read_link_quality(iface, host=host, port=port)
        except TypeError:
            return read_link_quality(iface)

    def capabilities(self) -> list[str]:
        """Only what this robot can actually honour (protocol rule 4)."""
        caps: list[str] = []
        if self.nav_client is not None or self.traj_client is not None:
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
        services = self.cfg.get("services") or {}
        if any(services.get(name) for name in BODY_ACTIONS):
            caps.append("body")
        return caps

    # ------------------------------------------------------------- ROS inputs

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Reduce one registered scan for the 2D map and optional 3D view."""
        points = self._cloud_xyz(msg)
        if not len(points):
            return

        now = time.monotonic()
        cloud_period = max(
            0.1, float(self.cfg.get("rates", {}).get("cloud_period_s", 4.0))
        )
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
        try:
            pose = self.pose7()
            if pose is not None:
                stamp = self._stamp_seconds(getattr(msg, "header", None)) or time.time()
                self._keyframes.consider(points, pose, stamp)
        except Exception:
            # Keyframe production is best-effort. A missing TF, a test double
            # without a header, or a too-small cloud must not starve the scan
            # map that the operator is looking at.
            pass

    def _on_plan(self, msg: NavPath) -> None:
        self._receive_plan(msg, local=False)

    def _on_local_plan(self, msg: NavPath) -> None:
        """Keep DWB's currently selected local trajectory for the UI."""
        self._receive_plan(msg, local=True)

    def _receive_plan(self, msg: NavPath, *, local: bool) -> None:
        """Publish the planner's intended route, in ``map_frame``.

        Nav2's global and DWB local plans normally already use ``map_frame``.
        Keeping the transform path here as well makes this safe for a controller
        or vendor planner that publishes its route in another connected frame,
        and avoids drawing a perfectly plausible route at the wrong place.
        """
        if not msg.poses:
            if local:
                self._local_planned_path = []
            else:
                self._global_planned_path = []
            self.planned_path = (
                self._local_planned_path
                if len(self._local_planned_path) > 1
                else self._global_planned_path
            )
            return

        frame = str(getattr(msg.header, "frame_id", "") or "").lstrip("/")
        if not frame or frame == self.map_frame:
            points = np.array(
                [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                 for ps in msg.poses],
                dtype=np.float64,
            )
        else:
            try:
                stamp = getattr(msg.header, "stamp", None)
                tf_time = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, tf_time)
            except Exception as exc:
                if not self._plan_frame_warned:
                    self._plan_frame_warned = True
                    self.node.get_logger().warn(
                        f"[{self.id}] no {self.map_frame} -> {frame} transform for "
                        f"the {'local' if local else 'global'} planned path; "
                        f"not publishing it: {exc}"
                    )
                if local:
                    self._local_planned_path = []
                else:
                    self._global_planned_path = []
                self.planned_path = (
                    self._local_planned_path
                    if len(self._local_planned_path) > 1
                    else self._global_planned_path
                )
                return
            points = np.array(
                [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                 for ps in msg.poses],
                dtype=np.float64,
            )
            points = transform_points(points, tf.transform)
            if points is None:
                if local:
                    self._local_planned_path = []
                else:
                    self._global_planned_path = []
                self.planned_path = (
                    self._local_planned_path
                    if len(self._local_planned_path) > 1
                    else self._global_planned_path
                )
                return

        # Planner paths can contain thousands of poses. Keep enough detail for
        # a smooth line while preventing one robot_state from monopolising the
        # websocket payload.
        max_points = 120
        if len(points) > max_points:
            indices = np.linspace(0, len(points) - 1, max_points, dtype=int)
            points = points[indices]
        path = [
            {"x": round(float(x), 3), "y": round(float(y), 3)}
            for x, y in points[:, :2]
        ]
        if local:
            self._local_planned_path = path
        else:
            self._global_planned_path = path
        self.planned_path = (
            self._local_planned_path
            if len(self._local_planned_path) > 1
            else self._global_planned_path
        )

    def _on_camera_compressed(self, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower():
            return
        jpeg = bytes(msg.data)
        # Queue for detection; do NOT run inference here. See run_detection().
        self._detect_pending = (jpeg, getattr(msg, "header", None))

    def _on_camera_raw(self, msg: Image) -> None:
        # Detection takes JPEG internally. Imported lazily so a robot with no
        # camera needs no OpenCV. Hardware video is produced separately by the
        # H.264 RTSP media publisher; this conversion never goes to the backend.
        try:
            import cv2
        except ImportError:
            return
        frame = self._image_to_bgr(msg)
        if frame is None:
            if not self._camera_encoding_warned:
                self._camera_encoding_warned = True
                self.node.get_logger().warn(
                    f"[{self.id}] cannot decode camera encoding "
                    f"{getattr(msg, 'encoding', '?')!r}; detection has no frames"
                )
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        jpeg = buf.tobytes()
        self._detect_pending = (jpeg, getattr(msg, "header", None))

    def run_detection(self) -> None:
        """Detect on the newest queued frame. Runs OFF the ROS executor thread.

        Inference is a blocking HTTP round trip to the sidecar (up to
        `timeout_s`), and `_depth_map_position` adds a tf2 lookup per detection.
        Called from a subscription callback, as it used to be, all of that ran
        inside the single-threaded executor and stalled every other callback on
        the node — odometry, the map, and the very TF the pose report depends
        on — several times a second. `adapter_sim` has always run detection off
        the ROS thread; this is the same arrangement.
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
            stamp_msg = getattr(depth_header, "stamp", None)
            stamp = (
                rclpy.time.Time.from_msg(stamp_msg)
                if stamp_msg is not None
                else rclpy.time.Time()
            )
            tf = self.tf_buffer.lookup_transform(color_frame, depth_frame, stamp)
            return {
                "color_camera_info": color_info,
                "depth_to_color": tf.transform,
            }
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_depth_warning_at >= 10.0:
                self._last_depth_warning_at = now
                self.node.get_logger().warn(
                    f"[{self.id}] cannot join colour detection to depth: {exc}"
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
                stamp_msg = getattr(source_header, "stamp", None)
                stamp = (
                    rclpy.time.Time.from_msg(stamp_msg)
                    if stamp_msg is not None
                    else rclpy.time.Time()
                )
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame_id, stamp)
                map_point = transform_point(camera_point, tf.transform)
            if map_point is None:
                return None
            return {"x": round(float(map_point[0]), 3), "y": round(float(map_point[1]), 3)}
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_depth_warning_at >= 10.0:
                self._last_depth_warning_at = now
                self.node.get_logger().warn(
                    f"[{self.id}] cannot place camera detection in {self.map_frame}: {exc}"
                )
            return None

    # ------------------------------------------------------------- pose

    def map_pose(self) -> dict[str, float]:
        """The robot's pose in its navigation-map frame, via tf2.

        A tf2 lookup rather than composing transforms we recognise by name.
        `adapter_sim` can compose `map_frame -> odom -> base_link` because we
        built that tree; a real robot's tree has links we do not know about, and
        hardcoding a chain through them is how an adapter ends up reporting a
        pose that is subtly wrong (this project has already made that mistake
        once, with a 0.47 m error nobody noticed).

        Falls back to raw odometry only if TF is unavailable, and says so once —
        reporting the map origin forever would look like a stationary robot.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
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
                if getattr(self, "_odom_frame", "") == self.map_frame:
                    detail = (
                        f"using direct {self.map_frame}-frame odometry instead"
                    )
                else:
                    detail = (
                        "falling back to raw odometry, which DRIFTS. Check that SLAM "
                        "or localisation is running and publishing TF"
                    )
                self.node.get_logger().warn(
                    f"[{self.id}] no {self.map_frame} -> {self.base_frame} transform; "
                    f"{detail}."
                )
            return dict(fallback)

    def pose7(self) -> np.ndarray | None:
        """Full ``T_map_base`` as ``[x,y,z,qx,qy,qz,qw]`` for keyframe upload."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            return np.array(
                [t.x, t.y, t.z, q.x, q.y, q.z, q.w], dtype=np.float64
            )
        except Exception:
            stored = getattr(self, "_odom_pose7", None)
            if stored is not None:
                return np.asarray(stored, dtype=np.float64)
            fallback = getattr(self, "_odom_pose", None)
            if fallback is None:
                return None
            return pose7_from_xy_yaw(fallback["x"], fallback["y"], fallback["yaw"])

    # ------------------------------------------------------------- commands

    def drive(self, linear: float, angular: float) -> None:
        if self.pub_cmd is None:
            return
        moving = abs(linear) > 1e-3 or abs(angular) > 1e-3
        # Operator motion always preempts autonomy. Changing nav_status stops
        # the isolated velocity relay immediately; cancel_goal() also asks the
        # action server to terminate its work.
        if moving and self.nav_status == "active":
            self.cancel_goal()
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub_cmd.publish(twist)
        self.mode = "teleop" if moving else self.mode
        self._last_drive_at = time.monotonic() if moving else 0.0

    def body_command(self, action: str) -> None:
        """Claim/release the body lease, or sit/stand.

        `stand` powers the motors first when `services.power_on` is set —
        Clearpath's `/stand` fails if the robot is still sitting unpowered.
        `claim` also releases the software e-stop and drops a leftover tablet
        keepalive; without that, `/power_on` returns KeepaliveMotorsOffError
        and the GUI button looks like a no-op. Each call is a Trigger;
        failures are logged and not retried here.
        """
        action = str(action or "")
        if action not in BODY_ACTIONS:
            return
        if action == "claim":
            self._call_trigger("claim")
            self._call_trigger("estop_release")
            self._call_trigger("clear_keepalive")
            return
        if action == "stand":
            self._call_trigger("estop_release")
            self._call_trigger("clear_keepalive")
            self._call_trigger("power_on")
            self._call_trigger("stand")
            return
        self._call_trigger(action)

    def _call_trigger(self, name: str) -> bool:
        client = self._body_clients.get(name)
        if client is None or Trigger is None:
            if name not in OPTIONAL_BODY_SERVICES:
                self.node.get_logger().warn(
                    f"[{self.id}] body command {name!r} has no service configured"
                )
            return False
        if not client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn(
                f"[{self.id}] body service {name!r} is not up "
                "(is spot_driver running?)"
            )
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 20.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.node.get_logger().warn(
                f"[{self.id}] body service {name!r} timed out"
            )
            return False
        try:
            resp = future.result()
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] body service {name!r} failed: {exc}"
            )
            return False
        ok = bool(getattr(resp, "success", True))
        message = str(getattr(resp, "message", "") or "")
        if ok:
            self.node.get_logger().info(
                f"[{self.id}] body {name}: ok" + (f" ({message})" if message else "")
            )
        else:
            self.node.get_logger().warn(
                f"[{self.id}] body {name}: refused" + (f" ({message})" if message else "")
            )
        return ok

    def navigate_to(self, goal: dict[str, float]) -> None:
        if self.traj_client is not None:
            self._navigate_trajectory(goal)
            return
        if self.nav_client is None:
            return
        # Goal submission runs in the adapter's executor worker, not the
        # websocket receive coroutine, so a brief startup wait cannot stop the
        # 5 Hz state heartbeat. This matters when Nav2 is still activating: a
        # click during that window should not be silently turned into a failed
        # goal just because the action server was a moment late.
        if (
            not self.nav_client.server_is_ready()
            and not self.nav_client.wait_for_server(timeout_sec=3.0)
        ):
            self.node.get_logger().warn(
                f"[{self.id}] navigation action server not available; goal dropped"
            )
            self.nav_status = "failed"
            return
        self._goal_generation += 1
        generation = self._goal_generation

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.map_frame
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal["x"])
        msg.pose.pose.position.y = float(goal["y"])
        # NavigateToPose requires an orientation even though SwarmDeck's GUI
        # command is a point. The Bunker/Nav2 point-goal checker intentionally
        # accepts any final heading, so this neutral quaternion is not a
        # request to rotate the robot to yaw zero.
        yaw = float(goal.get("yaw", 0.0))
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._arm_goal(goal)
        future = self.nav_client.send_goal_async(msg)
        future.add_done_callback(
            lambda f, g=generation: self._on_goal_response(f, g)
        )

    def _navigate_trajectory(self, goal: dict[str, float]) -> None:
        """Spot click-to-pose: map-frame goal -> body-frame Trajectory.

        Same path as `spot_high_level_controller`: TF `map` into `body`, then
        Clearpath `/trajectory`. The driver rejects any other frame_id.
        """
        if Trajectory is None or (
            not self.traj_client.server_is_ready()
            and not self.traj_client.wait_for_server(timeout_sec=3.0)
        ):
            self.node.get_logger().warn(
                f"[{self.id}] Spot trajectory action not available; goal dropped"
            )
            self.nav_status = "failed"
            return
        pose = self._goal_in_body(goal)
        if pose is None:
            self.nav_status = "failed"
            return
        if not self._apply_trajectory_velocity_limit():
            # A configured limit is a safety constraint, not a preference. Do
            # not let Spot execute at its unrestricted default merely because
            # the driver restarted or its service is temporarily unavailable.
            self.nav_status = "failed"
            self.mode = "idle"
            return
        self._goal_generation += 1
        generation = self._goal_generation
        tcfg = self.cfg.get("trajectory") or {}
        dur_s = max(1.0, float(tcfg.get("duration_s", 30.0)))
        msg = Trajectory.Goal()
        msg.target_pose = pose
        msg.duration.sec = int(dur_s)
        msg.duration.nanosec = int((dur_s - int(dur_s)) * 1e9)
        msg.precise_positioning = bool(tcfg.get("precise_positioning", True))
        msg.disable_obstacle_avoidance = bool(
            tcfg.get("disable_obstacle_avoidance", False)
        )
        self._arm_goal(goal)
        future = self.traj_client.send_goal_async(msg)
        future.add_done_callback(
            lambda f, g=generation: self._on_goal_response(f, g)
        )

    def _apply_trajectory_velocity_limit(self) -> bool:
        """Apply Spot's configured mobility limit before accepting a goal.

        The limit lives in spot_driver rather than in the Trajectory action.
        Applying it for every goal makes the setting survive a driver restart
        that did not also restart this adapter.
        """
        limit = (self.cfg.get("trajectory") or {}).get("velocity_limit") or {}
        if not limit:
            return True
        client = self._velocity_client
        if client is None or SetVelocity is None:
            self.node.get_logger().warn(
                f"[{self.id}] trajectory velocity limit configured but "
                "spot_msgs/SetVelocity is unavailable; goal dropped"
            )
            return False
        if not client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn(
                f"[{self.id}] Spot max_velocity service is not up; goal dropped"
            )
            return False
        try:
            linear_x = float(limit["linear_x"])
            linear_y = float(limit.get("linear_y", linear_x))
            angular_z = float(limit["angular_z"])
        except (KeyError, TypeError, ValueError):
            self.node.get_logger().warn(
                f"[{self.id}] invalid trajectory velocity_limit; goal dropped"
            )
            return False
        if (
            not all(math.isfinite(v) for v in (linear_x, linear_y, angular_z))
            or min(linear_x, linear_y, angular_z) <= 0.0
        ):
            self.node.get_logger().warn(
                f"[{self.id}] trajectory velocity limits must be positive; "
                "goal dropped"
            )
            return False

        request = SetVelocity.Request()
        request.velocity_limit.linear.x = linear_x
        request.velocity_limit.linear.y = linear_y
        request.velocity_limit.angular.z = angular_z
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.node.get_logger().warn(
                f"[{self.id}] Spot max_velocity service timed out; goal dropped"
            )
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] Spot max_velocity service failed: {exc}"
            )
            return False
        if not bool(getattr(response, "success", False)):
            message = str(getattr(response, "message", "") or "")
            self.node.get_logger().warn(
                f"[{self.id}] Spot rejected the trajectory velocity limit"
                + (f": {message}" if message else "")
            )
            return False
        return True

    def _arm_goal(self, goal: dict[str, float]) -> None:
        self.goal = {"x": float(goal["x"]), "y": float(goal["y"])}
        self.nav_status = "active"
        self.mode = "nav"

    def _goal_in_body(self, goal: dict[str, float]) -> PoseStamped | None:
        frame = str((self.cfg.get("trajectory") or {}).get("frame") or "body")
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.map_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] {self.map_frame} -> {frame} TF failed; goal dropped: {exc}"
            )
            return None
        xyz = transform_point(
            (float(goal["x"]), float(goal["y"]), 0.0), tf.transform
        )
        if xyz is None:
            return None
        if "yaw" not in goal or goal["yaw"] is None:
            # A map click is a point, not a pose. Identity in the current body
            # frame tells Spot to keep the heading it had when the goal was
            # issued. Treating an absent yaw as map yaw zero made it reach the
            # point and then perform a surprising final turn.
            body_yaw = 0.0
        else:
            yaw = float(goal["yaw"])
            heading = transform_point(
                (math.cos(yaw), math.sin(yaw), 0.0),
                type("T", (), {
                    "rotation": tf.transform.rotation,
                    "translation": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                })(),
            )
            if heading is None:
                return None
            body_yaw = math.atan2(float(heading[1]), float(heading[0]))
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(body_yaw / 2.0)
        pose.pose.orientation.w = math.cos(body_yaw / 2.0)
        return pose

    def _on_goal_response(self, future, generation: int) -> None:
        try:
            handle = future.result()
        except Exception:
            if generation != self._goal_generation:
                return
            self.nav_status = "failed"
            return
        # A cancellation or newer goal can win while send_goal_async is still
        # in flight. Cancel an accepted stale handle instead of abandoning an
        # action that would continue computing and publishing velocity forever.
        if generation != self._goal_generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.nav_status = "failed"
            self.mode = "idle"
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, g=generation: self._on_goal_result(f, g)
        )

    def _on_goal_result(self, future, generation: int) -> None:
        if generation != self._goal_generation:
            return
        from action_msgs.msg import GoalStatus

        try:
            status = future.result().status
        except Exception:
            self.nav_status = "failed"
            self.mode = "idle"
            return
        self.nav_status = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "cancelled",
        }.get(status, "failed")
        self.mode = "idle"
        self.goal = None
        self.planned_path = []
        self._local_planned_path = []
        self._global_planned_path = []
        self._goal_handle = None

    def cancel_goal(self) -> None:
        self._goal_generation += 1
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        # Clearpath's ROS 2 Trajectory server never checks cancel/preempt
        # (the ROS 1 path that called spot_wrapper.stop() is commented out).
        # The SDK `/stop` Trigger is what actually halts the body.
        if self.traj_client is not None:
            self._call_trigger("stop")
        self.goal = None
        self.planned_path = []
        self._local_planned_path = []
        self._global_planned_path = []
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
            self.node.get_logger().warn(f"[{self.id}] map upload failed: {exc}")
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
        """Upload registered XY returns for server-side occupancy raytracing."""
        if not self._scan_dirty or self._scan_points is None:
            return
        self._scan_dirty = False
        if not len(self._scan_points):
            return
        # The pose these points were captured at, not the pose now. See _on_map_cloud.
        origin = self._scan_origin or self.map_pose()
        points = self._scan_points
        # A deck-mounted 3D lidar can see the chassis beneath and behind it.
        # Those returns are already in the map frame, so remove the robot's
        # configured footprint around the same origin used for raytracing.
        # Otherwise ScanGridAccumulator quite correctly treats each self-hit
        # as permanently occupied and the moving robot paints a trail of itself.
        footprint_radius = max(0.0, float(self.cfg.get("footprint_radius", 0.0)))
        if footprint_radius:
            offsets = points - np.array(
                [origin["x"], origin["y"]], dtype=np.float32
            )
            outside_footprint = np.einsum("ij,ij->i", offsets, offsets) > (
                footprint_radius * footprint_radius
            )
            points = points[outside_footprint]
        if not len(points):
            return
        quantised = np.round(points / MAP_CLOUD_SCALE).astype(np.int16)
        url = (
            f"{self.http_url}/api/adapter/scan?robot_id={self.id}"
            f"&origin_x={origin['x']}&origin_y={origin['y']}"
            f"&scale={MAP_CLOUD_SCALE}"
            f"&retain_free_space={1 if self.cfg.get('retain_free_space', False) else 0}"
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
            self.node.get_logger().warn(f"[{self.id}] scan upload failed: {exc}")

    def upload_cloud(self) -> None:
        """Upload a voxel-reduced XYZ scan for the optional 3D map view."""
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
            self.node.get_logger().warn(f"[{self.id}] 3D cloud upload failed: {exc}")

    def upload_keyframe(self) -> None:
        """Best-effort keyframe POST. Drops rather than blocking telemetry."""
        if not self._keyframes.upload_one() and self._keyframes.last_error:
            self.node.get_logger().warn(
                f"[{self.id}] keyframe upload failed: {self._keyframes.last_error}"
            )

    def pull_nav_map(self) -> None:
        """Publish the merged occupancy in this robot's map frame for Nav2."""
        client = getattr(self, "_nav_map", None)
        pub = getattr(self, "pub_global_map", None)
        if client is None or pub is None:
            return
        downloaded = client.poll()
        if downloaded is None:
            if client.last_error:
                self.node.get_logger().warn(
                    f"[{self.id}] nav map download failed: {client.last_error}"
                )
            return
        grid = OccupancyGrid()
        pose = self.map_pose()
        apply_to_occupancy_grid(
            grid,
            downloaded,
            self.map_frame,
            pose_xy=(pose["x"], pose["y"]),
            clear_radius_m=float(self.cfg.get("footprint_radius", 0.35)) + 0.15,
        )
        grid.header.stamp = self.node.get_clock().now().to_msg()
        pub.publish(grid)


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
                    "adapter": "adapter_ros2/0.1.0",
                    "ros": bridge.cfg["ros_distro"],
                    # `local`: a real robot's pose and grid are in its own
                    # navigation-map frame. The backend does the merging.
                    "coordinate_frame": "local",
                    "capabilities": bridge.capabilities(),
                    "footprint_radius": bridge.cfg["footprint_radius"],
                    "footprint": bridge.cfg.get("footprint") or None,
                }))
                bridge.node.get_logger().info(
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
                        elif kind == "body_command":
                            await asyncio.get_running_loop().run_in_executor(
                                None, bridge.body_command, msg.get("action", "")
                            )
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
                    """The telemetry heartbeat. Nothing slow may run in here.

                    This used to share one loop with the uploads below, which
                    tied liveness to backend latency: `upload_map` blocks for up
                    to its 5 s timeout, and no state frame went out while it did.
                    The server calls a robot offline after 4 s (OFFLINE_AFTER_S)
                    and `link_watchdog` cancels an active goal after
                    `link_timeout_s` (1.5 s), so a single slow upload took the
                    robot off the dashboard AND stopped its mission.

                    That made the failure a property of FLEET SIZE rather than of
                    this robot: every map upload queues behind one lock on the
                    server, so a fourth robot connecting is what makes the first
                    one drop out. Uploading is best-effort and may lag; being
                    reachable is not. Keeping them in separate coroutines is what
                    makes a slow backend cost only map freshness.
                    """
                    period = 1.0 / float(bridge.cfg["rates"]["state_hz"])
                    while True:
                        await send(bridge.state())
                        # A completed send means the frame reached the socket,
                        # which is the freshness signal the autonomy deadman
                        # reads. On a wedged link this is exactly where it stops
                        # completing, which is the point.
                        bridge.note_link_activity()
                        # No drive_watchdog() here: it runs on a ROS timer so it
                        # keeps ticking when this loop cannot. See __init__.
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
                        upload_kf = getattr(bridge, "upload_keyframe", None)
                        if callable(upload_kf):
                            await loop.run_in_executor(None, upload_kf)
                        pull_nav = getattr(bridge, "pull_nav_map", None)
                        if callable(pull_nav):
                            await loop.run_in_executor(None, pull_nav)
                        if now - last_settings > 5.0:
                            last_settings = now
                            await loop.run_in_executor(None, bridge.refresh_settings)
                        await asyncio.sleep(tick)

                async def tx_camera() -> None:
                    """Camera and detections, on their own clock.

                    Separate from `tx_maps` because the two block for very
                    different reasons: a map upload waits on the server's
                    registration lock for seconds at a time, and sharing a loop
                    with it froze the operator's video for exactly as long.
                    """
                    rates = bridge.cfg["rates"]
                    tick = 1.0 / float(rates["state_hz"])
                    loop = asyncio.get_running_loop()
                    last_detect = 0.0
                    while True:
                        now = time.monotonic()

                        # Detection runs at the configured rate regardless of
                        # which camera is on screen. The H.264 media publisher
                        # owns the operator video path and never uses this
                        # protocol connection for camera frames.
                        if now - last_detect > float(rates["camera_period_s"]):
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
                            last_detect = time.monotonic()

                        await asyncio.sleep(tick)

                # Not `gather`: it leaves the siblings of a failed coroutine
                # running against a socket that is already closing, and their
                # exceptions surface later as "never retrieved". The first one to
                # fail is the reconnect trigger, and the rest must be torn down
                # with it.
                await run_until_first_failure(rx(), tx_state(), tx_maps(), tx_camera())
        except Exception as exc:
            bridge.node.get_logger().warn(
                f"[{bridge.id}] disconnected ({exc}); retrying in {backoff:.0f}s"
            )
            # Stop the robot on link loss. A robot that keeps driving after
            # losing its operator is the failure that hurts someone.
            #
            # cancel_goal() FIRST, and not only drive(0, 0): while nav_status is
            # still "active" the relay in `_on_nav_cmd_vel` overwrites a zero
            # Twist with Nav2's next sample, so zeroing alone stops an
            # autonomous robot for milliseconds and nothing more.
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

    rclpy.init()
    node = rclpy.create_node(f"swarmdeck_adapter_{args.robot_id}")
    bridge = HardwareBridge(node, args.robot_id, cfg, http_url)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    # SIGTERM, not just Ctrl-C. `docker stop`, a Compose recreate and a systemd
    # restart all send SIGTERM, whose DEFAULT action kills the interpreter
    # outright: the `finally` below never runs, nothing zeroes the driver, and
    # the robot keeps executing its last velocity with every deadman in this
    # process now gone. Stop first, then unwind through the normal path.
    def _on_signal(_signum: int, _frame: Any) -> None:
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
        # Shut the context down first so `rclpy.spin` returns, then join it.
        # Tearing the node down under a still-spinning executor aborts in C++
        # ("terminate called without an active exception", exit 133), which
        # turns every ordinary restart into a crash and makes it impossible to
        # tell from the outside whether the stop above was flushed.
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin.join(timeout=2.0)


if __name__ == "__main__":
    main()
