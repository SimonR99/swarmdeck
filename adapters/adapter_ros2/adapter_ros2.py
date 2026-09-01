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

HARDWARE VALIDATION
-------------------
The generic adapter is unit-tested, and the Asimov profile has been smoke-tested
against a live Unitree G1: its native odometry, camera, point cloud, TF, and
command-velocity bridge are all discovered at runtime. Other hardware profiles
still require the same topic, QoS, and frame validation described in
docs/operations/hardware-bringup.md.
"""

from __future__ import annotations

import argparse
import asyncio
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
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    CompressedImage,
    Image,
    PointCloud2,
)
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
    AdapterHelloMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
    deep_merge,
    load_yaml_profile,
    map_cloud_height_limits,
    stamp_seconds,
    yaw_of,
    unique_row_index,
)
from adapters.session import run_adapter_session
from adapters.keyframe_producer import KeyframeUploader, pose7_from_xy_yaw
from adapters.costmap import CostmapSnapshot, normalize_costmap
from adapters.map_downlink import NavMapClient, apply_to_occupancy_grid
from ros2_defaults import DEFAULTS

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
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - depends on the robot's install
    String = None

try:
    from spot_msgs.action import Trajectory
except ImportError:  # pragma: no cover - depends on the robot's install
    Trajectory = None

try:
    from spot_msgs.srv import SetStandHeight, SetVelocity
except ImportError:  # pragma: no cover - depends on the robot's install
    SetStandHeight = None
    SetVelocity = None

# Spot and Unitree G1 humanoid body services.
BODY_ACTIONS = (
    "claim",
    "release",
    "sit",
    "stand",
    "damping",
    "lie_to_stand",
    "lock_stand",
    "walk_mode",
    "run_mode",
    "wave",
    "set_height",
)
# Trigger names that are not GUI body actions: motors, software e-stop allow,
# tablet keepalive clear, and the SDK stop used because Clearpath's ROS 2
# Trajectory server does not honour cancel/preempt.
BODY_SERVICE_NAMES = (
    *BODY_ACTIONS,
    "power_on",
    "stop",
    "estop_release",
    "clear_keepalive",
)
# Missing these is normal on non-Spot robots; do not warn.
OPTIONAL_BODY_SERVICES = frozenset({"power_on", "estop_release", "clear_keepalive"})


class HardwareBridge(
    AdapterHelloMixin,
    AdapterDetectionMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
):
    """One real robot's ROS interface, expressed as the SwarmDeck protocol."""

    adapter_name = "adapter_ros2/0.1.0"
    coordinate_frame = "local"

    _TRACK_IDS = staticmethod(track_ids) if track_ids is not None else None

    # Deliberately stale: a link that has never been heard from is not a link
    # that may drive the robot. Subscriptions exist before `__init__` finishes,
    # so `_on_nav_cmd_vel` can fire against a half-built bridge — and the safe
    # answer to "is the operator there?" before anyone has connected is no.
    _last_link_at: float = 0.0
    _target_body_height: float = 0.0

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
            timeout_s=float(cfg.get("upload_timeout_s", 0.5)),
            height_band=cfg.get("map_cloud_height_band"),
            lidar_height_m=cfg.get("lidar_height_m"),
        )
        self._nav_map = NavMapClient(http_url, robot_id)

        self.map_frame = cfg["map_frame"]
        self.base_frame = cfg["base_frame"]
        topics = cfg["topics"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False
        self._costmaps: dict[str, CostmapSnapshot] = {}
        self._costmap_dirty: set[str] = set()
        self._costmap_lock = threading.Lock()
        self._costmap_warned_at: dict[str, float] = {}
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
        # When Spot's lateral velocity is zero, an arbitrary (x, y) body goal
        # is not executable as one holonomic trajectory. Keep the map target
        # across a short rotate/drive/rotate sequence instead.
        self._trajectory_target: dict[str, float] | None = None
        self._trajectory_step = ""
        self._trajectory_step_count = 0
        self._trajectory_step_error: float | None = None
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
        if self._detection_enabled and (
            topics.get("camera") or topics.get("camera_compressed")
        ):
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
            node.create_subscription(
                OccupancyGrid, topics["map"], self._on_map, latched
            )
        if topics.get("map_cloud"):
            node.create_subscription(
                PointCloud2,
                topics["map_cloud"],
                self._on_map_cloud,
                qos_profile_sensor_data,
            )
        if topics.get("plan"):
            node.create_subscription(NavPath, topics["plan"], self._on_plan, 10)
        if topics.get("local_plan"):
            node.create_subscription(
                NavPath, topics["local_plan"], self._on_local_plan, 10
            )
        if topics.get("global_costmap"):
            node.create_subscription(
                OccupancyGrid,
                topics["global_costmap"],
                lambda msg: self._on_costmap(msg, "global"),
                qos_profile_sensor_data,
            )
        if topics.get("local_costmap"):
            node.create_subscription(
                OccupancyGrid,
                topics["local_costmap"],
                lambda msg: self._on_costmap(msg, "local"),
                qos_profile_sensor_data,
            )
        if topics.get("battery"):
            battery_topic = topics["battery"]
            msg_cls = BatteryState
            if "bunker_status" in battery_topic:
                try:
                    from bunker_msgs.msg import BunkerStatus

                    msg_cls = BunkerStatus
                except ImportError:
                    try:
                        from rosidl_runtime_py.utilities import get_message

                        msg_cls = (
                            get_message("bunker_msgs/msg/BunkerStatus") or BatteryState
                        )
                    except Exception:
                        pass
            elif "battery_states" in battery_topic:
                # Spot exposes its two packs as spot_msgs/BatteryStateArray,
                # not sensor_msgs/BatteryState. The callback reduces the pack
                # levels to the conservative whole-robot value.
                try:
                    from spot_msgs.msg import BatteryStateArray

                    msg_cls = BatteryStateArray
                except ImportError:
                    try:
                        from rosidl_runtime_py.utilities import get_message

                        msg_cls = (
                            get_message("spot_msgs/msg/BatteryStateArray") or BatteryState
                        )
                    except Exception:
                        pass
            node.create_subscription(
                msg_cls, battery_topic, self._on_battery, qos_profile_sensor_data
            )
        # Prefer compressed: a raw camera stream at full rate is the single most
        # expensive thing an adapter can subscribe to over a robot's network.
        # Frames stay on-robot for detection; the operator picture is WebRTC.
        if topics.get("camera_compressed"):
            node.create_subscription(
                CompressedImage,
                topics["camera_compressed"],
                self._on_camera_compressed,
                qos_profile_sensor_data,
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
            if topics.get("cmd_vel")
            else None
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

        body_cmd_topic = (cfg.get("topics") or {}).get("body_cmd")
        self.pub_body_cmd = None
        if body_cmd_topic and String is not None:
            self.pub_body_cmd = node.create_publisher(String, body_cmd_topic, 10)

        body_pose_topic = (cfg.get("topics") or {}).get("body_pose")
        self.pub_body_pose = None
        if body_pose_topic and Pose is not None:
            self.pub_body_pose = node.create_publisher(Pose, body_pose_topic, 10)
        self._target_body_height: float = 0.0

        max_velocity_name = (cfg.get("services") or {}).get("max_velocity")
        self._velocity_client = None
        if max_velocity_name and SetVelocity is not None:
            self._velocity_client = node.create_client(SetVelocity, max_velocity_name)

        stand_height_name = (cfg.get("services") or {}).get("set_stand_height")
        self._stand_height_client = None
        if stand_height_name and SetStandHeight is not None:
            self._stand_height_client = node.create_client(
                SetStandHeight, stand_height_name
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
        if self.cfg["topics"].get("camera") or self.cfg["topics"].get(
            "camera_compressed"
        ):
            caps.append("camera")
        if self.cfg["topics"].get("battery"):
            caps.append("battery")
        if self.cfg.get("network_iface"):
            caps.append("network")
        if self.pub_cmd is not None:
            caps.append("estop")
        services = self.cfg.get("services") or {}
        topics = self.cfg.get("topics") or {}
        if (
            any(services.get(name) for name in BODY_ACTIONS)
            or bool(topics.get("body_cmd"))
            or getattr(self, "pub_body_cmd", None) is not None
        ):
            caps.append("body")
        return caps

    # ------------------------------------------------------------- ROS inputs

    def _warn_costmap(self, kind: str, reason: str) -> None:
        now = time.monotonic()
        if now - self._costmap_warned_at.get(kind, 0.0) < 10.0:
            return
        self._costmap_warned_at[kind] = now
        self.node.get_logger().warn(
            f"[{self.id}] {kind} costmap unavailable for overlay: {reason}"
        )

    def _on_costmap(self, msg: OccupancyGrid, kind: str) -> None:
        """Capture Nav2's planner view without changing navigation inputs."""
        source = str(
            getattr(getattr(msg, "header", None), "frame_id", "") or ""
        ).lstrip("/")
        target = str(self.map_frame or "").lstrip("/")
        transform = (0.0, 0.0, 0.0)
        if source and source != target:
            try:
                stamp = getattr(getattr(msg, "header", None), "stamp", None)
                tf_time = (
                    rclpy.time.Time.from_msg(stamp)
                    if stamp is not None
                    else rclpy.time.Time()
                )
                tf = self.tf_buffer.lookup_transform(
                    target, source, tf_time, timeout=Duration(seconds=0.1)
                )
                t = tf.transform
                transform = (
                    float(t.translation.x),
                    float(t.translation.y),
                    float(yaw_of(t.rotation)),
                )
            except Exception as exc:
                self._warn_costmap(kind, f"no {target} <- {source} transform: {exc}")
                return
        try:
            snapshot = normalize_costmap(
                msg, target_frame=target or source, transform=transform
            )
        except (TypeError, ValueError) as exc:
            self._warn_costmap(kind, str(exc))
            return
        with self._costmap_lock:
            self._costmaps[kind] = snapshot
            self._costmap_dirty.add(kind)

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Normalize one cloud into ``map_frame`` before producing map data.

        Most hardware profiles subscribe to an already-registered cloud whose
        header is the configured map frame.  Asimov's Mid-360 publishes raw
        sweeps in ``livox_frame`` instead.  Treating those XYZ values as world
        coordinates makes the keyframe producer apply the inverse robot pose
        to sensor-frame points; the optimizer then receives contradictory
        clouds and fans repeated observations into several rotated maps.

        Honour the PointCloud2 header and use the transform at the cloud stamp.
        A cloud with no header remains the legacy/test-double case and is
        assumed to already be registered.
        """
        points = self._cloud_xyz(msg)
        if not len(points):
            return

        header = getattr(msg, "header", None)
        source = str(getattr(header, "frame_id", "") or "").lstrip("/")
        target = str(self.map_frame or "").lstrip("/")
        stamp = getattr(header, "stamp", None) if header is not None else None
        tf_time = (
            rclpy.time.Time.from_msg(stamp)
            if stamp is not None
            else rclpy.time.Time()
        )
        if source and target and source != target:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target, source, tf_time, timeout=Duration(seconds=0.1)
                )
                mapped = transform_points(points, tf.transform)
                if mapped is None:
                    raise ValueError("cloud transform produced no points")
                points = mapped
            except Exception as exc:
                now = time.monotonic()
                warned_at = float(getattr(self, "_map_cloud_frame_warned_at", 0.0))
                if now - warned_at >= 10.0:
                    self._map_cloud_frame_warned_at = now
                    self.node.get_logger().warn(
                        f"[{self.id}] dropping map cloud: no {target} <- {source} "
                        f"transform at capture time: {exc}"
                    )
                return

        pose = self.pose7(tf_time)
        if pose is not None:
            pose = np.asarray(pose, dtype=np.float64)
            if pose.shape != (7,) or not np.isfinite(pose).all():
                pose = None

        now = time.monotonic()
        cloud_period = max(
            0.1, float(self.cfg.get("rates", {}).get("cloud_period_s", 4.0))
        )
        if now - self._last_cloud_prepare_at >= cloud_period:
            self._last_cloud_prepare_at = now
            cloud_keys = np.round(points / MAP_CLOUD_3D_VOXEL).astype(np.int32)
            cloud_keep = unique_row_index(cloud_keys)
            self._cloud_points = points[cloud_keep]
            self._cloud_dirty = True

        min_z, max_z = map_cloud_height_limits(self.cfg.get("map_cloud_height_band"))
        xy = points[(points[:, 2] >= min_z) & (points[:, 2] <= max_z)][:, :2]
        if not len(xy):
            self._scan_points = np.zeros((0, 2), dtype=np.float32)
            self._scan_dirty = True
            return
        keys = np.round(xy / MAP_CLOUD_VOXEL).astype(np.int32)
        keep = unique_row_index(keys)
        self._scan_points = xy[keep]
        # Pair the points with the pose they were captured AT. upload_scan used
        # to read the pose at upload time, up to map_period_s later, and the
        # backend raytraces free space from that origin — so at 0.5 m/s with a
        # 2 s period every ray was traced from a point up to a metre from where
        # the beam actually left the sensor, carving free space through geometry
        # it never crossed. That corrupts precisely the free/occupied contrast
        # registration.py relies on to break rotational symmetry.
        if pose is not None:
            qx, qy, qz, qw = pose[3:]
            scan_yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            self._scan_origin = {
                "x": float(pose[0]),
                "y": float(pose[1]),
                "yaw": float(scan_yaw),
            }
        else:
            self._scan_origin = self.map_pose()
        self._scan_dirty = True
        try:
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
                [
                    [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                    for ps in msg.poses
                ],
                dtype=np.float64,
            )
        else:
            try:
                stamp = getattr(msg.header, "stamp", None)
                tf_time = (
                    rclpy.time.Time.from_msg(stamp)
                    if stamp is not None
                    else rclpy.time.Time()
                )
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
                [
                    [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                    for ps in msg.poses
                ],
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
            {"x": round(float(x), 3), "y": round(float(y), 3)} for x, y in points[:, :2]
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
            try:
                tf = self.tf_buffer.lookup_transform(
                    color_frame, depth_frame, stamp, timeout=Duration(seconds=0.1)
                )
            except Exception:
                tf = self.tf_buffer.lookup_transform(
                    color_frame, depth_frame, rclpy.time.Time()
                )
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
        max_age = float(perception.get("depth_max_age_s", 1.0))
        min_range = float(perception.get("depth_min_m", 0.15))
        max_range = float(perception.get("depth_max_m", 8.0))
        camera_point = None
        source_header = None

        depth_image = self._camera_depth_image
        camera_info = self._camera_info
        if depth_image is not None and camera_info is not None:
            depth_header = getattr(depth_image, "header", None)
            depth_time = self._stamp_seconds(depth_header)
            if (
                image_time is None
                or depth_time is None
                or abs(image_time - depth_time) <= max_age
            ):
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
                        depth_scale=(
                            None
                            if configured_scale is None
                            else float(configured_scale)
                        ),
                        **extra,
                    )
                    source_header = depth_header

        cloud = self._camera_depth_cloud
        if camera_point is None and cloud is not None:
            cloud_header = getattr(cloud, "header", None)
            cloud_time = self._stamp_seconds(cloud_header)
            if (
                image_time is None
                or cloud_time is None
                or abs(image_time - cloud_time) <= max_age
            ):
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
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.map_frame, frame_id, stamp, timeout=Duration(seconds=0.1)
                    )
                except Exception:
                    tf = self.tf_buffer.lookup_transform(
                        self.map_frame, frame_id, rclpy.time.Time()
                    )
                map_point = transform_point(camera_point, tf.transform)
            if map_point is None:
                return None
            return {
                "x": round(float(map_point[0]), 3),
                "y": round(float(map_point[1]), 3),
            }
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
                    detail = f"using direct {self.map_frame}-frame odometry instead"
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

    def pose7(self, stamp: Any | None = None) -> np.ndarray | None:
        """Full ``T_map_base`` as ``[x,y,z,qx,qy,qz,qw]`` for keyframe upload."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                stamp if stamp is not None else rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            return np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w], dtype=np.float64)
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

    def set_stand_height(self, height: float) -> bool:
        """Command Spot's stand height in range [-0.15, 0.15] meters relative to default."""
        try:
            h = float(height)
        except (ValueError, TypeError):
            self.node.get_logger().warn(f"[{self.id}] invalid stand height {height!r}")
            return False
        clamped_h = max(-0.15, min(0.15, h))
        self._target_body_height = clamped_h

        # 1. Update Spot mobility parameters via /body_pose so walking (cmd_vel)
        # and locomotion commands maintain this body height offset.
        if getattr(self, "pub_body_pose", None) is not None and Pose is not None:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = float(clamped_h)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            self.pub_body_pose.publish(pose)

        # 2. Call /set_stand_height service to immediately update the stand posture if standing.
        client = self._stand_height_client
        if client is not None and SetStandHeight is not None:
            if client.wait_for_service(timeout_sec=1.0):
                req = SetStandHeight.Request()
                req.height = float(clamped_h)
                future = client.call_async(req)
                deadline = time.monotonic() + 5.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if future.done():
                    try:
                        resp = future.result()
                        ok = bool(getattr(resp, "success", True))
                        message = str(getattr(resp, "message", "") or "")
                        if ok:
                            self.node.get_logger().info(
                                f"[{self.id}] set_stand_height({clamped_h:+.2f}m): ok"
                                + (f" ({message})" if message else "")
                            )
                        else:
                            self.node.get_logger().warn(
                                f"[{self.id}] set_stand_height({clamped_h:+.2f}m): refused"
                                + (f" ({message})" if message else "")
                            )
                    except Exception as exc:
                        self.node.get_logger().warn(
                            f"[{self.id}] set_stand_height failed: {exc}"
                        )
        return True

    def body_command(
        self, action: str, height: float | None = None, **kwargs: Any
    ) -> None:
        """Claim/release the body lease, sit/stand, or adjust stand height.

        `stand` powers the motors first when `services.power_on` is set —
        Clearpath's `/stand` fails if the robot is still sitting unpowered.
        `claim` also releases the software e-stop and drops a leftover tablet
        keepalive; without that, `/power_on` returns KeepaliveMotorsOffError
        and the GUI button looks like a no-op. Each call is a Trigger;
        failures are logged and not retried here.
        """
        action = str(action or "")
        if action == "set_height" or (action == "stand" and height is not None):
            if height is not None:
                self.set_stand_height(float(height))
                return
        if action not in BODY_ACTIONS:
            return
        pub = getattr(self, "pub_body_cmd", None)
        if pub is not None and String is not None:
            msg = String()
            msg.data = action
            pub.publish(msg)
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
            if self._target_body_height != 0.0:
                self.set_stand_height(self._target_body_height)
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
            self.node.get_logger().warn(f"[{self.id}] body service {name!r} timed out")
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
                f"[{self.id}] body {name}: refused"
                + (f" ({message})" if message else "")
            )
        return ok

    def _call_trigger_async(self, name: str) -> bool:
        """Start a Trigger request without holding up the control path.

        This is reserved for commands such as Spot's SDK stop where waiting for
        the service response would delay the operator's next velocity command.
        Ordinary body transitions remain synchronous so their required order
        (claim, e-stop release, power on, stand) is preserved.
        """
        client = self._body_clients.get(name)
        if client is None or Trigger is None:
            if name not in OPTIONAL_BODY_SERVICES:
                self.node.get_logger().warn(
                    f"[{self.id}] body command {name!r} has no service configured"
                )
            return False
        try:
            if not client.wait_for_service(timeout_sec=0.0):
                self.node.get_logger().warn(
                    f"[{self.id}] body service {name!r} is not up "
                    "(is spot_driver running?)"
                )
                return False
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] body service {name!r} failed to start: {exc}"
            )
            return False

        def report_result(done) -> None:
            try:
                resp = done.result()
            except Exception as exc:
                self.node.get_logger().warn(
                    f"[{self.id}] body service {name!r} failed: {exc}"
                )
                return
            ok = bool(getattr(resp, "success", True))
            message = str(getattr(resp, "message", "") or "")
            log = self.node.get_logger().info if ok else self.node.get_logger().warn
            state = "ok" if ok else "refused"
            log(
                f"[{self.id}] body {name}: {state}"
                + (f" ({message})" if message else "")
            )

        future.add_done_callback(report_result)
        return True

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
        self._trajectory_target = None
        self._trajectory_step = ""
        self._trajectory_step_count = 0
        self._trajectory_step_error = None

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
        future.add_done_callback(lambda f, g=generation: self._on_goal_response(f, g))

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
        self._arm_goal(goal)
        if self._trajectory_is_diff_drive():
            self._trajectory_target = {
                "x": float(goal["x"]),
                "y": float(goal["y"]),
            }
            if "yaw" in goal and goal["yaw"] is not None:
                self._trajectory_target["yaw"] = float(goal["yaw"])
            self._trajectory_step_count = 0
            self._continue_diff_trajectory(generation)
            return
        self._trajectory_target = None
        self._send_trajectory_pose(pose, generation, "holonomic")

    def _trajectory_is_diff_drive(self) -> bool:
        tcfg = self.cfg.get("trajectory") or {}
        control_mode = str(tcfg.get("control_mode") or "").strip().lower()
        if control_mode:
            return control_mode == "differential"
        # Backwards compatibility for profiles written before control_mode was
        # explicit. New Spot profiles keep a small non-zero SDK lateral limit:
        # an exactly zero-width velocity envelope makes its trajectory planner
        # report straight goals as BLOCKED on the current robot software.
        limit = tcfg.get("velocity_limit") or {}
        try:
            return math.isclose(float(limit.get("linear_y", 1.0)), 0.0, abs_tol=1e-9)
        except (TypeError, ValueError):
            return False

    def _body_trajectory_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(
            (self.cfg.get("trajectory") or {}).get("frame") or "body"
        )
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _progress_frame(self) -> str:
        """Frame to read remaining map-frame error from between diff-drive phases.

        Distinct from `trajectory.frame` (the frame_id Spot's driver requires
        on the outgoing command, and rejects any other value for). This is
        only ever used to compute numeric dx/dy/yaw internally — those numbers
        get repackaged by `_body_trajectory_pose` with `trajectory.frame` set
        again, so pointing this at a different, faster-updating frame never
        reaches the driver. See spot.yaml's `trajectory.progress_frame`
        comment for why Spot profiles set this to `body_fast`.
        """
        tcfg = self.cfg.get("trajectory") or {}
        frame = str(tcfg.get("progress_frame") or "").strip()
        return frame or str(tcfg.get("frame") or "body")

    def _continue_diff_trajectory(self, generation: int) -> None:
        """Issue the next rotate/straight/rotate step for a map-frame target."""
        if generation != self._goal_generation or self._trajectory_target is None:
            return
        tcfg = self.cfg.get("trajectory") or {}
        max_steps = max(1, int(tcfg.get("max_diff_drive_steps", 8)))
        if self._trajectory_step_count >= max_steps:
            self.node.get_logger().warn(
                f"[{self.id}] Spot differential trajectory did not converge "
                f"after {max_steps} steps"
            )
            self._finish_goal("failed")
            return

        relative = self._goal_in_body(
            self._trajectory_target, frame=self._progress_frame()
        )
        if relative is None:
            self._finish_goal("failed")
            return
        dx = float(relative.pose.position.x)
        dy = float(relative.pose.position.y)
        distance = math.hypot(dx, dy)
        position_tolerance = max(
            0.01, float(tcfg.get("position_tolerance_m", 0.15))
        )
        heading_tolerance = max(
            0.01, float(tcfg.get("heading_tolerance_rad", 0.08))
        )

        if distance > position_tolerance:
            bearing = math.atan2(dy, dx)
            # Do not ask Spot for a tiny in-place Trajectory rotation merely
            # because the map click is a few degrees off the body axis. The
            # Clearpath server commonly stops those commands immediately as
            # `not at goal`. If the resulting lateral miss is already inside
            # our position tolerance, drive the longitudinal projection now.
            if abs(dy) > position_tolerance:
                pose = self._body_trajectory_pose(0.0, 0.0, bearing)
                step = "align"
                step_error = abs(bearing)
            else:
                # Re-evaluate the map target after every completed action. Once
                # aligned, a body-x command is the differential-drive segment;
                # never pass the residual body-y error to Spot.
                pose = self._body_trajectory_pose(dx, 0.0, 0.0)
                step = "drive"
                step_error = distance
        elif "yaw" in self._trajectory_target:
            final_yaw = yaw_of(relative.pose.orientation)
            if abs(final_yaw) <= heading_tolerance:
                self.node.get_logger().info(
                    f"[{self.id}] Spot differential trajectory reached goal"
                )
                self._finish_goal("succeeded")
                return
            pose = self._body_trajectory_pose(0.0, 0.0, final_yaw)
            step = "final_turn"
            step_error = abs(final_yaw)
        else:
            self.node.get_logger().info(
                f"[{self.id}] Spot differential trajectory reached goal"
            )
            self._finish_goal("succeeded")
            return

        self._trajectory_step_count += 1
        self._trajectory_step_error = step_error
        self._send_trajectory_pose(pose, generation, step)

    def _send_trajectory_pose(
        self, pose: PoseStamped, generation: int, step: str
    ) -> None:
        tcfg = self.cfg.get("trajectory") or {}
        dur_s = max(1.0, float(tcfg.get("duration_s", 30.0)))
        msg = Trajectory.Goal()
        msg.target_pose = pose
        msg.duration.sec = int(dur_s)
        msg.duration.nanosec = int((dur_s - int(dur_s)) * 1e9)
        # Each differential phase is followed by a fresh map-frame error
        # check, so NEAR_GOAL is sufficient. The current Clearpath ROS 2 port
        # ignores this flag, but setting it correctly also supports newer
        # driver versions that honour it.
        msg.precise_positioning = (
            False
            if self._trajectory_target is not None
            else bool(tcfg.get("precise_positioning", True))
        )
        msg.disable_obstacle_avoidance = bool(
            tcfg.get("disable_obstacle_avoidance", False)
        )
        self._trajectory_step = step
        yaw = yaw_of(pose.pose.orientation)
        self.node.get_logger().info(
            f"[{self.id}] Spot trajectory step={step} "
            f"x={float(pose.pose.position.x):.3f} "
            f"y={float(pose.pose.position.y):.3f} yaw={yaw:.3f}"
        )
        try:
            future = self.traj_client.send_goal_async(msg)
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] Spot trajectory submission failed: {exc}"
            )
            self._finish_goal("failed")
            return
        future.add_done_callback(lambda f, g=generation: self._on_goal_response(f, g))

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
            or linear_x <= 0.0
            or linear_y < 0.0
            or angular_z <= 0.0
        ):
            self.node.get_logger().warn(
                f"[{self.id}] trajectory linear_x/angular_z limits must be positive "
                "and linear_y must be non-negative; goal dropped"
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

    def _goal_in_body(
        self, goal: dict[str, float], *, frame: str | None = None
    ) -> PoseStamped | None:
        frame = frame or str((self.cfg.get("trajectory") or {}).get("frame") or "body")
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.map_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] {self.map_frame} -> {frame} TF failed; goal dropped: {exc}"
            )
            return None
        xyz = transform_point((float(goal["x"]), float(goal["y"]), 0.0), tf.transform)
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
                type(
                    "T",
                    (),
                    {
                        "rotation": tf.transform.rotation,
                        "translation": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                    },
                )(),
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
        except Exception as exc:
            if generation != self._goal_generation:
                return
            self.node.get_logger().warn(
                f"[{self.id}] navigation goal response failed: {exc}"
            )
            self._finish_goal("failed")
            return
        # A cancellation or newer goal can win while send_goal_async is still
        # in flight. Cancel an accepted stale handle instead of abandoning an
        # action that would continue computing and publishing velocity forever.
        if generation != self._goal_generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.node.get_logger().warn(
                f"[{self.id}] navigation goal rejected"
                + (f" during {self._trajectory_step}" if self._trajectory_step else "")
            )
            self._finish_goal("failed")
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
            outcome = future.result()
            status = outcome.status
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.id}] navigation result failed: {exc}"
            )
            self._finish_goal("failed")
            return
        result = getattr(outcome, "result", None)
        reported_success = bool(getattr(result, "success", True))
        message = str(getattr(result, "message", "") or "")
        if status == GoalStatus.STATUS_SUCCEEDED and reported_success:
            if self._trajectory_target is not None:
                self.node.get_logger().info(
                    f"[{self.id}] Spot trajectory step={self._trajectory_step} complete"
                )
                self._goal_handle = None
                self._continue_diff_trajectory(generation)
                return
            self._finish_goal("succeeded")
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._finish_goal("cancelled")
            return
        if (
            self._trajectory_target is not None
            and message == "not at goal"
            and self._diff_trajectory_made_progress()
        ):
            self.node.get_logger().info(
                f"[{self.id}] Spot trajectory step={self._trajectory_step} "
                "stopped early after making progress; recomputing target"
            )
            self._goal_handle = None
            self._continue_diff_trajectory(generation)
            return
        self.node.get_logger().warn(
            f"[{self.id}] navigation failed status={status}"
            + (f" step={self._trajectory_step}" if self._trajectory_step else "")
            + (f": {message}" if message else "")
        )
        self._finish_goal("failed")

    def _diff_trajectory_made_progress(self) -> bool:
        """Accept a driver `not at goal` only when TF proves useful motion."""
        target = self._trajectory_target
        before = self._trajectory_step_error
        if target is None or before is None:
            return False
        relative = self._goal_in_body(target, frame=self._progress_frame())
        if relative is None:
            return False
        dx = float(relative.pose.position.x)
        dy = float(relative.pose.position.y)
        tcfg = self.cfg.get("trajectory") or {}
        if self._trajectory_step == "drive":
            after = math.hypot(dx, dy)
            tolerance = max(0.01, float(tcfg.get("position_tolerance_m", 0.15)))
            minimum = max(0.005, float(tcfg.get("minimum_progress_m", 0.03)))
        elif self._trajectory_step == "align":
            after = abs(math.atan2(dy, dx))
            tolerance = max(0.01, float(tcfg.get("heading_tolerance_rad", 0.08)))
            minimum = max(
                0.005, float(tcfg.get("minimum_progress_rad", 0.02))
            )
        elif self._trajectory_step == "final_turn":
            after = abs(yaw_of(relative.pose.orientation))
            tolerance = max(0.01, float(tcfg.get("heading_tolerance_rad", 0.08)))
            minimum = max(
                0.005, float(tcfg.get("minimum_progress_rad", 0.02))
            )
        else:
            return False
        self.node.get_logger().info(
            f"[{self.id}] Spot trajectory step={self._trajectory_step} "
            f"remaining_error={after:.3f} previous_error={before:.3f}"
        )
        return after <= tolerance or after <= before - minimum

    def _finish_goal(self, status: str) -> None:
        self.nav_status = status
        self.mode = "idle"
        self.goal = None
        self.planned_path = []
        self._local_planned_path = []
        self._global_planned_path = []
        self._goal_handle = None
        self._trajectory_target = None
        self._trajectory_step = ""
        self._trajectory_step_count = 0
        self._trajectory_step_error = None

    def cancel_goal(self) -> None:
        self._goal_generation += 1
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self.goal = None
        self.planned_path = []
        self._local_planned_path = []
        self._global_planned_path = []
        self.nav_status = "cancelled"
        self.mode = "idle"
        self._trajectory_target = None
        self._trajectory_step = ""
        self._trajectory_step_count = 0
        self._trajectory_step_error = None
        # Clearpath's ROS 2 Trajectory server never checks cancel/preempt
        # (the ROS 1 path that called spot_wrapper.stop() is commented out).
        # A zero cmd_vel preempts the SDK trajectory immediately. Also enqueue
        # `/stop` as a backstop, but never wait for that service round trip here:
        # drive() calls this inline before publishing the operator's command.
        if self.traj_client is not None:
            if self.pub_cmd is not None:
                zero = Twist()
                zero.linear.x = 0.0
                zero.linear.y = 0.0
                zero.angular.z = 0.0
                self.pub_cmd.publish(zero)
            self._call_trigger_async("stop")

    def stop_for_exit(self) -> None:
        """Flush a synchronous SDK stop before the ROS executor is torn down."""
        for _ in range(3):
            try:
                self.cancel_goal()
                self.drive(0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.05)
        if self.traj_client is not None:
            self._call_trigger("stop")

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

    def upload_costmaps(self) -> None:
        """Push the newest global/local Nav2 snapshots to the read-only overlay."""
        with self._costmap_lock:
            pending = [
                (kind, self._costmaps[kind])
                for kind in tuple(self._costmap_dirty)
                if kind in self._costmaps
            ]
            for kind, _ in pending:
                self._costmap_dirty.discard(kind)

        for kind, snapshot in pending:
            body = zlib.compress(
                np.ascontiguousarray(snapshot.cells, dtype=np.int8).tobytes(), 1
            )
            frame = urllib.parse.quote(snapshot.frame_id, safe="")
            url = (
                f"{self.http_url}/api/adapter/costmap?robot_id={self.id}"
                f"&kind={kind}&resolution={snapshot.resolution}"
                f"&width={snapshot.width}&height={snapshot.height}"
                f"&origin_x={snapshot.origin_x}&origin_y={snapshot.origin_y}"
                f"&frame_id={frame}"
            )
            try:
                urllib.request.urlopen(
                    urllib.request.Request(
                        url,
                        data=body,
                        headers={"Content-Type": "application/octet-stream"},
                    ),
                    timeout=float(self.cfg.get("upload_timeout_s", 25.0)),
                ).read()
            except Exception as exc:
                with self._costmap_lock:
                    if self._costmaps.get(kind) is snapshot:
                        self._costmap_dirty.add(kind)
                self.node.get_logger().warn(
                    f"[{self.id}] {kind} costmap upload failed: {exc}"
                )

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
            offsets = points - np.array([origin["x"], origin["y"]], dtype=np.float32)
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
                    url,
                    data=zlib.compress(quantised.tobytes()),
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
                    url,
                    data=zlib.compress(quantised.tobytes(), 1),
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


async def run_robot(bridge: HardwareBridge, ws_url: str) -> None:
    """Connect, announce, then pump state until the link drops. Repeat.

    Protocol rule 2: reconnect with backoff and re-send `hello` every time.
    """
    await run_adapter_session(bridge, ws_url, connect=websockets.connect)


def load_config(path: str | None) -> dict:
    return load_yaml_profile(path, DEFAULTS)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--robot-id",
        required=True,
        help="Stable identity, used as the key everywhere (rule 5)",
    )
    ap.add_argument(
        "--config", default="", help="YAML of topics/frames/rates for this robot type"
    )
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
