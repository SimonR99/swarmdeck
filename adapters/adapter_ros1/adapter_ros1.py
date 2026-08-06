#!/usr/bin/env python3
"""Hardware adapter: a real ROS 1 robot -> SwarmDeck adapter protocol.

    python3 adapter_ros1.py --robot-id tars_0 --config robot.yaml

One process per robot, running ON the robot (or on a machine that shares its ROS
graph). This is `adapter_ros2` ported to `rospy`/`actionlib`, not a different
design: same protocol, same config schema, same capability/deadman rules. It
exists because some robots in this fleet (the AgileX Scout/Bunker units) run a
real ROS 1 stack today and porting *that* to ROS 2 is out of scope for joining
a fleet — see docs/fleet-status.md for which robot is which.

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
hypothesis until a robot proves it. See docs/hardware-bringup.md.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
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
from sensor_msgs.msg import BatteryState, CompressedImage, Image, PointCloud2
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener

# Keep perception independent of ROS packaging, as adapter_sim does.  Hardware
# containers run this file directly, so the repository root is not otherwise on
# sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

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
    },
    # Height band for `map_cloud` points, metres, in the map_frame (NOT
    # relative to the robot — this stack has no live z estimate to be relative
    # to, since the EKF publishing map_frame runs `two_d_mode: true`). Points
    # outside are dropped before upload: ground and ceiling returns would
    # otherwise flood the grid with false occupied cells. The default is a
    # starting guess, not a calibration — verify against the actual map that
    # comes out, per docs/hardware-bringup.md.
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
        "camera_period_s": 0.2,   # 5 Hz, per the protocol's preview cap
    },
    "perception": {
        "enabled": True,
        "period_s": 0.2,
        "sensitivity": 0.55,
    },
    # A drive command older than this stops the robot. Teleop over a network is
    # only safe with a deadman: if the link drops mid-command the robot must not
    # keep executing the last velocity it heard.
    "drive_timeout_s": 0.45,
}


def deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class HardwareBridge:
    """One real robot's ROS interface, expressed as the SwarmDeck protocol."""

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
        self._camera_jpeg: bytes | None = None
        self._camera_dirty = False
        perception = cfg.get("perception", {})
        self._detector = None
        self._detection_enabled = bool(perception.get("enabled", True))
        if self._detection_enabled and (topics.get("camera") or topics.get("camera_compressed")):
            try:
                from adapters.perception.duck_detector import RubberDuckDetector

                self._detector = RubberDuckDetector(perception.get("sensitivity", 0.55))
            except ImportError as exc:
                self._detection_enabled = False
                rospy.logwarn(f"[{self.id}] camera detection disabled: {exc}")
        self._detection_period_s = max(0.05, float(perception.get("period_s", 0.2)))
        self._last_detection_at = 0.0
        self._detections: list[dict] | None = None
        self._pose_warned = False

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
        # expensive thing an adapter can subscribe to over a robot's network, and
        # the preview is throttled to 5 Hz anyway.
        if topics.get("camera_compressed"):
            rospy.Subscriber(
                topics["camera_compressed"], CompressedImage,
                self._on_camera_compressed, queue_size=1,
            )
        elif topics.get("camera"):
            rospy.Subscriber(topics["camera"], Image, self._on_camera_raw, queue_size=1)
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

        # `nav_goal` (topic-based, e.g. local_planner) takes priority over
        # `actions.navigate_to_pose` (actionlib, e.g. move_base) when both are
        # configured — see the DEFAULTS comment on `topics.nav_goal`.
        action_name = cfg.get("actions", {}).get("navigate_to_pose")
        self.nav_client = None
        if self.pub_nav_goal is None and action_name and actionlib is not None:
            self.nav_client = actionlib.SimpleActionClient(action_name, MoveBaseAction)

    # ------------------------------------------------------------- capabilities

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
        if self.pub_cmd is not None:
            caps.append("estop")
        return caps

    # ------------------------------------------------------------- ROS inputs

    def _on_odom(self, msg: Odometry) -> None:
        # Kept only as a FALLBACK for the pose. See map_pose(): odometry drifts
        # without bound and is not where a robot's map-frame pose comes from.
        p = msg.pose.pose
        self._odom_pose = {
            "x": p.position.x, "y": p.position.y, "yaw": yaw_of(p.orientation)
        }

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        self._grid_dirty = True

    @staticmethod
    def _cloud_xyz(msg: PointCloud2) -> np.ndarray:
        """Extract xyz from a PointCloud2 without pulling in point_cloud2 helpers.

        Reads the x/y/z field offsets rather than assuming they are the first
        three floats: LVI-SAM's registered cloud carries intensity too, and its
        placement is not something to guess at. Mirrors adapter_sim's
        `_cloud_xyz` exactly — same wire format, same reasoning.
        """
        offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
        if len(offsets) != 3:
            return np.zeros((0, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        count = len(raw) // msg.point_step if msg.point_step else 0
        if not count:
            return np.zeros((0, 3), dtype=np.float32)
        rows = raw[: count * msg.point_step].reshape(count, msg.point_step)
        columns = [
            rows[:, offsets[axis] : offsets[axis] + 4].copy().view(np.float32).ravel()
            for axis in ("x", "y", "z")
        ]
        points = np.stack(columns, axis=1)
        return points[np.isfinite(points).all(axis=1)]

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

        band = self.cfg.get("map_cloud_height_band", {})
        min_z = float(band.get("min_z", -1e9))
        max_z = float(band.get("max_z", 1e9))
        xy = points[(points[:, 2] >= min_z) & (points[:, 2] <= max_z)][:, :2]
        if not len(xy):
            self._scan_points = np.zeros((0, 2), dtype=np.float32)
            self._scan_dirty = True
            return
        keys = np.round(xy / MAP_CLOUD_VOXEL).astype(np.int32)
        _, keep = np.unique(keys, axis=0, return_index=True)
        self._scan_points = xy[keep]
        self._scan_dirty = True

    def _on_plan(self, msg: NavPath) -> None:
        self.planned_path = [
            {"x": ps.pose.position.x, "y": ps.pose.position.y} for ps in msg.poses
        ]

    def _on_battery(self, msg: BatteryState) -> None:
        # REP-147 percentage is 0..1, but plenty of drivers publish 0..100 and
        # some publish NaN when the value is unknown. Normalise and refuse NaN
        # rather than reporting a battery the GUI would draw as empty.
        value = float(msg.percentage)
        if math.isnan(value):
            self.battery = None
            return
        self.battery = value / 100.0 if value > 1.0 else value

    def _on_camera_compressed(self, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower():
            return
        self._camera_jpeg = bytes(msg.data)
        self._camera_dirty = True
        self._detect_jpeg(self._camera_jpeg)

    def _on_camera_raw(self, msg: Image) -> None:
        # Encoding here rather than shipping raw: the protocol's preview channel
        # is JPEG. Imported lazily so a robot with no camera needs no OpenCV.
        try:
            import cv2
        except ImportError:
            return
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
            if msg.encoding in ("rgb8", "rgba8"):
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self._camera_jpeg = buf.tobytes()
                self._camera_dirty = True
                self._detect_bgr(frame)
        except (ValueError, TypeError):
            return

    def _detect_jpeg(self, jpeg: bytes) -> None:
        if not self._detection_due():
            return
        try:
            import cv2

            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self._detect_bgr(frame, due_checked=True)
        except (ValueError, TypeError):
            return

    def _detection_due(self) -> bool:
        if not self._detection_enabled or self._detector is None:
            return False
        now = time.monotonic()
        if now - self._last_detection_at < self._detection_period_s:
            return False
        self._last_detection_at = now
        return True

    def _detect_bgr(self, frame: np.ndarray, *, due_checked: bool = False) -> None:
        if not due_checked and not self._detection_due():
            return
        self._detections = [
            detection.as_protocol(f"duck_{index}")
            for index, detection in enumerate(self._detector.detect_bgr(frame))
        ]

    def take_detections(self) -> list[dict] | None:
        current = self._detections
        self._detections = None
        return current

    def refresh_settings(self) -> None:
        """Apply dashboard perception controls without coupling them to ROS."""
        try:
            with urllib.request.urlopen(f"{self.http_url}/api/settings", timeout=2) as response:
                payload = json.loads(response.read())
            settings = payload.get("settings", {})
            enabled = bool(settings.get("detection_enabled", True))
            if self._detection_enabled and not enabled:
                self._detections = []
            self._detection_enabled = enabled
            if self._detector is not None:
                self._detector.sensitivity = max(
                    0.1,
                    min(1.0, float(settings.get("detection_sensitivity", 0.55))),
                )
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] settings refresh failed: {exc}")

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

    def state(self) -> dict[str, Any]:
        return {
            "type": "robot_state",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "pose": self.map_pose(),
            "battery": self.battery,
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
            "planned_path": self.planned_path,
        }

    # ------------------------------------------------------------- commands

    def _on_nav_cmd_vel(self, msg: Twist) -> None:
        """Relay a topic-based nav stack's own cmd_vel — ONLY while navigating.

        `pathFollower` runs its control loop continuously and publishes to
        cmd_vel at ~27 Hz regardless of whether a goal is active — confirmed
        live, it was still streaming zero-velocity Twists with nothing to do.
        `topics.nav_stop` makes ITS commanded value zero but does not stop it
        from publishing, so a single teleop publish racing that continuous
        stream gets overwritten within tens of milliseconds either way.
        `topics.nav_cmd_vel` (see launch/local_planner_muxed.launch) remaps
        its output away from the real cmd_vel so this adapter is the ONLY
        thing that ever publishes there — teleop direct, nav relayed through
        here, never both racing the same topic.
        """
        if self.nav_status == "active" and self.pub_cmd is not None:
            self.pub_cmd.publish(msg)

    def drive(self, linear: float, angular: float) -> None:
        if self.pub_cmd is None:
            return
        moving = abs(linear) > 1e-3 or abs(angular) > 1e-3
        # Belt-and-suspenders even with nav_cmd_vel relaying: also tell a nav
        # stack that respects nav_stop to actually stop trying, and cancel our
        # own bookkeeping so state() stops reporting a goal teleop just pre-empted.
        if moving and self.pub_nav_stop is not None:
            self.pub_nav_stop.publish(Int8(data=1))
            if self.nav_status == "active":
                self.nav_status = "cancelled"
                self.goal = None
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub_cmd.publish(twist)
        self.mode = "teleop" if moving else self.mode
        self._last_drive_at = time.monotonic() if moving else 0.0

    def drive_watchdog(self) -> None:
        """Stop if teleop commands stop arriving.

        Not optional on hardware. The GUI sends `drive` continuously while a
        button is held; if the network drops mid-hold, the last command would
        otherwise execute forever.
        """
        if self.mode != "teleop" or self._last_drive_at == 0.0:
            return
        if time.monotonic() - self._last_drive_at > self.cfg["drive_timeout_s"]:
            self.drive(0.0, 0.0)
            self.mode = "idle"
            self._last_drive_at = 0.0

    def stop(self) -> None:
        self.drive(0.0, 0.0)
        self.cancel_goal()
        self.mode = "estop"

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
                timeout=5,
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
        origin = self.map_pose()
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
                timeout=5,
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
                timeout=5,
            ).read()
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] 3D cloud upload failed: {exc}")

    def upload_camera(self) -> None:
        if not self._camera_dirty or self._camera_jpeg is None:
            return
        self._camera_dirty = False
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    f"{self.http_url}/api/adapter/camera?robot_id={self.id}",
                    data=self._camera_jpeg,
                    headers={"Content-Type": "image/jpeg"},
                ),
                timeout=5,
            ).read()
        except Exception as exc:
            rospy.logwarn(f"[{self.id}] camera upload failed: {exc}")


async def run_robot(bridge: HardwareBridge, ws_url: str) -> None:
    """Connect, announce, then pump state until the link drops. Repeat.

    Protocol rule 2: reconnect with backoff and re-send `hello` every time. The
    backoff matters more on hardware than in simulation — a robot that hammers a
    downed backend over Wi-Fi wastes the airtime its own teleop needs.
    """
    backoff = 1.0
    while True:
        try:
            async with websockets.connect(ws_url, ping_interval=20) as ws:
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
                }))
                rospy.loginfo(
                    f"[{bridge.id}] connected; capabilities="
                    f"{bridge.capabilities()}"
                )
                backoff = 1.0

                async def rx() -> None:
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except ValueError:
                            continue
                        kind = msg.get("type")
                        if kind == "navigate_to":
                            bridge.navigate_to(msg.get("goal", {}))
                        elif kind == "cancel_goal":
                            bridge.cancel_goal()
                        elif kind == "drive":
                            bridge.drive(
                                msg.get("linear", 0.0), msg.get("angular", 0.0)
                            )
                        elif kind == "stop":
                            bridge.stop()
                        elif kind == "set_mode":
                            bridge.mode = msg.get("mode", bridge.mode)
                        # Rule 3: unknown types are ignored, not fatal.

                async def tx() -> None:
                    rates = bridge.cfg["rates"]
                    period = 1.0 / float(rates["state_hz"])
                    loop = asyncio.get_running_loop()
                    last_map = 0.0
                    last_cloud = 0.0
                    last_cam = 0.0
                    last_settings = 0.0
                    while True:
                        await ws.send(json.dumps(bridge.state()))
                        bridge.drive_watchdog()
                        bridge._check_topic_nav_progress()
                        now = time.monotonic()
                        if now - last_map > float(rates["map_period_s"]):
                            last_map = now
                            meta = await loop.run_in_executor(None, bridge.upload_map)
                            if meta:
                                await ws.send(json.dumps(meta))
                            await loop.run_in_executor(None, bridge.upload_scan)
                        if now - last_cloud > float(rates["cloud_period_s"]):
                            last_cloud = now
                            await loop.run_in_executor(None, bridge.upload_cloud)
                        if now - last_cam > float(rates["camera_period_s"]):
                            last_cam = now
                            await loop.run_in_executor(None, bridge.upload_camera)
                            detections = bridge.take_detections()
                            if detections is not None:
                                await ws.send(json.dumps({
                                    "type": "detections",
                                    "robot_id": bridge.id,
                                    "t_mono": round(now - bridge.t0, 4),
                                    "camera": "front",
                                    "items": detections,
                                }))
                        if now - last_settings > 5.0:
                            last_settings = now
                            await loop.run_in_executor(None, bridge.refresh_settings)
                        await asyncio.sleep(period)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            rospy.logwarn(f"[{bridge.id}] disconnected ({exc}); retrying in {backoff:.0f}s")
            # Stop the robot on link loss. A robot that keeps driving after
            # losing its operator is the failure that hurts someone.
            try:
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
    try:
        asyncio.run(run_robot(bridge, ws_url))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.drive(0.0, 0.0)
        except Exception:
            pass
        rospy.signal_shutdown("adapter exiting")


if __name__ == "__main__":
    main()
