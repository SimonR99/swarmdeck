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
object to hang callbacks off. `Nav2 point-goal action` and `actionlib`'s
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
import math
import sys
import signal
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any

import numpy as np
import rospy
import websockets
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    CompressedImage,
    Image,
    Joy,
    PointCloud2,
)
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener

# Keep perception independent of ROS packaging, as adapter_sim does.  Hardware
# containers run this file directly, so the repository root is not otherwise on
# sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "protocol"))

from adapters.perception.depth_projection import (
    point_for_bbox,
    point_for_depth_image,
    transform_point,
    transform_points,
)
from adapters.runtime import (
    AdapterDetectionMixin,
    AdapterHelloMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
    deep_merge,
    load_yaml_profile,
    stamp_seconds,
    yaw_of,
)
from adapters.session import run_adapter_session
from ros1_defaults import DEFAULTS

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


class HardwareBridge(
    AdapterHelloMixin,
    AdapterDetectionMixin,
    AdapterLinkMixin,
    AdapterSensorMixin,
    AdapterTelemetryMixin,
):
    """One real robot's ROS interface, expressed as the SwarmDeck protocol."""

    adapter_name = "adapter_ros1/0.1.0"
    coordinate_frame = "local"

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
        self.navigation_frame = cfg["navigation_frame"]
        self.base_frame = cfg["base_frame"]
        topics = cfg["topics"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

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
        if self._detection_enabled and (
            topics.get("camera") or topics.get("camera_compressed")
        ):
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
        if topics.get("plan"):
            rospy.Subscriber(topics["plan"], NavPath, self._on_plan, queue_size=10)
        if topics.get("battery"):
            battery_topic = topics["battery"]
            msg_cls = BatteryState
            if "scout_status" in battery_topic:
                try:
                    from scout_msgs.msg import ScoutStatus

                    msg_cls = ScoutStatus
                except ImportError:
                    try:
                        import roslib.message

                        msg_cls = (
                            roslib.message.get_message_class("scout_msgs/ScoutStatus")
                            or BatteryState
                        )
                    except Exception:
                        pass
            rospy.Subscriber(battery_topic, msg_cls, self._on_battery, queue_size=10)
        # Prefer compressed: a raw camera stream at full rate is the single most
        # expensive thing an adapter can subscribe to over a robot's network.
        # Frames stay on-robot for detection; the operator picture is WebRTC.
        if topics.get("camera_compressed"):
            rospy.Subscriber(
                topics["camera_compressed"],
                CompressedImage,
                self._on_camera_compressed,
                queue_size=1,
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
            if topics.get("cmd_vel")
            else None
        )
        self.pub_nav_goal = (
            rospy.Publisher(topics["nav_goal"], PoseStamped, queue_size=1)
            if topics.get("nav_goal")
            else None
        )
        self.pub_nav_stop = (
            rospy.Publisher(topics["nav_stop"], Int8, queue_size=1)
            if topics.get("nav_stop")
            else None
        )
        self.pub_nav_joy = (
            rospy.Publisher(topics["nav_joy"], Joy, queue_size=1)
            if topics.get("nav_joy")
            else None
        )
        self._nav_joy_throttle = float(cfg.get("nav_joy_throttle", 0.5))

        # Vendor goal actions and topic planners are pass-through targets.
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

    def capabilities(self) -> list[str]:
        """Only what this robot can actually honour (protocol rule 4)."""
        caps: list[str] = []
        if self.nav_client is not None or self.pub_nav_goal is not None:
            caps.append("navigate")
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
        return caps

    # ------------------------------------------------------------- ROS inputs

    def _on_plan(self, msg: NavPath) -> None:
        """Publish the planner's intended route, in `navigation_frame`.

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
            self._local_planned_path = []
            return

        frame = msg.header.frame_id.lstrip("/")
        if not frame:
            frame = self.base_frame
        if frame == self.navigation_frame:
            self.planned_path = [
                {"x": ps.pose.position.x, "y": ps.pose.position.y} for ps in msg.poses
            ]
            if self.pub_nav_goal is not None:
                self._local_planned_path = self.planned_path.copy()
            else:
                self._global_planned_path = self.planned_path.copy()
            return

        try:
            stamp_val = getattr(msg.header, "stamp", None)
            if stamp_val is None:
                stamp = rospy.Time(0)
            elif hasattr(stamp_val, "to_sec"):
                stamp = stamp_val if stamp_val.to_sec() > 0 else rospy.Time(0)
            elif isinstance(stamp_val, (int, float)):
                stamp = (
                    rospy.Time.from_sec(stamp_val) if stamp_val > 0 else rospy.Time(0)
                )
            else:
                stamp = stamp_val
            tf = self.tf_buffer.lookup_transform(
                self.navigation_frame, frame, stamp, rospy.Duration(0.1)
            )
        except Exception:
            # Drop the path rather than draw it in the wrong frame: an operator
            # reading a route that is confidently somewhere the robot is not is
            # worse off than one reading no route at all.
            if not self._plan_frame_warned:
                self._plan_frame_warned = True
                rospy.logwarn(
                    f"[{self.id}] no {self.navigation_frame} -> {frame} transform for the "
                    f"planned path; not publishing it. The plan topic is in a frame "
                    f"this robot's TF tree does not connect to {self.navigation_frame}."
                )
            self.planned_path = []
            self._local_planned_path = []
            return

        points = np.array(
            [
                [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                for ps in msg.poses
            ],
            dtype=np.float64,
        )
        mapped = transform_points(points, tf.transform)
        mapped_list = [
            {"x": round(float(x), 3), "y": round(float(y), 3)} for x, y in mapped[:, :2]
        ]
        self.planned_path = mapped_list
        self._local_planned_path = mapped_list

    def _on_camera_compressed(self, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower():
            return
        # Queue for detection; do NOT run inference here. See run_detection().
        jpeg = bytes(msg.data)
        self._detect_pending = (jpeg, getattr(msg, "header", None))

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
        jpeg = buf.tobytes()
        self._detect_pending = (jpeg, getattr(msg, "header", None))

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
            try:
                tf = self.tf_buffer.lookup_transform(
                    color_frame, depth_frame, stamp, rospy.Duration(0.1)
                )
            except Exception:
                tf = self.tf_buffer.lookup_transform(
                    color_frame, depth_frame, rospy.Time(0)
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
            if frame_id == self.navigation_frame:
                map_point = camera_point
            else:
                stamp = getattr(source_header, "stamp", rospy.Time(0))
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.navigation_frame, frame_id, stamp, rospy.Duration(0.1)
                    )
                except Exception:
                    tf = self.tf_buffer.lookup_transform(
                        self.navigation_frame, frame_id, rospy.Time(0)
                    )
                map_point = transform_point(camera_point, tf.transform)
            if map_point is None:
                return None
            return {
                "x": round(float(map_point[0]), 3),
                "y": round(float(map_point[1]), 3),
            }
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0,
                f"[{self.id}] cannot place camera detection in {self.navigation_frame}: {exc}",
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
                self.navigation_frame, self.base_frame, rospy.Time(0)
            )
            t = tf.transform
            return {
                "x": t.translation.x,
                "y": t.translation.y,
                "z": t.translation.z,
                "yaw": yaw_of(t.rotation),
            }
        except Exception:
            fallback = getattr(self, "_odom_pose", None)
            if fallback is None:
                return {"x": 0.0, "y": 0.0, "yaw": 0.0}
            if not self._pose_warned:
                self._pose_warned = True
                rospy.logwarn(
                    f"[{self.id}] no {self.navigation_frame} -> {self.base_frame} transform; "
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
        self._nav_waypoints = []
        self._nav_route_key = None
        self._nav_route_blocked = False
        self._local_planned_path = []
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
        msg.target_pose.header.frame_id = self.navigation_frame
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
            msg,
            done_cb=lambda status, result, g=generation: self._on_goal_done(status, g),
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
        self._nav_waypoints = []

    def _navigate_to_topic(self, goal: dict[str, float]) -> None:
        """`move_base_simple/goal`-style: a plain publish, not an action.

        No actionlib means no "accepted"/"succeeded" callback — a stack like
        local_planner just starts driving. Progress is instead polled each
        state tick by `_check_topic_nav_progress` against this adapter's own
        tf2 pose, the same source `state()` already reports from.
        """
        self._goal_generation += 1
        msg = PoseStamped()
        msg.header.frame_id = self.navigation_frame
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
        if (
            self.pub_nav_goal is None
            or self.nav_status != "active"
            or self.goal is None
        ):
            return
        pose = self.map_pose()
        dist = math.hypot(pose["x"] - self.goal["x"], pose["y"] - self.goal["y"])
        if dist <= float(self.cfg.get("nav_goal_tolerance_m", 0.5)):
            self.nav_status = "succeeded"
            self.mode = "idle"
            self.goal = None
            self._nav_waypoints = []
            self.planned_path = []
            self._local_planned_path = []
            self._global_planned_path = []

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

        When global path waypoints are available, advance the lookahead waypoint
        along the global collision-free route so the robot follows paths around
        corners and walls rather than beelining in a straight line.
        """
        if self.pub_nav_joy is None:
            return
        msg = Joy()
        if self.nav_status == "active" and self.goal is not None:
            pose = self.map_pose()
            target_pt = self.goal
            waypoints = getattr(self, "_nav_waypoints", None)
            if waypoints:
                target_pt = self.navigation_route_target(waypoints, pose)
                if target_pt is None:
                    # Gate the native velocity relay too: pathFollower smooths
                    # speed and may otherwise keep moving after zero joystick.
                    self.pub_cmd.publish(Twist())
                    msg.axes = [0.0, 0.0, 0.0]
                    msg.buttons = [0] * 8
                    self.pub_nav_joy.publish(msg)
                    rospy.logwarn_throttle(
                        2.0,
                        f"[{self.id}] route hold: "
                        f"{self._nav_route_tracker.blocked_reason}",
                    )
                    return
            else:
                self._nav_route_blocked = False

            dx = target_pt["x"] - pose["x"]
            dy = target_pt["y"] - pose["y"]
            c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
            forward = dx * c + dy * s
            left = -dx * s + dy * c
            bearing = math.atan2(left, forward)
            rospy.loginfo_throttle(
                1.0,
                f"[{self.id}] nav tracking pose=({pose['x']:.3f},{pose['y']:.3f},"
                f"{pose['yaw']:.3f}) target=({target_pt['x']:.3f},{target_pt['y']:.3f}) "
                f"bearing_deg={math.degrees(bearing):.1f} "
                f"progress_m={self._nav_route_tracker.progress if waypoints else 0.0:.3f} "
                f"target_m={self._nav_route_tracker.target_progress if waypoints else 0.0:.3f}",
            )
            t = self._nav_joy_throttle
            forward_axis = math.cos(bearing) * t
            lateral_axis = math.sin(bearing) * t
            # Scout's native localPlanner mirrors atan2 when reversing. Encode
            # its joystick convention so the decoded path bearing stays in the
            # intended quadrant, including when crossing +/-90 degrees.
            if forward_axis < 0 and self.cfg.get("nav_joy_reverse_steering", False):
                lateral_axis = -lateral_axis
            msg.axes = [0.0, forward_axis, lateral_axis]
        else:
            msg.axes = [0.0, 0.0, 0.0]
        # localPlanner reads buttons[4] and buttons[6] to toggle obstacle
        # checking. Short messages cause unchecked native vector reads.
        msg.buttons = [0] * 8
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
        self._nav_waypoints = []
        self.planned_path = []
        self._local_planned_path = []
        self._global_planned_path = []
        self.nav_status = "cancelled"
        self.mode = "idle"

    # ------------------------------------------------------------- uploads

    def session_state_tick(self) -> None:
        self._check_topic_nav_progress()
        self._pump_nav_joy()


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

    # disable_signals: we drive our own asyncio.run() as the process's main
    # loop and handle KeyboardInterrupt ourselves, same shape as adapter_ros2's
    # rclpy.init()/rclpy.shutdown() bracketing.
    rospy.init_node(
        f"swarmdeck_adapter_{args.robot_id}", anonymous=False, disable_signals=True
    )
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
