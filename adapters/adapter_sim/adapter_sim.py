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
import json
import math
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
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Keep perception reusable by real adapters without packaging it into ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.perception.duck_detector import RubberDuckDetector


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


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class RobotBridge:
    """Per-robot ROS subscriptions and command publisher."""

    def __init__(self, node: Node, robot_id: str, http_url: str) -> None:
        self.node = node
        self.id = robot_id
        self.http_url = http_url
        self.t0 = time.monotonic()

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
        self._detector = RubberDuckDetector()
        self._detection_enabled = True
        self._detections: list[dict] | None = None
        self._goal_handle = None
        self._goal_generation = 0
        self._last_drive_at = 0.0

        node.create_subscription(Odometry, f"/{robot_id}/odom", self._on_odom, 10)
        node.create_subscription(TFMessage, f"/{robot_id}/tf", self._on_tf, 20)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, f"/{robot_id}/map", self._on_map, latched)
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
        node.create_subscription(
            Image,
            f"/{robot_id}/camera/image_raw",
            self._on_camera,
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
        wheels (8.8-30.5 m of error measured in docs/KNOWN_ISSUES.md), which is
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

    @staticmethod
    def _cloud_xyz(msg: PointCloud2) -> np.ndarray:
        """Extract xyz from a PointCloud2 without pulling in point_cloud2 helpers.

        Reads the x/y/z field offsets rather than assuming they are the first
        three floats: RTAB-Map's cloud carries intensity and ring fields too, and
        their placement is not something to guess at.
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

    def upload_cloud(self) -> None:
        """Voxel-downsample the 3D map and push it, quantised to 1 cm.

        Downsampling happens here rather than in the backend because this is the
        expensive end of a link that should stay cheap: a full RTAB-Map cloud is
        millions of points, and the view draws each one as a single pixel.
        """
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
            "robot_type": "duckiebot_db21",
            "adapter": "adapter_sim/0.1.0",
            "ros": "jazzy",
            "capabilities": ["navigate", "map", "camera", "battery", "estop"],
            "footprint_radius": 0.3,
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

    def _cancel_nav(self) -> None:
        self._goal_generation += 1  # Ignore callbacks from the superseded goal.
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None

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

    def upload_map(self) -> None:
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

    def upload_camera(self) -> None:
        """Encode the newest ROS image as a compact JPEG preview."""
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

            self._detections = (
                [
                    detection.as_protocol(f"duck_{index}")
                    for index, detection in enumerate(self._detector.detect_bgr(image))
                ]
                if self._detection_enabled
                else []
            )

            ok, encoded = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 78]
            )
            if not ok:
                return
            url = f"{self.http_url}/api/adapter/camera?robot_id={self.id}"
            urllib.request.urlopen(
                urllib.request.Request(
                    url,
                    data=encoded.tobytes(),
                    headers={"Content-Type": "image/jpeg"},
                ),
                timeout=2,
            ).read()
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] camera upload failed: {exc}")

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
        except Exception as exc:
            self.node.get_logger().warn(f"[{self.id}] settings refresh failed: {exc}")


async def run_robot(bridge: RobotBridge, ws_url: str) -> None:
    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                await ws.send(json.dumps(bridge.hello()))
                print(f"[adapter_sim] {bridge.id} connected")

                async def rx() -> None:
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

                async def tx() -> None:
                    last_map = 0.0
                    last_cloud = 0.0
                    last_graph = 0.0
                    last_camera = 0.0
                    last_settings = 0.0
                    loop = asyncio.get_running_loop()
                    while True:
                        await ws.send(json.dumps(bridge.state()))
                        now = time.monotonic()
                        bridge.drive_watchdog()
                        if now - last_map > 2.0:
                            last_map = now
                            await loop.run_in_executor(None, bridge.upload_map)
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
                            await ws.send(json.dumps(payload))
                        if now - last_cloud > 4.0:
                            last_cloud = now
                            await loop.run_in_executor(None, bridge.upload_cloud)
                        if now - last_camera > 0.2:
                            last_camera = now
                            await loop.run_in_executor(None, bridge.upload_camera)
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
                        await asyncio.sleep(0.2)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            print(f"[adapter_sim] {bridge.id} disconnected ({exc}); retry in 2s")
            await asyncio.sleep(2)


async def main_async(bridges: list[RobotBridge], ws_url: str) -> None:
    await asyncio.gather(*(run_robot(b, ws_url) for b in bridges))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=None,
                    help="override the persisted fleet count")
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("swarmdeck_adapter_sim")
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
    bridges = [
        RobotBridge(node, f"{args.prefix}{i}", http_url) for i in range(robot_count)
    ]

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
