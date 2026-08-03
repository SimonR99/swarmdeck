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
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

# Keep perception reusable by real adapters without packaging it into ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.perception.duck_detector import RubberDuckDetector


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class RobotBridge:
    """Per-robot ROS subscriptions and command publisher."""

    def __init__(self, node: Node, robot_id: str, http_url: str) -> None:
        self.node = node
        self.id = robot_id
        self.http_url = http_url
        self.t0 = time.monotonic()

        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.goal: dict | None = None
        self.planned_path: list[dict[str, float]] = []
        self.nav_status = "idle"
        self.mode = "idle"
        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False
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
        p = msg.pose.pose
        self.pose = {"x": p.position.x, "y": p.position.y, "yaw": yaw_of(p.orientation)}

    def _on_tf(self, msg: TFMessage) -> None:
        """Track SLAM's map->odom correction from the namespaced TF topic."""
        map_frame = f"{self.id}/map_frame"
        odom_frame = f"{self.id}/odom"
        for stamped in msg.transforms:
            if stamped.header.frame_id == map_frame and stamped.child_frame_id == odom_frame:
                t = stamped.transform
                self._map_to_odom = {
                    "x": t.translation.x,
                    "y": t.translation.y,
                    "yaw": yaw_of(t.rotation),
                }

    def map_pose(self) -> dict[str, float]:
        """Compose map->odom with odom->base so the icon matches the SLAM grid."""
        tf = self._map_to_odom
        c, s = math.cos(tf["yaw"]), math.sin(tf["yaw"])
        return {
            "x": tf["x"] + self.pose["x"] * c - self.pose["y"] * s,
            "y": tf["y"] + self.pose["x"] * s + self.pose["y"] * c,
            "yaw": (tf["yaw"] + self.pose["yaw"] + math.pi) % (2 * math.pi) - math.pi,
        }

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        self._grid_dirty = True

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
            "protocol": 1,
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
