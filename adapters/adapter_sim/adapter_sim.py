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
import threading
import time
import urllib.request
import zlib

import numpy as np
import rclpy
import websockets
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


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
        self.goal: dict | None = None
        self.nav_status = "idle"
        self.mode = "idle"
        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False

        node.create_subscription(Odometry, f"/{robot_id}/odom", self._on_odom, 10)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, f"/{robot_id}/map", self._on_map, latched)

        self.pub_goal = node.create_publisher(PoseStamped, f"/{robot_id}/goal_pose", 10)
        self.pub_cmd = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = {"x": p.position.x, "y": p.position.y, "yaw": yaw_of(p.orientation)}

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        self._grid_dirty = True

    # -- protocol side -------------------------------------------------

    def hello(self) -> dict:
        return {
            "type": "hello",
            "protocol": 1,
            "robot_id": self.id,
            "robot_type": "diffdrive",
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
            "pose": self.pose,
            "battery": None,  # simulation has no battery model
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
        }

    def navigate_to(self, goal: dict) -> None:
        """Planner-agnostic command -> Nav2 goal_pose (FR-N8)."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(goal["x"])
        msg.pose.position.y = float(goal["y"])
        yaw = float(goal.get("yaw", 0.0))
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.pub_goal.publish(msg)
        self.goal = {"x": goal["x"], "y": goal["y"]}
        self.nav_status, self.mode = "active", "nav"

    def stop(self) -> None:
        self.pub_cmd.publish(Twist())
        self.goal = None
        self.nav_status, self.mode = "idle", "estop"

    def cancel(self) -> None:
        self.pub_cmd.publish(Twist())
        self.goal = None
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

                async def tx() -> None:
                    last_map = 0.0
                    loop = asyncio.get_running_loop()
                    while True:
                        await ws.send(json.dumps(bridge.state()))
                        now = time.monotonic()
                        if now - last_map > 2.0:
                            last_map = now
                            await loop.run_in_executor(None, bridge.upload_map)
                        await asyncio.sleep(0.2)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            print(f"[adapter_sim] {bridge.id} disconnected ({exc}); retry in 2s")
            await asyncio.sleep(2)


async def main_async(bridges: list[RobotBridge], ws_url: str) -> None:
    await asyncio.gather(*(run_robot(b, ws_url) for b in bridges))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=4)
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("swarmdeck_adapter_sim")
    http_url = f"http://{args.host}:{args.port}"
    bridges = [
        RobotBridge(node, f"{args.prefix}{i}", http_url) for i in range(min(args.robots, 5))
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
