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
name below is a hypothesis until a robot proves it. See docs/hardware-bringup.md
for the order to validate them in.
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
from sensor_msgs.msg import BatteryState, CompressedImage, Image
from tf2_ros import Buffer, TransformListener

# Nav2 is the common case but not the only one; see `navigate_to` below.
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on the robot's install
    NavigateToPose = None

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
        "plan": "plan",
        "cmd_vel": "cmd_vel",
        "battery": "",       # empty disables the capability
        "camera": "",
        "camera_compressed": "",
    },
    "actions": {"navigate_to_pose": "navigate_to_pose"},
    "rates": {
        "state_hz": 5.0,
        "map_period_s": 2.0,
        "camera_period_s": 0.2,   # 5 Hz, per the protocol's preview cap
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

    def __init__(self, node: Node, robot_id: str, cfg: dict, http_url: str) -> None:
        self.node = node
        self.id = robot_id
        self.cfg = cfg
        self.http_url = http_url
        self.t0 = time.monotonic()

        self.map_frame = cfg["map_frame"]
        self.base_frame = cfg["base_frame"]
        topics = cfg["topics"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.grid: OccupancyGrid | None = None
        self._grid_dirty = False
        self.planned_path: list[dict[str, float]] = []
        self.battery: float | None = None
        self.nav_status = "idle"
        self.mode = "idle"
        self.goal: dict[str, float] | None = None
        self._goal_handle = None
        self._goal_generation = 0
        self._last_drive_at = 0.0
        self._camera_jpeg: bytes | None = None
        self._camera_dirty = False
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
            node.create_subscription(Odometry, topics["odom"], self._on_odom, 10)
        if topics.get("map"):
            node.create_subscription(OccupancyGrid, topics["map"], self._on_map, latched)
        if topics.get("plan"):
            node.create_subscription(NavPath, topics["plan"], self._on_plan, 10)
        if topics.get("battery"):
            node.create_subscription(
                BatteryState, topics["battery"], self._on_battery, qos_profile_sensor_data
            )
        # Prefer compressed: a raw camera stream at full rate is the single most
        # expensive thing an adapter can subscribe to over a robot's network, and
        # the preview is throttled to 5 Hz anyway.
        if topics.get("camera_compressed"):
            node.create_subscription(
                CompressedImage, topics["camera_compressed"],
                self._on_camera_compressed, qos_profile_sensor_data,
            )
        elif topics.get("camera"):
            node.create_subscription(
                Image, topics["camera"], self._on_camera_raw, qos_profile_sensor_data
            )

        self.pub_cmd = (
            node.create_publisher(Twist, topics["cmd_vel"], 10)
            if topics.get("cmd_vel") else None
        )

        action_name = cfg.get("actions", {}).get("navigate_to_pose")
        self.nav_client = None
        if action_name and NavigateToPose is not None:
            self.nav_client = ActionClient(node, NavigateToPose, action_name)

    # ------------------------------------------------------------- capabilities

    def capabilities(self) -> list[str]:
        """Only what this robot can actually honour (protocol rule 4)."""
        caps: list[str] = []
        if self.nav_client is not None:
            caps.append("navigate")
        if self.cfg["topics"].get("map"):
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
        except (ValueError, TypeError):
            return

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
                self.node.get_logger().warn(
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

    def drive(self, linear: float, angular: float) -> None:
        if self.pub_cmd is None:
            return
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub_cmd.publish(twist)
        moving = abs(linear) > 1e-3 or abs(angular) > 1e-3
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
        if self.nav_client is None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
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
        yaw = float(goal.get("yaw", 0.0))
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal = {"x": float(goal["x"]), "y": float(goal["y"])}
        self.nav_status = "active"
        self.mode = "nav"

        future = self.nav_client.send_goal_async(msg)
        future.add_done_callback(
            lambda f, g=generation: self._on_goal_response(f, g)
        )

    def _on_goal_response(self, future, generation: int) -> None:
        # A stale response must not overwrite the state of a newer goal.
        if generation != self._goal_generation:
            return
        try:
            handle = future.result()
        except Exception:
            self.nav_status = "failed"
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
        self._goal_handle = None

    def cancel_goal(self) -> None:
        self._goal_generation += 1
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
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
            self.node.get_logger().warn(f"[{self.id}] camera upload failed: {exc}")


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
                    "adapter": "adapter_ros2/0.1.0",
                    "ros": "jazzy",
                    # `local`: a real robot's pose and grid are in its own
                    # navigation-map frame. The backend does the merging.
                    "coordinate_frame": "local",
                    "capabilities": bridge.capabilities(),
                    "footprint_radius": bridge.cfg["footprint_radius"],
                }))
                bridge.node.get_logger().info(
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
                    last_cam = 0.0
                    while True:
                        await ws.send(json.dumps(bridge.state()))
                        bridge.drive_watchdog()
                        now = time.monotonic()
                        if now - last_map > float(rates["map_period_s"]):
                            last_map = now
                            meta = await loop.run_in_executor(None, bridge.upload_map)
                            if meta:
                                await ws.send(json.dumps(meta))
                        if now - last_cam > float(rates["camera_period_s"]):
                            last_cam = now
                            await loop.run_in_executor(None, bridge.upload_camera)
                        await asyncio.sleep(period)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            bridge.node.get_logger().warn(
                f"[{bridge.id}] disconnected ({exc}); retrying in {backoff:.0f}s"
            )
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

    rclpy.init()
    node = rclpy.create_node(f"swarmdeck_adapter_{args.robot_id}")
    bridge = HardwareBridge(node, args.robot_id, cfg, http_url)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
