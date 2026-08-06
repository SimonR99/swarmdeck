#!/usr/bin/env python3
"""Mock adapter — synthetic robots, zero ROS.

Proves the adapter contract and lets the whole stack run on any machine:

    python3 mock_adapter.py --robots 4

Requires only `websockets` and `numpy`.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import time
import urllib.request
import zlib

import numpy as np
import websockets

RES = 0.05
N = 800
ORIGIN = (-20.0, -20.0)
REVEAL = 45


def build_truth() -> np.ndarray:
    t = np.zeros((N, N), dtype=np.int8)

    def wall(x0, y0, x1, y1):
        t[y0:y1, x0:x1] = 100

    wall(60, 60, 740, 68)
    wall(60, 732, 740, 740)
    wall(60, 60, 68, 740)
    wall(732, 60, 740, 740)
    wall(250, 60, 258, 320)
    wall(250, 440, 258, 740)
    wall(500, 60, 508, 260)
    wall(500, 380, 508, 740)
    wall(258, 380, 500, 388)
    wall(508, 500, 740, 508)
    return t


class MockRobot:
    def __init__(self, idx: int, host: str, fleet_size: int = 1) -> None:
        self.id = f"robot_{idx}"
        # Who else is out there, for the synthetic pose graph in slam_graph().
        self.peers = [f"robot_{i}" for i in range(fleet_size) if i != idx]
        self.type = "spot" if idx == 0 else "diffdrive"
        starts = [(-14, -14), (10, -14), (-14, 10), (10, 10), (0, 0)]
        self.x, self.y = starts[idx % 5]
        self.yaw = random.uniform(0, math.tau)
        self.target: dict | None = None
        self.battery = random.uniform(0.7, 1.0)
        self.mode = "idle"
        self.nav_status = "idle"
        self.host = host
        self.truth = build_truth()
        self.known = np.full((N, N), -1, dtype=np.int8)
        self.t0 = time.monotonic()

    def reveal(self) -> None:
        cx = int((self.x - ORIGIN[0]) / RES)
        cy = int(N - (self.y - ORIGIN[1]) / RES)
        y0, y1 = max(0, cy - REVEAL), min(N, cy + REVEAL)
        x0, x1 = max(0, cx - REVEAL), min(N, cx + REVEAL)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= REVEAL**2
        region = self.known[y0:y1, x0:x1]
        region[mask] = self.truth[y0:y1, x0:x1][mask]

    def step(self, dt: float) -> None:
        if self.mode == "estop":
            return
        if not self.target:
            if random.random() < 0.02:
                self.target = {"x": random.uniform(-16, 16), "y": random.uniform(-16, 16)}
                self.nav_status, self.mode = "active", "nav"
        else:
            dx, dy = self.target["x"] - self.x, self.target["y"] - self.y
            d = math.hypot(dx, dy)
            if d < 0.25:
                self.target = None
                self.nav_status, self.mode = "succeeded", "idle"
            else:
                self.yaw = math.atan2(dy, dx)
                self.x += dx / d * 1.1 * dt
                self.y += dy / d * 1.1 * dt
        self.battery = max(0.05, self.battery - dt * 0.0004)
        self.reveal()

    def state(self) -> dict:
        return {
            "type": "robot_state",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 4)},
            "battery": round(self.battery, 3),
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.target,
        }

    def slam_graph(self) -> dict:
        """A synthetic collaborative pose graph (protocol 2, optional).

        Real swarm SLAM reports this from its own back end. Here it is modelled
        just faithfully enough to drive the GUI: keyframes accumulate with time,
        a robot joins the common frame only once it has closed a loop with
        someone, and closures against near neighbours accrue faster than against
        distant ones — because meeting is what produces them.
        """
        elapsed = time.monotonic() - self.t0
        links = []
        for other in self.peers:
            # Neighbours in the fleet ordering stand in for spatial proximity.
            gap = abs(int(other.split("_")[-1]) - int(self.id.split("_")[-1]))
            count = max(0, int(elapsed / (12.0 * gap)) if gap else 0)
            if count:
                links.append({"other": other, "count": count, "last_t": round(elapsed, 1)})
        return {
            "type": "slam_graph",
            "robot_id": self.id,
            "t_mono": round(elapsed, 4),
            "keyframes": int(elapsed * 1.5),
            "in_common_frame": bool(links),
            "residual": round(0.12 / (1 + len(links)), 4),
            "inter_robot": links,
        }

    def upload_cloud(self) -> None:
        """A synthetic 3D cloud: this robot's known walls, extruded upward.

        Real robots send an accumulated 3D map from their SLAM back end. This
        stands in for it so the GUI's 3D view can be developed and demonstrated
        with no ROS at all, exactly as the synthetic grid does for the 2D map.
        """
        ys, xs = np.nonzero(self.known >= 50)
        if not len(xs):
            return
        # One column of points per occupied cell, so walls read as walls.
        heights = np.arange(0.1, 1.9, 0.2, dtype=np.float32)
        wx = (xs * RES + ORIGIN[0]).astype(np.float32)
        wy = ((N - ys) * RES + ORIGIN[1]).astype(np.float32)
        points = np.column_stack(
            [
                np.repeat(wx, len(heights)),
                np.repeat(wy, len(heights)),
                np.tile(heights, len(wx)),
            ]
        )
        body = zlib.compress(np.round(points / 0.01).astype(np.int16).tobytes(), 1)
        req = urllib.request.Request(
            f"{self.host}/api/adapter/cloud?robot_id={self.id}&scale=0.01",
            data=body,
            headers={"Content-Type": "application/octet-stream"},
        )
        try:
            urllib.request.urlopen(req, timeout=5).read()
        except Exception as exc:
            print(f"[{self.id}] cloud upload failed: {exc}")

    def upload_map(self) -> None:
        body = zlib.compress(np.ascontiguousarray(self.known).tobytes())
        url = (
            f"{self.host}/api/adapter/map?robot_id={self.id}&resolution={RES}"
            f"&width={N}&height={N}&origin_x={ORIGIN[0]}&origin_y={ORIGIN[1]}"
        )
        req = urllib.request.Request(
            url, data=body, headers={"Content-Type": "application/octet-stream"}
        )
        try:
            urllib.request.urlopen(req, timeout=5).read()
        except Exception as exc:
            print(f"[{self.id}] map upload failed: {exc}")


async def run_robot(robot: MockRobot, ws_url: str, http_url: str) -> None:
    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                await ws.send(
                    json.dumps(
                        {
                            "type": "hello",
                            "protocol": 2,
                            "robot_id": robot.id,
                            "robot_type": robot.type,
                            "adapter": "adapter_mock/0.1.0",
                            "ros": "none",
                            # Synthetic poses and maps already share one global frame.
                            "coordinate_frame": "merged",
                            "capabilities": ["navigate", "map", "camera", "battery", "estop"],
                            "footprint_radius": 0.35,
                        }
                    )
                )
                print(f"[{robot.id}] connected")

                async def rx() -> None:
                    async for raw in ws:
                        msg = json.loads(raw)
                        t = msg.get("type")
                        if t == "navigate_to":
                            robot.target = msg["goal"]
                            robot.nav_status, robot.mode = "active", "nav"
                        elif t == "cancel_goal":
                            robot.target = None
                            robot.nav_status, robot.mode = "cancelled", "idle"
                        elif t == "stop":
                            robot.target = None
                            robot.nav_status, robot.mode = "idle", "estop"

                async def tx() -> None:
                    last_map = 0.0
                    last_graph = 0.0
                    last_cloud = 0.0
                    while True:
                        robot.step(0.2)
                        await ws.send(json.dumps(robot.state()))
                        if random.random() < 0.01:
                            await ws.send(
                                json.dumps(
                                    {
                                        "type": "detections",
                                        "robot_id": robot.id,
                                        "camera": "front",
                                        "t_mono": time.monotonic() - robot.t0,
                                        "items": [
                                            {
                                                "class": "duck",
                                                "score": round(random.uniform(0.7, 0.98), 3),
                                                "bbox": [0.3, 0.3, 0.16, 0.22],
                                                "map_position": {
                                                    "x": robot.x + random.uniform(-2, 2),
                                                    "y": robot.y + random.uniform(-2, 2),
                                                },
                                            }
                                        ],
                                    }
                                )
                            )
                        now = time.monotonic()
                        # A synthetic collaborative pose graph (protocol 2).
                        # Real swarm SLAM reports this from its own back end;
                        # here it exists so the GUI's swarm panel and the
                        # backend's `merge_mode: cslam` can be exercised without
                        # a ROS 2 fleet, exactly as the rest of this adapter
                        # exercises the map and detection paths.
                        if now - last_graph > 3.0:
                            last_graph = now
                            await ws.send(json.dumps(robot.slam_graph()))
                        if now - last_map > 2.0:
                            last_map = now
                            await asyncio.get_running_loop().run_in_executor(
                                None, robot.upload_map
                            )
                        if now - last_cloud > 4.0:
                            last_cloud = now
                            await asyncio.get_running_loop().run_in_executor(
                                None, robot.upload_cloud
                            )
                        await asyncio.sleep(0.2)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            print(f"[{robot.id}] disconnected ({exc}); retrying in 2s")
            await asyncio.sleep(2)


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=4)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    ws_url = f"ws://{args.host}:{args.port}/adapter"
    http_url = f"http://{args.host}:{args.port}"
    count = min(args.robots, 5)
    robots = [MockRobot(i, http_url, count) for i in range(count)]
    print(f"[adapter_mock] {len(robots)} robots -> {ws_url}")
    await asyncio.gather(*(run_robot(r, ws_url, http_url) for r in robots))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
