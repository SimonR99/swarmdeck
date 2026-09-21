#!/usr/bin/env python3
"""ROS-free adapter with synthetic replica geometry."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import struct
import time
import urllib.request
import uuid
from dataclasses import asdict

import numpy as np
import websockets

from autonomy.contracts import (
    ChunkRef,
    ComponentRevision,
    IDENTITY_SE3,
    KeyframeId,
    MapManifest,
    MapSnapshot,
    SubmapId,
    SubmapRevision,
)
from autonomy.map_epochs import robot_run_id
from autonomy.mapping import XYZ_F32_ENCODING
from autonomy.replication import ReplicaClient
from adapters.runtime import hello_message


def _points() -> np.ndarray:
    rows = []
    for x in np.arange(-14.0, 14.01, 0.08):
        rows.extend(((x, -14.0, z) for z in np.arange(0.1, 1.9, 0.16)))
        rows.extend(((x, 14.0, z) for z in np.arange(0.1, 1.9, 0.16)))
    for y in np.arange(-14.0, 14.01, 0.08):
        rows.extend(((-14.0, y, z) for z in np.arange(0.1, 1.9, 0.16)))
        rows.extend(((14.0, y, z) for z in np.arange(0.1, 1.9, 0.16)))
    for x0, x1, y0 in ((-5.0, -4.8, 0.0), (4.0, 4.2, -4.0)):
        for x in np.arange(x0, x1, 0.08):
            rows.extend(((x, y0, z) for z in np.arange(0.1, 1.9, 0.16)))
    return np.asarray(rows, dtype="<f4")


def _chunk(points: np.ndarray) -> bytes:
    return b"SDXYZ1\x00\x00" + struct.pack("<Q", len(points)) + points.tobytes()


class MockRobot:
    def __init__(self, idx: int, host: str, fleet_size: int = 1) -> None:
        self.id = f"robot_{idx}"
        self.peers = [f"robot_{i}" for i in range(fleet_size) if i != idx]
        self.type = "spot" if idx == 0 else "diffdrive"
        starts = [(-10, -10), (8, -10), (-10, 8), (8, 8), (0, 0)]
        self.x, self.y = starts[idx % len(starts)]
        self.yaw = 0.0
        self.target: dict | None = None
        self.battery = 0.9
        self.mode = "idle"
        self.nav_status = "idle"
        self.host = host
        self.t0 = time.monotonic()
        self.session_id = str(
            uuid.uuid5(uuid.NAMESPACE_URL, f"swarmdeck-mock:{self.id}")
        )
        self.revision = 0
        self.points = _points()
        payload = _chunk(self.points)
        self.chunk_data = payload
        self.chunk_sha256 = hashlib.sha256(payload).hexdigest()
        self.replica = ReplicaClient(host)

    def step(self, dt: float) -> None:
        if self.target:
            dx, dy = self.target["x"] - self.x, self.target["y"] - self.y
            distance = math.hypot(dx, dy)
            if distance < 0.25:
                self.target = None
                self.nav_status, self.mode = "succeeded", "idle"
            else:
                self.yaw = math.atan2(dy, dx)
                self.x += dx / distance * 1.1 * dt
                self.y += dy / distance * 1.1 * dt
        self.battery = max(0.05, self.battery - dt * 0.0004)

    def state(self) -> dict:
        quality = max(4.0, min(100.0, 98.0 - math.hypot(self.x - 3, self.y + 2) * 3.2))
        return {
            "type": "robot_state",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "pose": {
                "x": round(self.x, 3),
                "y": round(self.y, 3),
                "yaw": round(self.yaw, 4),
            },
            "battery": round(self.battery, 3),
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.target,
            "live_mapping": {
                "navigation_frame": f"{self.id}/odom",
                "planning_frame": f"{self.id}/odom",
                "map_epoch": 0,
                "mapping_graph_revision": self.revision,
                "geometry_revision": self.chunk_sha256,
                "component_id": f"component:{self.id}",
                "mission_id": self.session_id,
                "run_id": robot_run_id(self.session_id, self.id, 0),
                "robot_id": self.id,
                "T_component_navigation": IDENTITY_SE3,
            },
            "network": {
                "interface": "mock-wlan0",
                "quality_pct": round(quality, 1),
                "rssi_dbm": round(-90 + quality * 0.4, 1),
                "ssid": "SwarmFleet-AP1",
                "ping_ms": round(8 + (100 - quality) * 0.8, 1),
            },
        }

    def slam_graph(self) -> dict:
        elapsed = time.monotonic() - self.t0
        return {
            "type": "slam_graph",
            "robot_id": self.id,
            "t_mono": round(elapsed, 4),
            "keyframes": int(elapsed * 1.5),
            "in_common_frame": bool(self.peers),
            "residual": 0.04,
            "inter_robot": self.peers,
        }

    def replica_envelope(self) -> dict:
        revision = ComponentRevision(f"component:{self.id}", 0, self.revision)
        keyframe = KeyframeId(self.id, robot_run_id(self.session_id, self.id, 0), 0)
        submap_id = SubmapId(self.id, self.session_id, 0)
        bounds = (
            (
                float(self.points[:, 0].min()),
                float(self.points[:, 1].min()),
                float(self.points[:, 2].min()),
            ),
            (
                float(self.points[:, 0].max()),
                float(self.points[:, 1].max()),
                float(self.points[:, 2].max()),
            ),
        )
        chunk = ChunkRef(
            self.chunk_sha256,
            XYZ_F32_ENCODING,
            len(self.chunk_data),
            bounds,
            len(self.points),
        )
        submap = SubmapRevision(
            submap_id,
            0,
            revision,
            IDENTITY_SE3,
            (keyframe,),
            (chunk,),
            bounds,
            0.08,
            observed_at_ns=time.time_ns(),
        )
        manifest = MapManifest(
            "onboard",
            "persistent_geometry",
            self.id,
            revision,
            self.chunk_sha256,
            (submap,),
            (chunk,),
            (),
        )
        snapshot_id = hashlib.sha256(
            json.dumps(
                [manifest.to_dict()], sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        snapshot = MapSnapshot(snapshot_id, time.time_ns(), (manifest,))
        return {
            "version": 1,
            "robot_id": self.id,
            "session_id": self.session_id,
            "map_epoch": 0,
            "run_id": robot_run_id(self.session_id, self.id, 0),
            "anchor": asdict(keyframe),
            "participant_robot_ids": [self.id],
            "robot_map_epochs": {self.id: 0},
            "revision": self.revision,
            "solution_order": [0, self.revision],
            "component_id": f"component:{self.id}",
            "chunks": [{"sha256": self.chunk_sha256, "size": len(self.chunk_data)}],
            "snapshot": snapshot.to_dict(),
        }

    def publish_replica(self) -> None:
        self.replica.sync(self.replica_envelope(), lambda digest: self.chunk_data)


async def run_robot(robot: MockRobot, ws_url: str) -> None:
    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                await ws.send(
                    json.dumps(
                        hello_message(
                            robot_id=robot.id,
                            robot_type=robot.type,
                            adapter="adapter_mock/0.1.0",
                            ros="none",
                            capabilities=[
                                "plan_objective",
                                "camera",
                                "battery",
                                "network",
                                "estop",
                            ],
                            footprint_radius=0.35,
                            coordinate_frame="local",
                        )
                    )
                )

                async def rx() -> None:
                    async for raw in ws:
                        msg = json.loads(raw)
                        if (
                            msg.get("type") == "plan_objective"
                            and msg.get("objective") == "navigate"
                        ):
                            robot.target = msg.get("goal") or {}
                            robot.nav_status, robot.mode = "active", "nav"
                        elif msg.get("type") in {"cancel_goal", "stop"}:
                            robot.target = None
                            robot.nav_status, robot.mode = "cancelled", "idle"

                async def tx() -> None:
                    last_replica = 0.0
                    last_graph = 0.0
                    while True:
                        robot.step(0.2)
                        await ws.send(json.dumps(robot.state()))
                        now = time.monotonic()
                        if now - last_graph > 3:
                            last_graph = now
                            await ws.send(json.dumps(robot.slam_graph()))
                        if now - last_replica > 5:
                            last_replica = now
                            await asyncio.get_running_loop().run_in_executor(
                                None, robot.publish_replica
                            )
                        await asyncio.sleep(0.2)

                await asyncio.gather(rx(), tx())
        except Exception as exc:
            print(f"[{robot.id}] disconnected ({exc}); retrying in 2s")
            await asyncio.sleep(2)


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=4)
    ap.add_argument("--host", default="http://localhost:8080")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    base = args.host if "://" in args.host else f"http://{args.host}:{args.port}"
    ws_host = base.split("://", 1)[1]
    ws_url = f"ws://{ws_host}/adapter"
    robots = [
        MockRobot(i, base, min(args.robots, 5)) for i in range(min(args.robots, 5))
    ]
    print(f"[adapter_mock] {len(robots)} robots -> {ws_url}")
    await asyncio.gather(*(run_robot(r, ws_url) for r in robots))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
