#!/usr/bin/env python3
"""ROS-free microbenchmarks; run with server/.venv/bin/python scripts/benchmark-sim.py."""

from pathlib import Path
import asyncio
import json
import statistics
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "server"))
sys.path.insert(0, str(ROOT / "deploy/docker/fast_livo2"))
sys.path.insert(0, str(ROOT / "swarmdeck_ros/src/swarmdeck_sim/nodes"))
from adapters.runtime import unique_row_index
from fast_livo_link import png_encode_rgb
from swarmdeck_argos_bridge import (
    LIDAR_DTYPE,
    project_laserscan_proximity,
    project_laserscan_slice,
    proximity_spec,
)
from swarmdeck_server.api.broadcast import JsonBroadcaster


def measure(label, operation, repeats=30):
    operation()  # Warm imports/allocators before measuring.
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        operation()
        samples.append((time.perf_counter() - start) * 1000)
    print(f"{label:36s} median {statistics.median(samples):8.3f} ms")


async def broadcast_benchmark():
    """Compare JSON fan-out work with in-memory consumers, without network I/O."""

    class Client:
        async def send_text(self, text):
            pass

    clients = {Client() for _ in range(4)}
    broadcaster = JsonBroadcaster(clients)
    message = {
        "type": "robot_state",
        "global_planned_path": [
            {"x": i * 0.05, "y": i * 0.025, "z": 0.2} for i in range(2000)
        ],
    }

    async def per_client_encoding():
        for client in clients:
            await client.send_text(
                json.dumps(message, separators=(",", ":"), ensure_ascii=False)
            )

    for label, operation in (
        ("4 clients: per-client JSON", per_client_encoding),
        ("4 clients: shared JSON", lambda: broadcaster.publish(message)),
    ):
        await operation()
        samples = []
        for _ in range(30):
            start = time.perf_counter()
            await operation()
            samples.append((time.perf_counter() - start) * 1000)
        print(f"{label:36s} median {statistics.median(samples):8.3f} ms")


def main():
    rng = np.random.default_rng(42)
    points = rng.uniform(-20, 20, (32000, 3)).astype(np.float32)
    keys = np.round(points / 0.1).astype(np.int32)
    old = lambda: np.unique(keys, axis=0, return_index=True)[1]
    packed = lambda: unique_row_index(keys)
    np.testing.assert_array_equal(old(), packed())
    measure("32k voxels: structured unique", old)
    measure("32k voxels: packed unique", packed)
    hits = np.zeros(len(points), dtype=LIDAR_DTYPE)
    for i, axis in enumerate(("x", "y", "z")):
        hits[axis] = points[:, i]
    projection = proximity_spec("scout_mini")
    measure(
        "32k hits: planar + proximity scans",
        lambda: (
            project_laserscan_slice(hits, range_max=30.0),
            project_laserscan_proximity(hits, **projection),
        ),
    )
    rgb = rng.integers(0, 256, (240, 320, 3), dtype=np.uint8).tobytes()
    measure("320x240 noisy RGB: PNG encode", lambda: png_encode_rgb(320, 240, rgb))
    asyncio.run(broadcast_benchmark())
    print("PNG encoding is skipped entirely with no compressed-image subscribers.")
    print(
        "Synthetic CPU timings; these do not measure ROS latency, map accuracy, or FPS."
    )


if __name__ == "__main__":
    main()
