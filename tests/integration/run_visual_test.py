#!/usr/bin/env python3
"""Capture what the simulated fleet actually sees, as PNGs.

This is the check that the photorealistic path is wired end to end: it starts
ARGoS, speaks the bridge's own protocol, and writes each robot's RGB frame,
metric depth frame and lidar bird's-eye view, plus one contact-sheet
`fleet_visual_dashboard.png`.

It exists because every failure in this path is silent. A robot with no glTF
descriptor is skipped by the renderer and is simply invisible to its
neighbours; a lidar mounted below the deck reports a ring of obstacles at its
own radius; a camera whose frames never arrive publishes black. None of those
raise, and none of them are visible in a log. They are all obvious in a
picture.

    python3 tests/integration/run_visual_test.py
    python3 tests/integration/run_visual_test.py --config configs/2robot.yaml

Runs with `--no-estimator`, so it needs no Ultra-Fusion sidecar: sensor frames
do not depend on the odometry source. Requires numpy and pillow.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
SIM = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim"
SCENARIO = SIM / "scenario"

OBSERVATION_MAGIC = b"SDB2"
COMMAND_MAGIC = b"SDCMD"
LIDAR_READING = 19  # range f32, x f32, y f32, z f32, ring u16, hit u8


def recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks, got = [], 0
    while got < count:
        chunk = sock.recv(min(65536, count - got))
        if not chunk:
            raise EOFError("bridge socket closed")
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks)


def colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Metric depth to a turbo-ish ramp, with no-return pixels left black."""
    valid = np.isfinite(depth) & (depth > 0.05) & (depth < 39.0)
    if not valid.any():
        return np.zeros(depth.shape + (3,), dtype=np.uint8)
    lo, hi = np.percentile(depth[valid], 2), np.percentile(depth[valid], 98)
    if hi <= lo:
        hi = lo + 1.0
    t = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    rgb = np.stack([
        np.clip(1.5 - np.abs(t * 4.0 - 3.0), 0.0, 1.0),
        np.clip(1.5 - np.abs(t * 4.0 - 2.0), 0.0, 1.0),
        np.clip(1.5 - np.abs(t * 4.0 - 1.0), 0.0, 1.0),
    ], axis=-1)
    rgb[~valid] = 0.0
    return (rgb * 255.0).astype(np.uint8)


def render_lidar_bev(points, width=320, height=240, extent_m=15.0) -> np.ndarray:
    """Top-down view of one scan, coloured by height, with range rings."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cx, cy = width // 2, height // 2
    px_per_m = (height / 2.0) / extent_m
    yy, xx = np.ogrid[:height, :width]
    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    for ring_m in (2, 5, 10):
        img[np.abs(dist - ring_m * px_per_m) < 1.0] = (35, 40, 50)
    for x, y, z in points:
        px = int(cx - y * px_per_m)
        py = int(cy - x * px_per_m)
        if 0 <= px < width and 0 <= py < height:
            t = float(np.clip((z + 0.5) / 2.5, 0.0, 1.0))
            img[py, px] = (int(255 * (1 - t)), int(255 * t), 255)
    img[cy - 2:cy + 3, cx - 2:cx + 3] = (0, 255, 0)
    return img


def capture(sock_path: str, ticks: int):
    """Serve the bridge socket for `ticks` exchanges, keeping the last frames."""
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(sock_path)
    server.listen(1)
    server.settimeout(240)
    print(f"[visual] waiting for ARGoS on {sock_path}")
    client, _ = server.accept()
    print("[visual] ARGoS connected")

    rgbs, depths, scans, poses = {}, {}, {}, {}
    started = time.time()
    sim_seconds = 0.0
    for _ in range(ticks):
        magic, tick, tps, count = struct.unpack("<4sIII", recv_exact(client, 16))
        if magic != OBSERVATION_MAGIC:
            raise RuntimeError(f"bad observation magic {magic!r}; the loop "
                               f"function and this script disagree on the "
                               f"protocol version")
        sim_seconds = tick / float(tps)
        ids = []
        for _r in range(count):
            rid = recv_exact(client, struct.unpack("<B", recv_exact(client, 1))[0])
            rid = rid.decode()
            ids.append(rid)
            gt = struct.unpack("<13d", recv_exact(client, 13 * 8))
            poses[rid] = gt[:3]
            if struct.unpack("<B", recv_exact(client, 1))[0]:      # odometry
                recv_exact(client, 1 + 13 * 8 + 4)
            if struct.unpack("<B", recv_exact(client, 1))[0]:      # encoders
                recv_exact(client, 4 * 8)
            if struct.unpack("<B", recv_exact(client, 1))[0]:      # imu
                recv_exact(client, 6 * 8)
            if struct.unpack("<B", recv_exact(client, 1))[0]:      # lidar
                _t, _rings, _az, _max, n = struct.unpack(
                    "<IIIfI", recv_exact(client, 20))
                raw = recv_exact(client, n * LIDAR_READING)
                pts = []
                for i in range(n):
                    off = i * LIDAR_READING
                    _rng, x, y, z, _ring, hit = struct.unpack(
                        "<ffffHB", raw[off:off + LIDAR_READING])
                    if hit:
                        pts.append((x, y, z))
                scans[rid] = pts
            if struct.unpack("<B", recv_exact(client, 1))[0]:      # camera
                _t, w, h, _fov = struct.unpack("<IIIf", recv_exact(client, 16))
                rgb = np.frombuffer(recv_exact(client, w * h * 3),
                                    dtype=np.uint8).reshape((h, w, 3))
                rgbs[rid] = rgb
                if struct.unpack("<B", recv_exact(client, 1))[0]:
                    depths[rid] = np.frombuffer(
                        recv_exact(client, w * h * 4),
                        dtype=np.float32).reshape((h, w))

        # Drive gently so the frames are not all of the spawn pose.
        out = bytearray(COMMAND_MAGIC + struct.pack("<II", tick, len(ids)))
        for rid in ids:
            b = rid.encode()
            out += struct.pack("<B", len(b)) + b
            out += struct.pack("<ffB", 0.30, 0.15, 0)
        out += struct.pack("<B", 0)
        client.sendall(out)

    wall = time.time() - started
    print(f"[visual] {ticks} exchanges, {sim_seconds:.1f}s simulated in "
          f"{wall:.1f}s wall ({sim_seconds / wall:.2f}x real time)")
    client.close()
    server.close()
    return rgbs, depths, scans, poses


def write_outputs(outdir: Path, rgbs, depths, scans, poses) -> int:
    outdir.mkdir(parents=True, exist_ok=True)
    ids = sorted(set(rgbs) | set(depths) | set(scans))
    if not ids:
        print("[visual] FAIL: no robot produced any sensor data", file=sys.stderr)
        return 1

    tiles, failures = [], []
    for rid in ids:
        row = []
        rgb = rgbs.get(rid)
        if rgb is None:
            failures.append(f"{rid}: no camera frame")
        else:
            Image.fromarray(rgb).save(outdir / f"{rid}_rgb.png")
            # An all-black frame is what an unlit scene, a camera inside the
            # chassis and a renderer that never ran all look like.
            if int(rgb.max()) < 8:
                failures.append(f"{rid}: camera frame is black "
                                f"(max channel {int(rgb.max())})")
            row.append(rgb)

        depth = depths.get(rid)
        if depth is None:
            failures.append(f"{rid}: no depth frame")
        else:
            col = colorize_depth(depth)
            Image.fromarray(col).save(outdir / f"{rid}_depth.png")
            row.append(col)

        pts = scans.get(rid)
        if not pts:
            failures.append(f"{rid}: lidar returned no hits")
        else:
            bev = render_lidar_bev(pts)
            Image.fromarray(bev).save(outdir / f"{rid}_lidar.png")
            row.append(bev)
            near = sum(1 for x, y, _z in pts if math.hypot(x, y) < 0.8)
            # A ring of returns at the robot's own radius is the signature of
            # a lidar mounted low enough to see its own deck. It presents as a
            # robot that has "found nowhere to go" and sits still.
            if near > len(pts) * 0.25:
                failures.append(f"{rid}: {near}/{len(pts)} lidar hits within "
                                f"0.8 m, which looks like self-intersection")

        if row:
            height = max(t.shape[0] for t in row)
            width = sum(t.shape[1] for t in row)
            strip = np.zeros((height, width, 3), dtype=np.uint8)
            x = 0
            for tile in row:
                strip[:tile.shape[0], x:x + tile.shape[1]] = tile
                x += tile.shape[1]
            tiles.append(strip)

    if tiles:
        width = max(t.shape[1] for t in tiles)
        board = np.zeros((sum(t.shape[0] for t in tiles), width, 3),
                         dtype=np.uint8)
        y = 0
        for tile in tiles:
            board[y:y + tile.shape[0], :tile.shape[1]] = tile
            y += tile.shape[0]
        Image.fromarray(board).save(outdir / "fleet_visual_dashboard.png")
        print(f"[visual] wrote {outdir / 'fleet_visual_dashboard.png'}")

    for rid in ids:
        if rid in poses:
            x, y, z = poses[rid]
            print(f"[visual] {rid} at ({x:+.2f}, {y:+.2f}, {z:+.2f}) "
                  f"lidar_hits={len(scans.get(rid, []))}")

    if failures:
        print("[visual] FAIL:", file=sys.stderr)
        for line in failures:
            print(f"  {line}", file=sys.stderr)
        return 1
    print("[visual] OK: every robot produced RGB, depth and lidar returns")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default="configs/4robot.yaml")
    ap.add_argument("--ticks", type=int, default=40,
                    help="Bridge exchanges to collect before stopping")
    ap.add_argument("--outdir", default="/tmp/swarmdeck_visual")
    ap.add_argument("--gui", action="store_true",
                    help="Open the interactive Filament viewer as well")
    ap.add_argument("--argos-build",
                    default=os.environ.get("SWARMDECK_ARGOS_BUILD",
                                           str(REPO / "argos" / "build")),
                    help="Directory holding libswarmdeck_argos.so")
    ap.add_argument("--argos-root",
                    default=os.environ.get("ARGOS_ROOT",
                                           str(Path.home() / "Projects" / "argos3")),
                    help="ARGoS source tree, for its build-tree plugins and "
                         "photorealism assets when it is not installed")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    rundir = outdir / "run"
    (rundir / "props").mkdir(parents=True, exist_ok=True)
    for glb in (REPO / "argos" / "assets" / "props").glob("*.glb"):
        (rundir / "props" / glb.name).write_bytes(glb.read_bytes())

    subprocess.run([sys.executable, str(SCENARIO / "make_argos_world.py"),
                    "-o", str(rundir / "indoor.gltf")], check=True)

    # Short by necessity: a Unix socket path is capped at 107 bytes, and the
    # bind fails with "AF_UNIX path too long" rather than anything descriptive.
    sock_path = f"/tmp/sd_visual_{os.getpid()}.sock"
    argos_file = rundir / "visual.argos"
    gen = [sys.executable, str(SCENARIO / "make_argos_session.py"),
           "--config", args.config, "-o", str(argos_file),
           "--socket", sock_path, "--no-estimator"]
    if args.gui:
        gen.append("--gui")
    subprocess.run(gen, check=True)

    env = os.environ.copy()
    root = Path(args.argos_root)
    plugin_dirs = [args.argos_build, "/usr/local/lib/argos3"]
    build_plugins = root / "build" / "plugins"
    if build_plugins.is_dir():
        plugin_dirs[1:1] = [
            str(build_plugins / "robots" / name)
            for name in ("bunker", "scout-mini", "spot")
        ] + [
            str(build_plugins / "simulator" / "physics_engines" / "jolt"),
            str(build_plugins / "simulator" / "photorealism"),
            str(build_plugins / "simulator" / "visualizations" / "filament"),
        ]
    env["ARGOS_PLUGIN_PATH"] = ":".join(
        d for d in plugin_dirs + [env.get("ARGOS_PLUGIN_PATH", "")] if d)
    assets = root / "src" / "plugins" / "simulator" / "photorealism" / "assets"
    if assets.is_dir():
        env.setdefault("ARGOS_PHOTOREALISM_ASSET_PATH", str(assets))
    # Headless Vulkan on a machine with no usable GPU. Harmless where there is
    # one, since the ICD is only consulted if it is the only one listed.
    if not args.gui and "VK_DRIVER_FILES" not in env:
        lavapipe = Path("/usr/share/vulkan/icd.d/lvp_icd.json")
        if lavapipe.exists():
            env["VK_DRIVER_FILES"] = str(lavapipe)

    print(f"[visual] argos3 -c {argos_file}")
    proc = subprocess.Popen(["argos3", "-c", str(argos_file)],
                            cwd=str(rundir), env=env)
    try:
        rgbs, depths, scans, poses = capture(sock_path, args.ticks)
        return write_outputs(outdir, rgbs, depths, scans, poses)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


if __name__ == "__main__":
    sys.exit(main())
