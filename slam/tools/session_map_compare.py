"""Render this session's keyframes several ways and dump comparison PNGs.

Used to iterate the optimized occupancy against the robots' own SLAM maps
without re-flying Gazebo. Reads ``sessions/captures/<dataset>/keyframes``.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
import zlib
from collections import defaultdict
from pathlib import Path

import numpy as np

from swarmdeck_protocol import decode_keyframe
from swarmdeck_slam.backend import CollaborativeBackend
from swarmdeck_slam.render import (
    FREE,
    OCCUPIED,
    RenderConfig,
    RenderedGrid,
    render_occupancy,
    render_per_robot,
)
from swarmdeck_slam.types import Component, Keyframe, OptimizedGraph, se3_from_quat_xyz

UNKNOWN_RGB = (214, 218, 224)
FREE_RGB = (255, 255, 255)
OCCUPIED_RGB = (52, 58, 68)

RENDER = RenderConfig(
    floor_z=0.0,
    min_z=0.15,
    max_z=1.80,
    native_map_resolution=0.05,
    odometry_as_pose=True,
    close_occupied=1,
    hit_weight=5,
)
POSE_PRIOR = np.array([0.05, 0.05, 0.05, 0.10, 0.10, 0.15])
SPAWN = {
    "robot_0": (-9.0, 0.0, 0.0),
    "robot_1": (-3.0, 0.0, 0.0),
    "robot_2": (3.0, 0.0, math.pi),
    "robot_3": (9.0, 0.0, math.pi),
}


def yaw_se3(x: float, y: float, yaw: float) -> np.ndarray:
    return se3_from_quat_xyz(
        [x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
    )


def write_png(path: Path, rgb: np.ndarray) -> None:
    height, width = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def grid_rgb(grid: RenderedGrid) -> np.ndarray:
    img = np.zeros((grid.height, grid.width, 3), dtype=np.uint8)
    img[...] = UNKNOWN_RGB
    img[grid.cells == FREE] = FREE_RGB
    img[grid.cells == OCCUPIED] = OCCUPIED_RGB
    return np.flipud(img)


def occupied_count(grid: RenderedGrid) -> int:
    return int((grid.cells == OCCUPIED).sum())


def load_packets(dataset: Path) -> list:
    blobs = sorted((dataset / "keyframes").glob("*.kf"))
    packets = []
    for path in blobs:
        try:
            packets.append(decode_keyframe(path.read_bytes()))
        except Exception as exc:
            print(f"skip {path.name}: {exc}", file=sys.stderr)
    return packets


def raw_graph(keyframes: list[Keyframe]) -> OptimizedGraph:
    by_traj: dict = defaultdict(list)
    for kf in keyframes:
        by_traj[kf.id.trajectory].append(kf.id)
    components = []
    for i, trajectory in enumerate(sorted(by_traj)):
        ids = by_traj[trajectory]
        components.append(
            Component(
                i,
                frozenset({trajectory.robot_id}),
                min(ids),
                frozenset({trajectory}),
            )
        )
    return OptimizedGraph(
        poses={kf.id: kf.t_odom_base for kf in keyframes},
        components=components,
    )


def spawn_aligned_graph(keyframes: list[Keyframe]) -> OptimizedGraph:
    """Upper bound: each robot's SLAM map placed at its Gazebo spawn pose."""
    frames = {
        robot_id: yaw_se3(x, y, yaw) for robot_id, (x, y, yaw) in SPAWN.items()
    }
    by_traj: dict = defaultdict(list)
    poses = {}
    traj_frames = {}
    for kf in keyframes:
        frame = frames[kf.id.robot_id]
        poses[kf.id] = frame @ kf.t_odom_base
        traj_frames[kf.id.trajectory] = frame
        by_traj[kf.id.trajectory].append(kf.id)
    anchor = min(kf.id for kf in keyframes)
    return OptimizedGraph(
        poses=poses,
        t_world_map=frames,
        t_world_trajectory=traj_frames,
        components=[
            Component(
                0,
                frozenset(frames),
                anchor,
                frozenset(by_traj),
            )
        ],
    )


def save_robot_grids(
    out: Path, prefix: str, grids: dict[str, RenderedGrid]
) -> dict[str, int]:
    out.mkdir(parents=True, exist_ok=True)
    counts = {}
    for robot_id, grid in sorted(grids.items()):
        write_png(out / f"{prefix}-{robot_id}.png", grid_rgb(grid))
        counts[robot_id] = occupied_count(grid)
        print(
            f"  {prefix:16} {robot_id:10} occupied={counts[robot_id]:6}  "
            f"{grid.width}x{grid.height}"
        )
    return counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        default="/app/sessions/captures/live-20260829",
    )
    parser.add_argument("--out", default="/app/artefacts")
    parser.add_argument("--modes", default="raw,graph")
    args = parser.parse_args()
    dataset = Path(args.dataset)
    out_root = Path(args.out)
    packets = load_packets(dataset)
    print(f"loaded {len(packets)} keyframes from {dataset}")
    if not packets:
        return 1

    backend = CollaborativeBackend(
        render=RENDER,
        registration_mode="graph",
        pose_prior_sigmas=POSE_PRIOR,
        t_world_map_hint={
            robot_id: yaw_se3(x, y, yaw) for robot_id, (x, y, yaw) in SPAWN.items()
        },
    )
    t0 = time.monotonic()
    for packet in packets:
        backend.ingest_packet(packet)
    print(f"ingested {len(backend)} keyframes in {time.monotonic() - t0:.1f}s")
    keyframes = list(backend._keyframes.values())

    modes = {m.strip() for m in args.modes.split(",") if m.strip()}
    if "raw" in modes:
        print("\n== raw (SLAM poses, no extra solve) ==")
        grids = render_per_robot(raw_graph(keyframes), keyframes, RENDER)
        save_robot_grids(out_root / "01-slam-poses", "raw", grids)

    if "graph" in modes:
        print("\n== graph (odometry suggestion + verified closures) ==")
        t0 = time.monotonic()
        snapshot = backend.optimize_and_render()
        print(f"optimize+render {time.monotonic() - t0:.1f}s")
        if snapshot is None:
            print("no snapshot", file=sys.stderr)
            return 1
        print(
            f"  closures={snapshot.accepted_closures} "
            f"inter_robot={snapshot.inter_robot_closures} "
            f"components={len(snapshot.optimized.components)}"
        )
        save_robot_grids(out_root / "03-rigid", "opt", snapshot.robot_grids)
        merged = max(
            snapshot.grids.values(),
            key=lambda g: (len(g.robots), g.width * g.height),
            default=None,
        )
        if merged is not None:
            write_png(out_root / "03-rigid" / "opt-merged.png", grid_rgb(merged))
            print(
                f"  {'merged':16} robots={sorted(merged.robots)} "
                f"occupied={occupied_count(merged):6}  {merged.width}x{merged.height}"
            )

    if "spawn" in modes:
        print("\n== spawn-aligned (SLAM poses at Gazebo start poses) ==")
        graph = spawn_aligned_graph(keyframes)
        merged = next(iter(render_occupancy(graph, keyframes, RENDER).values()))
        out = out_root / "04-spawn-aligned"
        out.mkdir(parents=True, exist_ok=True)
        write_png(out / "spawn-merged.png", grid_rgb(merged))
        print(
            f"  merged occupied={occupied_count(merged):6}  "
            f"{merged.width}x{merged.height}"
        )

    if "strict" in modes:
        print("\n== graph, stricter inter-robot gates ==")
        strict = CollaborativeBackend(
            render=RENDER,
            registration_mode="graph",
            pose_prior_sigmas=POSE_PRIOR,
            descriptor_ratio=0.8,
            min_pcm_clique_size=4,
        )
        for packet in packets:
            strict.ingest_packet(packet)
        t0 = time.monotonic()
        snapshot = strict.optimize_and_render()
        print(f"optimize+render {time.monotonic() - t0:.1f}s")
        if snapshot is None:
            return 1
        print(
            f"  closures={snapshot.accepted_closures} "
            f"inter_robot={snapshot.inter_robot_closures} "
            f"components={len(snapshot.optimized.components)}"
        )
        save_robot_grids(out_root / "05-strict", "strict", snapshot.robot_grids)
        merged = max(
            snapshot.grids.values(),
            key=lambda g: (len(g.robots), g.width * g.height),
            default=None,
        )
        if merged is not None:
            write_png(out_root / "05-strict" / "strict-merged.png", grid_rgb(merged))
            print(
                f"  {'merged':16} robots={sorted(merged.robots)} "
                f"occupied={occupied_count(merged):6}  {merged.width}x{merged.height}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
