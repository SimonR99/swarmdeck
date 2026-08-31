"""Score a keyframe reconstruction against the Gazebo indoor world.

Occupancy is compared to the building, not to onboard SLAM. The
reconstruction gauge is removed with one rigid SE(2) before IoU.

Usage from ``slam/``::

    python tools/score_map_vs_world.py ../sessions/captures/live-20260829 \
        --out ../artefacts/08-gt
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np

from swarmdeck_protocol import decode_keyframe
from swarmdeck_slam.backend import CollaborativeBackend
from swarmdeck_slam.render import RenderConfig, render_per_robot
from swarmdeck_slam.types import se3_from_quat_xyz
from swarmdeck_slam.world_occupancy import (
    boxes_from_sdf,
    occupancy_from_grid,
    overlay_png,
    rasterize,
    score_occupancy,
    score_surfaces,
)

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


def load_packets(dataset: Path) -> list:
    blobs = sorted((dataset / "keyframes").glob("*.kf"))
    packets = []
    for path in blobs:
        try:
            packets.append(decode_keyframe(path.read_bytes()))
        except Exception as exc:
            print(f"skip {path.name}: {exc}", file=sys.stderr)
    return packets


def report(name: str, score, surface=None, world_surface=None) -> None:
    suffix = ""
    if surface is not None:
        suffix = (
            f"  surface@{surface.tolerance_m:.2f}m: "
            f"F1={surface.f1:.3f} P={surface.precision:.3f} "
            f"R={surface.recall:.3f} p95={surface.symmetric_p95_m:.3f}m"
        )
    if world_surface is not None:
        suffix += (
            f"  surveyed-world: F1={world_surface.f1:.3f} "
            f"P={world_surface.precision:.3f} "
            f"R={world_surface.recall:.3f}"
        )
    print(
        f"{name:16} IoU={score.iou:.3f}  P={score.precision:.3f}  "
        f"R={score.recall:.3f}  yaw={math.degrees(score.yaw_rad):+.1f}deg  "
        f"shift=({score.shift_x_m:+.2f},{score.shift_y_m:+.2f})m{suffix}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--yaw-step-deg", type=float, default=2.0)
    parser.add_argument("--max-keyframes", type=int)
    parser.add_argument(
        "--sdf",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "swarmdeck_ros/src/swarmdeck_sim/worlds/indoor.sdf",
    )
    args = parser.parse_args()
    packets = load_packets(args.dataset)
    if args.max_keyframes:
        packets = packets[: args.max_keyframes]
    if not packets:
        print("no keyframes", file=sys.stderr)
        return 1
    print(f"loaded {len(packets)} keyframes from {args.dataset}")

    truth = rasterize(boxes_from_sdf(args.sdf), resolution=args.resolution)
    args.out.mkdir(parents=True, exist_ok=True)
    write_png(
        args.out / "world-gt.png",
        overlay_png(truth, truth, score_occupancy(truth, truth, yaw_step_deg=90.0)),
    )

    render = RenderConfig(
        floor_z=0.0,
        min_z=0.15,
        max_z=1.80,
        native_map_resolution=args.resolution,
        odometry_as_pose=False,
        close_occupied=1,
        hit_weight=8,
        peer_exclusion_radius_m=0.80,
        peer_exclusion_max_dt_s=2.0,
    )
    hints = {robot_id: yaw_se3(x, y, yaw) for robot_id, (x, y, yaw) in SPAWN.items()}
    backend = CollaborativeBackend(
        render=render,
        registration_mode="odom_free",
        t_world_map_hint=hints,
    )
    started = time.monotonic()
    for packet in packets:
        backend.ingest_packet(packet)
    snapshot = backend.optimize_and_render()
    print(f"odom-free reconstruct {time.monotonic() - started:.1f}s")
    if snapshot is None:
        print("no snapshot", file=sys.stderr)
        return 1

    grids = list(snapshot.grids.values()) + list(snapshot.robot_grids.values())
    best = None
    best_grid = None
    best_surface = None
    best_world_surface = None
    for grid in grids:
        occupancy = occupancy_from_grid(grid)
        score = score_occupancy(occupancy, truth, yaw_step_deg=args.yaw_step_deg)
        surface = score_surfaces(occupancy, truth, score)
        # The production back-end has already fixed the component gauge from
        # surveyed starts. Report that absolute-world comparison explicitly:
        # exact filled-wall IoU can prefer a one-cell shift that is immaterial
        # to IoU but harmful to the distance-tolerant surface metric.
        world_alignment = dataclasses.replace(
            score, yaw_rad=0.0, shift_x_m=0.0, shift_y_m=0.0
        )
        world_surface = score_surfaces(occupancy, truth, world_alignment)
        label = f"comp{grid.component_id}" if hasattr(grid, "component_id") else "grid"
        robots = ",".join(sorted(grid.robots))
        report(f"{label}[{robots}]", score, surface, world_surface)
        if best is None or score.iou > best.iou:
            best = score
            best_grid = occupancy
            best_surface = surface
            best_world_surface = world_surface
    assert best is not None and best_grid is not None
    write_png(args.out / "reconstructed-vs-gt.png", overlay_png(best_grid, truth, best))
    report("best", best, best_surface, best_world_surface)

    # Onboard SLAM posed at spawn, for reference -- not the reconstruction.
    from swarmdeck_slam.types import Component, OptimizedGraph

    keyframes = list(backend._keyframes.values())
    poses = {
        kf.id: hints[kf.id.robot_id] @ kf.t_odom_base
        for kf in keyframes
        if kf.id.robot_id in hints
    }
    trajectories = frozenset(kf.id.trajectory for kf in keyframes if kf.id in poses)
    slam_graph = OptimizedGraph(
        poses=poses,
        t_world_map=hints,
        t_world_trajectory={
            kf.id.trajectory: hints[kf.id.robot_id]
            for kf in keyframes
            if kf.id.robot_id in hints
        },
        components=[
            Component(
                0,
                frozenset(hints),
                min(kf.id for kf in keyframes if kf.id in poses),
                trajectories,
            )
        ],
    )
    from swarmdeck_slam.render import render_occupancy

    slam_merged = render_occupancy(slam_graph, keyframes, render)
    for grid in slam_merged.values():
        occupancy = occupancy_from_grid(grid)
        score = score_occupancy(occupancy, truth, yaw_step_deg=args.yaw_step_deg)
        report("slam-spawn", score, score_surfaces(occupancy, truth, score))
        write_png(
            args.out / "slam-spawn-vs-gt.png",
            overlay_png(occupancy_from_grid(grid), truth, score),
        )
    slam_grids = render_per_robot(slam_graph, keyframes, render)
    for robot_id, grid in sorted(slam_grids.items()):
        occupancy = occupancy_from_grid(grid)
        score = score_occupancy(occupancy, truth, yaw_step_deg=args.yaw_step_deg)
        report(
            f"slam[{robot_id}]",
            score,
            score_surfaces(occupancy, truth, score),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
