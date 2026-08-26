"""Replay a captured keyframe run against the back-end, and score it.

A Gazebo run costs ten minutes and is not reproducible; a parameter change
deserves seconds and an exact repeat. This turns a captured run into that:
`SWARMDECK_SLAM_CAPTURE_DIR` records the blobs the live service accepted,
`scripts/record_ground_truth.py` records the truth beside them, and this module
replays the pair through `CollaborativeBackend` as many times as you like.

Replay is faithful, not approximate. The blobs are the exact bytes the fleet
sent, ingested in the exact arrival order the filenames preserve, so the
odometry chain (`backend._last_of`) is rebuilt edge-for-edge. What comes out is
the graph the live run would have produced under whichever config you ask for.

    python -m tools.replay --dataset runs/3d-01 --config isotropic
    python -m tools.replay --dataset runs/3d-01 --ablate isotropic hessian

`--ablate` is the point of the tool: identical input, differing only in the
setting under test, tabulated by `evaluation.Ablation`. Scores come from
ground truth, so "better" is a number rather than an opinion about a PNG.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import struct
import sys
import zlib
from dataclasses import replace
from typing import Iterable

import numpy as np

from swarmdeck_protocol import decode_keyframe
from swarmdeck_slam.backend import PRODUCTION_VERIFY, CollaborativeBackend
from swarmdeck_slam.evaluation import Ablation, Report, align_rigid, evaluate
from swarmdeck_slam.render import RenderConfig, RenderedGrid
from swarmdeck_slam.types import KeyframeId, se3_from_quat_xyz

# Named configs under test. Each is a (verify, render) override pair applied to
# the production defaults, so a variant states only what it changes and the
# rest cannot drift apart from the live service by accident.
# name -> (verify overrides, backend overrides). Split so a variant states only
# what it changes and the rest cannot drift from the live service by accident.
CONFIGS: dict[str, tuple[dict, dict]] = {
    # What the live back-end runs today: the Hessian is computed, the degeneracy
    # gates run on it, and then it is replaced by I6 * 400 * fitness.
    "isotropic": ({"information": "isotropic"}, {}),
    # Keeps GICP's conditioned Hessian, so a corridor-slide edge admits its
    # unconstrained direction instead of claiming full confidence in six DoF.
    "hessian": ({"information": "hessian"}, {}),
    # Tighten geometric verification. Everything measured so far says the bad
    # edges are robot_0's OWN closures and that only the odometry prior is
    # holding them down, so the remaining lever is refusing them at verify time.
    # 35 deg of yaw slack is generous against 15 deg of accumulated drift: it
    # leaves GICP room to walk to a shifted minimum one corridor over and still
    # be accepted.
    "yaw-20": ({"information": "hessian", "max_yaw_deviation_from_prior_rad": math.radians(20.0)}, {}),
    "yaw-12": ({"information": "hessian", "max_yaw_deviation_from_prior_rad": math.radians(12.0)}, {}),
    # max_mean_error was calibrated on tests/synthetic.py, where true matches
    # measured 0.045-0.12. Real captured data is an order of magnitude higher:
    # on 3d-run-01, inter-robot candidates whose ground-truth separation is
    # under 8 m have a MEDIAN mean_error of 0.547 (p5 0.073, p95 6.05), so the
    # 0.15 default sits below the typical true match and rejects 52% of all
    # inter-robot candidates. It also barely discriminates -- false pairs
    # measured a LOWER median (0.334) than true ones. These sweep whether the
    # gate is earning its rejections or just throwing away closures.
    # Seed GICP with the current estimate instead of yaw + zero translation.
    # The yaw-only seed caps closure at ~max_correspondence_distance of true
    # separation; see CollaborativeBackend.use_registration_prior.
    "prior-none": ({"information": "hessian"}, {"registration_prior": "none"}),
    "prior-intra": ({"information": "hessian"}, {"registration_prior": "intra"}),
    "prior-all": ({"information": "hessian"}, {"registration_prior": "all"}),
    "mean-err-0.6": ({"information": "hessian", "max_mean_error": 0.6}, {}),
    "mean-err-1.5": ({"information": "hessian", "max_mean_error": 1.5}, {}),
    "inlier-0.80": ({"information": "hessian", "min_inlier_ratio": 0.80}, {}),
    "inlier-0.90": ({"information": "hessian", "min_inlier_ratio": 0.90}, {}),
    "strict": (
        {
            "information": "hessian",
            "max_yaw_deviation_from_prior_rad": math.radians(20.0),
            "min_inlier_ratio": 0.85,
        },
        {},
    ),
    # Loosen the odometry chain so loop closures can actually correct drift.
    "odom-0.25": ({"information": "hessian"}, {"odom_information_scale": 0.25}),
    "odom-0.05": ({"information": "hessian"}, {"odom_information_scale": 0.05}),
    "odom-0.01": ({"information": "hessian"}, {"odom_information_scale": 0.01}),
    # Diagnostic: each robot alone in the same graph. Isolates whether the
    # rotation damage comes from a robot's own closures or from the ties to its
    # peer. Cannot merge, so its collaborative metrics are meaningless by design.
    "intra-only": ({"information": "isotropic"}, {"allow_inter_robot": False}),
    # Lowe ratio test on the place descriptor, at three strictnesses. This is
    # the one that can act at bootstrap, where the wrong merge is actually born.
    "ratio-0.9": ({"information": "isotropic"}, {"descriptor_ratio": 0.9}),
    "ratio-0.8": ({"information": "isotropic"}, {"descriptor_ratio": 0.8}),
    "ratio-0.7": ({"information": "isotropic"}, {"descriptor_ratio": 0.7}),
    # Best-guess stack: ambiguity rejected at bootstrap, honest information
    # afterwards, spatial gate holding the result together.
    "ratio-0.8+hessian": ({"information": "hessian"}, {"descriptor_ratio": 0.8}),
}


def load_ground_truth(path: pathlib.Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """`robot_id -> (stamps, poses7)`, sorted by stamp, for interpolation."""
    rows: dict[str, list[tuple[float, list[float]]]] = {}
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows.setdefault(row["robot_id"], []).append(
                (
                    float(row["stamp"]),
                    [float(row[k]) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")],
                )
            )
    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for robot_id, items in rows.items():
        items.sort(key=lambda item: item[0])
        out[robot_id] = (
            np.array([i[0] for i in items], dtype=np.float64),
            np.array([i[1] for i in items], dtype=np.float64),
        )
    return out


def truth_at(
    truth: dict[str, tuple[np.ndarray, np.ndarray]],
    robot_id: str,
    stamp: float,
    max_gap_s: float,
) -> np.ndarray | None:
    """Nearest true pose as a 4x4, or None if nothing is close enough.

    Nearest rather than interpolated: ground truth arrives far faster than
    keyframes are gated, so the nearest sample is sub-millisecond away, and
    slerping quaternions here would add a second thing to be wrong about
    without moving any number that matters.
    """
    entry = truth.get(robot_id)
    if entry is None:
        return None
    stamps, poses = entry
    if not len(stamps):
        return None
    index = int(np.argmin(np.abs(stamps - stamp)))
    if abs(float(stamps[index]) - stamp) > max_gap_s:
        return None
    return se3_from_quat_xyz(poses[index])


def load_packets(dataset: pathlib.Path) -> list:
    """Decoded keyframe packets in arrival order."""
    blobs = sorted((dataset / "keyframes").glob("*.kf"))
    if not blobs:
        raise SystemExit(f"no keyframes under {dataset / 'keyframes'}")
    packets = []
    for path in blobs:
        try:
            packets.append(decode_keyframe(path.read_bytes()))
        except Exception as exc:  # a truncated tail must not lose the run
            print(f"  skipping {path.name}: {exc}", file=sys.stderr)
    return packets


def filter_by_turn_rate(packets: list, max_deg_s: float) -> list:
    """Drop keyframes captured while rotating faster than ``max_deg_s``.

    Tests the motion-skew hypothesis without touching the simulator. A spinning
    lidar sweeps over a finite time, so a fast rotation smears one revolution
    across a changing pose -- and Gazebo publishes no per-point timestamps, so
    nothing downstream can de-skew it. The cloud is then a slightly wrong SHAPE,
    which is worse than a noisy one: GICP fits it confidently, the degeneracy
    gates see a well-conditioned match, and the result is a geometrically
    excellent constraint to the wrong place. That is precisely the signature
    measured here -- edges that survive a 20 deg yaw gate and an 85% inlier
    ratio untouched, while still degrading the trajectory.

    Rate is estimated between consecutive keyframes of one robot, which is the
    only clock available offline; keyframes are time-gated at 2 s, so this is
    an average over that window and understates brief peaks.
    """
    if max_deg_s <= 0:
        return packets
    from swarmdeck_slam.types import se3_from_quat_xyz

    last: dict[str, tuple[float, float]] = {}
    kept = []
    for packet in packets:
        pose = se3_from_quat_xyz(packet.t_odom_base)
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        stamp = float(packet.stamp)
        previous = last.get(packet.robot_id)
        if previous is not None:
            prev_yaw, prev_stamp = previous
            dt = stamp - prev_stamp
            if dt > 1e-3:
                delta = abs((yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi)
                if math.degrees(delta) / dt > max_deg_s:
                    continue  # dropped: too much rotation inside one sweep
        kept.append(packet)
        last[packet.robot_id] = (yaw, stamp)
    return kept


def run_config(
    name: str,
    packets: Iterable,
    truth: dict[str, tuple[np.ndarray, np.ndarray]],
    render_config: RenderConfig,
    max_gap_s: float,
    optimize_every: int,
) -> tuple[Report, RenderedGrid | None, dict]:
    verify_overrides, backend_overrides = CONFIGS[name]
    backend = CollaborativeBackend(
        verify=replace(PRODUCTION_VERIFY, **verify_overrides),
        render=render_config,
        **backend_overrides,
    )
    # Optimize DURING ingest, not only at the end. The live service solves every
    # OPTIMIZE_EVERY_N keyframes, and that cadence is not a performance detail:
    # candidate gating reads the last solved graph, so a replay that ingested
    # everything first would leave the gate looking at None and silently measure
    # a configuration nobody runs. It also changes results without any gate --
    # later closures are found against corrected poses, exactly as they are live.
    for index, packet in enumerate(packets, start=1):
        backend.ingest_packet(packet)
        if index % optimize_every == 0:
            backend.optimize_and_render()
    snapshot = backend.optimize_and_render()
    if snapshot is None:
        raise SystemExit(f"{name}: nothing ingested")

    graph = snapshot.optimized
    truth_poses: dict[KeyframeId, np.ndarray] = {}
    for kf_id in graph.poses:
        stamp = getattr(backend._keyframes.get(kf_id), "stamp", None)
        if stamp is None:
            continue
        pose = truth_at(truth, kf_id.robot_id, float(stamp), max_gap_s)
        if pose is not None:
            truth_poses[kf_id] = pose

    # True T_world_map per robot, fitted over that robot's WHOLE trajectory.
    #
    # Not from the first keyframe. The pose in a keyframe is map_pose() -- the
    # SLAM-corrected pose in the robot's own map frame -- and SLAM Toolbox moves
    # map->odom every time it re-optimizes. So "the frame at keyframe 0" is not
    # the frame at keyframe 200, and pinning on one instant measures that
    # robot's early correction rather than its frame. Doing exactly that
    # reported 7.6 m of inter-robot error on a run whose joint ATE over the same
    # keyframes was 0.68 m -- two numbers that cannot both be true, because a
    # single rigid alignment cannot pull two trajectories onto truth if their
    # relative transform is metres wrong. The ATE was right and this was wrong.
    #
    # A least-squares fit over every keyframe is the honest summary of a frame
    # that drifts: it is the same Umeyama/Kabsch alignment ATE itself uses.
    truth_t_world_map: dict[str, np.ndarray] = {}
    by_robot: dict[str, dict[KeyframeId, np.ndarray]] = {}
    for kf_id in graph.poses:
        keyframe = backend._keyframes.get(kf_id)
        if keyframe is None or kf_id not in truth_poses:
            continue
        by_robot.setdefault(kf_id.robot_id, {})[kf_id] = keyframe.t_odom_base
    for robot_id, own_poses in by_robot.items():
        if len(own_poses) < 2:
            continue
        # align_rigid(estimated, truth) -> T_truth_estimated, which with
        # "estimated" being map-frame poses is exactly T_world_map.
        truth_t_world_map[robot_id], _ = align_rigid(
            own_poses, {k: truth_poses[k] for k in own_poses}
        )

    # Put truth in the SOLVER's gauge before scoring T_world_map.
    #
    # inter_robot_transform_error compares the two frames directly, with no
    # alignment step -- correct for the synthetic fixture, whose truth is built
    # in the same gauge the solver anchors in. Ours is not: truth here is Gazebo
    # world coordinates while the graph anchors on one keyframe, so every robot
    # picks up the SAME rigid offset and the metric reports it as per-robot
    # error. Left uncorrected it read ~8.8 m for both robots on a run whose
    # robots were within 0.2 m of each other -- a number that would have sent us
    # hunting a collaborative bug that does not exist.
    #
    # align_rigid returns T_truth_estimated, so its inverse carries truth into
    # the estimate's frame. ATE needs no such help: it aligns internally.
    if truth_poses:
        t_truth_est, _ = align_rigid(graph.poses, truth_poses)
        gauge = np.linalg.inv(t_truth_est)
        truth_t_world_map = {r: gauge @ t for r, t in truth_t_world_map.items()}

    # One scene: every robot was spawned into the same world, so a merge
    # between any pair is correct and a missed merge is a real miss.
    truth_groups = {robot_id: 0 for robot_id in truth_t_world_map}

    report = evaluate(
        name, graph, truth_poses, truth_t_world_map, truth_groups, rpe_deltas=(1, 5)
    )
    grid = max(
        snapshot.grids.values(), key=lambda g: (len(g.robots), g.width * g.height)
    ) if snapshot.grids else None
    stats = {
        "keyframes": len(backend._keyframes),
        "accepted_closures": snapshot.accepted_closures,
        "inter_robot_closures": snapshot.inter_robot_closures,
        "residual": graph.final_error,
        "components": [sorted(c.robots) for c in graph.components],
        "scored_keyframes": len(truth_poses),
        "ambiguous_matches": backend.ambiguous_matches,
        "relative_pair_error": _relative_pair_error(
            graph.t_world_map, truth_t_world_map
        ),
    }
    return report, grid, stats


def _relative_pair_error(
    estimated: dict[str, np.ndarray], truth: dict[str, np.ndarray]
) -> dict[str, str]:
    """Per-pair error in the RELATIVE transform between two robots' frames.

    Gauge-free by construction: T_a_b cancels whatever global frame both sides
    are expressed in, so unlike the per-robot T_world_map number this needs no
    alignment and cannot be flattered or spoiled by one. For a collaborative
    system this is the honest headline -- it asks the question the operator
    actually has, which is whether the two robots agree about where each other
    are, not where either is in some absolute frame nobody observes.
    """
    out: dict[str, str] = {}
    common = sorted(set(estimated) & set(truth))
    for i, a in enumerate(common):
        for b in common[i + 1:]:
            rel_est = np.linalg.inv(estimated[a]) @ estimated[b]
            rel_true = np.linalg.inv(truth[a]) @ truth[b]
            delta = np.linalg.inv(rel_true) @ rel_est
            trans = float(np.linalg.norm(delta[:3, 3]))
            cos = (np.trace(delta[:3, :3]) - 1.0) / 2.0
            rot = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
            out[f"{a}->{b}"] = f"{trans:.4f} m, {rot:.4f} deg"
    return out


def write_png(grid: RenderedGrid, path: pathlib.Path) -> None:
    """8-bit greyscale PNG, no image library.

    The slam venv is pinned hard (gtsam / numpy 1.26) and the server's Pillow
    is not in it; a occupancy dump is not worth widening that pin for.
    """
    lut = np.full(256, 200, dtype=np.uint8)          # -1 unknown -> grey
    lut[0] = 255                                      # 0 free     -> white
    lut[100] = 0                                      # 100 occupied -> black
    pixels = lut[grid.cells.astype(np.uint8)]
    raw = b"".join(b"\x00" + row.tobytes() for row in pixels)

    def chunk(tag: bytes, payload: bytes) -> bytes:
        body = tag + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(
            ">I", zlib.crc32(body) & 0xFFFFFFFF
        )

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", grid.width, grid.height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=pathlib.Path)
    parser.add_argument("--config", default="isotropic", choices=sorted(CONFIGS))
    parser.add_argument("--ablate", nargs="+", metavar="CONFIG", choices=sorted(CONFIGS))
    parser.add_argument("--out", type=pathlib.Path, default=None)
    parser.add_argument("--max-gap-s", type=float, default=0.5)
    parser.add_argument("--max-turn-rate", type=float, default=0.0,
                        help="drop keyframes rotating faster than this (deg/s); 0 keeps all")
    # Matches SWARMDECK_SLAM_OPTIMIZE_EVERY in the live service.
    parser.add_argument("--optimize-every", type=int, default=5)
    parser.add_argument("--floor-z", type=float, default=-0.1225)
    parser.add_argument("--min-z", type=float, default=0.150)
    parser.add_argument("--max-z", type=float, default=0.395)
    args = parser.parse_args()

    dataset = args.dataset
    out_dir = args.out or pathlib.Path("replay") / dataset.name
    out_dir.mkdir(parents=True, exist_ok=True)

    packets = load_packets(dataset)
    if args.max_turn_rate > 0:
        before = len(packets)
        packets = filter_by_turn_rate(packets, args.max_turn_rate)
        print(f"turn-rate filter {args.max_turn_rate} deg/s: "
              f"{before} -> {len(packets)} keyframes")
    truth = load_ground_truth(dataset / "ground_truth.csv")
    render_config = RenderConfig(
        floor_z=args.floor_z, min_z=args.min_z, max_z=args.max_z
    )
    print(
        f"dataset {dataset}: {len(packets)} keyframes, "
        f"{sum(len(v[0]) for v in truth.values())} truth samples "
        f"for {sorted(truth)}"
    )

    names = args.ablate or [args.config]
    reports: list[Report] = []
    for name in names:
        report, grid, stats = run_config(
            name, packets, truth, render_config, args.max_gap_s, args.optimize_every
        )
        reports.append(report)
        print(f"\n=== {name} ===")
        print(json.dumps(stats, indent=2, default=str))
        print(report.format())
        if grid is not None:
            png = out_dir / f"map-{name}.png"
            write_png(grid, png)
            print(f"map -> {png}")

    if len(reports) > 1:
        print("\n" + Ablation(tuple(reports)).format())
    return 0


if __name__ == "__main__":
    sys.exit(main())
