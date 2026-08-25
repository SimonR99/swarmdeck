"""Locate rotation error: front-end odometry, or the pose graph?

ATE reports rotation error on the SOLVED poses, which cannot say whether the
solver inherited it or introduced it. This compares three yaw signals per robot
against ground truth, over the same keyframes:

  raw        the pose the adapter shipped -- SLAM Toolbox's map_pose(), i.e.
             everything the front end knows.
  optimized  the same keyframes after the collaborative pose graph.

If optimized is worse than raw, the graph is spending rotation to satisfy bad
constraints and the fix is in the back end. If they track each other, the error
arrived with the odometry and no amount of graph work will remove it.

Yaw is compared after removing the one constant frame offset that is not an
error (each robot's map frame is arbitrary), by subtracting the median offset
rather than the mean -- a handful of large excursions must not redefine zero.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from tools.replay import (  # noqa: E402
    CONFIGS,
    filter_by_turn_rate,
    load_ground_truth,
    load_packets,
    truth_at,
)

from dataclasses import replace  # noqa: E402

from swarmdeck_slam.backend import PRODUCTION_VERIFY, CollaborativeBackend  # noqa: E402


def yaw_of(matrix: np.ndarray) -> float:
    return math.atan2(matrix[1, 0], matrix[0, 0])


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=pathlib.Path)
    parser.add_argument("--max-gap-s", type=float, default=0.5)
    parser.add_argument("--optimize-every", type=int, default=5)
    parser.add_argument("--jump-deg", type=float, default=3.0)
    parser.add_argument("--configs", nargs="+", default=["isotropic"], choices=sorted(CONFIGS))
    parser.add_argument("--max-turn-rate", type=float, default=0.0,
                        help="drop keyframes rotating faster than this (deg/s); 0 keeps all")
    args = parser.parse_args()

    packets = load_packets(args.dataset)
    if args.max_turn_rate > 0:
        before = len(packets)
        packets = filter_by_turn_rate(packets, args.max_turn_rate)
        print(f"turn-rate filter {args.max_turn_rate} deg/s: "
              f"{before} -> {len(packets)} keyframes ({before - len(packets)} dropped)")
    truth = load_ground_truth(args.dataset / "ground_truth.csv")

    summary: dict[str, dict[str, float]] = {}
    for config_name in args.configs:
        verify_overrides, backend_overrides = CONFIGS[config_name]
        backend = CollaborativeBackend(
            verify=replace(PRODUCTION_VERIFY, **verify_overrides), **backend_overrides
        )
        for index, packet in enumerate(packets, start=1):
            backend.ingest_packet(packet)
            if index % args.optimize_every == 0:
                backend.optimize_and_render()
        snapshot = backend.optimize_and_render()
        if snapshot is None:
            raise SystemExit("nothing ingested")
        solved = snapshot.optimized.poses
        print(f"\n######## config: {config_name} ########")
        summary[config_name] = _report(
            backend, truth, solved, args, robot_stats=True
        )

    if len(summary) > 1:
        robots = sorted({r for v in summary.values() for r in v})
        print("\n=== optimized yaw error RMSE [deg] ===")
        print("  robot      " + "".join(f"{c:>22}" for c in args.configs))
        for robot_id in robots:
            print(
                f"  {robot_id:<10}"
                + "".join(f"{summary[c].get(robot_id, float('nan')):>22.2f}" for c in args.configs)
            )
    return 0


def _report(backend, truth, solved, args, robot_stats: bool) -> dict[str, float]:

    per_robot: dict[str, list[tuple[int, float, float, float]]] = {}
    for kf_id, keyframe in sorted(
        backend._keyframes.items(), key=lambda kv: (kv[0].robot_id, kv[0].seq)
    ):
        true_pose = truth_at(truth, kf_id.robot_id, float(keyframe.stamp), args.max_gap_s)
        if true_pose is None or kf_id not in solved:
            continue
        per_robot.setdefault(kf_id.robot_id, []).append(
            (
                kf_id.seq,
                yaw_of(keyframe.t_odom_base),
                yaw_of(solved[kf_id]),
                yaw_of(true_pose),
            )
        )

    out: dict[str, float] = {}
    for robot_id, rows in sorted(per_robot.items()):
        seqs = np.array([r[0] for r in rows])
        raw = np.array([r[1] for r in rows])
        opt = np.array([r[2] for r in rows])
        true = np.array([r[3] for r in rows])

        raw_err = np.array([wrap(a - b) for a, b in zip(raw, true)])
        opt_err = np.array([wrap(a - b) for a, b in zip(opt, true)])
        raw_err = np.array([wrap(e - np.median(raw_err)) for e in raw_err])
        opt_err = np.array([wrap(e - np.median(opt_err)) for e in opt_err])

        deg = np.degrees
        print(f"\n=== {robot_id} ({len(rows)} keyframes) ===")
        for label, err in (("raw (front end)", raw_err), ("optimized (graph)", opt_err)):
            print(
                f"  {label:<20} yaw err  rmse={deg(np.sqrt(np.mean(err**2))):6.2f}  "
                f"mean|.|={deg(np.mean(np.abs(err))):6.2f}  "
                f"max|.|={deg(np.max(np.abs(err))):6.2f} deg"
            )

        # Where does raw yaw error step? A slip event is a jump, drift is a ramp.
        steps = np.abs(deg(np.diff(raw_err)))
        big = np.where(steps > args.jump_deg)[0]
        print(f"  raw-yaw steps > {args.jump_deg} deg between consecutive keyframes: {len(big)}")
        for i in big[:8]:
            print(
                f"    seq {seqs[i]:>4} -> {seqs[i+1]:<4}  "
                f"{deg(raw_err[i]):+7.2f} -> {deg(raw_err[i+1]):+7.2f} deg "
                f"(step {steps[i]:.2f})"
            )
        ramp = deg(raw_err[-1] - raw_err[0])
        print(f"  net raw yaw drift first->last keyframe: {ramp:+.2f} deg")
        out[robot_id] = float(deg(np.sqrt(np.mean(opt_err**2))))
    return out


if __name__ == "__main__":
    sys.exit(main())
