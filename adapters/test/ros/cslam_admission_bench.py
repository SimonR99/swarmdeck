#!/usr/bin/env python3
"""Measure the native Swarm-SLAM inter-robot admission gates offline.

Runs inside `swarmdeck-cslam:*`, with no ROS graph and no network. It replays a
recorded four-robot capture (`<dataset>/keyframes/*.kf` plus a
`ground_truth.csv`) through the SAME code the peers run: cslam's ScanContext
descriptor, cslam's `ScanContextMatching`, and cslam's TEASER++ registration
from `cslam/lidar_pr/icp_utils.py` with the repo's registration-budget patch
applied. The keyframe clouds are put into the shape the peer front end actually
receives by applying `cslam_bridge.py`'s Euclidean range truncation and then
cslam's own `frontend.voxel_size` downsample.

The gate chain it reproduces, in order:

  1. ScanContext cosine similarity >= `frontend.similarity_threshold`
     (cslam/loop_closure_sparse_matching.py, both match callbacks).
  2. Algebraic-connectivity budget selection, `inter_robot_loop_closure_budget`
     -- a rate limit on which candidates get verified, not an accuracy gate,
     so it is not simulated here; it cannot admit anything gate 1 refused.
  3. TEASER++ maximum-clique inlier count > `frontend.registration_min_inliers`
     (cslam/lidar_pr/icp_utils.py `solve_teaser`). This is the ONLY geometric
     gate: there is no residual or overlap check anywhere after it.
  4. Open3D point-to-point ICP refinement, whose result REPLACES the TEASER
     transform unconditionally and is never itself checked.

Because `min_inliers` only decides the boolean in step 3, one registration per
candidate pair scores every `registration_min_inliers` setting at once. The
refinement in step 4 is always run so the recorded transform is the one a peer
would have published had the pair been admitted.

Usage:
    python3 cslam_admission_bench.py DATASET [--out report.json]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

for candidate in ("/app/adapters/protocol", "adapters/protocol"):
    if Path(candidate).is_dir():
        sys.path.insert(0, candidate)
from swarmdeck_protocol.keyframe import decode_keyframe  # noqa: E402

import open3d  # noqa: E402

# cslam's `icp_utils` logs its refusals through `rclpy.logging` but imports only
# `rclpy`. A node process picks the submodule up transitively through
# `rclpy.node`; a bare script does not, and every refusal path would raise
# AttributeError instead of logging. Import it explicitly so the refusals this
# bench is measuring are reached the way a peer reaches them.
import rclpy.logging  # noqa: E402,F401
import cslam.lidar_pr.icp_utils as icp_utils  # noqa: E402
from cslam.lidar_pr.scancontext import ScanContext  # noqa: E402
from cslam.lidar_pr.scancontext_matching import ScanContextMatching  # noqa: E402

# A merge is false when the edge it would add is wrong by more than this. The
# thresholds are the ones the task fixes for the peer path, and they are far
# looser than the 0.03-0.11 m the central pipeline reaches when it merges well.
FALSE_MERGE_M = 0.5
FALSE_MERGE_DEG = 5.0

# (name, similarity_threshold, registration_min_inliers, registration_min_overlap).
# `shipped` is cslam_lidar.yaml as it stands. `upstream` is what MISTLab ships.
# The rest walk the two native knobs down towards the range this data actually
# produces, and then add the ICP overlap gate the peer path does not have.
SETTINGS = [
    ("was 0.95/250", 0.95, 250, 0.0),
    ("upstream 0.90/100", 0.90, 100, 0.0),
    ("0.70/60", 0.70, 60, 0.0),
    ("0.70/24", 0.70, 24, 0.0),
    ("0.70/18", 0.70, 18, 0.0),
    ("0.70/12", 0.70, 12, 0.0),
    ("0.70/18 + ovl 0.30", 0.70, 18, 0.30),
    ("0.70/18 + ovl 0.35", 0.70, 18, 0.35),
    ("0.70/18 + ovl 0.40 NEW", 0.70, 18, 0.40),
    ("0.70/18 + ovl 0.45", 0.70, 18, 0.45),
    ("0.70/18 + ovl 0.50", 0.70, 18, 0.50),
    ("0.70/12 + ovl 0.40", 0.70, 12, 0.40),
    ("0.60/12 + ovl 0.40", 0.60, 12, 0.40),
    ("0.00/12 + ovl 0.40", 0.00, 12, 0.40),
]


def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load_ground_truth(path: Path):
    import csv

    truth: dict[str, list] = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            pose = np.eye(4)
            pose[:3, :3] = quaternion_matrix(
                float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])
            )
            pose[:3, 3] = [float(row["x"]), float(row["y"]), float(row["z"])]
            truth.setdefault(row["robot_id"], []).append((float(row["stamp"]), pose))
    for samples in truth.values():
        samples.sort(key=lambda item: item[0])
    return truth


def truth_at(truth, robot, stamp, max_gap):
    samples = truth.get(robot)
    if not samples:
        return None
    stamps = np.array([s for s, _ in samples])
    index = int(np.argmin(np.abs(stamps - stamp)))
    if abs(stamps[index] - stamp) > max_gap:
        return None
    return samples[index][1]


def pose_error(estimated, expected):
    translation = float(np.linalg.norm(estimated[:3, 3] - expected[:3, 3]))
    relative = expected[:3, :3].T @ estimated[:3, :3]
    cosine = (np.trace(relative) - 1.0) / 2.0
    angle = math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
    return translation, angle


def register(src, dst, voxel_size):
    """cslam's `solve_teaser` plus its refinement, instrumented.

    Identical arithmetic to `icp_utils.solve_teaser`/`compute_transform`, except
    that the inlier count is returned instead of being compared to a threshold,
    and the refinement is run unconditionally so every threshold can be scored
    from one registration.
    """
    src_feats = icp_utils.extract_fpfh(src, voxel_size)
    dst_feats = icp_utils.extract_fpfh(dst, voxel_size)
    corrs_src, corrs_dst = icp_utils.find_correspondences(
        src_feats, dst_feats, mutual_filter=True
    )
    max_matched_pairs = (
        1000  # the cap deploy/patches/cslam-registration-budget.patch adds
    )
    if len(corrs_src) > max_matched_pairs:
        selected = np.linspace(0, len(corrs_src) - 1, max_matched_pairs, dtype=np.int64)
        corrs_src, corrs_dst = corrs_src[selected], corrs_dst[selected]
    src_xyz, dst_xyz = icp_utils.pcd2xyz(src), icp_utils.pcd2xyz(dst)
    solver = icp_utils.get_teaser_solver(voxel_size)
    solver.solve(src_xyz[:, corrs_src], dst_xyz[:, corrs_dst])
    solution = solver.getSolution()
    inliers = len(solver.getInlierMaxClique())
    teaser = icp_utils.Rt2T(solution.rotation, solution.translation)
    refined = open3d.pipelines.registration.registration_icp(
        src,
        dst,
        voxel_size,
        teaser,
        open3d.pipelines.registration.TransformationEstimationPointToPoint(),
        open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    transform = np.asarray(refined.transformation)
    # The patch publishes the inverse relation, T_robot0_robot1, for the
    # BetweenFactor. Score exactly what a peer would have published.
    published = np.eye(4)
    published[:3, :3] = transform[:3, :3].T
    published[:3, 3] = -published[:3, :3] @ transform[:3, 3]
    return {
        "inliers": int(inliers),
        "correspondences": int(len(corrs_src)),
        "fitness": float(refined.fitness),
        "inlier_rmse": float(refined.inlier_rmse),
        "published": published,
    }


def shipped_decision(src, dst, voxel_size, min_inliers, min_overlap):
    """Call the installed `compute_transform`, gates and all.

    This is the end-to-end check on the patch rather than on a model of it: what
    comes back is exactly the `(transform, success)` a peer front end would have
    published for this pair at these settings.
    """
    transform, success = icp_utils.compute_transform(
        src, dst, voxel_size, min_inliers, min_overlap
    )
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return bool(success), matrix


def correct(record):
    return (
        record["translation_error_m"] <= FALSE_MERGE_M
        and record["rotation_error_deg"] <= FALSE_MERGE_DEG
    )


def sweep(report, settings):
    """Score admission settings against the recorded registrations.

    `min_inliers` only decides `solve_teaser`'s boolean and `min_overlap` only
    reads back the refinement Open3D already reported, so every setting is
    scored from the same registrations without repeating any of them.
    """
    records = report["records"]
    rows = []
    for name, similarity, inliers, overlap in settings:
        admitted = [
            record
            for record in records
            if record["kind"] == "candidate"
            and record["similarity"] is not None
            and record["similarity"] >= similarity
        ]
        accepted = [
            record
            for record in admitted
            if record["inliers"] > inliers and record["fitness"] >= overlap
        ]
        leaked = [
            record
            for record in records
            if record["kind"] == "control"
            and record["inliers"] > inliers
            and record["fitness"] >= overlap
        ]
        false_merges = [record for record in accepted if not correct(record)]
        errors = [
            record["translation_error_m"] for record in accepted if correct(record)
        ]
        angles = [
            record["rotation_error_deg"] for record in accepted if correct(record)
        ]
        pairs = {
            tuple(sorted([record["robot0"], record["robot1"]])) for record in accepted
        }
        rows.append(
            {
                "setting": name,
                "similarity_threshold": similarity,
                "registration_min_inliers": inliers,
                "registration_min_overlap": overlap,
                "candidates": len(admitted),
                "accepted": len(accepted),
                "robot_pairs_linked": len(pairs),
                "false_merges": len(false_merges),
                "control_leaks": len(leaked),
                "translation_error_m": [
                    round(min(errors), 3) if errors else None,
                    round(max(errors), 3) if errors else None,
                ],
                "rotation_error_deg": [
                    round(min(angles), 2) if angles else None,
                    round(max(angles), 2) if angles else None,
                ],
                "worst_false_merge_m": (
                    round(max(r["translation_error_m"] for r in false_merges), 2)
                    if false_merges
                    else None
                ),
            }
        )
    return rows


def format_sweep(rows):
    header = (
        f"{'setting':22} {'sim':>5} {'inl':>4} {'ovl':>5} {'cand':>5} {'acc':>4} "
        f"{'pairs':>5} {'false':>5} {'leak':>4} {'err m (min-max)':>18} {'err deg':>14}"
    )
    lines = [header, "-" * len(header)]
    for row in rows:
        low, high = row["translation_error_m"]
        alow, ahigh = row["rotation_error_deg"]
        span = "-" if low is None else f"{low:.3f} - {high:.3f}"
        aspan = "-" if alow is None else f"{alow:.2f} - {ahigh:.2f}"
        lines.append(
            f"{row['setting']:22} {row['similarity_threshold']:>5.2f} "
            f"{row['registration_min_inliers']:>4} "
            f"{row['registration_min_overlap']:>5.2f} {row['candidates']:>5} "
            f"{row['accepted']:>4} {row['robot_pairs_linked']:>5} "
            f"{row['false_merges']:>5} {row['control_leaks']:>4} {span:>18} {aspan:>14}"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--summarize",
        action="store_true",
        help="re-sweep an existing report instead of registering again; "
        "DATASET is then the report JSON",
    )
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--per-robot", type=int, default=18)
    parser.add_argument("--voxel-size", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=30.0)
    parser.add_argument("--max-gap-s", type=float, default=0.5)
    parser.add_argument("--controls", type=int, default=60)
    parser.add_argument("--control-min-m", type=float, default=12.0)
    parser.add_argument("--intra", type=int, default=0)
    parser.add_argument("--intra-min-gap", type=int, default=20)
    parser.add_argument(
        "--check-overlap",
        type=float,
        default=None,
        help="also run the installed compute_transform at this overlap gate and "
        "record whether it accepted, proving the patch rather than a model of it",
    )
    parser.add_argument("--check-inliers", type=int, default=18)
    args = parser.parse_args()

    if args.summarize:
        report = json.loads(args.dataset.read_text())
        report["sweep"] = sweep(report, SETTINGS)
        args.dataset.write_text(json.dumps(report, indent=2))
        print(f"{report['dataset']}: {report['keyframes']} keyframes", flush=True)
        print(format_sweep(report["sweep"]), flush=True)
        return

    truth = load_ground_truth(args.dataset / "ground_truth.csv")
    blobs = sorted((args.dataset / "keyframes").glob("*.kf"))
    if not blobs:
        raise SystemExit(f"no keyframes under {args.dataset}/keyframes")
    frames = []
    for blob in blobs:
        try:
            packet = decode_keyframe(blob.read_bytes())
        except Exception as error:  # a truncated tail is normal in a live capture
            print(f"skipping {blob.name}: {error}", file=sys.stderr)
            continue
        pose = truth_at(truth, packet.robot_id, packet.stamp, args.max_gap_s)
        if pose is None:
            continue
        frames.append((packet, pose))
    robots = sorted({packet.robot_id for packet, _ in frames})
    print(f"{len(frames)} keyframes with ground truth, robots {robots}", flush=True)

    # Even coverage of each trajectory, in capture order so the candidate
    # generation below stays causal.
    selected = []
    for robot in robots:
        mine = [item for item in frames if item[0].robot_id == robot]
        picks = np.linspace(0, len(mine) - 1, min(args.per_robot, len(mine)))
        selected.extend(mine[int(round(p))] for p in picks)
    selected.sort(key=lambda item: item[0].stamp)

    descriptor = ScanContext({}, None)
    clouds, embeddings, poses, heights = {}, {}, {}, {}
    started = time.monotonic()
    for packet, pose in selected:
        key = (packet.robot_id, packet.seq)
        points = np.asarray(packet.points, dtype=np.float64)
        points = points[np.linalg.norm(points, axis=1) <= args.max_range]
        clouds[key] = icp_utils.downsample(points, args.voxel_size)
        embeddings[key] = descriptor.compute_embedding(
            np.asarray(clouds[key].points, dtype=np.float64)
        )
        poses[key] = pose
        heights[packet.robot_id] = float(getattr(packet, "lidar_height", 0.0) or 0.0)
    print(
        f"prepared {len(clouds)} keyframes in {time.monotonic() - started:.1f}s; "
        f"median points {int(np.median([len(c.points) for c in clouds.values()]))}; "
        f"lidar heights {heights}",
        flush=True,
    )

    # cslam's own candidate generation, both halves of it: `peers[(a, b)]` is
    # robot a's copy of robot b's descriptors, queried by a's fresh keyframe
    # (`add_local_global_descriptor`), and `local[a]` is a's own descriptor
    # store, queried by each arriving peer descriptor
    # (`add_other_robot_global_descriptor`). Both are fed only with descriptors
    # that arrived earlier, and `search_best` returns at most one match.
    peers = {
        (robot, peer): ScanContextMatching()
        for robot in robots
        for peer in robots
        if robot != peer
    }
    local = {robot: ScanContextMatching() for robot in robots}
    candidates, direction = {}, {}

    def propose(first, second, similarity):
        pair = tuple(sorted([first, second]))
        direction.setdefault(pair, (first, second))
        candidates[pair] = max(candidates.get(pair, -1.0), float(similarity))

    for packet, _ in selected:
        key = (packet.robot_id, packet.seq)
        for peer in robots:
            if peer == packet.robot_id:
                continue
            match, similarity = peers[(packet.robot_id, peer)].search_best(
                embeddings[key]
            )
            if match is not None:
                propose(key, (peer, match), similarity)
            match, similarity = local[peer].search_best(embeddings[key])
            if match is not None:
                propose((peer, match), key, similarity)
        local[packet.robot_id].add_item(embeddings[key], packet.seq)
        for peer in robots:
            if peer != packet.robot_id:
                peers[(peer, packet.robot_id)].add_item(embeddings[key], packet.seq)
    print(f"{len(candidates)} descriptor candidates proposed", flush=True)

    # A control set of cross-robot pairs the descriptor did NOT propose and that
    # ground truth puts far apart. Any acceptance among these is a false merge
    # the geometric gate let through on its own.
    rng = np.random.default_rng(20260908)
    keys = list(clouds)
    controls = []
    attempts = 0
    while len(controls) < args.controls and attempts < 20000:
        attempts += 1
        a, b = keys[rng.integers(len(keys))], keys[rng.integers(len(keys))]
        if a[0] == b[0] or tuple(sorted([a, b])) in candidates or (a, b) in controls:
            continue
        relative = np.linalg.inv(poses[a]) @ poses[b]
        if np.linalg.norm(relative[:3, 3]) < args.control_min_m:
            continue
        controls.append((a, b))
    print(f"{len(controls)} far control pairs sampled", flush=True)

    # The same thresholds also gate intra-robot closures, so a relaxation that
    # fixes the inter-robot path must not spoil the one that already worked.
    # `detect_intra` queries the robot's own store BEFORE adding the new
    # keyframe and drops matches closer than `intra_loop_min_inbetween_keyframes`.
    intra = {}
    if args.intra:
        store = {robot: ScanContextMatching() for robot in robots}
        for packet, _ in selected:
            key = (packet.robot_id, packet.seq)
            match, similarity = store[packet.robot_id].search_best(embeddings[key])
            if match is not None and abs(match - packet.seq) >= args.intra_min_gap:
                pair = ((packet.robot_id, match), key)
                intra.setdefault(pair, float(similarity))
            store[packet.robot_id].add_item(embeddings[key], packet.seq)
        intra = dict(list(intra.items())[: args.intra])
    print(f"{len(intra)} intra-robot candidates proposed", flush=True)

    records = []
    work = [(direction[pair], candidates[pair], "candidate") for pair in candidates]
    work += [(pair, None, "control") for pair in controls]
    work += [(pair, similarity, "intra") for pair, similarity in intra.items()]
    for number, (pair, similarity, kind) in enumerate(work, 1):
        (robot0, seq0), (robot1, seq1) = pair
        started = time.monotonic()
        result = register(
            clouds[(robot0, seq0)], clouds[(robot1, seq1)], args.voxel_size
        )
        expected = np.linalg.inv(poses[(robot0, seq0)]) @ poses[(robot1, seq1)]
        translation, angle = pose_error(result["published"], expected)
        record = {
            "kind": kind,
            "robot0": robot0,
            "keyframe0": int(seq0),
            "robot1": robot1,
            "keyframe1": int(seq1),
            "similarity": similarity,
            "inliers": result["inliers"],
            "correspondences": result["correspondences"],
            "fitness": result["fitness"],
            "inlier_rmse": result["inlier_rmse"],
            "true_separation_m": float(np.linalg.norm(expected[:3, 3])),
            "translation_error_m": translation,
            "rotation_error_deg": angle,
        }
        if args.check_overlap is not None:
            success, matrix = shipped_decision(
                clouds[(robot0, seq0)],
                clouds[(robot1, seq1)],
                args.voxel_size,
                args.check_inliers,
                args.check_overlap,
            )
            checked, checked_angle = pose_error(matrix, expected)
            record["shipped_success"] = success
            record["shipped_translation_error_m"] = checked if success else None
            record["shipped_rotation_error_deg"] = checked_angle if success else None
            record["shipped_expected"] = bool(
                result["inliers"] > args.check_inliers
                and result["fitness"] >= args.check_overlap
            )
        records.append(record)
        print(
            f"[{number}/{len(work)}] {kind} {robot0}#{seq0} -> {robot1}#{seq1} "
            f"sim={similarity if similarity is None else round(similarity, 4)} "
            f"inliers={result['inliers']} fit={result['fitness']:.3f} "
            f"rmse={result['inlier_rmse']:.3f} "
            f"err={translation:.2f}m/{angle:.1f}deg "
            f"({time.monotonic() - started:.1f}s)",
            flush=True,
        )

    report = {
        "dataset": str(args.dataset),
        "voxel_size": args.voxel_size,
        "max_range_m": args.max_range,
        "keyframes": len(clouds),
        "robots": robots,
        "lidar_heights": heights,
        "false_merge_m": FALSE_MERGE_M,
        "false_merge_deg": FALSE_MERGE_DEG,
        "records": records,
    }
    report["sweep"] = sweep(report, SETTINGS)
    out = args.out or (args.dataset / "cslam_admission_bench.json")
    out.write_text(json.dumps(report, indent=2))
    print(f"wrote {out}", flush=True)
    print(format_sweep(report["sweep"]), flush=True)


if __name__ == "__main__":
    main()
