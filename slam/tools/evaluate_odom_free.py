#!/usr/bin/env python3
"""Score an odometry-free reconstruction manifest against Gazebo ground truth.

The reconstruction has an arbitrary origin per disconnected component.  This
tool removes exactly one rigid transform per component (never scale), then
reports the residual trajectory error.  All robots in a merged component share
that one alignment: aligning each robot separately would hide a bad merge.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
from collections import defaultdict
from typing import Any

import numpy as np

from swarmdeck_slam.evaluation import ErrorStats
from swarmdeck_slam.types import se3_distance, se3_from_quat_xyz, se3_kabsch


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=pathlib.Path)
    parser.add_argument("--ground-truth", type=pathlib.Path)
    parser.add_argument("--max-gap-s", type=float, default=0.25)
    parser.add_argument("--output", type=pathlib.Path)
    return parser.parse_args()


def _load_truth(path: pathlib.Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    rows: dict[str, list[tuple[float, np.ndarray]]] = defaultdict(list)
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            pose = np.array(
                [float(row[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")],
                dtype=np.float64,
            )
            rows[row["robot_id"]].append((float(row["stamp"]), pose))
    result: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for robot_id, values in rows.items():
        values.sort(key=lambda item: item[0])
        result[robot_id] = (
            np.array([item[0] for item in values], dtype=np.float64),
            np.stack([item[1] for item in values]),
        )
    return result


def _truth_at(
    truth: dict[str, tuple[np.ndarray, np.ndarray]],
    robot_id: str,
    stamp: float,
    max_gap_s: float,
) -> np.ndarray | None:
    values = truth.get(robot_id)
    if values is None:
        return None
    stamps, poses = values
    index = int(np.searchsorted(stamps, stamp))
    choices = [item for item in (index - 1, index) if 0 <= item < len(stamps)]
    nearest = min(choices, key=lambda item: abs(float(stamps[item]) - stamp))
    if abs(float(stamps[nearest]) - stamp) > max_gap_s:
        return None
    return se3_from_quat_xyz(poses[nearest])


def _stats(errors: list[float], scale: float = 1.0) -> dict[str, float | int]:
    return ErrorStats.from_errors(np.asarray(errors) * scale).to_dict()


def _pose_errors(
    estimated: list[np.ndarray],
    actual: list[np.ndarray],
    alignment: np.ndarray,
) -> dict[str, Any]:
    translation: list[float] = []
    rotation: list[float] = []
    for estimate, truth in zip(estimated, actual, strict=True):
        trans, rot = se3_distance(truth, alignment @ estimate)
        translation.append(trans)
        rotation.append(rot)
    return {
        "translation_m": _stats(translation),
        "rotation_deg": _stats(rotation, 180.0 / math.pi),
        "poses": len(translation),
    }


def evaluate(
    manifest: dict[str, Any],
    truth: dict[str, tuple[np.ndarray, np.ndarray]],
    max_gap_s: float,
) -> dict[str, Any]:
    fragments = {fragment["id"]: fragment for fragment in manifest["fragments"]}
    components: list[dict[str, Any]] = []
    scored = 0
    for component in manifest["components"]:
        estimates: list[np.ndarray] = []
        actual: list[np.ndarray] = []
        robots: list[str] = []
        sequences: list[int] = []
        fragment_ids: list[str] = []
        for fragment_id in component["fragments"]:
            fragment = fragments[fragment_id]
            t_component_fragment = np.asarray(
                component["fragment_poses"][fragment_id], dtype=np.float64
            )
            for frame in fragment["frames"]:
                pose_truth = _truth_at(
                    truth, fragment["robot_id"], float(frame["stamp"]), max_gap_s
                )
                if pose_truth is None:
                    continue
                estimates.append(
                    np.asarray(frame["t_optimized_keyframe"], dtype=np.float64)
                    if "t_optimized_keyframe" in frame
                    else t_component_fragment
                    @ np.asarray(frame["t_fragment_keyframe"], dtype=np.float64)
                )
                actual.append(pose_truth)
                robots.append(fragment["robot_id"])
                sequences.append(int(frame["seq"]))
                fragment_ids.append(fragment_id)
        if len(estimates) < 2:
            components.append(
                {
                    "id": component["id"],
                    "keyframes": component["keyframes"],
                    "scored_poses": len(estimates),
                    "robots": sorted(set(robots)),
                }
            )
            continue

        alignment = se3_kabsch(
            np.stack([pose[:3, 3] for pose in estimates]),
            np.stack([pose[:3, 3] for pose in actual]),
        )
        by_robot: dict[str, dict[str, Any]] = {}
        independent_alignments: dict[str, np.ndarray] = {}
        for robot_id in sorted(set(robots)):
            indices = [index for index, value in enumerate(robots) if value == robot_id]
            robot_est = [estimates[index] for index in indices]
            robot_truth = [actual[index] for index in indices]
            entry = _pose_errors(robot_est, robot_truth, alignment)
            if len(indices) >= 2:
                independent = se3_kabsch(
                    np.stack([pose[:3, 3] for pose in robot_est]),
                    np.stack([pose[:3, 3] for pose in robot_truth]),
                )
                independent_alignments[robot_id] = independent
                entry["independently_aligned"] = _pose_errors(
                    robot_est, robot_truth, independent
                )
            by_robot[robot_id] = entry

        alignment_disagreement: dict[str, dict[str, float]] = {}
        robot_ids = sorted(independent_alignments)
        for index, robot_a in enumerate(robot_ids):
            for robot_b in robot_ids[index + 1 :]:
                trans, rot = se3_distance(
                    independent_alignments[robot_a], independent_alignments[robot_b]
                )
                alignment_disagreement[f"{robot_a}->{robot_b}"] = {
                    "translation_m": trans,
                    "rotation_deg": math.degrees(rot),
                }

        # Relative error is evaluated only within a verified temporal fragment;
        # crossing a fragment boundary would score an intentionally unknown edge.
        rpe: dict[str, dict[str, Any]] = {}
        for delta in (1, 5):
            trans_errors: list[float] = []
            rot_errors: list[float] = []
            grouped: dict[tuple[str, str], list[int]] = defaultdict(list)
            for index, key in enumerate(zip(robots, fragment_ids, strict=True)):
                grouped[key].append(index)
            for indices in grouped.values():
                indices.sort(key=lambda item: sequences[item])
                for offset in range(len(indices) - delta):
                    first, last = indices[offset], indices[offset + delta]
                    est_relative = np.linalg.inv(estimates[first]) @ estimates[last]
                    true_relative = np.linalg.inv(actual[first]) @ actual[last]
                    trans, rot = se3_distance(true_relative, est_relative)
                    trans_errors.append(trans)
                    rot_errors.append(rot)
            if trans_errors:
                rpe[str(delta)] = {
                    "translation_m": _stats(trans_errors),
                    "rotation_deg": _stats(rot_errors, 180.0 / math.pi),
                }

        scored += len(estimates)
        components.append(
            {
                "id": component["id"],
                "keyframes": component["keyframes"],
                "scored_poses": len(estimates),
                "robots": sorted(set(robots)),
                "joint_ate": _pose_errors(estimates, actual, alignment),
                "per_robot_joint_alignment": by_robot,
                "robot_alignment_disagreement": alignment_disagreement,
                "rpe": rpe,
            }
        )

    accepted = manifest.get("accepted_connections", [])
    fragment_robot = {
        fragment_id: fragment["robot_id"] for fragment_id, fragment in fragments.items()
    }
    return {
        "manifest": manifest.get("dataset"),
        "keyframes": manifest["keyframe_count"],
        "scored_poses": scored,
        "elapsed_seconds": manifest.get("elapsed_seconds"),
        "pair_registrations": manifest.get("pair_registrations"),
        "fragments": len(fragments),
        "components": components,
        "accepted_connections": len(accepted),
        "intra_fragment_loop_closures": len(
            manifest.get("intra_fragment_loop_closures", [])
        ),
        "accepted_intra_robot_connections": sum(
            fragment_robot[item["fragment_a"]] == fragment_robot[item["fragment_b"]]
            for item in accepted
        ),
        "accepted_inter_robot_connections": sum(
            fragment_robot[item["fragment_a"]] != fragment_robot[item["fragment_b"]]
            for item in accepted
        ),
    }


def main() -> None:
    arguments = _arguments()
    manifest = json.loads(arguments.manifest.read_text())
    ground_truth_path = arguments.ground_truth
    if ground_truth_path is None:
        ground_truth_path = pathlib.Path(manifest["dataset"]) / "ground_truth.csv"
    result = evaluate(manifest, _load_truth(ground_truth_path), arguments.max_gap_s)
    rendered = json.dumps(result, indent=2) + "\n"
    if arguments.output:
        arguments.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()
