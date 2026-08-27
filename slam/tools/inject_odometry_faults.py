#!/usr/bin/env python3
"""Create a keyframe dataset with severe, deterministic odometry faults.

Only the ``t_odom_base`` JSON header field is replaced.  Compressed point-cloud
and descriptor bodies remain byte-for-byte identical to the source capture.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil

from swarmdeck_protocol import decode_keyframe
from swarmdeck_slam.fault_injection import (
    generate_faulty_odometry,
    replace_wire_odometry,
    wire_body,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--seed", type=int, default=20260827)
    return parser.parse_args()


def main() -> None:
    arguments = _arguments()
    sources = sorted((arguments.dataset / "keyframes").glob("*.kf"))
    if not sources:
        raise SystemExit(f"no keyframes under {arguments.dataset / 'keyframes'}")
    if arguments.output.exists():
        raise SystemExit(f"output already exists: {arguments.output}")

    blobs = [path.read_bytes() for path in sources]
    packets = [decode_keyframe(blob) for blob in blobs]
    poses, report = generate_faulty_odometry(packets, seed=arguments.seed)

    keyframe_output = arguments.output / "keyframes"
    keyframe_output.mkdir(parents=True)
    source_hash = hashlib.sha256()
    output_hash = hashlib.sha256()
    for source, blob, pose in zip(sources, blobs, poses):
        corrupted = replace_wire_odometry(blob, pose)
        if wire_body(blob) != wire_body(corrupted):
            raise RuntimeError(f"cloud body changed while rewriting {source.name}")
        decoded = decode_keyframe(corrupted)
        if decoded.robot_id != decode_keyframe(blob).robot_id:
            raise RuntimeError(f"packet identity changed while rewriting {source.name}")
        (keyframe_output / source.name).write_bytes(corrupted)
        source_hash.update(blob)
        output_hash.update(corrupted)

    truth = arguments.dataset / "ground_truth.csv"
    if truth.exists():
        shutil.copyfile(truth, arguments.output / truth.name)
    report.update(
        {
            "source_dataset": str(arguments.dataset.resolve()),
            "output_dataset": str(arguments.output.resolve()),
            "keyframes": len(blobs),
            "source_packet_sha256": source_hash.hexdigest(),
            "corrupted_packet_sha256": output_hash.hexdigest(),
            "cloud_bodies_byte_identical": True,
        }
    )
    (arguments.output / "odometry_faults.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )

    print(f"wrote {len(blobs)} corrupted keyframes to {arguments.output}")
    for trajectory in report["trajectories"]:
        name = trajectory["robot_id"]
        if trajectory["session"]:
            name += f"@{trajectory['session']}"
        print(
            f"  {name}: {trajectory['keyframes']} frames, "
            f"{len(trajectory['events'])} jumps/resets, "
            f"max step {trajectory['max_reported_step_m']:.2f} m / "
            f"{trajectory['max_reported_step_yaw_deg']:.1f} deg"
        )
    print("  compressed cloud and descriptor bodies: byte-identical")


if __name__ == "__main__":
    main()
