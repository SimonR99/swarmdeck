#!/usr/bin/env python3
"""Passively record bounded multi-robot simulation evidence as canonical JSON.

The ARGoS bridge publishes each ``/<robot>/ground_truth`` pose in the shared
``world`` frame. This recorder never publishes commands. It retains stamped
truth poses, records map-authority state transitions, and computes a coarse XY
trajectory-cell overlap heuristic for quick inspection. That heuristic is not
a mapping coverage, loop-closure, or alignment-accuracy measurement.

Example, run on the isolated simulation ROS domain::

    python3 adapters/test/ros/simulation_evidence_recorder.py \
      --duration 90 --output /tmp/swarmdeck-simulation-evidence.json \
      --include-coordination
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

SCHEMA = "swarmdeck.simulation-evidence.v1"
AUTHORITY_FIELDS = (
    "robot_id",
    "mission_id",
    "component_id",
    "map_epoch",
    "mapping_graph_revision",
    "geometry_revision",
    "correction_revision",
    "map_source_stamp",
    "navigation_frame",
    "T_component_navigation",
    "anchor",
    "home",
    "solution_order",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def finite_pose(message: Odometry) -> bool:
    pose = message.pose.pose
    return all(
        math.isfinite(value)
        for value in (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
    )


def trajectory_summary(samples: list[dict], cell_size_m: float) -> dict:
    cells = sorted(
        {
            (
                math.floor(sample["position"]["x"] / cell_size_m),
                math.floor(sample["position"]["y"] / cell_size_m),
            )
            for sample in samples
        }
    )
    path_length = 0.0
    stamp_regressions = 0
    for previous, current in itertools.pairwise(samples):
        if current["stamp_ns"] < previous["stamp_ns"]:
            stamp_regressions += 1
            continue
        path_length += math.dist(
            tuple(previous["position"].values()),
            tuple(current["position"].values()),
        )
    net_displacement = None
    if samples and not stamp_regressions:
        net_displacement = math.dist(
            tuple(samples[0]["position"].values()),
            tuple(samples[-1]["position"].values()),
        )
    frames = sorted({sample["frame_id"] for sample in samples})
    return {
        "sample_count": len(samples),
        "first_stamp_ns": samples[0]["stamp_ns"] if samples else None,
        "last_stamp_ns": samples[-1]["stamp_ns"] if samples else None,
        "stamp_regressions": stamp_regressions,
        "frame_ids": frames,
        "net_displacement_3d_m": net_displacement,
        "path_length_3d_m": path_length if samples else None,
        "visited_xy_cell_count": len(cells),
        "visited_xy_cells": [list(cell) for cell in cells],
    }


def pairwise_overlap(robots: dict[str, dict], cell_size_m: float) -> list[dict]:
    overlaps = []
    for first, second in itertools.combinations(sorted(robots), 2):
        first_summary = robots[first]["trajectory_summary"]
        second_summary = robots[second]["trajectory_summary"]
        first_frames = first_summary["frame_ids"]
        second_frames = second_summary["frame_ids"]
        comparable = (
            len(first_frames) == 1
            and len(second_frames) == 1
            and first_frames == second_frames
        )
        first_cells = {tuple(cell) for cell in first_summary["visited_xy_cells"]}
        second_cells = {tuple(cell) for cell in second_summary["visited_xy_cells"]}
        intersection = first_cells & second_cells
        union = first_cells | second_cells
        record = {
            "robots": [first, second],
            "status": "ok" if comparable else "incomparable_or_missing_frames",
            "cell_size_m": cell_size_m,
            "intersection_cell_count": len(intersection) if comparable else None,
            "union_cell_count": len(union) if comparable else None,
            "jaccard": (
                len(intersection) / len(union) if comparable and union else None
            ),
            "fraction_of_first_cells_shared": (
                len(intersection) / len(first_cells)
                if comparable and first_cells
                else None
            ),
            "fraction_of_second_cells_shared": (
                len(intersection) / len(second_cells)
                if comparable and second_cells
                else None
            ),
        }
        overlaps.append(record)
    return overlaps


class EvidenceRecorder(Node):
    def __init__(
        self,
        robots: list[str],
        *,
        cell_size_m: float,
        max_samples_per_robot: int,
        max_authority_events_per_robot: int,
        max_keyframe_events_per_robot: int,
        include_coordination: bool,
        max_coordination_events: int,
    ) -> None:
        super().__init__("simulation_evidence_recorder")
        self.started_monotonic = time.monotonic()
        self.started_at_utc = utc_now()
        self.cell_size_m = cell_size_m
        self.max_samples_per_robot = max_samples_per_robot
        self.max_authority_events_per_robot = max_authority_events_per_robot
        self.max_keyframe_events_per_robot = max_keyframe_events_per_robot
        self.max_coordination_events = max_coordination_events
        self.robots = {
            robot: {
                "ground_truth_topic": f"/{robot}/ground_truth",
                "map_authority_topic": f"/{robot}/map_authority",
                "keyframe_metadata_topic": f"/{robot}/keyframes",
                "truth_samples": [],
                "truth_samples_dropped": 0,
                "invalid_truth_samples": 0,
                "authority_messages": 0,
                "invalid_authority_messages": 0,
                "authority_events_dropped": 0,
                "authority_events": [],
                "keyframe_metadata_messages": 0,
                "invalid_keyframe_metadata_messages": 0,
                "keyframe_events_dropped": 0,
                "keyframe_events": [],
                "_authority_signature": None,
                "_authority_state": None,
                "_component_change_count": 0,
                "_correction_change_count": 0,
            }
            for robot in robots
        }
        self.coordination = {
            "enabled": include_coordination,
            "events": [],
            "events_dropped": 0,
            "invalid_or_oversized_events": 0,
        }
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        keyframe_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for robot in robots:
            self.create_subscription(
                Odometry,
                f"/{robot}/ground_truth",
                lambda message, robot_id=robot: self.record_truth(robot_id, message),
                reliable,
            )
            self.create_subscription(
                String,
                f"/{robot}/map_authority",
                lambda message, robot_id=robot: self.record_authority(
                    robot_id, message
                ),
                5,
            )
            self.create_subscription(
                String,
                f"/{robot}/keyframes",
                lambda message, robot_id=robot: self.record_keyframe(robot_id, message),
                keyframe_qos,
            )
        if include_coordination:
            for topic in (
                "/swarmdeck/intentions",
                "/swarmdeck/exploration_reports",
            ):
                self.create_subscription(
                    String,
                    topic,
                    lambda message, source=topic: self.record_coordination(
                        source, message
                    ),
                    20,
                )

    def elapsed(self) -> float:
        return time.monotonic() - self.started_monotonic

    def record_truth(self, robot: str, message: Odometry) -> None:
        record = self.robots[robot]
        if not finite_pose(message):
            record["invalid_truth_samples"] += 1
            return
        if len(record["truth_samples"]) >= self.max_samples_per_robot:
            record["truth_samples_dropped"] += 1
            return
        pose = message.pose.pose
        record["truth_samples"].append(
            {
                "stamp_ns": (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                ),
                "received_elapsed_s": self.elapsed(),
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation_xyzw": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            }
        )

    def record_authority(self, robot: str, message: String) -> None:
        record = self.robots[robot]
        record["authority_messages"] += 1
        try:
            if len(message.data.encode("utf-8")) > 32_768:
                raise ValueError("payload exceeds 32768 UTF-8 bytes")
            value = json.loads(
                message.data,
                parse_constant=lambda value: (_ for _ in ()).throw(
                    ValueError(f"non-finite JSON constant {value}")
                ),
            )
            if not isinstance(value, dict):
                raise ValueError("authority payload is not an object")
            state = {field: value.get(field) for field in AUTHORITY_FIELDS}
            signature = json.dumps(
                state, sort_keys=True, separators=(",", ":"), allow_nan=False
            )
        except (TypeError, ValueError, UnicodeError, json.JSONDecodeError):
            record["invalid_authority_messages"] += 1
            return
        if signature == record["_authority_signature"]:
            return
        previous = record["_authority_state"]
        changed_fields = (
            ["initial"]
            if previous is None
            else [
                field for field in AUTHORITY_FIELDS if previous[field] != state[field]
            ]
        )
        event = {
            "received_elapsed_s": self.elapsed(),
            "latest_truth_stamp_ns": (
                record["truth_samples"][-1]["stamp_ns"]
                if record["truth_samples"]
                else None
            ),
            "changed_fields": changed_fields,
            "payload_sha256": hashlib.sha256(message.data.encode("utf-8")).hexdigest(),
            "state": state,
        }
        if len(record["authority_events"]) < self.max_authority_events_per_robot:
            record["authority_events"].append(event)
        else:
            record["authority_events_dropped"] += 1
        if previous is not None:
            record["_component_change_count"] += "component_id" in changed_fields
            record["_correction_change_count"] += (
                "correction_revision" in changed_fields
            )
        record["_authority_signature"] = signature
        record["_authority_state"] = state

    def record_keyframe(self, robot: str, message: String) -> None:
        record = self.robots[robot]
        record["keyframe_metadata_messages"] += 1
        try:
            if len(message.data.encode("utf-8")) > 32_768:
                raise ValueError("payload exceeds 32768 UTF-8 bytes")
            value = json.loads(
                message.data,
                parse_constant=lambda value: (_ for _ in ()).throw(
                    ValueError(f"non-finite JSON constant {value}")
                ),
            )
            if not isinstance(value, dict):
                raise ValueError("keyframe metadata is not an object")
            if (
                not isinstance(value.get("keyframe_id"), str)
                or not value["keyframe_id"]
                or type(value.get("stamp_ns")) is not int
            ):
                raise ValueError("keyframe metadata lacks a typed ID or stamp")
        except (TypeError, ValueError, UnicodeError, json.JSONDecodeError):
            record["invalid_keyframe_metadata_messages"] += 1
            return
        event = {
            "received_elapsed_s": self.elapsed(),
            "payload_sha256": hashlib.sha256(message.data.encode("utf-8")).hexdigest(),
            "metadata": value,
        }
        if len(record["keyframe_events"]) < self.max_keyframe_events_per_robot:
            record["keyframe_events"].append(event)
        else:
            record["keyframe_events_dropped"] += 1

    def record_coordination(self, topic: str, message: String) -> None:
        try:
            payload_size = len(message.data.encode("utf-8"))
        except UnicodeError:
            self.coordination["invalid_or_oversized_events"] += 1
            return
        if payload_size > 32_768:
            self.coordination["invalid_or_oversized_events"] += 1
            return
        event = {
            "received_elapsed_s": self.elapsed(),
            "topic": topic,
            "payload_utf8_bytes": payload_size,
            "payload": message.data,
        }
        if len(self.coordination["events"]) < self.max_coordination_events:
            self.coordination["events"].append(event)
        else:
            self.coordination["events_dropped"] += 1

    def result(self, *, requested_duration_s: float, stopped_early: bool) -> dict:
        output_robots = {}
        for robot, source in self.robots.items():
            summary = trajectory_summary(source["truth_samples"], self.cell_size_m)
            events = source["authority_events"]
            output_robots[robot] = {
                key: value for key, value in source.items() if not key.startswith("_")
            }
            output_robots[robot]["trajectory_summary"] = summary
            output_robots[robot]["authority_summary"] = {
                "recorded_state_count": len(events),
                "component_change_count": source["_component_change_count"],
                "correction_revision_change_count": source["_correction_change_count"],
                "distinct_component_ids": sorted(
                    {
                        event["state"]["component_id"]
                        for event in events
                        if isinstance(event["state"]["component_id"], str)
                    }
                ),
            }
            output_robots[robot]["keyframe_summary"] = {
                "recorded_event_count": len(source["keyframe_events"]),
                "distinct_keyframe_ids": len(
                    {
                        event["metadata"]["keyframe_id"]
                        for event in source["keyframe_events"]
                    }
                ),
            }
        return {
            "schema": SCHEMA,
            "started_at_utc": self.started_at_utc,
            "ended_at_utc": utc_now(),
            "requested_duration_s": requested_duration_s,
            "actual_duration_s": self.elapsed(),
            "stopped_early": stopped_early,
            "ros_domain_id_from_environment": os.environ.get("ROS_DOMAIN_ID"),
            "robots": output_robots,
            "trajectory_overlap_heuristic": {
                "definition": (
                    "Ground-truth positions are quantized into world-aligned XY "
                    "cells. Pairwise values compare unique visited cells and ignore "
                    "time, heading, altitude, sensing, and mapped free space. They "
                    "do not measure map coverage, loop-closure correctness, or "
                    "alignment accuracy."
                ),
                "cell_size_m": self.cell_size_m,
                "pairs": pairwise_overlap(output_robots, self.cell_size_m),
            },
            "authority_interpretation": (
                "Authority component and revision transitions record control-plane "
                "state only; shared component IDs do not establish accurate "
                "inter-robot alignment."
            ),
            "coordination": self.coordination,
        }


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robots",
        nargs="+",
        default=[f"robot_{index}" for index in range(4)],
        help="robot IDs; defaults to robot_0 through robot_3",
    )
    parser.add_argument("--duration", type=positive_float, default=90.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cell-size", type=positive_float, default=1.0)
    parser.add_argument("--max-samples-per-robot", type=positive_int, default=20_000)
    parser.add_argument(
        "--max-authority-events-per-robot", type=positive_int, default=4_096
    )
    parser.add_argument(
        "--max-keyframe-events-per-robot", type=positive_int, default=20_000
    )
    parser.add_argument("--include-coordination", action="store_true")
    parser.add_argument("--max-coordination-events", type=positive_int, default=4_096)
    args, ros_args = parser.parse_known_args()
    robots = [robot.strip("/") for robot in args.robots]
    if any(not robot for robot in robots) or len(set(robots)) != len(robots):
        parser.error("--robots must contain unique, nonempty ROS namespace names")
    if args.duration > 3_600:
        parser.error("--duration is capped at 3600 seconds")

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = EvidenceRecorder(
        robots,
        cell_size_m=args.cell_size,
        max_samples_per_robot=args.max_samples_per_robot,
        max_authority_events_per_robot=args.max_authority_events_per_robot,
        max_keyframe_events_per_robot=args.max_keyframe_events_per_robot,
        include_coordination=args.include_coordination,
        max_coordination_events=args.max_coordination_events,
    )
    stop_requested = False

    def stop(*_unused) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and not stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic()))
            )
    finally:
        result = node.result(
            requested_duration_s=args.duration,
            stopped_early=stop_requested or time.monotonic() < deadline,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
        os.replace(temporary, args.output)
        node.destroy_node()
        rclpy.try_shutdown()
    print(
        json.dumps(
            {
                "output": str(args.output),
                "duration_s": result["actual_duration_s"],
                "truth_samples": {
                    robot: len(value["truth_samples"])
                    for robot, value in result["robots"].items()
                },
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
