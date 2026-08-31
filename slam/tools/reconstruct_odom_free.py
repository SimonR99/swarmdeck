#!/usr/bin/env python3
"""Reconstruct saved keyframes from geometry, with odometry only as a mode vote.

Usage from ``slam/``::

    .venv/bin/python tools/reconstruct_odom_free.py \
        ../sessions/captures/hw-run-01 \
        --output ../sessions/analysis/hw-run-01-odom-free

The output directory contains a machine-readable manifest and one PNG per
independent map component. Keyframe selection flags make it possible to review
or exclude a suspicious range without modifying the capture.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import pathlib
import pickle
import sys
import time
from typing import Any

import numpy as np
from PIL import Image, ImageDraw
from swarmdeck_protocol import decode_keyframe

from swarmdeck_slam.odom_free import (
    OdomFreeConfig,
    RegistrationHypothesis,
    prepare_cloud,
    register_clouds,
)
from swarmdeck_slam.reconstruction import (
    Fragment,
    FragmentMatchConfig,
    ReconstructionFrame,
    TemporalConfig,
    build_temporal_fragments,
    filter_inter_robot_connections,
    find_fragment_connections,
    find_intra_fragment_loops,
    optimize_keyframe_poses,
    place_fragments,
)
from swarmdeck_slam.types import se3_from_quat_xyz


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument(
        "--robot", action="append", help="include this robot (repeatable)"
    )
    parser.add_argument(
        "--trajectory",
        action="append",
        default=[],
        metavar="ROBOT@SESSION",
        help=(
            "include one exact robot/session trajectory (repeatable); use an "
            "empty suffix, for example robot_0@, for a legacy session"
        ),
    )
    parser.add_argument("--start-seq", type=int)
    parser.add_argument("--end-seq", type=int)
    parser.add_argument(
        "--exclude",
        action="append",
        default=[],
        metavar="ROBOT:START-END",
        help="exclude an inclusive sequence range (repeatable)",
    )
    parser.add_argument(
        "--temporal-only",
        action="store_true",
        help="stop after local fragment construction",
    )
    parser.add_argument(
        "--max-keyframes", type=int, help="debug limit after filtering"
    )
    parser.add_argument("--no-cache", action="store_true")
    parser.add_argument("--render-resolution", type=float, default=0.08)
    parser.add_argument(
        "--pose-hints-json",
        default="",
        help=(
            "optional coarse/surveyed T_world_map mapping as JSON, used only "
            "to choose among already-valid symmetric registration modes"
        ),
    )
    defaults = OdomFreeConfig()
    parser.add_argument(
        "--min-z",
        type=float,
        default=defaults.min_z,
        help=f"lowest cloud z used for registration (default: {defaults.min_z:g} m)",
    )
    parser.add_argument(
        "--max-z",
        type=float,
        default=defaults.max_z,
        help=f"highest cloud z used for registration (default: {defaults.max_z:g} m)",
    )
    parser.add_argument(
        "--min-radius",
        type=float,
        default=defaults.min_radius,
        help=(
            "discard returns closer than this radius "
            f"(default: {defaults.min_radius:g} m)"
        ),
    )
    parser.add_argument(
        "--max-radius",
        type=float,
        default=defaults.max_radius,
        help=(
            "discard returns farther than this radius "
            f"(default: {defaults.max_radius:g} m)"
        ),
    )
    temporal_defaults = TemporalConfig()
    parser.add_argument(
        "--max-contiguous-gap-s",
        type=float,
        default=temporal_defaults.max_contiguous_gap_s,
        help=(
            "largest same-robot capture gap eligible for direct geometric "
            f"tracking (default: {temporal_defaults.max_contiguous_gap_s:g} s)"
        ),
    )
    parser.add_argument(
        "--ignore-odom",
        action="store_true",
        help="do not use recorded t_odom_base even as a mode vote",
    )
    return parser.parse_args()


def _excluded(specifications: list[str]) -> dict[str, list[tuple[int, int]]]:
    result: dict[str, list[tuple[int, int]]] = {}
    for specification in specifications:
        try:
            robot, interval = specification.rsplit(":", 1)
            first, last = (int(value) for value in interval.split("-", 1))
        except ValueError as exc:
            raise SystemExit(
                f"invalid --exclude {specification!r}; expected ROBOT:START-END"
            ) from exc
        result.setdefault(robot, []).append((min(first, last), max(first, last)))
    return result


def _trajectories(specifications: list[str]) -> set[tuple[str, str]]:
    result: set[tuple[str, str]] = set()
    for specification in specifications:
        if "@" not in specification:
            raise SystemExit(
                f"invalid --trajectory {specification!r}; expected ROBOT@SESSION"
            )
        robot, session = specification.rsplit("@", 1)
        if not robot:
            raise SystemExit(
                f"invalid --trajectory {specification!r}; robot cannot be empty"
            )
        result.add((robot, session))
    return result


def _load_packets(arguments: argparse.Namespace) -> tuple[list[Any], list[str]]:
    blobs = sorted((arguments.dataset / "keyframes").glob("*.kf"))
    if not blobs:
        raise SystemExit(f"no keyframes under {arguments.dataset / 'keyframes'}")
    exclusions = _excluded(arguments.exclude)
    trajectories = _trajectories(arguments.trajectory)
    packets: list[Any] = []
    names: list[str] = []
    for path in blobs:
        try:
            packet = decode_keyframe(path.read_bytes())
        except Exception as exc:
            print(f"skipping {path.name}: {exc}", file=sys.stderr)
            continue
        if arguments.robot and packet.robot_id not in arguments.robot:
            continue
        if trajectories and (packet.robot_id, packet.session) not in trajectories:
            continue
        if arguments.start_seq is not None and packet.seq < arguments.start_seq:
            continue
        if arguments.end_seq is not None and packet.seq > arguments.end_seq:
            continue
        if any(
            first <= packet.seq <= last
            for first, last in exclusions.get(packet.robot_id, [])
        ):
            continue
        packets.append(packet)
        names.append(path.name)
        if arguments.max_keyframes and len(packets) >= arguments.max_keyframes:
            break
    if not packets:
        raise SystemExit("keyframe selection is empty")
    return packets, names


class RegistrationMemo:
    """Persistent pair cache; recorded poses are absent from both key and value."""

    def __init__(
        self,
        path: pathlib.Path,
        fingerprint: str,
        frames: list[ReconstructionFrame],
        config: OdomFreeConfig,
        enabled: bool,
    ) -> None:
        self.path = path
        self.fingerprint = fingerprint
        self.frames = frames
        self.config = config
        self.enabled = enabled
        self.values: dict[tuple[int, int], list[RegistrationHypothesis]] = {}
        self.new_count = 0
        if enabled and path.exists():
            try:
                payload = pickle.loads(path.read_bytes())
                if payload.get("fingerprint") == fingerprint:
                    self.values = payload["values"]
                    print(f"loaded {len(self.values)} cached pair registrations")
            except Exception as exc:
                print(f"ignoring unreadable registration cache: {exc}", file=sys.stderr)

    def __call__(
        self, target: ReconstructionFrame, source: ReconstructionFrame
    ) -> list[RegistrationHypothesis]:
        key = (target.index, source.index)
        if key not in self.values:
            self.values[key] = register_clouds(target.cloud, source.cloud, self.config)
            self.new_count += 1
            total = len(self.values)
            if self.new_count % 25 == 0:
                print(f"  registered {self.new_count} new pairs ({total} cached total)")
                self.save()
        return self.values[key]

    def save(self) -> None:
        if not self.enabled:
            return
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_bytes(
            pickle.dumps(
                {"fingerprint": self.fingerprint, "values": self.values},
                protocol=pickle.HIGHEST_PROTOCOL,
            )
        )
        temporary.replace(self.path)


def _fingerprint(
    dataset: pathlib.Path,
    names: list[str],
    registration_config: OdomFreeConfig,
    packet_height_bands: list[tuple[float | None, float | None]],
) -> str:
    fields: dict[str, Any] = {
        "dataset": str(dataset.resolve()),
        "keyframes": names,
        "registration": dataclasses.asdict(registration_config),
    }
    # Preserve legacy cache keys byte-for-byte when no packet has calibration.
    if any(
        minimum is not None or maximum is not None
        for minimum, maximum in packet_height_bands
    ):
        fields["packet_height_bands"] = packet_height_bands
    payload = json.dumps(fields, sort_keys=True).encode()
    return hashlib.sha256(payload).hexdigest()


def _packet_registration_config(
    packet: Any, base: OdomFreeConfig
) -> OdomFreeConfig:
    """Use producer-measured physical height limits when the packet has them."""
    if (
        packet.ground_z is None
        or packet.min_height is None
        or packet.max_height is None
    ):
        return base
    return dataclasses.replace(
        base,
        min_z=float(packet.ground_z + packet.min_height),
        max_z=float(packet.ground_z + packet.max_height),
    )


def _component_points(
    component: frozenset[str],
    fragment_poses: dict[str, np.ndarray],
    fragment_by_id: dict[str, Fragment],
    frame_by_index: dict[int, ReconstructionFrame],
    optimized_frame_poses: dict[int, np.ndarray] | None = None,
) -> tuple[np.ndarray, dict[str, list[list[np.ndarray]]]]:
    point_sets: list[np.ndarray] = []
    paths: dict[str, list[list[np.ndarray]]] = {}
    for fragment_id in sorted(component):
        fragment = fragment_by_id[fragment_id]
        t_world_fragment = fragment_poses[fragment_id]
        path: list[np.ndarray] = []
        for frame_index in fragment.frame_indices:
            t_world_frame = (
                optimized_frame_poses[frame_index]
                if optimized_frame_poses is not None
                else t_world_fragment @ fragment.poses[frame_index]
            )
            points = frame_by_index[frame_index].cloud.points
            point_sets.append(points @ t_world_frame[:3, :3].T + t_world_frame[:3, 3])
            path.append(t_world_frame[:2, 3])
        paths.setdefault(fragment.robot_id, []).append(path)
    return np.vstack(point_sets), paths


def _render_component(
    path: pathlib.Path,
    points: np.ndarray,
    paths: dict[str, list[list[np.ndarray]]],
    resolution: float,
    title: str,
) -> dict[str, Any]:
    minimum = np.floor(points[:, :2].min(axis=0) / resolution) * resolution - 1.0
    maximum = np.ceil(points[:, :2].max(axis=0) / resolution) * resolution + 1.0
    size = np.ceil((maximum - minimum) / resolution).astype(int) + 1
    effective_resolution = resolution
    if max(size) > 4096:
        effective_resolution *= max(size) / 4096.0
        size = np.ceil((maximum - minimum) / effective_resolution).astype(int) + 1
    cells = np.floor((points[:, :2] - minimum) / effective_resolution).astype(int)
    counts = np.zeros((size[1], size[0]), dtype=np.uint32)
    np.add.at(counts, (cells[:, 1], cells[:, 0]), 1)
    image_values = np.full(counts.shape, 255, dtype=np.uint8)
    occupied = counts > 0
    if np.any(occupied):
        density = np.log1p(counts[occupied])
        scale = max(float(np.percentile(density, 99)), 1.0)
        image_values[occupied] = np.clip(230.0 - 220.0 * density / scale, 0, 230)
    image = Image.fromarray(image_values[::-1], mode="L").convert("RGB")
    draw = ImageDraw.Draw(image)
    colors = [(0, 94, 255), (230, 55, 55), (34, 160, 80), (170, 70, 200)]
    for color, (robot_id, path_segments) in zip(colors, sorted(paths.items())):
        first_pixel = None
        for positions in path_segments:
            pixels = [
                (
                    float((position[0] - minimum[0]) / effective_resolution),
                    float(
                        size[1]
                        - 1
                        - (position[1] - minimum[1]) / effective_resolution
                    ),
                )
                for position in positions
            ]
            if len(pixels) > 1:
                draw.line(pixels, fill=color, width=2)
            for pixel in pixels:
                draw.ellipse(
                    (pixel[0] - 1, pixel[1] - 1, pixel[0] + 1, pixel[1] + 1),
                    fill=color,
                )
            if pixels and first_pixel is None:
                first_pixel = pixels[0]
        if first_pixel is not None:
            draw.text(
                (first_pixel[0] + 4, first_pixel[1] + 4), robot_id, fill=color
            )
    image.save(path)
    return {
        "title": title,
        "file": path.name,
        "width": int(size[0]),
        "height": int(size[1]),
        "resolution": float(effective_resolution),
        "occupied_cells": int(np.count_nonzero(occupied)),
        "points": int(len(points)),
        "mean_points_per_occupied_cell": float(counts[occupied].mean()),
        "bounds_xy": [minimum.tolist(), maximum.tolist()],
    }


def _matrix(matrix: np.ndarray) -> list[list[float]]:
    return np.round(matrix, 8).tolist()


def _pose_hints(value: str) -> dict[str, np.ndarray] | None:
    if not value:
        return None
    try:
        payload = json.loads(value)
        result = {}
        for robot_id, pose in payload.items():
            yaw = float(pose.get("yaw", 0.0))
            result[str(robot_id)] = se3_from_quat_xyz(
                [
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    0.0,
                    0.0,
                    0.0,
                    math.sin(yaw / 2.0),
                    math.cos(yaw / 2.0),
                ]
            )
        return result or None
    except (TypeError, ValueError, AttributeError) as exc:
        raise SystemExit(f"invalid --pose-hints-json: {exc}") from exc


def main() -> None:
    arguments = _arguments()
    if arguments.render_resolution <= 0.0:
        raise SystemExit("--render-resolution must be positive")
    arguments.output.mkdir(parents=True, exist_ok=True)
    packets, names = _load_packets(arguments)
    registration_config = OdomFreeConfig(
        min_z=arguments.min_z,
        max_z=arguments.max_z,
        min_radius=arguments.min_radius,
        max_radius=arguments.max_radius,
    )
    temporal_config = TemporalConfig(
        max_contiguous_gap_s=arguments.max_contiguous_gap_s
    )
    match_config = FragmentMatchConfig()
    pose_hints = _pose_hints(arguments.pose_hints_json)
    print(
        f"loaded {len(packets)} keyframes from dataset "
        f"{arguments.dataset.resolve()}"
    )
    print(
        "recorded t_odom_base is a weak mode vote; "
        "kinematically impossible hops are ignored"
        if not arguments.ignore_odom
        else "recorded t_odom_base poses will not be read"
    )

    started = time.time()
    frames = [
        ReconstructionFrame(
            index,
            packet.robot_id,
            int(packet.seq),
            float(packet.stamp),
            prepare_cloud(
                packet.points,
                _packet_registration_config(packet, registration_config),
            ),
            packet.session,
            None
            if arguments.ignore_odom
            else se3_from_quat_xyz(packet.t_odom_base),
        )
        for index, packet in enumerate(packets)
    ]
    packet_height_bands = [
        (
            None
            if packet.ground_z is None or packet.min_height is None
            else float(packet.ground_z + packet.min_height),
            None
            if packet.ground_z is None or packet.max_height is None
            else float(packet.ground_z + packet.max_height),
        )
        for packet in packets
    ]
    fingerprint = _fingerprint(
        arguments.dataset, names, registration_config, packet_height_bands
    )
    memo = RegistrationMemo(
        arguments.output / "registrations.pickle",
        fingerprint,
        frames,
        registration_config,
        not arguments.no_cache,
    )

    fragments, boundaries = build_temporal_fragments(frames, memo, temporal_config)
    print(f"temporal stage: {len(fragments)} fragments, {len(boundaries)} boundaries")
    connections = []
    rejected_connections = []
    loop_closures = []
    if not arguments.temporal_only:
        connections, rejected_connections = find_fragment_connections(
            frames,
            fragments,
            memo,
            match_config,
            pose_hints=pose_hints,
        )
        connections, rejected_inter_robot = filter_inter_robot_connections(
            fragments, connections, match_config
        )
        rejected_connections.extend(rejected_inter_robot)
        loop_closures = find_intra_fragment_loops(
            frames, fragments, memo, match_config
        )
        print(
            f"global stage: {len(connections)} accepted fragment links, "
            f"{len(loop_closures)} intra-fragment loop closures, "
            f"{len(rejected_connections)} rejected candidates"
        )
    memo.save()
    placement = place_fragments(fragments, connections)
    optimized_frame_poses = optimize_keyframe_poses(
        fragments, connections, placement, loop_closures
    )

    fragment_by_id = {fragment.fragment_id: fragment for fragment in fragments}
    frame_by_index = {frame.index: frame for frame in frames}
    renders = []
    for component_index, component in enumerate(placement.components):
        points, paths = _component_points(
            component,
            placement.poses,
            fragment_by_id,
            frame_by_index,
            optimized_frame_poses,
        )
        renders.append(
            _render_component(
                arguments.output / f"component-{component_index:02d}.png",
                points,
                paths,
                arguments.render_resolution,
                f"component {component_index}",
            )
        )

    best_robot_renders: dict[str, dict[str, Any]] = {}
    for robot_id in sorted({fragment.robot_id for fragment in fragments}):
        candidates = [
            frozenset(
                fragment_id
                for fragment_id in component
                if fragment_by_id[fragment_id].robot_id == robot_id
            )
            for component in placement.components
        ]
        best_component = max(
            candidates,
            key=lambda component: sum(
                len(fragment_by_id[fragment_id].frame_indices)
                for fragment_id in component
            ),
        )
        points, paths = _component_points(
            best_component,
            placement.poses,
            fragment_by_id,
            frame_by_index,
            optimized_frame_poses,
        )
        render = _render_component(
            arguments.output / f"best-{robot_id}.png",
            points,
            paths,
            arguments.render_resolution,
            f"best verified component for {robot_id}",
        )
        render["keyframes"] = sum(
            len(fragment_by_id[fragment_id].frame_indices)
            for fragment_id in best_component
        )
        render["fragments"] = sorted(best_component)
        best_robot_renders[robot_id] = render

    manifest = {
        "algorithm": "odom-free multi-hypothesis Scan Context + FFT + GICP",
        "dataset": str(arguments.dataset.resolve()),
        "confirmed_hardware_dataset": arguments.dataset.name == "hw-run-01",
        "recorded_pose_or_odometry_used": (
            "not read" if arguments.ignore_odom else "weak mode vote only"
        ),
        "coarse_pose_hints_used": sorted(pose_hints or {}),
        "surveyed_start_poses": {
            robot_id: _matrix(pose)
            for robot_id, pose in sorted((pose_hints or {}).items())
        },
        "keyframe_count": len(frames),
        "robots": sorted({frame.robot_id for frame in frames}),
        "selected_trajectories": sorted(arguments.trajectory),
        "packet_height_calibration_keyframes": sum(
            packet.ground_z is not None
            and packet.min_height is not None
            and packet.max_height is not None
            for packet in packets
        ),
        "registration_config": dataclasses.asdict(registration_config),
        "temporal_config": dataclasses.asdict(temporal_config),
        "fragment_match_config": dataclasses.asdict(match_config),
        "elapsed_seconds": round(time.time() - started, 3),
        "pair_registrations": len(memo.values),
        "fragments": [
            {
                "id": fragment.fragment_id,
                "robot_id": fragment.robot_id,
                "keyframes": len(fragment.frame_indices),
                "frames": [
                    {
                        "capture_file": names[index],
                        "seq": frame_by_index[index].seq,
                        "stamp": frame_by_index[index].stamp,
                        "t_fragment_keyframe": _matrix(fragment.poses[index]),
                        "t_optimized_keyframe": _matrix(optimized_frame_poses[index]),
                    }
                    for index in fragment.frame_indices
                ],
            }
            for fragment in fragments
        ],
        "boundaries": [dataclasses.asdict(boundary) for boundary in boundaries],
        "accepted_connections": [
            {
                "fragment_a": connection.fragment_a,
                "fragment_b": connection.fragment_b,
                "support": connection.support,
                "pose_hint_support": connection.pose_hint_support,
                "score": connection.score,
                "t_a_b": _matrix(connection.t_a_b),
            }
            for connection in connections
        ],
        "intra_fragment_loop_closures": [
            {
                "target_index": closure.target_index,
                "source_index": closure.source_index,
                "target_robot": frame_by_index[closure.target_index].robot_id,
                "target_seq": frame_by_index[closure.target_index].seq,
                "source_seq": frame_by_index[closure.source_index].seq,
                "score": closure.registration.score,
                "path_translation_residual_m": closure.path_translation_residual_m,
                "path_rotation_residual_rad": closure.path_rotation_residual_rad,
                "t_target_source": _matrix(
                    closure.registration.t_target_source
                ),
            }
            for closure in loop_closures
        ],
        "rejected_connections": [
            dataclasses.asdict(connection) for connection in rejected_connections
        ],
        "components": [
            {
                "id": index,
                "fragments": sorted(component),
                "keyframes": sum(
                    len(fragment_by_id[fragment_id].frame_indices)
                    for fragment_id in component
                ),
                "fragment_poses": {
                    fragment_id: _matrix(placement.poses[fragment_id])
                    for fragment_id in sorted(component)
                },
            }
            for index, component in enumerate(placement.components)
        ],
        "renders": renders,
        "best_robot_renders": best_robot_renders,
    }
    manifest_path = arguments.output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {manifest_path}")
    for render in renders:
        print(
            f"  {render['file']}: {render['points']} points, "
            f"{render['mean_points_per_occupied_cell']:.2f} points/occupied cell"
        )


if __name__ == "__main__":
    main()
