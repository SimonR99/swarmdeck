"""Deterministic odometry fault injection for captured keyframe benchmarks.

The point cloud in a keyframe is expressed in the robot base frame.  Wheel slip
therefore corrupts ``t_odom_base``, not the cloud itself.  This module replaces
only that pose and deliberately combines the failure modes seen on hardware:

* independent sample jitter;
* slowly accumulating translation and yaw bias;
* persistent pose jumps caused by slip/encoder spikes; and
* arbitrary resets when localization is stopped and later re-enabled.

Fault histories are independent for each ``(robot_id, session)`` trajectory.
The reconstruction benchmark can consequently prove that a collaborative map
does not accidentally rely on a common odometry frame between robots.
"""

from __future__ import annotations

import dataclasses
import json
import math
import struct
from collections import defaultdict
from collections.abc import Sequence
from typing import Any

import numpy as np


@dataclasses.dataclass(frozen=True)
class OdometryFaultConfig:
    """Severity of the synthetic hardware-like odometry faults."""

    position_jitter_std_m: float = 0.12
    yaw_jitter_std_deg: float = 4.0
    increment_translation_std_m: float = 0.025
    increment_yaw_std_deg: float = 0.8
    translation_scale_bias_std: float = 0.08
    yaw_bias_std_deg_per_step: float = 0.25
    initial_position_offset_m: float = 8.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw(pose: np.ndarray) -> float:
    qx, qy, qz, qw = (float(value) for value in pose[3:7])
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _pose7(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    return np.array(
        [x, y, z, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
        dtype=np.float64,
    )


def _trajectory(packet: Any) -> tuple[str, str]:
    return (str(packet.robot_id), str(getattr(packet, "session", "")))


def generate_faulty_odometry(
    packets: Sequence[Any],
    *,
    seed: int = 20260827,
    config: OdometryFaultConfig = OdometryFaultConfig(),
) -> tuple[list[np.ndarray], dict[str, Any]]:
    """Return one corrupted pose per packet and a machine-readable report.

    The true consecutive motion is integrated through a biased/noisy wheel
    model.  Jitter is applied only to the reported sample, so it does not become
    an unrealistic random walk.  Slip jumps and resets modify the latent pose
    and therefore persist in subsequent samples.
    """

    if not packets:
        return [], {
            "seed": seed,
            "config": dataclasses.asdict(config),
            "trajectories": [],
        }

    grouped: dict[tuple[str, str], list[int]] = defaultdict(list)
    for index, packet in enumerate(packets):
        grouped[_trajectory(packet)].append(index)

    faulty: list[np.ndarray | None] = [None] * len(packets)
    trajectory_reports: list[dict[str, Any]] = []
    root_seed = np.random.SeedSequence(seed)

    for trajectory, child_seed in zip(sorted(grouped), root_seed.spawn(len(grouped))):
        rng = np.random.default_rng(child_seed)
        indices = sorted(
            grouped[trajectory],
            key=lambda index: (
                float(packets[index].stamp),
                int(packets[index].seq),
                index,
            ),
        )
        originals = [
            np.asarray(packets[index].t_odom_base, dtype=np.float64)
            for index in indices
        ]
        count = len(indices)

        first = originals[0]
        latent_xy = first[:2].copy() + rng.uniform(
            -config.initial_position_offset_m,
            config.initial_position_offset_m,
            size=2,
        )
        latent_yaw = _wrap(_yaw(first) + rng.uniform(-math.pi, math.pi))
        scale_bias = float(rng.normal(0.0, config.translation_scale_bias_std))
        yaw_bias = math.radians(
            float(rng.normal(0.0, config.yaw_bias_std_deg_per_step))
        )

        event_specs: dict[int, str] = {}
        if count >= 8:
            for fraction, kind in (
                (0.28, "wheel_slip"),
                (0.52, "encoder_spike"),
                (0.72, "odometry_reset"),
                (0.86, "slam_reconnect"),
            ):
                event_index = max(
                    1, min(count - 1, round(fraction * (count - 1)))
                )
                event_specs[event_index] = kind

        events: list[dict[str, Any]] = []
        reported_xy: list[np.ndarray] = []
        reported_yaw: list[float] = []

        for local_index, (packet_index, original) in enumerate(zip(indices, originals)):
            if local_index:
                previous = originals[local_index - 1]
                previous_yaw = _yaw(previous)
                world_delta = original[:2] - previous[:2]
                c, s = math.cos(previous_yaw), math.sin(previous_yaw)
                local_delta = np.array(
                    [c * world_delta[0] + s * world_delta[1],
                     -s * world_delta[0] + c * world_delta[1]],
                    dtype=np.float64,
                )
                local_delta *= 1.0 + scale_bias
                local_delta += rng.normal(
                    0.0, config.increment_translation_std_m, size=2
                )
                c, s = math.cos(latent_yaw), math.sin(latent_yaw)
                latent_xy += np.array(
                    [c * local_delta[0] - s * local_delta[1],
                     s * local_delta[0] + c * local_delta[1]],
                    dtype=np.float64,
                )
                true_yaw_delta = _wrap(_yaw(original) - previous_yaw)
                latent_yaw = _wrap(
                    latent_yaw
                    + true_yaw_delta
                    + yaw_bias
                    + math.radians(float(rng.normal(0.0, config.increment_yaw_std_deg)))
                )

            kind = event_specs.get(local_index)
            if kind in {"wheel_slip", "encoder_spike"}:
                distance_range = (2.5, 4.5) if kind == "wheel_slip" else (1.0, 2.0)
                distance = float(rng.uniform(*distance_range))
                direction = float(rng.uniform(-math.pi, math.pi))
                yaw_jump_deg = float(
                    rng.uniform(25.0, 65.0) * rng.choice(np.array([-1.0, 1.0]))
                    if kind == "wheel_slip"
                    else rng.uniform(10.0, 30.0) * rng.choice(np.array([-1.0, 1.0]))
                )
                jump = distance * np.array([math.cos(direction), math.sin(direction)])
                latent_xy += jump
                latent_yaw = _wrap(latent_yaw + math.radians(yaw_jump_deg))
                events.append(
                    {
                        "kind": kind,
                        "local_index": local_index,
                        "seq": int(packets[packet_index].seq),
                        "translation_jump_m": jump.tolist(),
                        "yaw_jump_deg": yaw_jump_deg,
                    }
                )
            elif kind in {"odometry_reset", "slam_reconnect"}:
                old_xy = latent_xy.copy()
                old_yaw = latent_yaw
                latent_xy = rng.uniform(
                    -config.initial_position_offset_m,
                    config.initial_position_offset_m,
                    size=2,
                )
                latent_yaw = float(rng.uniform(-math.pi, math.pi))
                events.append(
                    {
                        "kind": kind,
                        "local_index": local_index,
                        "seq": int(packets[packet_index].seq),
                        "translation_jump_m": (latent_xy - old_xy).tolist(),
                        "yaw_jump_deg": math.degrees(_wrap(latent_yaw - old_yaw)),
                    }
                )

            sample_xy = latent_xy + rng.normal(
                0.0, config.position_jitter_std_m, size=2
            )
            sample_yaw = _wrap(
                latent_yaw
                + math.radians(float(rng.normal(0.0, config.yaw_jitter_std_deg)))
            )
            faulty[packet_index] = _pose7(
                float(sample_xy[0]),
                float(sample_xy[1]),
                float(original[2]),
                sample_yaw,
            )
            reported_xy.append(sample_xy)
            reported_yaw.append(sample_yaw)

        consecutive_translation = [
            float(np.linalg.norm(reported_xy[index] - reported_xy[index - 1]))
            for index in range(1, count)
        ]
        consecutive_yaw = [
            abs(math.degrees(_wrap(reported_yaw[index] - reported_yaw[index - 1])))
            for index in range(1, count)
        ]
        trajectory_reports.append(
            {
                "robot_id": trajectory[0],
                "session": trajectory[1],
                "keyframes": count,
                "translation_scale_bias": scale_bias,
                "yaw_bias_deg_per_step": math.degrees(yaw_bias),
                "events": events,
                "max_reported_step_m": max(consecutive_translation, default=0.0),
                "max_reported_step_yaw_deg": max(consecutive_yaw, default=0.0),
            }
        )

    return [np.asarray(pose) for pose in faulty], {
        "seed": seed,
        "config": dataclasses.asdict(config),
        "trajectories": trajectory_reports,
    }


_WIRE_HEADER = struct.Struct("<4sHI")


def replace_wire_odometry(blob: bytes, pose: np.ndarray) -> bytes:
    """Replace only ``t_odom_base`` while preserving the compressed body."""

    if len(blob) < _WIRE_HEADER.size:
        raise ValueError("keyframe is too short")
    magic, version, header_length = _WIRE_HEADER.unpack_from(blob)
    header_end = _WIRE_HEADER.size + header_length
    if magic != b"SDKF" or version != 1 or header_end > len(blob):
        raise ValueError("unsupported or malformed keyframe")
    header = json.loads(blob[_WIRE_HEADER.size:header_end].decode("utf-8"))
    replacement = np.asarray(pose, dtype=np.float64).reshape(-1)
    if replacement.shape != (7,) or not np.isfinite(replacement).all():
        raise ValueError("replacement pose must contain seven finite values")
    quaternion_norm = float(np.linalg.norm(replacement[3:]))
    if not 0.9 < quaternion_norm < 1.1:
        raise ValueError("replacement quaternion must have unit length")
    header["t_odom_base"] = replacement.tolist()
    encoded_header = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return (
        _WIRE_HEADER.pack(magic, version, len(encoded_header))
        + encoded_header
        + blob[header_end:]
    )


def wire_body(blob: bytes) -> bytes:
    """Return the opaque compressed cloud/descriptor body for verification."""

    if len(blob) < _WIRE_HEADER.size:
        raise ValueError("keyframe is too short")
    header_length = _WIRE_HEADER.unpack_from(blob)[2]
    return blob[_WIRE_HEADER.size + header_length:]
