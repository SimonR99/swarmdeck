"""Swarm-SLAM solution adapter; the optimizer remains on each robot.

A mission uses a fresh ROS domain and frontend lifetime. Upstream keyframe IDs
have no restart epoch, so restarting a frontend requires a new fleet mission.
This boundary never invents a transform for an unlocalized peer.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import numpy as np

from .capture_providers import CaptureProvenance, CaptureProvider, capture_provider
from .contracts import (
    Calibration,
    ComponentRevision,
    GraphSolution,
    IDENTITY_SE3,
    KeyframeId,
    component_id_for_anchor,
    validate_se3,
)
from .mapping import CorrectionAwareMapper


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    xyz = np.asarray([p.x, p.y, p.z], dtype=float)
    xyzw = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(xyzw)
    if not np.isfinite(xyz).all() or not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid pose")
    x, y, z, w = xyzw / norm
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = xyz
    return validate_se3(T)


def publish_snapshot_if_new(
    path: Path,
    snapshot: dict,
    graph_revision: int,
    published_graph_revision: int,
) -> int:
    """Atomically publish a snapshot only when its map graph has advanced."""

    if graph_revision == published_graph_revision:
        return published_graph_revision
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(snapshot, allow_nan=False))
    os.replace(temporary, path)
    return graph_revision


class CslamMapper:
    """Apply own captures and causally ordered, anchored optimizer results."""

    def __init__(
        self,
        mapper: CorrectionAwareMapper,
        robot_id: str,
        robot_index: int,
        mission_id: str,
        robot_names: dict[int, str],
        capture_provider_name: str | CaptureProvider | None = None,
    ):
        self.mapper = mapper
        self.robot_id, self.robot_index = robot_id, robot_index
        self.mission_id, self.robot_names = mission_id, robot_names
        self.capture_provider = capture_provider(capture_provider_name)
        self.anchor = KeyframeId(robot_id, mission_id, 0)
        self.poses, self.local_poses, self.capture_digests = {}, {}, {}
        self.T_component_local = np.eye(4)
        self.solution_order = (0, -1)
        self.revision = 0
        self.replica_revision = 0
        self.correction_revision = 0
        self.epoch = 0
        self._envelope = None
        self._envelope_map_revision = -1

    def key(self, seq):
        return KeyframeId(self.robot_id, self.mission_id, seq)

    def capture(
        self,
        seq,
        stamp_ns,
        T_local_base,
        points_base,
        covariance=None,
        *,
        T_base_sensor=IDENTITY_SE3,
        sensor_frame="base",
        provenance: CaptureProvenance | None = None,
        colors_rgba: np.ndarray | None = None,
    ):
        key = self.key(seq)
        local = validate_se3(T_local_base)
        if not self.local_poses and seq != 0:
            raise ValueError(
                "First keyframe must be zero; start the bridge before the frontend"
            )
        mount = np.asarray(validate_se3(T_base_sensor))
        calibration_id = (
            "extrinsic:"
            + hashlib.sha256(mount.astype("<f8").tobytes()).hexdigest()[:24]
        )
        calibration = Calibration(
            calibration_id, sensor_frame, "x-forward/y-left/z-up", (), "none", (), mount
        )
        provenance = provenance or CaptureProvenance.unqualified(key, stamp_ns)
        if provenance.transform_timestamp_ns != stamp_ns:
            raise ValueError(
                "keyframe pose timestamp does not match capture provenance"
            )
        capture = self.capture_provider.capture(
            key,
            provenance,
            calibration,
            local,
            covariance,
        )
        identity = json.dumps(
            {"capture": asdict(capture), "calibration": asdict(calibration)},
            sort_keys=True,
            separators=(",", ":"),
        ).encode()
        fingerprint = hashlib.sha256(
            np.asarray(points_base, dtype="<f4").tobytes()
            + b"\n"
            + identity
            + (
                b""
                if colors_rgba is None
                else np.asarray(colors_rgba, dtype=np.uint8).tobytes()
            )
        ).digest()
        if key in self.local_poses:
            if fingerprint != self.capture_digests[key]:
                raise ValueError("Keyframe identity reused with different capture data")
            return False
        points_sensor = (np.asarray(points_base) - mount[:3, 3]) @ mount[:3, :3]
        self.mapper.add_capture(
            capture, calibration, points_sensor, colors_rgba=colors_rgba
        )
        self.local_poses[key] = local
        self.capture_digests[key] = fingerprint
        self.poses[key] = validate_se3(self.T_component_local @ np.asarray(local))
        self._apply()
        return True

    def solution(self, message):
        if not message.success or message.mission_id != self.mission_id:
            return False
        order = (int(message.solution_clock), int(message.optimizer_robot_id))
        if order <= self.solution_order:
            return False
        # Anchor values are supplied by the solver, not inferred from robot
        # starts, proximity, or a dashboard transform.
        anchors = list(message.anchor_estimates)
        if len(anchors) != 1:
            return False
        value = anchors[0]
        origin = int(message.origin_robot_id)
        if value.key.robot_id != origin or value.key.keyframe_id != 0:
            raise ValueError("Optimizer supplied an inconsistent anchor")
        anchor = KeyframeId(self.robot_names[origin], self.mission_id, 0)
        anchor_pose = pose_matrix(value.pose)
        updates = {}
        for value in message.estimates:
            if value.key.robot_id != self.robot_index:
                raise ValueError("Optimizer supplied another robot's local estimate")
            key = self.key(value.key.keyframe_id)
            if key in self.local_poses:
                updates[key] = pose_matrix(value.pose)
        if not updates:
            return False
        latest = max(updates)
        correction = np.asarray(updates[latest]) @ np.linalg.inv(
            self.local_poses[latest]
        )
        poses = {
            key: updates.get(key, validate_se3(correction @ np.asarray(local)))
            for key, local in self.local_poses.items()
        }
        poses[anchor] = anchor_pose
        changed = anchor != self.anchor or any(
            key not in self.poses
            or not np.allclose(pose, self.poses[key], atol=1e-6, rtol=0)
            for key, pose in poses.items()
        )
        self.solution_order = order
        if not changed:
            # Accepted solver clocks are causal map state even when the poses
            # are numerically unchanged. Advance the replica publication
            # without manufacturing a new graph revision and forcing MOLA to
            # rebuild an identical pose product.
            self.replica_revision += 1
            return False
        if anchor != self.anchor:
            self.epoch += 1
        self.anchor, self.poses = anchor, poses
        self.T_component_local = correction
        self.correction_revision += 1
        self._apply()
        return True

    def _apply(self):
        self.revision += 1
        self.replica_revision += 1
        revision = ComponentRevision(
            component_id_for_anchor(self.anchor), self.epoch, self.revision
        )
        self.mapper.apply_solution(
            GraphSolution(revision, self.anchor, tuple(self.poses), self.poses)
        )

    def envelope(self):
        if self._envelope and self._envelope["revision"] == self.replica_revision:
            return self._envelope
        if self._envelope is not None and self._envelope_map_revision == self.revision:
            snapshot = self._envelope["snapshot"]
            chunk_declarations = self._envelope["chunks"]
        else:
            snapshot = self.mapper.snapshot_dict()
            chunks = {
                c["sha256"]: {"sha256": c["sha256"], "size": c["size_bytes"]}
                for m in snapshot["manifests"]
                for c in m["chunks"]
            }
            chunk_declarations = list(chunks.values())
        self._envelope = {
            "version": 1,
            "robot_id": self.robot_id,
            "session_id": self.mission_id,
            "revision": self.replica_revision,
            "solution_order": list(self.solution_order),
            "component_id": component_id_for_anchor(self.anchor),
            "chunks": chunk_declarations,
            "snapshot": snapshot,
        }
        self._envelope_map_revision = self.revision
        return self._envelope
