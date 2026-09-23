"""Swarm-SLAM solution adapter; the optimizer remains on each robot.

A fleet mission remains stable across independent robot map lifetimes. The
durable robot epoch gives restarted frontends new keyframe run identities;
component graph epochs continue to describe changes of solved anchor frame.
This boundary never invents a transform for an unlocalized peer.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Callable
import numpy as np

from .capture_providers import CaptureProvenance, CaptureProvider, capture_provider
from .contracts import (
    Calibration,
    ComponentRevision,
    GraphSolution,
    IDENTITY_SE3,
    KeyframeId,
    Matrix4,
    component_id_for_anchor,
    validate_se3,
)
from .mapping import CorrectionAwareMapper
from .map_epochs import robot_run_id

# Revisions whose frame state is remembered for the product-gated authority.
# About one revision per second while driving, so this covers over an hour of
# MOLA worker lag before an old product can no longer be advertised.
FRAME_HISTORY_LIMIT = 4096


@dataclass(frozen=True)
class FrameState:
    """The component frame that was in effect at one pose-graph revision.

    A MOLA product built from revision R places its geometry with the
    correction, solution order and home pose of R. The authority that
    advertises that product must carry exactly these, not whatever a later
    solution has moved the frame to, so every consumer composes the product
    with the transform it was built under.
    """

    component_id: str
    epoch: int
    T_component_local: Matrix4
    correction_revision: int
    solution_order: tuple[int, int]
    T_component_home: Matrix4 | None


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


# A solver result is adopted (a new correction revision, a rewritten snapshot,
# a MOLA rebuild, and a frame change for every consumer) only when it moves
# some pose beyond these. Measured on benchbot on 2026-09-19 (mission
# 1a8cc114, inter-robot closures on, four robots exploring for 243 s): the
# optimizer reported every 2 to 3 s and each report moved some pose by a few
# millimetres, so under the earlier 5 mm / 0.05 degree tolerances 24 of 36
# plan rejections were frame churn and three of four robots stalled, while
# the 270 closures of the whole run improved the inter-platform floor
# placement by no more than 5 cm. A millimetre refinement is not worth a
# frame change.
SOLUTION_CHANGE_TRANSLATION_M = 0.05
SOLUTION_CHANGE_ROTATION_RAD = math.radians(0.5)
# A result that moves a pose this far (a merge, a loop closure that moves
# poses by decimetres) is adopted at once. A smaller move beyond the change
# tolerance is adopted at most once per interval, so a stream of centimetre
# refinements costs consumers one frame change per interval, not one per
# report. Deferred refinements are not lost: every result is compared with
# the currently adopted poses, so they accumulate until they cross the large
# threshold or the interval passes.
SOLUTION_LARGE_CHANGE_TRANSLATION_M = 0.25
SOLUTION_LARGE_CHANGE_ROTATION_RAD = math.radians(2.0)
SOLUTION_ADOPTION_MIN_INTERVAL_S = 10.0


def pose_displacement(pose, previous) -> tuple[float, float]:
    """Translation (m) and rotation (rad) from one SE(3) pose to another.

    Both are infinite when either matrix is not a finite 4x4, so a malformed
    pose always counts as a large move.
    """

    current = np.asarray(pose, dtype=float)
    reference = np.asarray(previous, dtype=float)
    if current.shape != (4, 4) or reference.shape != (4, 4):
        return math.inf, math.inf
    if not np.isfinite(current).all() or not np.isfinite(reference).all():
        return math.inf, math.inf
    translation = float(np.linalg.norm(current[:3, 3] - reference[:3, 3]))
    relative = reference[:3, :3].T @ current[:3, :3]
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    rotation = float(math.acos(max(-1.0, min(1.0, cosine))))
    return translation, rotation


def pose_moved(pose, previous, translation_m, rotation_rad) -> bool:
    """True when two SE(3) poses differ beyond the given tolerances."""

    translation, rotation = pose_displacement(pose, previous)
    return translation > translation_m or rotation > rotation_rad


@dataclass(frozen=True)
class DeferredSolution:
    """The newest accepted solver result that the adoption interval held back.

    Its poses are not kept. The next result is compared with the currently
    adopted poses, so the refinement this one carried is part of whatever is
    adopted next; this record only says how far the adopted frame lags the
    solver, for diagnostics.
    """

    order: tuple[int, int]
    translation_m: float
    rotation_rad: float


def publish_snapshot_if_new(
    path: Path,
    snapshot: dict,
    graph_revision: int,
    published_graph_revision: int,
    replace: Callable[[Path, Path], object] = os.replace,
) -> int:
    """Atomically publish a snapshot only when its map graph has advanced.

    ``replace(temporary, path)`` moves the written temporary into place; a
    caller that must fence the rename (the bridge, under map_epoch_lock)
    passes its own.
    """

    if graph_revision == published_graph_revision:
        return published_graph_revision
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(snapshot, allow_nan=False))
    replace(temporary, path)
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
        *,
        clock: Callable[[], float] = time.monotonic,
        map_epoch: int = 0,
    ):
        self.mapper = mapper
        self.robot_id, self.robot_index = robot_id, robot_index
        # A solver re-optimizes on every accepted closure and on a timer, and
        # its poses move by numerical noise each time. Only a movement beyond
        # these tolerances is a correction that replaces map geometry; smaller
        # deltas leave the frame, the revision and the MOLA product untouched,
        # so an idle merged fleet does not rebuild its planning map every
        # solver cycle. Deltas accumulate against the last adopted poses, so a
        # slow drift is still adopted once it exceeds them.
        self.solution_change_translation_m = SOLUTION_CHANGE_TRANSLATION_M
        self.solution_change_rotation_rad = SOLUTION_CHANGE_ROTATION_RAD
        # A move beyond the change tolerance but below these is a refinement,
        # adopted at most once per interval; beyond them it is adopted at
        # once. The interval runs on the wall clock (the same clock as the
        # bridge's sensor liveness): the cost it bounds is the consumers'
        # rebuild and replan time, which is real time whatever the simulation
        # rate.
        self.solution_large_change_translation_m = SOLUTION_LARGE_CHANGE_TRANSLATION_M
        self.solution_large_change_rotation_rad = SOLUTION_LARGE_CHANGE_ROTATION_RAD
        self.solution_adoption_min_interval_s = SOLUTION_ADOPTION_MIN_INTERVAL_S
        self.clock = clock
        # Clock reading of the last adoption; None until the first one, which
        # is never held back.
        self.last_adoption_at: float | None = None
        # The newest accepted result held back by the interval, or None when
        # the newest accepted result was adopted or moved nothing.
        self.deferred_solution: DeferredSolution | None = None
        self.mission_id, self.robot_names = mission_id, robot_names
        self.capture_provider = capture_provider(capture_provider_name)
        self.map_epoch = map_epoch
        self.run_id = robot_run_id(mission_id, robot_id, map_epoch)
        self.robot_map_epochs = {index: 0 for index in robot_names}
        self.robot_map_epochs[robot_index] = map_epoch
        self.solution_participants = {robot_index}
        self.peer_epoch_revision = 0
        self.anchor = KeyframeId(robot_id, self.run_id, 0)
        self.poses, self.local_poses, self.capture_digests = {}, {}, {}
        self.T_component_local = np.eye(4)
        self.solution_order = (0, -1)
        # Newest solver clock accepted, whether or not it was adopted.
        self.solver_order = (0, -1)
        self.revision = 0
        self.replica_revision = 0
        self.correction_revision = 0
        self.epoch = 0
        # Frame state per applied revision, oldest first, bounded below.
        self.frame_history: dict[int, FrameState] = {}
        self.frame_history_limit = FRAME_HISTORY_LIMIT
        self._envelope = None
        self._envelope_map_revision = -1

    def key(self, seq):
        return KeyframeId(self.robot_id, self.run_id, seq)

    def observe_epoch(self, robot_index: int, map_epoch: int) -> bool:
        """Fence a peer lifetime without discarding this robot's own captures."""
        if (
            robot_index not in self.robot_map_epochs
            or type(map_epoch) is not int
            or map_epoch < self.robot_map_epochs[robot_index]
            or (robot_index == self.robot_index and map_epoch != self.map_epoch)
        ):
            return False
        if map_epoch == self.robot_map_epochs[robot_index]:
            return True
        self.robot_map_epochs[robot_index] = map_epoch
        self.peer_epoch_revision += 1
        self.deferred_solution = None
        self.replica_revision += 1
        self._envelope = None
        if robot_index in self.solution_participants:
            # A solution depends on every contributing graph, not just its
            # anchor. Retain own captures but retire every product/frame that
            # was solved with the old participant before accepting another.
            self.solution_participants = {self.robot_index}
            self.anchor = self.key(0)
            self.poses = dict(self.local_poses)
            self.T_component_local = np.eye(4)
            self.epoch += 1
            self.frame_history.clear()
            self.solution_order = (0, -1)
            self.solver_order = (0, -1)
            self.last_adoption_at = None
            self.correction_revision += 1
            if self.local_poses:
                self._apply()
        return True

    def accepts_epoch_vector(self, message) -> bool:
        if message.mission_id != self.mission_id:
            return False
        epochs = list(message.robot_map_epochs)
        if len(epochs) != len(self.robot_map_epochs):
            return False
        if int(epochs[self.robot_index]) != self.map_epoch:
            return False
        if any(
            type(epoch) is not int or epoch < self.robot_map_epochs[index]
            for index, epoch in enumerate(epochs)
        ):
            return False
        for index, epoch in enumerate(epochs):
            self.observe_epoch(index, epoch)
        return True

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
        participants = set(message.participant_robot_ids)
        if (
            not participants
            or not participants.issubset(self.robot_names)
            or self.robot_index not in participants
            or message.origin_robot_id not in participants
        ):
            return False
        if not self.accepts_epoch_vector(message):
            return False
        order = (int(message.solution_clock), int(message.optimizer_robot_id))
        if order <= max(self.solution_order, self.solver_order):
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
        anchor = KeyframeId(
            self.robot_names[origin],
            robot_run_id(
                self.mission_id, self.robot_names[origin], self.robot_map_epochs[origin]
            ),
            0,
        )
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
        # The largest move of any pose against the currently adopted poses
        # (not the last report), so deferred refinements accumulate. A new
        # anchor or a keyframe this frame has never placed is a merge, which
        # is always adopted at once.
        translation = rotation = 0.0
        # The first accepted solution is adopted whatever it moves: the
        # solution order names the frame every publisher of a merged
        # component shares, and a publisher left at the pre-optimizer
        # sentinel keeps the merged view unavailable ("Waiting for a common
        # accepted Swarm-SLAM solution"). The anchor robot's own poses do not
        # move in its own frame, so under the change tolerance alone it
        # would never adopt (benchbot 2026-09-19, mission bd4362c4: robot_0
        # at [0, -1] with solver clock 13 while the others were at 10 and 11).
        large = (
            anchor != self.anchor
            or participants != self.solution_participants
            or self.solution_order[1] == -1
        )
        for key, pose in poses.items():
            if key not in self.poses:
                large = True
                continue
            moved_m, moved_rad = pose_displacement(pose, self.poses[key])
            translation, rotation = max(translation, moved_m), max(rotation, moved_rad)
        self.solver_order = order
        changed = large or (
            translation > self.solution_change_translation_m
            or rotation > self.solution_change_rotation_rad
        )
        if not changed:
            # The advertised solution order names the component frame, and
            # every goal is checked against it. A result that moves no pose
            # leaves that frame as it was, so it must not rename it: the
            # solver reports every few seconds, the replica cannot follow a
            # revision per report, and the server then refuses every goal for
            # this robot as a stale frame (robot at [1597, 0], replica at
            # [88, 0] after four idle hours, 2026-09-17). Remember the clock
            # for ordering only.
            self.deferred_solution = None
            return False
        large = large or (
            translation > self.solution_large_change_translation_m
            or rotation > self.solution_large_change_rotation_rad
        )
        now = float(self.clock())
        if (
            not large
            and self.last_adoption_at is not None
            and now - self.last_adoption_at < self.solution_adoption_min_interval_s
        ):
            # A refinement within the interval of the last adoption. Every
            # adoption is a frame change for every consumer (routes
            # re-validated, plans refused as `requested snapshot is not
            # current`, MOLA rebuilt), and a stream of centimetre refinements
            # every 2 to 3 s stalled three of four robots on 2026-09-19. Like
            # a no-motion result this remembers the clock for ordering only;
            # the poses are not kept, because the next result after the
            # interval carries the whole correction against the adopted
            # poses. Nothing adopts a held result on its own: the solver
            # reports on a timer, so the adopted frame lags it by at most
            # the interval and one report.
            self.deferred_solution = DeferredSolution(order, translation, rotation)
            return False
        self.deferred_solution = None
        self.solution_order = order
        if anchor != self.anchor:
            self.epoch += 1
        self.anchor, self.poses = anchor, poses
        self.solution_participants = participants
        self.T_component_local = correction
        self.correction_revision += 1
        self.last_adoption_at = now
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
        self._record_frame(revision)

    def _record_frame(self, revision: ComponentRevision) -> None:
        """Remember the frame this revision's geometry was placed with."""

        home = self.poses.get(self.key(0))
        self.frame_history[revision.revision] = FrameState(
            revision.component_id,
            revision.epoch,
            validate_se3(self.T_component_local, "T_component_local"),
            self.correction_revision,
            (int(self.solution_order[0]), int(self.solution_order[1])),
            None if home is None else validate_se3(home, "T_component_home"),
        )
        while len(self.frame_history) > max(1, int(self.frame_history_limit)):
            self.frame_history.pop(next(iter(self.frame_history)))

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
            "map_epoch": self.map_epoch,
            "run_id": self.run_id,
            "anchor": asdict(self.anchor),
            "participant_robot_ids": [
                self.robot_names[index] for index in sorted(self.solution_participants)
            ],
            "robot_map_epochs": {
                self.robot_names[index]: epoch
                for index, epoch in self.robot_map_epochs.items()
            },
            "revision": self.replica_revision,
            "solution_order": list(self.solution_order),
            "component_id": component_id_for_anchor(self.anchor),
            "chunks": chunk_declarations,
            "snapshot": snapshot,
        }
        self._envelope_map_revision = self.revision
        return self._envelope
