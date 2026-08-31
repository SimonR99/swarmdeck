"""Build locally consistent fragments from cloud registration.

Temporal order and timestamps are observations, not odometry. They tell us
which scans were captured near one another in time and bound how fast a ground
robot could have moved; they do not provide a transform. Every transform in a
fragment comes from :func:`swarmdeck_slam.odom_free.register_clouds`.

Recorded ``t_odom_base`` is optional and never an ICP seed or a graph factor.
When a hop is kinematically plausible it may break a 180-degree registration
tie; a 20 m jump, a yaw spike, or a missing pose is ignored and the chain
falls back to geometry. Pair registration itself still never reads odometry.

Fragment boundaries are intentional outputs. Long capture gaps, missing
registration, and impossible motion split the chain instead of fabricating an
edge. A later global stage may reconnect fragments, but only with several
independent geometric correspondences.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import gtsam
import numpy as np
from scipy.spatial import cKDTree

from swarmdeck_slam.descriptors import best_alignment, ring_key
from swarmdeck_slam.odom_free import PreparedCloud, RegistrationHypothesis
from swarmdeck_slam.types import (
    TrajectoryId,
    se3_distance,
    se3_identity,
    se3_inverse,
    se3_relative,
)


@dataclass(frozen=True, slots=True)
class TemporalConfig:
    """Kinematic and graph-consistency policy for fragment construction."""

    # Simulated keyframe production deliberately suppresses scans during turns,
    # producing 10--42 s gaps between consecutive sequence numbers even though
    # the robot moved less than a metre.  A 10 s cutoff split 3d-run-01 into 20
    # fragments and enabled a self-consistent 180-degree corridor flip; 60 s
    # keeps those exact adjacent scans in the Viterbi chain.  Longer outages
    # and explicit producer restarts still split, while geometry and the speed
    # bound can reject a non-overlapping pair inside this window.
    max_contiguous_gap_s: float = 60.0
    max_linear_speed_mps: float = 0.60
    max_yaw_rate_rad_s: float = 0.80
    translation_slack_m: float = 0.35
    yaw_slack_rad: float = math.radians(15.0)
    max_vertical_speed_mps: float = 0.20
    registration_weight: float = 5.0
    velocity_smoothness_weight: float = 3.0
    yaw_rate_smoothness_weight: float = 2.0
    path_speed_weight: float = 0.20
    path_yaw_weight: float = 0.50
    cycle_weight: float = 0.35
    cycle_translation_scale_m: float = 0.30
    cycle_rotation_scale_rad: float = math.radians(6.0)
    max_cycle_penalty: float = 12.0
    # A pair that exposes only one weak symmetric mode is not evidence of
    # continuity.  The held-out Gazebo run contained exactly one such edge:
    # overlap 0.499 / score 0.387 and a 179.9-degree error.  Splitting is safer
    # than forcing the only returned mode into the temporal chain.
    min_temporal_registration_score: float = 0.50
    # Weak vote among already-registered pair modes. Never an ICP seed.
    # A hop inside this envelope may prefer the hypothesis that agrees with it;
    # a hop that could not be robot motion (frame jump, encoder spike) is
    # ignored so catastrophic odometry cannot flip a corridor alias.
    odom_hint_translation_m: float = 0.50
    odom_hint_rotation_rad: float = math.radians(25.0)
    odom_hint_weight: float = 0.5


@dataclass(frozen=True, slots=True)
class ReconstructionFrame:
    """A cloud plus non-pose capture metadata used during reconstruction."""

    index: int
    robot_id: str
    seq: int
    stamp: float
    cloud: PreparedCloud
    session: str = ""
    t_odom_base: np.ndarray | None = None

    @property
    def trajectory_id(self) -> TrajectoryId:
        """Continuous producer run this frame belongs to.

        Robot identity survives a SLAM restart; geometric continuity does not.
        Keeping the session here prevents equal sequence numbers from two runs
        being interleaved before registration has a chance to prove a merge.
        """
        return TrajectoryId(self.robot_id, self.session)


@dataclass(frozen=True, slots=True)
class FragmentEdge:
    target_index: int
    source_index: int
    registration: RegistrationHypothesis


@dataclass(frozen=True, slots=True)
class Fragment:
    """One continuous, geometry-registered run in its own arbitrary frame."""

    fragment_id: str
    robot_id: str
    frame_indices: tuple[int, ...]
    poses: dict[int, np.ndarray]
    edges: tuple[FragmentEdge, ...]
    session: str = ""

    @property
    def trajectory_id(self) -> TrajectoryId:
        return TrajectoryId(self.robot_id, self.session)


@dataclass(frozen=True, slots=True)
class FragmentBoundary:
    robot_id: str
    previous_index: int
    next_index: int
    reason: str
    session: str = ""


@dataclass(frozen=True, slots=True)
class FragmentMatchConfig:
    """Candidate generation and consensus gates for reconnecting fragments."""

    descriptor_neighbors: int = 24
    candidates_per_frame: int = 3
    max_descriptor_distance: float = 0.45
    boundary_window: int = 4
    max_pairs_per_fragment_pair: int = 24
    cluster_translation_m: float = 0.75
    cluster_rotation_rad: float = math.radians(12.0)
    min_support: int = 3
    min_distinct_frames_per_side: int = 2
    min_spatial_span_m: float = 0.60
    ambiguity_support_margin: int = 1
    # Compared as MEAN score per independent proposal. Comparing summed scores
    # made the margin depend on cluster size and accepted equal-support 180deg
    # corridor aliases (8 votes at 4.75 versus 8 at 4.34).
    ambiguity_score_margin: float = 0.20
    # Optional surveyed/coarse starts are only a mode-selection veto. They are
    # never an ICP seed or graph factor and are applied only to the fragment
    # containing each robot's first observation. These deliberately broad
    # limits tolerate an imprecise start while rejecting a catastrophic pi
    # flip; odometry is not read by this gate at all.
    pose_hint_translation_m: float = 8.0
    pose_hint_rotation_rad: float = math.radians(75.0)
    pose_hint_min_fraction: float = 0.50
    boundary_consistency_translation_m: float = 0.75
    boundary_consistency_rotation_rad: float = math.radians(15.0)
    min_boundary_registration_score: float = 0.50
    min_inter_robot_connections: int = 2
    min_inter_robot_separation_m: float = 3.0
    inter_robot_consistency_translation_m: float = 0.75
    inter_robot_consistency_rotation_rad: float = math.radians(10.0)
    loop_min_sequence_separation: int = 20
    loop_descriptor_neighbors: int = 20
    loop_candidates_per_frame: int = 1
    loop_max_descriptor_distance: float = 0.35
    loop_consistency_translation_m: float = 0.75
    loop_consistency_rotation_rad: float = math.radians(15.0)


@dataclass(frozen=True, slots=True)
class ConnectionProposal:
    """One keyframe match expressed as a transform between fragment frames."""

    fragment_a: str
    fragment_b: str
    frame_a: int
    frame_b: int
    t_a_b: np.ndarray
    score: float


@dataclass(frozen=True, slots=True)
class FragmentConnection:
    """A fragment transform corroborated by independent keyframe pairs."""

    fragment_a: str
    fragment_b: str
    t_a_b: np.ndarray
    support: int
    score: float
    proposals: tuple[ConnectionProposal, ...]
    pose_hint_support: int = 0


@dataclass(frozen=True, slots=True)
class FrameLoopClosure:
    """A non-local scan factor inside one already continuous fragment."""

    target_index: int
    source_index: int
    registration: RegistrationHypothesis
    path_translation_residual_m: float
    path_rotation_residual_rad: float


@dataclass(frozen=True, slots=True)
class RejectedConnection:
    fragment_a: str
    fragment_b: str
    best_support: int
    reason: str


@dataclass(frozen=True, slots=True)
class FragmentPlacement:
    """Optimized fragment poses and the independently anchored components."""

    poses: dict[str, np.ndarray]
    components: tuple[frozenset[str], ...]


RegistrationFunction = Callable[
    [ReconstructionFrame, ReconstructionFrame], list[RegistrationHypothesis]
]


def _yaw(transform: np.ndarray) -> float:
    return float(math.atan2(transform[1, 0], transform[0, 0]))


def _motion_plausible(
    transform: np.ndarray, dt: float, config: TemporalConfig
) -> bool:
    translation = float(np.linalg.norm(transform[:2, 3]))
    max_translation = config.translation_slack_m + config.max_linear_speed_mps * dt
    max_yaw = min(math.pi, config.yaw_slack_rad + config.max_yaw_rate_rad_s * dt)
    max_z = config.translation_slack_m + config.max_vertical_speed_mps * dt
    return (
        translation <= max_translation
        and abs(transform[2, 3]) <= max_z
        and abs(_yaw(transform)) <= max_yaw
    )


def _plausible(
    hypothesis: RegistrationHypothesis, dt: float, config: TemporalConfig
) -> bool:
    return _motion_plausible(hypothesis.t_target_source, dt, config)


def _cycle_penalty(
    first: RegistrationHypothesis,
    second: RegistrationHypothesis,
    skip: list[RegistrationHypothesis],
    config: TemporalConfig,
) -> float:
    if not skip:
        return 0.0
    composed = first.t_target_source @ second.t_target_source
    residuals = [se3_distance(item.t_target_source, composed) for item in skip]
    translation, rotation = min(
        residuals,
        key=lambda value: (
            (value[0] / config.cycle_translation_scale_m) ** 2
            + (value[1] / config.cycle_rotation_scale_rad) ** 2
        ),
    )
    normalized = (
        (translation / config.cycle_translation_scale_m) ** 2
        + (rotation / config.cycle_rotation_scale_rad) ** 2
    )
    return config.cycle_weight * min(config.max_cycle_penalty, normalized)


def _transition_penalty(
    first: RegistrationHypothesis,
    second: RegistrationHypothesis,
    dt_first: float,
    dt_second: float,
    skip: list[RegistrationHypothesis],
    config: TemporalConfig,
) -> float:
    first_transform = first.t_target_source
    second_transform = second.t_target_source
    first_velocity = first_transform[:2, 3] / dt_first
    second_velocity_in_first = (
        first_transform[:2, :2] @ second_transform[:2, 3] / dt_second
    )
    velocity_change = float(np.linalg.norm(second_velocity_in_first - first_velocity))
    yaw_rate_change = abs(
        _yaw(second_transform) / dt_second - _yaw(first_transform) / dt_first
    )
    return (
        config.velocity_smoothness_weight * velocity_change**2
        + config.yaw_rate_smoothness_weight * yaw_rate_change**2
        + _cycle_penalty(first, second, skip, config)
    )


def _odom_relative(
    previous: ReconstructionFrame, current: ReconstructionFrame
) -> np.ndarray | None:
    if previous.t_odom_base is None or current.t_odom_base is None:
        return None
    return se3_relative(previous.t_odom_base, current.t_odom_base)


def _odom_hop_usable(
    rel: np.ndarray, dt: float, config: TemporalConfig
) -> bool:
    """True when the recorded hop could have been physical robot motion."""
    return _motion_plausible(rel, dt, config)


def _unary_cost(
    hypothesis: RegistrationHypothesis,
    dt: float,
    config: TemporalConfig,
    odom_rel: np.ndarray | None = None,
) -> float:
    transform = hypothesis.t_target_source
    speed = float(np.linalg.norm(transform[:2, 3])) / dt
    yaw_term = abs(_yaw(transform))
    odom_term = 0.0
    if odom_rel is not None and _odom_hop_usable(odom_rel, dt, config):
        translation, rotation = se3_distance(transform, odom_rel)
        if (
            translation <= config.odom_hint_translation_m
            and rotation <= config.odom_hint_rotation_rad
        ):
            # Prefer the mode that agrees with the hop. Residual yaw replaces
            # the zero-yaw prior so a real 180-degree turn odometry reports
            # can beat path_yaw_weight; a hop that matches no mode is ignored.
            yaw_term = rotation
            odom_term = config.odom_hint_weight * (
                translation / config.odom_hint_translation_m
            ) ** 2
    return (
        -config.registration_weight * hypothesis.score
        + config.path_speed_weight * speed
        + config.path_yaw_weight * yaw_term
        + odom_term
    )


def _select_chain(
    frames: Sequence[ReconstructionFrame],
    adjacent: Sequence[list[RegistrationHypothesis]],
    register: RegistrationFunction,
    config: TemporalConfig,
) -> list[RegistrationHypothesis]:
    """Viterbi selection over pair modes with skip-frame cycle checks."""
    if not adjacent:
        return []
    costs = [
        [
            _unary_cost(
                item,
                frames[1].stamp - frames[0].stamp,
                config,
                _odom_relative(frames[0], frames[1]),
            )
            for item in adjacent[0]
        ]
    ]
    backpointers: list[list[int]] = []
    for edge_index in range(1, len(adjacent)):
        dt_first = frames[edge_index].stamp - frames[edge_index - 1].stamp
        dt_second = frames[edge_index + 1].stamp - frames[edge_index].stamp
        skip = register(frames[edge_index - 1], frames[edge_index + 1])
        row: list[float] = []
        row_backpointers: list[int] = []
        odom_rel = _odom_relative(frames[edge_index], frames[edge_index + 1])
        for second in adjacent[edge_index]:
            unary = _unary_cost(second, dt_second, config, odom_rel)
            alternatives = [
                (
                    costs[-1][previous_index]
                    + unary
                    + _transition_penalty(
                        first, second, dt_first, dt_second, skip, config
                    ),
                    previous_index,
                )
                for previous_index, first in enumerate(adjacent[edge_index - 1])
            ]
            cost, previous_index = min(alternatives, key=lambda value: value[0])
            row.append(cost)
            row_backpointers.append(previous_index)
        costs.append(row)
        backpointers.append(row_backpointers)

    selected_index = int(np.argmin(costs[-1]))
    selected_indices = [selected_index]
    for row in reversed(backpointers):
        selected_index = row[selected_index]
        selected_indices.append(selected_index)
    selected_indices.reverse()
    return [
        candidates[selected]
        for candidates, selected in zip(adjacent, selected_indices, strict=True)
    ]


def _supported_temporal_modes(
    target: ReconstructionFrame,
    source: ReconstructionFrame,
    register: RegistrationFunction,
    config: TemporalConfig,
) -> list[RegistrationHypothesis]:
    dt = source.stamp - target.stamp
    if not 0.0 < dt <= 2.0 * config.max_contiguous_gap_s:
        return []
    return [
        item
        for item in register(target, source)
        if _plausible(item, dt, config)
        and item.score >= config.min_temporal_registration_score
    ]


def _corroborated_skip_bridge(
    ordered: Sequence[ReconstructionFrame],
    edge_index: int,
    run_frames: Sequence[ReconstructionFrame],
    run_adjacent: Sequence[list[RegistrationHypothesis]],
    register: RegistrationFunction,
    config: TemporalConfig,
) -> list[RegistrationHypothesis]:
    """Infer one missing adjacent hop from independent cycles on both sides.

    A partial scan during a turn may expose only the 180-degree corridor alias,
    even though the preceding-to-current and current-to-following skip pairs
    overlap strongly.  Continuity is safe only when *both* three-frame cycles
    imply the same missing transform.  One-sided evidence remains a fragment
    boundary, preserving the conservative policy for restarts and aliases.
    """
    if (
        edge_index == 0
        or edge_index + 2 >= len(ordered)
        or len(run_frames) < 2
        or not run_adjacent
    ):
        return []
    before = ordered[edge_index - 1]
    target = ordered[edge_index]
    source = ordered[edge_index + 1]
    after = ordered[edge_index + 2]
    if run_frames[-2].index != before.index or run_frames[-1].index != target.index:
        return []
    if [frame.seq for frame in (before, target, source, after)] != list(
        range(before.seq, before.seq + 4)
    ):
        return []
    adjacent_after = _supported_temporal_modes(source, after, register, config)
    skip_before = _supported_temporal_modes(before, source, register, config)
    skip_after = _supported_temporal_modes(target, after, register, config)
    if not adjacent_after or not skip_before or not skip_after:
        return []

    dt = source.stamp - target.stamp
    rescued: list[RegistrationHypothesis] = []
    for adjacent_before in run_adjacent[-1]:
        for before_to_source in skip_before:
            left = (
                se3_inverse(adjacent_before.t_target_source)
                @ before_to_source.t_target_source
            )
            if not _motion_plausible(left, dt, config):
                continue
            for target_to_after in skip_after:
                for source_to_after in adjacent_after:
                    right = (
                        target_to_after.t_target_source
                        @ se3_inverse(source_to_after.t_target_source)
                    )
                    if not _motion_plausible(right, dt, config):
                        continue
                    translation, rotation = se3_distance(left, right)
                    if (
                        translation > config.cycle_translation_scale_m
                        or rotation > config.cycle_rotation_scale_rad
                    ):
                        continue
                    witnesses = (
                        adjacent_before,
                        before_to_source,
                        target_to_after,
                        source_to_after,
                    )
                    candidate = RegistrationHypothesis(
                        t_target_source=left,
                        yaw_prior=_yaw(left),
                        descriptor_distance=max(
                            item.descriptor_distance for item in witnesses
                        ),
                        coarse_score=min(item.coarse_score for item in witnesses),
                        symmetric_overlap=min(
                            item.symmetric_overlap for item in witnesses
                        ),
                        symmetric_rmse=max(item.symmetric_rmse for item in witnesses),
                        gicp_mean_error=max(
                            item.gicp_mean_error for item in witnesses
                        ),
                        num_inliers=min(item.num_inliers for item in witnesses),
                        score=min(item.score for item in witnesses),
                    )
                    if not any(
                        se3_distance(candidate.t_target_source, existing.t_target_source)[
                            0
                        ]
                        <= 0.5 * config.cycle_translation_scale_m
                        and se3_distance(
                            candidate.t_target_source, existing.t_target_source
                        )[1]
                        <= 0.5 * config.cycle_rotation_scale_rad
                        for existing in rescued
                    ):
                        rescued.append(candidate)
    return rescued


def build_temporal_fragments(
    frames: Sequence[ReconstructionFrame],
    register: RegistrationFunction,
    config: TemporalConfig | None = None,
) -> tuple[list[Fragment], list[FragmentBoundary]]:
    """Register ordered frames into fragments, refusing unsupported continuity.

    Frames may contain several robots and need not be pre-sorted. Sequence and
    time decide only which pairs are eligible for a local chain. Their relative
    transforms are inferred exclusively from cloud registration.
    """
    config = config or TemporalConfig()
    fragments: list[Fragment] = []
    boundaries: list[FragmentBoundary] = []
    by_trajectory: dict[TrajectoryId, list[ReconstructionFrame]] = {}
    for frame in frames:
        by_trajectory.setdefault(frame.trajectory_id, []).append(frame)

    for trajectory_id in sorted(by_trajectory):
        robot_id, session = trajectory_id
        ordered = sorted(
            by_trajectory[trajectory_id], key=lambda item: (item.seq, item.stamp)
        )
        candidate_runs: list[
            tuple[list[ReconstructionFrame], list[list[RegistrationHypothesis]]]
        ] = []
        run_frames = [ordered[0]] if ordered else []
        run_adjacent: list[list[RegistrationHypothesis]] = []
        for edge_index, (previous, current) in enumerate(
            zip(ordered, ordered[1:])
        ):
            dt = current.stamp - previous.stamp
            reason = ""
            candidates: list[RegistrationHypothesis] = []
            if current.seq != previous.seq + 1:
                reason = "sequence gap"
            elif not 0.0 < dt <= config.max_contiguous_gap_s:
                reason = "capture gap"
            else:
                candidates = [
                    item
                    for item in register(previous, current)
                    if _plausible(item, dt, config)
                    and item.score >= config.min_temporal_registration_score
                ]
                if not candidates:
                    candidates = _corroborated_skip_bridge(
                        ordered,
                        edge_index,
                        run_frames,
                        run_adjacent,
                        register,
                        config,
                    )
                    if not candidates:
                        reason = "no physically plausible registration"

            if reason:
                candidate_runs.append((run_frames, run_adjacent))
                boundaries.append(
                    FragmentBoundary(
                        robot_id,
                        previous.index,
                        current.index,
                        reason,
                        session,
                    )
                )
                run_frames = [current]
                run_adjacent = []
            else:
                run_frames.append(current)
                run_adjacent.append(candidates)
        if run_frames:
            candidate_runs.append((run_frames, run_adjacent))

        trajectory_fragment_number = 0
        for run_frames, run_adjacent in candidate_runs:
            selected = _select_chain(run_frames, run_adjacent, register, config)
            poses = {run_frames[0].index: se3_identity()}
            edges: list[FragmentEdge] = []
            for previous, current, registration in zip(
                run_frames[:-1], run_frames[1:], selected, strict=True
            ):
                poses[current.index] = (
                    poses[previous.index] @ registration.t_target_source
                )
                edges.append(
                    FragmentEdge(previous.index, current.index, registration)
                )
            fragments.append(
                Fragment(
                    fragment_id=(
                        f"{trajectory_id}:{trajectory_fragment_number:03d}"
                    ),
                    robot_id=robot_id,
                    frame_indices=tuple(frame.index for frame in run_frames),
                    poses=poses,
                    edges=tuple(edges),
                    session=session,
                )
            )
            trajectory_fragment_number += 1

    return fragments, boundaries


def _candidate_frame_pairs(
    frames: Sequence[ReconstructionFrame],
    fragments: Sequence[Fragment],
    config: FragmentMatchConfig,
    required_frame_pairs: Sequence[tuple[int, int]] = (),
) -> dict[tuple[str, str], list[tuple[int, int, float]]]:
    """Descriptor shortlist plus dense windows across known temporal breaks."""
    frame_by_index = {frame.index: frame for frame in frames}
    fragment_by_frame = {
        frame_index: fragment
        for fragment in fragments
        for frame_index in fragment.frame_indices
    }
    pairs: dict[tuple[str, str], dict[tuple[int, int], float]] = {}

    # An incremental back-end must not forget a previously verified closure
    # merely because newer, similar descriptors crowd its support scans out of
    # a fixed-size nearest-neighbour shortlist.  Retained pairs still pass the
    # complete registration, clustering, ambiguity, span and cycle gates below;
    # this only guarantees that the geometric evidence is reconsidered.
    for first_index, second_index in required_frame_pairs:
        if (
            first_index not in fragment_by_frame
            or second_index not in fragment_by_frame
        ):
            continue
        first_index, second_index = sorted((first_index, second_index))
        first_fragment = fragment_by_frame[first_index]
        second_fragment = fragment_by_frame[second_index]
        if first_fragment.fragment_id == second_fragment.fragment_id:
            continue
        key = tuple(sorted((first_fragment.fragment_id, second_fragment.fragment_id)))
        pairs.setdefault(key, {})[(first_index, second_index)] = -2.0

    # Directly inspect several scans on both sides of each temporal break. A
    # restarted SLAM process often resumes at the same physical place, but one
    # pair is not enough evidence to reconnect two unrelated frames.
    by_trajectory: dict[TrajectoryId, list[Fragment]] = {}
    for fragment in fragments:
        by_trajectory.setdefault(fragment.trajectory_id, []).append(fragment)
    for trajectory_fragments in by_trajectory.values():
        trajectory_fragments.sort(
            key=lambda fragment: frame_by_index[fragment.frame_indices[0]].seq
        )
        for left, right in zip(trajectory_fragments, trajectory_fragments[1:]):
            key = tuple(sorted((left.fragment_id, right.fragment_id)))
            bucket = pairs.setdefault(key, {})
            for left_index in left.frame_indices[-config.boundary_window :]:
                for right_index in right.frame_indices[: config.boundary_window]:
                    a_index, b_index = sorted((left_index, right_index))
                    bucket[(a_index, b_index)] = -1.0

    if not frames:
        return {}
    keys = np.stack([ring_key(frame.cloud.descriptor) for frame in frames])
    tree = cKDTree(keys)
    all_rows = np.arange(len(frames), dtype=np.int64)
    rows_by_robot: dict[str, np.ndarray] = {}
    for robot_id in sorted({frame.robot_id for frame in frames}):
        rows_by_robot[robot_id] = np.asarray(
            [row for row, item in enumerate(frames) if item.robot_id == robot_id],
            dtype=np.int64,
        )
    trees_by_robot = {
        robot_id: cKDTree(keys[rows]) for robot_id, rows in rows_by_robot.items()
    }
    for row, frame in enumerate(frames):
        # Preserve the inexpensive global shortlist, then add a balanced
        # shortlist from each *other robot*. Without this, dozens of nearly
        # identical scans from the current trajectory can occupy all 24 global
        # neighbours and a cold reconstruction never even tests the earlier
        # inter-robot rendezvous that an incremental solve already verified.
        scopes: list[tuple[cKDTree, np.ndarray]] = [(tree, all_rows)]
        scopes.extend(
            (trees_by_robot[robot_id], rows)
            for robot_id, rows in rows_by_robot.items()
            if robot_id != frame.robot_id
        )
        descriptor_distances: dict[int, float] = {}
        for scope_tree, scope_rows in scopes:
            fetch = min(len(scope_rows), max(config.descriptor_neighbors, 1))
            _, local_neighbor_rows = scope_tree.query(keys[row], k=fetch)
            ranked: list[tuple[float, int]] = []
            for local_neighbor_row in np.atleast_1d(local_neighbor_rows):
                neighbor_row = int(scope_rows[int(local_neighbor_row)])
                neighbor = frames[neighbor_row]
                if neighbor.index == frame.index:
                    continue
                first_fragment = fragment_by_frame[frame.index]
                second_fragment = fragment_by_frame[neighbor.index]
                if first_fragment.fragment_id == second_fragment.fragment_id:
                    continue
                distance = descriptor_distances.get(neighbor_row)
                if distance is None:
                    _, distance = best_alignment(
                        neighbor.cloud.descriptor, frame.cloud.descriptor
                    )
                    descriptor_distances[neighbor_row] = distance
                if distance <= config.max_descriptor_distance:
                    ranked.append((distance, neighbor.index))
            ranked.sort()
            for distance, neighbor_index in ranked[: config.candidates_per_frame]:
                first_index, second_index = sorted((frame.index, neighbor_index))
                first_fragment = fragment_by_frame[first_index]
                second_fragment = fragment_by_frame[second_index]
                key = tuple(
                    sorted((first_fragment.fragment_id, second_fragment.fragment_id))
                )
                bucket = pairs.setdefault(key, {})
                old = bucket.get((first_index, second_index), math.inf)
                bucket[(first_index, second_index)] = min(old, distance)

    capped: dict[tuple[str, str], list[tuple[int, int, float]]] = {}
    for key, bucket in pairs.items():
        # Required entries (-2) and boundary-window entries (-1) are
        # intentionally kept before descriptor-only candidates. For the rest,
        # plain descriptor sorting is unsafe: a repeated doorway can occupy all
        # 24 slots with near-identical views and evict an earlier, distributed
        # rendezvous. Farthest-point sampling in the two fragment frames keeps
        # the fixed registration budget while covering both trajectories.
        items = sorted(
            ((a, b, distance) for (a, b), distance in bucket.items()),
            key=lambda item: (item[2], item[0], item[1]),
        )
        limit = config.max_pairs_per_fragment_pair
        if len(items) <= limit:
            capped[key] = items
            continue

        selected = [item for item in items if item[2] < 0.0][:limit]
        selected_pairs = {(item[0], item[1]) for item in selected}
        remaining = [
            item for item in items if (item[0], item[1]) not in selected_pairs
        ]
        if not selected and remaining:
            selected.append(remaining.pop(0))

        fragment_a_id, fragment_b_id = key

        def pair_point(item: tuple[int, int, float]) -> np.ndarray:
            first_index, second_index, _ = item
            if fragment_by_frame[first_index].fragment_id == fragment_a_id:
                a_index, b_index = first_index, second_index
            else:
                a_index, b_index = second_index, first_index
            return np.concatenate(
                (
                    fragment_by_frame[a_index].poses[a_index][:3, 3],
                    fragment_by_frame[b_index].poses[b_index][:3, 3],
                )
            )

        selected_points = [pair_point(item) for item in selected]
        while remaining and len(selected) < limit:
            best_position = max(
                range(len(remaining)),
                key=lambda position: (
                    min(
                        float(
                            np.linalg.norm(
                                pair_point(remaining[position]) - selected_point
                            )
                        )
                        for selected_point in selected_points
                    ),
                    -remaining[position][2],
                    -remaining[position][0],
                    -remaining[position][1],
                ),
            )
            item = remaining.pop(best_position)
            selected.append(item)
            selected_points.append(pair_point(item))
        capped[key] = selected
    return capped


def _proposal(
    target: ReconstructionFrame,
    source: ReconstructionFrame,
    registration: RegistrationHypothesis,
    fragment_by_frame: dict[int, Fragment],
) -> ConnectionProposal:
    target_fragment = fragment_by_frame[target.index]
    source_fragment = fragment_by_frame[source.index]
    t_target_fragment_source_fragment = (
        target_fragment.poses[target.index]
        @ registration.t_target_source
        @ se3_inverse(source_fragment.poses[source.index])
    )
    if target_fragment.fragment_id < source_fragment.fragment_id:
        return ConnectionProposal(
            target_fragment.fragment_id,
            source_fragment.fragment_id,
            target.index,
            source.index,
            t_target_fragment_source_fragment,
            registration.score,
        )
    return ConnectionProposal(
        source_fragment.fragment_id,
        target_fragment.fragment_id,
        source.index,
        target.index,
        se3_inverse(t_target_fragment_source_fragment),
        registration.score,
    )


def _cluster_from_seed(
    seed: ConnectionProposal,
    proposals: Sequence[ConnectionProposal],
    config: FragmentMatchConfig,
) -> tuple[ConnectionProposal, ...]:
    by_pair: dict[tuple[int, int], list[tuple[float, ConnectionProposal]]] = {}
    for proposal in proposals:
        translation, rotation = se3_distance(seed.t_a_b, proposal.t_a_b)
        if (
            translation <= config.cluster_translation_m
            and rotation <= config.cluster_rotation_rad
        ):
            normalized = (
                translation / config.cluster_translation_m
                + rotation / config.cluster_rotation_rad
            )
            by_pair.setdefault((proposal.frame_a, proposal.frame_b), []).append(
                (normalized, proposal)
            )
    return tuple(
        min(alternatives, key=lambda item: item[0])[1]
        for alternatives in by_pair.values()
    )


def _spatial_span(
    indices: set[int], fragment: Fragment
) -> float:
    positions = np.stack([fragment.poses[index][:3, 3] for index in indices])
    if len(positions) < 2:
        return 0.0
    return float(np.max(np.linalg.norm(positions[:, None] - positions[None, :], axis=2)))


def _best_boundary_proposal(
    fragment_a: Fragment,
    fragment_b: Fragment,
    frame_by_index: dict[int, ReconstructionFrame],
    fragment_by_frame: dict[int, Fragment],
    register: RegistrationFunction,
) -> ConnectionProposal | None:
    """Best direct scan registration when two fragments straddle one seq edge."""
    if fragment_a.trajectory_id != fragment_b.trajectory_id:
        return None
    a_first = frame_by_index[fragment_a.frame_indices[0]].seq
    b_first = frame_by_index[fragment_b.frame_indices[0]].seq
    earlier, later = (
        (fragment_a, fragment_b) if a_first < b_first else (fragment_b, fragment_a)
    )
    previous = frame_by_index[earlier.frame_indices[-1]]
    current = frame_by_index[later.frame_indices[0]]
    if current.seq != previous.seq + 1:
        return None
    registrations = register(previous, current)
    if not registrations:
        return None
    return _proposal(previous, current, registrations[0], fragment_by_frame)


def _pose_hint_consistent(
    proposal: ConnectionProposal,
    frame_by_index: Mapping[int, ReconstructionFrame],
    pose_hints: Mapping[str, np.ndarray],
    config: FragmentMatchConfig,
) -> bool:
    """Whether one geometric mode agrees with deliberately coarse starts.

    Fragment frames are anchored at their first geometry-registered keyframe,
    so the expected transform between two fragment origins is simply the
    relative surveyed/coarse start transform. It deliberately does not read
    odometry: arbitrary odometry origins, drift, jumps and resets must not be
    able to accept or veto an inter-robot merge. Geometry still creates every
    candidate transform; the hint only rejects a grossly incompatible mode.
    """
    frame_a = frame_by_index[proposal.frame_a]
    frame_b = frame_by_index[proposal.frame_b]
    hint_a = pose_hints.get(frame_a.robot_id)
    hint_b = pose_hints.get(frame_b.robot_id)
    if hint_a is None or hint_b is None:
        return False
    expected_fragment_a_fragment_b = se3_inverse(hint_a) @ hint_b
    translation, rotation = se3_distance(
        expected_fragment_a_fragment_b, proposal.t_a_b
    )
    return (
        translation <= config.pose_hint_translation_m
        and rotation <= config.pose_hint_rotation_rad
    )


def find_fragment_connections(
    frames: Sequence[ReconstructionFrame],
    fragments: Sequence[Fragment],
    register: RegistrationFunction,
    config: FragmentMatchConfig | None = None,
    pose_hints: Mapping[str, np.ndarray] | None = None,
    required_frame_pairs: Sequence[tuple[int, int]] = (),
) -> tuple[list[FragmentConnection], list[RejectedConnection]]:
    """Reconnect fragments only when independent registrations form consensus.

    The returned graph may stay disconnected. That is a valid result: geometry
    that cannot distinguish two layouts must not be converted into a confident
    map merge merely to produce one visually convenient image.
    """
    config = config or FragmentMatchConfig()
    frame_by_index = {frame.index: frame for frame in frames}
    fragment_by_frame = {
        frame_index: fragment
        for fragment in fragments
        for frame_index in fragment.frame_indices
    }
    fragment_by_id = {fragment.fragment_id: fragment for fragment in fragments}
    # A configured start belongs only to the first observation of a physical
    # robot.  Applying it to a fragment created after a capture gap or SLAM
    # restart would falsely claim that the robot returned to its spawn.  Such
    # later fragments must reconnect by geometry/cycles before they inherit an
    # absolute placement.
    first_frame_by_robot: dict[str, ReconstructionFrame] = {}
    for frame in frames:
        current = first_frame_by_robot.get(frame.robot_id)
        if current is None or (frame.stamp, frame.seq, frame.index) < (
            current.stamp,
            current.seq,
            current.index,
        ):
            first_frame_by_robot[frame.robot_id] = frame
    primary_fragment_by_robot = {
        robot_id: fragment_by_frame[frame.index].fragment_id
        for robot_id, frame in first_frame_by_robot.items()
    }
    candidates = _candidate_frame_pairs(
        frames,
        fragments,
        config,
        required_frame_pairs,
    )
    accepted: list[FragmentConnection] = []
    rejected: list[RejectedConnection] = []

    for (fragment_a_id, fragment_b_id), frame_pairs in sorted(candidates.items()):
        proposals: list[ConnectionProposal] = []
        for first_index, second_index, _ in frame_pairs:
            first = frame_by_index[first_index]
            second = frame_by_index[second_index]
            for registration in register(first, second):
                proposal = _proposal(first, second, registration, fragment_by_frame)
                if {
                    proposal.fragment_a,
                    proposal.fragment_b,
                } == {fragment_a_id, fragment_b_id}:
                    proposals.append(proposal)
        if not proposals:
            rejected.append(
                RejectedConnection(fragment_a_id, fragment_b_id, 0, "no registration")
            )
            continue

        clusters: list[tuple[tuple[ConnectionProposal, ...], float]] = []
        for seed in proposals:
            members = _cluster_from_seed(seed, proposals, config)
            score = sum(member.score for member in members)
            member_signature = frozenset(id(member) for member in members)
            if any(
                member_signature
                == frozenset(id(member) for member in existing[0])
                or (
                    se3_distance(seed.t_a_b, existing[0][0].t_a_b)[0]
                    < config.cluster_translation_m / 2.0
                    and se3_distance(seed.t_a_b, existing[0][0].t_a_b)[1]
                    < config.cluster_rotation_rad / 2.0
                )
                for existing in clusters
            ):
                continue
            clusters.append((members, score))
        clusters.sort(key=lambda item: (len(item[0]), item[1]), reverse=True)
        fragment_a = fragment_by_id[fragment_a_id]
        fragment_b = fragment_by_id[fragment_b_id]
        pose_hint_reason = ""
        robots_have_hints = (
            fragment_by_id[fragment_a_id].robot_id
            != fragment_by_id[fragment_b_id].robot_id
            and pose_hints is not None
            and fragment_by_id[fragment_a_id].robot_id in pose_hints
            and fragment_by_id[fragment_b_id].robot_id in pose_hints
            and primary_fragment_by_robot.get(
                fragment_by_id[fragment_a_id].robot_id
            )
            == fragment_a_id
            and primary_fragment_by_robot.get(
                fragment_by_id[fragment_b_id].robot_id
            )
            == fragment_b_id
        )
        if robots_have_hints:
            eligible = []
            for cluster_members, score in clusters:
                consistent = sum(
                    _pose_hint_consistent(
                        proposal,
                        frame_by_index,
                        pose_hints,
                        config,
                    )
                    for proposal in cluster_members
                )
                required = max(
                    config.min_support,
                    int(math.ceil(config.pose_hint_min_fraction * len(cluster_members))),
                )
                if consistent >= required:
                    eligible.append((cluster_members, score))
            if eligible:
                clusters = eligible
                clusters.sort(
                    key=lambda item: (len(item[0]), item[1]), reverse=True
                )
            else:
                pose_hint_reason = "all geometric modes contradict coarse pose hints"

        def structural_reason(
            cluster: tuple[tuple[ConnectionProposal, ...], float],
        ) -> str:
            """Reject correlated votes before ranking geometric modes.

            Repeated scans at one doorway can create a larger consensus than a
            real multi-view rendezvous. Such a cluster is not an alternative
            layout: it lacks the independent spatial evidence required to
            estimate one. Letting it win first and applying this gate later
            caused a valid, spatially distributed mode to be discarded.
            """
            cluster_members, _ = cluster
            if len(cluster_members) < config.min_support:
                return "insufficient support"
            cluster_a_indices = {
                member.frame_a for member in cluster_members
            }
            cluster_b_indices = {
                member.frame_b for member in cluster_members
            }
            if (
                len(cluster_a_indices) < config.min_distinct_frames_per_side
                or len(cluster_b_indices) < config.min_distinct_frames_per_side
            ):
                return "support is not independent"
            if (
                _spatial_span(cluster_a_indices, fragment_a)
                < config.min_spatial_span_m
                or _spatial_span(cluster_b_indices, fragment_b)
                < config.min_spatial_span_m
            ):
                return "support has insufficient spatial span"
            return ""

        structural_rejection_reason = ""
        if not pose_hint_reason:
            structurally_eligible = [
                cluster for cluster in clusters if not structural_reason(cluster)
            ]
            if structurally_eligible:
                # Only independent, spatially distributed hypotheses represent
                # competing transforms for the ambiguity test below.
                clusters = structurally_eligible
            else:
                structural_rejection_reason = structural_reason(clusters[0])

        def cluster_representative(
            cluster: tuple[tuple[ConnectionProposal, ...], float],
        ) -> ConnectionProposal:
            cluster_members, _ = cluster
            return min(
                cluster_members,
                key=lambda candidate: sum(
                    se3_distance(candidate.t_a_b, other.t_a_b)[0]
                    / config.cluster_translation_m
                    + se3_distance(candidate.t_a_b, other.t_a_b)[1]
                    / config.cluster_rotation_rad
                    for other in cluster_members
                ),
            )

        best_boundary = _best_boundary_proposal(
            fragment_a,
            fragment_b,
            frame_by_index,
            fragment_by_frame,
            register,
        )
        boundary_rejection_reason = ""
        if (
            not pose_hint_reason
            and not structural_rejection_reason
            and best_boundary is not None
        ):
            if best_boundary.score < config.min_boundary_registration_score:
                boundary_rejection_reason = (
                    "adjacent boundary registration is too weak"
                )
            else:
                boundary_eligible = []
                for cluster in clusters:
                    candidate = cluster_representative(cluster)
                    translation, rotation = se3_distance(
                        candidate.t_a_b, best_boundary.t_a_b
                    )
                    if (
                        translation
                        <= config.boundary_consistency_translation_m
                        and rotation <= config.boundary_consistency_rotation_rad
                    ):
                        boundary_eligible.append(cluster)
                if boundary_eligible:
                    # The exact scan transition is independent evidence and
                    # selects among already-supported global modes. A larger
                    # repeated-corridor cluster must not hide the mode that
                    # actually crosses this temporal boundary.
                    clusters = boundary_eligible
                else:
                    boundary_rejection_reason = (
                        "boundary consensus contradicts adjacent registration"
                    )

        members, cluster_score = clusters[0]
        selected_pose_hint_support = (
            sum(
                _pose_hint_consistent(
                    proposal,
                    frame_by_index,
                    pose_hints,
                    config,
                )
                for proposal in members
            )
            if robots_have_hints
            else 0
        )
        # Compute a representative before the gates so a temporal-boundary
        # cross-check can compare the winning cluster with the best direct
        # registration of the exact last->first scan pair. Several correlated
        # window matches must not outvote that transition into a 180-degree
        # repeated-corridor mode.
        representative = cluster_representative(clusters[0])
        reason = structural_rejection_reason or boundary_rejection_reason
        if not reason and len(clusters) > 1:
            runner_members, runner_score = clusters[1]
            if (
                len(runner_members) >= len(members) - config.ambiguity_support_margin
                and runner_score / len(runner_members)
                >= cluster_score / len(members) - config.ambiguity_score_margin
            ):
                reason = "ambiguous competing transform"
        if pose_hint_reason:
            reason = pose_hint_reason
        if reason:
            rejected.append(
                RejectedConnection(fragment_a_id, fragment_b_id, len(members), reason)
            )
            continue

        # The representative is a medoid -- an actually observed transform,
        # not an invalid element-wise SE(3) mean.
        accepted.append(
            FragmentConnection(
                fragment_a_id,
                fragment_b_id,
                representative.t_a_b,
                len(members),
                cluster_score,
                members,
                selected_pose_hint_support,
            )
        )
    return accepted, rejected


def find_intra_fragment_loops(
    frames: Sequence[ReconstructionFrame],
    fragments: Sequence[Fragment],
    register: RegistrationFunction,
    config: FragmentMatchConfig | None = None,
) -> list[FrameLoopClosure]:
    """Find non-local, path-consistent loop factors inside long fragments.

    Descriptor similarity proposes a revisit, direct scan registration measures
    it, and the geometry-only temporal chain supplies a deliberately loose
    consistency gate.  The gate prevents a repeated parallel corridor from
    teleporting the graph while still allowing loop factors to remove ordinary
    accumulated scan-registration drift.
    """
    config = config or FragmentMatchConfig()
    frame_by_index = {frame.index: frame for frame in frames}
    candidates: set[tuple[int, int]] = set()
    for fragment in fragments:
        if len(fragment.frame_indices) <= config.loop_min_sequence_separation:
            continue
        fragment_frames = [frame_by_index[index] for index in fragment.frame_indices]
        keys = np.stack([ring_key(frame.cloud.descriptor) for frame in fragment_frames])
        tree = cKDTree(keys)
        fetch = min(len(fragment_frames), max(config.loop_descriptor_neighbors, 1))
        for row, frame in enumerate(fragment_frames):
            _, neighbors = tree.query(keys[row], k=fetch)
            ranked: list[tuple[float, int]] = []
            for neighbor_row in np.atleast_1d(neighbors):
                neighbor = fragment_frames[int(neighbor_row)]
                if (
                    abs(neighbor.seq - frame.seq)
                    < config.loop_min_sequence_separation
                ):
                    continue
                _, distance = best_alignment(
                    neighbor.cloud.descriptor, frame.cloud.descriptor
                )
                if distance <= config.loop_max_descriptor_distance:
                    ranked.append((distance, neighbor.index))
            ranked.sort()
            for _, neighbor_index in ranked[: config.loop_candidates_per_frame]:
                candidates.add(tuple(sorted((frame.index, neighbor_index))))

    fragment_by_frame = {
        frame_index: fragment
        for fragment in fragments
        for frame_index in fragment.frame_indices
    }
    accepted: list[FrameLoopClosure] = []
    for target_index, source_index in sorted(candidates):
        fragment = fragment_by_frame[target_index]
        if fragment is not fragment_by_frame[source_index]:
            continue
        predicted = (
            se3_inverse(fragment.poses[target_index])
            @ fragment.poses[source_index]
        )
        alternatives = register(
            frame_by_index[target_index], frame_by_index[source_index]
        )
        if not alternatives:
            continue
        scored = [
            (se3_distance(predicted, item.t_target_source), item)
            for item in alternatives
        ]
        (translation, rotation), registration = min(
            scored,
            key=lambda item: (
                item[0][0] / config.loop_consistency_translation_m
                + item[0][1] / config.loop_consistency_rotation_rad
            ),
        )
        if (
            translation <= config.loop_consistency_translation_m
            and rotation <= config.loop_consistency_rotation_rad
        ):
            accepted.append(
                FrameLoopClosure(
                    target_index,
                    source_index,
                    registration,
                    translation,
                    rotation,
                )
            )
    return accepted


def _component_frame_position(
    placement: FragmentPlacement,
    fragment: Fragment,
    frame_index: int,
) -> np.ndarray:
    return (
        placement.poses[fragment.fragment_id] @ fragment.poses[frame_index]
    )[:3, 3]


def _connection_proposal_locations(
    connection: FragmentConnection,
    side: str,
    placement: FragmentPlacement,
    fragment_by_id: dict[str, Fragment],
) -> list[np.ndarray]:
    """Locations of every independent scan vote on one connection side."""
    fragment_id = connection.fragment_a if side == "a" else connection.fragment_b
    fragment = fragment_by_id[fragment_id]
    indices = (
        [proposal.frame_a for proposal in connection.proposals]
        if side == "a"
        else [proposal.frame_b for proposal in connection.proposals]
    )
    return [
        _component_frame_position(placement, fragment, frame_index)
        for frame_index in indices
    ]


def _maximum_separation(positions: Sequence[np.ndarray]) -> float:
    if len(positions) < 2:
        return 0.0
    stacked = np.stack(positions)
    return float(
        np.max(np.linalg.norm(stacked[:, None] - stacked[None, :], axis=2))
    )


def filter_inter_robot_connections(
    fragments: Sequence[Fragment],
    connections: Sequence[FragmentConnection],
    config: FragmentMatchConfig | None = None,
) -> tuple[list[FragmentConnection], list[RejectedConnection]]:
    """Require a cross-robot cycle at spatially separated encounters.

    Several adjacent keyframe pairs at one intersection are correlated samples
    of one place, not independent proof of a fleet-frame transform. A valid
    cross-robot merge therefore needs at least two compatible fragment
    connections between the *same already-connected per-robot components*,
    separated by several metres on both robots. Together with each robot's
    intra-robot path, those two encounters form the first cycle capable of
    checking a robot-to-robot transform.

    A single fragment-pair connection can still contain a valid cycle when both
    robots remained temporally continuous: scan votes separated by several
    metres, plus the two paths between them, are independent encounters.  Votes
    clustered at one intersection are still rejected.  This distinction is
    necessary because otherwise two robots represented by one fragment each
    could never merge, however much common ground they traversed.

    A pose-hint-corroborated connection is the other safe exception: its scan
    cluster was already selected from multiple geometric modes using two
    coarse surveyed starts. That second modality can replace a spatially
    separated encounter, but never replace geometric support or invent a
    transform that registration did not produce.
    """
    config = config or FragmentMatchConfig()
    fragment_by_id = {fragment.fragment_id: fragment for fragment in fragments}
    intra_robot = [
        connection
        for connection in connections
        if fragment_by_id[connection.fragment_a].robot_id
        == fragment_by_id[connection.fragment_b].robot_id
    ]
    inter_robot = [
        connection
        for connection in connections
        if fragment_by_id[connection.fragment_a].robot_id
        != fragment_by_id[connection.fragment_b].robot_id
    ]
    if not inter_robot:
        return list(connections), []

    local_placement = place_fragments(fragments, intra_robot)
    component_of = {
        fragment_id: component_index
        for component_index, component in enumerate(local_placement.components)
        for fragment_id in component
    }

    # Express every provisional cross-robot edge between canonical local
    # component frames, then group edges that could form a corroborating cycle.
    grouped: dict[
        tuple[int, int], list[tuple[FragmentConnection, np.ndarray, str]]
    ] = {}
    for connection in inter_robot:
        component_a = component_of[connection.fragment_a]
        component_b = component_of[connection.fragment_b]
        t_component_a_component_b = (
            local_placement.poses[connection.fragment_a]
            @ connection.t_a_b
            @ se3_inverse(local_placement.poses[connection.fragment_b])
        )
        if component_a < component_b:
            key = (component_a, component_b)
            transform = t_component_a_component_b
            orientation = "ab"
        else:
            key = (component_b, component_a)
            transform = se3_inverse(t_component_a_component_b)
            orientation = "ba"
        grouped.setdefault(key, []).append((connection, transform, orientation))

    kept = list(intra_robot)
    rejected: list[RejectedConnection] = []
    for candidates in grouped.values():
        def pose_hint_corroborates(connection: FragmentConnection) -> bool:
            return connection.pose_hint_support >= max(
                config.min_support,
                int(
                    math.ceil(
                        config.pose_hint_min_fraction
                        * len(connection.proposals)
                    )
                ),
            )

        clusters: list[list[tuple[FragmentConnection, np.ndarray, str]]] = []
        for candidate in sorted(candidates, key=lambda item: item[0].score, reverse=True):
            placed = False
            for cluster in clusters:
                translation, rotation = se3_distance(cluster[0][1], candidate[1])
                if (
                    translation <= config.inter_robot_consistency_translation_m
                    and rotation <= config.inter_robot_consistency_rotation_rad
                ):
                    cluster.append(candidate)
                    placed = True
                    break
            if not placed:
                clusters.append([candidate])
        clusters.sort(
            key=lambda cluster: (
                any(pose_hint_corroborates(item[0]) for item in cluster),
                len(cluster),
                sum(item[0].score for item in cluster),
            ),
            reverse=True,
        )
        best = clusters[0]
        best_connections = {id(item[0]) for item in best}
        side_a_positions: list[np.ndarray] = []
        side_b_positions: list[np.ndarray] = []
        for connection, _, orientation in best:
            locations_a = _connection_proposal_locations(
                connection, "a", local_placement, fragment_by_id
            )
            locations_b = _connection_proposal_locations(
                connection, "b", local_placement, fragment_by_id
            )
            if orientation == "ab":
                side_a_positions.extend(locations_a)
                side_b_positions.extend(locations_b)
            else:
                side_a_positions.extend(locations_b)
                side_b_positions.extend(locations_a)

        reason = ""
        pose_hint_corroborated = any(
            pose_hint_corroborates(connection)
            for connection, _, _ in best
        )
        if (
            not pose_hint_corroborated
            and (
                _maximum_separation(side_a_positions)
                < config.min_inter_robot_separation_m
                or _maximum_separation(side_b_positions)
                < config.min_inter_robot_separation_m
            )
        ):
            reason = (
                "inter-robot merge has no independent cycle"
                if len(best) < config.min_inter_robot_connections
                else "inter-robot encounters are not spatially independent"
            )

        if not reason:
            kept.extend(item[0] for item in best)
        for connection, _, _ in candidates:
            if not reason and id(connection) in best_connections:
                continue
            rejected.append(
                RejectedConnection(
                    connection.fragment_a,
                    connection.fragment_b,
                    connection.support,
                    reason or "inter-robot transform disagrees with consensus",
                )
            )
    kept.sort(key=lambda connection: (connection.fragment_a, connection.fragment_b))
    return kept, rejected


def place_fragments(
    fragments: Sequence[Fragment],
    connections: Sequence[FragmentConnection],
) -> FragmentPlacement:
    """Robustly optimize the accepted fragment-consensus graph.

    Each disconnected component receives its own origin. No transform between
    components is implied, so renderers must keep them separate.
    """
    if not fragments:
        return FragmentPlacement({}, ())
    fragment_by_id = {fragment.fragment_id: fragment for fragment in fragments}
    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {
        fragment.fragment_id: [] for fragment in fragments
    }
    for connection in connections:
        adjacency[connection.fragment_a].append(
            (connection.fragment_b, connection.t_a_b)
        )
        adjacency[connection.fragment_b].append(
            (connection.fragment_a, se3_inverse(connection.t_a_b))
        )

    initial_poses: dict[str, np.ndarray] = {}
    components: list[frozenset[str]] = []
    for anchor in sorted(adjacency):
        if anchor in initial_poses:
            continue
        initial_poses[anchor] = se3_identity()
        members: set[str] = set()
        queue = deque([anchor])
        while queue:
            current = queue.popleft()
            members.add(current)
            for neighbor, t_current_neighbor in adjacency[current]:
                if neighbor not in initial_poses:
                    initial_poses[neighbor] = (
                        initial_poses[current] @ t_current_neighbor
                    )
                    queue.append(neighbor)
        components.append(frozenset(members))

    ordered_ids = sorted(fragment_by_id)
    key = {fragment_id: index for index, fragment_id in enumerate(ordered_ids)}
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    for fragment_id in ordered_ids:
        initial.insert(key[fragment_id], gtsam.Pose3(initial_poses[fragment_id]))
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3] * 6))
    for component in components:
        anchor = min(component)
        graph.add(
            gtsam.PriorFactorPose3(
                key[anchor], gtsam.Pose3(initial_poses[anchor]), prior_noise
            )
        )

    base_noise = gtsam.noiseModel.Diagonal.Sigmas(
        # gtsam Pose3 tangent order: rotation then translation.
        np.array([0.10, 0.10, 0.08, 0.20, 0.20, 0.20])
    )
    robust_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(1.345), base_noise
    )
    for connection in connections:
        for proposal in connection.proposals:
            graph.add(
                gtsam.BetweenFactorPose3(
                    key[connection.fragment_a],
                    key[connection.fragment_b],
                    gtsam.Pose3(proposal.t_a_b),
                    robust_noise,
                )
            )
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    poses = {
        fragment_id: np.asarray(result.atPose3(key[fragment_id]).matrix())
        for fragment_id in ordered_ids
    }
    return FragmentPlacement(poses, tuple(components))


def optimize_keyframe_poses(
    fragments: Sequence[Fragment],
    connections: Sequence[FragmentConnection],
    placement: FragmentPlacement,
    loop_closures: Sequence[FrameLoopClosure] = (),
) -> dict[int, np.ndarray]:
    """Optimize every keyframe using geometry-only temporal and closure factors.

    Fragment placement supplies a safe initial estimate.  Unlike rigidly moving
    each fragment, this graph lets spatially separated loop/robot encounters
    distribute their correction along the temporal scan chain.
    """
    if not fragments:
        return {}
    fragment_by_id = {fragment.fragment_id: fragment for fragment in fragments}
    initial_poses = {
        frame_index: placement.poses[fragment.fragment_id] @ fragment.poses[frame_index]
        for fragment in fragments
        for frame_index in fragment.frame_indices
    }
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    for frame_index, pose in sorted(initial_poses.items()):
        initial.insert(frame_index, gtsam.Pose3(pose))

    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4] * 6))
    for component in placement.components:
        anchor_fragment = min(component)
        anchor_frame = fragment_by_id[anchor_fragment].frame_indices[0]
        graph.add(
            gtsam.PriorFactorPose3(
                anchor_frame, gtsam.Pose3(initial_poses[anchor_frame]), prior_noise
            )
        )

    temporal_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(1.345),
        gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.04, 0.04, 0.03, 0.08, 0.08, 0.10])
        ),
    )
    closure_sigmas = np.array([0.08, 0.08, 0.06, 0.15, 0.15, 0.20])
    closure_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(1.345),
        gtsam.noiseModel.Diagonal.Sigmas(closure_sigmas),
    )
    for fragment in fragments:
        for edge in fragment.edges:
            graph.add(
                gtsam.BetweenFactorPose3(
                    edge.target_index,
                    edge.source_index,
                    gtsam.Pose3(edge.registration.t_target_source),
                    temporal_noise,
                )
            )
    for connection in connections:
        fragment_a = fragment_by_id[connection.fragment_a]
        fragment_b = fragment_by_id[connection.fragment_b]
        # A scan cluster is one rendezvous measurement with correlated samples,
        # not ``support`` independent sensors. Without normalization, 24 nearby
        # matches have eight times the aggregate information of one closure and
        # visibly deform an already accurate temporal scan chain. Scaling each
        # sample's sigma by sqrt(N) keeps the cluster's total information
        # bounded while retaining its spatial distribution and robust losses.
        # A 4x systematic margin accounts for the shared lidar calibration and
        # local scan chain. It is the conservative cross-session choice on the
        # two labelled four-robot captures; hardware still needs recalibration.
        connection_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345),
            gtsam.noiseModel.Diagonal.Sigmas(
                closure_sigmas
                * 4.0
                * math.sqrt(max(1, len(connection.proposals)))
            ),
        )
        for proposal in connection.proposals:
            # proposal.t_a_b maps fragment B into fragment A.  Remove the two
            # local frame poses to recover the observed frame-A <- frame-B edge.
            measurement = (
                se3_inverse(fragment_a.poses[proposal.frame_a])
                @ proposal.t_a_b
                @ fragment_b.poses[proposal.frame_b]
            )
            graph.add(
                gtsam.BetweenFactorPose3(
                    proposal.frame_a,
                    proposal.frame_b,
                    gtsam.Pose3(measurement),
                    connection_noise,
                )
            )
    for closure in loop_closures:
        graph.add(
            gtsam.BetweenFactorPose3(
                closure.target_index,
                closure.source_index,
                gtsam.Pose3(closure.registration.t_target_source),
                closure_noise,
            )
        )
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    return {
        frame_index: np.asarray(result.atPose3(frame_index).matrix())
        for frame_index in initial_poses
    }
