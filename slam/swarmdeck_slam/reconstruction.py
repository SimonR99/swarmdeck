"""Build locally consistent fragments without using recorded odometry poses.

Temporal order and timestamps are observations, not odometry. They tell us
which scans were captured near one another in time and bound how fast a ground
robot could have moved; they do not provide a transform. Every transform in a
fragment comes from :func:`swarmdeck_slam.odom_free.register_clouds`.

Fragment boundaries are intentional outputs. Long capture gaps, missing
registration, and impossible motion split the chain instead of fabricating an
edge. A later global stage may reconnect fragments, but only with several
independent geometric correspondences.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Sequence

import gtsam
import numpy as np
from scipy.spatial import cKDTree

from swarmdeck_slam.descriptors import best_alignment, ring_key
from swarmdeck_slam.odom_free import PreparedCloud, RegistrationHypothesis
from swarmdeck_slam.types import se3_distance, se3_identity, se3_inverse


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


@dataclass(frozen=True, slots=True)
class ReconstructionFrame:
    """A cloud plus non-pose capture metadata used during reconstruction."""

    index: int
    robot_id: str
    seq: int
    stamp: float
    cloud: PreparedCloud


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


@dataclass(frozen=True, slots=True)
class FragmentBoundary:
    robot_id: str
    previous_index: int
    next_index: int
    reason: str


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
    ambiguity_score_margin: float = 0.15
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


def _plausible(
    hypothesis: RegistrationHypothesis, dt: float, config: TemporalConfig
) -> bool:
    transform = hypothesis.t_target_source
    translation = float(np.linalg.norm(transform[:2, 3]))
    max_translation = config.translation_slack_m + config.max_linear_speed_mps * dt
    max_yaw = min(math.pi, config.yaw_slack_rad + config.max_yaw_rate_rad_s * dt)
    return translation <= max_translation and abs(_yaw(transform)) <= max_yaw


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


def _unary_cost(
    hypothesis: RegistrationHypothesis, dt: float, config: TemporalConfig
) -> float:
    speed = float(np.linalg.norm(hypothesis.t_target_source[:2, 3])) / dt
    return (
        -config.registration_weight * hypothesis.score
        + config.path_speed_weight * speed
        + config.path_yaw_weight * abs(_yaw(hypothesis.t_target_source))
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
            _unary_cost(item, frames[1].stamp - frames[0].stamp, config)
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
        for second in adjacent[edge_index]:
            unary = _unary_cost(second, dt_second, config)
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
    by_robot: dict[str, list[ReconstructionFrame]] = {}
    for frame in frames:
        by_robot.setdefault(frame.robot_id, []).append(frame)

    for robot_id in sorted(by_robot):
        ordered = sorted(by_robot[robot_id], key=lambda item: (item.seq, item.stamp))
        candidate_runs: list[
            tuple[list[ReconstructionFrame], list[list[RegistrationHypothesis]]]
        ] = []
        run_frames = [ordered[0]] if ordered else []
        run_adjacent: list[list[RegistrationHypothesis]] = []
        for previous, current in zip(ordered, ordered[1:]):
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
                    reason = "no physically plausible registration"

            if reason:
                candidate_runs.append((run_frames, run_adjacent))
                boundaries.append(
                    FragmentBoundary(robot_id, previous.index, current.index, reason)
                )
                run_frames = [current]
                run_adjacent = []
            else:
                run_frames.append(current)
                run_adjacent.append(candidates)
        if run_frames:
            candidate_runs.append((run_frames, run_adjacent))

        robot_fragment_number = 0
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
                    fragment_id=f"{robot_id}:{robot_fragment_number:03d}",
                    robot_id=robot_id,
                    frame_indices=tuple(frame.index for frame in run_frames),
                    poses=poses,
                    edges=tuple(edges),
                )
            )
            robot_fragment_number += 1

    return fragments, boundaries


def _candidate_frame_pairs(
    frames: Sequence[ReconstructionFrame],
    fragments: Sequence[Fragment],
    config: FragmentMatchConfig,
) -> dict[tuple[str, str], list[tuple[int, int, float]]]:
    """Descriptor shortlist plus dense windows across known temporal breaks."""
    frame_by_index = {frame.index: frame for frame in frames}
    fragment_by_frame = {
        frame_index: fragment
        for fragment in fragments
        for frame_index in fragment.frame_indices
    }
    pairs: dict[tuple[str, str], dict[tuple[int, int], float]] = {}

    # Directly inspect several scans on both sides of each temporal break. A
    # restarted SLAM process often resumes at the same physical place, but one
    # pair is not enough evidence to reconnect two unrelated frames.
    by_robot: dict[str, list[Fragment]] = {}
    for fragment in fragments:
        by_robot.setdefault(fragment.robot_id, []).append(fragment)
    for robot_fragments in by_robot.values():
        robot_fragments.sort(
            key=lambda fragment: frame_by_index[fragment.frame_indices[0]].seq
        )
        for left, right in zip(robot_fragments, robot_fragments[1:]):
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
    fetch = min(len(frames), max(config.descriptor_neighbors, 1))
    for row, frame in enumerate(frames):
        _, neighbor_rows = tree.query(keys[row], k=fetch)
        ranked: list[tuple[float, int]] = []
        for neighbor_row in np.atleast_1d(neighbor_rows):
            neighbor = frames[int(neighbor_row)]
            if neighbor.index == frame.index:
                continue
            first_fragment = fragment_by_frame[frame.index]
            second_fragment = fragment_by_frame[neighbor.index]
            if first_fragment.fragment_id == second_fragment.fragment_id:
                continue
            _, distance = best_alignment(
                neighbor.cloud.descriptor, frame.cloud.descriptor
            )
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
        # Boundary-window entries have distance -1 and are intentionally kept
        # before descriptor-only candidates when the per-pair cap is reached.
        items = sorted(
            ((a, b, distance) for (a, b), distance in bucket.items()),
            key=lambda item: (item[2], item[0], item[1]),
        )
        capped[key] = items[: config.max_pairs_per_fragment_pair]
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
    if fragment_a.robot_id != fragment_b.robot_id:
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


def find_fragment_connections(
    frames: Sequence[ReconstructionFrame],
    fragments: Sequence[Fragment],
    register: RegistrationFunction,
    config: FragmentMatchConfig | None = None,
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
    candidates = _candidate_frame_pairs(frames, fragments, config)
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
        members, cluster_score = clusters[0]
        a_indices = {member.frame_a for member in members}
        b_indices = {member.frame_b for member in members}
        fragment_a = fragment_by_id[fragment_a_id]
        fragment_b = fragment_by_id[fragment_b_id]
        # Compute a representative before the gates so a temporal-boundary
        # cross-check can compare the winning cluster with the best direct
        # registration of the exact last->first scan pair. Several correlated
        # window matches must not outvote that transition into a 180-degree
        # repeated-corridor mode.
        representative = min(
            members,
            key=lambda candidate: sum(
                se3_distance(candidate.t_a_b, other.t_a_b)[0]
                / config.cluster_translation_m
                + se3_distance(candidate.t_a_b, other.t_a_b)[1]
                / config.cluster_rotation_rad
                for other in members
            ),
        )
        best_boundary = _best_boundary_proposal(
            fragment_a,
            fragment_b,
            frame_by_index,
            fragment_by_frame,
            register,
        )
        reason = ""
        if (
            best_boundary is not None
            and best_boundary.score < config.min_boundary_registration_score
        ):
            reason = "adjacent boundary registration is too weak"
        if len(members) < config.min_support:
            reason = "insufficient support"
        elif (
            len(a_indices) < config.min_distinct_frames_per_side
            or len(b_indices) < config.min_distinct_frames_per_side
        ):
            reason = "support is not independent"
        elif (
            _spatial_span(a_indices, fragment_a) < config.min_spatial_span_m
            or _spatial_span(b_indices, fragment_b) < config.min_spatial_span_m
        ):
            reason = "support has insufficient spatial span"
        elif len(clusters) > 1:
            runner_members, runner_score = clusters[1]
            if (
                len(runner_members) >= len(members) - config.ambiguity_support_margin
                and runner_score >= cluster_score - config.ambiguity_score_margin
            ):
                reason = "ambiguous competing transform"
        if not reason and best_boundary is not None:
            translation, rotation = se3_distance(
                representative.t_a_b, best_boundary.t_a_b
            )
            if (
                translation > config.boundary_consistency_translation_m
                or rotation > config.boundary_consistency_rotation_rad
            ):
                reason = "boundary consensus contradicts adjacent registration"
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
        if (
            _maximum_separation(side_a_positions)
            < config.min_inter_robot_separation_m
            or _maximum_separation(side_b_positions)
            < config.min_inter_robot_separation_m
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
    closure_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(1.345),
        gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.08, 0.08, 0.06, 0.15, 0.15, 0.20])
        ),
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
                    closure_noise,
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
