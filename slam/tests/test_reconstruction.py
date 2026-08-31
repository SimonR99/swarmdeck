import math

import numpy as np

from swarmdeck_slam.odom_free import PreparedCloud, RegistrationHypothesis
from swarmdeck_slam.reconstruction import (
    ConnectionProposal,
    Fragment,
    FragmentConnection,
    FragmentMatchConfig,
    ReconstructionFrame,
    TemporalConfig,
    build_temporal_fragments,
    filter_inter_robot_connections,
    find_fragment_connections,
    place_fragments,
)
from swarmdeck_slam.types import se3_identity, se3_inverse


def _yaw_pose(x: float, yaw: float) -> np.ndarray:
    pose = se3_identity()
    cosine, sine = math.cos(yaw), math.sin(yaw)
    pose[:2, :2] = [[cosine, -sine], [sine, cosine]]
    pose[0, 3] = x
    return pose


def _frame(
    index: int,
    stamp: float,
    *,
    x: float | None = None,
    yaw: float = 0.0,
    robot_id: str = "robot",
    seq: int | None = None,
) -> ReconstructionFrame:
    empty = np.empty((0, 3), dtype=np.float64)
    cloud = PreparedCloud(empty, np.ones((20, 60), dtype=np.uint8), np.zeros((2, 2)))
    odom = None if x is None else _yaw_pose(x, yaw)
    return ReconstructionFrame(
        index,
        robot_id,
        index if seq is None else seq,
        stamp,
        cloud,
        t_odom_base=odom,
    )


def _hypothesis(yaw: float, x: float, score: float) -> RegistrationHypothesis:
    transform = se3_identity()
    c, s = math.cos(yaw), math.sin(yaw)
    transform[:2, :2] = [[c, -s], [s, c]]
    transform[0, 3] = x
    return RegistrationHypothesis(transform, yaw, 0.1, 0.8, 0.9, 0.05, 0.05, 500, score)


def _single_frame_fragment(
    fragment_id: str, robot_id: str, frame_index: int
) -> Fragment:
    return Fragment(
        fragment_id,
        robot_id,
        (frame_index,),
        {frame_index: se3_identity()},
        (),
    )


def _connection(
    fragment_a: Fragment,
    fragment_b: Fragment,
    transform: np.ndarray,
) -> FragmentConnection:
    frame_a = fragment_a.frame_indices[0]
    frame_b = fragment_b.frame_indices[0]
    proposal = ConnectionProposal(
        fragment_a.fragment_id,
        fragment_b.fragment_id,
        frame_a,
        frame_b,
        transform,
        0.9,
    )
    return FragmentConnection(
        fragment_a.fragment_id,
        fragment_b.fragment_id,
        transform,
        3,
        2.7,
        (proposal,),
    )


def test_temporal_fragments_reject_impossible_flip_and_split_at_gap() -> None:
    frames = [_frame(0, 0.0), _frame(1, 2.0), _frame(2, 4.0), _frame(3, 40.0)]
    correct = _hypothesis(0.0, 0.5, 0.8)
    flip = _hypothesis(math.pi, 0.5, 0.9)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        if source.index - target.index == 1:
            return [flip, correct]
        return [_hypothesis(0.0, 1.0, 0.8)]

    fragments, boundaries = build_temporal_fragments(
        frames,
        register,
        TemporalConfig(max_contiguous_gap_s=10.0, max_yaw_rate_rad_s=0.5),
    )

    assert [fragment.frame_indices for fragment in fragments] == [(0, 1, 2), (3,)]
    assert boundaries[0].reason == "capture gap"
    assert all(abs(edge.registration.yaw_prior) < 1e-6 for edge in fragments[0].edges)
    assert np.allclose(fragments[0].poses[2][:3, 3], [1.0, 0.0, 0.0])


def test_vertical_hop_is_not_physical_ground_robot_motion() -> None:
    frames = [_frame(0, 0.0), _frame(1, 2.0)]
    jump = se3_identity()
    jump[2, 3] = 3.0

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        return [
            RegistrationHypothesis(jump, 0.0, 0.1, 0.8, 0.9, 0.05, 0.05, 500, 0.9)
        ]

    fragments, boundaries = build_temporal_fragments(frames, register)
    assert [fragment.frame_indices for fragment in fragments] == [(0,), (1,)]
    assert boundaries[0].reason == "no physically plausible registration"


def test_two_skip_cycles_rescue_one_missing_turn_registration() -> None:
    frames = [_frame(index, float(index)) for index in range(4)]
    step = _hypothesis(0.0, 0.5, 0.8)
    skip = _hypothesis(0.0, 1.0, 0.75)
    false_flip = _hypothesis(math.pi, 0.5, 0.9)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        pair = (target.index, source.index)
        if pair == (1, 2):
            return [false_flip]
        if pair in {(0, 2), (1, 3)}:
            return [skip]
        if pair in {(0, 1), (2, 3)}:
            return [step]
        return []

    fragments, boundaries = build_temporal_fragments(frames, register)

    assert boundaries == []
    assert [fragment.frame_indices for fragment in fragments] == [(0, 1, 2, 3)]
    assert np.allclose(fragments[0].poses[3][:3, 3], [1.5, 0.0, 0.0])


def test_one_sided_skip_evidence_does_not_bridge_a_bad_pair() -> None:
    frames = [_frame(index, float(index)) for index in range(4)]
    step = _hypothesis(0.0, 0.5, 0.8)
    skip = _hypothesis(0.0, 1.0, 0.75)
    false_flip = _hypothesis(math.pi, 0.5, 0.9)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        pair = (target.index, source.index)
        if pair == (1, 2):
            return [false_flip]
        if pair == (0, 2):
            return [skip]
        if pair in {(0, 1), (2, 3)}:
            return [step]
        return []

    fragments, boundaries = build_temporal_fragments(frames, register)

    assert [fragment.frame_indices for fragment in fragments] == [(0, 1), (2, 3)]
    assert boundaries[0].reason == "no physically plausible registration"


def test_odom_hint_prefers_the_matching_yaw_mode() -> None:
    """A kinematically plausible odom hop may break a 180-degree alias.

    Both modes pass the physical gate. Geometry scores the flip higher; a
    hop that itself turned around selects that flip. Pair registration still
    never sees the pose -- only the already-computed modes are ranked.
    """
    frames = [
        _frame(0, 0.0, x=0.0, yaw=0.0),
        _frame(1, 5.0, x=0.5, yaw=math.pi),
    ]
    correct = _hypothesis(0.0, 0.5, 0.8)
    flip = _hypothesis(math.pi, 0.5, 0.9)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        return [flip, correct]

    fragments, boundaries = build_temporal_fragments(frames, register)
    assert boundaries == []
    assert len(fragments) == 1
    assert abs(fragments[0].edges[0].registration.yaw_prior - math.pi) < 1e-6


def test_catastrophic_odom_hop_is_ignored() -> None:
    """A 20 m jump is not robot motion; geometry keeps the zero-yaw mode."""
    frames = [
        _frame(0, 0.0, x=0.0, yaw=0.0),
        _frame(1, 5.0, x=20.5, yaw=math.pi),
    ]
    correct = _hypothesis(0.0, 0.5, 0.8)
    flip = _hypothesis(math.pi, 0.5, 0.9)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        return [flip, correct]

    fragments, _ = build_temporal_fragments(frames, register)
    assert abs(fragments[0].edges[0].registration.yaw_prior) < 1e-6


def test_temporal_fragments_never_chain_across_sessions() -> None:
    first = _frame(0, 0.0)
    second = ReconstructionFrame(
        1,
        first.robot_id,
        0,
        1.0,
        first.cloud,
        "next-run",
    )
    calls: list[tuple[int, int]] = []

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        calls.append((target.index, source.index))
        return [_hypothesis(0.0, 0.5, 0.9)]

    fragments, boundaries = build_temporal_fragments([first, second], register)

    assert calls == []
    assert boundaries == []
    assert [fragment.frame_indices for fragment in fragments] == [(0,), (1,)]
    assert fragments[0].trajectory_id != fragments[1].trajectory_id


def test_fragment_connection_requires_multi_frame_consensus() -> None:
    frames = [_frame(index, float(index)) for index in range(6)]
    poses_a = {index: _hypothesis(0.0, float(index), 1.0).t_target_source for index in range(3)}
    poses_b = {
        index: _hypothesis(0.0, float(index - 3), 1.0).t_target_source
        for index in range(3, 6)
    }
    fragment_a = Fragment("robot:000", "robot", (0, 1, 2), poses_a, ())
    fragment_b = Fragment("robot:001", "robot", (3, 4, 5), poses_b, ())
    t_a_b = _hypothesis(0.0, 10.0, 1.0).t_target_source

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        target_pose = poses_a[target.index]
        source_pose = poses_b[source.index]
        t_target_source = se3_inverse(target_pose) @ t_a_b @ source_pose
        item = _hypothesis(0.0, 0.0, 0.9)
        return [
            RegistrationHypothesis(
                t_target_source,
                0.0,
                item.descriptor_distance,
                item.coarse_score,
                item.symmetric_overlap,
                item.symmetric_rmse,
                item.gicp_mean_error,
                item.num_inliers,
                item.score,
            )
        ]

    accepted, rejected = find_fragment_connections(
        frames,
        [fragment_a, fragment_b],
        register,
        FragmentMatchConfig(
            boundary_window=3,
            max_pairs_per_fragment_pair=9,
            min_spatial_span_m=0.5,
        ),
    )

    assert not rejected
    assert len(accepted) == 1
    assert accepted[0].support == 9
    assert np.allclose(accepted[0].t_a_b, t_a_b)
    placement = place_fragments([fragment_a, fragment_b], accepted)
    assert placement.components == (frozenset({"robot:000", "robot:001"}),)
    assert np.allclose(placement.poses["robot:001"], t_a_b, atol=1e-6)


def test_required_pairs_reconsider_a_verified_connection_after_retrieval_crowding() -> None:
    frames = [
        _frame(index, float(index), robot_id="left", seq=index)
        for index in range(3)
    ] + [
        _frame(index, float(index - 3), robot_id="right", seq=index - 3)
        for index in range(3, 6)
    ]
    poses_left = {index: _yaw_pose(float(index), 0.0) for index in range(3)}
    poses_right = {
        index: _yaw_pose(float(index - 3), 0.0) for index in range(3, 6)
    }
    left = Fragment("left:000", "left", (0, 1, 2), poses_left, ())
    right = Fragment("right:000", "right", (3, 4, 5), poses_right, ())
    t_left_right = _yaw_pose(6.0, 0.0)

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        transform = (
            se3_inverse(poses_left[target.index])
            @ t_left_right
            @ poses_right[source.index]
        )
        template = _hypothesis(0.0, 0.0, 0.9)
        return [
            RegistrationHypothesis(
                transform,
                template.yaw_prior,
                template.descriptor_distance,
                template.coarse_score,
                template.symmetric_overlap,
                template.symmetric_rmse,
                template.gicp_mean_error,
                template.num_inliers,
                template.score,
            )
        ]

    accepted, rejected = find_fragment_connections(
        frames,
        [left, right],
        register,
        FragmentMatchConfig(
            descriptor_neighbors=1,
            candidates_per_frame=1,
            max_pairs_per_fragment_pair=3,
            min_spatial_span_m=0.5,
        ),
        required_frame_pairs=[(0, 3), (1, 4), (2, 5)],
    )

    assert rejected == []
    assert len(accepted) == 1
    assert accepted[0].support == 3
    assert np.allclose(accepted[0].t_a_b, t_left_right)


def test_spatially_correlated_alias_does_not_hide_a_valid_rendezvous() -> None:
    """Many votes from one place cannot outrank distributed observations."""
    frames = [
        _frame(index, float(index), robot_id="left", seq=index)
        for index in range(4)
    ] + [
        _frame(index, float(index - 4), robot_id="right", seq=index - 4)
        for index in range(4, 8)
    ]
    poses_left = {
        0: _yaw_pose(0.0, 0.0),
        1: _yaw_pose(0.0, 0.0),
        2: _yaw_pose(1.0, 0.0),
        3: _yaw_pose(2.0, 0.0),
    }
    poses_right = {
        4: _yaw_pose(0.0, 0.0),
        5: _yaw_pose(0.0, 0.0),
        6: _yaw_pose(1.0, 0.0),
        7: _yaw_pose(2.0, 0.0),
    }
    left = Fragment("left:000", "left", (0, 1, 2, 3), poses_left, ())
    right = Fragment("right:000", "right", (4, 5, 6, 7), poses_right, ())
    correlated_alias = _yaw_pose(20.0, math.pi)
    rendezvous = _yaw_pose(6.0, 0.0)
    rendezvous_pairs = {(0, 4), (2, 6), (3, 7)}

    def mode(
        target: ReconstructionFrame,
        source: ReconstructionFrame,
        fragment_transform: np.ndarray,
        score: float,
    ) -> RegistrationHypothesis:
        target_pose = poses_left[target.index]
        source_pose = poses_right[source.index]
        transform = se3_inverse(target_pose) @ fragment_transform @ source_pose
        template = _hypothesis(0.0, 0.0, score)
        return RegistrationHypothesis(
            transform,
            template.yaw_prior,
            template.descriptor_distance,
            template.coarse_score,
            template.symmetric_overlap,
            template.symmetric_rmse,
            template.gicp_mean_error,
            template.num_inliers,
            score,
        )

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        if target.robot_id == "right":
            target, source = source, target
        hypotheses = []
        # Eight mutually consistent matches all reuse the same physical pose
        # on the left, so this larger cluster must fail the span gate.
        if target.index in {0, 1}:
            hypotheses.append(mode(target, source, correlated_alias, 0.95))
        if (target.index, source.index) in rendezvous_pairs:
            hypotheses.append(mode(target, source, rendezvous, 0.85))
        return hypotheses

    accepted, rejected = find_fragment_connections(
        frames,
        [left, right],
        register,
        FragmentMatchConfig(
            descriptor_neighbors=8,
            candidates_per_frame=8,
            max_pairs_per_fragment_pair=9,
            min_spatial_span_m=0.5,
        ),
    )

    assert rejected == []
    assert len(accepted) == 1
    assert accepted[0].support == 3
    assert np.allclose(accepted[0].t_a_b, rendezvous)


def test_coarse_pose_hints_resolve_equal_support_pi_alias() -> None:
    """A broad prior selects a geometric mode but never creates one.

    This is the regression for the four-robot Gazebo capture: mirrored paths
    produced equal-support clusters at yaw 0 and pi, and summed scores chose
    the wrong yaw. With no hints the correct answer is to remain disconnected;
    with surveyed starts the lower-scoring, prior-consistent mode is safe.
    """
    frames = [
        _frame(i, float(i), x=float(i), robot_id="left", seq=i)
        for i in range(3)
    ] + [
        _frame(i + 3, float(i), x=float(i), robot_id="right", seq=i)
        for i in range(3)
    ]
    poses_left = {i: _yaw_pose(float(i), 0.0) for i in range(3)}
    poses_right = {i + 3: _yaw_pose(float(i), 0.0) for i in range(3)}
    left = Fragment("left:000", "left", (0, 1, 2), poses_left, ())
    right = Fragment("right:000", "right", (3, 4, 5), poses_right, ())
    correct = _yaw_pose(10.0, math.pi)
    wrong = _yaw_pose(4.0, 0.0)

    def registration_mode(
        target: ReconstructionFrame,
        source: ReconstructionFrame,
        fragment_transform: np.ndarray,
        score: float,
    ) -> RegistrationHypothesis:
        target_pose = poses_left[target.index]
        source_pose = poses_right[source.index]
        transform = se3_inverse(target_pose) @ fragment_transform @ source_pose
        template = _hypothesis(0.0, 0.0, score)
        return RegistrationHypothesis(
            transform,
            0.0,
            template.descriptor_distance,
            template.coarse_score,
            template.symmetric_overlap,
            template.symmetric_rmse,
            template.gicp_mean_error,
            template.num_inliers,
            score,
        )

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        if target.robot_id == source.robot_id:
            return []
        if target.robot_id == "right":
            target, source = source, target
            return [
                registration_mode(target, source, wrong, 0.9),
                registration_mode(target, source, correct, 0.8),
            ]
        return [
            registration_mode(target, source, wrong, 0.9),
            registration_mode(target, source, correct, 0.8),
        ]

    config = FragmentMatchConfig(
        descriptor_neighbors=6,
        candidates_per_frame=6,
        max_pairs_per_fragment_pair=36,
        min_spatial_span_m=0.5,
    )
    accepted, rejected = find_fragment_connections(
        frames, [left, right], register, config
    )
    assert accepted == []
    assert rejected[0].reason == "ambiguous competing transform"

    accepted, rejected = find_fragment_connections(
        frames,
        [left, right],
        register,
        config,
        pose_hints={"left": se3_identity(), "right": correct},
    )
    assert rejected == []
    assert len(accepted) == 1
    assert np.allclose(accepted[0].t_a_b, correct)


def test_coarse_start_is_not_reused_for_a_later_fragment() -> None:
    """A capture gap/restart does not teleport that fragment to spawn."""
    frames = [_frame(0, 0.0, robot_id="left", seq=0)] + [
        _frame(i + 1, 10.0 + i, robot_id="left", seq=10 + i)
        for i in range(3)
    ] + [
        _frame(i + 4, float(i), robot_id="right", seq=i)
        for i in range(3)
    ]
    left_primary = Fragment(
        "left:000", "left", (0,), {0: se3_identity()}, ()
    )
    poses_left = {i + 1: _yaw_pose(float(i), 0.0) for i in range(3)}
    poses_right = {i + 4: _yaw_pose(float(i), 0.0) for i in range(3)}
    left_later = Fragment(
        "left:001", "left", (1, 2, 3), poses_left, ()
    )
    right = Fragment("right:000", "right", (4, 5, 6), poses_right, ())
    correct = _yaw_pose(10.0, math.pi)
    wrong = _yaw_pose(4.0, 0.0)

    def mode(
        target: ReconstructionFrame,
        source: ReconstructionFrame,
        fragment_transform: np.ndarray,
        score: float,
    ) -> RegistrationHypothesis:
        target_pose = poses_left[target.index]
        source_pose = poses_right[source.index]
        transform = se3_inverse(target_pose) @ fragment_transform @ source_pose
        template = _hypothesis(0.0, 0.0, score)
        return RegistrationHypothesis(
            transform,
            0.0,
            template.descriptor_distance,
            template.coarse_score,
            template.symmetric_overlap,
            template.symmetric_rmse,
            template.gicp_mean_error,
            template.num_inliers,
            score,
        )

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        if target.index == 0 or source.index == 0 or target.robot_id == source.robot_id:
            return []
        if target.robot_id == "right":
            target, source = source, target
            return [mode(target, source, wrong, 0.9), mode(target, source, correct, 0.8)]
        return [mode(target, source, wrong, 0.9), mode(target, source, correct, 0.8)]

    config = FragmentMatchConfig(
        descriptor_neighbors=7,
        candidates_per_frame=7,
        max_pairs_per_fragment_pair=49,
        min_spatial_span_m=0.5,
    )
    accepted, rejected = find_fragment_connections(
        frames,
        [left_primary, left_later, right],
        register,
        config,
        pose_hints={"left": se3_identity(), "right": correct},
    )

    assert accepted == []
    assert any(
        item.fragment_a == "left:001"
        and item.fragment_b == "right:000"
        and item.reason == "ambiguous competing transform"
        for item in rejected
    )


def test_fragment_consensus_cannot_outvote_direct_boundary_registration() -> None:
    frames = [_frame(index, float(index)) for index in range(6)]
    poses_a = {
        index: _hypothesis(0.0, float(index), 1.0).t_target_source
        for index in range(3)
    }
    poses_b = {
        index: _hypothesis(0.0, float(index - 3), 1.0).t_target_source
        for index in range(3, 6)
    }
    fragment_a = Fragment("robot:000", "robot", (0, 1, 2), poses_a, ())
    fragment_b = Fragment("robot:001", "robot", (3, 4, 5), poses_b, ())
    correct = _hypothesis(0.0, 10.0, 1.0).t_target_source
    wrong = _hypothesis(math.pi, 10.0, 1.0).t_target_source

    def mode(
        target: ReconstructionFrame,
        source: ReconstructionFrame,
        fragment_transform: np.ndarray,
        score: float,
    ) -> RegistrationHypothesis:
        target_pose = poses_a[target.index]
        source_pose = poses_b[source.index]
        transform = se3_inverse(target_pose) @ fragment_transform @ source_pose
        item = _hypothesis(0.0, 0.0, score)
        return RegistrationHypothesis(
            transform,
            0.0,
            item.descriptor_distance,
            item.coarse_score,
            item.symmetric_overlap,
            item.symmetric_rmse,
            item.gicp_mean_error,
            item.num_inliers,
            item.score,
        )

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        wrong_mode = mode(target, source, wrong, 0.8)
        if (target.index, source.index) == (2, 3):
            return [mode(target, source, correct, 0.95), wrong_mode]
        return [wrong_mode]

    accepted, rejected = find_fragment_connections(
        frames,
        [fragment_a, fragment_b],
        register,
        FragmentMatchConfig(
            boundary_window=3,
            max_pairs_per_fragment_pair=9,
            min_spatial_span_m=0.5,
        ),
    )

    assert accepted == []
    assert any(
        item.reason == "boundary consensus contradicts adjacent registration"
        for item in rejected
    )


def test_boundary_registration_selects_a_supported_lower_ranked_mode() -> None:
    frames = [_frame(index, float(index)) for index in range(8)]
    poses_a = {index: _yaw_pose(float(index), 0.0) for index in range(4)}
    poses_b = {
        index: _yaw_pose(float(index - 4), 0.0) for index in range(4, 8)
    }
    fragment_a = Fragment("robot:000", "robot", (0, 1, 2, 3), poses_a, ())
    fragment_b = Fragment("robot:001", "robot", (4, 5, 6, 7), poses_b, ())
    correct = _yaw_pose(4.0, 0.0)
    repeated_corridor = _yaw_pose(10.0, math.pi)

    def mode(
        target: ReconstructionFrame,
        source: ReconstructionFrame,
        fragment_transform: np.ndarray,
        score: float,
    ) -> RegistrationHypothesis:
        transform = (
            se3_inverse(poses_a[target.index])
            @ fragment_transform
            @ poses_b[source.index]
        )
        template = _hypothesis(0.0, 0.0, score)
        return RegistrationHypothesis(
            transform,
            template.yaw_prior,
            template.descriptor_distance,
            template.coarse_score,
            template.symmetric_overlap,
            template.symmetric_rmse,
            template.gicp_mean_error,
            template.num_inliers,
            score,
        )

    def register(target: ReconstructionFrame, source: ReconstructionFrame):
        hypotheses = []
        if target.index == 3 and source.index == 4:
            hypotheses.append(mode(target, source, correct, 0.95))
        if source.index - target.index == 4:
            hypotheses.append(mode(target, source, correct, 0.80))
        hypotheses.append(mode(target, source, repeated_corridor, 0.90))
        return hypotheses

    accepted, rejected = find_fragment_connections(
        frames,
        [fragment_a, fragment_b],
        register,
        FragmentMatchConfig(
            boundary_window=4,
            descriptor_neighbors=8,
            candidates_per_frame=8,
            max_pairs_per_fragment_pair=16,
            min_spatial_span_m=0.5,
        ),
    )

    assert rejected == []
    assert len(accepted) == 1
    assert accepted[0].support == 5
    assert np.allclose(accepted[0].t_a_b, correct)


def test_inter_robot_bridges_do_not_count_as_corroboration() -> None:
    aslan_a = _single_frame_fragment("aslan:000", "aslan", 0)
    aslan_b = _single_frame_fragment("aslan:001", "aslan", 1)
    botman = _single_frame_fragment("botman:000", "botman", 2)
    connections = [
        _connection(aslan_a, botman, se3_identity()),
        _connection(aslan_b, botman, se3_identity()),
    ]

    kept, rejected = filter_inter_robot_connections(
        [aslan_a, aslan_b, botman], connections
    )

    assert kept == []
    assert len(rejected) == 2
    assert all(
        item.reason == "inter-robot merge has no independent cycle"
        for item in rejected
    )


def test_inter_robot_merge_needs_two_separated_consistent_encounters() -> None:
    aslan_a = _single_frame_fragment("aslan:000", "aslan", 0)
    aslan_b = _single_frame_fragment("aslan:001", "aslan", 1)
    botman_a = _single_frame_fragment("botman:000", "botman", 2)
    botman_b = _single_frame_fragment("botman:001", "botman", 3)
    five_metres = _hypothesis(0.0, 5.0, 1.0).t_target_source
    intra_aslan = _connection(aslan_a, aslan_b, five_metres)
    intra_botman = _connection(botman_a, botman_b, five_metres)
    encounter_a = _connection(aslan_a, botman_a, se3_identity())
    encounter_b = _connection(aslan_b, botman_b, se3_identity())

    kept, rejected = filter_inter_robot_connections(
        [aslan_a, aslan_b, botman_a, botman_b],
        [intra_aslan, intra_botman, encounter_a, encounter_b],
    )

    assert rejected == []
    assert set(map(id, kept)) == {
        id(intra_aslan),
        id(intra_botman),
        id(encounter_a),
        id(encounter_b),
    }


def test_survey_corroborated_mode_outranks_later_fragment_alias() -> None:
    aslan = _single_frame_fragment("aslan:000", "aslan", 0)
    botman_primary = _single_frame_fragment("botman:000", "botman", 1)
    botman_later = _single_frame_fragment("botman:001", "botman", 2)
    five_metres = _yaw_pose(5.0, 0.0)
    intra_botman = _connection(botman_primary, botman_later, five_metres)
    correct = _connection(aslan, botman_primary, se3_identity())
    correct = FragmentConnection(
        correct.fragment_a,
        correct.fragment_b,
        correct.t_a_b,
        correct.support,
        correct.score,
        correct.proposals,
        pose_hint_support=3,
    )
    alias = _connection(aslan, botman_later, se3_identity())
    alias = FragmentConnection(
        alias.fragment_a,
        alias.fragment_b,
        alias.t_a_b,
        alias.support,
        30.0,
        alias.proposals,
    )

    kept, rejected = filter_inter_robot_connections(
        [aslan, botman_primary, botman_later],
        [intra_botman, correct, alias],
    )

    assert set(map(id, kept)) == {id(intra_botman), id(correct)}
    assert len(rejected) == 1
    assert rejected[0].fragment_b == "botman:001"
    assert rejected[0].reason == "inter-robot transform disagrees with consensus"


def test_one_fragment_pair_can_contain_two_separated_encounters() -> None:
    four_metres = _hypothesis(0.0, 4.0, 1.0).t_target_source
    aslan = Fragment(
        "aslan:000",
        "aslan",
        (0, 1),
        {0: se3_identity(), 1: four_metres},
        (),
    )
    botman = Fragment(
        "botman:000",
        "botman",
        (2, 3),
        {2: se3_identity(), 3: four_metres},
        (),
    )
    proposals = (
        ConnectionProposal("aslan:000", "botman:000", 0, 2, se3_identity(), 0.9),
        ConnectionProposal("aslan:000", "botman:000", 1, 3, se3_identity(), 0.9),
    )
    encounter = FragmentConnection(
        "aslan:000", "botman:000", se3_identity(), 2, 1.8, proposals
    )

    kept, rejected = filter_inter_robot_connections([aslan, botman], [encounter])

    assert kept == [encounter]
    assert rejected == []
