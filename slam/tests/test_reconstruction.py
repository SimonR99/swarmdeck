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


def _frame(index: int, stamp: float) -> ReconstructionFrame:
    empty = np.empty((0, 3), dtype=np.float64)
    cloud = PreparedCloud(empty, np.zeros((20, 60), dtype=np.uint8), np.zeros((2, 2)))
    return ReconstructionFrame(index, "robot", index, stamp, cloud)


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
