"""Trajectory segments: one robot, several runs, individually selectable.

The failure these exist to pin down is silent in every direction. A robot
reboots, its ``seq`` counter restarts at zero, and the back-end -- which keyed
nodes on ``(robot_id, seq)`` -- dropped the repeats as duplicates and returned
False with nothing written to ``last_error``. Whatever survived past the old
maximum landed in the robot's NEW map frame, chained to the old one by an
odometry edge that GNC is structurally forbidden from rejecting.

Nothing about either failure is visible in a map, a count, or a log line, which
is why the assertions here are about identity and edges rather than about how
good the answer looks.
"""

from __future__ import annotations

import numpy as np
import pytest
import synthetic

from swarmdeck_protocol import ProtocolError, decode_keyframe, encode_keyframe
from swarmdeck_slam.backend import (
    LEGACY_SESSION_PREFIX,
    CollaborativeBackend,
    LegacySegmenter,
    scoped_grids,
    snapshot_update,
)
from swarmdeck_slam.graph import GtsamPoseGraph
from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    KeyframeId,
    TrajectoryId,
    quat_xyz_from_se3,
    se3_distance,
    se3_inverse,
    se3_relative,
)


def _blob(keyframe, session: str = "") -> bytes:
    return encode_keyframe(
        robot_id=keyframe.id.robot_id,
        seq=keyframe.id.seq,
        stamp=keyframe.stamp,
        points=keyframe.points,
        t_odom_base=quat_xyz_from_se3(keyframe.t_odom_base),
        session=session,
    )


def _ingest(backend: CollaborativeBackend, segments, *, on_wire: bool = True) -> int:
    """Stream every segment's keyframes in capture order. Returns how many landed."""
    keyed = [(kf, robot.session) for robot in segments for kf in robot.keyframes]
    keyed.sort(key=lambda item: (item[0].stamp, item[0].id.robot_id, item[0].id.seq))
    landed = 0
    for keyframe, session in keyed:
        if on_wire:
            landed += int(backend.ingest_packet(decode_keyframe(_blob(keyframe, session))))
        else:
            landed += int(backend.ingest_keyframe(keyframe))
    return landed


@pytest.fixture(scope="module")
def restarted():
    return synthetic.restarted_robot()


@pytest.fixture(scope="module")
def restarted_snapshot(restarted):
    _, segments = restarted
    backend = CollaborativeBackend()
    assert _ingest(backend, segments) == sum(len(r.keyframes) for r in segments)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    return backend, snapshot, segments


# --------------------------------------------------------------------------- #
# Failure 1: the seq collision
# --------------------------------------------------------------------------- #

def test_a_restarted_robot_does_not_collide_with_its_own_history(restarted) -> None:
    """Every keyframe of both runs lands.

    Before sessions, the second run's ``seq`` 0..N were indistinguishable from
    the first run's and ``ingest_keyframe`` dropped them one by one, returning
    False. Live, aslan_0 held 194 keyframes when it power-cycled, so its next
    ~194 -- roughly 97 m of driving at the 0.5 m capture gate -- would have
    disappeared with ``last_error`` empty the whole time.
    """
    _, segments = restarted
    expected = sum(len(robot.keyframes) for robot in segments)
    assert segments[0].keyframes[0].id.seq == segments[1].keyframes[0].id.seq == 0

    backend = CollaborativeBackend()
    assert _ingest(backend, segments) == expected
    assert len(backend) == expected
    assert backend.trajectory_ids() == [
        TrajectoryId("alpha", ""),
        TrajectoryId("alpha", "boot-2"),
    ]


def test_a_genuine_duplicate_is_still_dropped(restarted) -> None:
    """The collision fix must not turn retransmission into a second keyframe."""
    _, segments = restarted
    backend = CollaborativeBackend()
    packet = decode_keyframe(_blob(segments[1].keyframes[0], segments[1].session))
    assert backend.ingest_packet(packet)
    assert not backend.ingest_packet(packet)
    assert len(backend) == 1


# --------------------------------------------------------------------------- #
# Failure 2: the fabricated odometry edge
# --------------------------------------------------------------------------- #

def test_no_odometry_edge_spans_a_segment_boundary(restarted_snapshot) -> None:
    """An ODOMETRY edge is a GNC known-inlier -- nothing downstream can reject
    one. So the only defence against an edge between two unrelated map frames
    is never building it."""
    backend, _snapshot, _segments = restarted_snapshot
    odometry = [e for e in backend._edges if e.kind is EdgeKind.ODOMETRY]
    assert odometry, "fixture produced no odometry at all"
    assert all(e.src.trajectory == e.dst.trajectory for e in odometry)


def test_each_segment_is_chained_end_to_end(restarted_snapshot) -> None:
    """Splitting the chain must not leave gaps inside a segment: every keyframe
    but the first of each run still has an odometry edge into it."""
    backend, _snapshot, segments = restarted_snapshot
    into = {e.dst for e in backend._edges if e.kind is EdgeKind.ODOMETRY}
    for robot in segments:
        ids = sorted(kf.id for kf in robot.keyframes)
        assert not (set(ids[1:]) - into)
        assert ids[0] not in into


# --------------------------------------------------------------------------- #
# The safety property: PCM guards the re-merge
# --------------------------------------------------------------------------- #

def _rigid_closure(kind, src, dst, t_src_dst, weight: float = 400.0) -> Edge:
    return Edge(
        kind=kind,
        src=src,
        dst=dst,
        t_src_dst=t_src_dst,
        information=np.eye(6) * weight,
        fitness=1.0,
        inlier_ratio=1.0,
    )


def _graph_of(segments, closures, **kwargs) -> tuple[GtsamPoseGraph, list]:
    graph = GtsamPoseGraph(**kwargs)
    for robot in segments:
        previous = None
        for keyframe in robot.keyframes:
            graph.add_keyframe(keyframe)
            if previous is not None:
                graph.add_edge(
                    Edge(
                        kind=EdgeKind.ODOMETRY,
                        src=previous.id,
                        dst=keyframe.id,
                        t_src_dst=se3_relative(previous.t_odom_base, keyframe.t_odom_base),
                        information=np.eye(6) * 400.0,
                    )
                )
            previous = keyframe
    for edge in closures:
        graph.add_edge(edge)
    return graph, closures


def _true_closure(segments, index_before: int, index_after: int) -> Edge:
    """A perfect closure between the two runs, from ground truth."""
    before, after = segments
    src = before.keyframes[index_before]
    dst = after.keyframes[index_after]
    t_src_dst = se3_relative(before.truth[src.id], after.truth[dst.id])
    return _rigid_closure(EdgeKind.INTRA_LOOP, src.id, dst.id, t_src_dst)


def test_a_lone_closure_cannot_rejoin_a_robot_to_its_own_history(restarted) -> None:
    """One uncorroborated closure must NOT merge the two runs.

    This is the property that did not exist before trajectories: the segments
    were assumed to be one, so there was no merge for anything to guard. Now
    rejoining a robot to its own past clears the same bar as merging with a
    stranger -- because it is the same claim. The robot may have been carried
    to another floor while it was off, and nothing on the wire says otherwise.
    """
    _, segments = restarted
    graph, _ = _graph_of(segments, [_true_closure(segments, 4, 4)])
    result = graph.optimize()

    assert len(result.components) == 2
    assert {frozenset(c.trajectories) for c in result.components} == {
        frozenset({TrajectoryId("alpha", "")}),
        frozenset({TrajectoryId("alpha", "boot-2")}),
    }
    assert len(result.rejected_edges) == 1


def test_corroborated_closures_do_rejoin_a_robot_to_its_own_history(restarted) -> None:
    """The guard is a bar to clear, not a wall. Two mutually consistent
    closures merge the runs and recover the transform between the two map
    frames -- which nothing on the wire ever stated."""
    _, segments = restarted
    closures = [_true_closure(segments, i, i) for i in (4, 6)]
    graph, _ = _graph_of(segments, closures)
    result = graph.optimize()

    assert len(result.components) == 1
    assert result.components[0].trajectories == frozenset(
        {TrajectoryId("alpha", ""), TrajectoryId("alpha", "boot-2")}
    )
    assert not result.rejected_edges

    for robot in segments:
        estimated = result.t_world_trajectory[robot.trajectory_id]
        relative = se3_relative(result.t_world_trajectory[segments[0].trajectory_id], estimated)
        truth = se3_relative(segments[0].t_world_map_true, robot.t_world_map_true)
        translation, rotation = se3_distance(relative, truth)
        assert translation < 0.5, f"{robot.trajectory_id}: {translation:.3f} m"
        assert rotation < np.radians(5.0)


def test_two_closures_that_disagree_do_not_rejoin_the_runs(restarted) -> None:
    """Corroboration means agreement, not arithmetic. Two closures whose loop
    does not close leave the segments apart -- the same clique test that
    catches a wrong inter-robot merge, now reachable for a robot's own past."""
    _, segments = restarted
    graph, _ = _graph_of(
        segments,
        [_true_closure(segments, 4, 4), _true_closure(segments, 4, 12)],
    )
    result = graph.optimize()
    assert len(result.components) == 2
    assert len(result.rejected_edges) == 2


def test_rejoining_a_robots_own_history_is_not_counted_as_collaboration(restarted) -> None:
    """``is_inter_robot`` stays keyed on the robot id, so a robot meeting its
    own past does not inflate the operator's collaboration counter -- while
    ``is_inter_trajectory`` still routes it through PCM."""
    _, segments = restarted
    closures = [_true_closure(segments, i, i) for i in (4, 6)]
    assert all(not edge.is_inter_robot for edge in closures)
    assert all(edge.is_inter_trajectory for edge in closures)

    backend = CollaborativeBackend()
    _ingest(backend, segments)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert snapshot.inter_robot_closures == 0
    assert snapshot.accepted_closures > 0


def test_pcm_never_composes_across_a_segment_boundary(restarted) -> None:
    """PCM's consistency check bridges each side's two keyframes through that
    side's own odometry. Grouping candidates by ROBOT would put a pre-reboot
    keyframe and a post-reboot one on the same side of that bridge and subtract
    two unrelated map frames, so whichever way the test then fell would be an
    accident."""
    from swarmdeck_slam.graph import _canonical_pair

    _, segments = restarted
    edge = _true_closure(segments, 4, 4)
    assert _canonical_pair(edge) == frozenset(
        {TrajectoryId("alpha", ""), TrajectoryId("alpha", "boot-2")}
    )


def test_segments_in_different_places_are_left_unmerged() -> None:
    """Declining is the harder half. Two runs through different parts of the
    building have no true relative transform, and an unmerged pair of segments
    is the correct statement of that."""
    _, segments = synthetic.restarted_robot(overlap=False)
    backend = CollaborativeBackend()
    _ingest(backend, segments)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert len(snapshot.optimized.components) == 2


# --------------------------------------------------------------------------- #
# Per-segment frames
# --------------------------------------------------------------------------- #

def test_each_segment_gets_its_own_map_frame(restarted_snapshot) -> None:
    """Two frames, not one fitted across the discontinuity. A single rigid fit
    over both runs is a least-squares compromise between two unrelated gauges:
    it explains neither, and it is published straight to the operator."""
    _backend, snapshot, segments = restarted_snapshot
    frames = snapshot.optimized.t_world_trajectory
    assert set(frames) == {robot.trajectory_id for robot in segments}
    before = frames[segments[0].trajectory_id]
    after = frames[segments[1].trajectory_id]
    assert np.linalg.norm(before[:3, 3] - after[:3, 3]) > 1.0, (
        "the two runs' frames came out identical, which cannot be right"
    )


def test_the_robot_level_frame_is_the_newest_segments(restarted_snapshot) -> None:
    """``origins`` means "where is this robot now", so the robot-level frame is
    the one its live segment is publishing in -- picked by wall clock, since
    ``seq`` restarts and arrival order can be reordered by the service queue."""
    _backend, snapshot, segments = restarted_snapshot
    newest = max(segments, key=lambda robot: robot.keyframes[-1].stamp)
    assert np.allclose(
        snapshot.optimized.t_world_map["alpha"],
        snapshot.optimized.t_world_trajectory[newest.trajectory_id],
    )


def test_a_robot_with_unmerged_segments_appears_once_in_origins() -> None:
    """Its segments are in two components; the fleet view must still show one
    robot in one place, not the same machine twice."""
    _, segments = synthetic.restarted_robot(overlap=False)
    backend = CollaborativeBackend()
    _ingest(backend, segments)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert len(snapshot.optimized.components) == 2
    assert list(snapshot_update(snapshot)["origins"]) == ["alpha"]


# --------------------------------------------------------------------------- #
# Selection
# --------------------------------------------------------------------------- #

def test_excluding_a_segment_removes_it_from_the_solve_and_is_reversible(restarted) -> None:
    """Excluded means "not handed to the solver", never "deleted". The
    keyframes, their clouds and every edge that touched them stay, so putting
    the segment back restores exactly the graph that was there before."""
    _, segments = restarted
    dropped = segments[0].trajectory_id
    kept = segments[1].trajectory_id

    backend = CollaborativeBackend()
    _ingest(backend, segments)
    before = backend.optimize_and_render()
    assert before is not None

    assert backend.set_included(dropped, False) is True
    assert backend.set_included(dropped, False) is False, "second exclude is a no-op"
    excluded = backend.optimize_and_render()
    assert excluded is not None
    assert {k.trajectory for k in excluded.optimized.poses} == {kept}
    assert excluded.accepted_closures <= before.accepted_closures
    # stored, not deleted
    assert len(backend) == len(before.optimized.poses)
    assert dropped in backend.trajectory_ids()
    listed = {t.trajectory_id: t for t in excluded.trajectories}
    assert listed[dropped].included is False
    assert listed[dropped].keyframes == len(segments[0].keyframes)
    assert listed[dropped].component_id is None

    assert backend.set_included(dropped, True) is True
    restored = backend.optimize_and_render()
    assert restored is not None
    assert set(restored.optimized.poses) == set(before.optimized.poses)
    assert restored.accepted_closures == before.accepted_closures
    for kf_id, pose in before.optimized.poses.items():
        assert np.allclose(restored.optimized.poses[kf_id], pose, atol=1e-6)


def test_include_only_rebuilds_a_map_from_a_chosen_subset(restarted) -> None:
    """The 'recreate the map' path: name the segments, re-optimize, get a map
    built from those and nothing else."""
    _, segments = restarted
    backend = CollaborativeBackend()
    _ingest(backend, segments)
    backend.optimize_and_render()

    chosen = segments[1].trajectory_id
    backend.include_only([chosen])
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert {k.trajectory for k in snapshot.optimized.poses} == {chosen}
    assert set(snapshot.robot_grids) == {"alpha"}
    assert snapshot.keyframe_counts == {"alpha": len(segments[1].keyframes)}

    backend.include_only(backend.trajectory_ids())
    both = backend.optimize_and_render()
    assert both is not None
    assert {k.trajectory for k in both.optimized.poses} == {
        robot.trajectory_id for robot in segments
    }


def test_a_selection_change_marks_the_backend_dirty(restarted) -> None:
    """The service worker only re-solves a dirty back-end, so a selection that
    did not dirty it would sit unapplied until the next keyframe arrived."""
    _, segments = restarted
    backend = CollaborativeBackend()
    _ingest(backend, segments)
    backend.optimize_and_render()
    assert backend.dirty is False
    backend.set_included(segments[0].trajectory_id, False)
    assert backend.dirty is True


def test_excluded_segments_attract_no_new_closures(restarted) -> None:
    """An edge into an excluded trajectory would reference a keyframe the
    solver has never been given -- which the graph rejects outright."""
    _, segments = restarted
    backend = CollaborativeBackend()
    _ingest(backend, [segments[0]])
    backend.set_included(segments[0].trajectory_id, False)
    _ingest(backend, [segments[1]])
    assert all(
        segments[0].trajectory_id not in (e.src.trajectory, e.dst.trajectory)
        for e in backend._edges
        if e.kind.is_loop_closure and backend._edge_included(e)
    )
    assert backend.optimize_and_render() is not None


def test_trajectories_are_listed_with_their_spans(restarted_snapshot) -> None:
    _backend, snapshot, segments = restarted_snapshot
    rows = {t.trajectory_id: t for t in snapshot.trajectories}
    assert set(rows) == {robot.trajectory_id for robot in segments}
    for robot in segments:
        row = rows[robot.trajectory_id]
        assert row.robot_id == "alpha"
        assert row.keyframes == len(robot.keyframes)
        assert row.first_seq == robot.keyframes[0].id.seq
        assert row.last_seq == robot.keyframes[-1].id.seq
        assert row.first_stamp == robot.keyframes[0].stamp
        assert row.last_stamp == robot.keyframes[-1].stamp
        assert row.included is True
        assert row.to_dict()["id"] == str(robot.trajectory_id)


# --------------------------------------------------------------------------- #
# Scoped grids
# --------------------------------------------------------------------------- #

def test_a_segment_can_be_inspected_on_its_own_scope(restarted_snapshot) -> None:
    """``robot:`` keeps grouping a robot's runs together -- that is still one
    machine's coverage. ``trajectory:`` is added beside it so one run can be
    looked at alone, which is the only way to see whether the two halves agree
    about the building."""
    _backend, snapshot, segments = restarted_snapshot
    scopes = [scope for scope, _grid in scoped_grids(snapshot)]
    assert "robot:alpha" in scopes
    for robot in segments:
        assert f"trajectory:{robot.trajectory_id}" in scopes
    assert len(set(scopes)) == len(scopes)
    assert snapshot_update(snapshot)["scopes"] == scopes


def test_a_robot_that_never_restarted_gets_no_trajectory_scope() -> None:
    """It would be a byte-identical copy of that robot's own grid under a
    second name, and the ray walk that dominates a render is not worth paying
    twice for the same picture."""
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    _ingest(backend, fleet)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert snapshot.trajectory_grids == {}
    assert not [s for s, _ in scoped_grids(snapshot) if s.startswith("trajectory:")]


# --------------------------------------------------------------------------- #
# Backward compatibility
# --------------------------------------------------------------------------- #

def test_keyframe_id_is_still_constructible_positionally() -> None:
    assert KeyframeId("alpha", 3) == KeyframeId("alpha", 3, "")
    assert KeyframeId("alpha", 3).trajectory == TrajectoryId("alpha")
    assert str(KeyframeId("alpha", 3)) == "alpha#3"
    assert str(KeyframeId("alpha", 3, "boot-2")) == "alpha@boot-2#3"


def test_a_blob_without_a_session_decodes_to_one_trajectory_per_robot() -> None:
    """Every capture under sessions/captures predates the field, and they must
    keep decoding to exactly the KeyframeIds they decoded to before."""
    _, fleet = synthetic.two_robot_fleet()
    keyframe = fleet[0].keyframes[0]
    packet = decode_keyframe(_blob(keyframe))
    assert packet.session == ""
    assert packet.trajectory == ("alpha", "")


def test_a_session_survives_the_wire() -> None:
    _, segments = synthetic.restarted_robot()
    keyframe = segments[1].keyframes[0]
    packet = decode_keyframe(_blob(keyframe, "boot-2"))
    assert packet.session == "boot-2"
    assert packet.seq == keyframe.id.seq


def test_omitting_the_session_emits_the_pre_session_bytes() -> None:
    """An encoder that declares no session must be indistinguishable from one
    that has never heard of the field -- which is what keeps an un-upgraded
    reader working and what stops a wire-version bump being needed."""
    _, fleet = synthetic.two_robot_fleet()
    keyframe = fleet[0].keyframes[0]
    from swarmdeck_protocol import peek_keyframe_header

    assert "session" not in peek_keyframe_header(_blob(keyframe))
    assert "session" in peek_keyframe_header(_blob(keyframe, "boot-2"))


def test_a_session_that_would_not_survive_a_url_is_refused() -> None:
    """Session ids end up in query strings and scope names."""
    _, fleet = synthetic.two_robot_fleet()
    keyframe = fleet[0].keyframes[0]
    with pytest.raises(ProtocolError):
        _blob(keyframe, "boot 2/../etc")
    with pytest.raises(ProtocolError):
        _blob(keyframe, "x" * 200)


def test_trajectory_ids_round_trip_through_their_string_form() -> None:
    for trajectory in (TrajectoryId("botman_0"), TrajectoryId("botman_0", "1787-abc")):
        assert TrajectoryId.parse(str(trajectory)) == trajectory
    with pytest.raises(ValueError):
        TrajectoryId.parse("@orphan")


# --------------------------------------------------------------------------- #
# Legacy segmentation
# --------------------------------------------------------------------------- #

def test_a_seq_reset_starts_a_new_legacy_segment() -> None:
    segmenter = LegacySegmenter()
    for seq in range(5):
        assert segmenter.session_for("botman_0", seq, stamp=float(seq)) == ""
    assert segmenter.session_for("botman_0", 0, stamp=100.0) == f"{LEGACY_SESSION_PREFIX}1"
    assert segmenter.session_for("botman_0", 1, stamp=101.0) == f"{LEGACY_SESSION_PREFIX}1"
    assert segmenter.session_for("botman_0", 0, stamp=200.0) == f"{LEGACY_SESSION_PREFIX}2"
    assert segmenter.restarts == 2


def test_a_late_arriving_packet_is_not_a_restart() -> None:
    """hw-run-02 contains exactly this: aslan_0's seq 109 delivered after 120,
    a queue reorder. The tempting "seq <= the last one seen" rule splits that
    trajectory in three where two is the truth."""
    segmenter = LegacySegmenter()
    for seq in (0, 1, 2, 3, 4, 6, 7, 8):
        segmenter.session_for("aslan_0", seq, stamp=float(seq))
    assert segmenter.session_for("aslan_0", 5, stamp=5.0) == ""


def test_a_retransmitted_packet_is_not_a_restart() -> None:
    """Same seq AND same stamp is the same keyframe arriving twice, which
    ``ingest_keyframe`` must still be allowed to drop as a duplicate."""
    segmenter = LegacySegmenter()
    assert segmenter.session_for("aslan_0", 0, stamp=7.0) == ""
    assert segmenter.session_for("aslan_0", 1, stamp=8.0) == ""
    assert segmenter.session_for("aslan_0", 0, stamp=7.0) == ""
    assert segmenter.restarts == 0


def test_legacy_segmentation_never_runs_against_a_declared_session(restarted) -> None:
    """A packet that carries a session is authoritative; the heuristic must not
    get a second opinion."""
    _, segments = restarted
    backend = CollaborativeBackend()
    _ingest(backend, segments)
    assert backend._segmenter.restarts == 0
    assert backend.trajectory_ids() == [
        TrajectoryId("alpha", ""),
        TrajectoryId("alpha", "boot-2"),
    ]


def test_a_legacy_capture_with_a_reboot_in_it_is_split(restarted) -> None:
    """The recorded blobs carry no session, so the boundary has to come from
    the seq reset. Without this, replaying such a capture reproduces the bug
    rather than the fix."""
    _, segments = restarted
    backend = CollaborativeBackend()
    landed = _ingest(backend, [synthetic.SyntheticRobot(
        robot.robot_id, robot.keyframes, robot.truth, robot.t_world_map_true,
        robot.scene_id, "",  # session stripped, as a pre-session capture would be
    ) for robot in segments])
    assert landed == sum(len(robot.keyframes) for robot in segments)
    assert backend.trajectory_ids() == [
        TrajectoryId("alpha", ""),
        TrajectoryId("alpha", f"{LEGACY_SESSION_PREFIX}1"),
    ]


def test_legacy_segmentation_can_be_turned_off(restarted) -> None:
    """Off reproduces a pre-trajectory replay exactly, bug included -- the one
    case where the old answer is what you are trying to measure."""
    _, segments = restarted
    backend = CollaborativeBackend(legacy_session_split=False)
    stripped = [synthetic.SyntheticRobot(
        robot.robot_id, robot.keyframes, robot.truth, robot.t_world_map_true,
        robot.scene_id, "",
    ) for robot in segments]
    landed = _ingest(backend, stripped)
    assert landed < sum(len(robot.keyframes) for robot in segments)
    assert backend.trajectory_ids() == [TrajectoryId("alpha", "")]


def test_a_capture_with_no_reboot_is_untouched_by_segmentation() -> None:
    """The whole backward-compatibility claim: 3d-run-01, 3d-run-02 and
    hw-run-01 contain no seq reset, so the heuristic never fires on them."""
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    _ingest(backend, fleet)
    assert backend._segmenter.restarts == 0
    assert backend.trajectory_ids() == [TrajectoryId("alpha"), TrajectoryId("beta")]


# --------------------------------------------------------------------------- #
# Keys
# --------------------------------------------------------------------------- #

def test_the_solver_key_separates_two_runs_of_one_robot() -> None:
    """Keying the solver on the robot hands it one variable for two poses."""
    from swarmdeck_slam.types import KeyRegistry

    keys = KeyRegistry()
    a = keys.key(KeyframeId("alpha", 0))
    b = keys.key(KeyframeId("alpha", 0, "boot-2"))
    assert a != b
    assert keys.unkey(a) == KeyframeId("alpha", 0)
    assert keys.unkey(b) == KeyframeId("alpha", 0, "boot-2")
    assert keys.robots == ("alpha",)
    assert keys.trajectories == (TrajectoryId("alpha"), TrajectoryId("alpha", "boot-2"))
