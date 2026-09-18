from types import SimpleNamespace as NS
import json
import uuid
import numpy as np
import pytest
from autonomy.capture_providers import CaptureClock, CaptureGeometry, CaptureProvenance
from autonomy.contracts import DeskewStatus, IDENTITY_SE3, RayReturnSemantics
from autonomy.cslam import (
    FRAME_HISTORY_LIMIT,
    CslamMapper,
    FrameState,
    publish_snapshot_if_new,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from autonomy.replication import ReplicaStore, canonical


def value(robot, seq, x):
    return NS(
        key=NS(robot_id=robot, keyframe_id=seq),
        pose=NS(position=NS(x=x, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
    )


def test_peer_correction_reuses_geometry_and_rejects_old_results(tmp_path):
    mission = str(uuid.uuid4())
    mapper = CorrectionAwareMapper(SubmapStore(tmp_path / "map"))
    core = CslamMapper(mapper, "r1", 1, mission, {0: "r0", 1: "r1"})
    core.capture(0, 10, IDENTITY_SE3, [[0, 0, 0], [1, 0, 0]])
    before = core.envelope()
    assert canonical(before) == canonical(core.envelope())
    msg = NS(
        success=True,
        mission_id=mission,
        solution_clock=2,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(1, 0, 4)],
        anchor_estimates=[value(0, 0, 1)],
    )
    assert core.solution(msg)
    after = core.envelope()
    assert after["chunks"] == before["chunks"]
    submap = after["snapshot"]["manifests"][0]["submaps"][0]
    assert submap["T_component_submap"][0][3] == 4
    assert not core.solution(msg)
    msg.solution_clock = 1
    assert not core.solution(msg)
    msg.solution_clock = 3
    msg.mission_id = str(uuid.uuid4())
    assert not core.solution(msg)
    replica = ReplicaStore(tmp_path / "replica")
    for c in after["chunks"]:
        replica.put_chunk(c["sha256"], mapper.get_chunk(c["sha256"]))
    assert replica.publish(after)
    assert not replica.publish(core.envelope())
    # A new observation follows the most recent optimized pose without moving
    # old geometry or requiring the server to send corrections back.
    pose = np.eye(4)
    pose[0, 3] = 2
    core.capture(1, 20, pose, [[0, 0, 0]])
    assert core.poses[core.key(1)][0][3] == 6


def test_unanchored_solution_and_missing_initial_keyframe_fail_closed(tmp_path):
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path)),
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
    )
    with pytest.raises(ValueError, match="First keyframe"):
        core.capture(3, 1, IDENTITY_SE3, [[0, 0, 0]])
    msg = NS(
        success=True,
        mission_id=core.mission_id,
        solution_clock=1,
        optimizer_robot_id=0,
        anchor_estimates=[],
    )
    assert not core.solution(msg)


def test_normalized_points_keep_physical_lidar_origin(tmp_path):
    mapper = CorrectionAwareMapper(SubmapStore(tmp_path))
    core = CslamMapper(mapper, "r0", 0, str(uuid.uuid4()), {0: "r0"})
    mount = np.eye(4)
    mount[2, 3] = 1.2
    core.capture(
        0, 1, IDENTITY_SE3, [[3, 0, 1.2]], T_base_sensor=mount, sensor_frame="r0/lidar"
    )
    submap = mapper.store.active_submaps()[0]
    assert submap.sensor_origins == ((0, 0, 1.2),)


def test_identical_solver_results_do_not_rename_the_component_frame(tmp_path):
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path)),
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
    )
    core.capture(0, 1, IDENTITY_SE3, [[1, 0, 0]])
    msg = NS(
        success=True,
        mission_id=core.mission_id,
        solution_clock=1,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, 0)],
        anchor_estimates=[value(0, 0, 0)],
    )
    revision = core.revision
    initial = core.envelope()
    # The solver reports every few seconds whether or not anything moved. A
    # goal is only accepted for the advertised solution order, and the replica
    # cannot follow one revision per report, so an unchanged result must leave
    # the order, the map revision and the replica revision alone.
    for clock in (1, 2, 3):
        msg.solution_clock = clock
        assert not core.solution(msg)
        assert core.solution_order == (0, -1)
        assert core.revision == revision
        assert core.correction_revision == 0
        assert core.envelope() == initial

    # The clock is still remembered: a replayed result is not processed again.
    msg.solution_clock = 2
    msg.estimates = [value(0, 0, 1)]
    msg.anchor_estimates = [value(0, 0, 1)]
    assert not core.solution(msg)
    assert core.correction_revision == 0

    msg.solution_clock = 4
    assert core.solution(msg)
    assert core.solution_order == (4, 0)
    assert core.revision == revision + 1
    assert core.envelope()["revision"] == initial["revision"] + 1
    assert core.correction_revision == 1


def test_snapshot_publication_ignores_replica_only_updates(tmp_path):
    path = tmp_path / "snapshot.json"
    first = {"snapshot_id": "first", "manifests": [{"revision": 1}]}
    published = publish_snapshot_if_new(path, first, 1, -1)
    identity = path.stat().st_ino

    replica_only = {"snapshot_id": "replica-only", "manifests": []}
    published = publish_snapshot_if_new(path, replica_only, 1, published)

    assert published == 1
    assert path.stat().st_ino == identity
    assert json.loads(path.read_text()) == first

    second = {"snapshot_id": "second", "manifests": [{"revision": 2}]}
    published = publish_snapshot_if_new(path, second, 2, published)
    assert published == 2
    assert path.stat().st_ino != identity
    assert json.loads(path.read_text()) == second


def test_selected_provider_does_not_bless_a_cslam_keyframe_cloud(tmp_path):
    mapper = CorrectionAwareMapper(SubmapStore(tmp_path))
    core = CslamMapper(
        mapper,
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
        "simulation",
    )
    core.capture(0, 10, IDENTITY_SE3, [[1, 0, 0]])
    stored = mapper.store.get_capture(core.key(0))["capture"]
    assert stored["deskew_status"] == DeskewStatus.UNKNOWN.value
    assert stored["ray_return_semantics"] == RayReturnSemantics.UNKNOWN.value


def test_raw_simulation_capture_is_explicitly_paired_to_keyframe(tmp_path):
    mapper = CorrectionAwareMapper(SubmapStore(tmp_path))
    mission = str(uuid.uuid4())
    core = CslamMapper(mapper, "r0", 0, mission, {0: "r0"}, "simulation")
    keyframe = core.key(0)
    evidence = CaptureProvenance(
        keyframe,
        CaptureGeometry.RAW_RAY_CAPTURE,
        10,
        10,
        10,
        CaptureClock.ROS_SIM_TIME,
        mission,
        True,
        source_contract="argos.photorealistic_lidar.hit_endpoints.single_tick.v1",
    )
    core.capture(0, 10, IDENTITY_SE3, [[1, 0, 0]], provenance=evidence)
    stored = mapper.store.get_capture(keyframe)["capture"]
    assert stored["deskew_status"] == DeskewStatus.NOT_REQUIRED.value
    assert stored["ray_return_semantics"] == RayReturnSemantics.FIRST_RETURN.value
    assert mapper.store.active_submaps()[0].ray_evidence.certifies_free_space


def test_keyframe_identity_includes_provider_evidence(tmp_path):
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path)),
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
        "simulation",
    )
    core.capture(0, 10, IDENTITY_SE3, [[1, 0, 0]])
    keyframe = core.key(0)
    evidence = CaptureProvenance(
        keyframe,
        CaptureGeometry.RAW_RAY_CAPTURE,
        10,
        10,
        10,
        CaptureClock.ROS_SIM_TIME,
        keyframe.session_id,
        True,
        source_contract="argos.photorealistic_lidar.hit_endpoints.single_tick.v1",
    )
    with pytest.raises(ValueError, match="different capture data"):
        core.capture(0, 10, IDENTITY_SE3, [[1, 0, 0]], provenance=evidence)


def test_solver_noise_below_tolerance_is_not_a_correction(tmp_path):
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path / "maps")),
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
    )
    core.capture(0, 1, IDENTITY_SE3, [[1, 0, 0]])
    msg = NS(
        success=True,
        mission_id=core.mission_id,
        solution_clock=1,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, 0)],
        anchor_estimates=[value(0, 0, 0)],
    )
    assert not core.solution(msg)
    revision = core.revision
    replica_revision = core.envelope()["revision"]

    # One millimetre of solver noise: neither the frame nor the geometry moves.
    msg.solution_clock = 2
    msg.estimates = [value(0, 0, 0.001)]
    msg.anchor_estimates = [value(0, 0, 0.001)]
    assert not core.solution(msg)
    assert core.solution_order == (0, -1)
    assert core.revision == revision
    assert core.correction_revision == 0
    assert core.envelope()["revision"] == replica_revision

    # Repeated noise that never exceeds the tolerance still applies nothing.
    msg.solution_clock = 3
    msg.estimates = [value(0, 0, 0.004)]
    msg.anchor_estimates = [value(0, 0, 0.004)]
    assert not core.solution(msg)
    assert core.revision == revision

    # A real correction beyond the tolerance replaces geometry.
    msg.solution_clock = 4
    msg.estimates = [value(0, 0, 0.02)]
    msg.anchor_estimates = [value(0, 0, 0.02)]
    assert core.solution(msg)
    assert core.revision == revision + 1
    assert core.correction_revision == 1

    # Drift accumulates against the applied poses, not the last noisy result.
    msg.solution_clock = 5
    msg.estimates = [value(0, 0, 0.024)]
    msg.anchor_estimates = [value(0, 0, 0.024)]
    assert not core.solution(msg)
    msg.solution_clock = 6
    msg.estimates = [value(0, 0, 0.026)]
    msg.anchor_estimates = [value(0, 0, 0.026)]
    assert core.solution(msg)
    assert core.correction_revision == 2


def translated(x):
    pose = np.eye(4)
    pose[0, 3] = x
    return pose


def test_frame_history_records_the_frame_in_effect_at_each_revision(tmp_path):
    mission = str(uuid.uuid4())
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path / "map")),
        "r1",
        1,
        mission,
        {0: "r0", 1: "r1"},
    )
    assert core.frame_history == {}
    core.capture(0, 10, IDENTITY_SE3, [[0, 0, 0], [1, 0, 0]])
    core.capture(1, 20, translated(2.0), [[0, 0, 0]])
    assert sorted(core.frame_history) == [1, 2]
    for revision in (1, 2):
        frame = core.frame_history[revision]
        assert isinstance(frame, FrameState)
        assert frame.component_id == core.envelope()["component_id"]
        assert frame.epoch == 0
        np.testing.assert_allclose(frame.T_component_local, np.eye(4))
        assert frame.correction_revision == 0
        assert frame.solution_order == (0, -1)
        np.testing.assert_allclose(frame.T_component_home, np.eye(4))

    # A solution places the home keyframe at x=4 and the anchor at x=1: the
    # accepted revision carries the correction and the solver's order, while
    # the entries recorded before it keep the frame they were placed in.
    msg = NS(
        success=True,
        mission_id=mission,
        solution_clock=2,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(1, 0, 4), value(1, 1, 6)],
        anchor_estimates=[value(0, 0, 1)],
    )
    assert core.solution(msg)
    assert core.revision == 3
    corrected = core.frame_history[3]
    assert corrected.component_id != core.frame_history[2].component_id
    assert corrected.epoch == 1
    np.testing.assert_allclose(corrected.T_component_local, translated(4.0))
    assert corrected.correction_revision == 1
    assert corrected.solution_order == (2, 0)
    np.testing.assert_allclose(corrected.T_component_home, translated(4.0))
    before = core.frame_history[2]
    np.testing.assert_allclose(before.T_component_local, np.eye(4))
    assert before.solution_order == (0, -1)
    assert before.correction_revision == 0

    # A later capture inherits the corrected frame at its own revision.
    core.capture(2, 30, translated(3.0), [[0, 0, 0]])
    later = core.frame_history[4]
    np.testing.assert_allclose(later.T_component_local, translated(4.0))
    assert later.solution_order == (2, 0)
    assert later.correction_revision == 1
    # The recorded matrices are immutable copies, not the live correction.
    assert isinstance(later.T_component_local, tuple)


def test_frame_history_is_bounded_and_drops_the_oldest_revisions(tmp_path):
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path / "map")),
        "r0",
        0,
        str(uuid.uuid4()),
        {0: "r0"},
    )
    assert FRAME_HISTORY_LIMIT == 4096
    assert core.frame_history_limit == FRAME_HISTORY_LIMIT
    core.frame_history_limit = 3
    for seq in range(6):
        core.capture(seq, 10 * (seq + 1), translated(float(seq)), [[0, 0, 0]])
    assert core.revision == 6
    assert sorted(core.frame_history) == [4, 5, 6]
    core.frame_history_limit = 0
    core.capture(6, 70, translated(6.0), [[0, 0, 0]])
    # The current revision is always remembered, whatever the bound.
    assert sorted(core.frame_history) == [7]
