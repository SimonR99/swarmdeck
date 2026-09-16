from types import SimpleNamespace as NS
import json
import uuid
import numpy as np
import pytest
from autonomy.capture_providers import CaptureClock, CaptureGeometry, CaptureProvenance
from autonomy.contracts import DeskewStatus, IDENTITY_SE3, RayReturnSemantics
from autonomy.cslam import CslamMapper, publish_snapshot_if_new
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


def test_identical_new_solver_result_advances_replica_without_map_revision(tmp_path):
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
    replica_revision = initial["revision"]
    snapshot_id = initial["snapshot"]["snapshot_id"]
    geometry_revision = initial["snapshot"]["manifests"][0]["geometry_revision"]
    replica = ReplicaStore(tmp_path / "replica")
    for chunk in initial["chunks"]:
        replica.put_chunk(chunk["sha256"], core.mapper.get_chunk(chunk["sha256"]))
    assert replica.publish(initial)
    assert not core.solution(msg)
    assert core.solution_order == (1, 0)
    assert core.revision == revision
    assert core.correction_revision == 0
    envelope = core.envelope()
    assert envelope["solution_order"] == [1, 0]
    assert envelope["revision"] == replica_revision + 1
    assert envelope["snapshot"] == initial["snapshot"]
    assert envelope["snapshot"]["snapshot_id"] == snapshot_id
    assert (
        envelope["snapshot"]["manifests"][0]["graph_revision"]["revision"] == revision
    )
    assert (
        envelope["snapshot"]["manifests"][0]["geometry_revision"] == geometry_revision
    )
    assert replica.publish(envelope)

    msg.solution_clock = 2
    assert not core.solution(msg)
    second_noop = core.envelope()
    assert core.revision == revision
    assert second_noop["revision"] == replica_revision + 2
    assert second_noop["snapshot"] == initial["snapshot"]
    assert replica.publish(second_noop)

    msg.solution_clock = 3
    msg.estimates = [value(0, 0, 1)]
    msg.anchor_estimates = [value(0, 0, 1)]
    assert core.solution(msg)
    assert core.revision == revision + 1
    assert core.envelope()["revision"] == replica_revision + 3
    assert core.correction_revision == 1
    replica.close()


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

    # One millimetre of solver noise: causal state advances, geometry does not.
    msg.solution_clock = 2
    msg.estimates = [value(0, 0, 0.001)]
    msg.anchor_estimates = [value(0, 0, 0.001)]
    assert not core.solution(msg)
    assert core.solution_order == (2, 0)
    assert core.revision == revision
    assert core.correction_revision == 0
    assert core.envelope()["revision"] == replica_revision + 1

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
