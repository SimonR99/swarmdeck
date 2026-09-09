from types import SimpleNamespace as NS
import uuid
import numpy as np
import pytest
from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
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


def test_identical_new_solver_result_advances_clock_without_invalidating_map(tmp_path):
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
    assert not core.solution(msg)
    assert core.solution_order == (1, 0)
    assert core.revision == revision
    assert core.correction_revision == 0
    msg.solution_clock = 2
    msg.estimates = [value(0, 0, 1)]
    msg.anchor_estimates = [value(0, 0, 1)]
    assert core.solution(msg)
    assert core.correction_revision == 1
