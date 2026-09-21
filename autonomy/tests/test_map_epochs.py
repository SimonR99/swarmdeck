from types import SimpleNamespace as NS
from uuid import uuid4

import numpy as np
import pytest

from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
from autonomy.map_epochs import claim_map_epoch, read_map_epoch, robot_run_id
from autonomy.mapping import CorrectionAwareMapper, SubmapStore


def core(root, mission, robot, index, epoch):
    return CslamMapper(
        CorrectionAwareMapper(SubmapStore(root)),
        robot,
        index,
        mission,
        {0: "r0", 1: "r1"},
        map_epoch=epoch,
    )


def pose(robot, x):
    return NS(
        key=NS(robot_id=robot, keyframe_id=0),
        pose=NS(position=NS(x=x, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
    )


def solution(mission, epochs, clock, target=1):
    return NS(
        mission_id=mission,
        map_epoch=epochs[0],
        publisher_robot_id=0,
        robot_map_epochs=epochs,
        participant_robot_ids=[0, 1],
        success=True,
        solution_clock=clock,
        optimizer_robot_id=0,
        origin_robot_id=0,
        anchor_estimates=[pose(0, 0)],
        estimates=[pose(target, 4)],
    )


def test_restart_claim_discards_only_target_live_store(tmp_path):
    mission = str(uuid4())
    for robot in ("r0", "r1"):
        assert claim_map_epoch(tmp_path, mission, robot) == 0
        root = tmp_path / mission / robot
        (root / "snapshot.json").write_text(robot)
        (root / "geometry" / "old-keyframe").write_text(robot)
    old_run = read_map_epoch(tmp_path / mission / "r0")["run_id"]
    assert claim_map_epoch(tmp_path, mission, "r0") == 1
    target, peer = tmp_path / mission / "r0", tmp_path / mission / "r1"
    assert not (target / "snapshot.json").exists()
    assert not (target / "geometry" / "old-keyframe").exists()
    assert (peer / "snapshot.json").read_text() == "r1"
    assert (peer / "geometry" / "old-keyframe").read_text() == "r1"
    assert read_map_epoch(peer)["map_epoch"] == 0
    assert read_map_epoch(target)["run_id"] != old_run
    assert claim_map_epoch(tmp_path, mission, "r0", minimum=5) == 5
    assert claim_map_epoch(tmp_path, mission, "r0") == 6


def test_corrupt_epoch_record_never_restarts_counter(tmp_path):
    mission = str(uuid4())
    claim_map_epoch(tmp_path, mission, "r0")
    root = tmp_path / mission / "r0"
    for invalid in ("{}", '{"version":1,"mission_id":null}', "{"):
        (root / "map-epoch.json").write_text(invalid)
        with pytest.raises(ValueError):
            claim_map_epoch(tmp_path, mission, "r0")


def test_peer_anchor_epoch_advance_retains_own_captures_and_fences_delayed_solver(
    tmp_path,
):
    mission = str(uuid4())
    mapper = core(tmp_path, mission, "r1", 1, 0)
    mapper.capture(0, 1, IDENTITY_SE3, [[1, 0, 0]])
    assert mapper.solution(solution(mission, [0, 0], 1))
    old = mapper.envelope()
    old_anchor = mapper.anchor
    own_key = mapper.key(0)
    assert mapper.observe_epoch(0, 1)
    fresh = mapper.envelope()
    assert mapper.key(0) == own_key
    assert fresh["session_id"] == mission
    assert fresh["chunks"] == old["chunks"]
    assert mapper.anchor == own_key
    assert old_anchor not in mapper.poses
    assert not mapper.solution(solution(mission, [0, 0], 10_000))
    assert mapper.anchor == own_key
    assert fresh["robot_map_epochs"] == {"r0": 1, "r1": 0}


def test_new_lifetime_home_is_first_capture_and_old_solution_cannot_apply(tmp_path):
    mission = str(uuid4())
    old = core(tmp_path / "old", mission, "r0", 0, 0)
    old.capture(0, 1, IDENTITY_SE3, [[1, 0, 0]])
    fresh = core(tmp_path / "new", mission, "r0", 0, 1)
    at_clear = np.eye(4)
    at_clear[0, 3] = 12.5
    fresh.capture(0, 2, at_clear, [[1, 0, 0]])
    assert fresh.key(0) != old.key(0)
    assert fresh.run_id == robot_run_id(mission, "r0", 1)
    assert fresh.frame_history[1].T_component_home[0][3] == 12.5
    assert not fresh.solution(solution(mission, [0, 0], 100, target=0))
    assert fresh.poses[fresh.key(0)][0][3] == 12.5
