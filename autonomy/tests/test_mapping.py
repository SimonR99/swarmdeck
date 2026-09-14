from __future__ import annotations

import hashlib
import json
import sqlite3
import uuid

import numpy as np
import pytest

from autonomy.contracts import (
    IDENTITY_SE3,
    ZERO_COVARIANCE,
    Calibration,
    CalibratedCapture,
    ComponentRevision,
    DeskewStatus,
    GraphSolution,
    KeyframeId,
    RayEvidence,
    RayOriginAssociation,
    RayReturnSemantics,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.mapping import (
    MAX_CHUNK_BYTES,
    CorrectionAwareMapper,
    DuplicateConflictError,
    OccupancyState,
    StorageBudgetExceeded,
    StaleSolutionError,
    SubmapStore,
    decode_xyz_f32,
    decode_xyzrgba_f32_u8,
)

SESSION = str(uuid.UUID("fe721f0d-82b6-4678-bacb-a627ae5b51f4"))


def pose(x: float, y: float = 0.0, z: float = 0.0):
    value = [list(row) for row in IDENTITY_SE3]
    value[0][3], value[1][3], value[2][3] = x, y, z
    return tuple(tuple(row) for row in value)


def mapper(tmp_path):
    return CorrectionAwareMapper(SubmapStore(tmp_path), resolution_m=0.1)


def capture(
    seq: int = 0,
    *,
    end_ns: int = 100,
    T_local_base=IDENTITY_SE3,
    covariance=ZERO_COVARIANCE,
    deskew_status=DeskewStatus.DESKEWED,
    ray_return_semantics=RayReturnSemantics.UNKNOWN,
):
    return CalibratedCapture(
        KeyframeId("r1", SESSION, seq),
        end_ns - 10,
        end_ns,
        "r1/lidar",
        "lidar-v1",
        T_local_base,
        covariance,
        deskew_status,
        ray_return_semantics=ray_return_semantics,
    )


def calibration():
    return Calibration(
        "lidar-v1", "r1/lidar", "x-forward/y-left/z-up", (), "none", (), IDENTITY_SE3
    )


def solution(
    keyframe: KeyframeId, x: float, revision: int, *, epoch: int = 0, **kwargs
):
    return GraphSolution(
        ComponentRevision(component_id_for_anchor(keyframe), epoch, revision),
        keyframe,
        (keyframe,),
        {keyframe: pose(x)},
        **kwargs,
    )


def test_corrected_snapshot_reuses_immutable_geometry(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    cloud = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    assert m.add_capture(cap, calibration(), cloud) == 0
    before = m.snapshot()
    chunk = before.manifests[0].chunks[0]

    assert m.apply_solution(solution(cap.keyframe_id, 4.0, 1))
    after = m.snapshot()
    submap = after.manifests[0].submaps[0]
    assert submap.T_component_submap[0][3] == 4.0
    assert after.manifests[0].chunks[0].sha256 == chunk.sha256
    np.testing.assert_allclose(decode_xyz_f32(m.get_chunk(chunk.sha256)), cloud)
    # Transport output is canonical JSON and repeats each chunk only once.
    encoded = json.dumps(m.snapshot_dict(), sort_keys=True)
    assert (
        encoded.count(chunk.sha256) == 2
    )  # submap reference + flat manifest chunk table


def test_colored_capture_preserves_measured_rgba_in_versioned_chunk(tmp_path) -> None:
    m = mapper(tmp_path)
    cloud = np.array([[0, 0, 0], [1, 0, 0]], dtype=float)
    rgba = np.array([[240, 10, 20, 255], [148, 148, 148, 0]], dtype=np.uint8)
    m.add_capture(capture(), calibration(), cloud, colors_rgba=rgba)
    chunk = m.snapshot().manifests[0].chunks[0]
    assert chunk.encoding.endswith("xyzrgba-f32-u8.v1")
    points, colors = decode_xyzrgba_f32_u8(m.get_chunk(chunk.sha256))
    np.testing.assert_allclose(points, cloud)
    np.testing.assert_array_equal(colors, rgba)


def test_capture_local_pose_is_visible_before_first_graph_solution(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture(T_local_base=pose(3.5))
    m.add_capture(cap, calibration(), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    submap = m.snapshot().manifests[0].submaps[0]
    assert submap.T_component_submap[0][3] == 3.5


def test_calibrated_capture_is_durable_and_version_is_immutable(tmp_path) -> None:
    store_path = tmp_path / "store"
    m = CorrectionAwareMapper(SubmapStore(store_path))
    cap = capture()
    m.add_capture(cap, calibration(), [[0, 0, 0]])
    m.store.close()

    reopened = SubmapStore(store_path)
    record = reopened.get_capture(cap.keyframe_id)
    assert record["capture"]["deskew_status"] == "deskewed"
    assert record["calibration"]["sensor_frame"] == "r1/lidar"
    changed = Calibration(
        "lidar-v1", "r1/lidar", "x-forward/y-left/z-up", (), "none", (), pose(1)
    )
    with pytest.raises(DuplicateConflictError, match="calibration version"):
        reopened.record_capture(capture(1), changed)
    reopened.close()


def test_unknown_capture_covariance_round_trips_as_none(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture(covariance=None)
    m.add_capture(cap, calibration(), [[0, 0, 0]])
    assert m.store.get_capture(cap.keyframe_id)["capture"]["covariance"] is None


def test_geometry_replacement_removes_old_active_contribution(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    m.add_capture(cap, calibration(), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    old_hash = m.snapshot().manifests[0].chunks[0].sha256
    assert (
        m.add_capture(
            cap, calibration(), [[10, 0, 0], [11, 0, 0], [10, 1, 0]], replace=True
        )
        == 1
    )
    manifest = m.snapshot().manifests[0]
    assert manifest.submaps[0].replaces_geometry_revision == 0
    assert old_hash not in {chunk.sha256 for chunk in manifest.chunks}
    assert m.get_chunk(
        old_hash
    )  # immutable history remains available for journal replay
    component = component_id_for_anchor(cap.keyframe_id)
    assert m.occupancy_query(component, (0, 1, 0)).state is OccupancyState.UNKNOWN
    occupied = m.occupancy_query(component, (10, 0, 0))
    assert occupied.state is OccupancyState.OCCUPIED
    assert occupied.geometry_revision == manifest.geometry_revision


def test_duplicate_stale_and_conflicting_solutions(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    m.add_capture(cap, calibration(), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    update = solution(cap.keyframe_id, 2, 2)
    assert m.apply_solution(update)
    assert not m.apply_solution(update)
    with pytest.raises(DuplicateConflictError):
        m.apply_solution(solution(cap.keyframe_id, 3, 2))
    with pytest.raises(StaleSolutionError):
        m.apply_solution(solution(cap.keyframe_id, 1, 1))


def test_epoch_reset_can_restart_revision_counter(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    m.add_capture(cap, calibration(), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    m.apply_solution(solution(cap.keyframe_id, 5, 10, epoch=1))
    m.apply_solution(solution(cap.keyframe_id, 6, 0, epoch=2))
    submap = m.snapshot().manifests[0].submaps[0]
    assert submap.pose_revision == ComponentRevision(
        component_id_for_anchor(cap.keyframe_id), 2, 0
    )
    assert submap.T_component_submap[0][3] == 6


def test_retracted_keyframe_tombstones_and_removes_geometry(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    m.add_capture(cap, calibration(), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    retract = GraphSolution(
        ComponentRevision(component_id_for_anchor(cap.keyframe_id), 0, 1),
        cap.keyframe_id,
        (cap.keyframe_id,),
        {cap.keyframe_id: IDENTITY_SE3},
        retracted_constraints=("bad-loop",),
    )
    # Constraint retraction is provenance only and does not remove the keyframe.
    assert m.apply_solution(retract)
    second = KeyframeId("r1", SESSION, 1)
    retract_keyframe = GraphSolution(
        ComponentRevision(component_id_for_anchor(cap.keyframe_id), 0, 2),
        cap.keyframe_id,
        (cap.keyframe_id,),
        {cap.keyframe_id: IDENTITY_SE3},
        retracted_keyframes=(second,),
    )
    m.add_capture(capture(1), calibration(), [[2, 0, 0], [3, 0, 0], [2, 1, 0]])
    assert m.apply_solution(retract_keyframe)
    snapshot = m.snapshot()
    assert sum(len(manifest.submaps) for manifest in snapshot.manifests) == 1
    assert any(
        "keyframe_retracted" in value
        for manifest in snapshot.manifests
        for value in manifest.tombstones
    )

    # A later correction advances both the live geometry and the retained
    # negative membership. It must not leave a historical tombstone-only
    # manifest with the same component ID, which snapshot consumers reject.
    correction = solution(cap.keyframe_id, 3, 3)
    assert m.apply_solution(correction)
    snapshot = m.snapshot()
    assert len(snapshot.manifests) == 1
    assert snapshot.manifests[0].graph_revision == correction.revision
    assert len(snapshot.manifests[0].submaps) == 1
    assert snapshot.manifests[0].tombstones == (
        f"r1/{SESSION}/submap/1:keyframe_retracted",
    )

    m.store.close()
    reopened = SubmapStore(tmp_path)
    persisted = reopened.snapshot()
    assert len(persisted.manifests) == 1
    assert persisted.manifests[0].graph_revision == correction.revision
    assert persisted.manifests[0].tombstones == snapshot.manifests[0].tombstones
    reopened.close()


def test_terrain_query_reports_surface_and_unknown(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture(end_ns=1000)
    floor = [[x, y, 0.0] for x in (-0.2, 0, 0.2) for y in (-0.2, 0, 0.2)]
    m.add_capture(cap, calibration(), floor)
    component = component_id_for_anchor(cap.keyframe_id)
    terrain = m.terrain_query(component, (0, 0, 0.1), now_ns=1200)
    assert terrain.known
    assert terrain.ground_height_m == pytest.approx(0)
    assert terrain.ground_normal is not None and terrain.ground_normal[2] > 0.99
    assert terrain.clearance_m is None
    assert terrain.observation_age_ns == 200
    assert not m.terrain_query("component:missing", (0, 0, 0)).known


def test_nonfinite_cloud_never_enters_store(tmp_path) -> None:
    m = mapper(tmp_path)
    with pytest.raises(ValueError, match="nonfinite"):
        m.add_capture(capture(), calibration(), [[0, 0, float("nan")]])
    assert not m.snapshot().manifests


def test_store_fails_closed_at_configured_chunk_budget(tmp_path) -> None:
    m = CorrectionAwareMapper(SubmapStore(tmp_path, max_chunk_bytes=27))
    with pytest.raises(StorageBudgetExceeded, match="budget"):
        m.add_capture(capture(), calibration(), [[0, 0, 0]])
    assert not m.snapshot().manifests
    assert not list((tmp_path / "chunks").iterdir())


def test_chunk_boundary_rejects_oversize_and_path_digest(tmp_path) -> None:
    store = SubmapStore(tmp_path, max_chunk_bytes=MAX_CHUNK_BYTES * 2)
    with pytest.raises(ValueError, match="interchange limit"):
        store.put_chunk(
            b"x" * (MAX_CHUNK_BYTES + 1), point_count=0, bounds=((0, 0, 0), (0, 0, 0))
        )
    with pytest.raises(KeyError):
        store.get_chunk("../" + "a" * 61)


def test_multiple_unassociated_origins_never_infer_free_space(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture()
    m.store.add_submap(
        SubmapId.from_keyframe(cap.keyframe_id),
        [[2, 0, 0]],
        keyframe_poses_local={cap.keyframe_id: IDENTITY_SE3},
        sensor_origins_local=((0, 0, 0), (0, 1, 0)),
        resolution_m=0.1,
        observed_at_ns=cap.capture_end_ns,
    )
    component = component_id_for_anchor(cap.keyframe_id)
    assert m.occupancy_query(component, (1, 0, 0)).state is OccupancyState.UNKNOWN
    assert m.occupancy_query(component, (2, 0, 0)).state is OccupancyState.OCCUPIED


@pytest.mark.parametrize(
    "deskew_status", [DeskewStatus.DESKEWED, DeskewStatus.NOT_REQUIRED]
)
def test_explicit_motion_compensated_first_returns_certify_free_rays(
    tmp_path, deskew_status
) -> None:
    store_path = tmp_path / "qualified"
    m = CorrectionAwareMapper(SubmapStore(store_path), resolution_m=0.1)
    cap = capture(
        ray_return_semantics=RayReturnSemantics.FIRST_RETURN,
        deskew_status=deskew_status,
    )
    m.add_capture(cap, calibration(), [[2, 0, 0]])
    submap = m.snapshot().manifests[0].submaps[0]
    assert submap.ray_evidence == RayEvidence(
        RayReturnSemantics.FIRST_RETURN,
        deskew_status,
        RayOriginAssociation.SINGLE_CAPTURE,
    )
    encoded = m.snapshot_dict()["manifests"][0]["submaps"][0]["ray_evidence"]
    assert encoded == {
        "return_semantics": "first_return",
        "deskew": deskew_status.value,
        "origin_association": "single_capture",
    }
    component = component_id_for_anchor(cap.keyframe_id)
    assert m.occupancy_query(component, (1, 0, 0)).state is OccupancyState.FREE
    m.store.close()

    reopened = CorrectionAwareMapper(SubmapStore(store_path), resolution_m=0.1)
    assert reopened.snapshot().manifests[0].submaps[0].ray_evidence.certifies_free_space
    assert reopened.occupancy_query(component, (1, 0, 0)).state is OccupancyState.FREE
    reopened.store.close()


@pytest.mark.parametrize(
    ("deskew_status", "return_semantics"),
    [
        (DeskewStatus.UNKNOWN, RayReturnSemantics.FIRST_RETURN),
        (DeskewStatus.DESKEWED, RayReturnSemantics.UNKNOWN),
    ],
)
def test_incomplete_capture_provenance_stays_unknown(
    tmp_path, deskew_status, return_semantics
) -> None:
    m = mapper(tmp_path)
    cap = capture(deskew_status=deskew_status, ray_return_semantics=return_semantics)
    m.add_capture(cap, calibration(), [[2, 0, 0]])
    submap = m.snapshot().manifests[0].submaps[0]
    assert submap.ray_evidence == RayEvidence()
    component = component_id_for_anchor(cap.keyframe_id)
    assert m.occupancy_query(component, (1, 0, 0)).state is OccupancyState.UNKNOWN
    assert m.occupancy_query(component, (2, 0, 0)).state is OccupancyState.OCCUPIED


def test_generic_replacement_drops_qualified_ray_evidence(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture(ray_return_semantics=RayReturnSemantics.FIRST_RETURN)
    m.add_capture(cap, calibration(), [[2, 0, 0]])
    component = component_id_for_anchor(cap.keyframe_id)
    assert m.occupancy_query(component, (1, 0, 0)).state is OccupancyState.FREE

    m.replace_submap_geometry(
        SubmapId.from_keyframe(cap.keyframe_id),
        [[3, 0, 0]],
        keyframe_poses_local={cap.keyframe_id: IDENTITY_SE3},
        sensor_origins_local=((0, 0, 0),),
        observed_at_ns=cap.capture_end_ns,
    )
    assert m.snapshot().manifests[0].submaps[0].ray_evidence == RayEvidence()
    assert m.occupancy_query(component, (1, 0, 0)).state is OccupancyState.UNKNOWN
    assert m.occupancy_query(component, (3, 0, 0)).state is OccupancyState.OCCUPIED


def test_qualified_origin_and_timestamp_must_match_durable_capture(tmp_path) -> None:
    m = mapper(tmp_path)
    cap = capture(ray_return_semantics=RayReturnSemantics.FIRST_RETURN)
    m.store.record_capture(cap, calibration())
    qualified = RayEvidence(
        RayReturnSemantics.FIRST_RETURN,
        DeskewStatus.DESKEWED,
        RayOriginAssociation.SINGLE_CAPTURE,
    )
    with pytest.raises(ValueError, match="origin"):
        m.store.add_submap(
            SubmapId.from_keyframe(cap.keyframe_id),
            [[2, 0, 0]],
            keyframe_poses_local={cap.keyframe_id: IDENTITY_SE3},
            sensor_origins_local=((0.1, 0, 0),),
            resolution_m=0.1,
            observed_at_ns=cap.capture_end_ns,
            ray_evidence=qualified,
        )
    with pytest.raises(ValueError, match="timestamp"):
        m.store.add_submap(
            SubmapId.from_keyframe(cap.keyframe_id),
            [[2, 0, 0]],
            keyframe_poses_local={cap.keyframe_id: IDENTITY_SE3},
            sensor_origins_local=((0, 0, 0),),
            resolution_m=0.1,
            observed_at_ns=cap.capture_end_ns + 1,
            ray_evidence=qualified,
        )
    assert not m.snapshot().manifests
    assert not list((tmp_path / "chunks").iterdir())


def test_legacy_unknown_ray_rows_replay_after_restart(tmp_path) -> None:
    store_path = tmp_path / "legacy"
    m = CorrectionAwareMapper(SubmapStore(store_path))
    cap = capture()
    cloud = [[2, 0, 0]]
    assert m.add_capture(cap, calibration(), cloud) == 0
    m.store.close()

    database = sqlite3.connect(store_path / "mapping.sqlite3")
    capture_json, calibration_json = database.execute(
        "SELECT capture_json, calibration_json FROM captures"
    ).fetchone()
    capture_value = json.loads(capture_json)
    capture_value.pop("ray_return_semantics")
    old_capture_json = json.dumps(capture_value, sort_keys=True, separators=(",", ":"))
    old_digest = hashlib.sha256(
        (old_capture_json + "\n" + calibration_json).encode()
    ).hexdigest()
    database.execute(
        "UPDATE captures SET digest=?, capture_json=?", (old_digest, old_capture_json)
    )
    database.execute("UPDATE submap_revisions SET ray_evidence_json=NULL")
    database.commit()
    database.close()

    reopened = CorrectionAwareMapper(SubmapStore(store_path))
    assert reopened.snapshot().manifests[0].submaps[0].ray_evidence == RayEvidence()
    assert reopened.add_capture(cap, calibration(), cloud) == 0
    record = reopened.store.get_capture(cap.keyframe_id)
    assert record["capture"]["ray_return_semantics"] == "unknown"
    reopened.store.close()
