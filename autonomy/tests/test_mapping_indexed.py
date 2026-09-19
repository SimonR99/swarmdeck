from __future__ import annotations

import copy
import hashlib
import json
import math
import uuid
from pathlib import Path

import numpy as np
import pytest

from autonomy.capture_providers import endpoint_preserving_sample
from autonomy.contracts import (
    IDENTITY_SE3,
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
from autonomy.indexed_mapping import (
    IndexedGrid,
    IndexedMapView,
    QueryRequest,
    QueryStatus,
    SnapshotKey,
    VoxelOccupancy,
)
from autonomy.mapping import SubmapStore

SESSION = str(uuid.UUID("8c095aed-b093-43a4-997f-495b2a8b734c"))
QUALIFIED_RAYS = RayEvidence(
    RayReturnSemantics.FIRST_RETURN,
    DeskewStatus.DESKEWED,
    RayOriginAssociation.SINGLE_CAPTURE,
)


def pose(x: float = 0.0):
    result = [list(row) for row in IDENTITY_SE3]
    result[0][3] = x
    return tuple(tuple(row) for row in result)


def make_store(tmp_path, points, *, origin=((-0.1, 0.1, 0.5),)):
    store = SubmapStore(tmp_path)
    keyframe = KeyframeId("r0", SESSION, 0)
    evidence = RayEvidence()
    if len(origin) == 1:
        record_qualified_capture(store, keyframe, origin[0], observed_at_ns=100)
        evidence = QUALIFIED_RAYS
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        points,
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=origin,
        resolution_m=0.2,
        observed_at_ns=100,
        ray_evidence=evidence,
    )
    return store, keyframe


def record_qualified_capture(store, keyframe, origin, *, observed_at_ns):
    transform = [list(row) for row in IDENTITY_SE3]
    for axis, value in enumerate(origin):
        transform[axis][3] = value
    sensor_frame = f"{keyframe.robot_id}/lidar"
    calibration = Calibration(
        "lidar-v1",
        sensor_frame,
        "x-forward/y-left/z-up",
        (),
        "none",
        (),
        tuple(tuple(row) for row in transform),
    )
    capture = CalibratedCapture(
        keyframe,
        observed_at_ns,
        observed_at_ns,
        sensor_frame,
        calibration.version,
        IDENTITY_SE3,
        None,
        DeskewStatus.DESKEWED,
        ray_return_semantics=RayReturnSemantics.FIRST_RETURN,
    )
    store.record_capture(capture, calibration)


def snapshot_key(snapshot, component):
    manifest = next(
        m for m in snapshot.manifests if m.graph_revision.component_id == component
    )
    return SnapshotKey(
        component,
        manifest.graph_revision.epoch,
        manifest.graph_revision.revision,
        manifest.geometry_revision,
    )


def repair_snapshot_id(value):
    value["snapshot_id"] = hashlib.sha256(
        json.dumps(
            [
                {"schema": value["schema"], **manifest}
                for manifest in value["manifests"]
            ],
            sort_keys=True,
            separators=(",", ":"),
        ).encode()
    ).hexdigest()


def test_provider_publication_is_immutable_and_keeps_query_semantics() -> None:
    occupied = {(0, 0, 2)}
    free = {(0, 0, 3)}
    columns = {(0, 0): [0.0, 1.0]}
    key = SnapshotKey("component", 1, 2, "a" * 64)
    grid = IndexedGrid(
        key,
        "b" * 64,
        100,
        10,
        occupied,
        free,
        columns,
        0.2,
        2,
    )
    view = IndexedMapView()
    view.publish(grid)

    occupied.clear()
    free.clear()
    columns[(0, 0)].clear()
    columns[(1, 1)] = [4.0]

    result = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            now_monotonic_ns=11,
            max_snapshot_age_ns=2,
        )
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.OCCUPIED,)
    assert tuple(grid.columns) == ((0, 0),)

    stale = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            source_stamp_ns=101,
        )
    )
    assert stale.status is QueryStatus.STALE

    expired = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            now_monotonic_ns=13,
            max_snapshot_age_ns=2,
        )
    )
    assert expired.status is QueryStatus.UNAVAILABLE


KEY_A = SnapshotKey("component", 1, 1, "a" * 64)
KEY_B = SnapshotKey("component", 1, 2, "b" * 64)
KEY_C = SnapshotKey("component", 1, 3, "c" * 64)
KEY_D = SnapshotKey("component", 1, 4, "d" * 64)
SAMPLE = ((0.1, 0.1, 0.5),)
BODY = (0.1, 0.1, 0.1)
GRACE_NS = 1_000


def superseded_grid(key, *, received, occupied=True, points=1, stamp=100):
    """One body voxel, occupied or free, so a result names the index it used."""

    voxels = {(0, 0, 2)}
    return IndexedGrid(
        key,
        key.geometry_revision,
        stamp,
        received,
        voxels if occupied else set(),
        set() if occupied else voxels,
        {},
        0.2,
        points,
    )


def superseded_view(**overrides):
    view = IndexedMapView(superseded_grace_ns=GRACE_NS, **overrides)
    view.publish(superseded_grid(KEY_A, received=100, occupied=True))
    return view


def query_at(view, key, now, **overrides):
    return view.query(
        QueryRequest(key, SAMPLE, BODY, now_monotonic_ns=now, **overrides)
    )


def test_superseded_key_keeps_answering_within_the_grace() -> None:
    view = superseded_view()
    before = query_at(view, KEY_A, 150)
    assert before.status is QueryStatus.OK
    assert before.occupancy == (VoxelOccupancy.OCCUPIED,)

    view.publish(superseded_grid(KEY_B, received=200, occupied=False))
    assert view.superseded_keys == (KEY_A,)

    # Exactly the grace after the replacing product's coherent read.
    retained = query_at(view, KEY_A, 200 + GRACE_NS)
    assert retained == before
    assert retained.key == KEY_A
    current = query_at(view, KEY_B, 200 + GRACE_NS)
    assert current.status is QueryStatus.OK
    assert current.key == KEY_B
    assert current.occupancy == (VoxelOccupancy.FREE,)

    # The coherent-read age still governs the current index only: a retained
    # index is no longer refreshed, its bound is the grace.
    aged_current = query_at(view, KEY_B, 200 + GRACE_NS, max_snapshot_age_ns=10)
    assert aged_current.status is QueryStatus.UNAVAILABLE
    assert aged_current.detail == "indexed snapshot exceeded coherent-read age"
    aged_retained = query_at(view, KEY_A, 200 + GRACE_NS, max_snapshot_age_ns=10)
    assert aged_retained == before

    # The source stamp and sample checks apply to a retained index as well.
    wrong_stamp = query_at(view, KEY_A, 300, source_stamp_ns=101)
    assert wrong_stamp.status is QueryStatus.STALE
    assert wrong_stamp.key == KEY_A
    assert wrong_stamp.detail == "source stamp does not match indexed snapshot"
    assert query_at(view, KEY_A, 300, source_stamp_ns=100) == before
    small_view = superseded_view(max_samples=1)
    small_view.publish(superseded_grid(KEY_B, received=200, occupied=False))
    too_many = small_view.query(
        QueryRequest(KEY_A, SAMPLE * 2, BODY, now_monotonic_ns=300)
    )
    assert too_many.status is QueryStatus.UNAVAILABLE
    assert too_many.detail == "sample budget exceeded"


def test_superseded_key_is_stale_once_the_grace_has_passed() -> None:
    view = superseded_view()
    view.publish(superseded_grid(KEY_B, received=200, occupied=False))

    expired = query_at(view, KEY_A, 201 + GRACE_NS)
    assert expired.status is QueryStatus.STALE
    assert expired.key == KEY_B
    assert expired.detail == "requested snapshot is not current"
    # A key never published is refused the same way as before.
    unknown = query_at(view, KEY_C, 300)
    assert unknown.status is QueryStatus.STALE
    assert unknown.detail == "requested snapshot is not current"


def test_superseded_history_is_bounded_by_count_and_grace() -> None:
    view = superseded_view(superseded_keep=2)
    view.publish(superseded_grid(KEY_B, received=200, occupied=False))
    view.publish(superseded_grid(KEY_C, received=300, occupied=True))
    assert view.superseded_keys == (KEY_A, KEY_B)

    view.publish(superseded_grid(KEY_D, received=400, occupied=False))
    assert view.superseded_keys == (KEY_B, KEY_C)
    gone = query_at(view, KEY_A, 450)
    assert gone.status is QueryStatus.STALE
    assert gone.detail == "requested snapshot is not current"
    assert query_at(view, KEY_B, 450).occupancy == (VoxelOccupancy.FREE,)
    assert query_at(view, KEY_C, 450).occupancy == (VoxelOccupancy.OCCUPIED,)

    # A publication also drops entries whose grace has already run out, so a
    # slowly publishing source does not hold dead grids for the count bound.
    view.publish(superseded_grid(KEY_A, received=400 + GRACE_NS + 1, occupied=True))
    assert view.superseded_keys == (KEY_D,)

    # A key republished as current drops its retained copy: keys stay unique.
    view.publish(superseded_grid(KEY_D, received=400 + GRACE_NS + 2, occupied=False))
    assert view.superseded_keys == (KEY_A,)

    # A same-key liveness refresh replaces nothing and retains nothing.
    view.publish(superseded_grid(KEY_D, received=400 + GRACE_NS + 3, occupied=False))
    assert view.superseded_keys == (KEY_A,)
    assert view.stats.rebuilds == 6

    disabled = IndexedMapView(superseded_grace_ns=GRACE_NS, superseded_keep=0)
    disabled.publish(superseded_grid(KEY_A, received=100, occupied=True))
    disabled.publish(superseded_grid(KEY_B, received=200, occupied=False))
    assert disabled.superseded_keys == ()
    assert query_at(disabled, KEY_A, 250).status is QueryStatus.STALE


@pytest.mark.parametrize(
    "field, value",
    [
        ("superseded_keep", -1),
        ("superseded_keep", True),
        ("superseded_grace_ns", -1),
        ("superseded_grace_ns", 1.5),
    ],
)
def test_superseded_bounds_are_validated(field, value) -> None:
    with pytest.raises(ValueError, match=field):
        IndexedMapView(**{field: value})


def test_invalidate_drops_the_superseded_history() -> None:
    view = superseded_view()
    view.publish(superseded_grid(KEY_B, received=200, occupied=False))
    assert query_at(view, KEY_A, 250).status is QueryStatus.OK

    view.invalidate("snapshot changed during index build")
    assert view.superseded_keys == ()
    refused = query_at(view, KEY_A, 250)
    assert refused.status is QueryStatus.STALE
    assert refused.detail == "a different snapshot failed indexed publication"
    current = query_at(view, KEY_B, 250)
    assert current.status is QueryStatus.UNAVAILABLE
    assert current.detail == "snapshot changed during index build"

    # The next good publication starts a fresh history; the index that was
    # refused meanwhile is not retained because it stopped being served at a
    # moment this view cannot date.
    view.publish(superseded_grid(KEY_C, received=300, occupied=True))
    assert view.superseded_keys == ()
    assert query_at(view, KEY_B, 350).status is QueryStatus.STALE


def test_failed_newer_build_does_not_shadow_a_retained_key() -> None:
    view = superseded_view()
    view.publish(superseded_grid(KEY_B, received=200, occupied=False))
    with pytest.raises(ValueError, match="point budget"):
        view.publish(superseded_grid(KEY_C, received=300, points=10**9))
    assert view._unavailable_key == KEY_C

    retained = query_at(view, KEY_A, 200 + GRACE_NS)
    assert retained.status is QueryStatus.OK
    assert retained.key == KEY_A
    assert retained.occupancy == (VoxelOccupancy.OCCUPIED,)
    failed = query_at(view, KEY_C, 300)
    assert failed.status is QueryStatus.UNAVAILABLE
    assert failed.key == KEY_C
    assert failed.detail == "index point budget exceeded"
    # The existing fail-closed answers stand for everything else: the current
    # index while a newer build has failed, and the retained key once its
    # grace has passed.
    current = query_at(view, KEY_B, 300)
    assert current.status is QueryStatus.STALE
    assert current.key == KEY_C
    assert current.detail == "a different snapshot failed indexed publication"
    expired = query_at(view, KEY_A, 201 + GRACE_NS)
    assert expired.status is QueryStatus.STALE
    assert expired.detail == "a different snapshot failed indexed publication"

    # A failed republication of a retained key refuses that key: the failure
    # is not hidden behind the older copy.
    with pytest.raises(ValueError, match="point budget"):
        view.publish(superseded_grid(KEY_A, received=310, points=10**9))
    refused = query_at(view, KEY_A, 320)
    assert refused.status is QueryStatus.UNAVAILABLE
    assert refused.detail == "index point budget exceeded"

    # Recovery: the failed key lands. The index refused meanwhile is not
    # retained; the older retained key keeps its own grace.
    view.publish(superseded_grid(KEY_C, received=400, occupied=True))
    assert view.superseded_keys == (KEY_A,)
    assert query_at(view, KEY_C, 450).status is QueryStatus.OK
    assert query_at(view, KEY_A, 450).status is QueryStatus.OK
    assert query_at(view, KEY_B, 450).detail == "requested snapshot is not current"


def test_unqualified_sensor_origin_does_not_claim_free_space(tmp_path) -> None:
    store = SubmapStore(tmp_path)
    keyframe = KeyframeId("r0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=((-0.1, 0.1, 0.5),),
        resolution_m=0.2,
        observed_at_ns=100,
    )
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    key = view.refresh(snapshot, component, store.get_chunk)

    result = view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.UNKNOWN,)


def test_chunks_load_once_and_pose_revision_rebuilds_once(tmp_path) -> None:
    # A flat support patch plus one return whose ray observes the path center.
    points = [[3.1, 0.1, 0.5]] + [
        [x, y, 0.0] for x in (-0.1, 0.1, 0.3) for y in (-0.1, 0.1, 0.3)
    ]
    store, keyframe = make_store(tmp_path, points)
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView(max_build_s=2)
    first = store.snapshot()
    key0 = view.refresh(first, component, store.get_chunk, received_monotonic_ns=10)
    assert view.stats.chunk_loads == 1
    assert view.stats.rebuilds == 1
    view.refresh(first, component, store.get_chunk, received_monotonic_ns=11)
    assert view.stats.chunk_loads == 1
    assert view.stats.rebuilds == 1

    solution = GraphSolution(
        ComponentRevision(component, 0, 1),
        keyframe,
        (keyframe,),
        {keyframe: pose(1.0)},
    )
    store.apply_solution(solution)
    second = store.snapshot()
    key1 = view.refresh(second, component, store.get_chunk, received_monotonic_ns=20)
    assert key1 != key0
    assert view.stats.chunk_loads == 1
    assert view.stats.rebuilds == 2
    # The replaced key keeps answering for the default 15 s grace, measured
    # from the coherent-read time of the product that replaced it.
    grace_ns = 15_000_000_000
    within = view.query(
        QueryRequest(
            key0, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1), now_monotonic_ns=20 + grace_ns
        )
    )
    assert within.status is QueryStatus.OK
    assert within.key == key0
    expired = view.query(
        QueryRequest(
            key0,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            now_monotonic_ns=21 + grace_ns,
        )
    )
    assert expired.status is QueryStatus.STALE
    assert expired.detail == "requested snapshot is not current"


def test_exact_source_stamp_and_monotonic_age_fail_closed(tmp_path) -> None:
    store, keyframe = make_store(tmp_path, [[2.1, 0.1, 0.5]])
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    key = view.refresh(snapshot, component, store.get_chunk, received_monotonic_ns=100)
    stale_stamp = view.query(
        QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1), source_stamp_ns=101)
    )
    assert stale_stamp.status is QueryStatus.STALE
    aged = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            source_stamp_ns=100,
            now_monotonic_ns=201,
            max_snapshot_age_ns=100,
        )
    )
    assert aged.status is QueryStatus.UNAVAILABLE


def test_coherent_reread_refreshes_monotonic_ttl_without_rebuild(tmp_path) -> None:
    store, keyframe = make_store(tmp_path, [[2.1, 0.1, 0.5]])
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    key = view.refresh(snapshot, component, store.get_chunk, received_monotonic_ns=100)
    request = QueryRequest(
        key,
        ((0.1, 0.1, 0.5),),
        (0.1, 0.1, 0.1),
        now_monotonic_ns=250,
        max_snapshot_age_ns=100,
    )
    assert view.query(request).status is QueryStatus.UNAVAILABLE
    view.refresh(snapshot, component, store.get_chunk, received_monotonic_ns=200)
    assert view.query(request).status is QueryStatus.OK
    assert view.stats.rebuilds == 1
    assert view.stats.chunk_loads == 1


def test_same_key_changed_content_and_false_geometry_digest_are_rejected(
    tmp_path,
) -> None:
    store, keyframe = make_store(tmp_path, [[2.1, 0.1, 0.5]])
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    key = view.refresh(snapshot, component, store.get_chunk)

    changed_pose = copy.deepcopy(snapshot.to_dict())
    changed_pose["manifests"][0]["submaps"][0]["T_component_submap"][0][3] = 1.0
    repair_snapshot_id(changed_pose)
    with pytest.raises(ValueError, match="reused with different map content"):
        view.refresh(changed_pose, component, store.get_chunk)
    assert (
        view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))).status
        is QueryStatus.UNAVAILABLE
    )

    # Restore the valid source, then prove an internally inconsistent geometry
    # key is rejected even if its outer snapshot hash is recomputed.
    view.refresh(snapshot, component, store.get_chunk)
    false_geometry = copy.deepcopy(snapshot.to_dict())
    false_geometry["manifests"][0]["geometry_revision"] = "f" * 64
    repair_snapshot_id(false_geometry)
    with pytest.raises(ValueError, match="geometry_revision"):
        view.refresh(false_geometry, component, store.get_chunk)


def test_bounded_failed_refresh_makes_requested_revision_unavailable(tmp_path) -> None:
    store, keyframe = make_store(tmp_path, [[10.1, 0.1, 0.5]])
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView(max_ray_steps=2)
    key = snapshot_key(snapshot, component)
    with pytest.raises(ValueError, match="ray budget"):
        view.refresh(snapshot, component, store.get_chunk)
    result = view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    assert result.status is QueryStatus.UNAVAILABLE


def test_batched_wall_and_unknown_are_conservative(tmp_path) -> None:
    floor = [
        [x, y, 0.0]
        for x in (-0.1, 0.1, 0.3, 0.5, 0.7, 0.9, 1.1)
        for y in (-0.1, 0.1, 0.3)
    ]
    points = [[3.1, 0.1, 0.5], [0.7, 0.1, 0.5]] + floor
    store, keyframe = make_store(tmp_path, points)
    component = component_id_for_anchor(keyframe)
    snapshot = store.snapshot()
    view = IndexedMapView()
    key = view.refresh(snapshot, component, store.get_chunk)
    result = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5), (0.7, 0.1, 0.5)),
            (0.1, 0.1, 0.1),
            source_stamp_ns=100,
        )
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy[0] == VoxelOccupancy.FREE
    assert result.occupancy[1] == VoxelOccupancy.OCCUPIED

    unknown_store, unknown_keyframe = make_store(tmp_path / "unknown", floor, origin=())
    unknown_component = component_id_for_anchor(unknown_keyframe)
    unknown_snapshot = unknown_store.snapshot()
    unknown_view = IndexedMapView()
    unknown_key = unknown_view.refresh(
        unknown_snapshot, unknown_component, unknown_store.get_chunk
    )
    unknown = unknown_view.query(
        QueryRequest(unknown_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    )
    assert unknown.occupancy == (VoxelOccupancy.UNKNOWN,)
    assert math.isnan(unknown.clearance[0])


def test_gentle_ramp_passes_and_step_is_flagged(tmp_path) -> None:
    floor = [
        [x, y, 0.05 * x]
        for x in (-0.1, 0.1, 0.3, 0.5, 0.7, 0.9, 1.1)
        for y in (-0.1, 0.1, 0.3)
    ]
    store, keyframe = make_store(tmp_path, [[3.1, 0.1, 0.5]] + floor)
    snapshot = store.snapshot()
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView(max_step_m=0.2)
    key = view.refresh(snapshot, component, store.get_chunk)
    result = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5), (0.5, 0.1, 0.5), (0.9, 0.1, 0.5)),
            (0.1, 0.1, 0.1),
        )
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.FREE,) * 3
    assert not any(result.step)
    assert not any(result.drop)
    assert all(math.isfinite(value) for value in result.ground_z)
    assert all(math.isfinite(value) for value in result.clearance)

    stepped = [
        [x, y, 0.0 if x < 0.5 else 0.4]
        for x in (-0.1, 0.1, 0.3, 0.7, 0.9, 1.1)
        for y in (-0.1, 0.1, 0.3)
    ]
    step_store, step_keyframe = make_store(
        tmp_path / "step", [[3.1, 0.1, 0.5]] + stepped
    )
    step_snapshot = step_store.snapshot()
    step_component = component_id_for_anchor(step_keyframe)
    step_view = IndexedMapView(max_step_m=0.2)
    step_key = step_view.refresh(step_snapshot, step_component, step_store.get_chunk)
    step_result = step_view.query(
        QueryRequest(
            step_key,
            ((0.1, 0.1, 0.5), (0.9, 0.1, 0.9)),
            (0.1, 0.1, 0.1),
        )
    )
    assert any(step_result.step)


def test_query_uses_platform_step_and_drop_limits() -> None:
    key = SnapshotKey("platform", 0, 0, "a" * 64)
    columns = {
        (x, y): (0.0 if x < 3 else 0.2,)
        for x in (-1, 0, 1, 4, 5, 6)
        for y in (-1, 0, 1)
    }
    view = IndexedMapView(max_step_m=0.2, max_drop_m=0.2)
    view.publish(IndexedGrid(key, "b" * 64, 1, 1, set(), set(), columns, 0.2, 18))

    samples = ((0.1, 0.1, 0.5), (1.1, 0.1, 0.7))
    tracked = view.query(
        QueryRequest(
            key,
            samples,
            (0.1, 0.1, 0.1),
            stop_at_unknown=False,
            max_step_m=0.15,
            max_drop_m=0.15,
        )
    )
    spot = view.query(
        QueryRequest(
            key,
            samples,
            (0.1, 0.1, 0.1),
            stop_at_unknown=False,
            max_step_m=0.30,
            max_drop_m=0.30,
        )
    )
    tracked_reverse = view.query(
        QueryRequest(
            key,
            tuple(reversed(samples)),
            (0.1, 0.1, 0.1),
            stop_at_unknown=False,
            max_step_m=0.15,
            max_drop_m=0.15,
        )
    )

    assert tracked.step == (False, True)
    assert spot.step == (False, False)
    assert tracked_reverse.drop == (False, True)


def test_roughness_is_reported_separately_from_geometric_steps() -> None:
    key = SnapshotKey("roughness", 0, 0, "a" * 64)

    def query(amplitude: float):
        columns = {
            (-1, -1): (-amplitude,),
            (-1, 0): (amplitude,),
            (0, -1): (amplitude,),
            (0, 0): (-amplitude,),
        }
        view = IndexedMapView(max_roughness_m=0.08)
        view.publish(IndexedGrid(key, "b" * 64, 1, 1, set(), set(), columns, 0.2, 4))
        return view.query(
            QueryRequest(
                key,
                ((0.0, 0.0, 0.5),),
                (0.1, 0.1, 0.1),
                stop_at_unknown=False,
                max_step_m=0.15,
                max_drop_m=0.15,
            )
        )

    within_consumer_limit = query(0.09)
    assert within_consumer_limit.roughness == pytest.approx((0.09,))
    assert within_consumer_limit.step == (False,)
    assert within_consumer_limit.roughness[0] <= 0.10

    too_rough_for_consumer = query(0.12)
    assert too_rough_for_consumer.roughness == pytest.approx((0.12,))
    assert too_rough_for_consumer.step == (False,)
    assert too_rough_for_consumer.roughness[0] > 0.10


def test_unknown_terrain_resets_geometric_step_comparison() -> None:
    key = SnapshotKey("unknown-gap", 0, 0, "a" * 64)
    columns = {
        **{(x, y): (0.0,) for x in (-1, 0, 1) for y in (-1, 0, 1)},
        **{(x, y): (0.4,) for x in (19, 20, 21) for y in (-1, 0, 1)},
    }
    view = IndexedMapView()
    view.publish(IndexedGrid(key, "b" * 64, 1, 1, set(), set(), columns, 0.2, 18))

    result = view.query(
        QueryRequest(
            key,
            ((0.0, 0.0, 0.5), (2.0, 0.0, 0.5), (4.0, 0.0, 0.9)),
            (0.1, 0.1, 0.1),
            stop_at_unknown=False,
            max_step_m=0.15,
            max_drop_m=0.15,
        )
    )

    assert math.isnan(result.ground_z[1])
    assert result.ground_z[0] == pytest.approx(0.0)
    assert result.ground_z[2] == pytest.approx(0.4)
    assert result.step == (False, False, False)
    assert result.drop == (False, False, False)


@pytest.mark.parametrize("value", (0.0, -0.1, math.inf, math.nan, True))
def test_query_rejects_invalid_platform_terrain_limits(value) -> None:
    key = SnapshotKey("platform", 0, 0, "a" * 64)
    with pytest.raises(ValueError, match="max_step_m"):
        QueryRequest(
            key,
            ((0.0, 0.0, 0.0),),
            (0.1, 0.1, 0.1),
            max_step_m=value,
        )


def test_matter_within_the_terrain_band_is_relief_not_a_collision(tmp_path) -> None:
    # A road at z = 0 with a second floor 0.25 m above it under the body: the
    # layer a LiDAR-inertial map lays where keyframe heights disagree, or a
    # kerb lip. The body (0.3 by 0.3 by 0.6 m) stands at z = 0.4, so its box
    # spans 0.1 to 0.7 m and that floor's voxel (0.2 to 0.4 m) is above the
    # support layer. A platform that climbs 0.3 m drives over it, so does one
    # that climbs 0.15 m but may drop 0.3 m (the band is the larger limit);
    # one limited to 0.15 m both ways collides with it. A wall return at
    # 0.5 m is a collision for all. The body's upper voxels are not ray-proven free, so the
    # climber's first sample is unknown (a horizon), never occupied (a wall).
    floor = [
        [x, y, 0.0]
        for x in (-0.3, -0.1, 0.1, 0.3, 0.5, 0.7, 0.9, 1.1)
        for y in (-0.3, -0.1, 0.1, 0.3, 0.5)
    ]
    second_floor = [[x, y, 0.25] for x in (0.1, 0.3) for y in (0.1, 0.3)]
    points = floor + second_floor + [[0.9, 0.1, 0.5]]
    store, keyframe = make_store(tmp_path, points)
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    key = view.refresh(store.snapshot(), component, store.get_chunk)
    samples = ((0.1, 0.1, 0.4), (0.9, 0.1, 0.4))

    climber = view.query(
        QueryRequest(
            key,
            samples,
            (0.3, 0.3, 0.6),
            max_step_m=0.3,
            source_stamp_ns=100,
            stop_at_unknown=False,
        )
    )
    assert climber.status is QueryStatus.OK
    assert climber.occupancy[0] != VoxelOccupancy.OCCUPIED
    assert climber.occupancy[1] == VoxelOccupancy.OCCUPIED
    assert climber.ground_z[0] == pytest.approx(0.0, abs=0.05)

    dropper = view.query(
        QueryRequest(
            key,
            samples,
            (0.3, 0.3, 0.6),
            max_step_m=0.15,
            max_drop_m=0.3,
            source_stamp_ns=100,
            stop_at_unknown=False,
        )
    )
    assert dropper.occupancy[0] != VoxelOccupancy.OCCUPIED
    assert dropper.occupancy[1] == VoxelOccupancy.OCCUPIED

    stepper = view.query(
        QueryRequest(
            key,
            samples,
            (0.3, 0.3, 0.6),
            max_step_m=0.15,
            max_drop_m=0.15,
            source_stamp_ns=100,
            stop_at_unknown=False,
        )
    )
    assert stepper.occupancy[0] == VoxelOccupancy.OCCUPIED
    assert stepper.occupancy[1] == VoxelOccupancy.OCCUPIED


def test_stacked_surfaces_select_support_beneath_each_robot(tmp_path):
    levels = [
        [x, y, z]
        for z in (0.0, 2.0, 4.0)
        for x in (-0.3, -0.1, 0.1, 0.3, 0.5)
        for y in (-0.3, -0.1, 0.1, 0.3, 0.5)
    ]
    store, anchor = make_store(tmp_path, levels, origin=())
    view = IndexedMapView()
    key = view.refresh(
        store.snapshot(), component_id_for_anchor(anchor), store.get_chunk
    )
    result = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5), (0.1, 0.1, 2.5)),
            (0.2, 0.2, 0.6),
            stop_at_unknown=False,
        )
    )
    assert result.status == QueryStatus.OK
    assert result.ground_z == pytest.approx((0.0, 2.0))
    assert result.clearance == pytest.approx((2.0, 2.0))
    # Occupied returns alone cannot prove a clear swept body volume.
    assert result.occupancy == (VoxelOccupancy.UNKNOWN,) * 2


def test_down_step_and_missing_support_are_distinct(tmp_path):
    floor = [
        [x, y, z]
        for center, z in ((0.1, 0.4), (1.7, -0.4))
        for x in (center - 0.2, center, center + 0.2)
        for y in (-0.1, 0.1, 0.3)
    ]
    store, anchor = make_store(tmp_path, floor, origin=())
    view = IndexedMapView(max_drop_m=0.2)
    key = view.refresh(
        store.snapshot(), component_id_for_anchor(anchor), store.get_chunk
    )
    result = view.query(
        QueryRequest(
            key,
            ((0.1, 0.1, 0.9), (1.7, 0.1, 0.1), (5.0, 0.1, 0.1)),
            (0.2, 0.2, 0.4),
            stop_at_unknown=False,
        )
    )
    assert result.drop == (False, True, False)
    assert math.isnan(result.ground_z[2])
    assert result.occupancy[2] == VoxelOccupancy.UNKNOWN


@pytest.mark.parametrize(
    "body, sample",
    [
        ((1000, 1000, 1000), (0, 0, 0)),
        ((1e8, 1e-10, 1e-10), (0, 0, 0)),
        ((1e308, 1e308, 1e308), (0, 0, 0)),
        ((1, 1, 1), (1e308, 0, 0)),
    ],
)
def test_query_work_is_bounded_before_allocating_body_cells(tmp_path, body, sample):
    store, anchor = make_store(tmp_path, [[0.1, 0.1, 0.0]], origin=())
    view = IndexedMapView(max_query_voxels=1000)
    key = view.refresh(
        store.snapshot(), component_id_for_anchor(anchor), store.get_chunk
    )
    result = view.query(QueryRequest(key, (sample,), body))
    assert result.status == QueryStatus.UNAVAILABLE
    assert not result.occupancy


@pytest.mark.parametrize("platform", ("bunker", "scout_mini", "spot"))
def test_production_vlp16_preserves_unobserved_ground_at_the_physical_root(
    tmp_path, monkeypatch, platform
) -> None:
    """A real flat floor must stay unknown where neither sensor can observe it."""

    repository = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(repository / "adapters" / "protocol"))
    monkeypatch.syspath_prepend(
        str(repository / "swarmdeck_ros/src/swarmdeck_sim/scenario")
    )
    from make_argos_session import CAMERA_FOV_DEG
    from spawn_fleet import LIDAR_PROFILES, ROBOT_PROFILES

    robot = ROBOT_PROFILES[platform]
    lidar = LIDAR_PROFILES["vlp16"]
    sensor_origin = np.asarray((robot.lidar_x, 0.0, robot.lidar_z))
    sensor_height = robot.base_height + robot.lidar_z
    returns = []
    for elevation in np.linspace(-lidar.vfov, lidar.vfov, lidar.rings):
        if elevation >= 0.0:
            continue
        azimuths = np.linspace(-math.pi, math.pi, lidar.h_samples)
        directions = np.column_stack(
            (
                math.cos(elevation) * np.cos(azimuths),
                math.cos(elevation) * np.sin(azimuths),
                np.full_like(azimuths, math.sin(elevation)),
            )
        )
        ranges = sensor_height / -directions[:, 2]
        # The MOLA peer filters the production VLP-16 at 30 m before applying
        # its 4096-endpoint storage cap.
        in_range = ranges <= min(lidar.range_max, 30.0)
        returns.append(sensor_origin + ranges[in_range, None] * directions[in_range])
    points = endpoint_preserving_sample(np.concatenate(returns), 4096)
    assert len(points) == 4096

    store = SubmapStore(tmp_path)
    keyframe = KeyframeId(platform, SESSION, 0)
    record_qualified_capture(store, keyframe, sensor_origin, observed_at_ns=100)
    T_component_base = np.eye(4)
    T_component_base[2, 3] = robot.base_height
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        points,
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=(tuple(sensor_origin),),
        resolution_m=0.2,
        observed_at_ns=100,
        initial_T_component_submap=T_component_base,
        ray_evidence=QUALIFIED_RAYS,
    )

    view = IndexedMapView(max_build_s=5.0)
    component = component_id_for_anchor(keyframe)
    snapshot_key = view.refresh(store.snapshot(), component, store.get_chunk)
    body = (robot.length, robot.width, 2.0 * robot.base_height)
    graph_z = robot.base_height + robot.max_step_height + 0.175
    lidar_floor_x = robot.lidar_x + sensor_height / math.tan(lidar.vfov)
    result = view.query(
        QueryRequest(
            snapshot_key,
            ((0.0, 0.0, graph_z), (lidar_floor_x, 0.0, graph_z)),
            body,
            stop_at_unknown=False,
        )
    )

    support_radius = max(0.35, robot.length / 2.0, robot.width / 2.0)
    camera_floor_x = robot.camera_x + (
        (robot.base_height + robot.camera_z)
        / math.tan(math.radians(CAMERA_FOV_DEG) / 2.0)
    )
    assert min(camera_floor_x, lidar_floor_x) > support_radius
    assert result.status is QueryStatus.OK
    assert math.isnan(result.ground_z[0])
    assert math.isnan(result.roughness[0])
    assert math.isclose(result.ground_z[1], 0.0, abs_tol=0.11)
    assert math.isfinite(result.roughness[1])


def test_vlp16_history_accepts_body_corridor_and_rejects_wall_and_unknown(
    tmp_path,
) -> None:
    """Exercise the exact body-volume rule with sparse, physical lidar rays.

    The scanner is 0.72 m above a flat floor on the simulated Bunker. Nine
    captures along its travelled path contain the VLP-16 vertical rings over a
    forward 180-degree field and a wall at x=4 m. This is intentionally not a
    filled free-space cloud: only the cells crossed by actual returns count.
    """

    store = SubmapStore(tmp_path)
    sensor_origin_base = np.asarray((-0.15, 0.0, 0.52))
    wall_x = 4.0
    vertical_angles = np.deg2rad(np.arange(-15, 16, 2))
    azimuth_angles = np.deg2rad(np.arange(-90, 91, 5))
    for seq, base_x in enumerate(np.arange(-4.0, 0.01, 0.5)):
        base_component = np.asarray((base_x, 0.0, 0.2))
        origin_component = base_component + sensor_origin_base
        returns_base = []
        for elevation in vertical_angles:
            for azimuth in azimuth_angles:
                direction = np.asarray(
                    (
                        math.cos(elevation) * math.cos(azimuth),
                        math.cos(elevation) * math.sin(azimuth),
                        math.sin(elevation),
                    )
                )
                ranges = []
                if direction[2] < -1e-9:
                    ranges.append(origin_component[2] / -direction[2])
                if direction[0] > 1e-9:
                    ranges.append((wall_x - origin_component[0]) / direction[0])
                ranges = [distance for distance in ranges if 0.05 < distance <= 20.0]
                if ranges:
                    endpoint = origin_component + min(ranges) * direction
                    returns_base.append(endpoint - base_component)
        T_component_base = np.eye(4)
        T_component_base[:3, 3] = base_component
        keyframe = KeyframeId("r0", SESSION, seq)
        record_qualified_capture(
            store, keyframe, sensor_origin_base, observed_at_ns=100 + seq
        )
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            returns_base,
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=(tuple(sensor_origin_base),),
            resolution_m=0.1,
            observed_at_ns=100 + seq,
            initial_T_component_submap=T_component_base,
            ray_evidence=QUALIFIED_RAYS,
        )

    snapshot = store.snapshot()
    component = component_id_for_anchor(KeyframeId("r0", SESSION, 0))
    view = IndexedMapView(max_build_s=5.0)
    key = view.refresh(snapshot, component, store.get_chunk)
    bunker_body = (1.023, 0.778, 0.4)
    traversable = tuple((float(x), 0.0, 0.2) for x in np.arange(0.0, 1.61, 0.2))
    result = view.query(
        QueryRequest(key, traversable, bunker_body, stop_at_unknown=False)
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.FREE,) * len(traversable)
    assert all(math.isfinite(value) for value in result.ground_z)
    assert all(
        math.isfinite(value) and value >= bunker_body[2] for value in result.clearance
    )
    assert not any(result.step)
    assert not any(result.drop)

    unseen = view.query(QueryRequest(key, ((2.4, 0.0, 0.2),), bunker_body))
    assert unseen.occupancy == (VoxelOccupancy.UNKNOWN,)
    assert math.isnan(unseen.clearance[0])
    wall = view.query(QueryRequest(key, ((3.8, 0.0, 0.2),), bunker_body))
    assert wall.occupancy == (VoxelOccupancy.OCCUPIED,)


def test_geometry_replacement_removes_old_index_contribution(tmp_path) -> None:
    store, keyframe = make_store(tmp_path, [[0.1, 0.1, 0.5]], origin=())
    component = component_id_for_anchor(keyframe)
    view = IndexedMapView()
    first = store.snapshot()
    key0 = view.refresh(first, component, store.get_chunk)
    occupied = view.query(QueryRequest(key0, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    assert occupied.occupancy == (VoxelOccupancy.OCCUPIED,)

    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=(),
        resolution_m=0.2,
        observed_at_ns=200,
        replace=True,
    )
    second = store.snapshot()
    key1 = view.refresh(second, component, store.get_chunk)
    old_location = view.query(QueryRequest(key1, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    new_location = view.query(QueryRequest(key1, ((2.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    assert old_location.occupancy == (VoxelOccupancy.UNKNOWN,)
    assert new_location.occupancy == (VoxelOccupancy.OCCUPIED,)


def test_keyframe_retraction_removes_indexed_submap(tmp_path) -> None:
    store = SubmapStore(tmp_path)
    first = KeyframeId("r0", SESSION, 0)
    second = KeyframeId("r0", SESSION, 1)
    for keyframe, x in ((first, 0.1), (second, 2.1)):
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            [[x, 0.1, 0.5]],
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=(),
            resolution_m=0.2,
            observed_at_ns=100 + keyframe.seq,
        )
    component = component_id_for_anchor(first)
    view = IndexedMapView()
    initial = store.snapshot()
    initial_key = view.refresh(initial, component, store.get_chunk)
    assert view.query(
        QueryRequest(initial_key, ((2.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).occupancy == (VoxelOccupancy.OCCUPIED,)

    store.apply_solution(
        GraphSolution(
            ComponentRevision(component, 0, 1),
            first,
            (first,),
            {first: IDENTITY_SE3},
            retracted_keyframes=(second,),
        )
    )
    retracted = store.snapshot()
    retracted_key = view.refresh(retracted, component, store.get_chunk)
    assert view.query(
        QueryRequest(retracted_key, ((2.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).occupancy == (VoxelOccupancy.UNKNOWN,)
