from __future__ import annotations

import copy
import hashlib
import json
import math
import uuid

import numpy as np
import pytest

from autonomy.contracts import (
    IDENTITY_SE3,
    ComponentRevision,
    GraphSolution,
    KeyframeId,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.indexed_mapping import (
    IndexedMapView,
    QueryRequest,
    QueryStatus,
    SnapshotKey,
    VoxelOccupancy,
)
from autonomy.mapping import SubmapStore

SESSION = str(uuid.UUID("8c095aed-b093-43a4-997f-495b2a8b734c"))


def pose(x: float = 0.0):
    result = [list(row) for row in IDENTITY_SE3]
    result[0][3] = x
    return tuple(tuple(row) for row in result)


def make_store(tmp_path, points, *, origin=((-0.1, 0.1, 0.5),)):
    store = SubmapStore(tmp_path)
    keyframe = KeyframeId("r0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        points,
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=origin,
        resolution_m=0.2,
        observed_at_ns=100,
    )
    return store, keyframe


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
    assert (
        view.query(QueryRequest(key0, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))).status
        is QueryStatus.STALE
    )


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
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            returns_base,
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=(tuple(sensor_origin_base),),
            resolution_m=0.1,
            observed_at_ns=100 + seq,
            initial_T_component_submap=T_component_base,
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
