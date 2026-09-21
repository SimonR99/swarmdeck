from __future__ import annotations

import json
import os
import uuid

import pytest

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
    SnapshotDirectorySource,
    SnapshotKey,
    VoxelOccupancy,
)
from autonomy.mapping import SubmapStore
from autonomy.map_epochs import claim_map_epoch, robot_run_id, write_peer_epochs
from autonomy.map_epochs import read_map_epoch
from deploy.autonomy.indexed_map_server import IndexRegistry, parse_args

SESSION = str(uuid.UUID("caa7c022-0c69-43ef-bf06-1922b16b9f5e"))
QUALIFIED_RAYS = RayEvidence(
    RayReturnSemantics.FIRST_RETURN,
    DeskewStatus.DESKEWED,
    RayOriginAssociation.SINGLE_CAPTURE,
)


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
    store.record_capture(
        CalibratedCapture(
            keyframe,
            observed_at_ns,
            observed_at_ns,
            sensor_frame,
            calibration.version,
            IDENTITY_SE3,
            None,
            DeskewStatus.DESKEWED,
            ray_return_semantics=RayReturnSemantics.FIRST_RETURN,
        ),
        calibration,
    )


@pytest.mark.parametrize(
    "mission_id",
    ["not-a-uuid", SESSION.upper(), "{" + SESSION + "}"],
)
def test_registry_requires_canonical_mission_uuid(tmp_path, mission_id) -> None:
    with pytest.raises(ValueError, match="mission_id"):
        IndexRegistry(tmp_path, mission_id, snapshot_age_s=3, poll_s=0.1)


def test_registry_refreshes_through_selected_provider(tmp_path) -> None:
    peer = tmp_path / SESSION / "robot_0"
    peer.mkdir(parents=True)
    publication = peer / "provider.index"
    publication.write_text("ready")
    key = SnapshotKey("provider-component", 1, 2, "a" * 64)
    instances = []

    class Provider:
        def __init__(self, peer_root):
            self.peer_root = peer_root
            self.publication_path = peer_root / "provider.index"
            self.refreshes = 0
            instances.append(self)

        def component_ids(self):
            return (key.component_id,)

        def signature(self):
            stat = self.publication_path.stat()
            return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)

        def refresh(self, view, component_id):
            assert component_id == key.component_id
            self.refreshes += 1
            return view.publish(
                IndexedGrid(
                    key,
                    "b" * 64,
                    123,
                    self.refreshes,
                    {(0, 0, 2)},
                    set(),
                    {(0, 0): (0.5,)},
                    0.2,
                    1,
                )
            )

    registry = IndexRegistry(
        tmp_path,
        SESSION,
        snapshot_age_s=3,
        poll_s=0.1,
        provider_factory=Provider,
    )
    registry.refresh_once()
    registry.refresh_once()

    assert len(instances) == 1
    assert instances[0].refreshes == 2
    result = registry.query(
        "robot_0", QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.OCCUPIED,)


def test_registry_serves_persisted_snapshot_without_ros(tmp_path) -> None:
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    record_qualified_capture(store, keyframe, (-0.1, 0.1, 0.5), observed_at_ns=123)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=((-0.1, 0.1, 0.5),),
        resolution_m=0.2,
        observed_at_ns=123,
        ray_evidence=QUALIFIED_RAYS,
    )
    snapshot = store.snapshot()
    (peer / "snapshot.json").write_text(json.dumps(snapshot.to_dict()))
    component = component_id_for_anchor(keyframe)
    manifest = snapshot.manifests[0]
    key = SnapshotKey(component, 0, 0, manifest.geometry_revision)

    registry = IndexRegistry(tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1)
    registry.refresh_once()
    assert registry.robots() == ("robot_0",)
    result = registry.query(
        "robot_0",
        QueryRequest(
            key,
            ((0.1, 0.1, 0.5),),
            (0.1, 0.1, 0.1),
            source_stamp_ns=123,
        ),
    )
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.FREE,)


def test_registry_ignores_same_robot_from_historical_missions(tmp_path) -> None:
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=((-0.1, 0.1, 0.5),),
        resolution_m=0.2,
        observed_at_ns=123,
    )
    snapshot = store.snapshot()
    (peer / "snapshot.json").write_text(json.dumps(snapshot.to_dict()))
    historical = tmp_path / str(uuid.uuid4()) / "robot_0"
    historical.mkdir(parents=True)
    (historical / "snapshot.json").write_text('{"schema":"bad","manifests":[]}')
    registry = IndexRegistry(tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1)
    registry.refresh_once()
    key = SnapshotKey(
        component_id_for_anchor(keyframe),
        0,
        0,
        snapshot.manifests[0].geometry_revision,
    )
    result = registry.query(
        "robot_0", QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    )
    assert result.status is QueryStatus.OK


def test_registry_invalidates_previous_index_when_snapshot_corrupts(tmp_path) -> None:
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=((-0.1, 0.1, 0.5),),
        resolution_m=0.2,
        observed_at_ns=123,
    )
    snapshot = store.snapshot()
    (peer / "snapshot.json").write_text(json.dumps(snapshot.to_dict()))
    component = component_id_for_anchor(keyframe)
    key = SnapshotKey(component, 0, 0, snapshot.manifests[0].geometry_revision)
    registry = IndexRegistry(tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1)
    registry.refresh_once()
    request = QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    assert registry.query("robot_0", request).status is QueryStatus.OK

    (peer / "snapshot.json").write_text('{"schema":NaN,"manifests":[]}')
    registry.refresh_once()
    assert registry.query("robot_0", request).status is QueryStatus.UNAVAILABLE


def test_registry_reports_failed_new_revision_as_unavailable(
    tmp_path, monkeypatch
) -> None:
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[2.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=((-0.1, 0.1, 0.5),),
        resolution_m=0.2,
        observed_at_ns=123,
    )
    first = store.snapshot()
    (peer / "snapshot.json").write_text(json.dumps(first.to_dict()))
    component = component_id_for_anchor(keyframe)
    old_key = SnapshotKey(component, 0, 0, first.manifests[0].geometry_revision)
    registry = IndexRegistry(tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1)
    registry.refresh_once()
    assert (
        registry.query(
            "robot_0", QueryRequest(old_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        ).status
        is QueryStatus.OK
    )

    store.apply_solution(
        GraphSolution(
            ComponentRevision(component, 0, 1),
            keyframe,
            (keyframe,),
            {keyframe: IDENTITY_SE3},
        )
    )
    second = store.snapshot()
    (peer / "snapshot.json").write_text(json.dumps(second.to_dict()))
    new_key = SnapshotKey(component, 0, 1, second.manifests[0].geometry_revision)
    original_build = IndexedMapView._build

    def fail_new_revision(self, key, *args, **kwargs):
        if key == new_key:
            raise ValueError("index build time budget exceeded")
        return original_build(self, key, *args, **kwargs)

    monkeypatch.setattr(IndexedMapView, "_build", fail_new_revision)
    registry.refresh_once()
    request = QueryRequest(new_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    assert registry.query("robot_0", request).status is QueryStatus.UNAVAILABLE
    assert (
        registry.query(
            "robot_0",
            QueryRequest(old_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)),
        ).status
        is QueryStatus.STALE
    )


def test_registry_backoff_is_per_source_and_resets_on_snapshot_replacement(
    tmp_path, monkeypatch
) -> None:
    def write_peer(robot: str):
        peer = tmp_path / SESSION / robot
        store = SubmapStore(peer / "geometry")
        keyframe = KeyframeId(robot, SESSION, 0)
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            [[2.1, 0.1, 0.5]],
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=((-0.1, 0.1, 0.5),),
            resolution_m=0.2,
            observed_at_ns=123,
        )
        snapshot = store.snapshot()
        (peer / "snapshot.json").write_text(json.dumps(snapshot.to_dict()))
        return peer, SnapshotKey(
            component_id_for_anchor(keyframe),
            0,
            0,
            snapshot.manifests[0].geometry_revision,
        )

    bad_peer, bad_key = write_peer("robot_0")
    good_peer, good_key = write_peer("robot_1")
    calls = {"robot_0": 0, "robot_1": 0}
    original_refresh = SnapshotDirectorySource.refresh

    def refresh(self, view, component_id):
        robot = self.peer_root.name
        calls[robot] += 1
        if robot == "robot_0" and calls[robot] > 1:
            raise ValueError("index build time budget exceeded")
        return original_refresh(self, view, component_id)

    monkeypatch.setattr(SnapshotDirectorySource, "refresh", refresh)
    now = [0.0]
    registry = IndexRegistry(
        tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1, clock=lambda: now[0]
    )

    registry.refresh_once()
    assert calls == {"robot_0": 1, "robot_1": 1}
    bad_view = registry._views[("robot_0", bad_key.component_id)]
    decoded_chunks = bad_view._chunk_cache
    assert (
        registry.query(
            "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        ).status
        is QueryStatus.OK
    )
    assert (
        registry.query(
            "robot_1", QueryRequest(good_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        ).status
        is QueryStatus.OK
    )

    # The first failed build is retried after one second, while the healthy
    # source still refreshes on every normal poll. The failed view and its
    # decoded chunk cache stay resident during the retry window.
    now[0] = 0.1
    registry.refresh_once()
    assert calls == {"robot_0": 2, "robot_1": 2}
    assert registry._views[("robot_0", bad_key.component_id)] is bad_view
    assert bad_view._chunk_cache is decoded_chunks
    assert (
        registry.query(
            "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        ).status
        is QueryStatus.UNAVAILABLE
    )
    now[0] = 0.5
    registry.refresh_once()
    assert calls == {"robot_0": 2, "robot_1": 3}
    assert registry._views[("robot_0", bad_key.component_id)] is bad_view
    assert bad_view._chunk_cache is decoded_chunks
    now[0] = 1.1
    registry.refresh_once()
    assert calls == {"robot_0": 3, "robot_1": 4}
    now[0] = 2.1
    registry.refresh_once()
    assert calls == {"robot_0": 3, "robot_1": 5}
    now[0] = 3.1
    registry.refresh_once()
    assert calls == {"robot_0": 4, "robot_1": 6}
    now[0] = 5.1
    registry.refresh_once()
    assert calls == {"robot_0": 4, "robot_1": 7}
    now[0] = 7.1
    registry.refresh_once()
    assert calls == {"robot_0": 5, "robot_1": 8}

    # Atomic replacement starts a new source revision and bypasses the
    # remaining backoff instead of waiting for the next exponential slot.
    replacement = bad_peer / "snapshot.replacement"
    replacement.write_bytes((bad_peer / "snapshot.json").read_bytes())
    os.replace(replacement, bad_peer / "snapshot.json")
    registry.refresh_once()
    assert calls == {"robot_0": 6, "robot_1": 9}
    monkeypatch.setattr(SnapshotDirectorySource, "refresh", original_refresh)
    now[0] = 8.2
    registry.refresh_once()
    assert bad_peer not in registry._failed_sources
    assert registry._views[("robot_0", bad_key.component_id)] is bad_view
    assert (
        registry.query(
            "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        ).status
        is QueryStatus.OK
    )


def test_peer_epoch_advance_fences_inflight_and_retained_merged_geometry(
    tmp_path, monkeypatch
) -> None:
    owner = "robot_0"
    peer = tmp_path / SESSION / owner
    epoch = claim_map_epoch(tmp_path, SESSION, owner)
    write_peer_epochs(peer, SESSION, {owner: epoch, "robot_1": 0})
    owner_claim = read_map_epoch(peer)
    (peer / "provider.index").write_text("ready")
    merged = SnapshotKey("component", 1, 2, "a" * 64)
    independent = SnapshotKey("component", 1, 3, "b" * 64)

    class Provider:
        def __init__(self, peer_root):
            self.publication_path = peer_root / "provider.index"

        def signature(self):
            return (1,)

        def component_ids(self):
            return (merged.component_id,)

        def refresh(self, view, component_id):
            return view.publish(
                IndexedGrid(
                    merged,
                    "c" * 64,
                    123,
                    100,
                    {(0, 0, 2)},
                    set(),
                    {},
                    0.2,
                    1,
                    robot_map_epochs={owner: epoch, "robot_1": 0},
                )
            )

    registry = IndexRegistry(
        tmp_path,
        SESSION,
        snapshot_age_s=3,
        poll_s=0.1,
        provider_factory=Provider,
    )
    registry.refresh_once()
    request = QueryRequest(
        merged, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1), now_monotonic_ns=101
    )
    assert registry.query(owner, request).occupancy == (VoxelOccupancy.OCCUPIED,)
    view = registry._views[(owner, merged.component_id)]
    terrain = view._terrain

    def advance_peer_epoch(*args):
        result = terrain(*args)
        write_peer_epochs(peer, SESSION, {owner: epoch, "robot_1": 1})
        return result

    monkeypatch.setattr(view, "_terrain", advance_peer_epoch)
    assert registry.query(owner, request).status is QueryStatus.UNAVAILABLE
    assert read_map_epoch(peer) == owner_claim
    monkeypatch.setattr(view, "_terrain", terrain)

    # A successor no longer depending on the reset peer can be served, but
    # the old merged key cannot regain validity through retained-key grace.
    view.publish(
        IndexedGrid(
            independent,
            "d" * 64,
            124,
            102,
            set(),
            {(0, 0, 2)},
            {},
            0.2,
            1,
            robot_map_epochs={owner: epoch},
        )
    )
    assert registry.query(owner, request).status is QueryStatus.UNAVAILABLE
    assert registry.query(
        owner,
        QueryRequest(
            independent, request.samples, request.body_size_xyz, now_monotonic_ns=103
        ),
    ).occupancy == (VoxelOccupancy.FREE,)


def test_robot_reset_fences_inflight_and_replayed_index_without_touching_peer(
    tmp_path, monkeypatch
) -> None:
    keys = {}
    snapshots = {}
    for robot in ("robot_0", "robot_1"):
        epoch = claim_map_epoch(tmp_path, SESSION, robot)
        run_id = robot_run_id(SESSION, robot, epoch)
        peer = tmp_path / SESSION / robot
        store = SubmapStore(peer / "geometry")
        keyframe = KeyframeId(robot, run_id, 0)
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            [[0.1, 0.1, 0.5]],
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=(),
            resolution_m=0.2,
            observed_at_ns=123,
        )
        snapshot = store.snapshot()
        value = {
            **snapshot.to_dict(),
            "run_id": run_id,
            "robot_map_epoch": epoch,
            "robot_id": robot,
            "mission_id": SESSION,
            "participant_robot_ids": [robot],
            "robot_map_epochs": {robot: epoch},
        }
        write_peer_epochs(peer, SESSION, {robot: epoch})
        snapshots[robot] = json.dumps(value)
        (peer / "snapshot.json").write_text(snapshots[robot])
        keys[robot] = SnapshotKey(
            component_id_for_anchor(keyframe),
            0,
            0,
            snapshot.manifests[0].geometry_revision,
        )
    registry = IndexRegistry(tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1)
    registry.refresh_once()
    requests = {
        robot: QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
        for robot, key in keys.items()
    }
    for robot in keys:
        assert registry.query(robot, requests[robot]).status is QueryStatus.OK
    peer_before = registry.query("robot_1", requests["robot_1"])
    old_view = registry._views[("robot_0", keys["robot_0"].component_id)]
    query = old_view.query

    def reset_during_query(request):
        result = query(request)
        claim_map_epoch(tmp_path, SESSION, "robot_0")
        return result

    monkeypatch.setattr(old_view, "query", reset_during_query)
    assert (
        registry.query("robot_0", requests["robot_0"]).status is QueryStatus.UNAVAILABLE
    )
    # The next request is fenced before the next polling refresh too.
    assert (
        registry.query("robot_0", requests["robot_0"]).status is QueryStatus.UNAVAILABLE
    )
    assert registry.query("robot_1", requests["robot_1"]) == peer_before

    # Even an old writer putting the complete snapshot back cannot revive it.
    (tmp_path / SESSION / "robot_0" / "snapshot.json").write_text(snapshots["robot_0"])
    registry.refresh_once()
    assert (
        registry.query("robot_0", requests["robot_0"]).status is QueryStatus.UNAVAILABLE
    )
    assert registry.query("robot_1", requests["robot_1"]) == peer_before


@pytest.mark.parametrize("pending_in", ["component_ids", "refresh"])
def test_registry_keeps_serving_through_a_pending_publication(
    tmp_path, pending_in
) -> None:
    from autonomy.map_provider import PublicationPending

    peer = tmp_path / SESSION / "robot_0"
    peer.mkdir(parents=True)
    (peer / "provider.index").write_text("ready")
    key = SnapshotKey("provider-component", 1, 2, "a" * 64)
    newer = SnapshotKey("provider-component", 1, 3, "c" * 64)
    pending = {"component_ids": False, "refresh": False}
    now = [100.0]

    class Provider:
        def __init__(self, peer_root):
            self.peer_root = peer_root
            self.publication_path = peer_root / "provider.index"

        def component_ids(self):
            if pending["component_ids"]:
                raise PublicationPending("index does not match current snapshot")
            return (key.component_id,)

        def signature(self):
            return (1,)

        def refresh(self, view, component_id):
            if pending["refresh"]:
                raise PublicationPending("publication changed while reading")
            return view.publish(
                IndexedGrid(
                    key, "b" * 64, 123, 1, {(0, 0, 2)}, set(), {(0, 0): (0.5,)}, 0.2, 1
                )
            )

    registry = IndexRegistry(
        tmp_path,
        SESSION,
        snapshot_age_s=3,
        poll_s=0.1,
        clock=lambda: now[0],
        provider_factory=Provider,
    )
    registry.refresh_once()
    pending[pending_in] = True
    registry.refresh_once()

    samples = ((0.1, 0.1, 0.5),)
    served = registry.query("robot_0", QueryRequest(key, samples, (0.1, 0.1, 0.1)))
    assert served.status is QueryStatus.OK
    ahead = registry.query("robot_0", QueryRequest(newer, samples, (0.1, 0.1, 0.1)))
    assert ahead.status is QueryStatus.STALE
    assert ahead.detail == "requested snapshot is not current"

    # No failure backoff either: the same source is read again on the next poll.
    pending[pending_in] = False
    registry.refresh_once()
    assert (
        registry.query("robot_0", QueryRequest(key, samples, (0.1, 0.1, 0.1))).status
        is QueryStatus.OK
    )


def test_registry_serves_a_superseded_key_within_the_grace(tmp_path) -> None:
    peer = tmp_path / SESSION / "robot_0"
    peer.mkdir(parents=True)
    (peer / "provider.index").write_text("ready")
    first = SnapshotKey("provider-component", 1, 2, "a" * 64)
    second = SnapshotKey("provider-component", 1, 3, "c" * 64)
    second_ns = 2_000_000_000

    class Provider:
        def __init__(self, peer_root):
            self.peer_root = peer_root
            self.publication_path = peer_root / "provider.index"
            self.refreshes = 0

        def component_ids(self):
            return (first.component_id,)

        def signature(self):
            return (1,)

        def refresh(self, view, component_id):
            self.refreshes += 1
            # The first poll publishes an occupied body voxel under the first
            # key; every later poll publishes it free under the second key.
            if self.refreshes == 1:
                key, occupied, free, received = first, {(0, 0, 2)}, set(), 1_000_000_000
            else:
                key, occupied, free, received = second, set(), {(0, 0, 2)}, second_ns
            return view.publish(
                IndexedGrid(key, "b" * 64, 123, received, occupied, free, {}, 0.2, 1)
            )

    registry = IndexRegistry(
        tmp_path,
        SESSION,
        snapshot_age_s=3,
        poll_s=0.1,
        superseded_grace_s=1.0,
        provider_factory=Provider,
    )
    registry.refresh_once()
    registry.refresh_once()
    view = registry._views[("robot_0", first.component_id)]
    assert view.superseded_grace_ns == 1_000_000_000
    assert view.superseded_keys == (first,)

    samples = ((0.1, 0.1, 0.5),)
    body = (0.1, 0.1, 0.1)

    def query(key, now_ns):
        return registry.query(
            "robot_0",
            QueryRequest(
                key,
                samples,
                body,
                now_monotonic_ns=now_ns,
                max_snapshot_age_ns=registry.max_snapshot_age_ns,
            ),
        )

    retained = query(first, second_ns + 1_000_000_000)
    assert retained.status is QueryStatus.OK
    assert retained.key == first
    assert retained.occupancy == (VoxelOccupancy.OCCUPIED,)
    current = query(second, second_ns + 1_000_000_000)
    assert current.status is QueryStatus.OK
    assert current.key == second
    assert current.occupancy == (VoxelOccupancy.FREE,)
    expired = query(first, second_ns + 1_000_000_001)
    assert expired.status is QueryStatus.STALE
    assert expired.detail == "requested snapshot is not current"

    # A source-wide invalidation drops the retained key with the current one.
    registry._invalidate_root(peer, "publication signature failed")
    assert view.superseded_keys == ()
    assert query(first, second_ns + 500_000_000).status is QueryStatus.STALE
    assert query(second, second_ns + 500_000_000).status is QueryStatus.UNAVAILABLE


def test_server_superseded_grace_comes_from_flag_or_environment(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.delenv("SWARMDECK_MAP_QUERY_SUPERSEDED_GRACE_S", raising=False)
    assert parse_args(["--mission-id", SESSION]).superseded_grace_s == 15.0

    monkeypatch.setenv("SWARMDECK_MAP_QUERY_SUPERSEDED_GRACE_S", "4.5")
    assert parse_args(["--mission-id", SESSION]).superseded_grace_s == 4.5
    explicit = parse_args(["--mission-id", SESSION, "--superseded-grace-s", "0"])
    assert explicit.superseded_grace_s == 0.0
    assert explicit.max_snapshot_age_s == 15.0

    with pytest.raises(SystemExit):
        parse_args(["--mission-id", SESSION, "--superseded-grace-s", "-1"])
    with pytest.raises(SystemExit):
        parse_args(["--mission-id", SESSION, "--superseded-grace-s", "nan"])
    monkeypatch.setenv("SWARMDECK_MAP_QUERY_SUPERSEDED_GRACE_S", "soon")
    with pytest.raises(SystemExit):
        parse_args(["--mission-id", SESSION])

    with pytest.raises(ValueError, match="superseded_grace_s"):
        IndexRegistry(
            tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1, superseded_grace_s=-1.0
        )
    registry = IndexRegistry(
        tmp_path, SESSION, snapshot_age_s=3, poll_s=0.1, superseded_grace_s=2.5
    )
    assert registry.superseded_grace_ns == 2_500_000_000
