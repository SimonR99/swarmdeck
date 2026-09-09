from __future__ import annotations

import json
import os
import uuid

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
    SnapshotDirectorySource,
    SnapshotKey,
    VoxelOccupancy,
)
from autonomy.mapping import SubmapStore
from deploy.autonomy.indexed_map_server import IndexRegistry

SESSION = str(uuid.UUID("caa7c022-0c69-43ef-bf06-1922b16b9f5e"))


@pytest.mark.parametrize(
    "mission_id",
    ["not-a-uuid", SESSION.upper(), "{" + SESSION + "}"],
)
def test_registry_requires_canonical_mission_uuid(tmp_path, mission_id) -> None:
    with pytest.raises(ValueError, match="mission_id"):
        IndexRegistry(tmp_path, mission_id, snapshot_age_s=3, poll_s=0.1)


def test_registry_serves_persisted_snapshot_without_ros(tmp_path) -> None:
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


def test_registry_reports_failed_new_revision_as_unavailable(tmp_path, monkeypatch) -> None:
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
    assert registry.query(
        "robot_0", QueryRequest(old_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).status is QueryStatus.OK

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
    assert registry.query(
        "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).status is QueryStatus.OK
    assert registry.query(
        "robot_1", QueryRequest(good_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).status is QueryStatus.OK

    # The first failed build is retried after one second, while the healthy
    # source still refreshes on every normal poll. The failed view and its
    # decoded chunk cache stay resident during the retry window.
    now[0] = 0.1
    registry.refresh_once()
    assert calls == {"robot_0": 2, "robot_1": 2}
    assert registry._views[("robot_0", bad_key.component_id)] is bad_view
    assert bad_view._chunk_cache is decoded_chunks
    assert registry.query(
        "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).status is QueryStatus.UNAVAILABLE
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
    assert registry.query(
        "robot_0", QueryRequest(bad_key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).status is QueryStatus.OK
