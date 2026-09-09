from __future__ import annotations

import json
import uuid

import pytest

from autonomy.contracts import (
    IDENTITY_SE3,
    KeyframeId,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.indexed_mapping import (
    QueryRequest,
    QueryStatus,
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
