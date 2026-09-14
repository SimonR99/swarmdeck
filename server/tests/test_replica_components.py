from __future__ import annotations

import copy
import hashlib
import json
from unittest.mock import Mock
from uuid import uuid4

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from autonomy.contracts import (
    ComponentRevision,
    GraphSolution,
    IDENTITY_SE3,
    KeyframeId,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.mapping import SubmapStore
from autonomy.replication import ReplicaStore
from swarmdeck_server.api import autonomy_routes, replica_views
from swarmdeck_server.api.replica_components import ComponentCatalogue, _chunk


def hashed(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def stable_id(value):
    return SubmapId.from_dict(value).stable_id


def reseal(envelope):
    """Rebuild the exact producer hashes after a semantically valid test edit."""
    all_chunks = {}
    canonical_manifests = []
    for manifest in envelope["snapshot"]["manifests"]:
        manifest["geometry_revision"] = hashed(
            [
                (
                    stable_id(submap["submap_id"]),
                    submap["geometry_revision"],
                    [chunk["sha256"] for chunk in submap["chunks"]],
                )
                for submap in sorted(
                    manifest["submaps"], key=lambda item: stable_id(item["submap_id"])
                )
            ]
        )
        chunks = {
            chunk["sha256"]: copy.deepcopy(chunk)
            for submap in manifest["submaps"]
            for chunk in submap["chunks"]
        }
        manifest["chunks"] = [chunks[name] for name in sorted(chunks)]
        all_chunks.update(chunks)
        canonical_manifests.append({"schema": "swarmdeck.autonomy.v1", **manifest})
    envelope["snapshot"]["snapshot_id"] = hashed(canonical_manifests)
    envelope["chunks"] = [
        {"sha256": name, "size": all_chunks[name]["size_bytes"]}
        for name in sorted(all_chunks)
    ]
    return envelope


def peer(
    tmp_path, robot, session, *, anchor_robot=None, order=None, revision=1, epoch=0
):
    store = SubmapStore(tmp_path / f"{robot}-{uuid4()}")
    key = KeyframeId(robot, session, 0)
    anchor = KeyframeId(anchor_robot or robot, session, 0)
    store.add_submap(
        SubmapId.from_keyframe(key),
        [[1.0, 0.0, 0.0]],
        keyframe_poses_local={key: IDENTITY_SE3},
        sensor_origins_local=(),
        observed_at_ns=100,
        resolution_m=0.2,
    )
    component = component_id_for_anchor(anchor)
    poses = {anchor: IDENTITY_SE3, key: IDENTITY_SE3}
    store.apply_solution(
        GraphSolution(
            ComponentRevision(component, epoch, revision), anchor, tuple(poses), poses
        )
    )
    snapshot = store.snapshot().to_dict()
    chunks = {
        c["sha256"]: store.get_chunk(c["sha256"])
        for m in snapshot["manifests"]
        for c in m["chunks"]
    }
    store.close()
    return {
        "version": 1,
        "robot_id": robot,
        "session_id": session,
        "revision": revision,
        "solution_order": order or [0, -1],
        "snapshot": snapshot,
        "chunks": [{"sha256": h, "size": len(data)} for h, data in chunks.items()],
    }, chunks


def selected(envelope):
    return envelope["snapshot"]["manifests"][0]


def test_component_catalogue_accepts_versioned_colored_geometry(tmp_path):
    value = {
        "sha256": "a" * 64,
        "encoding": "application/vnd.swarmdeck.xyzrgba-f32-u8.v1",
        "size_bytes": 32,
        "point_count": 1,
        "bounds": ((0, 0, 0), (0, 0, 0)),
    }
    assert _chunk(value).encoding.endswith("xyzrgba-f32-u8.v1")


def test_unclosed_components_remain_visible_and_separate(tmp_path):
    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session)
    second, _ = peer(tmp_path, "robot_1", session)
    result = ComponentCatalogue([first, second]).index()["components"]
    assert len(result) == 2
    assert all(c["available"] and c["source_count"] == 1 for c in result)
    assert all(c["solution_order"] is None for c in result)
    assert all(c["solution_order_known"] is True for c in result)


def test_legacy_missing_order_remains_readable_but_is_marked_noninteractive(tmp_path):
    session = str(uuid4())
    source, _ = peer(tmp_path, "robot_0", session)
    del source["solution_order"]
    component_id = selected(source)["graph_revision"]["component_id"]

    direct = replica_views.build_view(source, component_id=component_id)
    assembled = ComponentCatalogue([source]).view(session, component_id)

    assert direct["selected"] is not None
    assert direct["solution_order"] is None
    assert direct["solution_order_known"] is False
    assert assembled["selected"] is not None
    assert assembled["solution_order"] is None
    assert assembled["solution_order_known"] is False


def test_explicit_initial_order_is_known_in_direct_and_assembled_views(tmp_path):
    session = str(uuid4())
    source, _ = peer(tmp_path, "robot_0", session)
    component_id = selected(source)["graph_revision"]["component_id"]

    direct = replica_views.build_view(source, component_id=component_id)
    assembled = ComponentCatalogue([source]).view(session, component_id)

    assert direct["solution_order"] == [0, -1]
    assert direct["solution_order_known"] is True
    assert assembled["solution_order"] is None
    assert assembled["solution_order_known"] is True


def test_merge_uses_common_solution_not_local_epochs_or_revisions(tmp_path):
    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session, order=[7, 0], revision=140)
    second, _ = peer(
        tmp_path,
        "robot_1",
        session,
        anchor_robot="robot_0",
        order=[7, 0],
        revision=22,
        epoch=1,
    )
    catalogue = ComponentCatalogue([first, second])
    (entry,) = catalogue.index()["components"]
    assert entry["robot_ids"] == ["robot_0", "robot_1"]
    assert entry["point_count"] == 2
    view = catalogue.view(session, entry["component_id"])
    assert view["selected"]["graph_revision"] is None
    assert view["revision"] is None
    assert len(view["selected"]["submaps"]) == 2
    assert len(view["chunks"]) == 1  # Same immutable XYZ bytes, distinct observations.
    assert view["solution_order"] == (7, 0)


@pytest.mark.parametrize("order", [[8, 0], [7, 1], [0, -1], None])
def test_incompatible_or_unknown_solution_keeps_merged_view_unavailable(
    tmp_path, order
):
    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session, order=[7, 0], revision=140)
    second, _ = peer(
        tmp_path,
        "robot_1",
        session,
        anchor_robot="robot_0",
        order=order,
        revision=22,
    )
    catalogue = ComponentCatalogue([first, second])
    (entry,) = catalogue.index()["components"]
    assert not entry["available"] and entry["status"] == "syncing"
    assert entry["source_count"] == len(entry["sources"]) == 2
    assert {source["robot_id"]: source["revision"] for source in entry["sources"]} == {
        "robot_0": 140,
        "robot_1": 22,
    }
    with pytest.raises(LookupError, match="common accepted"):
        catalogue.view(session, entry["component_id"])


def test_missions_never_share_components(tmp_path):
    first, _ = peer(tmp_path, "robot_0", str(uuid4()), order=[7, 0])
    second, _ = peer(tmp_path, "robot_0", str(uuid4()), order=[7, 0])
    source = selected(first)
    target = selected(second)
    target["graph_revision"]["component_id"] = source["graph_revision"]["component_id"]
    target["frame_id"] = source["frame_id"]
    for submap in target["submaps"]:
        submap["pose_revision"] = copy.deepcopy(target["graph_revision"])
    reseal(second)
    entries = ComponentCatalogue([first, second]).index()["components"]
    assert len(entries) == 2
    assert all(e["source_count"] == 1 for e in entries)


def add_relay(destination, source):
    selected(destination)["submaps"].extend(copy.deepcopy(selected(source)["submaps"]))
    for submap in selected(destination)["submaps"]:
        submap["pose_revision"] = selected(destination)["graph_revision"]
    reseal(destination)


def test_owner_retraction_prevents_relay_resurrection(tmp_path):
    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    second, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    add_relay(second, first)
    component = selected(first)["graph_revision"]["component_id"]
    initial = ComponentCatalogue([first, second]).view(session, component)
    assert len(initial["selected"]["submaps"]) == 2
    selected(first)["submaps"] = []
    reseal(first)
    view = ComponentCatalogue([first, second]).view(session, component)
    assert len(view["selected"]["submaps"]) == 1
    assert view["selected"]["submaps"][0]["submap_id"].startswith("robot_1/")


def test_absent_owner_conflicting_relay_copies_are_rejected(tmp_path):
    session = str(uuid4())
    original, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    first, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    second, _ = peer(tmp_path, "robot_2", session, anchor_robot="robot_0", order=[7, 0])
    add_relay(first, original)
    add_relay(second, original)
    selected(second)["submaps"][-1]["T_component_submap"][0][3] = 2.0
    reseal(second)
    (entry,) = ComponentCatalogue([first, second]).index()["components"]
    assert not entry["available"] and entry["status"] == "conflict"


def test_pose_updates_change_publication_without_geometry_or_chunk_changes(tmp_path):
    session = str(uuid4())
    source, _ = peer(tmp_path, "robot_0", session)
    component = selected(source)["graph_revision"]["component_id"]
    first = ComponentCatalogue([source]).view(session, component)
    selected(source)["submaps"][0]["T_component_submap"][0][3] = 2.0
    source["revision"] += 1
    reseal(source)
    second = ComponentCatalogue([source]).view(session, component)
    assert second["snapshot_id"] != first["snapshot_id"]
    assert (
        second["selected"]["geometry_revision"]
        == first["selected"]["geometry_revision"]
    )
    assert second["chunks"] == first["chunks"]


def test_catalogue_routes_use_real_committed_metadata_without_chunk_reads(
    tmp_path, monkeypatch
):
    session = str(uuid4())
    source, chunks = peer(tmp_path, "robot_0", session)
    store = ReplicaStore(tmp_path / "replicas")
    for h, raw in chunks.items():
        store.put_chunk(h, raw)
    store.publish(source)
    monkeypatch.setattr(replica_views, "store", lambda: store)
    monkeypatch.setattr(autonomy_routes, "store", lambda: store)
    monkeypatch.setattr(
        store, "read_chunk", lambda *args: pytest.fail("Catalogue read XYZ payloads")
    )
    app = FastAPI()
    app.include_router(autonomy_routes.router)  # Production registration order.
    app.include_router(replica_views.router)
    with TestClient(app) as client:
        monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
        assert (
            client.get("/api/autonomy/replicas/components").json()["active_session_id"]
            is None
        )
        monkeypatch.setenv("SWARMDECK_MISSION_ID", session)
        response = client.get(
            "/api/autonomy/replicas/components", params={"session_id": session}
        )
        assert response.status_code == 200
        assert response.json()["active_session_id"] == session
        (entry,) = response.json()["components"]
        response = client.get(
            f"/api/autonomy/replicas/components/view/{session}",
            params={"component_id": entry["component_id"]},
        )
        assert response.status_code == 200
        assert response.json()["scope"] == "fleet"
        assert (
            client.get("/api/autonomy/replicas/components?session_id=bad").status_code
            == 400
        )
        assert (
            client.get(
                f"/api/autonomy/replicas/components/view/{session}?component_id=absent"
            ).status_code
            == 404
        )


def test_replica_snapshot_read_has_metadata_budget(tmp_path):
    store = ReplicaStore(tmp_path / "replicas")
    session = str(uuid4())
    for i in range(129):
        store.db.execute(
            "INSERT INTO manifests VALUES (?, ?, ?, ?)", (f"r{i}", session, 0, b"{}")
        )
    store.db.commit()
    with pytest.raises(OverflowError, match="budget"):
        store.snapshots(session)
    assert store.snapshots(str(uuid4())) == []


def test_historical_tombstone_manifests_are_normalized_per_component(tmp_path):
    session = str(uuid4())
    source, _ = peer(tmp_path, "robot_0", session, order=[3, 0], revision=3, epoch=1)
    head = selected(source)
    component = head["graph_revision"]["component_id"]
    historical = copy.deepcopy(head)
    historical["graph_revision"]["revision"] = 2
    historical["submaps"] = []
    historical["chunks"] = []
    historical["tombstones"] = [f"robot_0/{session}/submap/9:keyframe_retracted"]
    source["snapshot"]["manifests"].insert(0, historical)
    reseal(source)

    assert len(source["snapshot"]["manifests"]) == 2
    assert {
        manifest["graph_revision"]["component_id"]
        for manifest in source["snapshot"]["manifests"]
    } == {component}
    view = ComponentCatalogue([source]).view(session, component)
    assert len(view["selected"]["submaps"]) == 1
    assert len(view["selected"]["tombstones"]) == 1
    assert len(view["sources"]) == 1


def test_filtered_catalogue_cache_tracks_only_its_coherent_source_versions(
    tmp_path, monkeypatch
):
    session = str(uuid4())
    source, chunks = peer(tmp_path, "robot_0", session, order=[3, 0])
    replica = ReplicaStore(tmp_path / "cache")
    for name, payload in chunks.items():
        replica.put_chunk(name, payload)
    replica.publish(source)
    snapshots = Mock(wraps=replica.snapshots)
    monkeypatch.setattr(replica, "snapshots", snapshots)
    monkeypatch.setattr(replica_views, "store", lambda: replica)

    first = replica_views.current_catalogue(session)
    assert replica_views.current_catalogue(session) is first
    assert snapshots.call_count == 1

    other, other_chunks = peer(tmp_path, "robot_9", str(uuid4()), order=[3, 0])
    for name, payload in other_chunks.items():
        replica.put_chunk(name, payload)
    replica.publish(other)
    assert replica_views.current_catalogue(session) is first
    assert snapshots.call_count == 1

    source["revision"] = 2
    selected(source)["submaps"][0]["T_component_submap"][0][3] = 2.0
    reseal(source)
    replica.publish(source)
    replacement = replica_views.current_catalogue(session)
    assert replacement is not first
    assert snapshots.call_count == 2
    assert replica_views.current_catalogue(session) is replacement
    assert snapshots.call_count == 2
    replica.close()


def test_unsealed_snapshot_mutation_is_rejected_atomically(tmp_path):
    session = str(uuid4())
    source, _ = peer(tmp_path, "robot_0", session)
    selected(source)["submaps"][0]["T_component_submap"][0][3] = 4.0

    catalogue = ComponentCatalogue([source])

    assert catalogue.index()["components"] == []
    assert catalogue.index()["source_errors"][0]["robot_id"] == "robot_0"


def test_invalid_direct_owner_blocks_relay_instead_of_looking_retracted(tmp_path):
    session = str(uuid4())
    owner, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    relay, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    add_relay(relay, owner)
    owner["snapshot"]["snapshot_id"] = "0" * 64

    (entry,) = ComponentCatalogue([owner, relay]).index()["components"]

    assert not entry["available"]
    assert entry["status"] == "conflict"
    assert "owner's direct publication is invalid" in entry["detail"]


def test_direct_owner_active_state_overrides_relayed_tombstone(tmp_path):
    session = str(uuid4())
    owner, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    relay, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    add_relay(relay, owner)
    owned_id = stable_id(selected(owner)["submaps"][0]["submap_id"])
    selected(relay)["tombstones"].append(f"{owned_id}:stale_relay")
    reseal(relay)

    component = selected(owner)["graph_revision"]["component_id"]
    view = ComponentCatalogue([owner, relay]).view(session, component)

    assert any(
        submap["submap_id"] == owned_id for submap in view["selected"]["submaps"]
    )
    assert view["selected"]["tombstones"] == []


def test_same_chunk_hash_with_conflicting_descriptors_is_rejected(tmp_path):
    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    second, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    selected(second)["submaps"][0]["chunks"][0]["bounds"][0][0] = -2.0
    reseal(second)

    (entry,) = ComponentCatalogue([first, second]).index()["components"]

    assert not entry["available"]
    assert "conflicting descriptors" in entry["detail"]


def test_component_source_work_is_bounded(tmp_path, monkeypatch):
    from swarmdeck_server.api import replica_components

    session = str(uuid4())
    first, _ = peer(tmp_path, "robot_0", session, order=[7, 0])
    second, _ = peer(tmp_path, "robot_1", session, anchor_robot="robot_0", order=[7, 0])
    monkeypatch.setattr(replica_components, "MAX_SOURCE_SUBMAPS", 1)

    (entry,) = ComponentCatalogue([first, second]).index()["components"]

    assert not entry["available"]
    assert "source record budget" in entry["detail"]
