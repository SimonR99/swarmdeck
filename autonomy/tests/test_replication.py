import hashlib
from uuid import uuid4

import pytest
from autonomy.replication import (
    MissingChunks,
    ReplicaClient,
    ReplicaStore,
    RevisionConflict,
    canonical,
)
from autonomy.map_epochs import robot_run_id


def envelope(session, data=b"points", revision=1, *, map_epoch=0, robot="robot_0"):
    digest = hashlib.sha256(data).hexdigest()
    return {
        "version": 1,
        "robot_id": robot,
        "session_id": session,
        "map_epoch": map_epoch,
        "run_id": robot_run_id(session, robot, map_epoch),
        "robot_map_epochs": {robot: map_epoch},
        "participant_robot_ids": [robot],
        "revision": revision,
        "chunks": [{"sha256": digest, "size": len(data)}],
        "snapshot": {"frame": "robot_0/map", "graph_revision": revision},
    }


def map_envelope(session, revision, submaps, chunks, *, robot="robot_0"):
    component = {
        "component_id": "component:robot_0",
        "epoch": 0,
        "revision": revision,
    }
    submaps = sorted(
        (dict(item, pose_revision=component) for item in submaps),
        key=lambda item: "/".join(map(str, ReplicaStore._submap_key(item))),
    )
    chunks = sorted(chunks, key=lambda item: item["sha256"])
    manifest = {
        "map_id": "onboard",
        "layer_id": "persistent_geometry",
        "frame_id": "component_robot_0",
        "graph_revision": component,
        "geometry_revision": ("a" if len(submaps) == 1 else "b") * 64,
        "submaps": list(submaps),
        "chunks": list(chunks),
        "tombstones": [],
    }
    snapshot = {
        "schema": "swarmdeck.autonomy.v1",
        "snapshot_id": hashlib.sha256(
            canonical([{"schema": "swarmdeck.autonomy.v1", **manifest}])
        ).hexdigest(),
        "generated_at_ns": revision,
        "manifests": [manifest],
    }
    return {
        "version": 1,
        "robot_id": robot,
        "session_id": session,
        "map_epoch": 0,
        "run_id": robot_run_id(session, robot, 0),
        "robot_map_epochs": {robot: 0},
        "participant_robot_ids": [robot],
        "revision": revision,
        "solution_order": [revision, 0],
        "component_id": component["component_id"],
        "chunks": [
            {"sha256": chunk["sha256"], "size": chunk["size_bytes"]} for chunk in chunks
        ],
        "snapshot": snapshot,
    }


def submap(session, seq, digest, *, x=0.0, revision=1):
    run = robot_run_id(session, "robot_0", 0)
    return {
        "submap_id": {"robot_id": "robot_0", "session_id": run, "seq": seq},
        "geometry_revision": seq,
        "pose_revision": {
            "component_id": "component:robot_0",
            "epoch": 0,
            "revision": revision,
        },
        "T_component_submap": [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "chunks": [
            {
                "sha256": digest,
                "encoding": "application/vnd.swarmdeck.xyz-f32.v1",
                "size_bytes": 28,
                "point_count": 1,
                "bounds": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            }
        ],
    }


def test_submap_delta_appends_only_new_history_and_negotiates_chunk(tmp_path):
    session = str(uuid4())
    first_data, second_data = b"0" * 28, b"1" * 28
    first_digest = hashlib.sha256(first_data).hexdigest()
    second_digest = hashlib.sha256(second_data).hexdigest()
    first = submap(session, 0, first_digest)
    second = submap(session, 1, second_digest)
    base = map_envelope(session, 1, [first], first["chunks"])
    current = map_envelope(
        session, 2, [first, second], first["chunks"] + second["chunks"]
    )

    patch = ReplicaClient._delta(base, current)
    assert patch is not None
    assert len(patch["snapshot_delta"]["components"][0]["upsert_submaps"]) == 1
    assert len(canonical(patch)) < len(canonical(current))

    store = ReplicaStore(tmp_path)
    store.put_chunk(first_digest, first_data)
    assert store.publish(base)
    with pytest.raises(MissingChunks):
        store.publish(patch)
    assert store.get("robot_0", session) == base
    store.put_chunk(second_digest, second_data)
    assert store.publish(patch)
    assert store.get("robot_0", session) == current
    store.close()


def test_submap_delta_pose_and_removal_reconstruct_exact_snapshot(tmp_path):
    session = str(uuid4())
    first_data, second_data = b"0" * 28, b"1" * 28
    first_digest = hashlib.sha256(first_data).hexdigest()
    second_digest = hashlib.sha256(second_data).hexdigest()
    first = submap(session, 0, first_digest)
    second = submap(session, 1, second_digest)
    base = map_envelope(session, 1, [first, second], first["chunks"] + second["chunks"])
    corrected = submap(session, 0, first_digest, x=4.0, revision=2)
    current = map_envelope(session, 2, [corrected], corrected["chunks"])

    patch = ReplicaClient._delta(base, current)
    component = patch["snapshot_delta"]["components"][0]
    assert component["upsert_submaps"] == []
    assert len(component["pose_updates"]) == 1
    assert component["removed_submaps"] == [second["submap_id"]]

    store = ReplicaStore(tmp_path)
    store.put_chunk(first_digest, first_data)
    store.put_chunk(second_digest, second_data)
    assert store.publish(base)
    assert store.publish(patch)
    assert store.get("robot_0", session) == current
    store.close()


def test_delta_gap_reports_resync_and_full_bootstrap_remains_valid(tmp_path):
    session = str(uuid4())
    data = b"0" * 28
    digest = hashlib.sha256(data).hexdigest()
    item = submap(session, 0, digest)
    base = map_envelope(session, 1, [item], item["chunks"])
    newer = map_envelope(
        session, 2, [submap(session, 0, digest, x=1.0, revision=2)], item["chunks"]
    )

    patch = ReplicaClient._delta(base, newer)
    store = ReplicaStore(tmp_path)
    store.put_chunk(digest, data)
    assert store.publish(base)
    assert store.publish(newer)
    with pytest.raises(RevisionConflict) as error_info:
        store.publish(patch)
    assert error_info.value.resync
    assert error_info.value.base_revision == 1
    assert error_info.value.current_revision == 2
    assert not store.publish(newer)
    store.close()


def test_delta_commit_fence_rechecks_base_after_reconstruction(tmp_path):
    session = str(uuid4())
    data = b"0" * 28
    digest = hashlib.sha256(data).hexdigest()
    item = submap(session, 0, digest)
    base = map_envelope(session, 1, [item], item["chunks"])
    newer = map_envelope(
        session, 2, [submap(session, 0, digest, x=1.0, revision=2)], item["chunks"]
    )
    patch = ReplicaClient._delta(base, newer)
    store = ReplicaStore(tmp_path)
    store.put_chunk(digest, data)
    assert store.publish(base)
    original_expand = store._expand_delta

    def reconstruct_then_publish(delta):
        expanded = original_expand(delta)
        assert store.publish(newer)
        return expanded

    store._expand_delta = reconstruct_then_publish
    with pytest.raises(RevisionConflict) as error_info:
        store.publish(patch)
    assert error_info.value.resync
    assert store.get("robot_0", session) == newer
    store.close()


def test_missing_chunks_cannot_replace_visible_snapshot(tmp_path):
    store = ReplicaStore(tmp_path)
    session = str(uuid4())
    original = envelope(session)
    digest = original["chunks"][0]["sha256"]
    with pytest.raises(MissingChunks):
        store.publish(original)
    assert store.get("robot_0", session) is None
    store.put_chunk(digest, b"points")
    assert store.publish(original)
    replacement = envelope(session, b"new points", 2)
    with pytest.raises(MissingChunks):
        store.publish(replacement)
    assert store.get("robot_0", session) == original
    store.close()


def test_restart_resumes_and_rejects_stale_or_conflicting_revision(tmp_path):
    session = str(uuid4())
    current = envelope(session, revision=2)
    store = ReplicaStore(tmp_path)
    digest = current["chunks"][0]["sha256"]
    store.put_chunk(digest, b"points")
    store.publish(current)
    store.close()
    recovered = ReplicaStore(tmp_path)
    assert recovered.read_chunk(digest) == b"points"
    assert not recovered.publish(current)
    with pytest.raises(RevisionConflict):
        recovered.publish(envelope(session, revision=1))
    with pytest.raises(RevisionConflict):
        recovered.publish({**current, "snapshot": {"frame": "wrong"}})
    recovered.close()


def test_checksums_and_storage_limits(tmp_path):
    store = ReplicaStore(tmp_path, max_bytes=6)
    a = hashlib.sha256(b"points").hexdigest()
    with pytest.raises(ValueError):
        store.put_chunk(a, b"corrupt")
    store.put_chunk(a, b"points")
    assert not store.put_chunk(a, b"points")
    with pytest.raises(OverflowError):
        store.put_chunk(hashlib.sha256(b"x").hexdigest(), b"x")
    with pytest.raises(ValueError):
        store.read_chunk("../replicas.sqlite")
    store.close()


def test_http_resume_and_atomic_publication(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from swarmdeck_server.api import autonomy_routes as routes

    store = ReplicaStore(tmp_path)
    monkeypatch.setattr(routes, "store", lambda: store)
    app = FastAPI()
    app.include_router(routes.router)
    item = envelope(str(uuid4()))
    digest = item["chunks"][0]["sha256"]
    with TestClient(app) as client:
        assert client.post("/api/autonomy/replicas", json=item).status_code == 409
        assert (
            client.put("/api/autonomy/chunks/" + digest, content=b"bad").status_code
            == 400
        )
        assert (
            client.put("/api/autonomy/chunks/" + digest, content=b"points").status_code
            == 200
        )
        assert client.head("/api/autonomy/chunks/" + digest).status_code == 200
        assert client.post("/api/autonomy/replicas", json=item).json()["changed"]
        assert not client.post("/api/autonomy/replicas", json=item).json()["changed"]
        assert (
            client.get("/api/autonomy/replicas/robot_0/" + item["session_id"]).json()
            == item
        )
        assert client.get("/api/autonomy/chunks/" + digest).content == b"points"
    store.close()


def test_crash_file_recovery_counts_against_budget(tmp_path):
    import hashlib

    data = b"orphaned atomic chunk"
    digest = hashlib.sha256(data).hexdigest()
    (tmp_path / "chunks").mkdir()
    (tmp_path / "chunks" / digest).write_bytes(data)
    store = ReplicaStore(tmp_path, max_bytes=len(data))
    with pytest.raises(OverflowError):
        store.put_chunk(hashlib.sha256(b"new").hexdigest(), b"new")


@pytest.mark.parametrize("body", [None, [], 1, "wrong"])
def test_manifest_requires_object(tmp_path, body):
    with pytest.raises(ValueError):
        ReplicaStore(tmp_path).publish(body)


def test_sender_negotiates_missing_chunks_once_and_pose_update_uploads_none(tmp_path):
    import io
    import json
    from urllib.error import HTTPError
    from autonomy.replication import ReplicaClient

    store = ReplicaStore(tmp_path)
    data = b"geometry"
    digest = hashlib.sha256(data).hexdigest()
    manifest = {
        "version": 1,
        "robot_id": "r0",
        "session_id": str(uuid4()),
        "revision": 1,
        "chunks": [{"sha256": digest, "size": len(data)}],
        "snapshot": {},
    }
    manifest.update(
        map_epoch=0,
        run_id=robot_run_id(manifest["session_id"], "r0", 0),
        robot_map_epochs={"r0": 0},
        participant_robot_ids=["r0"],
    )

    class Client(ReplicaClient):
        def __init__(self):
            self.calls = []

        def _request(self, path, method="GET", body=None):
            self.calls.append((method, path))
            if path == "/replicas":
                try:
                    changed = store.publish(json.loads(body))
                except MissingChunks as exc:
                    raise HTTPError(
                        path,
                        409,
                        "missing",
                        {},
                        io.BytesIO(json.dumps({"missing": exc.hashes}).encode()),
                    )
            else:
                changed = store.put_chunk(path.rsplit("/", 1)[1], body)
            return io.BytesIO(json.dumps({"changed": changed}).encode())

    client = Client()
    assert client.sync(manifest, lambda _: data)["uploaded_bytes"] == len(data)
    assert len(client.calls) == 3
    manifest["revision"] = 2
    manifest["snapshot"] = {"pose": "corrected"}
    assert client.sync(manifest, lambda _: data)["uploaded_bytes"] == 0
    assert client.calls[3:] == [("POST", "/replicas")]


def test_reserved_epoch_rejects_inflight_upload_after_restart_and_preserves_peer(
    tmp_path,
):
    session = str(uuid4())
    writer = ReplicaStore(tmp_path)
    old = envelope(session, revision=99)
    other = envelope(session, robot="robot_1")
    writer.put_chunk(old["chunks"][0]["sha256"], b"points")
    writer.publish(old)
    writer.publish(other)
    supervisor = ReplicaStore(tmp_path)
    assert supervisor.reserve_map_epoch("robot_0", session, 1)
    assert not supervisor.reserve_map_epoch("robot_0", session, 1)
    supervisor.close()
    # The uploader already negotiated/uploaded old chunks before the reset.
    writer.put_chunk(old["chunks"][0]["sha256"], b"points")
    with pytest.raises(RevisionConflict):
        writer.publish({**old, "revision": 100})
    writer.close()
    recovered = ReplicaStore(tmp_path)
    assert recovered.get("robot_0", session) is None
    assert recovered.get("robot_1", session) == other
    with pytest.raises(RevisionConflict):
        recovered.publish(old)
    fresh = envelope(session, revision=0, map_epoch=1)
    assert recovered.publish(fresh)
    assert not recovered.publish(fresh)
    assert recovered.get("robot_0", session)["run_id"] != old["run_id"]
    assert recovered.get("robot_1", session) == other
    recovered.close()


def test_new_epoch_atomically_replaces_revision_order_without_partial_commit(tmp_path):
    session = str(uuid4())
    store = ReplicaStore(tmp_path)
    old = envelope(session, revision=500)
    store.put_chunk(old["chunks"][0]["sha256"], b"points")
    store.publish(old)
    fresh = envelope(session, b"fresh", revision=0, map_epoch=1)
    with pytest.raises(MissingChunks):
        store.publish(fresh)
    assert store.map_epoch("robot_0", session) == 0
    assert store.get("robot_0", session) == old
    store.put_chunk(fresh["chunks"][0]["sha256"], b"fresh")
    assert store.publish(fresh)
    with pytest.raises(RevisionConflict):
        store.publish(old)
    with pytest.raises(ValueError):
        store.publish({**fresh, "run_id": old["run_id"], "revision": 1})
    assert store.get("robot_0", session) == fresh
    store.close()


def test_epoch_vector_only_fences_referenced_robots(tmp_path):
    session = str(uuid4())
    store = ReplicaStore(tmp_path)
    store.reserve_map_epoch("robot_0", session, 2)
    source = envelope(session, robot="robot_1")
    source["robot_map_epochs"]["robot_0"] = 0
    store.put_chunk(source["chunks"][0]["sha256"], b"points")
    assert store.publish(source)
    referenced = {
        **source,
        "revision": 2,
        "participant_robot_ids": ["robot_0", "robot_1"],
        "anchor": {
            "robot_id": "robot_0",
            "session_id": robot_run_id(session, "robot_0", 0),
            "seq": 0,
        },
    }
    with pytest.raises(RevisionConflict):
        store.publish(referenced)
    with pytest.raises(RevisionConflict):
        store.publish(
            {**source, "revision": 2, "participant_robot_ids": ["robot_0", "robot_1"]}
        )
    assert store.get("robot_1", session) == source
    store.close()


def test_delta_preserves_mapper_lexical_order_without_resending_revision_only_poses(
    tmp_path,
):
    session = str(uuid4())
    data = b"0" * 28
    digest = hashlib.sha256(data).hexdigest()
    history = [submap(session, seq, digest) for seq in range(120)]
    base = map_envelope(session, 1, history[:-1], history[0]["chunks"])
    current = map_envelope(session, 2, history, history[0]["chunks"])
    patch = ReplicaClient._delta(base, current)
    assert patch is not None
    assert len(canonical(patch)) < len(canonical(current)) / 10
    store = ReplicaStore(tmp_path)
    try:
        store.put_chunk(digest, data)
        store.publish(base)
        store.publish(patch)
        assert store.get("robot_0", session) == current
    finally:
        store.close()
