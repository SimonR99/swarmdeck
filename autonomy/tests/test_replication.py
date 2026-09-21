import hashlib
from uuid import uuid4

import pytest
from autonomy.replication import MissingChunks, ReplicaStore, RevisionConflict
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
