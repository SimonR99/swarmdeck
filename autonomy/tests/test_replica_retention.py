"""Storage pressure, migration, and concurrent publication/collection behavior."""

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import sqlite3
import threading
from uuid import uuid4

import pytest

from autonomy.replication import ReplicaStore


def manifest(session, data, revision=1, robot="r0"):
    return {
        "version": 1,
        "robot_id": robot,
        "session_id": session,
        "revision": revision,
        "snapshot": {},
        "chunks": [{"sha256": hashlib.sha256(data).hexdigest(), "size": len(data)}],
    }


def upload(store, item, data):
    store.put_chunk(item["chunks"][0]["sha256"], data)
    store.publish(item)


def test_replacement_grace_shared_owners_and_budget_reclamation(tmp_path):
    now = [0.0]
    store = ReplicaStore(tmp_path, max_bytes=9, retention_s=10, clock=lambda: now[0])
    session = str(uuid4())
    old = manifest(session, b"old")
    other = manifest(str(uuid4()), b"old", robot="r1")
    upload(store, old, b"old")
    store.publish(other)
    now[0] = 100
    fresh = manifest(session, b"new", 2)
    upload(store, fresh, b"new")
    now[0] = 200
    assert store.collect_unreferenced()["chunks"] == 0  # Other session still owns it.
    store.publish({**other, "revision": 2, "chunks": []})
    now[0] = 205
    assert store.collect_unreferenced()["chunks"] == 0  # Grace starts at retirement.
    now[0] = 211
    assert store.collect_unreferenced(dry_run=True) == {
        "chunks": 1,
        "bytes": 3,
        "dry_run": True,
    }
    assert store.read_chunk(old["chunks"][0]["sha256"]) == b"old"
    # Budget pressure collects eligible old geometry, never the published map.
    candidate = manifest(session, b"larger", 3)
    store.put_chunk(candidate["chunks"][0]["sha256"], b"larger")
    assert not store.has_chunk(old["chunks"][0]["sha256"])
    assert store.get("r0", session) == fresh
    store.publish(candidate)
    assert store.read_chunk(candidate["chunks"][0]["sha256"]) == b"larger"
    store.close()


def test_reupload_extends_pending_upload_grace_and_collection_is_bounded(tmp_path):
    now = [0.0]
    store = ReplicaStore(tmp_path, retention_s=10, clock=lambda: now[0])
    hashes = [hashlib.sha256(bytes([i])).hexdigest() for i in range(3)]
    for i, digest in enumerate(hashes):
        store.put_chunk(digest, bytes([i]))
    now[0] = 9
    assert not store.put_chunk(hashes[0], b"\0")
    now[0] = 11
    assert store.collect_unreferenced(limit=1)["chunks"] == 1
    assert store.collect_unreferenced(limit=1)["chunks"] == 1
    assert store.has_chunk(hashes[0])
    assert store.collect_unreferenced()["chunks"] == 0
    now[0] = 20
    assert store.collect_unreferenced()["chunks"] == 1
    store.close()


def test_failed_large_upload_commits_reclamation_without_changing_manifest(tmp_path):
    now = [0.0]
    store = ReplicaStore(tmp_path, max_bytes=5, retention_s=1, clock=lambda: now[0])
    item = manifest(str(uuid4()), b"keep")
    upload(store, item, b"keep")
    orphan = hashlib.sha256(b"x").hexdigest()
    store.put_chunk(orphan, b"x")
    now[0] = 2
    with pytest.raises(OverflowError):
        store.put_chunk(hashlib.sha256(b"big").hexdigest(), b"big")
    assert not store.has_chunk(orphan)
    assert store.db.execute("SELECT SUM(size) FROM chunks").fetchone()[0] == 4
    assert store.get("r0", item["session_id"]) == item
    store.close()


def test_old_schema_migration_preserves_references_and_recovers_interrupted_deletion(
    tmp_path,
):
    item = manifest(str(uuid4()), b"live")
    live = item["chunks"][0]["sha256"]
    gone = hashlib.sha256(b"gone").hexdigest()
    chunks = tmp_path / "chunks"
    chunks.mkdir()
    (chunks / live).write_bytes(b"live")
    db = sqlite3.connect(tmp_path / "replicas.sqlite")
    db.executescript(
        "CREATE TABLE chunks(hash TEXT PRIMARY KEY, size INTEGER NOT NULL);"
        "CREATE TABLE manifests(robot TEXT, session TEXT, revision INTEGER, body BLOB, PRIMARY KEY(robot, session));"
    )
    db.executemany("INSERT INTO chunks VALUES (?, 4)", [(live,), (gone,)])
    db.execute(
        "INSERT INTO manifests VALUES (?, ?, ?, ?)",
        ("r0", item["session_id"], 1, json.dumps(item)),
    )
    db.commit()
    db.close()
    store = ReplicaStore(tmp_path, max_bytes=5, retention_s=0)
    assert store.collect_unreferenced()["chunks"] == 0
    store.put_chunk(hashlib.sha256(b"x").hexdigest(), b"x")
    assert store.get("r0", item["session_id"]) == item
    store.close()


def test_separate_store_connections_cannot_collect_during_publication(
    tmp_path, monkeypatch
):
    writer = ReplicaStore(tmp_path, retention_s=0)
    collector = ReplicaStore(tmp_path, retention_s=0)
    item = manifest(str(uuid4()), b"live")
    digest = item["chunks"][0]["sha256"]
    writer.put_chunk(digest, b"live")
    publishing, collecting, proceed = (
        threading.Event(),
        threading.Event(),
        threading.Event(),
    )
    original = writer._set_references

    def pause_publication(*args):
        publishing.set()
        assert proceed.wait(3)
        original(*args)

    def collect():
        collecting.set()
        return collector.collect_unreferenced()

    monkeypatch.setattr(writer, "_set_references", pause_publication)
    with ThreadPoolExecutor(max_workers=2) as pool:
        published = pool.submit(writer.publish, item)
        try:
            assert publishing.wait(3)
            collected = pool.submit(collect)
            assert collecting.wait(3)
        finally:
            proceed.set()
        assert published.result(timeout=3)
        assert collected.result(timeout=3)["chunks"] == 0
    assert collector.get("r0", item["session_id"]) == item
    assert collector.read_chunk(digest) == b"live"
    writer.close()
    collector.close()
