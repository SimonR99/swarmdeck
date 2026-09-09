from __future__ import annotations

import hashlib
from uuid import uuid4

from autonomy.replication import ReplicaStore
from swarmdeck_server.api import replica_views


def _envelope(session: str, *, revision: int = 1, pose_x: float = 2.0):
    payload = b"SDXYZ1\x00\x00" + (1).to_bytes(8, "little") + b"\x00" * 12
    digest = hashlib.sha256(payload).hexdigest()
    identity = {
        "robot_id": "robot_0",
        "session_id": session,
        "seq": 0,
    }

    def submap(component: str):
        return {
            "submap_id": identity,
            "geometry_revision": 1,
            "pose_revision": {"component_id": component, "epoch": 0, "revision": revision},
            "T_component_submap": [
                [1, 0, 0, pose_x],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            "bounds": [[0, 0, 0], [1, 1, 1]],
            "resolution_m": 0.1,
            "observed_at_ns": 1,
            "chunks": [
                {
                    "sha256": digest,
                    "encoding": "application/vnd.swarmdeck.xyz-f32.v1",
                    "size_bytes": len(payload),
                    "point_count": 1,
                    "bounds": [[0, 0, 0], [1, 1, 1]],
                }
            ],
        }

    manifests = []
    for component, x in (("component:a", pose_x), ("component:b", 8.0)):
        current = submap(component)
        current["T_component_submap"][0][3] = x
        manifests.append(
            {
                "frame_id": component.replace(":", "_"),
                "graph_revision": {"component_id": component, "epoch": 0, "revision": revision},
                "geometry_revision": f"geometry-{component}",
                "submaps": [current],
                "chunks": current["chunks"],
                "tombstones": [],
            }
        )
    return {
        "version": 1,
        "robot_id": "robot_0",
        "session_id": session,
        "revision": revision,
        "snapshot": {
            "snapshot_id": f"snapshot-{revision}",
            "generated_at_ns": 1,
            "manifests": manifests,
        },
        "chunks": [{"sha256": digest, "size": len(payload)}],
    }, digest, payload


def test_replica_view_keeps_disconnected_components_separate(monkeypatch, tmp_path):
    session = str(uuid4())
    envelope, digest, payload = _envelope(session)
    store = ReplicaStore(tmp_path)
    store.put_chunk(digest, payload)
    store.publish(envelope)
    monkeypatch.setattr(replica_views, "store", lambda: store)
    overview = replica_views.build_view(envelope)
    assert overview["component_id"] is None
    assert [c["component_id"] for c in overview["components"]] == [
        "component:a",
        "component:b",
    ]
    selected = replica_views.build_view(envelope, component_id="component:a")
    assert selected["component_id"] == "component:a"
    assert selected["selected"]["submaps"][0]["T_component_submap"][0][3] == 2.0
    assert selected["chunks"][0]["sha256"] == digest
    assert selected["source_age_s"] is None
    assert selected["age_clock"] == "unknown"
    store.close()


def test_replica_view_keeps_chunk_hash_on_pose_only_revision(monkeypatch, tmp_path):
    session = str(uuid4())
    first, digest, payload = _envelope(session, revision=1, pose_x=2.0)
    second, _, _ = _envelope(session, revision=2, pose_x=4.0)
    store = ReplicaStore(tmp_path)
    store.put_chunk(digest, payload)
    assert store.publish(first)
    assert store.publish(second)
    monkeypatch.setattr(replica_views, "store", lambda: store)
    selected = replica_views.build_view(second, component_id="component:a")
    assert selected["revision"] == 2
    assert selected["selected"]["submaps"][0]["T_component_submap"][0][3] == 4.0
    assert selected["chunks"][0]["sha256"] == digest
    store.close()
