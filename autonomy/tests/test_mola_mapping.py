from __future__ import annotations

import hashlib
import json
import math
import struct
import uuid

import pytest

from autonomy.contracts import (
    IDENTITY_SE3,
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
from autonomy.mola_mapping import (
    GRID_MAGIC,
    GRID_SCHEMA,
    MolaDirectorySource,
)
from deploy.autonomy.indexed_map_server import provider_factory

SESSION = str(uuid.UUID("7649e7d5-91e8-4315-9378-99f90c893146"))


def canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def write_publication(
    tmp_path, *, free=((1, 0, 2),), qualified=0, alternate_json=False
):
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        [[0.1, 0.1, 0.5]],
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=(),
        resolution_m=0.2,
        observed_at_ns=123,
    )
    snapshot = store.snapshot().to_dict()
    source_raw = canonical(snapshot)
    (peer / "snapshot.json").write_bytes(source_raw)
    manifest = snapshot["manifests"][0]
    revision = manifest["graph_revision"]
    nested_source_id = "c" * 64
    nested_source_sha = "d" * 64
    standalone_manifest = {"schema": snapshot["schema"], **manifest}
    metadata = {
        "schema": GRID_SCHEMA,
        "graph_version": {
            "component_id": revision["component_id"],
            "epoch": revision["epoch"],
            "revision": revision["revision"],
            "digest": "e" * 64,
        },
        "identity": {
            "geometry_revision": manifest["geometry_revision"],
            "native_geometry_digest": "f" * 64,
            "canonical_manifest_digest": hashlib.sha256(
                canonical(standalone_manifest)
            ).hexdigest(),
            "source_snapshot_id": nested_source_id,
            "source_sha256": nested_source_sha,
            "reference_frame": manifest["frame_id"],
        },
        "source_stamp_ns": 123,
        "resolution_m": 0.2,
        "ray_angular_resolution_rad": math.radians(5.0),
        "ray_step_fraction": 0.75,
        "point_count": 1,
        "occupied_count": 1,
        "free_count": len(free),
        "surface_count": 1,
        "ray_steps": len(free),
        "qualified_ray_keyframes": qualified,
    }
    metadata_raw = (
        json.dumps(metadata, indent=1).replace("0.2", "2e-1").encode()
        if alternate_json
        else canonical(metadata)
    )
    body = struct.pack("<qqq", 0, 0, 2)
    body += b"".join(struct.pack("<qqq", *voxel) for voxel in free)
    body += struct.pack("<qqd", 0, 0, 0.5)
    grid_raw = GRID_MAGIC + struct.pack("<I", len(metadata_raw)) + metadata_raw + body
    components = peer / "mola" / "components"
    components.mkdir(parents=True)
    grid_path = components / "component.sdpg"
    grid_path.write_bytes(grid_raw)
    index = {
        "version": 1,
        "source_snapshot_id": snapshot["snapshot_id"],
        "source_sha256": hashlib.sha256(source_raw).hexdigest(),
        "artifacts": [
            {
                "component_id": revision["component_id"],
                "epoch": revision["epoch"],
                "revision": revision["revision"],
                "geometry_revision": manifest["geometry_revision"],
                "manifest_sha256": hashlib.sha256(canonical(manifest)).hexdigest(),
                "planner": {
                    "path": "components/component.sdpg",
                    "size_bytes": len(grid_raw),
                    "sha256": hashlib.sha256(grid_raw).hexdigest(),
                    "source_snapshot_id": nested_source_id,
                    "source_sha256": nested_source_sha,
                },
            }
        ],
    }
    (peer / "mola" / "index.json").write_bytes(canonical(index))
    return peer, revision["component_id"], manifest["geometry_revision"]


def test_mola_provider_imports_once_and_refreshes_liveness(tmp_path) -> None:
    peer, component, geometry = write_publication(
        tmp_path, free=(), qualified=0, alternate_json=True
    )
    times = iter((10, 20))
    provider = MolaDirectorySource(peer, clock_ns=lambda: next(times))
    view = IndexedMapView()

    assert provider.component_ids() == (component,)
    key = provider.refresh(view, component)
    first = view._index
    provider.refresh(view, component)
    second = view._index

    assert key.geometry_revision == geometry
    assert first is not second
    assert first.occupied is second.occupied
    assert first.columns is second.columns
    result = view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1)))
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.OCCUPIED,)


def test_mola_provider_rejects_free_space_without_qualified_rays(tmp_path) -> None:
    peer, component, geometry = write_publication(tmp_path, qualified=0)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()

    with pytest.raises(ValueError, match="lacks qualified ray evidence"):
        provider.refresh(view, component)
    assert (
        view.query(
            QueryRequest(
                SnapshotKey(component, 0, 0, geometry),
                ((0.1, 0.1, 0.5),),
                (0.1, 0.1, 0.1),
            )
        ).status
        is QueryStatus.UNAVAILABLE
    )


def test_mola_provider_invalidates_when_worker_index_is_not_current(tmp_path) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()
    key = provider.refresh(view, component)
    (peer / "snapshot.json").write_bytes((peer / "snapshot.json").read_bytes() + b"\n")

    with pytest.raises(ValueError, match="current snapshot bytes"):
        provider.refresh(view, component)
    assert (
        view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))).status
        is QueryStatus.UNAVAILABLE
    )


def test_mola_cache_rechecks_planner_source_descriptor(tmp_path) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()
    provider.refresh(view, component)
    index_path = peer / "mola" / "index.json"
    index = json.loads(index_path.read_bytes())
    index["artifacts"][0]["planner"]["source_snapshot_id"] = "9" * 64
    index_path.write_bytes(canonical(index))

    with pytest.raises(ValueError, match="identity does not match"):
        provider.refresh(view, component)


def test_mola_provider_rejects_duplicate_metadata_keys(tmp_path) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    grid_path = peer / "mola" / "components" / "component.sdpg"
    raw = grid_path.read_bytes()
    metadata_size = struct.unpack_from("<I", raw, 8)[0]
    metadata = raw[12 : 12 + metadata_size]
    duplicate = b'{"schema":"bad",' + metadata[1:]
    changed = (
        GRID_MAGIC
        + struct.pack("<I", len(duplicate))
        + duplicate
        + raw[12 + metadata_size :]
    )
    grid_path.write_bytes(changed)
    index_path = peer / "mola" / "index.json"
    index = json.loads(index_path.read_bytes())
    index["artifacts"][0]["planner"]["size_bytes"] = len(changed)
    index["artifacts"][0]["planner"]["sha256"] = hashlib.sha256(changed).hexdigest()
    index_path.write_bytes(canonical(index))

    with pytest.raises(ValueError, match="duplicate JSON key"):
        MolaDirectorySource(peer).refresh(IndexedMapView(), component)


@pytest.mark.parametrize(
    ("name", "value"),
    (("max_index_bytes", 0), ("max_grid_bytes", -1), ("max_grid_bytes", True)),
)
def test_mola_provider_requires_positive_bounded_limits(tmp_path, name, value) -> None:
    with pytest.raises(ValueError, match=name):
        MolaDirectorySource(tmp_path, **{name: value})


def test_backend_selection_is_explicit_and_has_no_fallback() -> None:
    assert provider_factory("indexed") is not provider_factory("mola")
    with pytest.raises(ValueError, match="unknown map provider"):
        provider_factory("automatic")
