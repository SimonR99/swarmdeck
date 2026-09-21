from __future__ import annotations

from dataclasses import replace
from functools import partial
import hashlib
import json
import math
import os
import struct
import time
import uuid

import pytest

import autonomy.mola_mapping as mola_mapping
from autonomy.contracts import (
    IDENTITY_SE3,
    DeskewStatus,
    KeyframeId,
    RayOriginAssociation,
    RayReturnSemantics,
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
from autonomy.map_provider import PublicationPending
from autonomy.map_epochs import (
    assert_map_epoch_dependencies,
    claim_map_epoch,
    robot_run_id,
    write_peer_epochs,
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
    tmp_path, *, free=((1, 0, 2),), qualified=0, alternate_json=False, retired=0
):
    peer = tmp_path / SESSION / "robot_0"
    store = SubmapStore(peer / "geometry")
    keyframe = KeyframeId("robot_0", SESSION, 0)
    # A retired endpoint keeps its place in the manifest's stored point count
    # but carries no surface sample and no occupied voxel. Its voxel (1, 0, 2)
    # is the one the qualified rays carved free.
    points = [[0.1, 0.1, 0.5]] + [[0.3, 0.1, 0.5]] * retired
    store.add_submap(
        SubmapId.from_keyframe(keyframe),
        points,
        keyframe_poses_local={keyframe: IDENTITY_SE3},
        sensor_origins_local=(),
        resolution_m=0.2,
        observed_at_ns=123,
    )
    snapshot = store.snapshot().to_dict()
    manifest = snapshot["manifests"][0]
    # The store only records qualified ray evidence for a submap with a durable
    # capture, which this provider test does not stage. Declare the evidence on
    # the published manifest directly, which is what the validator reads.
    if qualified:
        for submap in manifest["submaps"]:
            submap["sensor_origins"] = [[0.0, 0.0, 0.0]]
            submap["ray_evidence"] = {
                "return_semantics": RayReturnSemantics.FIRST_RETURN.value,
                "deskew": DeskewStatus.NOT_REQUIRED.value,
                "origin_association": RayOriginAssociation.SINGLE_CAPTURE.value,
            }
        # snapshot_id is the digest over the canonical manifests, so restate it
        # after declaring the evidence.
        snapshot["snapshot_id"] = hashlib.sha256(
            canonical(
                [
                    {**item, "schema": snapshot["schema"]}
                    for item in snapshot["manifests"]
                ]
            )
        ).hexdigest()
    source_raw = canonical(snapshot)
    # The bridge's input, which the worker consumes and the provider never
    # reads, and the product's own copy of the bytes the worker built from.
    (peer / "snapshot.json").write_bytes(source_raw)
    (peer / "mola").mkdir(parents=True)
    (peer / "mola" / "source.json").write_bytes(source_raw)
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
        "point_count": 1 + retired,
        "occupied_count": 1,
        "free_count": len(free),
        "surface_count": 1,
        "retired_count": retired,
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
    components.mkdir()
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


def test_mola_provider_does_not_reread_an_unchanged_planner_grid(
    tmp_path, monkeypatch
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    reads = 0
    stable_read = mola_mapping._bounded_stable_read

    def count_reads(path, maximum, field):
        nonlocal reads
        if field == "planner grid":
            reads += 1
        return stable_read(path, maximum, field)

    monkeypatch.setattr(mola_mapping, "_bounded_stable_read", count_reads)
    provider = MolaDirectorySource(peer, clock_ns=iter((10, 20)).__next__)
    view = IndexedMapView()

    provider.refresh(view, component)
    first_grid = provider._cache[component][1]
    provider.refresh(view, component)

    assert reads == 1
    assert provider._cache[component][1] is first_grid
    assert view._index.received_monotonic_ns == 20


def test_mola_cache_rehashes_same_size_corruption_with_retained_mtime(
    tmp_path,
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()
    key = provider.refresh(view, component)
    grid_path = peer / "mola" / "components" / "component.sdpg"
    before = grid_path.stat()
    raw = bytearray(grid_path.read_bytes())

    time.sleep(0.01)
    raw[-1] ^= 1
    grid_path.write_bytes(raw)
    os.utime(grid_path, ns=(before.st_atime_ns, before.st_mtime_ns))
    after = grid_path.stat()

    assert after.st_size == before.st_size
    assert after.st_mtime_ns == before.st_mtime_ns
    assert after.st_ctime_ns != before.st_ctime_ns
    with pytest.raises(ValueError, match="integrity check failed"):
        provider.refresh(view, component)
    assert (
        view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))).status
        is QueryStatus.UNAVAILABLE
    )


def test_mola_cache_reloads_an_atomically_replaced_artifact(
    tmp_path, monkeypatch
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    reads = 0
    stable_read = mola_mapping._bounded_stable_read

    def count_reads(path, maximum, field):
        nonlocal reads
        if field == "planner grid":
            reads += 1
        return stable_read(path, maximum, field)

    monkeypatch.setattr(mola_mapping, "_bounded_stable_read", count_reads)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()
    provider.refresh(view, component)
    first_grid = provider._cache[component][1]
    grid_path = peer / "mola" / "components" / "component.sdpg"
    replacement = grid_path.with_suffix(".replacement")
    replacement.write_bytes(grid_path.read_bytes())
    replacement.replace(grid_path)

    provider.refresh(view, component)

    assert reads == 2
    assert provider._cache[component][1] is not first_grid


def test_mola_source_replacement_during_refresh_keeps_prior_geometry(
    tmp_path, monkeypatch
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer, clock_ns=lambda: 100)
    view = IndexedMapView()
    key = provider.refresh(view, component)
    request = QueryRequest(
        key,
        ((0.1, 0.1, 0.5), (5.0, 5.0, 5.0)),
        (0.1, 0.1, 0.1),
        now_monotonic_ns=101,
        max_snapshot_age_ns=10,
    )
    before = view.query(request)
    assert before.occupancy == (VoxelOccupancy.OCCUPIED, VoxelOccupancy.UNKNOWN)
    assert provider.component_ids() == (component,)
    source_path = peer / "mola" / "source.json"
    stable_read = mola_mapping._bounded_stable_read_with_identity
    source_reads = 0

    def replace_after_read(path, maximum, field):
        nonlocal source_reads
        result = stable_read(path, maximum, field)
        if field == "MOLA source":
            assert path == source_path
            source_reads += 1
            if source_reads == 2:
                path.write_bytes(result[0] + b"\n")
        return result

    monkeypatch.setattr(
        mola_mapping, "_bounded_stable_read_with_identity", replace_after_read
    )

    with pytest.raises(PublicationPending):
        provider.refresh(view, component)

    assert source_path.read_bytes().endswith(b"\n")
    assert view.query(request) == before
    expired_request = replace(request, now_monotonic_ns=111)
    assert view.query(expired_request).status is QueryStatus.UNAVAILABLE

    # A later verified successor must not give this expired predecessor a
    # fresh retention grace. It was not live when the new key replaced it.
    successor = replace(
        view._index,
        key=SnapshotKey(component, key.epoch, key.graph_revision + 1, "b" * 64),
        manifest_digest="c" * 64,
        received_monotonic_ns=112,
    )
    view.publish(successor)
    assert (
        view.query(replace(expired_request, now_monotonic_ns=113)).status
        is QueryStatus.UNAVAILABLE
    )
    assert (
        view.query(
            replace(expired_request, key=successor.key, now_monotonic_ns=113)
        ).status
        is QueryStatus.OK
    )


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


def test_mola_provider_accepts_visibility_retired_endpoints(tmp_path) -> None:
    peer, component, geometry = write_publication(tmp_path, qualified=1, retired=1)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()

    key = provider.refresh(view, component)
    # point_count still covers every stored point; the retired endpoint simply
    # has no surface sample and no occupied voxel.
    assert view.stats.point_count == 2
    assert key == SnapshotKey(component, 0, 0, geometry)
    # The retired endpoint's own voxel is the free one the rays carved.
    assert view.query(
        QueryRequest(key, ((0.3, 0.1, 0.5),), (0.1, 0.1, 0.1))
    ).occupancy == (VoxelOccupancy.FREE,)


def test_mola_provider_rejects_surface_and_retired_counts_below_points(
    tmp_path,
) -> None:
    peer, component, _ = write_publication(tmp_path, qualified=1, retired=1)
    grid_path = peer / "mola" / "components" / "component.sdpg"
    raw = grid_path.read_bytes()
    metadata_size = struct.unpack_from("<I", raw, 8)[0]
    metadata = json.loads(raw[12 : 12 + metadata_size])
    # Claim nothing was retired while the manifest still stores two points.
    metadata["retired_count"] = 0
    replacement = canonical(metadata)
    grid_raw = (
        GRID_MAGIC
        + struct.pack("<I", len(replacement))
        + replacement
        + raw[12 + metadata_size :]
    )
    grid_path.write_bytes(grid_raw)
    index = json.loads((peer / "mola" / "index.json").read_bytes())
    index["artifacts"][0]["planner"]["size_bytes"] = len(grid_raw)
    index["artifacts"][0]["planner"]["sha256"] = hashlib.sha256(grid_raw).hexdigest()
    (peer / "mola" / "index.json").write_bytes(canonical(index))
    provider = MolaDirectorySource(peer)

    with pytest.raises(
        ValueError, match="surface and retired counts do not match point count"
    ):
        provider.refresh(IndexedMapView(), component)


def test_mola_provider_rejects_retirement_without_qualified_rays(tmp_path) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0, retired=1)
    provider = MolaDirectorySource(peer)

    with pytest.raises(ValueError, match="lacks qualified ray evidence"):
        provider.refresh(IndexedMapView(), component)


def test_mola_provider_rejects_a_grid_without_a_retired_count(tmp_path) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    grid_path = peer / "mola" / "components" / "component.sdpg"
    raw = grid_path.read_bytes()
    metadata_size = struct.unpack_from("<I", raw, 8)[0]
    metadata = json.loads(raw[12 : 12 + metadata_size])
    del metadata["retired_count"]
    replacement = canonical(metadata)
    grid_raw = (
        GRID_MAGIC
        + struct.pack("<I", len(replacement))
        + replacement
        + raw[12 + metadata_size :]
    )
    grid_path.write_bytes(grid_raw)
    index = json.loads((peer / "mola" / "index.json").read_bytes())
    index["artifacts"][0]["planner"]["size_bytes"] = len(grid_raw)
    index["artifacts"][0]["planner"]["sha256"] = hashlib.sha256(grid_raw).hexdigest()
    (peer / "mola" / "index.json").write_bytes(canonical(index))
    provider = MolaDirectorySource(peer)

    with pytest.raises(ValueError, match="metadata schema is invalid"):
        provider.refresh(IndexedMapView(), component)


def test_mola_provider_serves_product_when_bridge_snapshot_is_ahead(
    tmp_path,
) -> None:
    peer, component, geometry = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer)
    view = IndexedMapView()
    key = provider.refresh(view, component)
    signature = provider.signature()

    # The bridge lands a newer input while the worker is still building it.
    # The product describes itself through mola/source.json, so it stays
    # servable, and its signature does not move either.
    snapshot_path = peer / "snapshot.json"
    newer = json.loads(snapshot_path.read_bytes())
    newer["manifests"][0]["graph_revision"]["revision"] += 1
    newer["snapshot_id"] = "1" * 64
    snapshot_path.write_bytes(canonical(newer))

    assert provider.component_ids() == (component,)
    assert provider.refresh(view, component) == key
    assert key == SnapshotKey(component, 0, 0, geometry)
    assert provider.signature() == signature
    assert (
        view.query(QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))).status
        is QueryStatus.OK
    )

    # The provider does not read snapshot.json at all.
    snapshot_path.unlink()
    assert provider.refresh(view, component) == key
    assert provider.signature() == signature


@pytest.mark.parametrize("disagreement", ["bytes", "identity"])
def test_mola_provider_reports_a_pending_pair_when_source_and_index_disagree(
    tmp_path, disagreement
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    provider = MolaDirectorySource(peer, clock_ns=lambda: 100)
    view = IndexedMapView()
    key = provider.refresh(view, component)
    source_path = peer / "mola" / "source.json"
    index_path = peer / "mola" / "index.json"
    if disagreement == "bytes":
        # source.json already replaced, index.json not yet.
        source_path.write_bytes(source_path.read_bytes() + b"\n")
        expected = "does not match its published source bytes"
    else:
        # Same bytes, but the index claims another snapshot identity.
        index = json.loads(index_path.read_bytes())
        index["source_snapshot_id"] = "9" * 64
        index_path.write_bytes(canonical(index))
        expected = "does not match its published source identity"

    with pytest.raises(PublicationPending, match=expected):
        provider.component_ids()
    with pytest.raises(PublicationPending, match=expected):
        provider.refresh(view, component)
    request = QueryRequest(
        key,
        ((0.1, 0.1, 0.5),),
        (0.1, 0.1, 0.1),
        now_monotonic_ns=110,
        max_snapshot_age_ns=10,
    )
    result = view.query(request)
    assert result.status is QueryStatus.OK
    assert result.occupancy == (VoxelOccupancy.OCCUPIED,)
    # A pending publication is not a liveness refresh or a new map: the old
    # key expires on its original coherent-read bound and a cold view has none.
    assert (
        view.query(
            QueryRequest(
                key,
                request.samples,
                request.body_size_xyz,
                now_monotonic_ns=111,
                max_snapshot_age_ns=10,
            )
        ).status
        is QueryStatus.UNAVAILABLE
    )
    cold_view = IndexedMapView()
    with pytest.raises(PublicationPending):
        provider.refresh(cold_view, component)
    assert cold_view.query(request).status is QueryStatus.UNAVAILABLE

    # Once the pair agrees again, the same provider serves without a restart.
    if disagreement == "bytes":
        index = json.loads(index_path.read_bytes())
        index["source_sha256"] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        index_path.write_bytes(canonical(index))
    else:
        index = json.loads(index_path.read_bytes())
        index["source_snapshot_id"] = json.loads(source_path.read_bytes())[
            "snapshot_id"
        ]
        index_path.write_bytes(canonical(index))
    assert provider.refresh(view, component) == key


@pytest.mark.parametrize("reset_robot", ["robot_0", "robot_1"])
def test_pending_mola_product_cannot_preserve_a_retired_robot_lifetime(
    tmp_path, reset_robot
) -> None:
    peer, component, _ = write_publication(tmp_path, free=(), qualified=0)
    source_path = peer / "mola" / "source.json"
    index_path = peer / "mola" / "index.json"
    source = json.loads(source_path.read_bytes())
    index = json.loads(index_path.read_bytes())
    epoch = claim_map_epoch(tmp_path, SESSION, "robot_0")
    source.update(
        run_id=robot_run_id(SESSION, "robot_0", epoch),
        robot_map_epoch=epoch,
        robot_id="robot_0",
        mission_id=SESSION,
        participant_robot_ids=["robot_0", "robot_1"],
        robot_map_epochs={"robot_0": epoch, "robot_1": 0},
    )
    write_peer_epochs(peer, SESSION, {"robot_0": epoch, "robot_1": 0})
    raw = canonical(source)
    index["source_sha256"] = hashlib.sha256(raw).hexdigest()
    source_path.write_bytes(raw)
    index_path.write_bytes(canonical(index))
    provider = MolaDirectorySource(peer)
    view = IndexedMapView(
        dependency_validator=partial(assert_map_epoch_dependencies, peer)
    )
    key = provider.refresh(view, component)
    request = QueryRequest(key, ((0.1, 0.1, 0.5),), (0.1, 0.1, 0.1))
    assert view.query(request).status is QueryStatus.OK

    if reset_robot == "robot_0":
        claim_map_epoch(tmp_path, SESSION, "robot_0")
    else:
        # No remote filesystem is present: the accepted heartbeat watermark
        # fences a merged product even though its owner's claim is unchanged.
        write_peer_epochs(peer, SESSION, {"robot_0": epoch, "robot_1": 1})
    # An old writer racing reset may expose a partial pair. It must not gain
    # the same-lifetime publication grace for the cleared map.
    source_path.write_bytes(raw + b"\n")
    index_path.write_bytes(canonical(index))
    with pytest.raises(PublicationPending):
        provider.refresh(view, component)
    assert view.query(request).status is QueryStatus.UNAVAILABLE

    # A fully coherent old pair is not the new run's product either.
    source_path.write_bytes(raw)
    with pytest.raises(ValueError):
        provider.refresh(view, component)
    assert view.query(request).status is QueryStatus.UNAVAILABLE
    with pytest.raises(ValueError):
        MolaDirectorySource(peer).refresh(IndexedMapView(), component)


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
