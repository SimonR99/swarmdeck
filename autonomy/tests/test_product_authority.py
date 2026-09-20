from __future__ import annotations

import hashlib
import json
from types import SimpleNamespace as NS
import uuid

import numpy as np
import pytest

from adapters.mapping_authority import (
    accepts_authority_update,
    authority_for_frame,
    snapshot_values,
)
from autonomy.contracts import IDENTITY_SE3, SCHEMA_VERSION
from autonomy.cslam import CslamMapper, FrameState
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
import autonomy.product_authority as product_authority
from autonomy.product_authority import (
    ProductArtifact,
    PublishedProduct,
    WorkerStatus,
    build_authority,
    product_authority_key,
    read_published_product,
    read_worker_status,
)


def canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def manifest(component: str, epoch: int, revision: int, *, stamps=(123, 456)):
    return {
        "schema": SCHEMA_VERSION,
        "map_id": "onboard",
        "layer_id": "persistent_geometry",
        "frame_id": component.replace(":", "_"),
        "graph_revision": {
            "component_id": component,
            "epoch": epoch,
            "revision": revision,
        },
        "geometry_revision": f"{revision:064x}",
        "submaps": [{"observed_at_ns": stamp} for stamp in stamps],
        "chunks": [],
        "tombstones": [],
    }


def snapshot(*manifests):
    return {
        "schema": SCHEMA_VERSION,
        "snapshot_id": sha256(canonical(list(manifests))),
        "generated_at_ns": 100,
        "manifests": list(manifests),
    }


def write_product(
    peer,
    value,
    *,
    mutate_artifact=None,
    index_extra=None,
    source_sha=None,
    source_snapshot_id=None,
    write_index=True,
    write_source=True,
):
    """Publish ``value`` the way the MOLA worker does: source, then index."""

    mola = peer / "mola"
    mola.mkdir(parents=True, exist_ok=True)
    source_raw = canonical(value)
    artifacts = []
    for item in value["manifests"]:
        revision = item["graph_revision"]
        artifact = {
            "component_id": revision["component_id"],
            "epoch": revision["epoch"],
            "revision": revision["revision"],
            "geometry_revision": item["geometry_revision"],
            "manifest_sha256": sha256(canonical(item)),
            "geometry_fingerprint": "0" * 64,
            "path": "components/component.metricmap",
            "size_bytes": 1,
            "sha256": "1" * 64,
            "planner": {
                "path": "components/component.sdpg",
                "size_bytes": 1,
                "sha256": "2" * 64,
                "source_sha256": "3" * 64,
                "source_snapshot_id": "4" * 64,
            },
        }
        if mutate_artifact is not None:
            mutate_artifact(artifact)
        artifacts.append(artifact)
    index = {
        "version": 1,
        "source_snapshot_id": source_snapshot_id or value["snapshot_id"],
        "source_sha256": source_sha or sha256(source_raw),
        "generated_at_ns": 777,
        "artifacts": artifacts,
        **(index_extra or {}),
    }
    if write_source:
        (mola / "source.json").write_bytes(source_raw)
    if write_index:
        (mola / "index.json").write_bytes(canonical(index))
    return source_raw, index


def frame(component: str, epoch: int = 0, x: float = 0.0, order=(0, -1)):
    pose = np.eye(4)
    pose[0, 3] = x
    return FrameState(
        component,
        epoch,
        tuple(tuple(float(v) for v in row) for row in pose),
        0 if order == (0, -1) else 1,
        order,
        tuple(tuple(float(v) for v in row) for row in np.eye(4)),
    )


def test_coherent_pair_reads_one_artifact_per_component(tmp_path):
    value = snapshot(
        manifest("component:a", 0, 7, stamps=(10, 30, 20)),
        manifest("component:b", 2, 3, stamps=()),
    )
    write_product(tmp_path, value)
    sleeps = []

    product = read_published_product(tmp_path, sleep=sleeps.append)

    assert product == PublishedProduct(
        value["snapshot_id"],
        777,
        (
            ProductArtifact(
                "component:a",
                0,
                7,
                f"{7:064x}",
                sha256(canonical(value["manifests"][0])),
                30,
            ),
            ProductArtifact(
                "component:b",
                2,
                3,
                f"{3:064x}",
                sha256(canonical(value["manifests"][1])),
                0,
            ),
        ),
    )
    assert sleeps == []


def test_absent_files_are_not_a_product(tmp_path):
    sleeps = []
    assert read_published_product(tmp_path, sleep=sleeps.append) is None
    value = snapshot(manifest("component:a", 0, 1))
    write_product(tmp_path / "index-only", value, write_source=False)
    assert read_published_product(tmp_path / "index-only", sleep=sleeps.append) is None
    write_product(tmp_path / "source-only", value, write_index=False)
    assert read_published_product(tmp_path / "source-only", sleep=sleeps.append) is None
    # snapshot.json is never consulted: a peer root without a product has no
    # authority even when the bridge's own snapshot is present.
    (tmp_path / "snapshot.json").write_bytes(canonical(value))
    assert read_published_product(tmp_path, sleep=sleeps.append) is None
    assert sleeps == []


@pytest.mark.parametrize(
    "mismatch",
    [{"source_sha": "f" * 64}, {"source_snapshot_id": "e" * 64}],
)
def test_incoherent_pair_is_retried_then_abandoned(tmp_path, mismatch):
    value = snapshot(manifest("component:a", 0, 1))
    write_product(tmp_path, value, **mismatch)
    sleeps = []

    assert (
        read_published_product(
            tmp_path, attempts=3, retry_pause_s=0.002, sleep=sleeps.append
        )
        is None
    )
    # Two pauses between three reads; nothing after the last one.
    assert sleeps == [0.002, 0.002]
    assert read_published_product(tmp_path, attempts=1, sleep=sleeps.append) is None
    assert len(sleeps) == 2


def test_pair_caught_mid_publication_is_read_once_the_index_lands(tmp_path):
    old = snapshot(manifest("component:a", 0, 1))
    new = snapshot(manifest("component:a", 0, 2))
    write_product(tmp_path, old)
    # The worker replaces source.json first and index.json right after it.
    write_product(tmp_path, new, write_index=False)
    reads = []

    def land_index(pause_s):
        reads.append(pause_s)
        write_product(tmp_path, new, write_source=False)

    product = read_published_product(tmp_path, sleep=land_index)

    assert reads == [0.005]
    assert product is not None
    assert product.snapshot_id == new["snapshot_id"]
    assert [artifact.revision for artifact in product.artifacts] == [2]


@pytest.mark.parametrize(
    "mutate",
    [
        lambda item: item.update(revision=item["revision"] + 1),
        lambda item: item.update(epoch=item["epoch"] + 1),
        lambda item: item.update(geometry_revision="9" * 64),
        lambda item: item.update(component_id="component:other"),
        lambda item: item.pop("manifest_sha256"),
        lambda item: item.update(revision=-1),
    ],
)
def test_artifact_disagreeing_with_its_manifest_is_invalid(tmp_path, mutate):
    value = snapshot(manifest("component:a", 0, 4))
    write_product(tmp_path, value, mutate_artifact=mutate)
    sleeps = []

    # Invalid is not pending: nothing is retried.
    assert read_published_product(tmp_path, sleep=sleeps.append) is None
    assert sleeps == []


def test_index_must_cover_every_source_component(tmp_path):
    value = snapshot(manifest("component:a", 0, 4), manifest("component:b", 0, 4))
    write_product(
        tmp_path,
        value,
        index_extra={
            "artifacts": [
                {
                    "component_id": "component:a",
                    "epoch": 0,
                    "revision": 4,
                    "geometry_revision": f"{4:064x}",
                    "manifest_sha256": sha256(canonical(value["manifests"][0])),
                }
            ]
        },
    )
    assert read_published_product(tmp_path) is None


def test_oversized_or_malformed_files_are_ignored(tmp_path):
    value = snapshot(manifest("component:a", 0, 4))
    write_product(tmp_path, value)
    assert read_published_product(tmp_path) is not None
    assert read_published_product(tmp_path, max_bytes=64) is None
    write_product(tmp_path, value, index_extra={"version": 2})
    assert read_published_product(tmp_path) is None
    write_product(tmp_path, value, index_extra={"generated_at_ns": -1})
    assert read_published_product(tmp_path) is None
    write_product(tmp_path, value)
    (tmp_path / "mola" / "index.json").write_bytes(b"{not json")
    assert read_published_product(tmp_path) is None
    write_product(tmp_path, {**value, "schema": "other"})
    assert read_published_product(tmp_path) is None
    write_product(tmp_path, value)
    (tmp_path / "mola" / "source.json").write_bytes(b"[]")
    assert read_published_product(tmp_path) is None


def test_memo_skips_rereading_an_unchanged_pair(tmp_path, monkeypatch):
    reads = []
    bounded = product_authority._bounded_bytes

    def counting(path, maximum):
        reads.append(path.name)
        return bounded(path, maximum)

    monkeypatch.setattr(product_authority, "_bounded_bytes", counting)
    memo = {}
    # Nothing published: nothing is remembered, every tick looks again.
    assert read_published_product(tmp_path, memo=memo) is None
    assert memo == {}
    assert reads == ["index.json"]
    reads.clear()
    first = snapshot(manifest("component:a", 0, 1))
    write_product(tmp_path, first)

    product = read_published_product(tmp_path, memo=memo)
    again = read_published_product(tmp_path, memo=memo)

    assert product is not None and again is product
    assert reads == ["index.json", "source.json"]

    # An atomic replacement changes the identity and is read on the next tick.
    second = snapshot(manifest("component:a", 0, 2))
    write_product(tmp_path, second)
    replaced = read_published_product(tmp_path, memo=memo)
    assert replaced is not None
    assert replaced.snapshot_id == second["snapshot_id"]
    assert reads == ["index.json", "source.json"] * 2
    # A remembered None stands until the files change too.
    write_product(tmp_path, second, source_sha="f" * 64)
    assert read_published_product(tmp_path, memo=memo, attempts=1) is None
    assert read_published_product(tmp_path, memo=memo, attempts=1) is None
    assert reads == ["index.json", "source.json"] * 3
    # The pure call has no memo and reads every time.
    assert read_published_product(tmp_path, attempts=1) is None
    assert reads == ["index.json", "source.json"] * 4


def write_worker_status(root, **fields):
    (root / "mola").mkdir(exist_ok=True)
    status = {
        "version": 1,
        "updated_at_ns": 1_700_000_000_000_000_000,
        "source_sha256": "a" * 64,
        "error": "",
    }
    status.update(fields)
    (root / "mola/worker.json").write_text(json.dumps(status))


def test_worker_status_is_the_workers_last_attempt(tmp_path):
    """``mola/worker.json`` reaches ``status.json`` as ``product_error``.

    A component over the point budget keeps its last product while the worker
    refuses every rebuild; the refusal is only in the worker's log otherwise.
    """

    assert read_worker_status(tmp_path) is None
    write_worker_status(tmp_path)
    status = read_worker_status(tmp_path)
    assert status == WorkerStatus(1_700_000_000_000_000_000, "a" * 64, "")
    refusal = (
        "WorkerError: native runtime invalid_request: manifest exceeds point "
        "count limit of 2000000 points"
    )
    write_worker_status(tmp_path, error=refusal, source_sha256="b" * 64)
    status = read_worker_status(tmp_path)
    assert status is not None and status.error == refusal
    assert status.source_sha256 == "b" * 64
    # An unreadable snapshot leaves the attempted source empty.
    write_worker_status(
        tmp_path, error="snapshot exceeds 4194304 byte limit", source_sha256=""
    )
    status = read_worker_status(tmp_path)
    assert status is not None and status.source_sha256 == ""
    # The error is bounded to what the worker itself writes.
    write_worker_status(tmp_path, error="x" * 5000)
    status = read_worker_status(tmp_path)
    assert status is not None and len(status.error) == 2000


@pytest.mark.parametrize(
    "fields",
    [
        {"version": 2},
        {"updated_at_ns": -1},
        {"updated_at_ns": "1"},
        {"source_sha256": "not-a-digest"},
        {"error": None},
        {"error": 3},
    ],
)
def test_invalid_worker_status_is_unknown(tmp_path, fields):
    write_worker_status(tmp_path, **fields)
    assert read_worker_status(tmp_path) is None


def test_oversized_or_malformed_worker_status_is_unknown(tmp_path):
    (tmp_path / "mola").mkdir()
    (tmp_path / "mola/worker.json").write_text("{not json")
    assert read_worker_status(tmp_path) is None
    (tmp_path / "mola/worker.json").write_text("[]")
    assert read_worker_status(tmp_path) is None
    write_worker_status(tmp_path)
    assert read_worker_status(tmp_path, max_bytes=16) is None
    assert read_worker_status(tmp_path) is not None


def artifact(component: str, epoch: int, revision: int) -> ProductArtifact:
    return ProductArtifact(component, epoch, revision, "a" * 64, "b" * 64, 5)


def test_product_authority_key_requires_the_frame_of_that_revision():
    product = PublishedProduct("c" * 64, 1, (artifact("component:a", 0, 4),))
    history = {3: frame("component:a"), 4: frame("component:a", x=1.5)}

    assert product_authority_key(NS(revision=5, frame_history=history), None) is None
    assert product_authority_key(NS(revision=5, frame_history={}), product) is None
    assert product_authority_key(NS(revision=5), product) is None
    # The product is from a revision this core has not applied.
    assert product_authority_key(NS(revision=3, frame_history=history), product) is None
    # The frame at revision 4 belonged to another component or epoch.
    other = {4: frame("component:b", x=1.5)}
    assert product_authority_key(NS(revision=5, frame_history=other), product) is None
    later_epoch = {4: frame("component:a", epoch=1, x=1.5)}
    assert (
        product_authority_key(NS(revision=5, frame_history=later_epoch), product)
        is None
    )
    # The frame recorded at revision 4 is paired, not the newest one.
    selected = product_authority_key(NS(revision=5, frame_history=history), product)
    assert selected == (product.artifacts[0], history[4])

    # A product that still lists a retired component advertises the newest
    # advertisable artifact.
    with_retired = PublishedProduct(
        "c" * 64,
        1,
        (artifact("component:old", 0, 2), artifact("component:a", 1, 4)),
    )
    mixed = {2: frame("component:old"), 4: frame("component:a", epoch=1, x=1.5)}
    selected = product_authority_key(NS(revision=6, frame_history=mixed), with_retired)
    assert selected == (with_retired.artifacts[1], mixed[4])


def value(robot, seq, x):
    return NS(
        key=NS(robot_id=robot, keyframe_id=seq),
        pose=NS(position=NS(x=x, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
    )


def translated(x):
    pose = np.eye(4)
    pose[0, 3] = x
    return pose


def authority_for(core, peer, local_navigation):
    product = read_published_product(peer)
    assert product is not None
    selected = product_authority_key(core, product)
    assert selected is not None
    chosen, state = selected
    return build_authority(
        chosen,
        state,
        robot_id=core.robot_id,
        mission_id=core.mission_id,
        participants=list(core.robot_names.values()),
        navigation_frame="r0/map_frame",
        planning_frame="r0/odom",
        T_local_navigation=local_navigation,
        home_keyframe_id=core.key(0).stable_id,
        peer_slam={"keyframes": len(core.local_poses)},
    )


def test_authority_carries_the_frame_the_product_was_built_under(tmp_path):
    mission = str(uuid.uuid4())
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path / "map")),
        "r0",
        0,
        mission,
        {0: "r0"},
    )
    core.capture(0, 10, IDENTITY_SE3, [[0, 0, 0], [1, 0, 0]])
    core.capture(1, 20, translated(2.0), [[0, 0, 0]])
    assert core.revision == 2
    snapshot_at_2 = core.mapper.snapshot_dict()
    # Revision 3: the solver moves the whole graph by +5 m in x.
    msg = NS(
        success=True,
        mission_id=mission,
        solution_clock=9,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, 5.0), value(0, 1, 7.0)],
        anchor_estimates=[value(0, 0, 5.0)],
    )
    assert core.solution(msg)
    assert core.revision == 3
    core.capture(2, 30, translated(3.0), [[0, 0, 0]])
    assert core.revision == 4
    snapshot_at_4 = core.mapper.snapshot_dict()
    core.capture(3, 40, translated(4.0), [[0, 0, 0]])
    assert core.revision == 5

    peer_2 = tmp_path / "peer-2"
    peer_4 = tmp_path / "peer-4"
    write_product(peer_2, snapshot_at_2)
    write_product(peer_4, snapshot_at_4)
    local_navigation = np.eye(4)
    local_navigation[1, 3] = 0.25

    before = authority_for(core, peer_2, local_navigation)
    after = authority_for(core, peer_4, local_navigation)

    component = core.envelope()["component_id"]
    for authority in (before, after):
        assert authority["robot_id"] == "r0"
        assert authority["mission_id"] == mission
        assert authority["participants"] == ["r0"]
        assert authority["component_id"] == component
        assert authority["map_epoch"] == 0
        assert authority["navigation_frame"] == "r0/map_frame"
        assert authority["planning_frame"] == "r0/odom"
        assert authority["peer_slam"] == {"keyframes": 4}
        assert authority["home"]["keyframe_id"] == f"r0/{mission}/0"
        json.dumps(authority, allow_nan=False)

    # The product built at revision 2 predates the solution: identity
    # correction, the initial solution order and home at the origin.
    assert before["mapping_graph_revision"] == 2
    assert before["solution_order"] == [0, -1]
    assert before["correction_revision"] == 0
    np.testing.assert_allclose(before["T_component_planning"], np.eye(4))
    np.testing.assert_allclose(before["T_component_navigation"], local_navigation)
    np.testing.assert_allclose(
        before["home"]["T_navigation_home"], np.linalg.inv(local_navigation)
    )
    manifest_2 = snapshot_at_2["manifests"][0]
    assert before["geometry_revision"] == manifest_2["geometry_revision"]
    assert before["map_source_stamp"] == {"sec": 0, "nanosec": 20}

    # The product built at revision 4 was placed with the +5 m correction, the
    # accepted solver order and the moved home, even though the core has since
    # advanced to revision 5.
    assert after["mapping_graph_revision"] == 4
    assert after["solution_order"] == [9, 0]
    assert after["correction_revision"] == 1
    np.testing.assert_allclose(after["T_component_planning"], translated(5.0))
    np.testing.assert_allclose(
        after["T_component_navigation"], translated(5.0) @ local_navigation
    )
    np.testing.assert_allclose(
        after["home"]["T_navigation_home"],
        np.linalg.inv(translated(5.0) @ local_navigation) @ translated(5.0),
    )
    manifest_4 = snapshot_at_4["manifests"][0]
    assert after["geometry_revision"] == manifest_4["geometry_revision"]
    assert after["map_source_stamp"] == {"sec": 0, "nanosec": 30}

    # Consumers accept the product sequence as monotonic in (epoch, revision)
    # and solution order, refuse the rollback, and read both frame pairs.
    assert accepts_authority_update(before, None)
    assert accepts_authority_update(before, before)
    assert accepts_authority_update(after, before)
    assert not accepts_authority_update(before, after)
    assert snapshot_values(after)[:2] == (0, 4)
    planning = authority_for_frame(after, "r0/odom")
    np.testing.assert_allclose(planning["T_component_navigation"], translated(5.0))
    np.testing.assert_allclose(
        planning["home"]["T_navigation_home"],
        np.linalg.inv(translated(5.0)) @ translated(5.0),
    )
    # The current revision has no product and is never advertised.
    assert core.revision == 5
    assert (
        product_authority_key(
            core,
            PublishedProduct("c" * 64, 1, (artifact(component, 0, 6),)),
        )
        is None
    )
