from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import textwrap
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).parents[2] / "deploy/autonomy/mola_worker.py"
SPEC = importlib.util.spec_from_file_location("swarmdeck_mola_worker", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
worker_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = worker_module
SPEC.loader.exec_module(worker_module)
MolaWorker = worker_module.MolaWorker
WorkerError = worker_module.WorkerError
bounded_file_bytes = worker_module._bounded_file_bytes


def manifest(
    component: str, revision: int, *, geometry_revision: str | None = None
) -> dict[str, object]:
    return {
        "schema": "swarmdeck.autonomy.v1",
        "map_id": "onboard",
        "layer_id": "persistent_geometry",
        "frame_id": component,
        "graph_revision": {"component_id": component, "epoch": 2, "revision": revision},
        "geometry_revision": geometry_revision or f"{revision + 1:064x}",
        "submaps": [],
        "chunks": [],
        "tombstones": [],
    }


def write_snapshot(
    peer: Path, snapshot_id: str, manifests: list[dict[str, object]]
) -> None:
    peer.mkdir(parents=True, exist_ok=True)
    (peer / "geometry/chunks").mkdir(parents=True, exist_ok=True)
    value = {
        "schema": "swarmdeck.autonomy.v1",
        "snapshot_id": snapshot_id,
        "generated_at_ns": 100,
        "manifests": manifests,
    }
    temporary = peer / "snapshot.tmp"
    temporary.write_text(json.dumps(value, sort_keys=True))
    temporary.replace(peer / "snapshot.json")


def fake_runtime(tmp_path: Path, first_failure: str | None = None) -> Path:
    executable = tmp_path / "fake-mola-runtime"
    executable.write_text("#!/usr/bin/env python3\n" + textwrap.dedent(f"""
            import hashlib
            import json
            import os
            from pathlib import Path
            import sys
            import time

            root = Path(__file__).parent
            options = {{
                sys.argv[index]: int(sys.argv[index + 1])
                for index in range(2, len(sys.argv), 2)
            }}
            starts = root / "starts.log"
            prior = starts.read_text() if starts.exists() else ""
            process_number = len(prior.splitlines()) + 1
            with starts.open("a") as stream:
                stream.write(str(os.getpid()) + "\\n")
            with (root / "launches.log").open("a") as stream:
                stream.write(json.dumps(sys.argv[1:]) + "\\n")
            ready = {{
                "protocol": 1,
                "type": "ready",
                "limits": {{
                    "max_line_bytes": 65536,
                    "max_snapshot_bytes": 4194304,
                    "max_response_bytes": 65536,
                    "max_maps": options["--max-maps"],
                    "max_submaps_per_map": 4096,
                    "max_points_per_map": options["--max-points-per-map"],
                    "max_resident_points": options["--max-resident-points"],
                    "max_output_bytes": options["--max-output-bytes"],
                }},
            }}
            if process_number == 1 and {first_failure!r} == "total_timeout":
                time.sleep(0.3)
            print(json.dumps(ready), flush=True)
            resident = set()
            request_number = 0
            for line in sys.stdin:
                request = json.loads(line)
                request_number += 1
                with (root / "requests.log").open("a") as stream:
                    stream.write(json.dumps(request, sort_keys=True) + "\\n")
                if request["op"] == "release":
                    was_resident = request["map_id"] in resident
                    resident.discard(request["map_id"])
                    print(json.dumps({{
                        "protocol": 1,
                        "type": "response",
                        "request_id": request["request_id"],
                        "ok": True,
                        "op": "release",
                        "map_id": request["map_id"],
                        "result": "released" if was_resident else "absent",
                    }}), flush=True)
                    continue
                if (
                    (process_number == 1 and {first_failure!r} == "death")
                    or {first_failure!r} == "always_death"
                ):
                    os._exit(7)
                if process_number == 1 and {first_failure!r} == "malformed":
                    print("{{broken", flush=True)
                    continue
                if process_number == 1 and {first_failure!r} == "oversized":
                    print("x" * 65537, flush=True)
                    continue
                if (
                    (process_number == 1 and {first_failure!r} == "timeout")
                    or {first_failure!r} == "always_timeout"
                ):
                    time.sleep(2)
                if process_number == 1 and {first_failure!r} == "total_timeout":
                    time.sleep(0.3)
                if request["mode"] == "pose_only" and request["map_id"] not in resident:
                    print(json.dumps({{
                        "protocol": 1,
                        "type": "response",
                        "request_id": request["request_id"],
                        "ok": False,
                        "op": "apply",
                        "error": {{"code": "cache_miss", "message": "map is absent"}},
                    }}), flush=True)
                    continue
                source = Path(request["snapshot_path"]).read_bytes()
                snapshot = json.loads(source)
                manifest = snapshot["manifests"][0]
                revision = manifest["graph_revision"]
                artifact = (
                    request["map_id"] + "|" + request["mode"] + "|" +
                    str(revision["revision"])
                ).encode()
                Path(request["output_path"]).write_bytes(artifact)
                resident.add(request["map_id"])
                response = {{
                    "protocol": 1,
                    "type": "response",
                    "request_id": request["request_id"],
                    "ok": True,
                    "op": "apply",
                    "mode": request["mode"],
                    "result": (
                        "replaced" if request["mode"] == "replace" else "corrected"
                    ),
                    "map_id": request["map_id"],
                    "source_snapshot_id": snapshot["snapshot_id"],
                    "source_sha256": hashlib.sha256(source).hexdigest(),
                    "component_id": revision["component_id"],
                    "epoch": revision["epoch"],
                    "revision": revision["revision"],
                    "geometry_revision": manifest["geometry_revision"],
                    "submaps": len(manifest["submaps"]),
                    "points": 0,
                    "output_size_bytes": len(artifact),
                    "output_sha256": hashlib.sha256(artifact).hexdigest(),
                }}
                if {first_failure!r} == "wrong_hash_second" and request_number == 2:
                    response["output_sha256"] = "0" * 64
                if "planner_output_path" in request:
                    planner = b"planner|" + artifact
                    Path(request["planner_output_path"]).write_bytes(planner)
                    response["planner_output_size_bytes"] = len(planner)
                    response["planner_output_sha256"] = hashlib.sha256(planner).hexdigest()
                    if {first_failure!r} == "wrong_planner_hash_second" and request_number == 2:
                        response["planner_output_sha256"] = "0" * 64
                print(json.dumps(response), flush=True)
            """))
    executable.chmod(0o755)
    return executable


def runtime_requests(tmp_path: Path) -> list[dict[str, object]]:
    lines = (tmp_path / "requests.log").read_text().splitlines()
    return [json.loads(line) for line in lines]


def sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


@pytest.fixture
def publication_writes(monkeypatch) -> list[tuple[str, bytes, bytes | None]]:
    """Record every atomic publication write in order.

    Each entry is (file name, payload, source.json bytes already on disk when
    this write started), so a test can assert that source.json lands before
    index.json and holds the built snapshot bytes at that moment.
    """

    writes: list[tuple[str, bytes, bytes | None]] = []
    original = worker_module._atomic_bytes

    def recording(path: Path, payload: bytes) -> None:
        source = path.parent / "source.json"
        writes.append(
            (path.name, payload, source.read_bytes() if source.exists() else None)
        )
        original(path, payload)

    monkeypatch.setattr(worker_module, "_atomic_bytes", recording)
    return writes


def test_planner_products_reuse_unchanged_components_and_prune_pairs(tmp_path):
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 0)
    second = manifest("component:b", 0)
    write_snapshot(peer, "a" * 64, [first, second])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path),
        planner_maps=True,
        keep_generations=1,
        timeout_s=1,
    )
    try:
        worker.process_peer(peer)
        initial = json.loads((peer / "mola/index.json").read_text())
        held = initial["artifacts"][0]["planner"]
        for revision in range(1, 4):
            write_snapshot(
                peer, f"{revision:064x}", [first, manifest("component:b", revision)]
            )
            worker.process_peer(peer)
        current = json.loads((peer / "mola/index.json").read_text())
        assert current["artifacts"][0]["planner"] == held
        assert held["source_snapshot_id"] == "a" * 64
        assert len(runtime_requests(tmp_path)) == 5
        components = peer / "mola/components"
        assert len(list(components.glob("*.metricmap"))) == 2
        assert len(list(components.glob("*.sdpg"))) == 2
        for artifact in current["artifacts"]:
            assert (peer / "mola" / artifact["planner"]["path"]).is_file()
    finally:
        worker.close()


def test_invalid_planner_product_keeps_previous_generation(tmp_path):
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 0)])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "wrong_planner_hash_second"),
        planner_maps=True,
        timeout_s=1,
    )
    try:
        worker.process_peer(peer)
        previous = (peer / "mola/index.json").read_bytes()
        write_snapshot(peer, "b" * 64, [manifest("component:a", 1)])
        with pytest.raises(WorkerError, match="planner artifact"):
            worker.process_peer(peer)
        assert (peer / "mola/index.json").read_bytes() == previous
        assert not worker._resident_maps
    finally:
        worker.close()


def test_enabling_planner_products_upgrades_a_cached_metric_map(tmp_path):
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 0)])
    executable = fake_runtime(tmp_path)
    previous = MolaWorker(tmp_path, importer=executable, timeout_s=1)
    try:
        previous.process_peer(peer)
    finally:
        previous.close()
    upgraded = MolaWorker(tmp_path, importer=executable, planner_maps=True, timeout_s=1)
    try:
        assert not upgraded.run_once()
        assert len(runtime_requests(tmp_path)) == 2
        value = json.loads((peer / "mola/index.json").read_text())
        assert "planner" in value["artifacts"][0]
    finally:
        upgraded.close()


def test_bounded_read_stops_a_file_that_grows_after_stat() -> None:
    class RecordingStream(io.BytesIO):
        def close(self):
            pass

    stream = RecordingStream(b"x" * 1024)

    class GrowingPath:
        @staticmethod
        def stat():
            return SimpleNamespace(st_size=1)

        @staticmethod
        def open(_mode):
            return stream

    with pytest.raises(WorkerError, match="byte limit"):
        bounded_file_bytes(GrowingPath(), 16, "snapshot")
    assert stream.tell() == 17


def test_worker_imports_each_component_and_coalesces_same_snapshot(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(
        peer, "a" * 64, [manifest("component:a", 3), manifest("component:b", 4)]
    )
    calls: list[tuple[tuple[str, ...], float]] = []

    def importer(command, timeout):
        calls.append((tuple(command), timeout))
        Path(command[3]).write_bytes(b"real-metric-map")

    worker = MolaWorker(
        tmp_path,
        importer=Path("/bin/importer"),
        timeout_s=7,
        mode="oneshot",
        runner=importer,
    )
    assert worker.run_once() == {}
    assert len(calls) == 2
    assert all(call[1] == 7 for call in calls)
    index = json.loads((peer / "mola/index.json").read_text())
    assert index["source_snapshot_id"] == "a" * 64
    assert {item["component_id"] for item in index["artifacts"]} == {
        "component:a",
        "component:b",
    }
    for artifact in index["artifacts"]:
        assert (peer / "mola" / artifact["path"]).read_bytes() == b"real-metric-map"

    assert worker.run_once() == {}
    assert len(calls) == 2


def test_publication_writes_exact_source_bytes_before_index(
    tmp_path, publication_writes
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    # write_snapshot is not canonical JSON, so a re-serialised source.json
    # would hash differently from the bytes the worker read.
    built = (peer / "snapshot.json").read_bytes()
    assert built != json.dumps(json.loads(built), separators=(",", ":")).encode()

    def successful(command, _timeout):
        Path(command[3]).write_bytes(b"metric-map-v1")

    worker = MolaWorker(tmp_path, mode="oneshot", runner=successful)
    result = worker.process_peer(peer)

    assert result.published
    assert result.source_sha256 == sha256(built)
    assert [name for name, _, _ in publication_writes] == [
        "source.json",
        "index.json",
    ]
    source_write, index_write = publication_writes
    assert source_write[1] == built
    assert source_write[2] is None
    # source.json was already in place, with the built bytes, when index.json
    # started its replacement.
    assert index_write[2] == built
    assert (peer / "mola/source.json").read_bytes() == built
    index = json.loads((peer / "mola/index.json").read_text())
    assert index["source_sha256"] == sha256(built)
    assert index["source_snapshot_id"] == "a" * 64


def test_snapshot_replaced_during_build_publishes_earlier_bytes_then_newer(
    tmp_path, publication_writes
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    earlier = (peer / "snapshot.json").read_bytes()
    calls = 0

    def changing_import(command, _timeout):
        nonlocal calls
        calls += 1
        Path(command[3]).write_bytes(b"metric-map-%d" % calls)
        if calls == 1:
            # The bridge lands a newer snapshot while the build is running.
            write_snapshot(peer, "b" * 64, [manifest("component:a", 2)])

    worker = MolaWorker(tmp_path, mode="oneshot", runner=changing_import)
    assert worker.run_once() == {}
    newer = (peer / "snapshot.json").read_bytes()
    assert newer != earlier

    # The finished build is published as the product of the bytes it read.
    assert calls == 1
    assert (peer / "mola/source.json").read_bytes() == earlier
    assert publication_writes[1][2] == earlier
    index = json.loads((peer / "mola/index.json").read_text())
    assert index["source_snapshot_id"] == "a" * 64
    assert index["source_sha256"] == sha256(earlier)
    assert index["artifacts"][0]["revision"] == 1
    assert worker._completed[peer] == sha256(earlier)

    # The next poll sees that the newer snapshot is not what was built.
    assert worker.run_once() == {}
    assert calls == 2
    assert (peer / "mola/source.json").read_bytes() == newer
    index = json.loads((peer / "mola/index.json").read_text())
    assert index["source_snapshot_id"] == "b" * 64
    assert index["source_sha256"] == sha256(newer)
    assert index["artifacts"][0]["revision"] == 2
    assert [name for name, _, _ in publication_writes] == [
        "source.json",
        "index.json",
        "source.json",
        "index.json",
    ]

    # An unchanged snapshot is not rebuilt.
    assert worker.run_once() == {}
    assert calls == 2


def test_failed_new_generation_keeps_last_published_index(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])

    def successful(command, _timeout):
        Path(command[3]).write_bytes(b"metric-map-v1")

    worker = MolaWorker(tmp_path, mode="oneshot", runner=successful)
    assert worker.process_peer(peer).published
    prior = (peer / "mola/index.json").read_bytes()
    prior_source = (peer / "mola/source.json").read_bytes()
    write_snapshot(peer, "b" * 64, [manifest("component:a", 2)])

    def failed(_command, _timeout):
        raise WorkerError("native failure")

    worker.runner = failed
    with pytest.raises(WorkerError, match="native failure"):
        worker.process_peer(peer)
    assert (peer / "mola/index.json").read_bytes() == prior
    assert (peer / "mola/source.json").read_bytes() == prior_source


def test_worker_rejects_unbounded_or_invalid_snapshot_before_subprocess(
    tmp_path,
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    value = json.loads((peer / "snapshot.json").read_text())
    value["snapshot_id"] = "../bad"
    (peer / "snapshot.json").write_text(json.dumps(value))
    called = False

    def importer(_command, _timeout):
        nonlocal called
        called = True

    worker = MolaWorker(tmp_path, mode="oneshot", runner=importer)
    with pytest.raises(WorkerError, match="snapshot_id"):
        worker.process_peer(peer)
    assert not called


def test_persistent_runtime_reuses_exact_components_and_applies_pose_only(
    tmp_path,
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first_a = manifest("component:a", 1)
    first_b = manifest("component:b", 1)
    write_snapshot(peer, "a" * 64, [first_a, first_b])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.process_peer(peer).published
        first_index = json.loads((peer / "mola/index.json").read_text())
        first_a_path = next(
            item["path"]
            for item in first_index["artifacts"]
            if item["component_id"] == "component:a"
        )

        second_b = manifest(
            "component:b",
            2,
            geometry_revision=first_b["geometry_revision"],
        )
        write_snapshot(peer, "b" * 64, [first_a, second_b])
        assert worker.process_peer(peer).published
        second_index = json.loads((peer / "mola/index.json").read_text())
        second_a_path = next(
            item["path"]
            for item in second_index["artifacts"]
            if item["component_id"] == "component:a"
        )
        assert second_a_path == first_a_path
        assert [request["mode"] for request in runtime_requests(tmp_path)] == [
            "replace",
            "replace",
            "pose_only",
        ]
        assert len((tmp_path / "starts.log").read_text().splitlines()) == 1

        # Reuse requires both manifest identity and verified artifact bytes.
        (peer / "mola" / second_a_path).write_bytes(b"corrupt")
        third_b = manifest(
            "component:b",
            3,
            geometry_revision=first_b["geometry_revision"],
        )
        write_snapshot(peer, "c" * 64, [first_a, third_b])
        assert worker.process_peer(peer).published
        requests = runtime_requests(tmp_path)
        assert [request["mode"] for request in requests[-2:]] == [
            "pose_only",
            "pose_only",
        ]
        third_index = json.loads((peer / "mola/index.json").read_text())
        repaired = next(
            item
            for item in third_index["artifacts"]
            if item["component_id"] == "component:a"
        )
        payload = (peer / "mola" / repaired["path"]).read_bytes()
        assert repaired["sha256"] != ""
        assert payload != b"corrupt"
    finally:
        worker.close()


def test_removed_component_is_released_from_native_runtime(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first_a = manifest("component:a", 1)
    first_b = manifest("component:b", 1)
    write_snapshot(peer, "a" * 64, [first_a, first_b])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.process_peer(peer).published
        write_snapshot(peer, "b" * 64, [first_a])
        assert worker.process_peer(peer).published
        requests = runtime_requests(tmp_path)
        assert [request["op"] for request in requests] == [
            "apply",
            "apply",
            "release",
        ]
        assert requests[-1]["map_id"] not in worker._resident_maps
    finally:
        worker.close()


def test_disappeared_peer_releases_its_native_maps(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.run_once() == {}
        (peer / "snapshot.json").unlink()
        assert worker.run_once() == {}
        assert [request["op"] for request in runtime_requests(tmp_path)] == [
            "apply",
            "release",
        ]
        assert not worker._resident_maps
    finally:
        worker.close()


def test_source_race_publishes_build_and_keeps_native_state(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 1)
    write_snapshot(peer, "a" * 64, [first])
    earlier = (peer / "snapshot.json").read_bytes()
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    original_import = worker._persistent_import

    def changing_import(**kwargs):
        response = original_import(**kwargs)
        write_snapshot(
            peer,
            "b" * 64,
            [manifest("component:a", 2, geometry_revision=first["geometry_revision"])],
        )
        return response

    worker._persistent_import = changing_import
    try:
        result = worker.process_peer(peer)
        assert result.published
        assert result.source_sha256 == sha256(earlier)
        assert (peer / "mola/source.json").read_bytes() == earlier
        index = json.loads((peer / "mola/index.json").read_text())
        assert index["source_snapshot_id"] == "a" * 64
        # The resident native map is the published generation, so it stays.
        assert worker._resident_maps

        worker._persistent_import = original_import
        newer = (peer / "snapshot.json").read_bytes()
        assert worker.process_peer(peer).published
        assert (peer / "mola/source.json").read_bytes() == newer
        index = json.loads((peer / "mola/index.json").read_text())
        assert index["source_snapshot_id"] == "b" * 64
        # The pose-only correction bases on the generation index.json
        # described, without a native restart.
        requests = runtime_requests(tmp_path)
        assert [request["mode"] for request in requests] == ["replace", "pose_only"]
        assert len((tmp_path / "starts.log").read_text().splitlines()) == 1
    finally:
        worker.close()


@pytest.mark.parametrize(
    "failure", ["timeout", "total_timeout", "malformed", "oversized", "death"]
)
def test_persistent_runtime_failure_is_reaped_and_next_attempt_restarts(
    tmp_path, failure
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, failure),
        timeout_s=0.5,
        retry_s=0,
    )
    try:
        with pytest.raises(WorkerError):
            worker.process_peer(peer)
        assert not (peer / "mola/index.json").exists()
        assert not (peer / "mola/source.json").exists()
        assert worker.process_peer(peer).published
        assert len((tmp_path / "starts.log").read_text().splitlines()) == 2
        index = json.loads((peer / "mola/index.json").read_text())
        assert index["source_snapshot_id"] == "a" * 64
        assert (peer / "mola/source.json").read_bytes() == (
            peer / "snapshot.json"
        ).read_bytes()
    finally:
        worker.close()


def test_lost_native_cache_retries_pose_revision_as_coherent_replace(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 1)
    write_snapshot(peer, "a" * 64, [first])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.process_peer(peer).published
        assert worker._runtime is not None
        worker._runtime.close()  # Simulate a child lost between polling cycles.
        second = manifest(
            "component:a", 2, geometry_revision=first["geometry_revision"]
        )
        write_snapshot(peer, "b" * 64, [second])
        assert worker.process_peer(peer).published
        assert [request["mode"] for request in runtime_requests(tmp_path)] == [
            "replace",
            "pose_only",
            "replace",
        ]
        assert len((tmp_path / "starts.log").read_text().splitlines()) == 2
    finally:
        worker.close()


def test_native_artifact_claim_mismatch_keeps_last_complete_index(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 1)
    write_snapshot(peer, "a" * 64, [first])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "wrong_hash_second"),
        timeout_s=1,
    )
    try:
        assert worker.process_peer(peer).published
        prior_index = (peer / "mola/index.json").read_bytes()
        second = manifest(
            "component:a", 2, geometry_revision=first["geometry_revision"]
        )
        write_snapshot(peer, "b" * 64, [second])
        with pytest.raises(WorkerError, match="output_sha256|claim|match"):
            worker.process_peer(peer)
        assert (peer / "mola/index.json").read_bytes() == prior_index
    finally:
        worker.close()


@pytest.mark.parametrize("failure", ["always_death", "always_timeout"])
def test_repeated_runtime_failures_do_not_leak_process_file_descriptors(
    tmp_path, failure
) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, failure),
        timeout_s=0.2,
        retry_s=0,
    )
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    try:
        for _ in range(8):
            with pytest.raises(WorkerError):
                worker.process_peer(peer)
        after = len(tuple(Path("/proc/self/fd").iterdir()))
        assert after <= before + 1
        assert len((tmp_path / "starts.log").read_text().splitlines()) == 8
    finally:
        worker.close()


def test_discovery_can_be_restricted_to_one_canonical_mission(tmp_path) -> None:
    selected = "12345678-1234-5678-9234-567812345678"
    selected_peer = tmp_path / selected / "robot_0"
    other_peer = tmp_path / "87654321-4321-8765-9234-567812345678" / "robot_1"
    write_snapshot(selected_peer, "a" * 64, [])
    write_snapshot(other_peer, "b" * 64, [])
    worker = MolaWorker(tmp_path, mode="oneshot", mission_id=selected)
    assert worker.discover() == (selected_peer,)
    with pytest.raises(ValueError, match="canonical UUID"):
        MolaWorker(tmp_path, mode="oneshot", mission_id="old-mission")


def test_persistent_runtime_receives_configured_resource_limits(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path),
        timeout_s=1,
        max_points_per_map=1234,
        max_resident_points=5678,
        max_maps=12,
        max_output_bytes=9876,
    )
    try:
        assert worker.process_peer(peer).published
        launch = json.loads((tmp_path / "launches.log").read_text().splitlines()[0])
        assert launch == [
            "--serve",
            "--max-points-per-map",
            "1234",
            "--max-resident-points",
            "5678",
            "--max-maps",
            "12",
            "--max-output-bytes",
            "9876",
        ]
    finally:
        worker.close()
