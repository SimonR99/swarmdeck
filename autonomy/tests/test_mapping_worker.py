from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import textwrap
import threading
import time
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
    """Write a fake ``swarmdeck-mola-import --serve`` next to its logs.

    The worker starts one process per peer, possibly several at once, so
    every process appends to the shared logs under ``tmp_path``: ``starts.log``
    (one pid per start, numbered under a lock), ``launches.log``,
    ``requests.log`` and ``applies.log`` (pid and monotonic start/finish of
    every apply, which lets a test see two runtimes working at the same
    time). ``first_failure`` modes keyed on ``process_number`` refer to the
    first process started, so tests that rely on them start peers in order.
    """

    executable = tmp_path / "fake-mola-runtime"
    executable.write_text("#!/usr/bin/env python3\n" + textwrap.dedent(f"""
            import fcntl
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
            with (root / "starts.log").open("a+") as stream:
                fcntl.flock(stream, fcntl.LOCK_EX)
                stream.seek(0)
                process_number = len(stream.read().splitlines()) + 1
                stream.write(str(os.getpid()) + "\\n")
                stream.flush()
                fcntl.flock(stream, fcntl.LOCK_UN)
            with (root / "launches.log").open("a") as stream:
                stream.write(json.dumps(sys.argv[1:]) + "\\n")
            ready = {{
                "protocol": 1,
                "type": "ready",
                "limits": {{
                    "max_line_bytes": 65536,
                    "max_snapshot_bytes": {worker_module.MAX_SNAPSHOT_BYTES},
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
                if (
                    process_number == 1
                    and {first_failure!r} == "death_second"
                    and request_number == 2
                ):
                    os._exit(7)
                started = time.monotonic()
                if {first_failure!r} == "slow":
                    time.sleep(0.3)
                if {first_failure!r} == "budget" and (root / "over-budget").exists():
                    # The native loader's refusal of a component above the
                    # point budget, verbatim.
                    print(json.dumps({{
                        "protocol": 1,
                        "type": "response",
                        "request_id": request["request_id"],
                        "ok": False,
                        "op": "apply",
                        "error": {{
                            "code": "invalid_request",
                            "message": (
                                "manifest exceeds point count limit of 2000000 points"
                            ),
                        }},
                    }}), flush=True)
                    continue
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
                if {first_failure!r} == "wrong_output_size_second" and request_number == 2:
                    response["output_size_bytes"] = len(artifact) + 1
                if "planner_output_path" in request:
                    planner = b"planner|" + artifact
                    Path(request["planner_output_path"]).write_bytes(planner)
                    response["planner_output_size_bytes"] = len(planner)
                    response["planner_output_sha256"] = hashlib.sha256(planner).hexdigest()
                    if {first_failure!r} == "wrong_planner_size_second" and request_number == 2:
                        response["planner_output_size_bytes"] = len(planner) + 1
                with (root / "applies.log").open("a") as stream:
                    stream.write(json.dumps({{
                        "pid": os.getpid(),
                        "map_id": request["map_id"],
                        "started": started,
                        "finished": time.monotonic(),
                    }}) + "\\n")
                print(json.dumps(response), flush=True)
            """))
    executable.chmod(0o755)
    return executable


def runtime_requests(tmp_path: Path) -> list[dict[str, object]]:
    lines = (tmp_path / "requests.log").read_text().splitlines()
    return [json.loads(line) for line in lines]


def peer_applies(tmp_path: Path, peer: Path) -> list[dict[str, object]]:
    """The apply requests one peer's runtime received, in order."""

    return [
        request
        for request in runtime_requests(tmp_path)
        if str(request.get("snapshot_path", "")).startswith(str(peer))
    ]


def runtime_applies(tmp_path: Path) -> list[dict[str, object]]:
    lines = (tmp_path / "applies.log").read_text().splitlines()
    return [json.loads(line) for line in lines]


def runtime_starts(tmp_path: Path) -> list[int]:
    return [int(line) for line in (tmp_path / "starts.log").read_text().splitlines()]


def runtime_pid(worker, peer: Path) -> int:
    return worker._runtimes[peer]._process.pid


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
        # worker.json is the worker's own status, not part of the product.
        if path.name != "worker.json":
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
    """A native response whose claimed planner size disagrees with the file
    it actually wrote is still rejected: the worker trusts a fresh output's
    reported hash, but `stat`s its size rather than trusting that blindly.
    """
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 0)])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "wrong_planner_size_second"),
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
        assert not worker._resident_by_peer
        assert peer not in worker._runtimes
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


def test_run_once_skips_rehashing_an_unchanged_snapshot(tmp_path, monkeypatch) -> None:
    """snapshot.json is stat'd, not re-read and re-hashed, once its build
    has been published and its identity (size, inode, mtime) recurs.
    """
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.run_once() == {}
        reads = []

        def counting_read(path, *a, **k):
            if Path(path).name == "snapshot.json":
                reads.append(path)
            return bounded_file_bytes(path, *a, **k)

        monkeypatch.setattr(worker_module, "_bounded_file_bytes", counting_read)
        assert worker.run_once() == {}
        assert reads == []
        # A real change is still detected and read.
        write_snapshot(peer, "a" * 64, [manifest("component:a", 2)])
        assert worker.run_once() == {}
        # run_once's own due-check plus process_peer's `_read_snapshot`.
        assert len(reads) == 2
    finally:
        worker.close()


def test_reusing_a_published_artifact_trusts_its_recorded_hash(tmp_path) -> None:
    """An unchanged component is reused by `stat`, not by re-hashing its
    published bytes: the worker wrote them once and never modifies them
    (module docstring), so a later poll trusts the index's own sha256
    instead of re-reading a component that may be several megabytes.
    """
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 0)
    second = manifest("component:b", 0)
    write_snapshot(peer, "a" * 64, [first, second])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        worker.process_peer(peer)
        index = json.loads((peer / "mola/index.json").read_text())
        held = index["artifacts"][0]
        assert held["component_id"] == "component:a"
        artifact_path = peer / "mola" / held["path"]
        original = artifact_path.read_bytes()
        # Corrupt the file's content without changing its size: a real
        # re-hash would now disagree with the recorded sha256, but the
        # worker trusts an on-disk file it wrote once and never modifies.
        artifact_path.write_bytes((b"\x00" * len(original)))
        write_snapshot(peer, "b" * 64, [first, manifest("component:b", 1)])
        worker.process_peer(peer)
        current = json.loads((peer / "mola/index.json").read_text())
        assert current["artifacts"][0] == held
        # Reused, not rebuilt: only component:b's manifest triggered a native
        # request.
        assert len(runtime_requests(tmp_path)) == 3
    finally:
        worker.close()


def test_persistent_mode_trusts_a_fresh_native_output_hash_without_rehashing(
    tmp_path, monkeypatch
) -> None:
    """A component or planner map the persistent native runtime just wrote is
    published by trusting its reported hash, not by re-hashing the file: the
    process that wrote the bytes already hashed them, in the same request.
    `stat` still catches a size disagreement (a truncated or wrong write).
    """
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 0)])

    def forbidden(*a, **k):
        raise AssertionError("_sha256_file must not be called in persistent mode")

    monkeypatch.setattr(worker_module, "_sha256_file", forbidden)
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        result = worker.process_peer(peer)
        assert result.published
        index = json.loads((peer / "mola/index.json").read_text())
        artifact = index["artifacts"][0]
        artifact_path = peer / "mola" / artifact["path"]
        # The published sha256 is exactly what the fake runtime reported,
        # which happens to be correct here; the point is it was never
        # locally recomputed to get there (`forbidden` never raised above).
        assert artifact["sha256"] == hashlib.sha256(artifact_path.read_bytes()).hexdigest()
    finally:
        worker.close()


def test_persistent_mode_still_rejects_a_size_mismatch_without_rehashing(
    tmp_path, monkeypatch
) -> None:
    """Trusting the native runtime's reported hash does not mean trusting an
    unrelated file: a response whose claimed output size disagrees with what
    is actually on disk is rejected by `stat` alone, never by hashing.
    """
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 0)])

    def forbidden(*a, **k):
        raise AssertionError("_sha256_file must not be called in persistent mode")

    monkeypatch.setattr(worker_module, "_sha256_file", forbidden)
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "wrong_output_size_second"),
        timeout_s=1,
    )
    try:
        assert worker.process_peer(peer).published
        second = manifest("component:a", 1)
        write_snapshot(peer, "b" * 64, [second])
        with pytest.raises(WorkerError, match="does not match its response"):
            worker.process_peer(peer)
    finally:
        worker.close()


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
        assert requests[-1]["map_id"] not in worker._resident_by_peer[peer]
        assert len(worker._resident_by_peer[peer]) == 1
    finally:
        worker.close()


def test_disappeared_peer_closes_its_native_runtime(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    worker = MolaWorker(tmp_path, importer=fake_runtime(tmp_path), timeout_s=1)
    try:
        assert worker.run_once() == {}
        process = worker._runtimes[peer]._process
        assert process.poll() is None
        (peer / "snapshot.json").unlink()
        assert worker.run_once() == {}
        # The peer's runtime is closed whole rather than asked to release its
        # maps one by one; no other peer shares it.
        assert [request["op"] for request in runtime_requests(tmp_path)] == ["apply"]
        assert process.poll() is not None
        assert peer not in worker._runtimes
        assert not worker._resident_by_peer
        assert peer not in worker._completed
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
        assert worker._resident_by_peer[peer]

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
        # Simulate a child lost between polling cycles.
        worker._runtimes[peer].close()
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
    """A native response whose claimed output size disagrees with the file it
    actually wrote is still rejected: the worker trusts a fresh output's
    reported hash (deploy/autonomy/mola_worker.py `_stat_matches`), but
    `stat`s its size rather than trusting that blindly too.
    """
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 1)
    write_snapshot(peer, "a" * 64, [first])
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "wrong_output_size_second"),
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


def test_worker_status_reports_the_last_attempt_and_logs_a_failure_once(
    tmp_path, capsys
) -> None:
    """A component over the budget is reported, once, and cleared on recovery.

    On benchbot (mission 1a8cc114) the native refusal repeated on every retry,
    198 and 216 identical log lines per robot, while nothing outside the
    worker's log said the product had stopped growing. The outcome of every
    attempt now lands in ``mola/worker.json`` (which the bridge reports as
    ``product_error``), and the log carries a failure when it appears or
    changes and once more when the peer publishes again.
    """

    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    (tmp_path / "over-budget").touch()
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "budget"),
        timeout_s=1,
        retry_s=0,
        parallel_peers=1,
    )
    refusal = "manifest exceeds point count limit of 2000000 points"

    def worker_status() -> dict[str, object]:
        return json.loads((peer / "mola/worker.json").read_text())

    try:
        errors = worker.run_once()
        assert set(errors) == {peer} and refusal in errors[peer]
        status = worker_status()
        assert status["version"] == 1
        assert status["error"] == errors[peer]
        assert (
            status["source_sha256"]
            == hashlib.sha256((peer / "snapshot.json").read_bytes()).hexdigest()
        )
        assert not (peer / "mola/index.json").exists()
        # Retries of the same snapshot and new revisions with the same refusal
        # keep the status current but add no log line.
        assert worker.run_once() == errors
        write_snapshot(peer, "b" * 64, [manifest("component:a", 2)])
        assert worker.run_once() == errors
        assert (
            worker_status()["source_sha256"]
            == hashlib.sha256((peer / "snapshot.json").read_bytes()).hexdigest()
        )
        lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
        assert lines == [f"swarmdeck-mola-worker: {peer}: {errors[peer]}"]

        # The component fits again (a raised budget, or a rebuilt map): the
        # product is published, the status clears, and the recovery is logged.
        (tmp_path / "over-budget").unlink()
        write_snapshot(peer, "c" * 64, [manifest("component:a", 3)])
        assert worker.run_once() == {}
        status = worker_status()
        assert status["error"] == ""
        assert (
            status["source_sha256"]
            == hashlib.sha256((peer / "snapshot.json").read_bytes()).hexdigest()
        )
        assert (
            json.loads((peer / "mola/index.json").read_text())["source_snapshot_id"]
            == "c" * 64
        )
        assert worker.run_once() == {}
        lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
        assert lines == [f"swarmdeck-mola-worker: {peer}: published again"]
        # A different failure is a new line.
        (tmp_path / "over-budget").touch()
        write_snapshot(peer, "d" * 64, [manifest("component:a", 4)])
        errors = worker.run_once()
        assert refusal in errors[peer]
        assert worker_status()["error"] == errors[peer]
        assert capsys.readouterr().out.count(errors[peer]) == 1
    finally:
        worker.close()


def test_worker_status_records_an_unreadable_snapshot(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    (peer / "snapshot.json").write_bytes(b"x" * (worker_module.MAX_SNAPSHOT_BYTES + 1))
    worker = MolaWorker(tmp_path, mode="oneshot", runner=lambda *_: None)
    errors = worker.run_once()
    assert set(errors) == {peer} and "byte limit" in errors[peer]
    status = json.loads((peer / "mola/worker.json").read_text())
    assert status == {
        "version": 1,
        "updated_at_ns": status["updated_at_ns"],
        "source_sha256": "",
        "error": errors[peer],
    }


def two_peers(tmp_path: Path) -> list[Path]:
    peers = [tmp_path / "mission" / f"robot_{index}" for index in range(2)]
    for index, peer in enumerate(peers):
        write_snapshot(peer, f"{index + 1:064x}", [manifest("component:a", 1)])
    return peers


def test_parallel_peers_build_different_peers_at_the_same_time(tmp_path) -> None:
    peers = two_peers(tmp_path)
    # Both builds must be in flight at once, or the barrier breaks the test.
    in_flight = threading.Barrier(2, timeout=5)
    intervals: list[tuple[float, float]] = []

    def slow_import(command, _timeout):
        started = time.monotonic()
        in_flight.wait()
        time.sleep(0.05)
        Path(command[3]).write_bytes(b"metric-map")
        intervals.append((started, time.monotonic()))

    worker = MolaWorker(tmp_path, mode="oneshot", runner=slow_import, parallel_peers=2)
    assert worker.run_once() == {}
    assert len(intervals) == 2
    latest_start = max(started for started, _ in intervals)
    earliest_finish = min(finished for _, finished in intervals)
    assert latest_start < earliest_finish
    for peer in peers:
        assert (peer / "mola/index.json").is_file()
        assert peer in worker._completed
    # Both outcomes were recorded: nothing is rebuilt.
    assert worker.run_once() == {}
    assert len(intervals) == 2


def test_parallel_peers_one_builds_peers_in_order_on_the_calling_thread(
    tmp_path,
) -> None:
    peers = two_peers(tmp_path)
    builds: list[tuple[Path, float, float, str]] = []

    def slow_import(command, _timeout):
        started = time.monotonic()
        time.sleep(0.05)
        Path(command[3]).write_bytes(b"metric-map")
        builds.append(
            (
                Path(command[2]).parents[1],
                started,
                time.monotonic(),
                threading.current_thread().name,
            )
        )

    worker = MolaWorker(tmp_path, mode="oneshot", runner=slow_import, parallel_peers=1)
    assert worker.run_once() == {}
    assert [peer for peer, _, _, _ in builds] == peers
    first, second = builds
    assert first[2] <= second[1]
    assert {name for _, _, _, name in builds} == {threading.main_thread().name}


def test_parallel_peers_use_one_native_runtime_each(tmp_path) -> None:
    peers = two_peers(tmp_path)
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "slow"),
        timeout_s=2,
        parallel_peers=2,
    )
    try:
        assert worker.run_once() == {}
        applies = runtime_applies(tmp_path)
        assert len(applies) == 2
        assert len({item["pid"] for item in applies}) == 2
        # The two native processes were importing at the same time.
        latest_start = max(item["started"] for item in applies)
        earliest_finish = min(item["finished"] for item in applies)
        assert latest_start < earliest_finish
        assert sorted(runtime_starts(tmp_path)) == sorted(
            runtime_pid(worker, peer) for peer in peers
        )
        for peer in peers:
            assert len(worker._resident_by_peer[peer]) == 1
            assert peer_applies(tmp_path, peer)[0]["mode"] == "replace"
    finally:
        worker.close()


def test_native_failure_restarts_only_the_failing_peers_runtime(tmp_path) -> None:
    failing = tmp_path / "mission" / "robot_0"
    healthy = tmp_path / "mission" / "robot_1"
    first = manifest("component:a", 1)
    write_snapshot(failing, "a" * 64, [first])
    write_snapshot(healthy, "b" * 64, [first])
    # Peers are built in sorted order with parallel_peers=1, so robot_0's
    # runtime is process 1: it dies on its second apply request.
    worker = MolaWorker(
        tmp_path,
        importer=fake_runtime(tmp_path, "death_second"),
        timeout_s=1,
        retry_s=0,
        parallel_peers=1,
    )
    try:
        assert worker.run_once() == {}
        failing_pid = runtime_pid(worker, failing)
        healthy_pid = runtime_pid(worker, healthy)
        assert failing_pid != healthy_pid
        second = manifest(
            "component:a", 2, geometry_revision=first["geometry_revision"]
        )
        write_snapshot(failing, "c" * 64, [second])
        write_snapshot(healthy, "d" * 64, [second])

        errors = worker.run_once()
        assert set(errors) == {failing}
        assert "exited" in errors[failing]
        assert failing not in worker._runtimes
        assert failing not in worker._resident_by_peer
        # The healthy peer kept its runtime and its resident map: its build
        # was a pose-only correction on the same process.
        assert runtime_pid(worker, healthy) == healthy_pid
        assert worker._resident_by_peer[healthy]
        assert [item["mode"] for item in peer_applies(tmp_path, healthy)] == [
            "replace",
            "pose_only",
        ]
        healthy_index = json.loads((healthy / "mola/index.json").read_text())
        assert healthy_index["source_snapshot_id"] == "d" * 64
        assert len(runtime_starts(tmp_path)) == 2

        # The failing peer alone starts a fresh runtime and rebuilds from chunks.
        assert worker.run_once() == {}
        assert runtime_pid(worker, healthy) == healthy_pid
        assert runtime_pid(worker, failing) not in (failing_pid, healthy_pid)
        assert [item["mode"] for item in peer_applies(tmp_path, failing)] == [
            "replace",
            "pose_only",
            "replace",
        ]
        assert len(runtime_starts(tmp_path)) == 3
        failing_index = json.loads((failing / "mola/index.json").read_text())
        assert failing_index["source_snapshot_id"] == "c" * 64
    finally:
        worker.close()


def test_close_stops_every_peer_runtime(tmp_path) -> None:
    peers = two_peers(tmp_path)
    worker = MolaWorker(
        tmp_path, importer=fake_runtime(tmp_path), timeout_s=1, parallel_peers=2
    )
    assert worker.run_once() == {}
    processes = [worker._runtimes[peer]._process for peer in peers]
    assert len({process.pid for process in processes}) == 2
    assert all(process.poll() is None for process in processes)
    worker.close()
    assert not worker._runtimes
    assert not worker._resident_by_peer
    assert all(process.poll() is not None for process in processes)


def test_cli_parses_parallel_peers_flag_and_environment_default(
    monkeypatch,
) -> None:
    created: list[dict[str, object]] = []

    class RecordingWorker:
        def __init__(self, maps_root, **kwargs):
            created.append(kwargs)

        def run_forever(self):
            return None

    monkeypatch.setattr(worker_module, "MolaWorker", RecordingWorker)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "12345678-1234-5678-9234-567812345678")
    monkeypatch.delenv("SWARMDECK_MOLA_PARALLEL_PEERS", raising=False)
    monkeypatch.setattr(sys, "argv", ["swarmdeck-mola-worker"])
    worker_module.main()
    assert created[-1]["parallel_peers"] == 4

    monkeypatch.setenv("SWARMDECK_MOLA_PARALLEL_PEERS", "3")
    worker_module.main()
    assert created[-1]["parallel_peers"] == 3

    monkeypatch.setattr(sys, "argv", ["swarmdeck-mola-worker", "--parallel-peers", "2"])
    worker_module.main()
    assert created[-1]["parallel_peers"] == 2

    monkeypatch.setattr(sys, "argv", ["swarmdeck-mola-worker", "--parallel-peers", "0"])
    with pytest.raises(SystemExit):
        worker_module.main()
    monkeypatch.setenv("SWARMDECK_MOLA_PARALLEL_PEERS", "0")
    monkeypatch.setattr(sys, "argv", ["swarmdeck-mola-worker"])
    with pytest.raises(SystemExit):
        worker_module.main()
    assert len(created) == 3

    with pytest.raises(ValueError, match="parallel_peers"):
        MolaWorker(Path("/maps"), mode="oneshot", parallel_peers=0)


def test_reset_during_native_build_cannot_publish_retired_geometry(tmp_path):
    from autonomy.map_epochs import claim_map_epoch, read_map_epoch, write_peer_epochs

    mission = "00000000-0000-0000-0000-000000000001"
    claim_map_epoch(tmp_path, mission, "robot_0")
    peer = tmp_path / mission / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])
    snapshot = json.loads((peer / "snapshot.json").read_text())
    snapshot["run_id"] = read_map_epoch(peer)["run_id"]
    snapshot.update(
        mission_id=mission,
        robot_id="robot_0",
        robot_map_epoch=0,
        participant_robot_ids=["robot_0"],
        robot_map_epochs={"robot_0": 0},
    )
    write_peer_epochs(peer, mission, {"robot_0": 0})
    (peer / "snapshot.json").write_text(json.dumps(snapshot))
    other = tmp_path / mission / "robot_1"
    write_snapshot(other, "b" * 64, [manifest("component:b", 1)])
    (other / "mola").mkdir()
    (other / "mola/index.json").write_text("peer-product")

    def resetting_import(command, _timeout):
        Path(command[-1]).write_bytes(b"old native product")
        claim_map_epoch(tmp_path, mission, "robot_0")

    worker = MolaWorker(tmp_path, mode="oneshot", runner=resetting_import)
    with pytest.raises(WorkerError, match="epoch advanced"):
        worker.process_peer(peer)
    assert not (peer / "mola/index.json").exists()
    assert not (peer / "mola/source.json").exists()
    assert (other / "mola/index.json").read_text() == "peer-product"
