from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest

MODULE_PATH = Path(__file__).parents[2] / "deploy/autonomy/mola_worker.py"
SPEC = importlib.util.spec_from_file_location("swarmdeck_mola_worker", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
worker_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = worker_module
SPEC.loader.exec_module(worker_module)
MolaWorker = worker_module.MolaWorker
WorkerError = worker_module.WorkerError


def manifest(component: str, revision: int) -> dict[str, object]:
    return {
        "schema": "swarmdeck.autonomy.v1",
        "map_id": "onboard",
        "layer_id": "persistent_geometry",
        "frame_id": component,
        "graph_revision": {"component_id": component, "epoch": 2, "revision": revision},
        "geometry_revision": f"{revision + 1:064x}",
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
        tmp_path, importer=Path("/bin/importer"), timeout_s=7, runner=importer
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


def test_worker_does_not_publish_source_that_changed_during_import(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    first = manifest("component:a", 1)
    write_snapshot(peer, "a" * 64, [first])

    def changing_import(command, _timeout):
        Path(command[3]).write_bytes(b"metric-map")
        write_snapshot(peer, "b" * 64, [manifest("component:a", 2)])

    worker = MolaWorker(tmp_path, runner=changing_import)
    result = worker.process_peer(peer)
    assert not result.published
    assert not (peer / "mola/index.json").exists()
    assert not list((peer / "mola/components").iterdir())


def test_failed_new_generation_keeps_last_published_index(tmp_path) -> None:
    peer = tmp_path / "mission" / "robot_0"
    write_snapshot(peer, "a" * 64, [manifest("component:a", 1)])

    def successful(command, _timeout):
        Path(command[3]).write_bytes(b"metric-map-v1")

    worker = MolaWorker(tmp_path, runner=successful)
    assert worker.process_peer(peer).published
    prior = (peer / "mola/index.json").read_bytes()
    write_snapshot(peer, "b" * 64, [manifest("component:a", 2)])

    def failed(_command, _timeout):
        raise WorkerError("native failure")

    worker.runner = failed
    with pytest.raises(WorkerError, match="native failure"):
        worker.process_peer(peer)
    assert (peer / "mola/index.json").read_bytes() == prior


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

    worker = MolaWorker(tmp_path, runner=importer)
    with pytest.raises(WorkerError, match="snapshot_id"):
        worker.process_peer(peer)
    assert not called
