"""CPU-only contract tests for the durable reconstruction job slice."""

from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import struct
import subprocess
import sys
import time
import uuid

import numpy as np
import pytest

from autonomy.reconstruction import (
    DurableJobRunner,
    InputManifest,
    JobState,
    PoseSnapshot,
    ResourceBudget,
    StaleResult,
    UmamiBackend,
)
from autonomy.contracts import ComponentRevision, GraphSolution, KeyframeId
from scripts.reconstruction.umami import export_colmap
from scripts.reconstruction.capture_rgbd import associate_preceding_keyframe
from scripts.reconstruction.umami_native_fixture import make_fixture


def _capture(root: Path) -> None:
    root.mkdir()
    (root / "capture_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "capture_id": "capture-a",
                "robot_id": "robot-a",
                "session_id": "session-a",
                "submap_id": "submap-a",
                "calibration_version": "cal-3",
                "optical_frame": "camera_color_optical_frame",
            }
        )
    )
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.ones((2, 2), dtype=np.float32)
    np.savez_compressed(
        root / "00000000.npz",
        rgb=rgb,
        depth_m=depth,
        K=np.array([[2.0, 0, 1.0], [0, 2.0, 1.0], [0, 0, 1.0]]),
        T_world_camera=np.eye(4),
        stamp=1.0,
        keyframe_id="kf-1",
        calibration_version="cal-3",
    )


def _fake_umami(
    root: Path, *, sleep_seconds: float = 0, child_pid_path: Path | None = None
) -> Path:
    trainer = root / "bin" / "train_colmap"
    trainer.parent.mkdir(parents=True)
    trainer.write_text(
        """#!/usr/bin/env python3
import struct, subprocess, sys, time
from pathlib import Path
child = None
if %r and len(sys.argv) >= 5:
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    Path(%r).write_text(str(child.pid))
time.sleep(float(%r) if len(sys.argv) >= 5 else 0)
out = Path(sys.argv[3]) / 'point_cloud' / 'iteration_1'
out.mkdir(parents=True, exist_ok=True)
names = ['x','y','z','scale_0','scale_1','scale_2','rot_0','rot_1','rot_2','rot_3','f_dc_0','f_dc_1','f_dc_2','opacity']
header = 'ply\\nformat binary_little_endian 1.0\\nelement vertex 1\\n' + ''.join('property float '+name+'\\n' for name in names) + 'end_header\\n'
(out / 'point_cloud.ply').write_bytes(header.encode() + struct.pack('<14f', 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0.5, 0.5, 0.5, 4))
"""
        % (
            bool(child_pid_path),
            str(child_pid_path) if child_pid_path else "",
            sleep_seconds,
        )
    )
    trainer.chmod(trainer.stat().st_mode | stat.S_IXUSR)
    return root


def _runner(tmp_path: Path, *, sleep_seconds: float = 0):
    capture = tmp_path / "capture"
    _capture(capture)
    umami = _fake_umami(tmp_path / "umami", sleep_seconds=sleep_seconds)
    config = tmp_path / "umami.yaml"
    config.write_text("fixed_pose: true\\n")
    backend = UmamiBackend(umami_root=umami, config=config)
    runner = DurableJobRunner(tmp_path / "jobs", [backend])
    return runner, capture, backend


def test_fixed_pose_umami_batch_publishes_and_records_manifest(tmp_path):
    runner, capture, backend = _runner(tmp_path)
    target = tmp_path / "published" / "model.swgs"
    job = runner.submit(
        InputManifest.from_capture(capture),
        backend=backend.name,
        artifact_target=target,
        budget=ResourceBudget(max_frames=2, max_gaussians=10),
    )

    result = runner.run(job.job_id)

    assert result.state is JobState.READY
    assert target.read_bytes()[:4] == b"SWGS"
    manifest = json.loads(target.with_name("model.swgs.manifest.json").read_text())
    assert manifest["input_fingerprint"] == job.input_manifest.fingerprint
    assert manifest["source"]["capture_id"] == "capture-a"
    assert manifest["source"]["frame_count"] == 1
    assert (
        manifest["source"]["frame_manifest_sha256"]
        == job.input_manifest.frame_manifest_digest
    )
    assert backend.capabilities.fixed_camera_poses
    assert not backend.capabilities.checkpoint_resume
    assert not backend.capabilities.incremental_updates


def test_raw_odom_capture_is_published_as_local_frame(tmp_path):
    capture = tmp_path / "odom-capture"
    capture.mkdir()
    (capture / "capture_manifest.json").write_text(
        json.dumps(
            {
                "capture_id": "capture-odom",
                "robot_id": "robot-odom",
                "session_id": "session-odom",
                "world_frame": "odom",
            }
        )
    )
    (capture / "00000000.npz").write_bytes(b"raw-frame")

    manifest = InputManifest.from_capture(capture)
    source = manifest.publication_source()

    assert manifest.world_frame == "odom"
    assert source["frame"] == "local"
    assert source["frame_id"] == "odom"
    assert source["robot_id"] == "robot-odom"
    assert source["session_id"] == "session-odom"


def test_pose_snapshot_composes_corrected_component_camera_pose(tmp_path):
    session = str(uuid.uuid4())
    keyframe = KeyframeId("robot-a", session, 0)
    corrected = np.eye(4)
    corrected[0, 3] = 10.0
    solution = GraphSolution(
        ComponentRevision("component:corrected", 0, 7),
        keyframe,
        (keyframe,),
        {keyframe: corrected},
    )
    snapshot_path = tmp_path / "graph-solution.json"
    snapshot_path.write_text(
        json.dumps(
            {
                "schema": "swarmdeck.pose-snapshot.v1",
                "solution": solution.canonical_dict(),
            }
        )
    )
    snapshot = PoseSnapshot.from_file(snapshot_path)
    capture = tmp_path / "capture"
    capture.mkdir()
    (capture / "capture_manifest.json").write_text(
        json.dumps(
            {"robot_id": "robot-a", "session_id": session, "capture_id": "capture-a"}
        )
    )
    np.savez_compressed(
        capture / "00000000.npz",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        K=np.array([[2.0, 0, 1.0], [0, 2.0, 1.0], [0, 0, 1.0]]),
        T_world_camera=np.eye(4),
        T_keyframe_camera=np.array(
            [[1, 0, 0, 2], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float
        ),
        keyframe_id=keyframe.stable_id,
        stamp=1.0,
    )
    manifest = InputManifest.from_capture(capture, pose_snapshot=snapshot_path)
    assert manifest.pose_revision.snapshot_digest == snapshot.digest
    assert manifest.publication_source()["frame"] == "component"
    snapshot_path.write_text(snapshot_path.read_text() + "\n")
    changed = InputManifest.from_capture(capture, pose_snapshot=snapshot_path)
    assert changed.fingerprint != manifest.fingerprint
    snapshot_path.write_text(
        json.dumps(
            {
                "schema": "swarmdeck.pose-snapshot.v1",
                "solution": solution.canonical_dict(),
            }
        )
    )
    output = tmp_path / "colmap"
    export_colmap(capture, output, stride=1, max_points=10, pose_snapshot=snapshot_path)
    image_record = struct.unpack_from(
        "<I7dI", (output / "sparse" / "0" / "images.bin").read_bytes(), 8
    )
    assert image_record[5] == pytest.approx(-12.0)


def test_dynamic_capture_association_ignores_future_and_rejects_stale_keyframes():
    records = [
        {"keyframe_id": "r/s/0", "stamp_ns": 100},
        {"keyframe_id": "r/s/1", "stamp_ns": 300},
    ]
    assert associate_preceding_keyframe(records, 250, 1e-6)["keyframe_id"] == "r/s/0"
    with pytest.raises(ValueError, match="older"):
        associate_preceding_keyframe(records, 2_000, 1e-6)


def test_native_umami_fixture_writes_small_calibrated_colmap_dataset(tmp_path):
    dataset, config = make_fixture(tmp_path / "native-fixture")
    assert config.read_text().find("Optimization.max_num_iterations: 3") >= 0
    assert (dataset / "images" / "00000001.png").is_file()
    cameras = (dataset / "sparse" / "0" / "cameras.bin").read_bytes()
    images = (dataset / "sparse" / "0" / "images.bin").read_bytes()
    points = (dataset / "sparse" / "0" / "points3D.bin").read_bytes()
    assert struct.unpack_from("<Q", cameras)[0] == 3
    assert struct.unpack_from("<Q", images)[0] == 3
    assert struct.unpack_from("<Q", points)[0] > 0
    session = json.loads((dataset / "swarmdeck.json").read_text())["capture"][
        "session_id"
    ]
    assert str(uuid.UUID(session)) == session


def test_bridge_source_contract_has_dynamic_keyframe_and_atomic_solution_outputs():
    source = (
        Path(__file__)
        .parents[2]
        .joinpath("deploy", "autonomy", "cslam_bridge.py")
        .read_text()
    )
    assert 'f"/{self.robot}/keyframes"' in source
    assert '"T_odom_keyframe": self.core.local_poses[keyframe]' in source
    assert '"keyframe_id": keyframe.stable_id' in source
    assert "tuple(self.core.poses)" in source
    assert '"solution": solution.canonical_dict()' in source
    # Atomic, and fenced by the durable map epoch (`_replace_if_current`).
    assert "self._replace_if_current(temporary, self.graph_solution_file)" in source


def test_pose_snapshot_is_copied_into_job_before_training(tmp_path):
    session = str(uuid.uuid4())
    keyframe = KeyframeId("robot-a", session, 0)
    solution = GraphSolution(
        ComponentRevision("component:corrected", 0, 1),
        keyframe,
        (keyframe,),
        {keyframe: np.eye(4)},
    )
    source = tmp_path / "live-graph-solution.json"
    source.write_text(json.dumps({"solution": solution.canonical_dict()}))
    capture = tmp_path / "capture"
    _capture(capture)
    capture_metadata = json.loads((capture / "capture_manifest.json").read_text())
    capture_metadata.update({"robot_id": "robot-a", "session_id": session})
    (capture / "capture_manifest.json").write_text(json.dumps(capture_metadata))
    with np.load(capture / "00000000.npz", allow_pickle=False) as frame:
        frame_values = {key: frame[key] for key in frame.files}
    frame_values["keyframe_id"] = np.asarray(keyframe.stable_id)
    frame_values["T_keyframe_camera"] = np.eye(4)
    np.savez_compressed(capture / "00000000.npz", **frame_values)
    manifest = InputManifest.from_capture(capture, pose_snapshot=source)
    runner_root = tmp_path / "runner"
    runner_root.mkdir()
    runner, _, backend = _runner(runner_root)
    job = runner.submit(
        manifest,
        backend=backend.name,
        artifact_target=tmp_path / "immutable.swgs",
        job_id="immutable-input",
    )
    copied = Path(job.input_manifest.pose_snapshot_copy_path)
    assert copied.is_relative_to(runner.jobs_dir / job.job_id)
    assert copied.read_bytes() == source.read_bytes()
    source.write_text(json.dumps({"changed": True}))
    assert runner.get(job.job_id).input_manifest.pose_snapshot_path == str(
        source.resolve()
    )
    assert runner.get(job.job_id).input_manifest.pose_snapshot_copy_path == str(copied)
    assert not runner._is_current(job)


def test_changed_source_becomes_stale_but_training_uses_immutable_copy(tmp_path):
    session = str(uuid.uuid4())
    keyframe = KeyframeId("robot-a", session, 0)
    solution = GraphSolution(
        ComponentRevision("component:corrected", 0, 1),
        keyframe,
        (keyframe,),
        {keyframe: np.eye(4)},
    )
    source = tmp_path / "live-graph-solution.json"
    source.write_text(json.dumps({"solution": solution.canonical_dict()}))
    capture = tmp_path / "capture"
    _capture(capture)
    capture_metadata = json.loads((capture / "capture_manifest.json").read_text())
    capture_metadata.update({"robot_id": "robot-a", "session_id": session})
    (capture / "capture_manifest.json").write_text(json.dumps(capture_metadata))
    with np.load(capture / "00000000.npz", allow_pickle=False) as frame:
        frame_values = {key: frame[key] for key in frame.files}
    frame_values["keyframe_id"] = np.asarray(keyframe.stable_id)
    frame_values["T_keyframe_camera"] = np.eye(4)
    np.savez_compressed(capture / "00000000.npz", **frame_values)
    manifest = InputManifest.from_capture(capture, pose_snapshot=source)
    runner_root = tmp_path / "runner"
    runner_root.mkdir()
    runner, _, backend = _runner(runner_root, sleep_seconds=0)
    job = runner.submit(
        manifest,
        backend=backend.name,
        artifact_target=tmp_path / "model.swgs",
        job_id="source-changes-after-submit",
    )
    copied = Path(job.input_manifest.pose_snapshot_copy_path)
    source.write_text(json.dumps({"changed": True}))

    result = runner.run(job.job_id)

    assert result.state is JobState.STALE
    commands = (runner._workdir(job.job_id) / "commands.log").read_text()
    assert str(copied) in commands
    assert str(source.resolve()) not in commands


def test_stale_result_cannot_replace_new_pose_or_input_revision(tmp_path):
    runner, capture, backend = _runner(tmp_path)
    target = tmp_path / "model.swgs"
    job = runner.submit(
        capture,
        backend=backend.name,
        artifact_target=target,
        budget=ResourceBudget(max_frames=2, max_gaussians=10),
    )
    assert runner.run(job.job_id, publish=False).state is JobState.READY
    with (capture / "00000000.npz").open("ab") as stream:
        stream.write(b"newer-pose-snapshot")

    with pytest.raises(StaleResult):
        runner.export(job.job_id)
    assert runner.status(job.job_id)["state"] == JobState.STALE.value
    assert not target.exists()


def test_latest_job_wins_shared_publication_target(tmp_path):
    runner, capture, backend = _runner(tmp_path)
    target = tmp_path / "model.swgs"
    first = runner.submit(capture, backend=backend.name, artifact_target=target)
    assert runner.run(first.job_id, publish=False).state is JobState.READY
    second = runner.submit(capture, backend=backend.name, artifact_target=target)
    assert runner.run(second.job_id, publish=False).state is JobState.READY

    with pytest.raises(StaleResult):
        runner.export(first.job_id)
    assert runner.export(second.job_id).state is JobState.READY
    pointer = json.loads(target.with_name("model.swgs.manifest.json").read_text())
    assert pointer["job_id"] == second.job_id
    assert Path(pointer["artifact"]).is_file()


def test_cancel_terminates_inert_subprocess_and_does_not_publish(tmp_path):
    runner, capture, backend = _runner(tmp_path, sleep_seconds=3)
    target = tmp_path / "model.swgs"
    job = runner.submit(capture, backend=backend.name, artifact_target=target)
    thread = runner.run_async(job.job_id)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if runner.status(job.job_id)["state"] == JobState.TRAINING.value:
            break
        time.sleep(0.02)
    runner.cancel(job.job_id)
    thread.join(timeout=5)

    assert not thread.is_alive()
    assert runner.status(job.job_id)["state"] == JobState.CANCELED.value
    assert not target.exists()


def test_separate_cli_cancel_and_restart_do_not_reset_live_worker(tmp_path):
    runner, capture, backend = _runner(tmp_path, sleep_seconds=3)
    target = tmp_path / "model.swgs"
    job = runner.submit(capture, backend=backend.name, artifact_target=target)
    thread = runner.run_async(job.job_id)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if runner.status(job.job_id)["state"] == JobState.TRAINING.value:
            break
        time.sleep(0.02)
    restarted = DurableJobRunner(tmp_path / "jobs", [backend])
    assert restarted.status(job.job_id)["state"] == JobState.TRAINING.value

    subprocess.run(
        [
            sys.executable,
            "-m",
            "autonomy.reconstruction",
            "cancel",
            "--store",
            str(tmp_path / "jobs"),
            job.job_id,
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert runner.status(job.job_id)["state"] == JobState.CANCELED.value


def test_frame_budget_rejects_before_private_trainer_runs(tmp_path):
    runner, capture, backend = _runner(tmp_path)
    with pytest.raises(Exception, match="frames"):
        runner.submit(
            capture,
            backend=backend.name,
            budget=ResourceBudget(max_frames=0),
        )


def test_budget_exception_kills_trainer_process_group(tmp_path):
    child_pid_path = tmp_path / "child.pid"
    capture = tmp_path / "capture"
    _capture(capture)
    umami = _fake_umami(
        tmp_path / "umami", sleep_seconds=5, child_pid_path=child_pid_path
    )
    config = tmp_path / "umami.yaml"
    config.write_text("fixed_pose: true\\n")
    backend = UmamiBackend(umami_root=umami, config=config)
    runner = DurableJobRunner(tmp_path / "jobs", [backend])
    job = runner.submit(
        capture,
        backend=backend.name,
        budget=ResourceBudget(max_frames=2, max_gaussians=10, max_training_seconds=2.0),
    )

    result = runner.run(job.job_id, publish=False)
    assert result.state is JobState.FAILED
    child_pid = int(child_pid_path.read_text())
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            os.kill(child_pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.02)
    else:
        pytest.fail("trainer child process survived the resource budget failure")
