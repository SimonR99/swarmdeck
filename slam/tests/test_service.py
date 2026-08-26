"""The HTTP process, not just CollaborativeBackend: blobs in, merge out."""

from __future__ import annotations

import time

import pytest
import synthetic
from fastapi.testclient import TestClient

from swarmdeck_protocol import encode_keyframe
from swarmdeck_slam.types import quat_xyz_from_se3
import swarmdeck_slam.service as svc


@pytest.fixture
def slam_client(monkeypatch):
    monkeypatch.setattr(svc, "SERVER_URL", "")
    monkeypatch.setattr(svc, "OPTIMIZE_EVERY_N", 5)
    monkeypatch.setattr(svc, "OPTIMIZE_EVERY_S", 0.05)
    with TestClient(svc.app) as client:
        client.post("/reset")
        yield client
        client.post("/reset")


def test_http_service_merges_a_two_robot_fleet(slam_client) -> None:
    _, fleet = synthetic.two_robot_fleet()
    keyed = [kf for robot in fleet for kf in robot.keyframes]
    keyed.sort(key=lambda kf: (kf.stamp, kf.id.robot_id, kf.id.seq))
    for kf in keyed:
        blob = encode_keyframe(
            robot_id=kf.id.robot_id,
            seq=kf.id.seq,
            stamp=kf.stamp,
            points=kf.points,
            t_odom_base=quat_xyz_from_se3(kf.t_odom_base),
        )
        posted = slam_client.post("/keyframe", content=blob)
        assert posted.status_code == 200

    deadline = time.monotonic() + 30.0
    snapshot = None
    while time.monotonic() < deadline:
        status = slam_client.get("/status").json()
        if status.get("has_snapshot") and any(
            len(c["robots"]) >= 2 for c in status.get("components") or []
        ):
            snapshot = status
            break
        time.sleep(0.1)
    assert snapshot is not None, slam_client.get("/status").json()
    robots = {r for c in snapshot["components"] for r in c["robots"]}
    assert robots == {robot.robot_id for robot in fleet}
    assert snapshot["ingested"] >= 10


def test_capture_resumes_instead_of_overwriting(tmp_path, monkeypatch) -> None:
    """A restarted service must not clobber the dataset it was collecting.

    Filenames are the arrival index alone and the counter lives in module
    state, so a restart used to begin again at 000000 and overwrite an existing
    capture from the beginning -- silently, with no error, destroying exactly
    the run someone was collecting.
    """
    keyframes = tmp_path / "keyframes"
    keyframes.mkdir()
    for i in (0, 1, 2, 7):  # 7, not 3: gaps must not confuse the resume
        (keyframes / f"{i:06d}.kf").write_bytes(b"old")

    monkeypatch.setattr(svc, "CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr(svc, "_capture_seq", -1)
    monkeypatch.setattr(svc, "_capture_failed", False)

    svc._capture(b"new-a")
    svc._capture(b"new-b")

    assert (keyframes / "000008.kf").read_bytes() == b"new-a"
    assert (keyframes / "000009.kf").read_bytes() == b"new-b"
    for i in (0, 1, 2, 7):
        assert (keyframes / f"{i:06d}.kf").read_bytes() == b"old", "clobbered an existing capture"


def test_capture_starts_at_zero_in_a_fresh_directory(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(svc, "CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr(svc, "_capture_seq", -1)
    monkeypatch.setattr(svc, "_capture_failed", False)
    svc._capture(b"first")
    assert (tmp_path / "keyframes" / "000000.kf").read_bytes() == b"first"
