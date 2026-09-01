"""The HTTP process, not just CollaborativeBackend: blobs in, merge out."""

from __future__ import annotations

import time

import pytest
import synthetic
from fastapi.testclient import TestClient

from swarmdeck_protocol import decode_keyframe, encode_keyframe
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


def _post_fleet(client, segments) -> None:
    keyed = [(kf, robot.session) for robot in segments for kf in robot.keyframes]
    keyed.sort(key=lambda item: (item[0].stamp, item[0].id.robot_id, item[0].id.seq))
    for keyframe, session in keyed:
        blob = encode_keyframe(
            robot_id=keyframe.id.robot_id,
            seq=keyframe.id.seq,
            stamp=keyframe.stamp,
            points=keyframe.points,
            t_odom_base=quat_xyz_from_se3(keyframe.t_odom_base),
            session=session,
        )
        assert client.post("/keyframe", content=blob).status_code == 200


def _wait_for(client, predicate, timeout_s: float = 30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = client.get("/status").json()
        if predicate(status):
            return status
        time.sleep(0.1)
    raise AssertionError(f"timed out; last status: {client.get('/status').json()}")


def _wait_for_ingest(client, segments):
    """Every posted keyframe is in, and one solve has run over them.

    Selection acts on what the back-end holds, so a test that selects while
    the worker is still draining the queue is asserting about a different
    session than the one it posted.
    """
    total = sum(len(robot.keyframes) for robot in segments)
    return _wait_for(
        client,
        lambda s: s["keyframes"] == total and s["has_snapshot"] and not s["dirty"],
    )


def test_status_lists_a_restarted_robots_trajectories(slam_client) -> None:
    """A reboot is invisible in every other field on this endpoint: the robot
    count does not change, the keyframe count only goes up, and nothing says
    the second half is in a different frame from the first."""
    _, segments = synthetic.restarted_robot()
    _post_fleet(slam_client, segments)

    status = _wait_for_ingest(slam_client, segments)
    rows = {row["id"]: row for row in status["trajectories"]}
    assert set(rows) == {"alpha", "alpha@boot-2"}
    for robot in segments:
        row = rows[str(robot.trajectory_id)]
        assert row["robot_id"] == "alpha"
        assert row["session"] == robot.session
        assert row["first_seq"] == robot.keyframes[0].id.seq
        assert row["included"] is True


def test_a_trajectory_can_be_excluded_and_put_back(slam_client) -> None:
    """Excluded is a selection, not a delete: the segment stays listed, stops
    contributing, and comes back with everything it had."""
    _, segments = synthetic.restarted_robot()
    _post_fleet(slam_client, segments)
    _wait_for_ingest(slam_client, segments)

    response = slam_client.post(
        "/trajectories/select", json={"exclude": ["alpha@boot-2"]}
    )
    assert response.status_code == 200, response.text
    assert response.json()["changed"] == 1

    # The flag flips at once; the re-solve that acts on it belongs to the
    # worker, which is where the graph is allowed to be rebuilt.
    status = _wait_for(
        slam_client,
        lambda s: s["keyframe_counts"].get("alpha") == len(segments[0].keyframes),
    )
    excluded = next(r for r in status["trajectories"] if r["id"] == "alpha@boot-2")
    assert excluded["included"] is False
    assert excluded["keyframes"] == len(segments[1].keyframes), "excluded means stored"

    assert (
        slam_client.post(
            "/trajectories/select", json={"include": ["alpha@boot-2"]}
        ).status_code
        == 200
    )
    status = _wait_for(
        slam_client,
        lambda s: s["keyframe_counts"].get("alpha")
        == sum(len(robot.keyframes) for robot in segments),
    )
    assert all(row["included"] for row in status["trajectories"])


def test_selecting_a_subset_rebuilds_the_map_from_it(slam_client) -> None:
    """'only' is the recreate-the-map path."""
    _, segments = synthetic.restarted_robot()
    _post_fleet(slam_client, segments)
    _wait_for_ingest(slam_client, segments)

    response = slam_client.post("/trajectories/select", json={"only": ["alpha"]})
    assert response.status_code == 200, response.text
    status = _wait_for(
        slam_client,
        lambda s: s["keyframe_counts"].get("alpha") == len(segments[0].keyframes),
    )
    included = [row["id"] for row in status["trajectories"] if row["included"]]
    assert included == ["alpha"]


def test_a_malformed_selection_is_refused(slam_client) -> None:
    assert slam_client.post("/trajectories/select", json={}).status_code == 400
    assert (
        slam_client.post("/trajectories/select", json={"exclude": [3]}).status_code
        == 400
    )
    assert (
        slam_client.post(
            "/trajectories/select", json={"exclude": ["@nobody"]}
        ).status_code
        == 400
    )


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
        assert (
            keyframes / f"{i:06d}.kf"
        ).read_bytes() == b"old", "clobbered an existing capture"


def test_capture_starts_at_zero_in_a_fresh_directory(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(svc, "CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr(svc, "_capture_seq", -1)
    monkeypatch.setattr(svc, "_capture_failed", False)
    svc._capture(b"first")
    assert (tmp_path / "keyframes" / "000000.kf").read_bytes() == b"first"


def test_capture_restore_rebuilds_without_recapturing(tmp_path, monkeypatch) -> None:
    """A planned process replacement restores each durable blob exactly once."""
    _, fleet = synthetic.two_robot_fleet()
    keyframe = fleet[0].keyframes[0]
    blob = encode_keyframe(
        robot_id=keyframe.id.robot_id,
        seq=keyframe.id.seq,
        stamp=keyframe.stamp,
        points=keyframe.points,
        t_odom_base=quat_xyz_from_se3(keyframe.t_odom_base),
        session="restore-test",
    )
    keyframes = tmp_path / "keyframes"
    keyframes.mkdir()
    (keyframes / "000000.kf").write_bytes(blob)

    restored_backend = type(svc.backend)(registration_mode="graph")
    monkeypatch.setattr(svc, "backend", restored_backend)
    monkeypatch.setattr(svc, "CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr(svc, "RESTORE_CAPTURE", True)
    monkeypatch.setattr(svc, "_ingested", 0)
    monkeypatch.setattr(svc, "_restored", 0)

    assert svc._restore_capture() == 1
    assert len(restored_backend) == 1
    assert svc._ingested == 1
    assert svc._restored == 1
    assert sorted(p.name for p in keyframes.iterdir()) == ["000000.kf"]

    # A second call sees the same keyframe id and remains idempotent.
    assert svc._restore_capture() == 0
    assert len(restored_backend) == 1
    assert svc._ingested == 1


def test_delete_robot_archives_only_its_restorable_keyframes(
    tmp_path, monkeypatch
) -> None:
    _, fleet = synthetic.two_robot_fleet()
    monkeypatch.setattr(svc, "CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr(svc, "_capture_seq", -1)
    monkeypatch.setattr(svc, "_capture_failed", False)

    for robot in fleet:
        keyframe = robot.keyframes[0]
        svc._capture(
            encode_keyframe(
                robot_id=keyframe.id.robot_id,
                seq=keyframe.id.seq,
                stamp=keyframe.stamp,
                points=keyframe.points,
                t_odom_base=quat_xyz_from_se3(keyframe.t_odom_base),
                session="delete-test",
            )
        )

    target = fleet[0].robot_id
    response = svc.delete_robot_keyframes(target)
    assert response["ok"] is True
    assert response["archived_keyframes"] == 1
    remaining = [
        decode_keyframe(path.read_bytes()).robot_id
        for path in (tmp_path / "keyframes").glob("*.kf")
    ]
    assert remaining == [fleet[1].robot_id]
    archived = list((tmp_path / "discarded").glob("*/*.kf"))
    assert len(archived) == 1
    assert decode_keyframe(archived[0].read_bytes()).robot_id == target
    commands, _ = svc._take_controls()
    assert commands[-1] == ("delete_robot", target)


def test_stale_optimization_generation_cannot_publish(monkeypatch) -> None:
    _, fleet = synthetic.two_robot_fleet()
    backend = type(svc.backend)(registration_mode="graph")
    keyframe = fleet[0].keyframes[0]
    backend.ingest_keyframe(keyframe)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None

    monkeypatch.setattr(svc, "SERVER_URL", "http://must-not-be-called")
    opened = []
    monkeypatch.setattr(svc.urllib.request, "urlopen", lambda *a, **k: opened.append(a))
    current = svc._current_generation()
    svc._enqueue_control("reset")

    svc._publish_snapshot(snapshot, current)

    assert opened == []
    assert svc._last_snapshot is None
    commands, _ = svc._take_controls()
    assert commands[-1] == ("reset", None)


def test_config_endpoint_clamps_and_does_not_switch_mode(slam_client) -> None:
    before = slam_client.get("/config").json()
    assert before["ok"] is True
    assert "min_support" in before["settings"]
    applied = slam_client.put("/config", json={"min_support": 1, "registration_mode": "graph"})
    assert applied.status_code == 200
    body = applied.json()
    assert body["settings"]["min_support"] == 2
    assert body["settings"]["registration_mode"] == before["settings"]["registration_mode"]
