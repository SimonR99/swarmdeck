"""The simulation reset: what it calls, in what order, and what it refuses to do.

Everything here runs without ROS or Gazebo. What cannot be checked that way — that
Gazebo really does return the models to spawn, that slam_toolbox really does drop
its graph — is a live test, and these cover the parts that decide whether the live
test can succeed at all: the request shape, the ordering, and the caching rules
that stop a cleared map coming straight back.
"""

from __future__ import annotations

import math
import subprocess
import threading
import types
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def sim(sim_module):
    """adapter_sim with its fleet-wide reset state returned to a known start."""
    sim_module._world_name = None
    sim_module._world_reset_at = 0.0
    sim_module.CSLAM_GRID.clear()
    sim_module.SLAM_GRAPHS.clear()
    return sim_module


GZ_SERVICE_LISTING = """
/gazebo/resource_paths/get
/world/swarmdeck_indoor/control
/world/swarmdeck_indoor/control/state
/world/swarmdeck_indoor/playback/control
/world/swarmdeck_indoor/scene/info
"""


def fake_run(recorder, stdout="data: true\n", stderr=""):
    def run(cmd, **kwargs):
        recorder.append(cmd)
        if cmd[:3] == ["gz", "service", "-l"]:
            return types.SimpleNamespace(stdout=GZ_SERVICE_LISTING, stderr="", returncode=0)
        return types.SimpleNamespace(stdout=stdout, stderr=stderr, returncode=0)

    return run


# ------------------------------------------------------------ world discovery


def test_world_name_ignores_the_playback_control_service(sim, monkeypatch):
    """/world/<name>/playback/control also ends in /control and is not the world."""
    monkeypatch.setattr(subprocess, "run", fake_run([]))
    assert sim._gz_world_name(MagicMock()) == "swarmdeck_indoor"


def test_world_name_is_absent_rather_than_guessed(sim, monkeypatch):
    monkeypatch.setattr(subprocess, "run", lambda cmd, **kw: types.SimpleNamespace(
        stdout="/gazebo/resource_paths/get\n", stderr="", returncode=0
    ))
    assert sim._gz_world_name(MagicMock()) is None


# ---------------------------------------------------------------- world reset


def test_world_reset_asks_for_models_only_and_never_the_clock(sim, monkeypatch):
    """`all` would reset simulation time, and every node here runs on sim time.

    A /clock that jumps backwards invalidates the tf2 buffers, the Nav2 lifecycle
    timers and SLAM's scan queue at once. Only the poses are meant to go back.
    """
    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))

    assert sim.reset_world(MagicMock()) is True

    request = next(c for c in calls if "--req" in c)
    payload = request[request.index("--req") + 1]
    assert "model_only: true" in payload
    assert "all" not in payload
    assert "time_only" not in payload
    assert "/world/swarmdeck_indoor/control" in request


def test_world_reset_is_coalesced_across_the_fleet(sim, monkeypatch):
    """The protocol addresses one robot at a time; there is only one world."""
    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))

    assert all(sim.reset_world(MagicMock()) for _ in range(4))

    resets = [c for c in calls if "--req" in c]
    assert len(resets) == 1, f"world reset issued {len(resets)} times for one fleet"


def test_world_reset_detects_a_timeout_that_exits_zero(sim, monkeypatch):
    """`gz service` exits 0 whether the world answered or the call timed out.

    Verified against the running stack: a good call prints `data: true`, a call to
    a world that does not exist prints `Service call timed out` — and both exit 0.
    The reply body is the only evidence, so trusting the exit code would report
    every failed reset as a success.
    """
    monkeypatch.setattr(
        subprocess, "run", fake_run([], stdout="Service call timed out\n")
    )
    assert sim.reset_world(MagicMock()) is False


def test_a_failed_world_reset_is_not_remembered_as_done(sim, monkeypatch):
    """Otherwise the coalescing window would suppress the retry."""
    monkeypatch.setattr(
        subprocess, "run", fake_run([], stdout="Service call timed out\n")
    )
    assert sim.reset_world(MagicMock()) is False

    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))
    assert sim.reset_world(MagicMock()) is True
    assert any("--req" in c for c in calls)


def test_world_reset_drops_the_collaborative_grid(sim, monkeypatch):
    """It describes the world that was just reset."""
    monkeypatch.setattr(subprocess, "run", fake_run([]))
    sim.CSLAM_GRID.update({"grid": object(), "dirty": True})
    sim.SLAM_GRAPHS["robot_0"] = {"keyframes": 40}

    sim.reset_world(MagicMock())

    assert sim.CSLAM_GRID == {}
    assert sim.SLAM_GRAPHS == {}


# --------------------------------------------------------------- robot reset


def make_bridge(sim):
    bridge = sim.RobotBridge.__new__(sim.RobotBridge)
    bridge.node = MagicMock()
    bridge.id = "robot_0"
    bridge.t0 = 0.0
    bridge.pub_cmd = MagicMock()
    bridge._goal_handle = None
    bridge._goal_generation = 0
    bridge._last_drive_at = 12.0
    bridge._upload_lock = threading.Lock()
    bridge._service_clients = {}
    bridge._reset_report = None
    bridge._start_pose = {"x": -9.0, "y": 0.0, "yaw": 0.0}
    bridge.http_url = "http://server:8080"
    # Platform identity, normally resolved from the fleet config in main().
    bridge.platform = "bunker"
    bridge.robot_type = "agilex_bunker"
    bridge.footprint_radius = 0.643
    bridge.spawn_z = 0.230
    bridge.goal = {"x": 3.0, "y": 1.0}
    bridge.planned_path = [{"x": 1.0, "y": 1.0}]
    bridge.nav_status = "active"
    bridge.mode = "nav"
    bridge.grid = object()
    bridge._grid_dirty = True
    bridge._cloud = object()
    bridge._cloud_dirty = True
    bridge._camera_frame = object()
    bridge._camera_dirty = True
    bridge._detections = [{"id": "duck_0"}]
    return bridge


def instrument(sim, bridge, monkeypatch, order):
    monkeypatch.setattr(sim, "WORLD_SETTLE_S", 0.0)
    monkeypatch.setattr(
        sim, "reset_world", lambda logger: order.append("world") is None or True
    )
    monkeypatch.setattr(
        bridge, "_reset_pose", lambda: order.append("pose") is None or True
    )
    monkeypatch.setattr(
        bridge, "_reset_odometry", lambda: order.append("odometry") is None or True
    )
    monkeypatch.setattr(
        bridge, "_reset_slam", lambda: order.append("slam") is None or True
    )
    monkeypatch.setattr(
        bridge, "_clear_costmaps", lambda: order.append("costmaps") is None or True
    )


def test_reset_moves_the_robot_before_re_zeroing_what_measures_its_movement(
    sim, monkeypatch
):
    """Ordering is the design.

    A robot that re-zeroed its filter and restarted SLAM before the teleport
    moved it would read its own displacement as travel, and start the new map
    with a metre of motion in it that never happened.
    """
    bridge = make_bridge(sim)
    order: list[str] = []
    instrument(sim, bridge, monkeypatch, order)

    steps = bridge.reset()

    assert order == ["world", "pose", "odometry", "slam", "costmaps"]
    assert steps == {
        "world": True, "pose": True, "odometry": True,
        "slam": True, "costmaps": True,
    }


def test_reset_stops_the_robot_and_drops_every_cached_upload(sim, monkeypatch):
    """Anything still cached describes the world before the reset.

    Uploading it after the backend clears would put the old map straight back,
    which is the whole reason the backend waits for `reset_done`.
    """
    bridge = make_bridge(sim)
    instrument(sim, bridge, monkeypatch, [])

    bridge.reset()

    bridge.pub_cmd.publish.assert_called()
    assert bridge.goal is None
    assert bridge.planned_path == []
    assert (bridge.nav_status, bridge.mode) == ("idle", "idle")
    assert bridge.grid is None and bridge._grid_dirty is False
    assert bridge._cloud is None and bridge._cloud_dirty is False
    assert bridge._camera_frame is None and bridge._camera_dirty is False
    assert bridge._detections is None


def test_the_robot_is_moved_by_set_pose_not_by_the_world_reset(sim, monkeypatch):
    """Regression test for a bug found only by running it against Gazebo.

    A world reset restores what the world SDF declared. The fleet is created in
    an already-running world by spawn_fleet.py, so `reset: {model_only: true}`
    answers `data: true` and leaves every robot exactly where it was — measured
    live: robot_0 sat at (-2.48, 3.45) before and (-2.51, 3.03) after, which is
    continued driving, not a teleport to its spawn pose at (-9, 0).
    """
    bridge = make_bridge(sim)
    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))

    assert bridge._reset_pose() is True

    request = next(c for c in calls if "--req" in c)
    payload = request[request.index("--req") + 1]
    assert "/world/swarmdeck_indoor/set_pose" in request
    assert "gz.msgs.Pose" in request
    assert 'name: "robot_0"' in payload
    assert "x: -9.0" in payload and "y: 0.0" in payload
    # Spawn height is per platform. A single shared constant would bury a Spot
    # (hidden drive wheels 0.50 m below the body) or hang a Scout Mini in the air.
    assert "z: 0.23" in payload


def test_set_pose_encodes_yaw_as_a_quaternion(sim, monkeypatch):
    bridge = make_bridge(sim)
    bridge._start_pose = {"x": 3.0, "y": 0.0, "yaw": math.pi}
    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))

    assert bridge._reset_pose() is True

    payload = next(c for c in calls if "--req" in c)[-1]
    # yaw = pi -> (z, w) = (1, 0)
    assert "z: 1.000000000" in payload
    assert "w: 0.000000000" in payload


def test_a_robot_with_no_configured_start_pose_is_left_where_it_stands(sim, monkeypatch):
    """Guessing one would teleport it somewhere it was never spawned."""
    bridge = make_bridge(sim)
    bridge._start_pose = None
    monkeypatch.setattr(
        sim.urllib.request, "urlopen", lambda *a, **kw: (_ for _ in ()).throw(OSError("no backend"))
    )
    calls: list[list[str]] = []
    monkeypatch.setattr(subprocess, "run", fake_run(calls))

    assert bridge._reset_pose() is False
    assert not any("set_pose" in " ".join(c) for c in calls)


def test_reset_reports_which_step_failed_rather_than_just_false(sim, monkeypatch):
    """A world that moved and a SLAM that did not is a recognisable failure."""
    bridge = make_bridge(sim)
    monkeypatch.setattr(sim, "WORLD_SETTLE_S", 0.0)
    monkeypatch.setattr(sim, "reset_world", lambda logger: True)
    monkeypatch.setattr(bridge, "_reset_pose", lambda: True)
    monkeypatch.setattr(bridge, "_reset_odometry", lambda: True)
    monkeypatch.setattr(bridge, "_reset_slam", lambda: False)
    monkeypatch.setattr(bridge, "_clear_costmaps", lambda: True)

    steps = bridge.reset()

    assert steps["slam"] is False
    report = bridge.take_reset_report()
    assert report["type"] == "reset_done"
    assert report["robot_id"] == "robot_0"
    assert report["ok"] is False
    assert report["steps"]["slam"] is False
    assert bridge.take_reset_report() is None, "a report must only be sent once"


def test_the_reset_report_goes_out_through_the_tx_loop(sim, monkeypatch):
    """Not sent from the reset's own thread — that would be two writers on one
    websocket. take_reset_report() hands it to the loop that owns the socket."""
    bridge = make_bridge(sim)
    instrument(sim, bridge, monkeypatch, [])
    assert bridge.take_reset_report() is None

    bridge.reset()

    assert bridge.take_reset_report()["ok"] is True


def test_uploads_are_skipped_during_a_reset_rather_than_queued_behind_it(sim):
    """Blocking here would stall the 5 Hz state stream.

    The backend calls a robot offline after four seconds of silence, so an upload
    that waited on the reset lock would make every reset look like a fleet-wide
    disconnect. A skipped upload costs one 2 s cycle.
    """
    bridge = make_bridge(sim)
    uploaded = []
    bridge._upload_map_locked = lambda: uploaded.append("map")

    bridge._upload_lock.acquire()  # stand in for a reset in progress
    try:
        bridge.upload_map()
    finally:
        bridge._upload_lock.release()
    assert uploaded == [], "upload ran while a reset held the lock"

    bridge.upload_map()
    assert uploaded == ["map"], "upload did not resume after the reset"


def test_reset_waits_for_an_upload_already_in_flight(sim, monkeypatch):
    """The other half of the ordering: a reset must not clear the cache out from
    under an upload that is mid-request, or that upload sends a grid the backend
    will keep after it clears."""
    bridge = make_bridge(sim)
    instrument(sim, bridge, monkeypatch, [])
    released = []

    def slow_upload():
        released.append("started")
        # reset() must be blocked on the lock for as long as this holds it.
        assert bridge.grid is not None, "cache cleared mid-upload"

    bridge._upload_map_locked = slow_upload
    bridge.upload_map()
    assert released == ["started"]

    bridge.reset()
    assert bridge.grid is None


# ------------------------------------------------------------ service calling


def test_a_service_that_never_answers_is_a_failure_not_a_hang(sim, monkeypatch):
    bridge = make_bridge(sim)
    monkeypatch.setattr(sim, "SERVICE_TIMEOUT_S", 0.05)

    client = MagicMock()
    client.wait_for_service.return_value = True
    future = MagicMock()
    future.done.return_value = False
    client.call_async.return_value = future
    bridge.node.create_client.return_value = client

    assert bridge._call("/robot_0/set_pose", MagicMock(), MagicMock(), timeout_s=0.05) is False
    future.cancel.assert_called_once()


def test_a_missing_service_is_reported_rather_than_waited_on(sim):
    bridge = make_bridge(sim)
    client = MagicMock()
    client.wait_for_service.return_value = False
    bridge.node.create_client.return_value = client

    assert bridge._call("/robot_0/nope", MagicMock(), MagicMock(), timeout_s=0.01) is False
    client.call_async.assert_not_called()


def test_slam_reset_picks_the_back_end_that_is_actually_running(sim):
    """`slam_backend:=rtabmap` swaps the SLAM node out entirely."""
    bridge = make_bridge(sim)
    called: list[str] = []
    bridge._call = lambda name, srv_type, request, **kw: called.append(name) is None or True
    bridge.node.get_service_names_and_types.return_value = [
        ("/robot_0/rtabmap/reset", ["std_srvs/srv/Empty"]),
        ("/robot_0/odom", ["nav_msgs/msg/Odometry"]),
    ]

    assert bridge._reset_slam() is True
    assert called == ["/robot_0/rtabmap/reset"]


def test_slam_reset_prefers_slam_toolbox_when_both_are_advertised(sim):
    bridge = make_bridge(sim)
    called: list[str] = []
    bridge._call = lambda name, srv_type, request, **kw: called.append(name) is None or True
    bridge.node.get_service_names_and_types.return_value = [
        ("/robot_0/rtabmap/reset", ["std_srvs/srv/Empty"]),
        ("/robot_0/slam_toolbox/reset", ["slam_toolbox/srv/Reset"]),
    ]

    assert bridge._reset_slam() is True
    assert called == ["/robot_0/slam_toolbox/reset"]


def test_no_slam_reset_service_is_a_reported_failure(sim):
    """Silence here would mean a fleet back at spawn drawing its old map."""
    bridge = make_bridge(sim)
    bridge.node.get_service_names_and_types.return_value = []

    assert bridge._reset_slam() is False
    bridge.node.get_logger.return_value.warn.assert_called()


def test_odometry_reset_targets_the_robot_s_own_odom_frame(sim):
    bridge = make_bridge(sim)
    recorded = {}

    def record(name, srv_type, request, **kw):
        recorded["name"] = name
        recorded["frame"] = request.pose.header.frame_id
        return True

    bridge._call = record
    assert bridge._reset_odometry() is True
    assert recorded["name"] == "/robot_0/set_pose"
    assert recorded["frame"] == "robot_0/odom"
