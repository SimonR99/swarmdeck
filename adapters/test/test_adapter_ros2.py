"""Hardware adapter tests, with ROS stubbed out.

`adapter_ros2` imports the whole ROS stack at module scope, so this stubs it the
same way `test_adapter_sim_pose.py` does. What is testable without a robot is
exactly the part most likely to be wrong on one: capability advertisement,
config merging, battery normalisation, and the deadman.

What these CANNOT test is whether the topic names, QoS choices and frame names
are right for any particular robot. That needs hardware — see
docs/hardware-bringup.md.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

_STUBBED = [
    "rclpy", "rclpy.action", "rclpy.node", "rclpy.qos", "rclpy.time",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "nav2_msgs", "nav2_msgs.action",
    "sensor_msgs", "sensor_msgs.msg",
    "action_msgs", "action_msgs.msg",
    "tf2_ros", "websockets", "cv2",
    "std_srvs", "std_srvs.srv",
    "spot_msgs", "spot_msgs.action",
]


@pytest.fixture(scope="module")
def mod():
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    for name in _STUBBED:
        sys.modules[name] = MagicMock()
    sys.path.insert(0, str(REPO / "adapters" / "adapter_ros2"))
    try:
        import importlib

        module = importlib.import_module("adapter_ros2")
        yield module
    finally:
        sys.modules.pop("adapter_ros2", None)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def _bridge(mod, cfg_override=None):
    cfg = mod.deep_merge(mod.DEFAULTS, cfg_override or {})
    bridge = mod.HardwareBridge.__new__(mod.HardwareBridge)
    bridge.cfg = cfg
    bridge.id = "r0"
    bridge.map_frame = cfg["map_frame"]
    bridge.base_frame = cfg["base_frame"]
    bridge.node = MagicMock()
    bridge.pub_cmd = MagicMock() if cfg["topics"].get("cmd_vel") else None
    bridge.nav_client = (
        MagicMock()
        if cfg.get("actions", {}).get("navigate_to_pose")
        else None
    )
    bridge.traj_client = (
        MagicMock()
        if cfg.get("actions", {}).get("trajectory")
        else None
    )
    bridge.tf_buffer = MagicMock()
    bridge.mode = "idle"
    bridge.nav_status = "idle"
    bridge.goal = None
    bridge._goal_generation = 0
    bridge._goal_handle = None
    bridge._last_drive_at = 0.0
    # A connected robot, which is what every test that is not about the link
    # itself means to model. The class default is deliberately stale.
    bridge._last_link_at = __import__("time").monotonic()
    bridge._scan_points = None
    # Mirrors __init__: the pose the scan points were captured at.
    bridge._scan_origin = None
    bridge._scan_dirty = False
    bridge._cloud_points = None
    bridge._cloud_dirty = False
    bridge._last_cloud_prepare_at = 0.0
    bridge._body_clients = {}
    return bridge


def test_config_merge_is_deep_not_shallow(mod):
    """Overriding one topic must not delete the others.

    A shallow merge would silently drop `map` and `cmd_vel` when a robot config
    sets only `odom`, disabling capabilities the robot actually has.
    """
    merged = mod.deep_merge(mod.DEFAULTS, {"topics": {"odom": "wheel_odom"}})
    assert merged["topics"]["odom"] == "wheel_odom"
    assert merged["topics"]["map"] == "map"
    assert merged["topics"]["cmd_vel"] == "cmd_vel"


def test_capabilities_reflect_configuration_only(mod):
    """Protocol rule 4: never advertise a capability you cannot honour."""
    full = _bridge(mod, {"topics": {"battery": "battery_state",
                                    "camera_compressed": "cam/compressed"}})
    caps = full.capabilities()
    assert {"navigate", "map", "camera", "battery", "estop"} <= set(caps)

    bare = _bridge(mod, {
        "topics": {"odom": "odom", "map": "", "cmd_vel": "", "battery": "",
                   "camera": "", "camera_compressed": ""},
        "actions": {"navigate_to_pose": ""},
    })
    assert bare.capabilities() == []


def test_body_capability_needs_configured_services(mod):
    """Empty Trigger names must not advertise Claim/Stand the GUI cannot honour."""
    none = _bridge(mod)
    assert "body" not in none.capabilities()
    spot = _bridge(mod, {"services": {
        "claim": "/claim", "release": "/release",
        "sit": "/sit", "stand": "/stand", "power_on": "/power_on",
    }})
    assert "body" in spot.capabilities()


def _trigger_client(order, name):
    client = MagicMock()
    client.wait_for_service.return_value = True

    def call_async(_req):
        order.append(name)
        future = MagicMock()
        future.done.return_value = True
        resp = MagicMock()
        resp.success = True
        resp.message = ""
        future.result.return_value = resp
        return future

    client.call_async.side_effect = call_async
    return client


def test_body_command_stand_powers_on_first(mod):
    """Clearpath /stand fails if the motors are still off."""
    bridge = _bridge(mod, {"services": {
        "claim": "/claim", "release": "/release",
        "sit": "/sit", "stand": "/stand", "power_on": "/power_on",
    }})
    order: list[str] = []
    bridge._body_clients = {
        name: _trigger_client(order, name)
        for name in ("claim", "release", "sit", "stand", "power_on")
    }
    bridge.body_command("stand")
    assert order == ["power_on", "stand"]
    bridge.body_command("claim")
    assert order[-1] == "claim"
    bridge.body_command("not-a-thing")
    assert order[-1] == "claim"


def test_body_command_claim_clears_tablet_keepalive(mod):
    """Tablet Stop leaves a keepalive that blocks /power_on after Claim."""
    names = (
        "claim", "release", "sit", "stand", "power_on",
        "estop_release", "clear_keepalive",
    )
    bridge = _bridge(mod, {"services": {name: f"/{name}" for name in names}})
    order: list[str] = []
    bridge._body_clients = {name: _trigger_client(order, name) for name in names}
    bridge.body_command("claim")
    assert order == ["claim", "estop_release", "clear_keepalive"]
    order.clear()
    bridge.body_command("stand")
    assert order == ["estop_release", "clear_keepalive", "power_on", "stand"]


def _identity_tf():
    rot = type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
    trans = type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
    transform = type("X", (), {"rotation": rot, "translation": trans})()
    return type("TF", (), {"transform": transform})()


def test_trajectory_action_advertises_navigate(mod):
    """Spot has no Nav2 server; /trajectory is what click-to-pose talks to."""
    none = _bridge(mod, {"actions": {"navigate_to_pose": "", "trajectory": ""}})
    assert "navigate" not in none.capabilities()
    spot = _bridge(mod, {
        "topics": {"cmd_vel": ""},
        "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
    })
    assert "navigate" in spot.capabilities()
    assert "estop" not in spot.capabilities()


def test_trajectory_goal_is_transformed_into_body(mod):
    """spot_driver rejects any Trajectory frame_id other than body."""
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.return_value = _identity_tf()
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.5, "y": -2.0, "yaw": 0.0})

    bridge.traj_client.send_goal_async.assert_called_once()
    msg = bridge.traj_client.send_goal_async.call_args[0][0]
    assert msg.target_pose.header.frame_id == "body"
    assert msg.target_pose.pose.position.x == pytest.approx(1.5)
    assert msg.target_pose.pose.position.y == pytest.approx(-2.0)
    assert msg.duration.sec >= 1
    assert bridge.nav_status == "active"
    assert bridge.goal == {"x": 1.5, "y": -2.0}


def test_trajectory_goal_without_tf_is_dropped(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "", "trajectory": "/trajectory"}})
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    bridge.traj_client.server_is_ready.return_value = True

    bridge.navigate_to({"x": 1.0, "y": 1.0})

    bridge.traj_client.send_goal_async.assert_not_called()
    assert bridge.nav_status == "failed"


def test_cancel_trajectory_calls_spot_stop(mod):
    """Clearpath's ROS 2 Trajectory server does not honour cancel/preempt."""
    bridge = _bridge(mod, {
        "actions": {"navigate_to_pose": "", "trajectory": "/trajectory"},
        "services": {"stop": "/stop"},
    })
    order: list[str] = []
    bridge._body_clients = {"stop": _trigger_client(order, "stop")}
    handle = MagicMock()
    bridge._goal_handle = handle
    bridge.nav_status = "active"
    bridge.goal = {"x": 2.0, "y": 2.0}

    bridge.cancel_goal()

    handle.cancel_goal_async.assert_called_once_with()
    assert order == ["stop"]
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None


def test_camera_capability_needs_a_real_topic(mod):
    """An empty topic string must not advertise a camera the GUI will poll."""
    no_cam = _bridge(mod, {"topics": {"camera": "", "camera_compressed": ""}})
    assert "camera" not in no_cam.capabilities()
    raw_only = _bridge(mod, {"topics": {"camera": "image_raw"}})
    assert "camera" in raw_only.capabilities()


def test_battery_normalisation_handles_percent_and_nan(mod):
    """Drivers disagree about 0..1 vs 0..100, and NaN means 'unknown'."""
    bridge = _bridge(mod)

    bridge._on_battery(type("M", (), {"percentage": 0.82})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": 82.0})())
    assert bridge.battery == pytest.approx(0.82)

    bridge._on_battery(type("M", (), {"percentage": float("nan")})())
    assert bridge.battery is None, "NaN must not be reported as a real level"


def test_drive_watchdog_stops_a_robot_whose_operator_vanished(mod):
    """The failure this prevents is a robot that keeps driving after link loss."""
    import time

    bridge = _bridge(mod, {"drive_timeout_s": 0.05})
    bridge.drive(0.3, 0.0)
    assert bridge.mode == "teleop"

    time.sleep(0.08)
    bridge.drive_watchdog()
    assert bridge.mode == "idle"
    # Last publish must be a zero twist.
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_drive_watchdog_leaves_an_active_operator_alone(mod):
    bridge = _bridge(mod, {"drive_timeout_s": 5.0})
    bridge.drive(0.3, 0.0)
    bridge.drive_watchdog()
    assert bridge.mode == "teleop"


def test_map_frame_odometry_is_an_explicit_tf_fallback(mod):
    bridge = _bridge(mod)
    bridge.map_frame = "map"
    bridge.base_frame = "os_lidar"
    bridge._pose_warned = False
    bridge.tf_buffer = MagicMock()
    bridge.tf_buffer.lookup_transform.side_effect = RuntimeError("no TF")
    msg = MagicMock()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    bridge._on_odom(msg)

    assert bridge.map_pose() == {"x": 1.25, "y": -0.5, "yaw": 0.0}
    warning = bridge.node.get_logger().warn.call_args[0][0]
    assert "using direct map-frame odometry" in warning
    assert "DRIFTS" not in warning


def test_shipped_configs_are_valid_and_disable_what_they_lack(mod):
    import yaml

    for name in ("generic", "duckiebot", "bunker"):
        path = REPO / "adapters" / "adapter_ros2" / "config" / f"{name}.yaml"
        cfg = mod.deep_merge(mod.DEFAULTS, yaml.safe_load(path.read_text()))
        assert cfg["map_frame"] and cfg["base_frame"], f"{name} needs both frames"
        assert cfg["rates"]["state_hz"] > 0
        # Detection poll; never faster than the old 5 Hz preview cap.
        assert cfg["rates"]["camera_period_s"] >= 0.2, f"{name} exceeds the 5 Hz cap"


def test_registered_cloud_alone_advertises_map(mod):
    bridge = _bridge(mod, {"topics": {"map": "", "map_cloud": "/registered_scan"}})
    assert "map" in bridge.capabilities()

    neither = _bridge(mod, {"topics": {"map": "", "map_cloud": ""}})
    assert "map" not in neither.capabilities()


class _FakeField:
    def __init__(self, name: str, offset: int) -> None:
        self.name = name
        self.offset = offset


def _fake_cloud(points, extra_fields=()):
    import struct

    names = list(extra_fields) + ["x", "y", "z"]
    fields = [_FakeField(name, index * 4) for index, name in enumerate(names)]
    point_step = len(names) * 4
    data = b"".join(
        struct.pack("<%df" % len(names), *([0.0] * len(extra_fields)), *point)
        for point in points
    )
    return type("M", (), {"fields": fields, "point_step": point_step, "data": data})()


def test_registered_cloud_uses_declared_offsets_and_height_band(mod):
    bridge = _bridge(mod, {"map_cloud_height_band": {"min_z": 0.0, "max_z": 1.0}})
    msg = _fake_cloud(
        [(1.0, 2.0, -0.5), (3.0, 4.0, 0.5), (5.0, 6.0, 2.5)],
        extra_fields=["intensity"],
    )
    bridge._on_map_cloud(msg)

    assert bridge._scan_points.shape == (1, 2)
    assert bridge._scan_points[0].tolist() == pytest.approx([3.0, 4.0])
    assert bridge._cloud_points.shape == (3, 3)
    assert bridge._cloud_points[0].tolist() == pytest.approx([1.0, 2.0, -0.5])
    assert bridge._cloud_points[1].tolist() == pytest.approx([3.0, 4.0, 0.5])
    assert bridge._cloud_points[2].tolist() == pytest.approx([5.0, 6.0, 2.5])


def test_nav_cmd_vel_relay_forwards_only_while_navigating(mod):
    """Nav2 output must not reach the driver outside an active action goal."""
    bridge = _bridge(mod, {"topics": {"nav_cmd_vel": "cmd_vel_nav"}})
    twist = MagicMock()

    bridge.nav_status = "idle"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_not_called()

    bridge.nav_status = "active"
    bridge._on_nav_cmd_vel(twist)
    bridge.pub_cmd.publish.assert_called_once_with(twist)


def test_teleop_cancels_an_active_action_goal(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "navigate_to_pose"}})
    handle = MagicMock()
    bridge._goal_handle = handle
    bridge.nav_status = "active"
    bridge.goal = {"x": 2.0, "y": 2.0}

    bridge.drive(0.1, 0.0)

    handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None
    assert bridge.nav_status == "cancelled"
    assert bridge.goal is None
    assert bridge.mode == "teleop"


def test_stale_accepted_action_response_is_cancelled(mod):
    bridge = _bridge(mod, {"actions": {"navigate_to_pose": "navigate_to_pose"}})
    bridge._goal_generation = 2
    handle = MagicMock()
    handle.accepted = True
    future = MagicMock()
    future.result.return_value = handle

    bridge._on_goal_response(future, generation=1)

    handle.cancel_goal_async.assert_called_once_with()
    assert bridge._goal_handle is None


def test_teleop_zero_command_does_not_touch_an_idle_nav_state(mod):
    """drive(0, 0) is sent routinely (deadman, initial state) — it must not
    spuriously cancel a goal that isn't even active."""
    bridge = _bridge(mod)
    bridge.nav_status = "idle"

    bridge.drive(0.0, 0.0)
    assert bridge.nav_status == "idle"


def test_detection_batch_survives_concurrent_collection(mod):
    """The websocket may collect a batch while the ROS callback builds it."""
    bridge = _bridge(mod)
    bridge._detections = None
    bridge._depth_map_position = MagicMock(return_value=None)

    class Detection:
        bbox = (0, 0, 1, 1)
        polygon = ()
        label = "rubber_duck"

        def as_protocol(self, detection_id):
            return {"id": detection_id}

    class Detector:
        def detect_bgr(self, _frame):
            yield Detection()
            bridge.take_detections()
            yield Detection()

    bridge._detector = Detector()
    bridge._detect_bgr(MagicMock(), due_checked=True)

    assert [item["id"] for item in bridge.take_detections()] == ["rubber_duck_0", "rubber_duck_1"]


def test_upload_scan_excludes_returns_inside_robot_footprint(mod, monkeypatch):
    bridge = _bridge(mod, {"footprint_radius": 0.65})
    bridge.http_url = "http://backend"
    bridge.map_pose = MagicMock(return_value={"x": 1.0, "y": 2.0, "yaw": 0.0})
    bridge._scan_points = mod.np.array(
        [[1.1, 2.1], [1.3, 2.4], [1.7, 2.0]], dtype=mod.np.float32
    )
    bridge._scan_dirty = True
    captured = {}

    class Response:
        def read(self):
            return b"{}"

    def urlopen(request, timeout):
        captured["request"] = request
        return Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)
    bridge.upload_scan()

    request = captured["request"]
    assert "origin_x=1.0&origin_y=2.0" in request.full_url
    decoded = mod.np.frombuffer(mod.zlib.decompress(request.data), dtype=mod.np.int16)
    assert decoded.reshape(-1, 2).tolist() == [[170, 200]]


def test_upload_cloud_uses_shared_xyz_transport(mod, monkeypatch):
    bridge = _bridge(mod)
    bridge.http_url = "http://backend"
    bridge._cloud_points = mod.np.array([[1.25, -2.5, 0.75]], dtype=mod.np.float32)
    bridge._cloud_dirty = True
    captured = {}

    class Response:
        def read(self):
            return b"{}"

    def urlopen(request, timeout):
        captured["request"] = request
        return Response()

    monkeypatch.setattr(mod.urllib.request, "urlopen", urlopen)
    bridge.upload_cloud()

    request = captured["request"]
    assert request.full_url == "http://backend/api/adapter/cloud?robot_id=r0&scale=0.01"
    decoded = mod.np.frombuffer(mod.zlib.decompress(request.data), dtype=mod.np.int16)
    assert decoded.reshape(-1, 3).tolist() == [[125, -250, 75]]


def test_link_watchdog_stops_autonomy_when_the_operator_link_goes_stale(mod):
    """The Botman accident, 2026-08-12: a goal ran on after the link dropped.

    `drive_timeout_s` cannot cover this. Held teleop repeats, so its silence is
    detectable; an active goal sends nothing, and the relay is gated on
    `nav_status` alone — which nothing revoked when the link wedged.
    """
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()

    # Fresh link: autonomy is relayed as before.
    bridge._on_nav_cmd_vel(MagicMock())
    assert bridge.pub_cmd.publish.call_count == 1

    time.sleep(0.08)
    bridge.link_watchdog()

    assert bridge.nav_status == "cancelled", (
        "the goal must be cancelled, not merely overridden: the relay is gated "
        "on nav_status, so leaving it active lets Nav2's next sample overwrite "
        "the stop within milliseconds"
    )
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_nav_relay_refuses_to_drive_while_the_link_is_stale(mod):
    """Belt and braces: the invariant holds in the relay, not just in a timer."""
    import time

    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 0.05},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()
    time.sleep(0.08)

    bridge._on_nav_cmd_vel(MagicMock())
    bridge.pub_cmd.publish.assert_not_called()


def test_link_watchdog_leaves_a_healthy_link_navigating(mod):
    """A deadman that trips on a working link is worse than none."""
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"}, "link_timeout_s": 5.0},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()

    bridge.link_watchdog()
    assert bridge.nav_status == "active"

    bridge._on_nav_cmd_vel(MagicMock())
    assert bridge.pub_cmd.publish.call_count == 1


def test_link_watchdog_ignores_a_robot_that_is_not_navigating(mod):
    """No goal, nothing to cancel — teleop keeps its own separate deadman."""
    import time

    bridge = _bridge(mod, {"link_timeout_s": 0.05})
    bridge.nav_status = "idle"
    time.sleep(0.08)

    bridge.link_watchdog()
    assert bridge.nav_status == "idle"
    bridge.pub_cmd.publish.assert_not_called()


def test_stop_for_exit_cancels_and_zeroes_before_the_process_dies(mod):
    """A restarted adapter must not leave a driving robot behind.

    SIGTERM's default action kills the interpreter without running `finally`,
    so `docker stop` and every Compose recreate used to end with the base still
    executing its last velocity and no deadman left anywhere to countermand it.
    """
    bridge = _bridge(
        mod,
        {"topics": {"nav_cmd_vel": "cmd_vel_nav"},
         "actions": {"navigate_to_pose": "navigate_to_pose"}},
    )
    bridge.nav_status = "active"
    bridge.note_link_activity()
    bridge.drive(0.4, 0.2)

    bridge.stop_for_exit()

    assert bridge.nav_status == "cancelled"
    last = bridge.pub_cmd.publish.call_args[0][0]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


# --------------------------------------------------------------- link plumbing


class _FakeWs:
    """A socket that records what was written and never delivers anything."""

    def __init__(self) -> None:
        self.sent: list[dict] = []

    async def send(self, raw: str) -> None:
        import json

        self.sent.append(json.loads(raw))

    def __aiter__(self):
        return self

    async def __anext__(self):
        import asyncio

        await asyncio.sleep(3600)  # the operator sends nothing in this scenario
        raise StopAsyncIteration


def _link_bridge(mod, hooks):
    """A bridge that is nothing but the surface `run_robot` drives."""

    class _Bridge:
        cfg = mod.deep_merge(
            mod.DEFAULTS,
            # A fast pump and an always-due map upload, so the scenario reaches
            # the blocking call immediately instead of waiting out a real period.
            {"rates": {"state_hz": 50.0, "map_period_s": 0.0, "camera_period_s": 3600.0}},
        )
        id = "r0"
        t0 = 0.0

        def __init__(self):
            self.node = MagicMock()

        def state(self):
            return {"type": "robot_state", "robot_id": "r0"}

        def capabilities(self):
            return []

        def note_link_activity(self):
            pass

        def upload_map(self):
            return hooks["upload_map"]()

        def upload_scan(self):
            pass

        def upload_cloud(self):
            pass

        def upload_camera(self):
            pass

        def run_detection(self):
            pass

        def take_detections(self):
            return None

        def refresh_settings(self):
            pass

        def cancel_goal(self):
            pass

        def drive(self, *_args):
            pass

    return _Bridge()


def test_a_blocking_upload_does_not_stall_the_state_pump(mod, monkeypatch):
    """The four-robot regression: uploads must never gate telemetry.

    `upload_map` blocks for up to its 5 s timeout, and it blocks for that long
    precisely when the fleet is large — every robot's map queues behind one lock
    on the server. While state and uploads shared a coroutine, that stalled the
    5 Hz pump too, and the backend declares a robot offline after 4 s
    (OFFLINE_AFTER_S) while `link_watchdog` cancels its active goal after 1.5 s.
    So a fourth robot connecting took control of the first one away.
    """
    import asyncio
    import json
    import threading

    ws = _FakeWs()

    class _Conn:
        async def __aenter__(self):
            return ws

        async def __aexit__(self, *_exc):
            return False

    monkeypatch.setattr(mod.websockets, "connect", lambda *a, **k: _Conn())

    started = threading.Event()
    release = threading.Event()

    def blocking_upload():
        started.set()
        release.wait(5.0)  # stands in for the real 5 s urllib timeout
        return None

    bridge = _link_bridge(mod, {"upload_map": blocking_upload})

    def count_states():
        return sum(1 for m in ws.sent if m.get("type") == "robot_state")

    async def scenario():
        task = asyncio.ensure_future(mod.run_robot(bridge, "ws://test"))
        loop = asyncio.get_running_loop()
        # Wait for the upload to actually be in flight before measuring.
        await loop.run_in_executor(None, started.wait, 5.0)
        before = count_states()
        await asyncio.sleep(0.4)
        during = count_states()
        release.set()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        return before, during

    before, during = asyncio.run(asyncio.wait_for(scenario(), 15))

    assert started.is_set(), "the scenario never reached the blocking upload"
    # 0.4 s at 50 Hz is ~20 frames; anything above a handful proves the pump ran.
    assert during - before >= 5, (
        f"state stopped while an upload was in flight ({during - before} frames "
        "sent during a 0.4 s block) — uploads are gating telemetry again"
    )
