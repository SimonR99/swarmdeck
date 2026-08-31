"""ROS-free regression tests for the shared adapter protocol runtime."""

from __future__ import annotations

import math
import types

import numpy as np
import pytest

from adapters.runtime import (
    AdapterHelloMixin,
    AdapterTelemetryMixin,
    HELLO_FIELDS,
    PROTOCOL_VERSION,
    TRANSPORT_DEFAULTS,
    cloud_xyz,
    deep_merge,
    detections_message,
    hello_message,
    map_cloud_height_limits,
    next_backoff,
    project_occupied_cloud,
    stamp_seconds,
    websocket_connect_kwargs,
    yaw_of,
)


def test_deep_merge_preserves_profile_siblings():
    merged = deep_merge(
        {"topics": {"odom": "odom", "map": "map"}},
        {"topics": {"odom": "wheel_odom"}},
    )
    assert merged == {"topics": {"odom": "wheel_odom", "map": "map"}}


def test_load_yaml_profile_deep_merges_an_optional_file(tmp_path):
    from adapters.runtime import load_yaml_profile

    path = tmp_path / "robot.yaml"
    path.write_text("topics:\n  odom: wheel_odom\n")
    defaults = {"topics": {"odom": "odom", "map": "map"}, "robot_type": "generic"}
    assert load_yaml_profile(None, defaults) == defaults
    assert load_yaml_profile(str(path), defaults) == {
        "topics": {"odom": "wheel_odom", "map": "map"},
        "robot_type": "generic",
    }


def test_stamp_seconds_accepts_both_ros_timestamp_shapes():
    ros1 = types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 2.5))
    ros2 = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=2, nanosec=500_000_000)
    )
    assert stamp_seconds(ros1) == 2.5
    assert stamp_seconds(ros2) == 2.5
    assert stamp_seconds(types.SimpleNamespace(stamp=None)) is None


def test_map_cloud_height_limits_can_be_expressed_above_a_floor():
    assert map_cloud_height_limits(
        {
            "floor_z": -0.5,
            "min_z": 0.15,
            "max_z": 0.65,
        }
    ) == pytest.approx((-0.35, 0.15))


def test_map_cloud_height_limits_preserves_legacy_map_frame_profiles():
    assert map_cloud_height_limits({"min_z": -0.3, "max_z": 0.5}) == pytest.approx(
        (-0.3, 0.5)
    )


def test_project_occupied_cloud_keeps_unknown_cells_unknown():
    result = project_occupied_cloud(
        np.array([[1.0, 2.0], [1.1, 2.1], [math.nan, 4.0]]),
        resolution=0.5,
        padding_m=0.5,
    )
    assert result is not None
    resolution, width, height, origin_x, origin_y, cells = result
    assert resolution == pytest.approx(0.5)
    assert (width, height) == (3, 3)
    assert (origin_x, origin_y) == pytest.approx((0.5, 1.5))
    assert int((cells == 100).sum()) == 1
    assert int((cells == -1).sum()) == 8


def test_cloud_xyz_honours_field_offsets_and_drops_nonfinite_rows():
    fields = [
        types.SimpleNamespace(name="intensity", offset=0),
        types.SimpleNamespace(name="x", offset=4),
        types.SimpleNamespace(name="y", offset=8),
        types.SimpleNamespace(name="z", offset=12),
    ]
    rows = np.zeros((2, 16), dtype=np.uint8)
    rows[0, 4:16] = np.frombuffer(
        np.array([1.0, -2.0, 3.0], dtype="<f4").tobytes(), dtype=np.uint8
    )
    rows[1, 4:16] = np.frombuffer(
        np.array([math.inf, 0.0, 1.0], dtype="<f4").tobytes(), dtype=np.uint8
    )
    msg = types.SimpleNamespace(fields=fields, data=rows.tobytes(), point_step=16)
    np.testing.assert_allclose(cloud_xyz(msg), [[1.0, -2.0, 3.0]])


def test_yaw_of_uses_planar_quaternion_component():
    q = types.SimpleNamespace(
        w=math.cos(math.pi / 4), z=math.sin(math.pi / 4), x=0.0, y=0.0
    )
    assert yaw_of(q) == pytest.approx(math.pi / 2)


def test_hello_envelope_is_shared_and_local_by_default():
    msg = hello_message(
        robot_id="robot_0",
        robot_type="agilex_bunker",
        adapter="adapter_sim/0.1.0",
        ros="jazzy",
        capabilities=["navigate", "reset"],
        footprint_radius=0.64,
    )
    assert tuple(msg) == HELLO_FIELDS
    assert msg["protocol"] == PROTOCOL_VERSION
    assert msg["coordinate_frame"] == "local"
    other = hello_message(
        robot_id="r0",
        robot_type="generic",
        adapter="adapter_ros2/0.1.0",
        ros="jazzy",
        capabilities=["navigate"],
        footprint_radius=0.35,
        coordinate_frame="world",
    )
    assert other["coordinate_frame"] == "local"


class _ProtocolBridge(AdapterHelloMixin, AdapterTelemetryMixin):
    coordinate_frame = "local"

    def __init__(self, adapter_name: str, extra_caps: list[str] | None = None):
        self.adapter_name = adapter_name
        self.id = "robot_0"
        self.t0 = 0.0
        self.cfg = deep_merge(
            TRANSPORT_DEFAULTS,
            {
                "robot_type": "agilex_bunker",
                "ros_distro": "jazzy",
                "footprint_radius": 0.64,
                "network_iface": "",
            },
        )
        self.battery = None
        self.mode = "idle"
        self.nav_status = "idle"
        self.goal = None
        self.planned_path = [{"x": 1.0, "y": 2.0}]
        self._caps = ["navigate", "map", "camera", "estop", *(extra_caps or [])]

    def capabilities(self):
        return list(self._caps)

    def map_pose(self):
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}


def test_sim_and_hardware_hello_share_keys_protocol_and_frame():
    sim = _ProtocolBridge("adapter_sim/0.1.0", extra_caps=["reset"])
    hardware = _ProtocolBridge("adapter_ros2/0.1.0")
    assert set(sim.hello()) == set(hardware.hello()) == set(HELLO_FIELDS)
    assert sim.hello()["protocol"] == hardware.hello()["protocol"] == PROTOCOL_VERSION
    assert "reset" in sim.hello()["capabilities"]
    assert "reset" not in hardware.hello()["capabilities"]
    assert "battery" not in sim.hello()["capabilities"]


def test_shared_telemetry_envelope_includes_split_paths():
    state = _ProtocolBridge("adapter_sim/0.1.0").state()
    assert state["type"] == "robot_state"
    assert "global_planned_path" in state
    assert "local_planned_path" in state
    assert "network" not in state
    assert state["planned_path"] == [{"x": 1.0, "y": 2.0}]


def test_transport_defaults_are_the_hardware_keepalive():
    assert websocket_connect_kwargs(TRANSPORT_DEFAULTS) == {
        "ping_interval": 2.0,
        "ping_timeout": 4.0,
    }
    assert next_backoff(1.0) == 2.0
    assert next_backoff(20.0) == 30.0


def test_detections_message_stamps_t_mono():
    assert detections_message("robot_0", t0=10.0, items=[{"id": "a"}], now=12.5) == {
        "type": "detections",
        "robot_id": "robot_0",
        "t_mono": 2.5,
        "camera": "front",
        "items": [{"id": "a"}],
    }
