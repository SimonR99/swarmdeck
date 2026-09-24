"""Missionless development supports commands, not live mapping authority."""

import asyncio

import pytest
from fastapi.testclient import TestClient

from swarmdeck_server.api import map_routes, state
from swarmdeck_server.api.app import app
from swarmdeck_server.api.gui_socket import handle_gui_message
from swarmdeck_server.fleet.registry import Registry
from tests.test_peer_slam_status import report
from tests.test_replica_live import MISSION, payload


@pytest.fixture
def missionless(monkeypatch):
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    registry = Registry(command_guard=map_routes.command_guard)
    monkeypatch.setattr(state, "registry", registry)
    monkeypatch.setattr(map_routes, "registry", registry)
    monkeypatch.setattr(state, "_gui_clients", set())
    monkeypatch.setattr(
        state.settings_store, "value", state.settings_store.validate({})
    )
    return registry


@pytest.mark.parametrize(
    "method,path",
    [
        ("POST", "/api/map/reset"),
        ("POST", "/api/map/reset/r0"),
        ("GET", "/api/map/reset/r0"),
    ],
)
def test_map_reset_requires_an_active_mission(missionless, method, path):
    response = TestClient(app).request(method, path)
    assert response.status_code == 409
    assert response.json()["error"] == "no active mission"


@pytest.mark.parametrize("method", ["GET", "POST"])
def test_live_component_operations_require_an_active_mission(missionless, method):
    path = f"/api/autonomy/replicas/components/live/{MISSION}"
    if method == "GET":
        response = TestClient(app).get(path, params={"component_id": "component"})
    else:
        response = TestClient(app).post(
            path + "/goal",
            json={
                "robot_id": "r0",
                "component_id": "component",
                "solution_order": [0, -1],
                "goal": {"x": 1.0, "y": 2.0},
            },
        )
    assert response.status_code == 409
    assert response.json()["detail"] == "no active mission"


def test_mapping_authority_is_not_admitted_without_a_mission(missionless):
    robot = missionless.hello({"robot_id": "r0"}, None)
    missionless.update_state(
        {
            "robot_id": "r0",
            "pose": {"x": 3.0, "y": 4.0, "yaw": 0.0},
            "peer_slam": report(),
            "live_mapping": payload(),
        }
    )
    assert robot.peer_slam is None
    assert robot.live_mapping is None
    assert robot.pose["x"] == 3.0  # Ordinary mock telemetry is still useful.


def test_unfenced_home_pose_is_not_used_as_mapping_authority(missionless):
    robot = missionless.hello({"robot_id": "r0"}, None)
    missionless.update_state(
        {
            "robot_id": "r0",
            "home_pose": {"x": 2.0, "y": 3.0, "yaw": 0.0},
        }
    )
    assert robot.home_pose is None


def test_mock_commands_still_work_without_a_mission(missionless):
    class Socket:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    socket = Socket()
    missionless.hello(
        {
            "robot_id": "r0",
            "robot_type": "spot",
            "capabilities": ["navigate", "plan_objective", "explore", "body_control"],
        },
        socket,
    )
    client = TestClient(app)
    assert client.get("/api/config").status_code == 200
    drive = client.post("/api/robot/r0/drive", json={"linear": 0.2})
    goal = client.post("/api/robot/r0/goal", json={"x": 1.0, "y": 2.0})
    body = client.post("/api/robot/r0/body", json={"action": "sit"})
    assert drive.status_code == goal.status_code == body.status_code == 200
    asyncio.run(handle_gui_message({"type": "start_explore", "robot_id": "r0"}))
    assert [message["type"] for message in socket.messages] == [
        "drive",
        "plan_objective",
        "body_command",
        "explore",
    ]
