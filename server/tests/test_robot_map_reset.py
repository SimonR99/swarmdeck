import asyncio
import os
from uuid import uuid4

from fastapi import FastAPI
from fastapi.testclient import TestClient

from autonomy.replication import ReplicaStore
from swarmdeck_server.api import map_routes
from swarmdeck_server.api.map_routes import reset_costmaps
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.mapsvc.service import MapService


class Sink:
    def __init__(self):
        self.messages = []

    async def send_json(self, message):
        self.messages.append(message)


def test_reset_all_rejects_peer_mission(monkeypatch):
    monkeypatch.setenv("SWARMDECK_MISSION_ID", str(uuid4()))
    response = asyncio.run(map_routes.reset_all_maps())
    assert response.status_code == 409


def test_targeted_epoch_retirement_clears_only_target(monkeypatch):
    mission = str(uuid4())
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    registry = Registry()
    sinks = {rid: Sink() for rid in ("r0", "r1")}
    for rid, sink in sinks.items():
        registry.hello({"robot_id": rid}, sink)
    registry.robots["r0"].goal = {"x": 1.0, "y": 2.0}
    registry.robots["r0"].nav_status = "active"
    registry.robots["r1"].goal = {"x": 3.0, "y": 4.0}
    registry.robots["r1"].nav_status = "active"
    service = MapService()
    service.set_transform("r0", 1.0, 2.0, 0.0)
    service.set_transform("r1", 3.0, 4.0, 0.0)
    monkeypatch.setattr(map_routes, "registry", registry)
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(map_routes, "_optimized", {})
    monkeypatch.setattr(map_routes, "_server_scopes", set())
    monkeypatch.setattr(map_routes, "_optimized_seq", {})
    monkeypatch.setattr(map_routes, "_raster_generation", 0)

    async def publish(message):
        return None

    from swarmdeck_server.api import app
    monkeypatch.setattr(app, "CONFIG", {"map": {"start_poses": {"r0": {}}}})
    monkeypatch.setattr(app, "broadcast", publish)
    asyncio.run(map_routes.retire_robot_epoch("r0", mission, 2))

    assert registry.robots["r0"].goal is None
    assert registry.robots["r0"].nav_status == "cancelled"
    assert registry.robots["r1"].goal == {"x": 3.0, "y": 4.0}
    assert "r0" not in service._network_grids
    assert sinks["r0"].messages[-1]["type"] == "stop"


def test_local_costmap_rejects_global_kind():
    app = FastAPI()
    app.add_api_route("/api/adapter/costmap", map_routes.post_costmap, methods=["POST"])
    with TestClient(app) as client:
        response = client.post(
            "/api/adapter/costmap?robot_id=r0&kind=global",
            content=b"not-a-grid",
        )
    assert response.status_code == 400
