"""Durable target reset admission, stale writers, and isolation across restarts."""

import asyncio
import json
import time
from uuid import uuid4

import numpy as np
import pytest
from fastapi import FastAPI, Request
from fastapi.testclient import TestClient

from autonomy.map_epochs import robot_run_id
from autonomy.replication import ReplicaStore
from swarmdeck_server.api import app, autonomy_routes, map_routes, replica_views
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.mapsvc.service import GridMeta, MapService
from tests.test_replica_components import peer


class Sink:
    def __init__(self):
        self.messages = []

    async def send_json(self, message):
        self.messages.append(message)


@pytest.fixture
def reset_api(tmp_path, monkeypatch):
    mission = str(uuid4())
    root = tmp_path / "reset"
    root.mkdir()
    (root / "supervisor.json").write_text(
        json.dumps(
            {
                "version": 1,
                "updated_at_ns": time.time_ns(),
                "mission_id": mission,
                "backend": "mola",
                "supported_robot_ids": ["robot_0", "robot_1"],
            }
        )
    )
    replicas = ReplicaStore(tmp_path / "replicas")
    registry = Registry(
        epoch_store=lambda: replicas, command_guard=map_routes.robot_command_error
    )
    service = MapService(resolution=0.1, size_m=4.0)
    sinks, sources = {}, {}
    for robot_id in ("robot_0", "robot_1"):
        source, chunks = peer(tmp_path, robot_id, mission)
        for digest, body in chunks.items():
            replicas.put_chunk(digest, body)
        replicas.publish(source)
        sources[robot_id] = source
        sink = sinks[robot_id] = Sink()
        robot = registry.hello(
            {"robot_id": robot_id, "capabilities": ["navigate", "explore"]}, sink
        )
        robot.goal = {"x": 2.0, "y": 0.0}
        robot.nav_status = "active"
        robot.home_pose = {"x": -1.0, "y": 0.0, "yaw": 0.0}
        service.ingest(
            robot_id, GridMeta(0.1, 2, 2, 0.0, 0.0), np.zeros((2, 2), dtype=np.int8)
        )
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    monkeypatch.setenv("SWARMDECK_SIM_RESET_DIR", str(root))
    monkeypatch.setattr(autonomy_routes, "store", lambda: replicas)
    monkeypatch.setattr(replica_views, "store", lambda: replicas)
    monkeypatch.setattr(map_routes, "registry", registry)
    monkeypatch.setattr(app, "registry", registry)
    monkeypatch.setattr(app, "CONFIG", {})
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(map_routes, "_optimized", {})
    monkeypatch.setattr(map_routes, "_server_scopes", set())
    monkeypatch.setattr(map_routes, "_optimized_seq", {})
    monkeypatch.setattr(map_routes, "_costmaps", {})
    monkeypatch.setattr(map_routes, "_robot_epoch_locks", {})
    monkeypatch.setattr(map_routes, "_raster_generation", 0)
    api = FastAPI()
    api.add_api_route(
        "/api/map/reset/{robot_id}", app.reset_robot_map, methods=["POST"]
    )
    api.add_api_route(
        "/api/map/reset/{robot_id}", app.get_robot_map_reset, methods=["GET"]
    )
    api.add_api_route("/api/map/reset", map_routes.reset_all_maps, methods=["POST"])
    api.add_api_route("/api/adapter/map", map_routes.post_map, methods=["POST"])
    api.add_api_route(
        "/api/adapter/keyframe", map_routes.post_keyframe, methods=["POST"]
    )
    api.add_api_route(
        "/api/adapter/global_map", map_routes.post_global_map, methods=["POST"]
    )
    api.add_api_route("/api/slam/update", map_routes.post_slam_update, methods=["POST"])
    api.add_api_route(
        "/api/slam/optimized_map", map_routes.post_optimized_map, methods=["POST"]
    )
    api.include_router(autonomy_routes.router)
    with TestClient(api) as client:
        yield client, root, replicas, registry, service, sinks, sources, mission
    replicas.close()


def finish(root, robot, accepted, *, ok=True, error=None):
    status = {**accepted, "phase": "done" if ok else "failed", "ok": ok, "error": error}
    directory = root / "robots" / robot
    for path in (
        directory / "status.json",
        directory / "requests" / f"{status['request_id']}.json",
    ):
        path.write_text(json.dumps(status))
    return status


def test_acceptance_fences_old_uploads_and_cancels_only_target(reset_api):
    client, root, replicas, registry, service, sinks, sources, mission = reset_api
    request_id = str(uuid4())
    response = client.post(f"/api/map/reset/robot_0?request_id={request_id}")
    assert response.status_code == 202
    accepted = response.json()
    assert accepted["ok"] is None and accepted["phase"] == "accepted"
    assert accepted["mission_id"] == mission
    assert replicas.get("robot_0", mission) is None
    assert replicas.get("robot_1", mission) == sources["robot_1"]
    assert "robot_0" not in service.robot_grids and "robot_1" in service.robot_grids
    assert registry.robots["robot_0"].home_pose is None
    assert registry.robots["robot_1"].goal == {"x": 2.0, "y": 0.0}
    assert sinks["robot_1"].messages == []
    assert sinks["robot_0"].messages[-1]["type"] == "stop"
    late = client.post(
        "/api/autonomy/replicas", json={**sources["robot_0"], "revision": 100}
    )
    assert late.status_code == 409
    assert client.get(accepted["status_url"]).json()["phase"] == "accepted"
    assert map_routes.robot_command_error("robot_0") is not None
    assert client.post("/api/map/reset").status_code == 409


def test_duplicate_uuid_survives_newer_request_and_server_store_restart(reset_api):
    client, root, replicas, _, _, sinks, _, mission = reset_api
    request_id = str(uuid4())
    url = f"/api/map/reset/robot_0?request_id={request_id}"
    accepted = client.post(url).json()
    assert client.post(url).json()["map_epoch"] == accepted["map_epoch"]
    assert len(sinks["robot_0"].messages) == 1
    done = finish(root, "robot_0", accepted)
    second = client.post(f"/api/map/reset/robot_0?request_id={uuid4()}").json()
    assert second["map_epoch"] == accepted["map_epoch"] + 1
    replay = client.post(url)
    assert replay.status_code == 200 and replay.json()["run_id"] == done["run_id"]
    assert len(sinks["robot_0"].messages) == 2
    recovered = ReplicaStore(replicas.root)
    assert recovered.map_epoch("robot_0", mission) == second["map_epoch"]
    assert recovered.get("robot_0", mission) is None
    recovered.close()


def test_unavailable_or_unsupported_supervisor_never_clears_a_map(reset_api):
    client, root, replicas, _, service, sinks, sources, mission = reset_api
    (root / "supervisor.json").write_text(
        json.dumps(
            {
                "updated_at_ns": time.time_ns(),
                "mission_id": mission,
                "supported_robot_ids": [],
                "backend": "hardware-unavailable",
            }
        )
    )
    response = client.post(f"/api/map/reset/robot_0?request_id={uuid4()}")
    assert response.status_code == 503 and response.json()["ok"] is False
    assert replicas.get("robot_0", mission) == sources["robot_0"]
    assert "robot_0" in service.robot_grids
    assert sinks["robot_0"].messages == []
    assert not (root / "robots" / "robot_0" / "request.json").exists()


def test_failed_restart_stays_visible_and_blocks_new_commands(reset_api):
    client, root, _, registry, _, sinks, _, _ = reset_api
    accepted = client.post(f"/api/map/reset/robot_0?request_id={uuid4()}").json()
    finish(root, "robot_0", accepted, ok=False, error="SLAM reset service unavailable")
    status = client.get(accepted["status_url"]).json()
    assert (
        status["phase"] == "failed"
        and status["error"] == "SLAM reset service unavailable"
    )
    assert not asyncio.run(
        registry.send("robot_0", {"type": "explore", "enabled": True})
    )
    assert [message["type"] for message in sinks["robot_0"].messages] == ["stop"]


def test_stale_websocket_cannot_restore_home_goal_or_authority(reset_api):
    client, _, _, registry, _, _, _, mission = reset_api
    client.post(f"/api/map/reset/robot_0?request_id={uuid4()}")
    robot = registry.robots["robot_0"]
    assert (
        registry.update_state(
            {
                "robot_id": "robot_0",
                "home_pose": {"x": 99, "y": 0, "yaw": 0},
                "goal": {"x": 99, "y": 0},
                "navigation_ready": True,
                "live_mapping": {
                    "mission_id": mission,
                    "robot_map_epoch": 0,
                    "run_id": robot_run_id(mission, "robot_0", 0),
                },
            }
        )
        is None
    )
    assert robot.home_pose is None and robot.goal is None and robot.live_mapping is None
    assert robot.navigation_ready is False


def test_upload_started_before_reset_is_rejected_after_body_arrives(reset_api):
    _, _, replicas, _, service, _, _, mission = reset_api

    async def scenario():
        entered, release = asyncio.Event(), asyncio.Event()
        import zlib

        body = zlib.compress(bytes([0, 0, 0, 0]))

        async def receive():
            entered.set()
            await release.wait()
            return {"type": "http.request", "body": body, "more_body": False}

        request = Request(
            {
                "type": "http",
                "method": "POST",
                "path": "/api/adapter/map",
                "query_string": b"robot_id=robot_0&width=2&height=2&resolution=0.1",
                "headers": [
                    (b"x-mission-id", mission.encode()),
                    (b"x-map-epoch", b"0"),
                    (b"x-run-id", robot_run_id(mission, "robot_0", 0).encode()),
                ],
            },
            receive,
        )
        upload = asyncio.create_task(map_routes.post_map(request))
        await entered.wait()
        replicas.reserve_map_epoch("robot_0", mission, 1)
        await service.reset_robot_async("robot_0")
        release.set()
        assert (await upload).status_code == 409
        assert "robot_0" not in service.robot_grids
        assert "robot_1" in service.robot_grids

    asyncio.run(scenario())


def test_newer_replica_retires_old_server_products_without_reset_request(
    reset_api, tmp_path
):
    client, _, replicas, registry, service, sinks, _, mission = reset_api
    fresh, chunks = peer(tmp_path, "robot_0", mission, revision=0, map_epoch=1)
    for digest, body in chunks.items():
        replicas.put_chunk(digest, body)
    response = client.post("/api/autonomy/replicas", json=fresh)
    assert response.status_code == 200
    assert replicas.get("robot_0", mission)["run_id"] == fresh["run_id"]
    assert "robot_0" not in service.robot_grids
    assert "robot_1" in service.robot_grids
    assert registry.robots["robot_0"].goal is None
    assert registry.robots["robot_1"].goal == {"x": 2.0, "y": 0.0}
    assert sinks["robot_1"].messages == []


def test_surveyed_home_survives_but_inferred_home_uses_new_first_keyframe(
    reset_api, monkeypatch
):
    client, _, _, registry, _, _, _, mission = reset_api
    monkeypatch.setattr(
        app, "CONFIG", {"map": {"start_poses": {"robot_1": {"x": 2.0}}}}
    )
    surveyed = dict(registry.robots["robot_1"].home_pose)
    client.post(f"/api/map/reset/robot_1?request_id={uuid4()}")
    assert registry.robots["robot_1"].home_pose == surveyed
    accepted = client.post(f"/api/map/reset/robot_0?request_id={uuid4()}").json()
    from tests.test_replica_deployment_composite import live_payload

    live = live_payload("robot_0", "component:fresh", mission)
    live.update(robot_map_epoch=accepted["map_epoch"], run_id=accepted["run_id"])
    live["home"] = {
        "keyframe_id": f"robot_0/{accepted['run_id']}/0",
        "T_navigation_home": [[1, 0, 0, 4], [0, 1, 0, -2], [0, 0, 1, 0], [0, 0, 0, 1]],
    }
    registry.update_state(
        {
            "robot_id": "robot_0",
            "live_mapping": live,
            "home_pose": {"x": -99.0, "y": 0.0, "yaw": 0.0},
        }
    )
    assert registry.robots["robot_0"].home_pose == {"x": 4, "y": -2, "yaw": 0}


def test_inflight_command_keeps_old_run_fence_and_cannot_restore_command_state():
    async def scenario():
        entered, release = asyncio.Event(), asyncio.Event()
        delivered = []

        class DelayedSink:
            async def send_json(self, message):
                entered.set()
                await release.wait()
                delivered.append(message)

        mission = str(uuid4())
        registry = Registry()
        robot = registry.hello({"robot_id": "robot_0"}, DelayedSink())
        robot.live_mapping = {
            "mission_id": mission,
            "robot_map_epoch": 0,
            "run_id": robot_run_id(mission, "robot_0", 0),
        }
        command = asyncio.create_task(
            registry.send(
                "robot_0",
                {
                    "type": "explore",
                    "enabled": True,
                    "run_id": "fleet-exploration",
                },
            )
        )
        await entered.wait()
        robot.command_generation += 1
        robot.live_mapping = {
            "mission_id": mission,
            "robot_map_epoch": 1,
            "run_id": robot_run_id(mission, "robot_0", 1),
        }
        release.set()
        assert not await command
        assert delivered[0]["map_run_id"] == robot_run_id(mission, "robot_0", 0)
        assert delivered[0]["robot_map_epoch"] == 0
        assert delivered[0]["run_id"] == "fleet-exploration"

    asyncio.run(scenario())


def test_peer_mission_rejects_delayed_central_graph_publications(reset_api):
    client, _, _, _, service, _, _, _ = reset_api
    for path in (
        "/api/adapter/keyframe",
        "/api/adapter/global_map",
        "/api/slam/update",
        "/api/slam/optimized_map",
    ):
        response = client.post(path, content=b"old queued publication")
        assert response.status_code == 409
    assert service.global_grid is None
    assert map_routes._optimized == {}


def test_stale_upload_is_rejected_without_reading_its_body(reset_api):
    _, _, replicas, _, _, _, _, mission = reset_api
    replicas.reserve_map_epoch("robot_0", mission, 1)

    async def scenario():
        async def receive():
            raise AssertionError("stale upload body must not be consumed")

        request = Request(
            {
                "type": "http",
                "method": "POST",
                "path": "/api/adapter/map",
                "query_string": b"robot_id=robot_0&width=2&height=2",
                "headers": [
                    (b"x-mission-id", mission.encode()),
                    (b"x-map-epoch", b"0"),
                    (b"x-run-id", robot_run_id(mission, "robot_0", 0).encode()),
                ],
            },
            receive,
        )
        assert (await map_routes.post_map(request)).status_code == 409

    asyncio.run(scenario())
