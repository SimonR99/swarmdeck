"""The deployment composite: single-robot components placed in the world frame."""

from __future__ import annotations

import math
import threading
from unittest.mock import AsyncMock
from uuid import uuid4

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from autonomy.contracts import IDENTITY_SE3
from autonomy.replication import ReplicaStore
from autonomy.map_epochs import robot_run_id
from swarmdeck_server.api import autonomy_routes, replica_live, replica_views
from swarmdeck_server.fleet.registry import Registry
from tests.test_replica_components import peer, reseal, selected

# Navigation -> component for robot_0: a 90 degree yaw plus a translation, so
# the composition is only right when both halves are applied in order.
T_CN_0 = (
    (0.0, -1.0, 0.0, 10.0),
    (1.0, 0.0, 0.0, 20.0),
    (0.0, 0.0, 1.0, 2.0),
    (0.0, 0.0, 0.0, 1.0),
)
TRANSFORMS = {"robot_0": (1.0, 2.0, math.pi / 2), "robot_1": (-3.0, 0.5, 0.0)}
SUBMAP_X = {"robot_0": 2.0, "robot_1": -1.0}


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


def live_payload(robot_id, component_id, session, transform=IDENTITY_SE3):
    return dict(
        robot_id=robot_id,
        mission_id=session,
        robot_map_epoch=0,
        run_id=robot_run_id(session, robot_id, 0),
        component_id=component_id,
        navigation_frame=f"{robot_id}/map",
        solution_order=[0, -1],
        T_component_navigation=transform,
        authority_age_s=0.2,
        pose=dict(x=2, y=3, yaw=0),
        goal=None,
        planned_path=[dict(x=2, y=3)],
        global_planned_path=[],
        local_planned_path=[],
    )


class MapServiceStub:
    def __init__(self, transforms):
        self._state_lock = threading.RLock()
        self.transforms = dict(transforms)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    session = str(uuid4())
    store = ReplicaStore(tmp_path / "replicas")
    components = {}
    for robot_id in ("robot_0", "robot_1", "robot_2"):
        source, chunks = peer(tmp_path, robot_id, session)
        selected(source)["submaps"][0]["T_component_submap"][0][3] = SUBMAP_X.get(
            robot_id, 0.0
        )
        reseal(source)
        for name, raw in chunks.items():
            store.put_chunk(name, raw)
        assert store.publish(source)
        components[robot_id] = selected(source)["graph_revision"]["component_id"]
    monkeypatch.setattr(replica_views, "store", lambda: store)
    monkeypatch.setattr(autonomy_routes, "store", lambda: store)

    registry = Registry()
    sink = AsyncMock()
    for robot_id in ("robot_0", "robot_1", "robot_2"):
        registry.hello(
            dict(robot_id=robot_id, capabilities=["navigate", "plan_objective"]), sink
        )
        registry.update_state(
            dict(
                robot_id=robot_id,
                pose=dict(x=900, y=800),
                live_mapping=live_payload(
                    robot_id,
                    components[robot_id],
                    session,
                    T_CN_0 if robot_id == "robot_0" else IDENTITY_SE3,
                ),
            )
        )
    monkeypatch.setattr(replica_views, "registry", registry)
    monkeypatch.setattr(replica_live, "registry", registry)
    # robot_2 has a replica and live authority but no surveyed placement.
    map_service = MapServiceStub(TRANSFORMS)
    monkeypatch.setattr(replica_views, "map_service", map_service)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", session)

    app = FastAPI()
    app.include_router(autonomy_routes.router)
    app.include_router(replica_views.router)
    with TestClient(app) as client:
        yield dict(
            client=client,
            session=session,
            registry=registry,
            sink=sink,
            map_service=map_service,
            components=components,
            store=store,
        )
    store.close()


def composite_id(session):
    return f"deployment:{session}"


def expected_world_component(robot_id):
    world_navigation = se2(*TRANSFORMS[robot_id])
    component_navigation = np.array(T_CN_0 if robot_id == "robot_0" else IDENTITY_SE3)
    return world_navigation @ np.linalg.inv(component_navigation)


def test_catalogue_lists_a_ready_composite_of_placed_single_robot_components(setup):
    client, session = setup["client"], setup["session"]
    response = client.get("/api/autonomy/replicas/components")
    assert response.status_code == 200
    entries = response.json()["components"]
    real = [entry for entry in entries if not entry.get("composite")]
    assert len(real) == 3
    assert all("composite" not in entry for entry in real)
    (entry,) = [entry for entry in entries if entry.get("composite")]
    assert entry == {
        "session_id": session,
        "component_id": composite_id(session),
        "frame_id": "deployment",
        "robot_ids": ["robot_0", "robot_1"],
        "source_count": 2,
        "point_count": 2,
        "submap_count": 2,
        "available": True,
        "status": "ready",
        "detail": "",
        "solution_order": None,
        "solution_order_known": True,
        "sources": entry["sources"],
        "composite": True,
    }
    assert [source["robot_id"] for source in entry["sources"]] == [
        "robot_0",
        "robot_1",
    ]
    assert all(source["revision"] == 1 for source in entry["sources"])
    # An explicit active-session filter lists the same composite; another
    # session never does.
    scoped = client.get(
        "/api/autonomy/replicas/components", params={"session_id": session}
    )
    assert any(entry.get("composite") for entry in scoped.json()["components"])
    other = client.get(
        "/api/autonomy/replicas/components", params={"session_id": str(uuid4())}
    )
    assert other.json()["components"] == []


def test_composite_needs_two_placed_members(setup):
    client, session, map_service = (
        setup["client"],
        setup["session"],
        setup["map_service"],
    )
    # Placing robot_2 adds it; removing robot_1's placement drops it.
    map_service.transforms["robot_2"] = (0.0, 0.0, 0.0)
    del map_service.transforms["robot_1"]
    entries = client.get("/api/autonomy/replicas/components").json()["components"]
    (entry,) = [entry for entry in entries if entry.get("composite")]
    assert entry["robot_ids"] == ["robot_0", "robot_2"]

    # A robot without live mapping authority has no T_component_navigation.
    setup["registry"].update_state(dict(robot_id="robot_2", pose=dict(x=1, y=1)))
    entries = client.get("/api/autonomy/replicas/components").json()["components"]
    assert not any(entry.get("composite") for entry in entries)
    assert (
        client.get(
            f"/api/autonomy/replicas/components/view/{session}",
            params={"component_id": composite_id(session)},
        ).status_code
        == 404
    )


def test_composite_keeps_a_member_whose_replica_runs_ahead_of_its_authority(setup):
    # The replica follows every pose-moving solver result; the authority names
    # the frame of the last published product. A robot with loop closures is
    # ahead most of the time and must not vanish from the fleet map.
    client, session = setup["client"], setup["session"]
    setup["registry"].robots["robot_1"].live_mapping["solution_order"] = [3, 0]
    entries = client.get("/api/autonomy/replicas/components").json()["components"]
    composite = next(entry for entry in entries if entry.get("composite"))
    assert composite["component_id"] == composite_id(session)
    view = client.get(
        f"/api/autonomy/replicas/components/view/{session}",
        params={"component_id": composite_id(session)},
    ).json()
    member = next(m for m in view["members"] if m["robot_id"] == "robot_1")
    # The member record carries the authority's frame revision, the fence
    # the live overlay and goal dispatch apply to that robot.
    assert member["solution_order"] == [3, 0]


def test_composite_view_places_each_submap_with_the_world_component_product(setup):
    client, session = setup["client"], setup["session"]
    response = client.get(
        f"/api/autonomy/replicas/components/view/{session}",
        params={"component_id": composite_id(session)},
    )
    assert response.status_code == 200
    view = response.json()
    assert view["scope"] == "fleet"
    assert view["robot_id"] == "fleet"
    assert view["session_id"] == session
    assert view["component_id"] == composite_id(session)
    assert view["composite"] is True
    assert view["solution_order"] is None
    assert view["solution_order_known"] is True
    assert view["revision"] is None
    selected_component = view["selected"]
    assert selected_component["component_id"] == composite_id(session)
    assert selected_component["frame_id"] == "deployment"
    assert selected_component["graph_revision"] is None
    assert [member["robot_id"] for member in view["members"]] == [
        "robot_0",
        "robot_1",
    ]
    assert [source["robot_id"] for source in view["sources"]] == [
        "robot_0",
        "robot_1",
    ]

    submaps = {submap["submap_id"]: submap for submap in selected_component["submaps"]}
    assert len(submaps) == 2
    for robot_id in ("robot_0", "robot_1"):
        (submap,) = [
            value for key, value in submaps.items() if key.startswith(f"{robot_id}/")
        ]
        component_submap = np.eye(4)
        component_submap[0, 3] = SUBMAP_X[robot_id]
        expected = expected_world_component(robot_id) @ component_submap
        assert np.array(submap["T_component_submap"]) == pytest.approx(
            expected, abs=1e-9
        )
        (member,) = [m for m in view["members"] if m["robot_id"] == robot_id]
        assert np.array(member["T_world_component"]) == pytest.approx(
            expected_world_component(robot_id), abs=1e-9
        )
        assert np.array(member["T_world_navigation"]) == pytest.approx(
            se2(*TRANSFORMS[robot_id]), abs=1e-9
        )
    # Hand check for robot_0: T_world_component is a pure translation of
    # (-9, -18, -2), so its submap at component x=2 lands at world (-7, -18, -2).
    (robot_0_submap,) = [v for k, v in submaps.items() if k.startswith("robot_0/")]
    assert [
        row[3] for row in robot_0_submap["T_component_submap"][:3]
    ] == pytest.approx([-7.0, -18.0, -2.0])

    # Chunks pass through untouched: the browser caches them by hash.
    member_chunks = {}
    for robot_id in ("robot_0", "robot_1"):
        member_view = client.get(
            f"/api/autonomy/replicas/components/view/{session}",
            params={"component_id": setup["components"][robot_id]},
        ).json()
        for chunk in member_view["chunks"]:
            member_chunks[chunk["sha256"]] = chunk
    assert {chunk["sha256"]: chunk for chunk in view["chunks"]} == member_chunks
    for submap in selected_component["submaps"]:
        for chunk in submap["chunks"]:
            assert chunk == member_chunks[chunk["sha256"]]


def test_composite_publication_identity_follows_placement(setup):
    client, session, map_service = (
        setup["client"],
        setup["session"],
        setup["map_service"],
    )

    def view():
        return client.get(
            f"/api/autonomy/replicas/components/view/{session}",
            params={"component_id": composite_id(session)},
        ).json()

    first = view()
    assert view()["snapshot_id"] == first["snapshot_id"]
    map_service.transforms["robot_1"] = (-3.0, 4.5, 0.0)
    moved = view()
    assert moved["snapshot_id"] != first["snapshot_id"]
    assert (
        moved["selected"]["geometry_revision"] == first["selected"]["geometry_revision"]
    )
    assert moved["chunks"] == first["chunks"]


def test_oversized_composite_answers_413_and_is_listed_unavailable(setup, monkeypatch):
    client, session = setup["client"], setup["session"]
    monkeypatch.setattr(replica_views, "MAX_SUBMAPS", 1)
    response = client.get(
        f"/api/autonomy/replicas/components/view/{session}",
        params={"component_id": composite_id(session)},
    )
    assert response.status_code == 413
    assert "budget" in response.json()["error"]
    entries = client.get("/api/autonomy/replicas/components").json()["components"]
    (entry,) = [entry for entry in entries if entry.get("composite")]
    assert entry["available"] is False
    assert entry["status"] == "conflict"
    assert "budget" in entry["detail"]
    # Member components are unaffected.
    assert all(entry["available"] for entry in entries if not entry.get("composite"))


def test_composite_view_is_only_served_for_the_active_mission(setup, monkeypatch):
    client, session = setup["client"], setup["session"]
    other = str(uuid4())
    assert (
        client.get(
            f"/api/autonomy/replicas/components/view/{other}",
            params={"component_id": composite_id(other)},
        ).status_code
        == 404
    )
    # A composite id that names another session is not this session's.
    assert (
        client.get(
            f"/api/autonomy/replicas/components/view/{session}",
            params={"component_id": composite_id(other)},
        ).status_code
        == 404
    )


LIVE = "/api/autonomy/replicas/components/live/{session}"


def test_live_frame_places_member_poses_with_the_world_navigation_transform(setup):
    client, session = setup["client"], setup["session"]
    response = client.get(
        LIVE.format(session=session), params={"component_id": composite_id(session)}
    )
    assert response.status_code == 200
    frame = response.json()
    assert frame["frame_id"] == "deployment"
    assert frame["component_id"] == composite_id(session)
    assert frame["solution_order"] == [0, -1]
    robots = {robot["robot_id"]: robot for robot in frame["robots"]}
    assert sorted(robots) == ["robot_0", "robot_1"]
    for robot_id, robot in robots.items():
        assert robot["component_id"] == composite_id(session)
        assert robot["solution_order"] == [0, -1]
        assert robot["member"] == {
            "component_id": setup["components"][robot_id],
            "solution_order": [0, -1],
        }
        assert np.array(robot["T_component_navigation"]) == pytest.approx(
            se2(*TRANSFORMS[robot_id]), abs=1e-9
        )
        # The pose stays raw in the navigation frame; the transform the
        # browser applies to it yields the 2D fleet map's world position.
        assert robot["pose"] == {"x": 2.0, "y": 3.0, "z": 0.0, "yaw": 0.0}
    placed = np.array(robots["robot_0"]["T_component_navigation"]) @ np.array(
        [2.0, 3.0, 0.0, 1.0]
    )
    assert placed[:2] == pytest.approx([-2.0, 4.0])


def test_live_frame_drops_a_member_with_stale_or_foreign_authority(setup):
    client, session, registry = setup["client"], setup["session"], setup["registry"]
    setup["map_service"].transforms["robot_2"] = (0.0, 0.0, 0.0)
    # Stale telemetry keeps robot_1's geometry placed but hides its pose; a
    # robot whose authority moved to another component leaves the composite.
    registry.robots["robot_1"].live_mapping_received_at -= 4
    registry.robots["robot_0"].live_mapping["component_id"] = "other"
    frame = client.get(
        LIVE.format(session=session), params={"component_id": composite_id(session)}
    ).json()
    assert [robot["robot_id"] for robot in frame["robots"]] == ["robot_2"]
    view = client.get(
        f"/api/autonomy/replicas/components/view/{session}",
        params={"component_id": composite_id(session)},
    ).json()
    assert [member["robot_id"] for member in view["members"]] == [
        "robot_1",
        "robot_2",
    ]
    # Without two members left there is no composite to overlay.
    registry.robots["robot_2"].live_mapping["component_id"] = "other"
    assert (
        client.get(
            LIVE.format(session=session), params={"component_id": composite_id(session)}
        ).status_code
        == 409
    )


def test_composite_goal_is_converted_to_the_member_navigation_frame(setup):
    client, session, sink = setup["client"], setup["session"], setup["sink"]
    world_goal = dict(x=4, y=5, z=1, yaw=0)
    response = client.post(
        LIVE.format(session=session) + "/goal",
        json=dict(
            robot_id="robot_0",
            component_id=composite_id(session),
            solution_order=[0, -1],
            goal=world_goal,
        ),
    )
    assert response.status_code == 200, response.json()
    sent = sink.send_json.call_args.args[0]
    assert sent["type"] == "plan_objective"
    goal = sent["goal"]
    # inv(T_world_navigation) applied once: SE2(1, 2, 90deg) maps world (4, 5)
    # to navigation (3, -3), heading turned back by 90 degrees.
    assert goal["x"] == pytest.approx(3.0)
    assert goal["y"] == pytest.approx(-3.0)
    assert goal["z"] == pytest.approx(1.0)
    assert goal["yaw"] == pytest.approx(-math.pi / 2)
    assert goal["frame_id"] == "robot_0/map"
    assert goal["mission_id"] == session
    # Dispatched as a goal in the member's own component, fenced by its own
    # frame revision; the robot re-resolves component_goal itself.
    assert goal["component_id"] == setup["components"]["robot_0"]
    assert goal["solution_order"] == [0, -1]
    assert goal["deployment_goal"] == dict(x=4.0, y=5.0, z=1.0, yaw=0.0)
    component_goal = goal["component_goal"]
    assert component_goal["x"] == pytest.approx(13.0)
    assert component_goal["y"] == pytest.approx(23.0)
    assert component_goal["z"] == pytest.approx(3.0)
    assert component_goal["yaw"] == pytest.approx(0.0)
    expected = np.linalg.inv(expected_world_component("robot_0")) @ np.array(
        [4.0, 5.0, 1.0, 1.0]
    )
    assert [component_goal["x"], component_goal["y"], component_goal["z"]] == (
        pytest.approx(expected[:3].tolist())
    )


@pytest.mark.parametrize("change,status", [("stale", 409), ("member", 409)])
def test_invalid_composite_goals_do_not_dispatch(setup, change, status):
    client, session, sink = setup["client"], setup["session"], setup["sink"]
    command = dict(
        robot_id="robot_0",
        component_id=composite_id(session),
        solution_order=[0, -1],
        goal=dict(x=1, y=2),
    )
    if change == "stale":
        command["solution_order"] = [1, 0]
    else:
        command["robot_id"] = "robot_2"
    assert (
        client.post(LIVE.format(session=session) + "/goal", json=command).status_code
        == status
    )
    sink.send_json.assert_not_called()
