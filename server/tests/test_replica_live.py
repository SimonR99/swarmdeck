from __future__ import annotations

from unittest.mock import AsyncMock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from autonomy.contracts import IDENTITY_SE3
from autonomy.live_mapping import validate_live_mapping
from autonomy.map_epochs import robot_run_id
from swarmdeck_server.api import replica_live, replica_views
from swarmdeck_server.fleet.registry import Registry

MISSION = "00000000-0000-4000-8000-000000000001"
RUN = robot_run_id(MISSION, "r0", 0)


def payload():
    return dict(
        robot_id="r0",
        mission_id=MISSION,
        robot_map_epoch=0,
        run_id=RUN,
        component_id="component",
        navigation_frame="r0/map",
        solution_order=[0, -1],
        T_component_navigation=IDENTITY_SE3,
        authority_age_s=0.2,
        pose=dict(x=2, y=3, yaw=0),
        goal=None,
        planned_path=[dict(x=2, y=3)],
        global_planned_path=[],
        local_planned_path=[],
    )


@pytest.fixture
def setup(monkeypatch):
    registry = Registry()
    sink = AsyncMock()
    registry.hello(
        dict(robot_id="r0", capabilities=["navigate", "plan_objective"]), sink
    )
    registry.update_state(
        dict(robot_id="r0", pose=dict(x=900, y=800), live_mapping=payload())
    )
    monkeypatch.setattr(replica_live, "registry", registry)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", MISSION)

    class Catalogue:
        def __init__(self):
            self.snapshot_id = "view-snapshot"
            self.solution_order = None
            self.solution_orders = None

        def view(self, session, component):
            if component != "component":
                raise KeyError(component)
            result = {
                "snapshot_id": self.snapshot_id,
                "solution_order_known": hasattr(self, "solution_order"),
                "selected": {"frame_id": "verified_component"},
            }
            if hasattr(self, "solution_order"):
                result["solution_order"] = self.solution_order
            if self.solution_orders is not None:
                result["solution_orders"] = self.solution_orders
            return result

    catalogue = Catalogue()
    monkeypatch.setattr(replica_views, "current_catalogue", lambda session: catalogue)
    app = FastAPI()
    app.include_router(replica_views.router)
    with TestClient(app) as client:
        yield client, registry, sink, catalogue


BASE = f"/api/autonomy/replicas/components/live/{MISSION}"


def test_live_overlay_uses_raw_navigation_pose_and_authority_age(setup):
    client, registry, _, _ = setup
    registry.robots["r0"].live_mapping_received_at -= 0.5
    response = client.get(BASE, params={"component_id": "component"})
    assert response.status_code == 200
    value = response.json()
    assert value["frame_id"] == "verified_component"
    assert value["solution_order"] == [0, -1]
    robot = value["robots"][0]
    assert robot["pose"]["x"] == 2
    assert 0.7 <= robot["freshness"]["pose_s"] < 1.5
    assert robot["freshness"]["goal_s"] is None


def test_live_api_exposes_qualified_home_schema(setup):
    client, registry, _, _ = setup
    value = payload()
    value["home"] = {
        "keyframe_id": f"r0/{RUN}/0",
        "T_navigation_home": [
            [1, 0, 0, 4],
            [0, 1, 0, -2],
            [0, 0, 1, 0.3],
            [0, 0, 0, 1],
        ],
    }
    registry.robots["r0"].live_mapping = validate_live_mapping(value, "r0")

    response = client.get(BASE, params={"component_id": "component"})

    assert response.status_code == 200
    home = response.json()["robots"][0]["home"]
    assert home["keyframe_id"] == f"r0/{RUN}/0"
    assert [row[3] for row in home["T_navigation_home"][:3]] == [4, -2, 0.3]


def test_wrong_component_home_authority_is_not_exposed(setup):
    client, registry, _, _ = setup
    value = payload()
    value["component_id"] = "other"
    value["home"] = {
        "keyframe_id": f"r0/{RUN}/0",
        "T_navigation_home": IDENTITY_SE3,
    }
    registry.robots["r0"].live_mapping = validate_live_mapping(value, "r0")

    assert client.get(BASE, params={"component_id": "component"}).status_code == 404


@pytest.mark.parametrize(
    "change", ["stale", "mission", "component", "disconnect", "hello"]
)
def test_unqualified_telemetry_is_not_rendered(setup, change):
    client, registry, sink, _ = setup
    robot = registry.robots["r0"]
    if change == "stale":
        robot.live_mapping_received_at -= 4
    elif change in ("mission", "component"):
        robot.live_mapping[change + "_id"] = "other"
    elif change == "disconnect":
        registry.disconnect("r0", sink)
    else:
        registry.hello(dict(robot_id="r0"), sink)
    assert client.get(BASE, params={"component_id": "component"}).status_code == 404


def test_old_socket_cleanup_does_not_clear_new_live_authority(setup):
    _, registry, _, _ = setup
    registry.disconnect("r0", object())
    assert registry.robots["r0"].live_mapping is not None


def test_goal_is_inverted_once_and_keeps_component_anchor(setup):
    client, registry, sink, _ = setup
    registry.robots["r0"].nav_failure_reason = "previous planner rejection"
    # Navigation -> component: 90deg yaw plus a nonzero translation/height.
    registry.robots["r0"].live_mapping["T_component_navigation"] = (
        (0, -1, 0, 10),
        (1, 0, 0, 20),
        (0, 0, 1, 2),
        (0, 0, 0, 1),
    )
    goal = dict(x=7, y=24, z=3, yaw=0)
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[0, -1],
            goal=goal,
        ),
    )
    assert response.status_code == 200
    sent = sink.send_json.call_args.args[0]
    assert sent["type"] == "plan_objective"
    assert sent["goal"]["component_goal"] == goal
    assert sent["goal"]["frame_id"] == "r0/map"
    assert sent["goal"]["solution_order"] == [0, -1]
    assert sent["goal"]["x"] == 4
    assert sent["goal"]["y"] == 3
    assert sent["goal"]["z"] == 1
    assert sent["goal"]["yaw"] == pytest.approx(-1.5707963267948966)
    assert registry.robots["r0"].nav_failure_reason is None
    # Ordinary goals carry no exploration option.
    assert "explore_if_unknown" not in sent


def test_explore_if_unknown_reaches_the_robot(setup):
    client, _registry, sink, _ = setup
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[0, -1],
            goal=dict(x=7, y=24, z=0, yaw=0),
            explore_if_unknown=True,
        ),
    )
    assert response.status_code == 200
    sent = sink.send_json.call_args.args[0]
    assert sent["type"] == "plan_objective"
    assert sent["explore_if_unknown"] is True


def test_new_publication_with_same_frame_revision_keeps_goal_valid(setup):
    client, _, sink, catalogue = setup
    catalogue.snapshot_id = "new-publication"
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[0, -1],
            goal=dict(x=1, y=2),
        ),
    )
    assert response.status_code == 200
    sink.send_json.assert_awaited_once()


def test_changed_frame_revision_rejects_old_overlay_and_goal(setup):
    client, _, sink, catalogue = setup
    catalogue.solution_order = [2, 3]
    assert client.get(BASE, params={"component_id": "component"}).status_code == 404
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[0, -1],
            goal=dict(x=1, y=2),
        ),
    )
    assert response.status_code == 409
    assert response.json()["detail"] == "Displayed component frame is stale"
    sink.send_json.assert_not_called()


def test_authority_lagging_its_replica_by_solver_reports_is_listed(setup):
    # The authority is product-gated and names the last published product's
    # frame; the replica follows the solver. Benchbot 2026-09-19 (mission
    # 1a8cc114): robot_0's authority at [413, 0] against its replica at
    # [443, 0] hid it from the 3D view. One optimizer, a later clock: the same
    # frame a few reports apart.
    client, registry, _, catalogue = setup
    catalogue.solution_order = [443, 0]
    registry.robots["r0"].live_mapping["solution_order"] = [413, 0]
    response = client.get(BASE, params={"component_id": "component"})
    assert response.status_code == 200
    value = response.json()
    assert value["solution_order"] == [443, 0]
    (robot,) = value["robots"]
    assert robot["robot_id"] == "r0"
    # The robot reports its own authority's order; its pose is placed with
    # that authority's transform, a few solver steps behind the drawn map.
    assert robot["solution_order"] == [413, 0]


def test_authority_lagging_its_own_publication_of_a_merged_view_is_listed(setup):
    # A merged view reports the newest publisher's order and each publisher's
    # own order under solution_orders; the authority trails the robot's own
    # publication, which trails the view.
    client, registry, _, catalogue = setup
    catalogue.solution_order = [450, 0]
    catalogue.solution_orders = {"r0": [443, 0], "r1": [450, 0]}
    registry.robots["r0"].live_mapping["solution_order"] = [413, 0]
    response = client.get(BASE, params={"component_id": "component"})
    assert response.status_code == 200
    assert [robot["robot_id"] for robot in response.json()["robots"]] == ["r0"]


def test_equal_real_solution_orders_are_listed(setup):
    client, registry, _, catalogue = setup
    catalogue.solution_order = [443, 0]
    registry.robots["r0"].live_mapping["solution_order"] = [443, 0]
    response = client.get(BASE, params={"component_id": "component"})
    assert response.status_code == 200
    assert response.json()["robots"][0]["solution_order"] == [443, 0]


@pytest.mark.parametrize(
    "view_order,authority_order",
    [
        ([443, 0], [443, 1]),
        ([443, 0], [413, 1]),
        ([443, 0], [0, -1]),
        (None, [443, 0]),
    ],
)
def test_authority_in_another_optimizer_or_before_the_solver_is_not_listed(
    setup, view_order, authority_order
):
    # Another optimizer's solution, or the pre-optimizer sentinel on one side
    # only (the catalogue writes it as None), is not the displayed frame.
    client, registry, _, catalogue = setup
    catalogue.solution_order = view_order
    registry.robots["r0"].live_mapping["solution_order"] = authority_order
    assert client.get(BASE, params={"component_id": "component"}).status_code == 404


def test_goal_for_a_lagging_authority_is_dispatched_in_its_own_frame(setup):
    # The server converts the click with the robot's own authority transform
    # and sends that authority's order, so the robot's own fence
    # (adapters/objective_planning._goal_solution_order_error) admits it.
    client, registry, sink, catalogue = setup
    catalogue.solution_order = [443, 0]
    registry.robots["r0"].live_mapping["solution_order"] = [413, 0]
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[443, 0],
            goal=dict(x=1, y=2),
        ),
    )
    assert response.status_code == 200
    sent = sink.send_json.call_args.args[0]["goal"]
    assert sent["solution_order"] == [413, 0]
    assert sent["component_goal"] == dict(x=1.0, y=2.0, z=0.0, yaw=0.0)


@pytest.mark.parametrize(
    "first,second,expected",
    [
        ((0, -1), (0, -1), True),
        ((443, 0), (443, 0), True),
        ((413, 0), (443, 0), True),
        ((443, 0), (413, 0), True),
        ([413, 0], (443, 0), True),
        ((443, 0), (443, 1), False),
        ((413, 0), (443, 1), False),
        ((0, -1), (443, 0), False),
        ((443, 0), (0, -1), False),
    ],
)
def test_compatible_solution_orders(first, second, expected):
    assert replica_live.compatible_solution_orders(first, second) is expected


def test_missing_frame_revision_is_not_treated_as_initial_sentinel(setup):
    client, _, sink, catalogue = setup
    del catalogue.solution_order
    response = client.post(
        BASE + "/goal",
        json=dict(
            robot_id="r0",
            component_id="component",
            solution_order=[0, -1],
            goal=dict(x=1, y=2),
        ),
    )
    assert response.status_code == 409
    sink.send_json.assert_not_called()


@pytest.mark.parametrize(
    "change,status",
    [
        ("stale", 409),
        ("component", 409),
        ("robot", 404),
        ("unsupported", 409),
        ("invalid", 422),
        ("disconnected", 409),
    ],
)
def test_invalid_goals_do_not_dispatch(setup, change, status):
    client, registry, sink, _ = setup
    command = dict(
        robot_id="r0",
        component_id="component",
        solution_order=[0, -1],
        goal=dict(x=1, y=2),
    )
    if change == "stale":
        registry.robots["r0"].live_mapping_received_at -= 4
    elif change == "component":
        command["component_id"] = "other"
    elif change == "robot":
        command["robot_id"] = "unknown"
    elif change == "unsupported":
        registry.robots["r0"].capabilities = []
    elif change == "invalid":
        command["goal"]["x"] = "NaN"
    else:
        registry.disconnect("r0")
    assert client.post(BASE + "/goal", json=command).status_code == status
    sink.send_json.assert_not_called()


def test_legacy_state_clears_cached_live_metadata(setup):
    _, registry, _, _ = setup
    registry.update_state(dict(robot_id="r0", pose=dict(x=5, y=6)))
    assert registry.robots["r0"].live_mapping is None


@pytest.mark.parametrize("split", [False, True])
def test_explicit_empty_route_clears_previous_route_during_planning(setup, split):
    _, registry, _, _ = setup
    field = "global_planned_path" if split else "planned_path"
    path = [dict(x=0, y=0), dict(x=10, y=0)]
    registry.update_state(dict(robot_id="r0", nav_status="active", **{field: path}))
    assert registry.robots["r0"].global_planned_path == path
    assert registry.robots["r0"].planned_path == path
    registry.update_state(
        dict(
            robot_id="r0",
            nav_status="active",
            goal=dict(x=20, y=0),
            **{field: []},
        )
    )
    assert registry.robots["r0"].global_planned_path == []
    assert registry.robots["r0"].planned_path == []


@pytest.mark.parametrize("status", ["failed", "cancelled", "succeeded"])
def test_native_navigation_failure_reason_is_cleared_by_terminal_state(setup, status):
    _, registry, _, _ = setup
    reason = "current pose rejected: no mapped ground support"
    registry.update_state(
        dict(robot_id="r0", nav_status="failed", nav_failure_reason=reason)
    )
    assert registry.robots["r0"].to_state()["nav_failure_reason"] == reason

    registry.update_state(dict(robot_id="r0", nav_status=status))
    assert registry.robots["r0"].to_state()["nav_failure_reason"] is None


def test_native_navigation_failure_reason_is_cleared_by_reconnect(setup):
    _, registry, sink, _ = setup
    registry.update_state(
        dict(
            robot_id="r0",
            nav_status="failed",
            nav_failure_reason="native planner rejection",
        )
    )
    registry.hello(dict(robot_id="r0"), sink)
    assert registry.robots["r0"].to_state()["nav_failure_reason"] is None


def test_navigation_failure_reason_is_bounded_at_registry_boundary(setup):
    _, registry, _, _ = setup
    registry.update_state(
        dict(robot_id="r0", nav_status="failed", nav_failure_reason="x" * 1000)
    )
    assert len(registry.robots["r0"].nav_failure_reason) == 512


def test_rejects_oversized_paths_and_invalid_transforms(setup):
    _, registry, _, _ = setup
    for field, value in (
        ("planned_path", [dict(x=1, y=2)] * 201),
        ("T_component_navigation", [[1]]),
    ):
        live = payload()
        live[field] = value
        registry.update_state(dict(robot_id="r0", live_mapping=live))
        assert registry.robots["r0"].live_mapping is None
