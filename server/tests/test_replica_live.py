from __future__ import annotations

from unittest.mock import AsyncMock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from autonomy.contracts import IDENTITY_SE3
from swarmdeck_server.api import replica_live, replica_views
from swarmdeck_server.fleet.registry import Registry


def payload():
    return dict(
        robot_id="r0",
        mission_id="mission",
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
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "mission")

    class Catalogue:
        def __init__(self):
            self.snapshot_id = "view-snapshot"
            self.solution_order = None

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
            return result

    catalogue = Catalogue()
    monkeypatch.setattr(replica_views, "current_catalogue", lambda session: catalogue)
    app = FastAPI()
    app.include_router(replica_views.router)
    with TestClient(app) as client:
        yield client, registry, sink, catalogue


BASE = "/api/autonomy/replicas/components/live/mission"


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


def test_complete_objective_phase_is_forwarded_and_terminal_state_clears_it(setup):
    _, registry, _, _ = setup
    continuation = dict(
        objective="navigate",
        phase="following_final",
        evidence_source="mgg_native",
    )
    registry.update_state(
        dict(robot_id="r0", nav_status="active", objective_continuation=continuation)
    )
    assert registry.robots["r0"].to_state()["objective_continuation"] == continuation
    registry.update_state(dict(robot_id="r0", nav_status="succeeded"))
    assert registry.robots["r0"].to_state()["objective_continuation"] is None


def test_long_split_paths_retain_destinations_in_fleet_state(setup):
    _, registry, _, _ = setup
    path = [dict(x=i / 10, y=0) for i in range(1001)]
    registry.update_state(dict(
        robot_id="r0", nav_status="active",
        global_planned_path=path, local_planned_path=path,
    ))
    robot = registry.robots["r0"]
    for shown in (robot.global_planned_path, robot.local_planned_path):
        assert len(shown) == 200
        assert shown[0] == path[0]
        assert shown[-1] == path[-1]
    assert len(path) == 1001


@pytest.mark.parametrize("split", [False, True])
def test_explicit_empty_route_clears_previous_route_during_planning(setup, split):
    _, registry, _, _ = setup
    field = "global_planned_path" if split else "planned_path"
    path = [dict(x=0, y=0), dict(x=10, y=0)]
    registry.update_state(dict(robot_id="r0", nav_status="active", **{field: path}))
    assert registry.robots["r0"].global_planned_path == path
    assert registry.robots["r0"].planned_path == path
    registry.update_state(dict(
        robot_id="r0", nav_status="active", goal=dict(x=20, y=0),
        objective_continuation=dict(
            objective="navigate", phase="planning", evidence_source="mgg_native",
        ), **{field: []},
    ))
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
