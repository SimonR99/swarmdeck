from types import SimpleNamespace

import pytest
import numpy as np

from adapters.live_mapping import live_state
from autonomy.contracts import IDENTITY_SE3


def bridge():
    authority = dict(
        robot_id="r0",
        mission_id="mission",
        component_id="component",
        navigation_frame="r0/map",
        solution_order=[0, -1],
        T_component_navigation=IDENTITY_SE3,
    )
    state = dict(pose=dict(x=2, y=3, yaw=0), goal=None, planned_path=[])
    return SimpleNamespace(
        id="r0",
        map_frame="r0/map",
        state=lambda: state,
        _mapping_authority=SimpleNamespace(
            current=lambda: authority,
            clock=lambda: 10.0,
            received_at=9.5,
        ),
    )


def test_navigation_state_is_qualified_without_mutating_original():
    robot = bridge()
    result = live_state(robot)
    assert result["live_mapping"]["pose"] == dict(x=2, y=3, z=0, yaw=0)
    assert result["live_mapping"]["authority_age_s"] == 0.5
    assert "live_mapping" not in robot.state()


def test_qualified_home_survives_live_mapping_validation():
    robot = bridge()
    robot._mapping_authority.current()["home"] = {
        "keyframe_id": "r0/mission/0",
        "T_navigation_home": [
            [1, 0, 0, 4],
            [0, 1, 0, -2],
            [0, 0, 1, 0.3],
            [0, 0, 0, 1],
        ],
    }

    home = live_state(robot)["live_mapping"]["home"]

    assert home["keyframe_id"] == "r0/mission/0"
    assert home["T_navigation_home"][0][3] == 4


def test_malformed_home_rejects_the_live_authority():
    robot = bridge()
    robot._mapping_authority.current()["home"] = {
        "keyframe_id": "r0/mission/0",
        "T_navigation_home": [[1]],
    }

    assert live_state(robot)["live_mapping"] is None


@pytest.mark.parametrize(
    "field,value",
    [
        ("robot_id", "other"),
        ("navigation_frame", "other/map"),
        ("T_component_navigation", [[1]]),
        ("mission_id", ""),
        ("solution_order", None),
        ("solution_order", [1, -1]),
    ],
)
def test_invalid_authority_has_no_live_overlay(field, value):
    robot = bridge()
    robot._mapping_authority.current()[field] = value
    assert live_state(robot)["live_mapping"] is None


def test_stale_authority_and_legacy_bridges_still_send_state():
    robot = bridge()
    robot._mapping_authority.received_at = 1.0
    assert live_state(robot)["live_mapping"] is None
    del robot._mapping_authority
    assert live_state(robot)["pose"] == dict(x=2, y=3, yaw=0)


def test_retained_objective_is_decorated_before_live_metadata():
    robot = bridge()
    robot.objective_planner = SimpleNamespace(
        decorate_state=lambda state: dict(
            state,
            nav_status="active",
            goal=dict(x=40, y=0, yaw=1),
        )
    )
    result = live_state(robot)
    assert result["nav_status"] == "active"
    assert result["live_mapping"]["goal"]["x"] == 40


def test_long_route_display_retains_destination_without_changing_controller_path():
    robot = bridge()
    full_path = [dict(x=i / 10, y=0) for i in range(1001)]
    robot.state()["planned_path"] = full_path
    result = live_state(robot)
    shown = result["live_mapping"]["planned_path"]
    assert len(shown) == 200
    assert shown[0]["x"] == 0
    assert shown[-1]["x"] == 100
    assert all(a["x"] < b["x"] for a, b in zip(shown, shown[1:]))
    assert result["planned_path"] is full_path
    assert len(full_path) == 1001


def stable_route_robot():
    from adapters.exploration import PlannerPath, PlannerPose

    robot = bridge()
    authority = robot._mapping_authority.current()
    authority.update(planning_frame="r0/odom", T_component_planning=IDENTITY_SE3)
    authority["T_component_navigation"] = [
        [0, -1, 0, 10],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    robot._goal_generation = 7
    robot._follow_path_display = (
        7,
        PlannerPath(
            "r0/odom",
            1,
            tuple(PlannerPose(10, i / 100, 0, 0, 0, 0, 1) for i in range(1201)),
        ),
    )
    state = robot.state()
    state.update(nav_status="active", goal=dict(x=10, y=12, yaw=0, frame_id="r0/odom"))
    return robot


def test_stable_route_display_tracks_map_gauge_without_mutating_controller():
    robot = stable_route_robot()
    original = robot._follow_path_display[1]
    result = live_state(robot)
    assert result["goal"]["x"] == pytest.approx(12)
    assert result["goal"]["y"] == pytest.approx(0)
    assert result["goal"]["yaw"] == pytest.approx(-1.5707963267948966)
    assert result["goal"]["frame_id"] == "r0/map"
    assert len(result["planned_path"]) == 200
    assert result["planned_path"][-1] == dict(x=12, y=0, z=0)
    assert result["global_planned_path"] == result["planned_path"]
    robot._mapping_authority.current()["T_component_navigation"][0][3] += 0.27
    after = live_state(robot)
    assert after["planned_path"][-1]["y"] == pytest.approx(0.27)
    assert robot._follow_path_display[1] is original
    assert len(original.poses) == 1201
    assert robot.state()["goal"]["frame_id"] == "r0/odom"


def test_rolling_home_keeps_global_route_while_local_chunk_advances():
    from adapters.exploration import PlannerPath, PlannerPose

    robot = stable_route_robot()
    global_plan = robot._follow_path_display[1]
    robot.objective_planner = SimpleNamespace(
        decorate_state=lambda state: state,
        global_display_plan=lambda: global_plan,
    )
    first_local = PlannerPath(
        "r0/odom",
        2,
        tuple(PlannerPose(10, y, 0, 0, 0, 0, 1) for y in (0.0, 2.0, 4.0)),
    )
    robot._follow_path_display = (7, first_local)

    first = live_state(robot)
    assert first["global_planned_path"][-1] == dict(x=12, y=0, z=0)
    assert first["local_planned_path"][-1] == dict(x=4, y=0, z=0)
    assert first["planned_path"] == first["local_planned_path"]

    second_local = PlannerPath(
        "r0/odom",
        3,
        tuple(PlannerPose(10, y, 0, 0, 0, 0, 1) for y in (4.0, 6.0, 8.0)),
    )
    robot._follow_path_display = (7, second_local)
    second = live_state(robot)

    assert second["global_planned_path"] == first["global_planned_path"]
    assert second["local_planned_path"][-1] == dict(x=8, y=0, z=0)
    assert second["planned_path"] == second["local_planned_path"]


def test_rolling_home_hides_completed_local_chunk_while_refining():
    robot = stable_route_robot()
    global_plan = robot._follow_path_display[1]
    robot.objective_planner = SimpleNamespace(
        decorate_state=lambda state: {
            **state,
            "nav_status": "active",
            "objective_continuation": {
                "objective": "return_home",
                "phase": "planning",
            },
        },
        global_display_plan=lambda: global_plan,
    )
    robot.planned_path = [{"x": 99, "y": 99}]

    result = live_state(robot)

    assert result["planned_path"] == []
    assert result["local_planned_path"] == []
    assert result["global_planned_path"][-1] == dict(x=12, y=0, z=0)


def test_stable_route_has_fixed_component_geometry_across_rotating_map_gauge():
    robot = stable_route_robot()

    def in_component(result, field):
        transform = np.asarray(result["live_mapping"]["T_component_navigation"])
        point = result["live_mapping"][field][-1]
        return transform @ np.array([point["x"], point["y"], point.get("z", 0), 1])

    before = live_state(robot)
    fixed_endpoint = in_component(before, "planned_path")
    assert fixed_endpoint[:2] == pytest.approx([10, 12])

    # A SLAM gauge correction changes map<-odom, including rotation. The
    # controller route remains in odom; both 2D map coordinates and their 3D
    # component projection must still describe the same physical endpoint.
    robot._mapping_authority.current()["T_component_navigation"] = [
        [0, 1, 0, -4],
        [-1, 0, 0, 7],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    after = live_state(robot)
    assert after["planned_path"][-1] != before["planned_path"][-1]
    assert in_component(after, "planned_path") == pytest.approx(fixed_endpoint)
    assert robot._follow_path_display[1].poses[-1].x == 10
    assert robot._follow_path_display[1].poses[-1].y == 12


def test_missing_stable_transform_hides_unqualified_display():
    robot = stable_route_robot()
    robot._mapping_authority.current().pop("T_component_planning")
    result = live_state(robot)
    assert result["goal"] is None
    assert result["planned_path"] == []
    assert result["global_planned_path"] == []


def test_replaced_generation_never_replays_retained_route():
    robot = stable_route_robot()
    robot._goal_generation += 1
    robot.state().update(goal=None, planned_path=[], nav_status="idle")
    result = live_state(robot)
    assert result["planned_path"] == []
    assert result["goal"] is None
