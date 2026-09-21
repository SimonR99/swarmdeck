import json
import math
import os
import time

import pytest

from adapters.test.ros.onboard_return_home_observer import (
    _safe_baseline_robot,
    authority_home,
    load_authority,
    summarize_return_home,
)
from autonomy.map_epochs import robot_run_id

MISSION = "88492d31-5c28-4de9-bbbd-8bfb1a74014d"
RUN = robot_run_id(MISSION, "robot_3", 0)


def authority():
    return {
        "robot_id": "robot_3",
        "mission_id": MISSION,
        "robot_map_epoch": 0,
        "run_id": RUN,
        "component_id": "component:test",
        "navigation_frame": "robot_3/map_frame",
        "correction_revision": 2,
        "map_epoch": 1,
        "mapping_graph_revision": 8,
        "geometry_revision": "a" * 64,
        "home": {
            "keyframe_id": f"robot_3/{RUN}/0",
            "T_navigation_home": [
                [1, 0, 0, -0.25],
                [0, 1, 0, 0.5],
                [0, 0, 1, 0.1],
                [0, 0, 0, 1],
            ],
        },
    }


def fleet_sample(
    pose,
    nav_status,
    path=(),
    exploration_status="stopped",
    transform=None,
):
    return {
        "fleet": {
            "ok": True,
            "data": {
                "robots": [
                    {
                        "robot_id": "robot_3",
                        "online": True,
                        "capabilities": ["navigate", "plan_objective"],
                        "pose": {"x": pose[0], "y": pose[1]},
                        "exploration_status": exploration_status,
                        "nav_status": nav_status,
                        "goal": None,
                        "global_planned_path": list(path),
                        "local_planned_path": [],
                    }
                ]
            },
        },
        "map_status": {
            "ok": True,
            "data": {
                "transforms": {"robot_3": transform or {"x": 0.0, "y": 0.0, "yaw": 0.0}}
            },
        },
    }


def test_loads_fresh_plain_json_authority(tmp_path):
    fixture = tmp_path / "authority.json"
    fixture.write_text(json.dumps(authority()))

    loaded, age = load_authority(fixture, 10.0)
    home = authority_home(loaded, "robot_3", MISSION)

    assert age < 1.0
    assert home["x"] == -0.25
    assert home["y"] == 0.5
    assert home["landmark_id"] == f"robot_3/{RUN}/0"

    os.utime(fixture, (time.time() - 20, time.time() - 20))
    with pytest.raises(ValueError, match="old"):
        load_authority(fixture, 10.0)


def test_summary_requires_full_route_success_and_arrival_at_authority_home():
    home = authority_home(authority(), "robot_3", MISSION)
    transform = {"x": -14.0, "y": 6.0, "yaw": -math.pi / 2}
    route = [
        {"x": -18.0, "y": 2.0},
        {"x": -16.0, "y": 4.0},
        {"x": -13.5, "y": 6.25},
    ]
    result = summarize_return_home(
        "robot_3",
        home,
        fleet_sample((4.0, 4.0), "idle", transform=transform),
        [
            fleet_sample((-18.0, 2.0), "active", route, transform=transform),
            fleet_sample((-13.48, 6.2), "succeeded", transform=transform),
        ],
        0.5,
    )

    assert result["multi_waypoint_route_observed"]
    assert result["route_endpoint_matches_authority_home"]
    assert result["server_frame_arrival_consistent"]
    assert result["server_evidence_consistent_with_objective_execution"]


def test_blocked_or_failed_motion_is_not_objective_success():
    home = authority_home(authority(), "robot_3", MISSION)
    result = summarize_return_home(
        "robot_3",
        home,
        fleet_sample((4.0, 4.0), "idle", transform={"x": 0, "y": 0, "yaw": 0}),
        [
            fleet_sample(
                (3.5, 3.5),
                "failed",
                exploration_status="blocked",
                transform={"x": 0, "y": 0, "yaw": 0},
            )
        ],
        0.5,
    )

    assert result["blocked_reported"]
    assert result["terminal_navigation_status_before_final_stop"] == "failed"
    assert not result["server_evidence_consistent_with_objective_execution"]


def test_baseline_requires_stopped_idle_onboard_planner():
    safe = fleet_sample((4.0, 4.0), "idle")
    assert _safe_baseline_robot(safe, "robot_3")["online"]
    unsafe = fleet_sample((4.0, 4.0), "active", [{"x": 3.0, "y": 3.0}])
    with pytest.raises(Exception, match="idle navigation"):
        _safe_baseline_robot(unsafe, "robot_3")
