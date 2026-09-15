import importlib.util
from pathlib import Path

from autonomy.contracts import IDENTITY_SE3
from autonomy.live_mapping import validate_live_mapping


def acceptance_module():
    path = Path(__file__).with_name("navigation_home_acceptance.py")
    spec = importlib.util.spec_from_file_location("navigation_home_acceptance", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_harness_reads_the_validated_live_home_contract():
    value = {
        "robot_id": "robot_0",
        "mission_id": "mission",
        "component_id": "component",
        "navigation_frame": "robot_0/map_frame",
        "solution_order": [0, -1],
        "T_component_navigation": IDENTITY_SE3,
        "authority_age_s": 0.1,
        "pose": {"x": 1, "y": 2, "yaw": 0},
        "goal": None,
        "planned_path": [],
        "global_planned_path": [],
        "local_planned_path": [],
        "home": {
            "keyframe_id": "robot_0/mission/0",
            "T_navigation_home": [
                [1, 0, 0, 4],
                [0, 1, 0, -2],
                [0, 0, 1, 0.3],
                [0, 0, 0, 1],
            ],
        },
    }

    qualified = validate_live_mapping(value, "robot_0")

    assert acceptance_module().home_target(qualified) == (4.0, -2.0, 0.3)


def test_rolling_home_ignores_local_controller_success():
    completed = acceptance_module().objective_completed

    assert not completed("home", "succeeded", 4, ["following_local"], "following_local")
    assert not completed("home", "succeeded", 4, ["following_local"], None, True)
    assert completed(
        "home",
        "succeeded",
        4,
        ["following_local", "following_final"],
        None,
        True,
    )
    assert completed("navigate", "succeeded", 1, [])
    assert not completed("home", "succeeded", 0, ["following_final"])
    assert completed("home", "succeeded", 1, [])
    assert not completed("home", "succeeded", 1, [], "planning")
