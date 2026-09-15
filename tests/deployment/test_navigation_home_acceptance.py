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


def test_endpoint_evidence_retains_worst_qualified_xy_error():
    module = acceptance_module()
    summary = {
        "local_endpoint_error_m": None,
        "local_endpoint_error_m_invalid": False,
    }

    module.track_path_endpoint_error(
        summary,
        {"local_planned_path": [{"x": 0.2, "y": 0.0}]},
        (0.0, 0.0, 0.0),
        "local_planned_path",
        "local_endpoint_error_m",
    )
    module.track_path_endpoint_error(
        summary,
        {"local_planned_path": [{"x": 0.0, "y": 0.0}]},
        (0.0, 0.0, 0.0),
        "local_planned_path",
        "local_endpoint_error_m",
    )

    assert summary["local_endpoint_error_m"] == 0.2
    assert not module.endpoint_error_is_acceptable(summary, "local_endpoint_error_m")


def test_missing_endpoint_does_not_erase_observed_maximum():
    module = acceptance_module()
    summary = {
        "global_endpoint_error_m": 0.04,
        "global_endpoint_error_m_invalid": False,
    }

    module.track_path_endpoint_error(
        summary,
        {"global_planned_path": []},
        (0.0, 0.0, 0.0),
        "global_planned_path",
        "global_endpoint_error_m",
    )

    assert summary["global_endpoint_error_m"] == 0.04
    assert module.endpoint_error_is_acceptable(summary, "global_endpoint_error_m")


def test_invalid_or_nonfinite_endpoint_evidence_fails_closed():
    module = acceptance_module()
    for endpoint in ({"x": float("nan"), "y": 0.0}, {"x": "bad", "y": 0.0}, {}):
        summary = {
            "local_endpoint_error_m": 0.0,
            "local_endpoint_error_m_invalid": False,
        }
        module.track_path_endpoint_error(
            summary,
            {"local_planned_path": [endpoint]},
            (0.0, 0.0, 0.0),
            "local_planned_path",
            "local_endpoint_error_m",
        )

        assert summary["local_endpoint_error_m"] == 0.0
        assert summary["local_endpoint_error_m_invalid"]
        assert not module.endpoint_error_is_acceptable(
            summary, "local_endpoint_error_m"
        )
