from copy import deepcopy
from datetime import datetime, timedelta, timezone

import pytest

from adapters.test.ros.return_home_evidence import AUTHORITY_FIELDS, analyze
from autonomy.map_epochs import robot_run_id

BASE = datetime(2026, 9, 10, 10, 0, tzinfo=timezone.utc)
MISSION = "126bed69-9327-4d8f-a4b2-075fa7569d8e"
ROBOT = "robot_0"
RUN = robot_run_id(MISSION, ROBOT, 0)


def _iso(seconds: float) -> str:
    return (BASE + timedelta(seconds=seconds)).isoformat()


def _matrix(x: float = 0.0, y: float = 0.0) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _authority() -> dict:
    return {
        "robot_id": ROBOT,
        "mission_id": MISSION,
        "robot_map_epoch": 0,
        "run_id": RUN,
        "component_id": "component:test",
        "map_epoch": 0,
        "mapping_graph_revision": 8,
        "geometry_revision": 4,
        "correction_revision": 2,
        "map_source_stamp": {"sec": 10, "nanosec": 0},
        "navigation_frame": f"{ROBOT}/navigation_frame",
        "T_component_navigation": _matrix(),
        "anchor": {"robot_id": ROBOT},
        "home": {
            "keyframe_id": f"{ROBOT}/{RUN}/0",
            "T_navigation_home": _matrix(),
        },
        "solution_order": [8, ROBOT],
    }


def _fleet(
    at: float, status: str, *, path: bool = False, baseline: bool = False
) -> dict:
    return {
        "observed_at": _iso(at),
        "fleet": {
            "ok": True,
            "data": {
                "robots": [
                    {
                        "robot_id": ROBOT,
                        "online": True,
                        "capabilities": ["navigate", "plan_objective"],
                        "exploration_status": "stopped",
                        "nav_status": status,
                        "goal": None,
                        "global_planned_path": (
                            [] if baseline or not path else [{"x": 2.0}, {"x": 0.0}]
                        ),
                        "local_planned_path": [],
                    }
                ]
            },
        },
    }


def _observer() -> dict:
    return {
        "schema_version": 1,
        "robot_id": ROBOT,
        "mission_id_filter": MISSION,
        "started_at": _iso(0),
        "finished_at": _iso(22),
        "checkpoint": {"phase": "complete", "written_at": _iso(22)},
        "authority_fixture": {
            "path": "/evidence/authority.json",
            "age_at_start_s": 0.2,
            "max_age_s": 30.0,
            "raw": _authority(),
        },
        "baseline": _fleet(0.1, "idle", baseline=True),
        "samples": [_fleet(1, "active", path=True), _fleet(20, "succeeded")],
        "objective_activity_observed": True,
        "commands": {
            "return_home": {
                "sent": True,
                "frame_written": True,
                "payload": {"type": "return_home", "robot_id": ROBOT},
            },
            "stop_all": {
                "sent": True,
                "frame_written": True,
                "payload": {"type": "stop_all"},
            },
        },
        "post_stop": _fleet(21, "idle", baseline=True),
        "summary": {
            "initial_server_frame_distance_to_authority_home_m": 2.0,
            "server_evidence_consistent_with_objective_execution": True,
            "post_stop_state": {"all_eligible_reported_stopped_and_inactive": True},
        },
        "errors": [],
    }


def _truth(elapsed: int, x: float, stamp: int | None = None) -> dict:
    return {
        "received_elapsed_s": float(elapsed),
        "stamp_ns": elapsed * 1_000_000_000 if stamp is None else stamp,
        "frame_id": "world",
        "child_frame_id": f"{ROBOT}/base_link",
        "position": {"x": x, "y": 0.0, "z": 0.15},
        "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def _evidence() -> dict:
    recorder_start = BASE - timedelta(seconds=60)
    authority = _authority()
    samples = [_truth(9, 0.0), _truth(10, 0.0)]
    samples.extend(
        _truth(elapsed, 2.0 if elapsed < 80 else 0.1) for elapsed in range(60, 83)
    )
    source = {
        "ground_truth_topic": f"/{ROBOT}/ground_truth",
        "map_authority_topic": f"/{ROBOT}/map_authority",
        "keyframe_metadata_topic": f"/{ROBOT}/keyframes",
        "truth_samples": samples,
        "truth_samples_dropped": 0,
        "invalid_truth_samples": 0,
        "authority_messages": 10,
        "invalid_authority_messages": 0,
        "authority_events_dropped": 0,
        "authority_events": [
            {
                "received_elapsed_s": 55.0,
                "latest_truth_stamp_ns": 10_000_000_000,
                "changed_fields": ["initial"],
                "payload_sha256": "a" * 64,
                "state": {field: authority.get(field) for field in AUTHORITY_FIELDS},
            }
        ],
        "keyframe_metadata_messages": 1,
        "invalid_keyframe_metadata_messages": 0,
        "keyframe_events_dropped": 0,
        "keyframe_events": [
            {
                "received_elapsed_s": 10.0,
                "payload_sha256": "b" * 64,
                "metadata": {
                    "schema": "swarmdeck.keyframe-metadata.v1",
                    "keyframe_id": f"{ROBOT}/{RUN}/0",
                    "stamp_ns": 10_000_000_000,
                    "odom_frame": f"{ROBOT}/odom",
                    "T_odom_keyframe": _matrix(),
                    "component_id": "component:test",
                },
            }
        ],
    }
    return {
        "schema": "swarmdeck.simulation-evidence.v1",
        "started_at_utc": recorder_start.isoformat(),
        "ended_at_utc": _iso(30),
        "requested_duration_s": 1800.0,
        "actual_duration_s": 90.0,
        "stopped_early": True,
        "ros_domain_id_from_environment": "187",
        "robots": {ROBOT: source},
    }


def test_complete_bound_trial_passes():
    result = analyze(_observer(), _evidence(), ROBOT)
    assert result["status"] == "passed", result["reasons"]
    assert result["checks"]["authority_snapshot_matched"]
    assert result["checks"]["precommand_truth_distance_from_home_m"] == 2.0
    assert result["checks"]["settled_duration_s"] >= 1.0
    assert result["checks"]["post_stop_truth_coverage"]


def test_missing_authority_and_mission_cannot_pass():
    observer = _observer()
    observer.pop("mission_id_filter")
    observer.pop("authority_fixture")
    assert analyze(observer, _evidence(), ROBOT)["status"] != "passed"


def test_wrong_recorder_mission_authority_fails():
    evidence = _evidence()
    evidence["robots"][ROBOT]["authority_events"][0]["state"]["mission_id"] = "wrong"
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] != "passed"
    assert any("matching recorder event" in reason for reason in result["reasons"])


def test_null_navigation_frame_fails_without_crashing():
    observer = _observer()
    observer["authority_fixture"]["raw"]["navigation_frame"] = None
    result = analyze(observer, _evidence(), ROBOT)
    assert result["status"] == "failed"
    assert any("navigation_frame" in reason for reason in result["reasons"])


def test_home_must_match_recorded_mission_keyframe_zero():
    evidence = _evidence()
    evidence["robots"][ROBOT]["keyframe_events"][0]["metadata"][
        "keyframe_id"
    ] = f"{ROBOT}/{RUN}/19"
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "inconclusive"
    assert any("keyframe-0" in reason for reason in result["reasons"])


def test_observer_route_consistency_and_post_stop_state_are_required():
    observer = _observer()
    observer["summary"]["server_evidence_consistent_with_objective_execution"] = False
    observer["summary"]["post_stop_state"] = {}
    assert analyze(observer, _evidence(), ROBOT)["status"] == "failed"


def test_robot_already_home_before_command_fails_despite_old_departure():
    evidence = _evidence()
    for sample in evidence["robots"][ROBOT]["truth_samples"]:
        if sample["received_elapsed_s"] >= 60:
            sample["position"]["x"] = 0.1
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "failed"
    assert any("before the objective" in reason for reason in result["reasons"])


def test_repeated_ros_stamp_cannot_fake_post_stop_truth():
    evidence = _evidence()
    for sample in evidence["robots"][ROBOT]["truth_samples"]:
        if sample["received_elapsed_s"] >= 80:
            sample["stamp_ns"] = 80_000_000_000
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "failed"
    assert result["checks"]["nonadvancing_truth_stamps"] > 0


def test_missing_or_late_post_stop_truth_is_inconclusive():
    evidence = _evidence()
    evidence["robots"][ROBOT]["truth_samples"] = [
        sample
        for sample in evidence["robots"][ROBOT]["truth_samples"]
        if sample["received_elapsed_s"] <= 81
    ]
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "inconclusive"
    assert any("Stop All" in reason for reason in result["reasons"])


def test_temporal_gap_across_objective_is_inconclusive():
    evidence = _evidence()
    evidence["robots"][ROBOT]["truth_samples"] = [
        sample
        for sample in evidence["robots"][ROBOT]["truth_samples"]
        if not 66 <= sample["received_elapsed_s"] <= 75
    ]
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "inconclusive"
    assert any("not continuous" in reason for reason in result["reasons"])


def test_later_unrelated_return_cannot_supply_settled_window():
    evidence = _evidence()
    for sample in evidence["robots"][ROBOT]["truth_samples"]:
        if sample["received_elapsed_s"] >= 80:
            sample["position"]["x"] = 2.0
    evidence["robots"][ROBOT]["truth_samples"].extend(
        [_truth(100, 0.1), _truth(101, 0.1), _truth(102, 0.1)]
    )
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "failed"
    assert result["checks"]["settled_sample_count"] == 0
    assert any("outside the home tolerance" in reason for reason in result["reasons"])


def test_malformed_full_pose_truth_fails():
    evidence = _evidence()
    evidence["robots"][ROBOT]["truth_samples"][5].pop("orientation_xyzw")
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "failed"
    assert result["checks"]["malformed_truth_samples"] == 1


def test_authority_change_during_trial_fails():
    evidence = _evidence()
    changed = deepcopy(evidence["robots"][ROBOT]["authority_events"][0])
    changed["received_elapsed_s"] = 70.0
    changed["state"]["correction_revision"] = 3
    evidence["robots"][ROBOT]["authority_events"].append(changed)
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "failed"
    assert result["checks"]["authority_changes_during_trial"] == 1


def test_geometry_revision_and_small_transform_heartbeat_are_allowed():
    evidence = _evidence()
    changed = deepcopy(evidence["robots"][ROBOT]["authority_events"][0])
    changed["received_elapsed_s"] = 59.0
    changed["state"]["mapping_graph_revision"] = 9
    changed["state"]["geometry_revision"] = 5
    changed["state"]["map_source_stamp"] = {"sec": 11, "nanosec": 0}
    changed["state"]["T_component_navigation"] = _matrix(0.01, 0.0)
    evidence["robots"][ROBOT]["authority_events"].append(changed)
    result = analyze(_observer(), evidence, ROBOT)
    assert result["status"] == "passed", result["reasons"]
    assert result["checks"]["authority_changes_during_trial"] == 0


def _corrected_evidence():
    evidence = _evidence()
    changed = deepcopy(evidence["robots"][ROBOT]["authority_events"][0])
    changed["received_elapsed_s"] = 70.0
    changed["state"]["correction_revision"] += 1
    changed["state"]["T_component_navigation"] = _matrix(0.1)
    changed["state"]["home"]["T_navigation_home"] = _matrix(-0.1)
    evidence["robots"][ROBOT]["authority_events"].append(changed)
    return evidence, changed["state"]


def test_recovery_mode_accepts_physical_arrival_to_same_home():
    evidence, _ = _corrected_evidence()
    assert analyze(_observer(), evidence, ROBOT)["status"] == "failed"
    result = analyze(_observer(), evidence, ROBOT, allow_authority_replanning=True)
    assert result["status"] == "passed", result["reasons"]
    assert result["checks"]["authority_changes_during_trial"] == 1
    assert "not established" in result["checks"]["replanning_transition_audit"]


@pytest.mark.parametrize(
    "field", ["mission_id", "component_id", "navigation_frame", "map_epoch"]
)
def test_recovery_mode_rejects_replaced_identity(field):
    evidence, changed = _corrected_evidence()
    changed[field] = 99 if field == "map_epoch" else "replacement"
    result = analyze(_observer(), evidence, ROBOT, allow_authority_replanning=True)
    assert result["status"] == "failed"
    assert any("Home identity" in reason for reason in result["reasons"])


def test_recovery_mode_still_requires_physical_arrival():
    evidence, _ = _corrected_evidence()
    for sample in evidence["robots"][ROBOT]["truth_samples"]:
        if sample["received_elapsed_s"] >= 70:
            sample["position"]["x"] = 2.0
    result = analyze(_observer(), evidence, ROBOT, allow_authority_replanning=True)
    assert result["status"] == "failed"
    assert any("outside the home tolerance" in reason for reason in result["reasons"])


@pytest.mark.parametrize("home", [None, {"keyframe_id": "another-home"}])
def test_recovery_mode_rejects_missing_or_replaced_home(home):
    evidence, changed = _corrected_evidence()
    changed["home"] = home
    result = analyze(_observer(), evidence, ROBOT, allow_authority_replanning=True)
    assert result["status"] == "failed"
    assert any("Home identity" in reason for reason in result["reasons"])


def test_reactivation_after_success_fails():
    observer = _observer()
    observer["samples"].append(_fleet(20.5, "active", path=True))
    result = analyze(observer, _evidence(), ROBOT)
    assert result["status"] == "failed"
    assert any("reactivated" in reason for reason in result["reasons"])


def test_overflow_elapsed_and_null_checkpoint_are_inconclusive_not_exceptions():
    observer = _observer()
    observer["checkpoint"] = None
    evidence = _evidence()
    evidence["robots"][ROBOT]["truth_samples"][0]["received_elapsed_s"] = 1e300
    result = analyze(observer, evidence, ROBOT)
    assert result["status"] != "passed"
