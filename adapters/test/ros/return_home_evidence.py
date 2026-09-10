#!/usr/bin/env python3
"""Correlate one Return Home observer report with independent ARGoS truth."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path
import sys
from typing import Any

SCHEMA = "swarmdeck.return-home-evidence.v1"
RECORDER_SCHEMA = "swarmdeck.simulation-evidence.v1"
AUTHORITY_FIELDS = (
    "robot_id",
    "mission_id",
    "component_id",
    "map_epoch",
    "mapping_graph_revision",
    "geometry_revision",
    "correction_revision",
    "map_source_stamp",
    "navigation_frame",
    "T_component_navigation",
    "anchor",
    "home",
    "solution_order",
)


def _time(value: Any) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        result = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if result.tzinfo is None:
        return None
    return result.astimezone(timezone.utc)


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _wall(start: datetime | None, elapsed: Any) -> datetime | None:
    seconds = _number(elapsed)
    if start is None or seconds is None or seconds < 0:
        return None
    try:
        return start + timedelta(seconds=seconds)
    except OverflowError:
        return None


def _robot(snapshot: Any, robot_id: str) -> dict[str, Any] | None:
    fleet = snapshot.get("fleet") if isinstance(snapshot, dict) else None
    data = fleet.get("data") if isinstance(fleet, dict) and fleet.get("ok") else None
    robots = data.get("robots") if isinstance(data, dict) else None
    if not isinstance(robots, list):
        return None
    return next(
        (
            item
            for item in robots
            if isinstance(item, dict) and item.get("robot_id") == robot_id
        ),
        None,
    )


def _status(snapshot: Any, robot_id: str) -> str | None:
    robot = _robot(snapshot, robot_id)
    value = robot.get("nav_status") if robot else None
    return value if isinstance(value, str) else None


def _active(snapshot: Any, robot_id: str) -> bool:
    robot = _robot(snapshot, robot_id)
    return bool(
        robot
        and (
            robot.get("nav_status") in {"active", "nav"}
            or robot.get("goal")
            or robot.get("global_planned_path")
            or robot.get("local_planned_path")
        )
    )


def _matrix4(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(row, list) and len(row) == 4 for row in value)
        and all(_number(item) is not None for row in value for item in row)
    )


def _projection(value: dict[str, Any]) -> dict[str, Any]:
    return {field: value.get(field) for field in AUTHORITY_FIELDS}


def _transform_delta(first: Any, second: Any) -> tuple[float, float] | None:
    if not _matrix4(first) or not _matrix4(second):
        return None
    translation = math.sqrt(
        sum(
            (float(first[index][3]) - float(second[index][3])) ** 2
            for index in range(3)
        )
    )
    trace = sum(
        float(first[row][column]) * float(second[row][column])
        for row in range(3)
        for column in range(3)
    )
    rotation = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    return translation, rotation


def _material_authority_change(
    original: dict[str, Any], updated: dict[str, Any], tolerance: float = 0.02
) -> bool:
    for field in (
        "robot_id",
        "mission_id",
        "component_id",
        "correction_revision",
        "navigation_frame",
    ):
        if updated.get(field) != original.get(field):
            return True
    original_home = original.get("home")
    updated_home = updated.get("home")
    if not isinstance(original_home, dict) or not isinstance(updated_home, dict):
        return True
    if updated_home.get("keyframe_id") != original_home.get("keyframe_id"):
        return True
    deltas = (
        _transform_delta(
            original.get("T_component_navigation"),
            updated.get("T_component_navigation"),
        ),
        _transform_delta(
            original_home.get("T_navigation_home"),
            updated_home.get("T_navigation_home"),
        ),
    )
    return any(
        delta is None or delta[0] > tolerance or delta[1] > tolerance
        for delta in deltas
    )


def _same_home_identity(original: dict[str, Any], updated: dict[str, Any]) -> bool:
    """Allow corrections within one Home mission, never a replacement landmark."""
    if any(
        original.get(field) != updated.get(field)
        for field in (
            "robot_id",
            "mission_id",
            "component_id",
            "map_epoch",
            "navigation_frame",
        )
    ):
        return False
    original_home = original.get("home")
    home = updated.get("home")
    if (
        not isinstance(original_home, dict)
        or not isinstance(home, dict)
        or home.get("keyframe_id") != original_home.get("keyframe_id")
    ):
        return False
    revision = updated.get("correction_revision")
    original_revision = original.get("correction_revision")
    return (
        type(revision) is int
        and type(original_revision) is int
        and revision >= original_revision >= 0
        and _matrix4(updated.get("T_component_navigation"))
        and _matrix4(home.get("T_navigation_home"))
    )


def _pose(sample: dict[str, Any]) -> tuple[float, float, float] | None:
    position = sample.get("position")
    orientation = sample.get("orientation_xyzw")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    xyz = tuple(_number(position.get(axis)) for axis in "xyz")
    xyzw = tuple(_number(orientation.get(axis)) for axis in "xyzw")
    if any(value is None for value in (*xyz, *xyzw)):
        return None
    return xyz  # type: ignore[return-value]


def _distance(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> float:
    return math.dist(first[:2], second[:2])


def _result(
    status: str, robot_id: str, checks: dict[str, Any], reasons: list[str]
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "status": status,
        "robot_id": robot_id,
        "checks": checks,
        "reasons": reasons,
    }


def analyze(
    observer: dict[str, Any],
    truth_report: dict[str, Any],
    robot_id: str,
    *,
    tolerance_m: float = 0.5,
    settled_samples: int = 3,
    max_gap_s: float = 1.0,
    min_settled_s: float = 1.0,
    post_stop_grace_s: float = 1.0,
    allow_authority_replanning: bool = False,
) -> dict[str, Any]:
    """Return ``passed`` only for one fully bound, continuously observed trial"""

    if not isinstance(observer, dict) or not isinstance(truth_report, dict):
        return _result(
            "inconclusive",
            robot_id,
            {},
            ["observer and recorder evidence must be JSON objects"],
        )
    bounds = (tolerance_m, max_gap_s, min_settled_s)
    if (
        any(not math.isfinite(value) or value <= 0 for value in bounds)
        or not math.isfinite(post_stop_grace_s)
        or post_stop_grace_s < 0
        or type(settled_samples) is not int
        or settled_samples < 1
    ):
        raise ValueError("invalid analyzer bounds")

    unsure: list[str] = []
    failed: list[str] = []
    checks: dict[str, Any] = {
        "tolerance_m": tolerance_m,
        "required_settled_samples": settled_samples,
        "max_gap_s": max_gap_s,
        "minimum_settled_duration_s": min_settled_s,
        "post_stop_grace_s": post_stop_grace_s,
        "allow_authority_replanning": allow_authority_replanning,
    }

    if observer.get("schema_version") != 1:
        unsure.append("observer schema_version is not 1")
    checkpoint = observer.get("checkpoint")
    if not isinstance(checkpoint, dict) or checkpoint.get("phase") != "complete":
        unsure.append("observer report is not a complete checkpoint")
    if truth_report.get("schema") != RECORDER_SCHEMA:
        unsure.append("recorder schema is not simulation-evidence.v1")

    observer_start = _time(observer.get("started_at"))
    observer_finish = _time(observer.get("finished_at"))
    recorder_start = _time(truth_report.get("started_at_utc"))
    recorder_end = _time(truth_report.get("ended_at_utc"))
    if observer_start is None or observer_finish is None:
        unsure.append("observer wall-clock bounds are missing or invalid")
    if recorder_start is None or recorder_end is None:
        unsure.append("recorder wall-clock bounds are missing or invalid")
    if observer.get("robot_id") != robot_id:
        failed.append("observer robot_id does not match requested robot")

    mission_id = observer.get("mission_id_filter")
    if not isinstance(mission_id, str) or not mission_id:
        unsure.append("observer mission_id_filter is required")
        mission_id = None
    fixture = observer.get("authority_fixture")
    authority = fixture.get("raw") if isinstance(fixture, dict) else None
    if not isinstance(authority, dict):
        unsure.append("observer authority fixture is required")
        authority = None
    else:
        if authority.get("robot_id") != robot_id:
            failed.append("authority robot_id does not match requested robot")
        if mission_id is None or authority.get("mission_id") != mission_id:
            failed.append("authority mission_id does not match observer mission")
        navigation_frame = authority.get("navigation_frame")
        if (
            not isinstance(navigation_frame, str)
            or navigation_frame.lstrip("/") != f"{robot_id}/map_frame"
        ):
            failed.append("authority navigation_frame does not match robot map frame")
        if not isinstance(authority.get("component_id"), str) or not authority.get(
            "component_id"
        ):
            failed.append("authority component_id is missing")
        for field in ("correction_revision", "map_epoch", "mapping_graph_revision"):
            value = authority.get(field)
            if type(value) is not int or value < 0:
                failed.append(f"authority {field} is invalid")
        home = authority.get("home")
        expected_home_id = f"{robot_id}/{mission_id}/0" if mission_id else None
        if not isinstance(home, dict):
            failed.append("authority home is missing")
        else:
            if home.get("keyframe_id") != expected_home_id:
                failed.append("authority home is not this mission's keyframe 0")
            if not _matrix4(home.get("T_navigation_home")):
                failed.append("authority home transform is invalid")
        if not _matrix4(authority.get("T_component_navigation")):
            failed.append("authority component transform is invalid")
    if isinstance(fixture, dict):
        age = _number(fixture.get("age_at_start_s"))
        max_age = _number(fixture.get("max_age_s"))
        if age is None or max_age is None or age < 0 or max_age <= 0 or age > max_age:
            failed.append("authority fixture was not fresh at observer start")

    if observer.get("errors") != []:
        failed.append("observer report contains errors or lacks an empty error list")
    commands = observer.get("commands")
    if not isinstance(commands, dict):
        unsure.append("observer command records are missing")
        commands = {}
    expected_commands = {
        "return_home": {"type": "return_home", "robot_id": robot_id},
        "stop_all": {"type": "stop_all"},
    }
    for name, payload in expected_commands.items():
        command = commands.get(name)
        if (
            not isinstance(command, dict)
            or command.get("sent") is not True
            or command.get("frame_written") is not True
            or command.get("payload") != payload
        ):
            failed.append(f"{name} command record is invalid")

    baseline = observer.get("baseline")
    baseline_time = (
        _time(baseline.get("observed_at")) if isinstance(baseline, dict) else None
    )
    baseline_robot = _robot(baseline, robot_id)
    if baseline_time is None or baseline_robot is None:
        unsure.append("observer baseline sample is missing or invalid")
    elif (
        baseline_robot.get("online") is not True
        or baseline_robot.get("exploration_status") != "stopped"
        or baseline_robot.get("nav_status") != "idle"
        or baseline_robot.get("goal")
        or baseline_robot.get("global_planned_path")
        or baseline_robot.get("local_planned_path")
        or not {"navigate", "plan_objective"}.issubset(
            set(baseline_robot.get("capabilities") or [])
        )
    ):
        failed.append("observer baseline is not online, stopped, idle onboard planning")

    summary = observer.get("summary")
    if not isinstance(summary, dict):
        unsure.append("observer summary is missing")
    else:
        if (
            summary.get("server_evidence_consistent_with_objective_execution")
            is not True
        ):
            failed.append("observer does not establish full-route objective execution")
        initial_distance = _number(
            summary.get("initial_server_frame_distance_to_authority_home_m")
        )
        if initial_distance is None or initial_distance <= tolerance_m:
            failed.append("observer did not start beyond the home tolerance")
        post_state = summary.get("post_stop_state")
        if (
            not isinstance(post_state, dict)
            or post_state.get("all_eligible_reported_stopped_and_inactive") is not True
        ):
            failed.append("observer post-stop state is not stopped and inactive")

    raw_samples = observer.get("samples")
    if not isinstance(raw_samples, list) or not raw_samples:
        unsure.append("observer objective samples are missing")
        raw_samples = []
    timed_samples: list[tuple[datetime, dict[str, Any]]] = []
    invalid_observer = 0
    for sample in raw_samples:
        when = _time(sample.get("observed_at")) if isinstance(sample, dict) else None
        if when is None or _robot(sample, robot_id) is None:
            invalid_observer += 1
        else:
            timed_samples.append((when, sample))
    checks["invalid_observer_samples"] = invalid_observer
    if invalid_observer:
        unsure.append("observer contains invalid objective samples")
    if any(
        current[0] <= previous[0]
        for previous, current in zip(timed_samples, timed_samples[1:])
    ):
        failed.append("observer sample times do not strictly advance")
    terminals = [
        item
        for item in timed_samples
        if _status(item[1], robot_id) in {"succeeded", "failed", "cancelled"}
    ]
    terminal = terminals[-1] if terminals else None
    terminal_time = terminal[0] if terminal else None
    activity_times = [
        when for when, sample in timed_samples if _active(sample, robot_id)
    ]
    activity = observer.get("objective_activity_observed") is True and bool(
        activity_times
    )
    checks["objective_activity_observed"] = activity
    checks["terminal_status"] = _status(terminal[1], robot_id) if terminal else None
    if not activity:
        unsure.append("objective activity is not present in observer samples")
    if terminal is None or _status(terminal[1], robot_id) != "succeeded":
        failed.append("latest terminal navigation state is not succeeded")
    if terminal_time and not any(when <= terminal_time for when in activity_times):
        failed.append("no objective activity precedes terminal success")
    if terminal is not None:
        terminal_index = timed_samples.index(terminal)
        if any(
            _active(sample, robot_id)
            for _when, sample in timed_samples[terminal_index + 1 :]
        ):
            failed.append("navigation reactivated after terminal success")

    post_stop = observer.get("post_stop")
    stop_time = (
        _time(post_stop.get("observed_at")) if isinstance(post_stop, dict) else None
    )
    if stop_time is None:
        unsure.append("observer post-stop sample time is missing")
    timeline = [
        observer_start,
        baseline_time,
        terminal_time,
        stop_time,
        observer_finish,
    ]
    if all(value is not None for value in timeline) and any(
        current < previous for previous, current in zip(timeline, timeline[1:])
    ):
        failed.append("observer trial times are out of order")

    robots = truth_report.get("robots")
    source = robots.get(robot_id) if isinstance(robots, dict) else None
    raw_truth = source.get("truth_samples") if isinstance(source, dict) else None
    if not isinstance(source, dict) or not isinstance(raw_truth, list):
        unsure.append(f"recorder has no truth samples for {robot_id}")
        source, raw_truth = {}, []
    for field in (
        "invalid_truth_samples",
        "truth_samples_dropped",
        "invalid_authority_messages",
        "authority_events_dropped",
        "invalid_keyframe_metadata_messages",
        "keyframe_events_dropped",
    ):
        value = source.get(field)
        checks[field] = value
        if type(value) is not int or value < 0:
            unsure.append(f"recorder counter {field} is missing or invalid")
        elif value:
            failed.append(f"recorder counter {field} is nonzero")

    truth: list[dict[str, Any]] = []
    malformed = 0
    for sample in raw_truth:
        wall = (
            _wall(recorder_start, sample.get("received_elapsed_s"))
            if isinstance(sample, dict)
            else None
        )
        pose = _pose(sample) if isinstance(sample, dict) else None
        stamp = sample.get("stamp_ns") if isinstance(sample, dict) else None
        frame = sample.get("frame_id") if isinstance(sample, dict) else None
        child = sample.get("child_frame_id") if isinstance(sample, dict) else None
        if (
            wall is None
            or pose is None
            or type(stamp) is not int
            or stamp < 0
            or frame != "world"
            or not isinstance(child, str)
            or not child
        ):
            malformed += 1
        else:
            truth.append({"wall": wall, "stamp": stamp, "pose": pose})
    checks["truth_sample_count"] = len(truth)
    checks["malformed_truth_samples"] = malformed
    if malformed:
        failed.append("recorder contains malformed or nonfinite truth samples")
    if not truth:
        unsure.append("no valid world-frame truth samples remain")
    nonadvancing_stamps = sum(
        current["stamp"] <= previous["stamp"]
        for previous, current in zip(truth, truth[1:])
    )
    nonadvancing_receipts = sum(
        current["wall"] <= previous["wall"]
        for previous, current in zip(truth, truth[1:])
    )
    checks["nonadvancing_truth_stamps"] = nonadvancing_stamps
    checks["nonadvancing_truth_receipts"] = nonadvancing_receipts
    if nonadvancing_stamps or nonadvancing_receipts:
        failed.append("truth stamps and receipt times must strictly advance")

    parsed_authority: list[tuple[datetime, dict[str, Any]]] = []
    authority_events = source.get("authority_events")
    if not isinstance(authority_events, list):
        unsure.append("recorder authority events are missing")
    else:
        for event in authority_events:
            when = (
                _wall(recorder_start, event.get("received_elapsed_s"))
                if isinstance(event, dict)
                else None
            )
            state = event.get("state") if isinstance(event, dict) else None
            if when is None or not isinstance(state, dict):
                failed.append("recorder contains an invalid authority event")
            else:
                parsed_authority.append((when, state))
    authority_match = None
    if authority is not None and observer_start is not None:
        preceding_matches = [
            item
            for item in parsed_authority
            if item[0] <= observer_start
            and _projection(item[1]) == _projection(authority)
        ]
        authority_match = preceding_matches[-1] if preceding_matches else None
        if authority_match is None:
            unsure.append(
                "fresh observer authority has no matching recorder event before this objective"
            )
    checks["authority_snapshot_matched"] = bool(
        authority_match
        and authority is not None
        and _projection(authority_match[1]) == _projection(authority)
    )

    expected_home_id = f"{robot_id}/{mission_id}/0" if mission_id else None
    home_event = None
    keyframe_events = source.get("keyframe_events")
    if not isinstance(keyframe_events, list):
        unsure.append("recorder keyframe events are missing")
    elif expected_home_id:
        matches = []
        for event in keyframe_events:
            when = (
                _wall(recorder_start, event.get("received_elapsed_s"))
                if isinstance(event, dict)
                else None
            )
            metadata = event.get("metadata") if isinstance(event, dict) else None
            if when is None or not isinstance(metadata, dict):
                failed.append("recorder contains an invalid keyframe event")
            elif metadata.get("keyframe_id") == expected_home_id:
                matches.append((when, metadata))
        home_event = matches[0] if matches else None
    if home_event is None:
        unsure.append("recorder lacks the authority home keyframe-0 event")
    elif (
        home_event[1].get("schema") != "swarmdeck.keyframe-metadata.v1"
        or type(home_event[1].get("stamp_ns")) is not int
        or home_event[1]["stamp_ns"] < 0
    ):
        failed.append("home keyframe metadata is invalid")

    anchor = None
    if truth and home_event and type(home_event[1].get("stamp_ns")) is int:
        home_stamp = home_event[1]["stamp_ns"]
        anchor = min(truth, key=lambda item: abs(item["stamp"] - home_stamp))
        stamp_error_s = abs(anchor["stamp"] - home_stamp) / 1e9
        checks["home_truth_stamp_error_s"] = round(stamp_error_s, 6)
        if stamp_error_s > max_gap_s:
            unsure.append("no ground truth sample is close to the home keyframe stamp")
            anchor = None
        if anchor is not None and truth[0]["stamp"] > home_stamp:
            unsure.append("ground truth starts after the home keyframe")
        elif (
            anchor is not None
            and _distance(truth[0]["pose"], anchor["pose"]) > tolerance_m
        ):
            failed.append("home keyframe is not at the initial ground-truth pose")
    checks["home_anchor_matched"] = anchor is not None

    precommand = None
    if truth and baseline_time:
        prior = [item for item in truth if item["wall"] <= baseline_time]
        precommand = prior[-1] if prior else None
        if (
            precommand is None
            or (baseline_time - precommand["wall"]).total_seconds() > max_gap_s
        ):
            unsure.append(
                "no fresh ground truth sample precedes the Return Home command"
            )
            precommand = None
        elif anchor is not None:
            distance = _distance(anchor["pose"], precommand["pose"])
            checks["precommand_truth_distance_from_home_m"] = round(distance, 3)
            if distance <= tolerance_m:
                failed.append(
                    "robot was not physically away from home before the objective"
                )

    target = stop_time + timedelta(seconds=post_stop_grace_s) if stop_time else None
    coverage = None
    if truth and target:
        later = [item for item in truth if item["wall"] >= target]
        coverage = later[0] if later else None
        if coverage is None or (coverage["wall"] - target).total_seconds() > max_gap_s:
            unsure.append(
                "truth does not cover Stop All plus the required grace period"
            )
            coverage = None
    checks["post_stop_truth_coverage"] = coverage is not None

    if precommand and coverage:
        trial = [
            item
            for item in truth
            if precommand["wall"] <= item["wall"] <= coverage["wall"]
        ]
        gaps = [
            (current["wall"] - previous["wall"]).total_seconds()
            for previous, current in zip(trial, trial[1:])
        ]
        checks["max_trial_truth_gap_s"] = max(gaps, default=0.0)
        if not gaps or any(gap > max_gap_s for gap in gaps):
            unsure.append(
                "ground truth is not continuous across this objective and stop"
            )

    settled: list[dict[str, Any]] = []
    if anchor and terminal_time and coverage:
        for item in [
            item for item in truth if terminal_time <= item["wall"] <= coverage["wall"]
        ]:
            if _distance(anchor["pose"], item["pose"]) <= tolerance_m:
                settled.append(item)
            else:
                settled.clear()
        duration = (
            (settled[-1]["wall"] - settled[0]["wall"]).total_seconds()
            if settled
            else 0.0
        )
        gaps = [
            (current["wall"] - previous["wall"]).total_seconds()
            for previous, current in zip(settled, settled[1:])
        ]
        checks.update(
            {
                "settled_sample_count": len(settled),
                "settled_duration_s": round(duration, 3),
                "max_settled_gap_s": max(gaps, default=0.0),
                "settled_max_error_m": (
                    round(
                        max(
                            (
                                _distance(anchor["pose"], item["pose"])
                                for item in settled
                            ),
                            default=math.nan,
                        ),
                        3,
                    )
                    if settled
                    else None
                ),
            }
        )
        if len(settled) < settled_samples:
            unsure.append(
                f"fewer than {settled_samples} consecutive truth samples settled near home"
            )
        if duration < min_settled_s:
            unsure.append("settled truth duration is too short")
        if any(gap > max_gap_s for gap in gaps):
            unsure.append("settled truth samples have an excessive time gap")
        if _distance(anchor["pose"], coverage["pose"]) > tolerance_m:
            failed.append("final post-stop ground truth is outside the home tolerance")
    else:
        checks.update(
            {
                "settled_sample_count": 0,
                "settled_duration_s": 0.0,
                "settled_max_error_m": None,
            }
        )

    if authority is not None and authority_match and coverage:
        trial_authorities = [
            state
            for when, state in parsed_authority
            if authority_match[0] < when <= coverage["wall"]
        ]
        changed = [
            state
            for state in trial_authorities
            if _material_authority_change(authority, state)
        ]
        checks["authority_changes_during_trial"] = len(changed)
        if changed and not allow_authority_replanning:
            failed.append("map authority changed during the accepted trial window")
        if allow_authority_replanning:
            if not all(
                _same_home_identity(authority, state) for state in trial_authorities
            ):
                failed.append(
                    "authority replanning changed Home identity or lost valid transforms"
                )
            checks["replanning_transition_audit"] = (
                "not established by these samples; validate cancellation and retry lifecycle separately"
            )
    if recorder_end and coverage and recorder_end < coverage["wall"]:
        failed.append("recorder end precedes selected post-stop truth")

    reasons = failed + unsure
    status = "failed" if failed else "inconclusive" if unsure else "passed"
    return _result(status, robot_id, checks, reasons)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observer-report", required=True, type=Path)
    parser.add_argument("--truth-report", required=True, type=Path)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--tolerance-m", type=float, default=0.5)
    parser.add_argument("--settled-samples", type=int, default=3)
    parser.add_argument("--max-gap-s", type=float, default=1.0)
    parser.add_argument("--min-settled-s", type=float, default=1.0)
    parser.add_argument("--post-stop-grace-s", type=float, default=1.0)
    parser.add_argument(
        "--allow-authority-replanning",
        action="store_true",
        help="accept physical arrival across corrections to the same Home identity; does not audit every retry",
    )
    args = parser.parse_args(argv)
    try:
        observer = json.loads(args.observer_report.read_text())
        truth = json.loads(args.truth_report.read_text())
        result = analyze(
            observer,
            truth,
            args.robot_id,
            tolerance_m=args.tolerance_m,
            settled_samples=args.settled_samples,
            max_gap_s=args.max_gap_s,
            min_settled_s=args.min_settled_s,
            post_stop_grace_s=args.post_stop_grace_s,
            allow_authority_replanning=args.allow_authority_replanning,
        )
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
        result = _result(
            "inconclusive", args.robot_id, {}, [f"unable to analyze evidence: {exc}"]
        )
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return {"passed": 0, "failed": 1, "inconclusive": 2}[result["status"]]


if __name__ == "__main__":
    sys.exit(main())
