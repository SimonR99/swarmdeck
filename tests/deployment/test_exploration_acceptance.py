import importlib.util
import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest


def acceptance_module():
    path = Path(__file__).with_name("exploration_acceptance.py")
    spec = importlib.util.spec_from_file_location("exploration_acceptance", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def live_sample(x, *, frame="robot_0/odom", order=(0, -1), path=True, transform=None):
    transform = transform or [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    return {
        "robot_id": "robot_0",
        "navigation_frame": frame,
        "solution_order": list(order),
        "T_component_navigation": transform,
        "pose": {"x": x, "y": 0.0},
        "local_planned_path": (
            [{"x": x + 1.0, "y": 0.0}, {"x": x + 2.0, "y": 0.0}] if path else []
        ),
    }


def status_sample(status="exploring"):
    return {"exploration_status": status, "nav_status": "active"}


def test_graph_solution_revision_does_not_replace_stable_authority():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")

    assert (
        module.record_robot_sample(
            evidence,
            live_sample(0.0, order=(1, 2)),
            status_sample(),
            "mission",
            "component",
        )
        is None
    )
    assert (
        module.record_robot_sample(
            evidence,
            live_sample(0.8, order=(1, 3)),
            status_sample(),
            "mission",
            "component",
        )
        is None
    )

    assert evidence["authority_changed"] is False
    assert evidence["max_displacement_m"] == pytest.approx(0.8)
    assert module.assess_robot(evidence, 0.25)["passed"]


def test_component_correction_without_navigation_pose_motion_is_ignored():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    module.record_robot_sample(
        evidence, live_sample(0.0), status_sample(), "mission", "component"
    )
    module.record_robot_sample(
        evidence,
        live_sample(
            0.0,
            transform=[
                [1, 0, 0, 100],
                [0, 1, 0, 100],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
        ),
        status_sample(),
        "mission",
        "component",
    )

    assert evidence["max_displacement_m"] == 0.0


def test_waiting_without_dispatch_or_motion_fails():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    waiting = status_sample("waiting")
    assert (
        module.record_robot_sample(
            evidence,
            live_sample(0.0, path=False),
            waiting,
            "mission",
            "component",
        )
        is None
    )

    result = module.assess_robot(evidence, 0.25)
    assert result["passed"] is False
    assert "no waypoint/path observation" in result["reasons"]
    assert any(
        reason.startswith("insufficient XY progress") for reason in result["reasons"]
    )


def test_recovery_retains_bounded_last_navigation_failure():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    failure = {
        "exploration_status": "waiting",
        "nav_status": "failed",
        "nav_failure_reason": "terrain rejected: " + "x" * 600,
        "exploration_reason": "Waiting for traversable ground: " + "x" * 600,
    }
    module.record_robot_sample(
        evidence, live_sample(0.0), failure, "mission", "component"
    )
    module.record_robot_sample(
        evidence, live_sample(0.5), status_sample(), "mission", "component"
    )

    assert evidence["last_navigation_failure"] == failure["nav_failure_reason"][:512]
    assert evidence["last_exploration_reason"] == failure["exploration_reason"][:512]
    assert evidence["last_navigation_status"] == "active"
    assert evidence["last_exploration_status"] == "exploring"


def test_missing_baseline_pose_fails_closed_before_any_motion_evidence():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")

    error = module.record_robot_sample(
        evidence,
        {"navigation_frame": "robot_0/odom"},
        {"exploration_status": "stopped", "nav_status": "idle"},
        "mission",
        "component",
    )

    assert "no navigation pose" in error
    assert evidence["samples"] == 0
    assert not module.assess_robot(evidence, 0.25)["passed"]


def test_mission_component_or_navigation_frame_replacement_fails():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    module.record_robot_sample(
        evidence, live_sample(0.0), status_sample(), "mission", "component"
    )

    assert (
        module.record_robot_sample(
            evidence,
            live_sample(0.5, frame="robot_0/new_odom"),
            status_sample(),
            "mission",
            "component",
        )
        == "mission/component/navigation authority changed"
    )
    result = module.assess_robot(evidence, 0.25)
    assert result["passed"] is False
    assert "mission/component/navigation authority changed" in result["reasons"]


def test_initial_authority_can_pin_separate_components(monkeypatch):
    module = acceptance_module()
    args = SimpleNamespace(base_url="http://unused")
    responses = {
        "/api/autonomy/replicas/components": {
            "active_session_id": "mission",
            "components": [
                {
                    "session_id": "mission",
                    "component_id": "c0",
                    "available": True,
                    "robot_ids": ["robot_0"],
                },
                {
                    "session_id": "mission",
                    "component_id": "c1",
                    "available": True,
                    "robot_ids": ["robot_1"],
                },
            ],
        },
        "/api/autonomy/replicas/components/live/mission?component_id=c0": {
            "mission_id": "mission",
            "component_id": "c0",
            "robots": [
                {"robot_id": "robot_0", "mission_id": "mission", "component_id": "c0"}
            ],
        },
        "/api/autonomy/replicas/components/live/mission?component_id=c1": {
            "mission_id": "mission",
            "component_id": "c1",
            "robots": [
                {"robot_id": "robot_1", "mission_id": "mission", "component_id": "c1"}
            ],
        },
    }

    def request(_base_url, path, **_kwargs):
        return responses[path]

    monkeypatch.setattr(module, "json_request", request)
    mission, components, live = module._catalogue_live(
        args, {"robot_0", "robot_1"}, 999.0
    )

    assert mission == "mission"
    assert components == {"robot_0": "c0", "robot_1": "c1"}
    assert set(live) == {"robot_0", "robot_1"}


def test_simulation_guard_failure_opens_no_command_websocket(monkeypatch):
    module = acceptance_module()
    calls = []

    def reject(_args):
        raise RuntimeError("not a simulation fleet")

    async def connect(*_args, **_kwargs):
        calls.append("connect")
        raise AssertionError("command websocket opened before simulation guard")

    monkeypatch.setattr(module, "simulation_fleet", reject)
    monkeypatch.setattr(module.websockets, "connect", connect)
    args = SimpleNamespace(
        duration=5.0,
        min_displacement=0.25,
        base_url="http://127.0.0.1:8080",
    )

    assert asyncio.run(module.run(args)) is False
    assert calls == []


@pytest.mark.parametrize("available", (None, False))
def test_completed_reset_without_live_supervisor_opens_no_socket(
    monkeypatch, available
):
    module = acceptance_module()
    requests = []

    def request(_url, path):
        requests.append(path)
        return {"version": 1, "phase": "succeeded", "supervisor_available": available}

    async def connect(*_args, **_kwargs):
        raise AssertionError("stale simulation status authorized a command socket")

    monkeypatch.setattr(module, "json_request", request)
    monkeypatch.setattr(module.websockets, "connect", connect)
    args = SimpleNamespace(
        duration=5.0, min_displacement=0.25, base_url="http://unused"
    )
    assert asyncio.run(module.run(args)) is False
    assert requests == ["/api/sim/reset"]


def _evidence(module, frames):
    evidence = {}
    for robot_id, (component, frame) in frames.items():
        record = module.new_robot_evidence(robot_id)
        record["authority"] = ("mission", component, frame)
        evidence[robot_id] = record
    return evidence


def _live(frames):
    return {
        robot_id: {"robot_id": robot_id, "navigation_frame": frame}
        for robot_id, (_, frame) in frames.items()
    }


def test_verified_merge_continues_the_trial_and_is_recorded():
    module = acceptance_module()
    before = {"robot_0": "c:a", "robot_1": "c:a", "robot_2": "c:b", "robot_3": "c:c"}
    after = {"robot_0": "c:m", "robot_1": "c:m", "robot_2": "c:m", "robot_3": "c:c"}
    frames = {r: (before[r], f"{r}/map_frame") for r in before}
    verdict = module.classify_authority_change(
        "mission", before, "mission", after, _live(frames), _evidence(module, frames)
    )
    assert verdict is not None
    kind, merges = verdict
    assert kind == "merged"
    assert {(m["robot_id"], m["from"], m["to"]) for m in merges} == {
        ("robot_0", "c:a", "c:m"),
        ("robot_1", "c:a", "c:m"),
        ("robot_2", "c:b", "c:m"),
    }


def test_unchanged_components_are_a_transient_not_a_merge():
    module = acceptance_module()
    components = {"robot_0": "c:a", "robot_1": "c:a"}
    frames = {r: (components[r], f"{r}/map_frame") for r in components}
    assert module.classify_authority_change(
        "mission",
        components,
        "mission",
        dict(components),
        _live(frames),
        _evidence(module, frames),
    ) == ("transient", [])


@pytest.mark.parametrize(
    "mission, after, frame_change",
    [
        ("other-mission", {"robot_0": "c:a", "robot_1": "c:a"}, False),
        ("mission", {"robot_0": "c:a", "robot_1": "c:z"}, False),
        ("mission", {"robot_0": "c:m", "robot_1": "c:m"}, True),
        ("mission", {"robot_0": "c:a"}, False),
    ],
)
def test_split_mission_or_frame_replacement_still_fails(mission, after, frame_change):
    module = acceptance_module()
    before = {"robot_0": "c:a", "robot_1": "c:a"}
    frames = {r: (before[r], f"{r}/map_frame") for r in before}
    live = _live(frames)
    if frame_change:
        live["robot_1"]["navigation_frame"] = "robot_1/other_frame"
    assert (
        module.classify_authority_change(
            "mission", before, mission, after, live, _evidence(module, frames)
        )
        is None
    )
