import importlib.util
import asyncio
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError

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


def test_baseline_only_evidence_reports_unmeasured_progress_not_zero():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    idle = {"exploration_status": "idle", "nav_status": "idle"}
    assert (
        module.record_robot_sample(
            evidence,
            live_sample(0.0, path=False),
            idle,
            "mission",
            "component",
            trial=False,
        )
        is None
    )

    assert evidence["samples"] == 1
    assert evidence["trial_samples"] == 0
    assert evidence["initial_navigation_pose"] == (0.0, 0.0)
    result = module.assess_robot(evidence, 5.0)
    assert result["passed"] is False
    assert result["max_displacement_m"] is None
    assert result["reasons"] == [
        "no qualified live samples after Explore started (XY progress unmeasured)"
    ]


def test_static_robot_observed_during_the_trial_still_fails_on_progress():
    module = acceptance_module()
    evidence = module.new_robot_evidence("robot_0")
    idle = {"exploration_status": "idle", "nav_status": "idle"}
    module.record_robot_sample(
        evidence,
        live_sample(0.0, path=False),
        idle,
        "mission",
        "component",
        trial=False,
    )
    module.record_robot_sample(
        evidence, live_sample(0.0), status_sample(), "mission", "component"
    )

    result = module.assess_robot(evidence, 5.0)
    assert evidence["trial_samples"] == 1
    assert result["max_displacement_m"] == 0.0
    assert result["reasons"] == [
        "insufficient XY progress (max 0.000 m, minimum 5.000 m)"
    ]


ROBOT_IDS = [f"robot_{index}" for index in range(4)]


class FakeServer:
    """Fleet telemetry that drives while the qualified live channel breaks.

    ``/api/fleet`` advances every robot's pose on each poll once Explore has
    been started, so the server demonstrably sees motion. The live component
    endpoint serves the idle baseline and then fails in the requested way for
    the rest of the trial, which is what an operator saw as ``0.000 m`` for
    robots that a ground-truth trace showed driving.
    """

    def __init__(self, live_failure):
        self.live_failure = live_failure
        self.exploring = False
        self.x = 0.0
        self.live_calls_while_exploring = 0
        self.socket_messages = []

    def fleet(self):
        return {
            "robots": [
                {
                    "robot_id": robot_id,
                    "online": True,
                    "capabilities": ["explore"],
                    "mode": "explore" if self.exploring else "idle",
                    "nav_status": "active" if self.exploring else "idle",
                    "exploration_status": ("exploring" if self.exploring else "idle"),
                    "goal": None,
                    "pose": {"x": self.x, "y": 0.0},
                    "local_planned_path": [],
                }
                for robot_id in ROBOT_IDS
            ]
        }

    def live_robot(self, robot_id):
        robot = {
            "robot_id": robot_id,
            "mission_id": "mission",
            "component_id": "component",
            "navigation_frame": f"{robot_id}/map_frame",
            "pose": {"x": 0.0, "y": 0.0},
        }
        if self.exploring and self.live_failure == "no_pose":
            del robot["pose"]
        return robot

    def request(self, _base_url, path, body=None, timeout=5.0):
        if path == "/api/sim/reset":
            return {"version": 1, "phase": "succeeded", "supervisor_available": True}
        if path == "/api/fleet":
            if self.exploring:
                self.x += 1.0
            return self.fleet()
        if path == "/api/autonomy/replicas/components":
            return {
                "active_session_id": "mission",
                "components": [
                    {
                        "session_id": "mission",
                        "component_id": "component",
                        "available": True,
                        "robot_ids": ROBOT_IDS,
                    }
                ],
            }
        assert path.startswith("/api/autonomy/replicas/components/live/mission")
        if self.exploring:
            self.live_calls_while_exploring += 1
            if self.live_failure == "404":
                raise HTTPError(path, 404, "No fresh robot telemetry", None, None)
        return {
            "mission_id": "mission",
            "component_id": "component",
            "robots": [self.live_robot(robot_id) for robot_id in ROBOT_IDS],
        }

    async def send(self, raw):
        message = json.loads(raw)
        self.socket_messages.append(message["type"])
        if message["type"] == "stop_all":
            self.exploring = False
        elif message["type"] == "start_explore":
            self.exploring = True

    async def close(self):
        pass

    def __aiter__(self):
        return self

    async def __anext__(self):
        await asyncio.sleep(3600)


def _run_trial(module, monkeypatch, server, capsys):
    async def connect(*_args, **_kwargs):
        return server

    monkeypatch.setattr(module, "json_request", server.request)
    monkeypatch.setattr(module.websockets, "connect", connect)
    args = SimpleNamespace(
        base_url="http://unused",
        duration=1.2,
        poll=0.1,
        authority_timeout=2.0,
        sample_timeout=0.5,
        min_displacement=5.0,
        max_authority_transients=20,
    )
    passed = asyncio.run(module.run(args))
    summary = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    return passed, summary


@pytest.mark.parametrize(
    "live_failure, expected_error",
    [
        ("404", "HTTP 404"),
        ("no_pose", "robot_3: qualified live robot has no navigation pose"),
    ],
)
def test_lost_live_channel_is_not_reported_as_zero_progress(
    monkeypatch, capsys, live_failure, expected_error
):
    module = acceptance_module()
    server = FakeServer(live_failure)

    passed, summary = _run_trial(module, monkeypatch, server, capsys)

    assert passed is False
    assert server.socket_messages[:5] == ["stop_all"] + ["start_explore"] * 4
    assert server.x >= 2.0, "fleet telemetry must have moved during the trial"
    assert server.live_calls_while_exploring > 0
    assert summary["outcome"] == "failed"
    assert summary["stop_all_verified"] is True
    assert summary["observation_errors"] > 0
    assert summary["last_observation_error"].startswith(expected_error)
    assert "observation errors (last: " + expected_error in summary["failure_reason"]
    assert "insufficient XY progress" not in summary["failure_reason"]
    assert "0.000 m" not in summary["failure_reason"]
    for robot_id in ROBOT_IDS:
        robot = summary["robots"][robot_id]
        assert robot["initial_navigation_pose"] == [0.0, 0.0]
        assert robot["samples"] == 1
        assert robot["trial_samples"] == 0
        assert robot["max_displacement_m"] is None
        assert robot["reasons"] == [
            "no qualified live samples after Explore started "
            "(XY progress unmeasured)"
        ]
        assert "Explore never reported an executing state" not in robot["reasons"]


def test_main_prints_trial_exit_after_the_summary(monkeypatch, capsys):
    module = acceptance_module()

    async def failing_run(_args):
        print(json.dumps({"outcome": "failed"}))
        return False

    monkeypatch.setattr(module, "run", failing_run)
    monkeypatch.setattr(sys, "argv", ["exploration_acceptance.py", "--simulation"])
    with pytest.raises(SystemExit) as exit_info:
        module.main()

    lines = capsys.readouterr().out.strip().splitlines()
    assert exit_info.value.code == 1
    assert lines[-1] == "trial_exit=1"
    assert json.loads(lines[-2]) == {"outcome": "failed"}


def test_main_reports_a_crashed_trial_as_a_nonzero_exit(monkeypatch, capsys):
    module = acceptance_module()

    async def crashing_run(_args):
        raise KeyError("robots")

    monkeypatch.setattr(module, "run", crashing_run)
    monkeypatch.setattr(sys, "argv", ["exploration_acceptance.py", "--simulation"])
    with pytest.raises(SystemExit) as exit_info:
        module.main()

    lines = capsys.readouterr().out.strip().splitlines()
    assert exit_info.value.code == 1
    assert lines[-1] == "trial_exit=1"
    summary = json.loads(lines[-2])
    assert summary["outcome"] == "failed"
    assert summary["failure_reason"] == "KeyError: 'robots'"


def test_main_exits_zero_with_trial_exit_line_on_success(monkeypatch, capsys):
    module = acceptance_module()

    async def passing_run(_args):
        print(json.dumps({"outcome": "succeeded"}))
        return True

    monkeypatch.setattr(module, "run", passing_run)
    monkeypatch.setattr(sys, "argv", ["exploration_acceptance.py", "--simulation"])
    with pytest.raises(SystemExit) as exit_info:
        module.main()

    assert exit_info.value.code == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "trial_exit=0"


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
