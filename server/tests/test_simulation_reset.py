import json
import os
import shutil
import subprocess
import time
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

from swarmdeck_server.api.simulation_reset import request_reset, reset_status
from swarmdeck_server.fleet.registry import Registry
from deploy.simulation_reset import (
    Supervisor,
    ensure_deployment_env,
    next_domain,
    write_deployment_env,
)


def mark_supervisor_live(root):
    root.mkdir(parents=True, exist_ok=True)
    (root / "supervisor.json").write_text(
        json.dumps(
            {
                "version": 1,
                "pid": 123,
                "updated_at_ns": time.time_ns(),
            }
        )
    )


def test_request_is_atomic_and_coalesces_active_reset(tmp_path):
    mark_supervisor_live(tmp_path)
    first = request_reset(tmp_path)
    assert first["phase"] == "accepted"
    request = json.loads((tmp_path / "request.json").read_text())
    assert request["request_id"] == first["request_id"]
    (tmp_path / "status.json").write_text(json.dumps(first))
    assert request_reset(tmp_path)["request_id"] == first["request_id"]


def test_lost_post_response_retry_returns_completion_without_second_reset(tmp_path):
    mark_supervisor_live(tmp_path)
    request_id = "894b5812-2c98-48b8-b617-379abc852d9e"
    accepted = request_reset(tmp_path, request_id)
    assert accepted["request_id"] == request_id
    assert request_reset(tmp_path, request_id)["request_id"] == request_id
    completed = {**accepted, "phase": "done", "ok": True}
    (tmp_path / "status.json").write_text(json.dumps(completed))

    assert request_reset(tmp_path, request_id) == completed
    assert (
        json.loads((tmp_path / "request.json").read_text())["request_id"] == request_id
    )


def test_client_request_id_must_be_a_canonical_uuid(tmp_path):
    mark_supervisor_live(tmp_path)
    for invalid in ("not-a-uuid", "894B5812-2C98-48B8-B617-379ABC852D9E"):
        try:
            request_reset(tmp_path, invalid)
        except ValueError as exc:
            assert "request_id" in str(exc)
        else:
            raise AssertionError(f"accepted invalid request ID {invalid}")


def test_navigation_readiness_is_cleared_on_reconnect_and_updated_from_state():
    registry = Registry()
    hello = {"robot_id": "r0", "capabilities": ["navigate"]}
    robot = registry.hello(hello, object())
    registry.update_state({"robot_id": "r0", "navigation_ready": True})
    assert robot.to_state()["navigation_ready"] is True
    registry.hello(hello, object())
    assert robot.to_state()["navigation_ready"] is None


def test_concurrent_requests_are_coalesced_to_one_identity(tmp_path):
    mark_supervisor_live(tmp_path)
    with ThreadPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(lambda _: request_reset(tmp_path), range(32)))
    assert len({result["request_id"] for result in results}) == 1
    assert json.loads((tmp_path / "request.json").read_text())["request_id"] == (
        results[0]["request_id"]
    )


def test_request_fails_immediately_when_no_supervisor_is_watching(tmp_path):
    result = request_reset(tmp_path)
    assert result["phase"] == "failed"
    assert "unavailable" in result["error"]
    assert not (tmp_path / "request.json").exists()


def test_missing_status_is_idle(tmp_path):
    assert reset_status(tmp_path) == {
        "version": 1,
        "phase": "idle",
        "ok": None,
        "supervisor_available": False,
    }


def test_completed_reset_does_not_prove_a_live_supervisor(tmp_path):
    completed = {"version": 1, "phase": "succeeded", "ok": True}
    (tmp_path / "status.json").write_text(json.dumps(completed))
    assert reset_status(tmp_path) == {**completed, "supervisor_available": False}
    mark_supervisor_live(tmp_path)
    assert reset_status(tmp_path) == {**completed, "supervisor_available": True}
    for timestamp in (
        time.time_ns() - 20_000_000_000,
        time.time_ns() + 20_000_000_000,
        True,
    ):
        (tmp_path / "supervisor.json").write_text(
            json.dumps({"updated_at_ns": timestamp})
        )
        assert reset_status(tmp_path) == {**completed, "supervisor_available": False}


def test_stale_accepted_request_fails_instead_of_queueing_forever(tmp_path):
    stale = {
        "version": 1,
        "request_id": "old",
        "phase": "accepted",
        "ok": None,
        "updated_at_ns": time.time_ns() - 20_000_000_000,
    }
    (tmp_path / "status.json").write_text(json.dumps(stale))
    (tmp_path / "request.json").write_text(json.dumps(stale))
    result = request_reset(tmp_path)
    assert result["phase"] == "failed"
    assert "unavailable" in result["error"]
    assert reset_status(tmp_path)["phase"] == "failed"


def test_supervisor_stops_before_new_epoch_and_recreates(tmp_path):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\n")
    request = {"version": 1, "request_id": "request-1", "requested_at_ns": 1}
    (root / "request.json").parent.mkdir()
    (root / "request.json").write_text(json.dumps(request))
    supervisor = Supervisor(
        root, env, ["docker", "compose", "-p", "test"], ["sim", "peer0"]
    )
    with patch(
        "deploy.simulation_reset.subprocess.run",
        return_value=types.SimpleNamespace(stdout="sim\npeer0\n"),
    ) as run:
        assert supervisor.poll_once()
    assert run.call_args_list[0].args[0][-3:] == ["stop", "sim", "peer0"]
    start = run.call_args_list[1].args[0]
    assert start[-2:] == ["sim", "peer0"]
    assert start[start.index("up") : start.index("sim")] == [
        "up",
        "-d",
        "--force-recreate",
        "--wait",
        "--wait-timeout",
        "120",
    ]
    values = dict(line.split("=", 1) for line in env.read_text().splitlines())
    assert values["SWARMDECK_MISSION_ID"] != "old"
    assert values["SWARMDECK_TEST_DOMAIN"] == "202"
    assert json.loads((root / "status.json").read_text())["phase"] == "done"
    assert run.call_count == 3, "service liveness must be checked without robot gating"
    assert not supervisor.poll_once()


def test_supervisor_prunes_earlier_missions_between_stop_and_start(tmp_path):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=200\n")
    request = {"version": 1, "request_id": "request-prune", "requested_at_ns": 1}
    supervisor = Supervisor(
        root,
        env,
        ["docker", "compose", "-p", "test"],
        ["sim"],
        prune_volume="test_peer_maps",
        prune_image="swarmdeck-mapping:test",
    )
    with patch(
        "deploy.simulation_reset.subprocess.run",
        return_value=types.SimpleNamespace(stdout="sim\n"),
    ) as run:
        supervisor.run(request)
    stop, prune, start = (call.args[0] for call in run.call_args_list[:3])
    assert stop[-2:] == ["stop", "sim"]
    assert prune == [
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--volume",
        "test_peer_maps:/maps",
        "--entrypoint",
        "sh",
        "swarmdeck-mapping:test",
        "-c",
        "find /maps -mindepth 1 -maxdepth 1 -exec rm -rf {} +",
    ]
    assert "up" in start
    # The one-off container belongs to the new epoch, never to the old one.
    assert run.call_args_list[1].kwargs["env"]["SWARMDECK_MISSION_ID"] != "old"
    assert json.loads((root / "status.json").read_text())["phase"] == "done"


def test_epoch_update_preserves_deployment_settings_and_mode(tmp_path):
    env = tmp_path / "deployment.env"
    env.write_text(
        "# planning bench\nSWARMDECK_CONFIG=/app/configs/bench.yaml\n"
        "SWARMDECK_TEST_SERVER_PORT=18080\nSWARMDECK_MISSION_ID=old\n"
        "SWARMDECK_TEST_DOMAIN=201\n"
    )
    env.chmod(0o640)
    write_deployment_env(env, "new", 202)
    contents = env.read_text()
    assert "# planning bench" in contents
    assert "SWARMDECK_CONFIG=/app/configs/bench.yaml" in contents
    assert "SWARMDECK_TEST_SERVER_PORT=18080" in contents
    assert "SWARMDECK_MISSION_ID=new" in contents
    assert "SWARMDECK_TEST_DOMAIN=202" in contents
    assert env.stat().st_mode & 0o777 == 0o640


def test_new_epoch_overrides_inherited_shell_values_for_compose(tmp_path, monkeypatch):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=file-old\nSWARMDECK_TEST_DOMAIN=201\n")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "shell-old")
    monkeypatch.setenv("SWARMDECK_TEST_DOMAIN", "99")
    request = {"version": 1, "request_id": "request-env", "requested_at_ns": 1}
    supervisor = Supervisor(root, env, ["docker", "compose"], ["sim"])
    with patch(
        "deploy.simulation_reset.subprocess.run",
        return_value=types.SimpleNamespace(stdout="sim\n"),
    ) as run:
        supervisor.run(request)
    stop_env = run.call_args_list[0].kwargs["env"]
    start_env = run.call_args_list[1].kwargs["env"]
    assert stop_env["SWARMDECK_MISSION_ID"] == "file-old"
    assert stop_env["SWARMDECK_TEST_DOMAIN"] == "201"
    assert start_env["SWARMDECK_MISSION_ID"] not in {"file-old", "shell-old"}
    assert start_env["SWARMDECK_TEST_DOMAIN"] == "202"


def test_missing_env_file_is_seeded_before_compose_stop(tmp_path, monkeypatch):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "active")
    monkeypatch.setenv("SWARMDECK_TEST_DOMAIN", "41")
    request = {"version": 1, "request_id": "request-bootstrap", "requested_at_ns": 1}
    supervisor = Supervisor(root, env, ["docker", "compose"], ["sim"])
    seen = []

    def run(*args, **kwargs):
        seen.append((env.exists(), env.read_text(), kwargs["env"]))
        return types.SimpleNamespace(stdout="sim\n")

    with patch("deploy.simulation_reset.subprocess.run", side_effect=run):
        supervisor.run(request)
    assert seen[0][0] is True
    assert "SWARMDECK_MISSION_ID=active" in seen[0][1]
    assert seen[1][2]["SWARMDECK_TEST_DOMAIN"] == "42"


def test_domain_wraps_inside_supported_range():
    assert next_domain(232) == 1


def test_domain_rejects_values_outside_supported_range():
    for invalid in (-1, 0, 233):
        try:
            next_domain(invalid)
        except ValueError as exc:
            assert "1..232" in str(exc)
        else:
            raise AssertionError(f"accepted invalid DDS domain {invalid}")


def test_existing_environment_rejects_invalid_domain_before_compose(tmp_path):
    env = tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=999\n")
    try:
        ensure_deployment_env(env)
    except ValueError as exc:
        assert "1..232" in str(exc)
    else:
        raise AssertionError("accepted invalid DDS domain from deployment env")


def test_stop_failure_preserves_previous_epoch(tmp_path):
    import subprocess

    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\n")
    request = {"version": 1, "request_id": "request-2", "requested_at_ns": 1}
    supervisor = Supervisor(root, env, ["docker", "compose"], ["sim"])
    with patch(
        "deploy.simulation_reset.subprocess.run",
        side_effect=subprocess.CalledProcessError(1, ["docker"]),
    ):
        supervisor.run(request)
    assert "SWARMDECK_MISSION_ID=old" in env.read_text()
    status = json.loads((root / "status.json").read_text())
    assert status["phase"] == "failed" and status["ok"] is False


def test_restarted_supervisor_recovers_claimed_request(tmp_path):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    root.mkdir()
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\n")
    request = {"version": 1, "request_id": "interrupted", "requested_at_ns": 1}
    (root / "request.json").write_text(json.dumps(request))
    (root / "status.json").write_text(
        json.dumps(
            {
                **request,
                "phase": "starting",
                "ok": None,
                "updated_at_ns": time.time_ns(),
            }
        )
    )
    supervisor = Supervisor(root, env, ["docker", "compose"], ["sim"])
    with patch.object(supervisor, "run") as run:
        assert supervisor.poll_once(recover_active=True)
    run.assert_called_once_with(request)
    assert json.loads((root / "status.json").read_text())["phase"] == "stopping"


def test_verification_waits_for_navigation_not_just_online_adapter(tmp_path):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\n")
    request = {"version": 1, "request_id": "ready-check", "requested_at_ns": 1}
    supervisor = Supervisor(
        root,
        env,
        ["docker", "compose"],
        ["sim"],
        server_url="http://server",
        expected_robots=1,
    )
    bodies = [
        {
            "robots": [
                {
                    "robot_id": "hardware",
                    "online": True,
                    "navigation_ready": True,
                    "capabilities": ["navigate"],
                },
                {
                    "robot_id": "r0",
                    "online": True,
                    "navigation_ready": False,
                    "capabilities": ["navigate", "reset"],
                },
            ]
        },
        {
            "robots": [
                {
                    "robot_id": "r0",
                    "online": True,
                    "navigation_ready": True,
                    "capabilities": ["navigate", "reset"],
                }
            ]
        },
    ]

    class Response:
        def __init__(self, body):
            self.body = body

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def read(self):
            return json.dumps(self.body).encode()

    with (
        patch(
            "deploy.simulation_reset.subprocess.run",
            return_value=types.SimpleNamespace(stdout="sim\n"),
        ),
        patch("deploy.simulation_reset.time.sleep"),
        patch(
            "deploy.simulation_reset.urllib.request.urlopen",
            side_effect=[Response(body) for body in bodies],
        ) as urlopen,
    ):
        supervisor.run(request)
    assert urlopen.call_count == 2
    assert json.loads((root / "status.json").read_text())["phase"] == "done"


def test_verification_fails_if_a_requested_service_exited(tmp_path):
    root, env = tmp_path / "reset", tmp_path / "deployment.env"
    env.write_text("SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\n")
    request = {"version": 1, "request_id": "exit-check", "requested_at_ns": 1}
    supervisor = Supervisor(
        root,
        env,
        ["docker", "compose"],
        ["sim", "argos"],
        server_url="http://server",
        expected_robots=1,
    )

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return False

        def read(self):
            return json.dumps(
                {
                    "robots": [
                        {
                            "robot_id": "r0",
                            "online": True,
                            "navigation_ready": True,
                            "capabilities": ["navigate", "reset"],
                        }
                    ]
                }
            ).encode()

    commands = [
        types.SimpleNamespace(stdout=""),
        types.SimpleNamespace(stdout=""),
        types.SimpleNamespace(stdout="sim\n"),
    ]
    with (
        patch("deploy.simulation_reset.subprocess.run", side_effect=commands),
        patch(
            "deploy.simulation_reset.urllib.request.urlopen", return_value=Response()
        ),
    ):
        supervisor.run(request)
    status = json.loads((root / "status.json").read_text())
    assert status["phase"] == "failed"
    assert "argos" in status["error"]


def test_rendered_onboard_compose_uses_one_resettable_ros_domain():
    if shutil.which("docker") is None:
        return
    repo = Path(__file__).resolve().parents[2]
    compose = repo / "deploy" / "compose"
    files = [
        "docker-compose.yml",
    ]
    command = ["docker", "compose", "-p", "reset-contract"]
    for name in files:
        command.extend(("-f", str(compose / name)))
    command.extend(("--profile", "*", "config", "--format", "json"))
    environment = dict(os.environ)
    environment.update(
        SWARMDECK_MISSION_ID="00000000-0000-0000-0000-000000000001",
        SWARMDECK_TEST_DOMAIN="201",
    )
    rendered = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        cwd=repo,
        env=environment,
        timeout=30,
    )
    services = json.loads(rendered.stdout)["services"]
    assert {
        services[name]["environment"]["ROS_DOMAIN_ID"]
        for name in ("sim", "mgg", "peer0")
    } == {"201"}


def test_fleet_reset_without_supervisor_fails_without_touching_adapters(monkeypatch):
    """Every adapter resets through the epoch supervisor; there is no fallback."""
    import asyncio

    from swarmdeck_server.api import state

    class Sink:
        def __init__(self):
            self.messages = []

        async def send_json(self, message):
            self.messages.append(message)

    monkeypatch.delenv("SWARMDECK_SIM_RESET_DIR", raising=False)
    registry = Registry()
    sink = Sink()
    registry.hello({"robot_id": "robot_0", "capabilities": ["navigate", "reset"]}, sink)
    registry.robots["robot_0"].goal = {"x": 1.0, "y": 2.0}
    broadcasts = []

    async def broadcast(message):
        broadcasts.append(message)

    monkeypatch.setattr(state, "registry", registry)
    monkeypatch.setattr(state, "broadcast", broadcast)
    monkeypatch.setitem(state._detections, "kept", {"id": "kept"})

    result = asyncio.run(state.reset_fleet())

    assert result["phase"] == "failed"
    assert result["ok"] is False
    assert "SWARMDECK_SIM_RESET_DIR" in result["error"]
    assert sink.messages == []
    assert registry.robots["robot_0"].goal == {"x": 1.0, "y": 2.0}
    assert "kept" in state._detections
    assert broadcasts == [
        {
            "type": "sim_reset",
            "phase": "done",
            "request_id": None,
            "ok": False,
            "error": result["error"],
            "skipped": [],
        }
    ]
