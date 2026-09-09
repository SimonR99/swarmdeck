"""Exercise public Make commands without starting Docker or touching hardware."""

import json
import os
from pathlib import Path
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def command_env(tmp_path):
    docker = tmp_path / "docker"
    docker.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, sys\n"
        "with open(os.environ['CAPTURE'], 'a') as stream:\n"
        "    stream.write(json.dumps({'args': sys.argv[1:], "
        "'config': os.environ.get('SWARMDECK_CONFIG'), "
        "'odometry': os.environ.get('SWARMDECK_ODOMETRY'), "
        "'explore': os.environ.get('EXPLORE_SECONDS')}) + '\\n')\n"
    )
    docker.chmod(0o755)
    capture = tmp_path / "commands.jsonl"
    env = dict(
        os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}", CAPTURE=str(capture)
    )
    return env, capture


def run_make(*args, env, cwd=REPO):
    return subprocess.run(
        ["make", "--no-print-directory", *args],
        cwd=cwd,
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )


def calls(capture):
    return [json.loads(line) for line in capture.read_text().splitlines()]


def test_default_simulation_is_argos_and_does_not_start_exploring(command_env):
    env, capture = command_env
    run_make("up-sim", "COMPOSE_PROJECT=review-test", env=env)
    call = calls(capture)[0]
    assert call["args"][call["args"].index("--profile") + 1] == "argos"
    assert call["args"][call["args"].index("-p") + 1] == "review-test"
    assert "fast_livo2" in call["args"]
    assert "mgg" in call["args"]
    assert "gazebo" not in call["args"]
    assert "agent" not in call["args"]
    assert call["config"] == "/app/configs/4robot.yaml"
    assert call["odometry"] == "fast_livo2"
    assert call["explore"] == "0"


@pytest.mark.parametrize("render", ["software", "gpu", "dri"])
@pytest.mark.parametrize("odometry", ["fast_livo2", "drift"])
def test_build_and_start_select_the_same_stack(command_env, render, odometry):
    env, capture = command_env
    options = [
        "COMPOSE_PROJECT=review-test",
        "SCENARIO=bistro",
        f"RENDER={render}",
        f"ODOMETRY={odometry}",
    ]
    run_make("build-sim", *options, env=env)
    run_make("up-sim", *options, "SIM_ARGS=--no-build", env=env)
    build, start = calls(capture)
    build_index = build["args"].index("build")
    start_index = start["args"].index("up")
    assert build["args"][:build_index] == start["args"][:start_index]
    assert build["args"][build_index + 1 :] == start["args"][start_index + 2 :]
    assert "up" not in build["args"]
    assert build["config"] == start["config"] == "/app/configs/4robot_bistro.yaml"
    assert ("fast_livo2" in build["args"]) == (odometry != "drift")
    if render != "software":
        assert any(f"docker-compose.{render}.yml" in arg for arg in build["args"])


def test_down_sim_stops_argos_and_preserves_core_services(command_env):
    env, capture = command_env
    run_make("down-sim", "COMPOSE_PROJECT=review-test", env=env)
    stop, remove, volume = calls(capture)
    assert stop["args"][-5:] == ["stop", "mgg", "argos", "sim", "fast_livo2"]
    assert remove["args"][-6:] == ["rm", "-f", "mgg", "argos", "sim", "fast_livo2"]
    assert volume["args"] == ["volume", "rm", "review-test_swarmdeck_runtime"]


def test_help_and_dry_run_do_not_execute_docker(command_env):
    env, capture = command_env
    help_result = run_make(env=env)
    assert "up-sim" in help_result.stdout
    assert "docker-purge" in help_result.stdout
    run_make("up-sim", "SIM_ARGS=--dry-run", env=env)
    run_make("build-sim", "SIM_ARGS=--dry-run", env=env)
    assert not capture.exists()


def test_clean_only_removes_local_outputs(command_env, tmp_path):
    env, capture = command_env
    # Execute the real recipe in an isolated checkout with disposable sentinels.
    (tmp_path / "Makefile").write_text((REPO / "Makefile").read_text())
    generated = [
        "server/.venv",
        "slam/.venv",
        "agent/.venv",
        "ui/dist",
        "swarmdeck_ros/build",
    ]
    retained = ["sessions/captures", "research/notes", "server/swarmdeck_server"]
    for relative in generated + retained:
        path = tmp_path / relative
        path.mkdir(parents=True)
        (path / "sentinel").write_text("keep or remove according to scope")
    run_make("clean", env=env, cwd=tmp_path)
    assert all(not (tmp_path / path).exists() for path in generated)
    assert all((tmp_path / path / "sentinel").exists() for path in retained)
    assert not capture.exists()
