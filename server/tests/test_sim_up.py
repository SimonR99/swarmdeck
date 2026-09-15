"""The public simulation launcher selects one complete, coherent stack."""

import json
import os
from pathlib import Path
import subprocess
import types
from unittest.mock import patch

import pytest

from deploy import simulation_launch as launch

REPO = Path(__file__).resolve().parents[2]


def arguments(*values):
    return launch.parser().parse_args(list(values))


def test_default_stack_is_mola_with_four_scenario_peers():
    spec = launch.build_spec(arguments("--drift", "--no-build"), "test")

    assert spec["backend"] == "mola"
    assert spec["robot_names"] == ["robot_0", "robot_1", "robot_2", "robot_3"]
    assert [service for service in spec["services"] if service.startswith("peer")] == [
        "peer0",
        "peer1",
        "peer2",
        "peer3",
    ]
    assert "mapping" in spec["services"]
    assert "mapping-query" in spec["services"]
    assert "fast_livo2" not in spec["services"]
    assert str(launch.COMPOSE / "docker-compose.peers.yml") in spec["compose_files"]
    assert str(launch.COMPOSE / "docker-compose.mapping.yml") in spec["compose_files"]
    assert (
        str(launch.COMPOSE / "docker-compose.onboard-planning.yml")
        in spec["compose_files"]
    )
    environment = launch.process_environment(
        spec,
        {"SWARMDECK_MISSION_ID": "mission", "SWARMDECK_TEST_DOMAIN": "42"},
    )
    assert environment["SWARMDECK_CAPTURE_PROVIDER"] == "simulation"
    assert json.loads(environment["SWARMDECK_PEER_NAMES"]) == spec["robot_names"]
    assert environment["SWARMDECK_MGG_MAP_BACKEND"] == "mola_snapshot"
    assert environment["SWARMDECK_PLANNER_MAP_PROVIDER"] == "mola"
    assert environment["SWARMDECK_MOLA_PLANNER_MAPS"] == "true"


def test_backend_selection_overrides_contradictory_host_environment(monkeypatch):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "cloud_octomap")
    monkeypatch.setenv("SWARMDECK_PLANNER_MAP_PROVIDER", "indexed")
    monkeypatch.setenv("SWARMDECK_MOLA_PLANNER_MAPS", "false")

    mola = launch.build_spec(arguments("--mola"), "test")
    mola_environment = launch.process_environment(mola)
    legacy = launch.build_spec(arguments("--legacy-cloud"), "test")
    legacy_environment = launch.process_environment(legacy)

    assert mola_environment["SWARMDECK_MGG_MAP_BACKEND"] == "mola_snapshot"
    assert mola_environment["SWARMDECK_PLANNER_MAP_PROVIDER"] == "mola"
    assert mola_environment["SWARMDECK_MOLA_PLANNER_MAPS"] == "true"
    assert legacy_environment["SWARMDECK_MGG_MAP_BACKEND"] == "cloud_octomap"
    assert legacy_environment["SWARMDECK_PLANNER_MAP_PROVIDER"] == "indexed"
    assert legacy_environment["SWARMDECK_MOLA_PLANNER_MAPS"] == "false"


def test_dev_selects_three_peers_and_never_fast_livo2():
    spec = launch.build_spec(arguments("--dev"), "test")

    assert spec["render"] == "dri"
    assert spec["odometry"] == "drift"
    assert [service for service in spec["services"] if service.startswith("peer")] == [
        "peer0",
        "peer1",
        "peer2",
    ]
    assert "fast_livo2" not in spec["services"]


def test_fast_livo2_is_explicitly_selectable():
    spec = launch.build_spec(arguments("--fast-livo2"), "test")
    assert spec["services"][-1] == "fast_livo2"
    assert "fast_livo2" in spec["reset_services"]


def test_legacy_cloud_omits_onboard_mapping_overlays():
    spec = launch.build_spec(arguments("--legacy-cloud", "--drift"), "test")

    assert spec["backend"] == "legacy-cloud"
    assert not any(service.startswith("peer") for service in spec["services"])
    assert "mapping" not in spec["services"]
    assert "mapping-query" not in spec["services"]
    assert all("peers.yml" not in path for path in spec["compose_files"])
    assert all("mapping.yml" not in path for path in spec["compose_files"])
    assert all("onboard-planning.yml" not in path for path in spec["compose_files"])


def test_custom_scenario_mounts_config_and_selects_exact_peers(tmp_path, monkeypatch):
    config = tmp_path / "two.yaml"
    config.write_text("fleet:\n  robot_count: 2\n  robot_prefix: rover_\n")
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path / "state")

    spec = launch.build_spec(
        arguments("--scenario", str(config), "--fast-livo2"), "test"
    )

    assert spec["robot_names"] == ["rover_0", "rover_1"]
    assert [service for service in spec["services"] if service.startswith("peer")] == [
        "peer0",
        "peer1",
    ]
    assert "fast_livo2" in spec["services"]
    overlay = Path(spec["compose_files"][-1])
    assert str(config) in overlay.read_text()
    assert "/app/configs/swarmdeck-launch.yaml" in overlay.read_text()


def test_custom_scenario_cannot_silently_omit_extra_peers(tmp_path):
    config = tmp_path / "five.yaml"
    config.write_text("fleet:\n  robot_count: 5\n  robot_prefix: robot_\n")
    with pytest.raises(ValueError, match="at most 4 peers"):
        launch.build_spec(arguments("--scenario", str(config)), "test")


def test_custom_scenario_rejects_invalid_ros_robot_prefix(tmp_path):
    config = tmp_path / "invalid-prefix.yaml"
    config.write_text("fleet:\n  robot_count: 2\n  robot_prefix: bad-name-\n")
    with pytest.raises(ValueError, match="ROS namespace"):
        launch.build_spec(arguments("--scenario", str(config)), "test")


def test_new_launch_advances_persisted_epoch_without_dropping_other_settings(
    tmp_path, monkeypatch
):
    reset_root = tmp_path / "reset"
    env_file = reset_root / "deployment.env"
    env_file.parent.mkdir()
    env_file.write_text(
        "SWARMDECK_MISSION_ID=old\nSWARMDECK_TEST_DOMAIN=201\nKEEP_ME=yes\n"
    )
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", env_file)

    epoch = launch.new_epoch()

    assert epoch["SWARMDECK_MISSION_ID"] != "old"
    assert epoch["SWARMDECK_TEST_DOMAIN"] == "202"
    assert epoch["KEEP_ME"] == "yes"


def test_lifecycle_reuses_saved_compose_stack(tmp_path, monkeypatch):
    reset_root = tmp_path / "reset"
    state_root = tmp_path / "state"
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", reset_root / "deployment.env")
    monkeypatch.setattr(launch, "STATE_ROOT", state_root)
    calls = []

    def run(argv, **kwargs):
        calls.append((argv, kwargs.get("env", {})))
        return types.SimpleNamespace(returncode=0)

    with (
        patch("deploy.simulation_launch.subprocess.run", side_effect=run),
        patch("deploy.simulation_launch.start_supervisor"),
    ):
        assert launch.main(["--scenario", "3robot", "--gpu", "--drift"]) == 0
        assert launch.main(["--status"]) == 0

    up = next(call for call in calls if "up" in call[0])
    status = next(call for call in calls if call[0][-1] == "ps")
    assert str(launch.COMPOSE / "docker-compose.gpu.yml") in up[0]
    assert str(launch.COMPOSE / "docker-compose.gpu.yml") in status[0]
    assert up[1]["SWARMDECK_ROBOT_COUNT"] == "3"
    assert status[1]["SWARMDECK_ROBOT_COUNT"] == "3"
    assert all(peer in up[0] for peer in ("peer0", "peer1", "peer2"))
    assert "peer3" not in up[0]
    assert "fast_livo2" not in up[0]
    assert status[0][-1] == "ps"


def test_changed_launch_retires_only_services_absent_from_new_stack(
    tmp_path, monkeypatch
):
    reset_root = tmp_path / "reset"
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", reset_root / "deployment.env")
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path / "state")
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        return types.SimpleNamespace(returncode=0)

    with (
        patch("deploy.simulation_launch.subprocess.run", side_effect=run),
        patch("deploy.simulation_launch.start_supervisor"),
        patch("deploy.simulation_launch.stop_supervisor"),
    ):
        assert launch.main(["--scenario", "4robot", "--fast-livo2"]) == 0
        calls.clear()
        assert launch.main(["--dev"]) == 0

    stop = next(call for call in calls if "stop" in call)
    remove = next(call for call in calls if "rm" in call)
    assert set(stop[stop.index("stop") + 1 :]) == {
        "server",
        "slam",
        "sim",
        "argos",
        "mgg",
        "peer0",
        "peer1",
        "peer2",
        "peer3",
        "mapping",
        "mapping-query",
        "fast_livo2",
    }
    assert remove[remove.index("rm") + 2 :] == ["peer3", "fast_livo2"]
    up = next(call for call in calls if "up" in call)
    assert all(peer in up for peer in ("peer0", "peer1", "peer2"))
    assert "peer3" not in up
    assert "fast_livo2" not in up


def test_switch_to_legacy_retires_every_mola_only_service(tmp_path, monkeypatch):
    reset_root = tmp_path / "reset"
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", reset_root / "deployment.env")
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path / "state")
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        return types.SimpleNamespace(returncode=0)

    with (
        patch("deploy.simulation_launch.subprocess.run", side_effect=run),
        patch("deploy.simulation_launch.start_supervisor"),
        patch("deploy.simulation_launch.stop_supervisor"),
    ):
        assert launch.main(["--drift"]) == 0
        calls.clear()
        assert launch.main(["--legacy-cloud", "--drift"]) == 0

    stop = next(call for call in calls if "stop" in call)
    assert stop[stop.index("stop") + 1 :] == [
        "server",
        "slam",
        "sim",
        "argos",
        "mgg",
        "peer0",
        "peer1",
        "peer2",
        "peer3",
        "mapping",
        "mapping-query",
    ]


def test_up_stops_prior_mission_before_epoch_and_force_recreates_stack(
    tmp_path, monkeypatch
):
    reset_root = tmp_path / "reset"
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", reset_root / "deployment.env")
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path / "state")
    events = []

    def run(argv, **kwargs):
        events.append(("docker", argv))
        return types.SimpleNamespace(returncode=0)

    original_new_epoch = launch.new_epoch

    def new_epoch():
        events.append(("epoch", None))
        return original_new_epoch()

    with (
        patch("deploy.simulation_launch.subprocess.run", side_effect=run),
        patch("deploy.simulation_launch.new_epoch", side_effect=new_epoch),
        patch("deploy.simulation_launch.start_supervisor"),
        patch("deploy.simulation_launch.stop_supervisor"),
    ):
        assert launch.main(["--dev", "--no-build"]) == 0

    stop_index = next(
        index
        for index, event in enumerate(events)
        if event[0] == "docker" and "stop" in event[1]
    )
    epoch_index = next(
        index for index, event in enumerate(events) if event[0] == "epoch"
    )
    up_index = next(
        index
        for index, event in enumerate(events)
        if event[0] == "docker" and "up" in event[1]
    )
    assert stop_index < epoch_index < up_index
    stop = events[stop_index][1]
    assert "argos" in stop and "sim" in stop and "mgg" in stop
    up = events[up_index][1]
    assert up[up.index("up") + 1 : up.index("server")] == [
        "--force-recreate",
        "-d",
    ]
    assert "fast_livo2" not in up


@pytest.mark.parametrize(
    ("arguments", "registry", "retired"),
    [
        (
            ["--dev", "--no-build"],
            "0123456789ab\tfast_livo2\n123456789abc\tpeer3\n",
            {"0123456789ab", "123456789abc"},
        ),
        (
            ["--legacy-cloud", "--drift", "--no-build"],
            (
                "0123456789ab\tpeer0\n"
                "123456789abc\tmapping\n"
                "23456789abcd\tmapping-query\n"
                "3456789abcde\tserver\n"
            ),
            {"0123456789ab", "123456789abc", "23456789abcd"},
        ),
    ],
)
def test_first_launch_retires_only_superseded_known_sidecars(
    tmp_path, monkeypatch, arguments, registry, retired
):
    reset_root = tmp_path / "reset"
    monkeypatch.setattr(launch, "RESET_ROOT", reset_root)
    monkeypatch.setattr(launch, "ENV_FILE", reset_root / "deployment.env")
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path / "state")
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        stdout = registry if argv[:3] == ["docker", "ps", "-a"] else ""
        return types.SimpleNamespace(returncode=0, stdout=stdout)

    with (
        patch("deploy.simulation_launch.subprocess.run", side_effect=run),
        patch("deploy.simulation_launch.start_supervisor"),
        patch("deploy.simulation_launch.stop_supervisor"),
    ):
        assert launch.main(arguments) == 0

    direct_stop = next(call for call in calls if call[:2] == ["docker", "stop"])
    direct_remove = next(call for call in calls if call[:3] == ["docker", "rm", "-f"])
    assert set(direct_stop[2:]) == retired
    assert set(direct_remove[3:]) == retired
    assert "3456789abcde" not in direct_stop


def test_supervisor_rejects_known_other_compose_project(monkeypatch):
    monkeypatch.setattr(launch, "supervisor_owner", lambda: "other-project")
    with pytest.raises(RuntimeError, match="other-project"):
        launch.stop_supervisor("this-project")


@pytest.mark.parametrize("render", ["software", "gpu", "dri"])
def test_script_keeps_legacy_cloud_quickstart_working(tmp_path, render):
    docker = tmp_path / "docker"
    docker.write_text("""#!/usr/bin/env python3
import json, os, sys
with open(os.environ['CAPTURE'], 'w') as stream:
    json.dump({'args': sys.argv[1:], 'config': os.environ.get('SWARMDECK_CONFIG')}, stream)
""")
    docker.chmod(0o755)
    capture = tmp_path / "call.json"
    env = dict(
        os.environ,
        PATH=f'{tmp_path}:{os.environ["PATH"]}',
        CAPTURE=str(capture),
        COMPOSE_PROJECT=f"legacy-test-{render}",
    )
    subprocess.run(
        [
            str(REPO / "scripts/sim-up"),
            "--render",
            render,
            "--scenario",
            "bistro",
            "--legacy-cloud",
            "--drift",
            "--no-build",
        ],
        env=env,
        check=True,
        capture_output=True,
    )
    call = json.loads(capture.read_text())
    assert str(REPO / "deploy/compose/docker-compose.mgg.yml") in call["args"]
    assert "docker-compose.mapping.yml" not in " ".join(call["args"])
    assert "fast_livo2" not in call["args"]
    assert call["args"][-1] == "mgg"
    assert call["config"] == "/app/configs/4robot_bistro.yaml"
