"""The public simulation launcher selects one complete, coherent stack."""

import json
from pathlib import Path

import pytest

from deploy import simulation_launch as launch

REPO = Path(__file__).resolve().parents[2]


def arguments(*values):
    return launch.parser().parse_args(list(values))


def test_default_stack_has_one_backend_and_all_products(monkeypatch):
    monkeypatch.delenv("SWARMDECK_SLAM_BACKEND", raising=False)
    spec = launch.build_spec(arguments("--drift", "--no-build"), "test")
    assert spec["version"] == 2
    assert "backend" not in spec
    assert spec["robot_names"] == ["robot_0", "robot_1", "robot_2", "robot_3"]
    assert {"peer0", "peer1", "peer2", "peer3"}.issubset(spec["services"])
    assert {"mapping", "mgg"}.issubset(spec["services"])
    assert "slam" not in spec["services"]
    assert spec["compose_files"] == [str(launch.COMPOSE / "docker-compose.yml")]
    environment = launch.process_environment(
        spec, {"SWARMDECK_MISSION_ID": "mission", "SWARMDECK_TEST_DOMAIN": "42"}
    )
    assert environment["SWARMDECK_SLAM_BACKEND"] == "cslam"
    assert environment["SWARMDECK_MOLA_PLANNER_MAPS"] == "true"
    assert "SWARMDECK_MGG_MAP_BACKEND" not in environment
    assert json.loads(environment["SWARMDECK_PEER_NAMES"]) == spec["robot_names"]


def test_unknown_backend_fails_closed(monkeypatch):
    monkeypatch.setenv("SWARMDECK_SLAM_BACKEND", "rtabmap")
    with pytest.raises(ValueError, match="SWARMDECK_SLAM_BACKEND"):
        launch.build_spec(arguments("--drift"), "test")


def test_removed_mapping_flags_are_not_accepted():
    with pytest.raises(SystemExit):
        arguments("--legacy-cloud")
    with pytest.raises(SystemExit):
        arguments("--mola")


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


def test_custom_scenario_mounts_config_and_selects_exact_peers(tmp_path):
    config = tmp_path / "two.yaml"
    config.write_text("fleet:\n  robot_count: 2\n  robot_prefix: rover_\n")
    spec = launch.build_spec(
        arguments("--scenario", str(config), "--fast-livo2"), "test"
    )
    assert spec["robot_names"] == ["rover_0", "rover_1"]
    assert [service for service in spec["services"] if service.startswith("peer")] == [
        "peer0",
        "peer1",
    ]
    overlay = Path(spec["compose_files"][-1])
    assert str(config) in overlay.read_text()


def test_custom_scenario_rejects_more_than_four_peers(tmp_path):
    config = tmp_path / "five.yaml"
    config.write_text("fleet:\n  robot_count: 5\n  robot_prefix: robot_\n")
    with pytest.raises(ValueError, match="at most 4 peers"):
        launch.build_spec(arguments("--scenario", str(config)), "test")


def test_state_loader_rejects_obsolete_versions(tmp_path, monkeypatch):
    monkeypatch.setattr(launch, "STATE_ROOT", tmp_path)
    (tmp_path / "test.json").write_text(
        json.dumps(
            {
                "version": 1,
                "project": "test",
                "compose_files": [],
                "services": [],
                "reset_services": [],
            }
        )
    )
    with pytest.raises(ValueError, match="obsolete"):
        launch.load_state("test")


def test_fast_livo2_is_explicitly_selectable():
    spec = launch.build_spec(arguments("--fast-livo2"), "test")
    assert spec["services"][-1] == "fast_livo2"
    assert "fast_livo2" in spec["reset_services"]
