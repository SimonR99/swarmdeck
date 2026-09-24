"""Deployment contracts for the persistent native MOLA map product."""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
COMPOSE_DIR = REPO / "deploy" / "compose"
IMPORTER = "/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import"
BOUNDED_OPTIONS = {
    "SWARMDECK_MOLA_TIMEOUT_S": "${SWARMDECK_MOLA_TIMEOUT_S:-30}",
    "SWARMDECK_MOLA_POLL_S": "${SWARMDECK_MOLA_POLL_S:-1}",
    "SWARMDECK_MOLA_RETRY_S": "${SWARMDECK_MOLA_RETRY_S:-5}",
    "SWARMDECK_MOLA_MAX_OUTPUT_BYTES": "${SWARMDECK_MOLA_MAX_OUTPUT_BYTES:-268435456}",
    "SWARMDECK_MOLA_KEEP_GENERATIONS": "${SWARMDECK_MOLA_KEEP_GENERATIONS:-2}",
}


def _mapping_service(name: str) -> dict:
    compose = yaml.safe_load((COMPOSE_DIR / "docker-compose.yml").read_text())
    return compose["services"][name]


def _peer_service(name: str) -> dict:
    compose = yaml.safe_load(
        (COMPOSE_DIR / "docker-compose.robot-peer.yml").read_text()
    )
    return compose["services"][name]


def test_mapping_image_builds_and_smokes_the_persistent_runtime():
    dockerfile = (REPO / "deploy/docker/Dockerfile.mapping").read_text()
    assert "swarmdeck-mola-import" in dockerfile
    assert "swarmdeck-mola-import --serve" in dockerfile
    assert (
        "COPY swarmdeck_ros/src/swarmdeck_peer/swarmdeck_peer/mola_process.py"
        " /usr/local/bin/mola_process.py" in dockerfile
    )
    assert "COPY swarmdeck_ros/src/swarmdeck_mola src/swarmdeck_mola" in dockerfile
    assert "--packages-select swarmdeck_mapping swarmdeck_mola" in dockerfile
    assert (
        "COPY tests/deployment/mola_jsonl_smoke.py /usr/local/bin/mola-jsonl-smoke.py"
        in dockerfile
    )
    assert (
        "COPY tests/deployment/mola_launcher_smoke.py /usr/local/bin/mola-launcher-smoke.py"
        in dockerfile
    )
    assert "timeout 45s python3 /usr/local/bin/mola-jsonl-smoke.py" in dockerfile
    assert "MOLA_MODULES_LIB_PATH=/mapping_ws/install/swarmdeck_mola/lib" in dockerfile
    assert "make_fixture.py /tmp/module-fixture" in dockerfile
    assert "timeout 30s build/swarmdeck_mola/bin/test_mola_module" in dockerfile
    assert "timeout 15s python3 /usr/local/bin/mola-launcher-smoke.py" in dockerfile
    assert "share/swarmdeck_mola/config/external-map.yaml" in dockerfile
    assert "make_fixture.py /tmp/map-fixture" in dockerfile
    assert "ctest --test-dir build/swarmdeck_mapping --output-on-failure" in dockerfile
    assert "ctest --test-dir build/swarmdeck_mola --output-on-failure" in dockerfile


def test_central_mapping_worker_is_mission_pinned_and_bounded():
    service = _mapping_service("mapping")
    env = service["environment"]
    assert env["SWARMDECK_MISSION_ID"] == (
        "${SWARMDECK_MISSION_ID:?Set a fresh fleet mission UUID}"
    )
    assert "--all-missions" not in service["command"]
    assert "--mode" not in service["command"]
    assert env["SWARMDECK_MOLA_IMPORTER"] == IMPORTER
    assert "SWARMDECK_MOLA_MODE" not in env
    for key, value in BOUNDED_OPTIONS.items():
        assert env[key] == value
    # The fleet worker builds every robot's product; peers are built
    # concurrently, one native runtime each.
    assert env["SWARMDECK_MOLA_PARALLEL_PEERS"] == "${SWARMDECK_MOLA_PARALLEL_PEERS:-4}"
    assert service["image"] == "swarmdeck-mapping:mola-native"
    assert service["command"] == [
        "swarmdeck-mola-worker",
        "--maps-root",
        "/maps",
        "--timeout",
        "${SWARMDECK_MOLA_TIMEOUT_S:-30}",
        "--poll",
        "${SWARMDECK_MOLA_POLL_S:-1}",
        "--retry",
        "${SWARMDECK_MOLA_RETRY_S:-5}",
        "--max-output-bytes",
        "${SWARMDECK_MOLA_MAX_OUTPUT_BYTES:-268435456}",
        "--keep-generations",
        "${SWARMDECK_MOLA_KEEP_GENERATIONS:-2}",
        "--parallel-peers",
        "${SWARMDECK_MOLA_PARALLEL_PEERS:-4}",
    ]


def test_robot_peer_worker_is_pinned_to_one_robot_and_uses_the_native_runtime():
    service = _peer_service("peer_mola_mapping")
    env = service["environment"]
    assert service["profiles"] == ["peer_mapping"]
    assert env["SWARMDECK_MISSION_ID"] == (
        "${SWARMDECK_MISSION_ID:?Set a fresh fleet mission UUID}"
    )
    assert "--all-missions" not in service["command"]
    assert "--mode" not in service["command"]
    assert env["SWARMDECK_MOLA_IMPORTER"] == IMPORTER
    assert "SWARMDECK_MOLA_MODE" not in env
    for key, value in BOUNDED_OPTIONS.items():
        assert env[key] == value
    # A hardware peer builds only its own product: one build at a time.
    assert env["SWARMDECK_MOLA_PARALLEL_PEERS"] == "${SWARMDECK_MOLA_PARALLEL_PEERS:-1}"
    assert service["image"] == "swarmdeck-mapping:mola-native"
    assert service["command"] == [
        "swarmdeck-mola-worker",
        "--maps-root",
        "/maps",
        "--timeout",
        "${SWARMDECK_MOLA_TIMEOUT_S:-30}",
        "--poll",
        "${SWARMDECK_MOLA_POLL_S:-1}",
        "--retry",
        "${SWARMDECK_MOLA_RETRY_S:-5}",
        "--max-output-bytes",
        "${SWARMDECK_MOLA_MAX_OUTPUT_BYTES:-268435456}",
        "--keep-generations",
        "${SWARMDECK_MOLA_KEEP_GENERATIONS:-2}",
        "--parallel-peers",
        "${SWARMDECK_MOLA_PARALLEL_PEERS:-1}",
    ]


def test_remote_acceptance_requires_explicit_source_for_builds():
    script = (REPO / "tests/deployment/mola_remote_acceptance.sh").read_text()
    assert 'if [[ -n "${SOURCE:-}" && -z "${IMAGE:-}" ]]' in script
    assert 'if [[ -z "${SOURCE:-}" && -z "${IMAGE:-}" ]]' in script
    assert "docker build --pull=false" in script
    assert "--network none" in script
    assert "os.replace(stream.name, path)" in script
    assert "swarmdeck-mola-import --serve" in script
    # The worker publishes a self-described product: source.json carries the
    # bytes index.json's source_sha256 names. The acceptance checks that pair.
    assert 'sha256sum "$tmp/maps/$mission/$robot/mola/source.json"' in script
    assert 'test "$published_source" = "$second"' in script


def test_launcher_smoke_uses_real_config_and_bounded_shutdown():
    script = (REPO / "tests/deployment/mola_launcher_smoke.py").read_text()
    assert 'shutil.which("mola-cli")' in script
    assert "external-map.yaml" not in script
    assert "SWARMDECK_MOLA_SNAPSHOT" in script
    assert "SWARMDECK_MOLA_CHUNKS" in script
    assert "SWARMDECK_MOLA_COMPONENT" in script
    assert "signal.SIGINT" in script
    assert "process.kill()" in script


def test_every_peer_launch_receives_the_mission_gc_settings():
    # peer.launch.py reads these; a setting the compose file does not pass
    # is silently ignored and GC deletes with its defaults.
    expected = {
        "SWARMDECK_MISSION_GC_KEEP_RECENT": "${SWARMDECK_MISSION_GC_KEEP_RECENT:-3}",
        "SWARMDECK_MISSION_GC_DRY_RUN": "${SWARMDECK_MISSION_GC_DRY_RUN:-false}",
    }
    services = [_peer_service("peer_mapping")] + [
        _mapping_service(f"peer{index}") for index in range(4)
    ]
    for service in services:
        env = service["environment"]
        for key, value in expected.items():
            assert env[key] == value
