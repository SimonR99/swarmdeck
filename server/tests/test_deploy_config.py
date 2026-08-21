"""Static contracts for the operator-side robot deployment configuration."""

import re
import http.server
import os
import subprocess
import threading
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
FLEET_ENV = REPO / "deploy/fleet.env"
PROFILE_DIR = REPO / "deploy/robots"
COMPOSE_DIR = REPO / "deploy/compose"

SHARED_KEYS = {
    "BACKEND_HOST",
    "BACKEND_PORT",
    "MEDIA_RTSP_PORT",
    "VIDEO_BITRATE_KBPS",
    "VIDEO_FPS",
    "VIDEO_WIDTH",
    "VIDEO_HEIGHT",
}
LEGACY_KEYS = {"MEDIA_HOST", "ASLAN_REPO", "BOTMAN_REPO", "SPOT_REPO"}

COMPOSE_BY_PROFILE = {
    "aslan": "docker-compose.robot-aslan.yml",
    "botman": "docker-compose.robot-botman.yml",
    "scout": "docker-compose.robot-ros1.yml",
    "spot": "docker-compose.robot-spot.yml",
}


def _assigned_defaults(source: str) -> set[str]:
    return set(re.findall(r'^:\s+"\$\{([A-Z][A-Z0-9_]*):=', source, re.MULTILINE))


def _env_keys(source: str) -> set[str]:
    match = re.search(r'DEPLOY_ENV_KEYS="(.*?)"', source, re.DOTALL)
    assert match is not None
    return set(re.findall(r"\b[A-Z][A-Z0-9_]*\b", match.group(1)))


def test_shared_defaults_live_only_in_fleet_config():
    fleet_defaults = _assigned_defaults(FLEET_ENV.read_text())
    assert fleet_defaults == SHARED_KEYS

    for profile in PROFILE_DIR.glob("*.env"):
        source = profile.read_text()
        assert _assigned_defaults(source).isdisjoint(SHARED_KEYS)
        assert _env_keys(source).isdisjoint(SHARED_KEYS)


def test_profile_override_keys_have_a_robot_side_consumer():
    helper_source = "\n".join(
        (REPO / path).read_text()
        for path in ("scripts/scout-up", "scripts/aslan-build-overlay")
    )
    for name, compose_name in COMPOSE_BY_PROFILE.items():
        profile_source = (PROFILE_DIR / f"{name}.env").read_text()
        consumer_source = (COMPOSE_DIR / compose_name).read_text() + helper_source
        unused = {
            key
            for key in _env_keys(profile_source)
            if re.search(rf"\b{re.escape(key)}\b", consumer_source) is None
        }
        assert not unused, f"{name} exports unused variables: {sorted(unused)}"


def test_deployment_uses_one_repo_key_and_no_separate_media_host():
    deployment_source = "\n".join(
        path.read_text()
        for root in (PROFILE_DIR, COMPOSE_DIR, REPO / "scripts")
        for path in root.iterdir()
        if path.is_file()
    )
    assert not (LEGACY_KEYS & set(re.findall(r"\b[A-Z][A-Z0-9_]*\b", deployment_source)))
    for compose_name in COMPOSE_BY_PROFILE.values():
        compose = (COMPOSE_DIR / compose_name).read_text()
        assert "BACKEND_HOST" in compose
        assert "ROBOT_REPO" in compose


def test_compose_profiles_define_backend_identity_for_readiness_checks():
    for name in COMPOSE_BY_PROFILE:
        source = (PROFILE_DIR / f"{name}.env").read_text()
        assert re.search(r'^:\s+"\$\{DEPLOY_ROBOT_ID:=', source, re.MULTILINE)

    deploy = (REPO / "scripts/deploy").read_text()
    remote = (REPO / "scripts/deploy-remote").read_text()
    verifier = (REPO / "scripts/deploy-verify").read_text()
    assert '"$BACKEND_HOST" "$BACKEND_PORT" "$DEPLOY_ROBOT_ID"' in deploy
    assert '"$repo/scripts/deploy-verify"' in remote
    assert "/api/fleet" in verifier
    assert "State.Health" in verifier


def test_scout_deployment_has_clean_refresh_and_native_opt_out():
    deploy = (REPO / "scripts/deploy").read_text()
    scout = (REPO / "scripts/scout-up").read_text()
    profile = (PROFILE_DIR / "scout.env").read_text()

    assert "--no-native-reset" in deploy
    assert "SCOUT_NATIVE_RESET" in profile
    assert "SCOUT_CONTAINER_RESET" in profile
    assert "reset_native_graph" in scout
    assert '"${compose[@]}" down --remove-orphans' in scout
    assert '"${compose[@]}" up -d --force-recreate --remove-orphans' in scout
    assert 'docker rm -f "$container"' in scout
    assert "--project-name swarmdeck" in (REPO / "scripts/deploy-remote").read_text()


def test_hardware_camera_policy_is_h264_640x480_without_jpeg_fallback():
    media_scripts = [
        (REPO / "adapters/media/ros1_rtsp.py").read_text(),
        (REPO / "adapters/media/ros2_rtsp.py").read_text(),
    ]
    for source in media_scripts:
        assert "videoscale" in source
        assert "width={width},height={height}" in source
        assert "x264enc tune=zerolatency speed-preset=ultrafast" in source

    for compose_name in COMPOSE_BY_PROFILE.values():
        source = (COMPOSE_DIR / compose_name).read_text()
        assert '"${VIDEO_WIDTH:-640}"' in source
        assert '"${VIDEO_HEIGHT:-480}"' in source

    for path in (
        REPO / "adapters/adapter_ros1/adapter_ros1.py",
        REPO / "adapters/adapter_ros2/adapter_ros2.py",
        REPO / "adapters/adapter_sim/adapter_sim.py",
        REPO / "ui/src/lib/components/video/CameraPanel.svelte",
    ):
        source = path.read_text()
        assert "/api/adapter/camera" not in source
        assert "/api/camera/" not in source


class _FleetHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802 - stdlib handler API
        if self.path != "/api/fleet":
            self.send_response(404)
            self.end_headers()
            return
        body = b'{"robots":[{"robot_id":"botman_0","online":true}]}'
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        return


def test_deploy_verify_checks_live_backend_registration():
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _FleetHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        result = subprocess.run(
            [
                str(REPO / "scripts/deploy-verify"),
                str(REPO),
                "",
                "127.0.0.1",
                str(server.server_port),
                "botman_0",
                "5",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    finally:
        server.shutdown()
        thread.join(timeout=2)

    assert result.returncode == 0, result.stderr
    assert "registered and reporting live state" in result.stdout


def test_deploy_verify_accepts_running_healthy_containers(tmp_path):
    fake_docker = tmp_path / "docker"
    fake_docker.write_text(
        """#!/usr/bin/env bash
set -eu
[[ $1 == inspect && $2 == -f ]] || exit 2
case $3 in
  '{{.State.Status}}') echo running ;;
  '{{if .State.Health}}{{.State.Health.Status}}{{end}}') echo healthy ;;
  *) echo "$FAKE_REPO" ;;
esac
"""
    )
    fake_docker.chmod(0o755)

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _FleetHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        environment = os.environ.copy()
        environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
        environment["FAKE_REPO"] = str(REPO)
        result = subprocess.run(
            [
                str(REPO / "scripts/deploy-verify"),
                str(REPO),
                "adapter media",
                "127.0.0.1",
                str(server.server_port),
                "botman_0",
                "5",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
            timeout=10,
        )
    finally:
        server.shutdown()
        thread.join(timeout=2)

    assert result.returncode == 0, result.stderr
