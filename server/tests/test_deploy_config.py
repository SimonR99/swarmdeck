"""Static contracts for the operator-side robot deployment configuration."""

import re
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
