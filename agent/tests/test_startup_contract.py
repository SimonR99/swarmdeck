from pathlib import Path
import re


REPO = Path(__file__).resolve().parents[2]


def test_cortex_has_an_explicit_opt_in_start_target():
    makefile = (REPO / "Makefile").read_text()
    compose = (REPO / "deploy" / "compose" / "docker-compose.yml").read_text()

    assert "up-agent:" in makefile
    assert "$(COMPOSE) --profile agent up --build -d agent" in makefile
    assert 'profiles: ["agent"]' in compose
    assert "$(COMPOSE) up --build -d server ui slam agent" not in makefile


def test_local_ui_has_specific_cortex_proxy_before_generic_api():
    vite = (REPO / "ui" / "vite.config.ts").read_text()

    assert vite.index("'/api/agent'") < vite.index("'/api'")


def test_local_ai_shadow_keeps_agy_live_and_ollama_toolless():
    makefile = (REPO / "Makefile").read_text()
    compose = (REPO / "deploy" / "compose" / "docker-compose.yml").read_text()

    shadow_target = makefile.split("local-ai-shadow:", 1)[1].split(
        "local-ai-down:", 1
    )[0]
    assert "CORTEX_SHADOW_PLANNER=true" in shadow_target
    assert "CORTEX_PLANNER_PROVIDER=ollama" in shadow_target
    assert "CORTEX_PROVIDER=" not in shadow_target
    assert "up -d --no-deps agent" in shadow_target
    assert "The local planner is opt-in shadow traffic and never executes tools" in compose
    assert 'OLLAMA_NO_CLOUD: "true"' in compose


def test_robot_start_diagnose_and_repair_commands_remain_present():
    robot_tool = (REPO / "scripts" / "robot_tool.py").read_text()
    skills = (REPO / "agent" / "agent_cortex" / "skills.py").read_text()

    for command in ('"deploy"', '"doctor"', '"stop"', '"navigate"', '"drive"'):
        assert re.search(rf"add_parser\(\s*{command}", robot_tool)
    for command in ('command="/doctor"', 'command="/deploy"', 'command="/restart"'):
        assert command in skills
