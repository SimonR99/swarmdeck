from pathlib import Path
import re


REPO = Path(__file__).resolve().parents[2]


def test_operator_start_targets_include_cortex_agent():
    makefile = (REPO / "Makefile").read_text()

    assert "$(COMPOSE) up --build -d server ui slam agent" in makefile
    assert (
        "docker compose $(ZENOH_COMPOSE) up --build -d server ui agent "
        "mediamtx zenoh-router slam"
    ) in makefile
    assert "docker compose $(ZENOH_COMPOSE) build server ui agent" in makefile


def test_local_ui_has_specific_cortex_proxy_before_generic_api():
    vite = (REPO / "ui" / "vite.config.ts").read_text()

    assert vite.index("'/api/agent'") < vite.index("'/api'")


def test_robot_start_diagnose_and_repair_commands_remain_present():
    robot_tool = (REPO / "scripts" / "robot_tool.py").read_text()
    skills = (REPO / "agent" / "agent_cortex" / "skills.py").read_text()

    for command in ('"deploy"', '"doctor"', '"stop"', '"navigate"', '"drive"'):
        assert re.search(rf"add_parser\(\s*{command}", robot_tool)
    for command in ('command="/doctor"', 'command="/deploy"', 'command="/restart"'):
        assert command in skills
