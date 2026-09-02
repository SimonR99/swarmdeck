import asyncio
import json
from pathlib import Path

from agent_cortex.providers import (
    AgyProvider,
    NdjsonBridgeProvider,
    OpenCodeProvider,
    ProviderRequest,
)


REPO = Path(__file__).resolve().parents[2]


def test_agy_events_are_normalized(monkeypatch):
    monkeypatch.setattr("agent_cortex.providers.find_agy_binary", lambda: "/tmp/agy")
    provider = AgyProvider()

    assert provider.parse_event(
        json.dumps({"event": "init", "conversation_id": "conversation-1"})
    ) == {"type": "init", "conversation_id": "conversation-1"}
    assert provider.parse_event(
        json.dumps(
            {
                "event": "step_update",
                "step_update": {
                    "step_type": "tool",
                    "state": "ACTIVE",
                    "tool_name": "run_command",
                    "tool_info": {"parameters": {"CommandLine": "robot_tool.py doctor"}},
                },
            }
        )
    ) == {
        "type": "tool_call",
        "tool": "run_command",
        "params": {"CommandLine": "robot_tool.py doctor"},
    }
    assert provider.parse_event(
        json.dumps(
            {
                "event": "result",
                "result": {"status": "ERROR", "error": "provider stopped"},
            }
        )
    ) == {
        "type": "done",
        "response": "⚠️ provider stopped",
        "status": "ERROR",
        "usage": {},
    }


def test_ndjson_bridge_accepts_only_cortex_events(monkeypatch):
    monkeypatch.setenv("CORTEX_PROVIDER_COMMAND", '["/bin/echo"]')
    provider = NdjsonBridgeProvider()

    assert provider.status()["available"] is True
    assert provider.parse_event('{"type":"token","delta":"hello"}') == {
        "type": "token",
        "delta": "hello",
    }
    assert provider.parse_event('{"event":"agy-specific"}') is None
    assert provider.parse_event("not json") is None


def test_ndjson_bridge_receives_one_normalized_request(tmp_path, monkeypatch):
    output = tmp_path / "request.json"
    bridge = tmp_path / "bridge.py"
    bridge.write_text(
        "#!/usr/bin/env python3\n"
        "import pathlib, sys\n"
        f"pathlib.Path({str(output)!r}).write_text(sys.stdin.readline())\n"
        "print('{\"type\":\"done\",\"response\":\"ok\"}', flush=True)\n"
    )
    bridge.chmod(0o755)
    monkeypatch.setenv("CORTEX_PROVIDER_COMMAND", json.dumps([str(bridge)]))
    provider = NdjsonBridgeProvider()

    async def run_bridge():
        process = await provider.start(
            ProviderRequest(prompt="hello", workspace=str(tmp_path), conversation_id="c-1")
        )
        assert process.stdout is not None
        event = provider.parse_event((await process.stdout.readline()).decode())
        assert event == {"type": "done", "response": "ok"}
        assert await process.wait() == 0

    asyncio.run(run_bridge())
    request = json.loads(output.read_text())
    assert request == {
        "protocol": "cortex-ndjson-v1",
        "prompt": "hello",
        "workspace": str(tmp_path),
        "conversation_id": "c-1",
    }


def test_opencode_events_are_normalized(monkeypatch):
    monkeypatch.setenv("CORTEX_OPENCODE_COMMAND", "/bin/echo")
    provider = OpenCodeProvider()

    assert provider.status()["available"] is True
    assert provider.parse_event(
        json.dumps(
            {
                "type": "step_start",
                "sessionID": "session-1",
                "part": {"type": "step-start"},
            }
        )
    ) == {"type": "init", "conversation_id": "session-1"}
    assert provider.parse_event(
        json.dumps(
            {
                "type": "text",
                "sessionID": "session-1",
                "part": {"type": "text", "text": "hello"},
            }
        )
    ) == {"type": "token", "delta": "hello"}
    assert provider.finish_event(0, "") == {
        "type": "done",
        "response": "hello",
        "status": "SUCCESS",
        "usage": {},
    }


def test_opencode_invocation_keeps_workspace_model_and_session(monkeypatch, tmp_path):
    captured = {}

    class Process:
        pass

    async def fake_subprocess(*args, **kwargs):
        captured["args"] = args
        captured["kwargs"] = kwargs
        return Process()

    monkeypatch.setenv("CORTEX_OPENCODE_COMMAND", "/bin/echo")
    monkeypatch.setenv("CORTEX_MODEL", "ollama/qwen-test")
    monkeypatch.setenv("CORTEX_OPENCODE_AGENT", "build")
    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_subprocess)
    provider = OpenCodeProvider()

    asyncio.run(
        provider.start(
            ProviderRequest(
                prompt="repair request",
                workspace=str(tmp_path),
                conversation_id="session-1",
            )
        )
    )

    assert captured["args"] == (
        "/bin/echo",
        "run",
        "--format",
        "json",
        "--auto",
        "--dir",
        str(tmp_path),
        "--model",
        "ollama/qwen-test",
        "--agent",
        "build",
        "--session",
        "session-1",
        "repair request",
    )
    assert captured["kwargs"]["cwd"] == str(tmp_path)


def test_operator_prompt_requires_evidence_based_diagnostics():
    prompt = (REPO / "agent" / "agent_cortex" / "server.py").read_text()

    assert "robot_tool.py doctor <robot>" in prompt
    assert "--services" in prompt
    assert "Never claim all services are healthy from telemetry alone" in prompt
    assert "Never claim dashboard video is live" in prompt
    assert "Use profile `scout` for tars_0" in prompt
