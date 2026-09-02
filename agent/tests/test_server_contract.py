import asyncio
import json
import types
import sys

# The production Cortex image includes python-multipart. The shared backend
# test venv does not, and these contract tests do not exercise upload parsing.
if "multipart" not in sys.modules:
    multipart = types.ModuleType("multipart")
    multipart.__version__ = "0.0.20"
    multipart_parser = types.ModuleType("multipart.multipart")
    multipart_parser.parse_options_header = lambda value: (value, {})
    sys.modules["multipart"] = multipart
    sys.modules["multipart.multipart"] = multipart_parser

from agent_cortex import server
from agent_cortex.supervisor import build_supervisor


class FakeProvider:
    name = "fake"

    def status(self):
        return {"name": self.name, "available": True, "model": "test-model"}

    async def start(self, _request):
        events = [
            {"type": "init", "conversation_id": "provider-conversation"},
            {"type": "token", "delta": "checking"},
            {"type": "tool_call", "tool": "doctor", "params": {"robot": "tars_0"}},
            {"type": "tool_output", "tool": "doctor", "output": "healthy"},
            {"type": "done", "response": "checking", "status": "SUCCESS", "usage": {}},
        ]

        class Reader:
            def __init__(self, lines):
                self.lines = iter(lines)

            async def readline(self):
                return next(self.lines, b"")

            async def read(self, _size=-1):
                return b""

        class Process:
            stdout = Reader([(json.dumps(event) + "\n").encode() for event in events])
            stderr = Reader([])

            async def wait(self):
                return 0

            def kill(self):
                pass

        return Process()

    def parse_event(self, line):
        return json.loads(line)


def _sse_events(response):
    return [
        json.loads(line.removeprefix("data: "))
        for line in response.splitlines()
        if line.startswith("data: ")
    ]


def test_chat_keeps_request_and_sse_contract(tmp_path, monkeypatch):
    monkeypatch.setattr(server, "HISTORY_DIR", tmp_path / "history")
    monkeypatch.setattr(server, "get_provider", lambda: FakeProvider())
    monkeypatch.setattr(
        server,
        "query_fleet",
        lambda _url: [
            {
                "robot_id": "tars_0",
                "robot_type": "scout",
                "online": True,
                "battery": 0.5,
                "pose": {"x": 1, "y": 2},
            }
        ],
    )
    monkeypatch.setenv("CORTEX_SUPERVISOR_MODE", "observe")
    monkeypatch.setenv("CORTEX_SHADOW_PLANNER", "false")
    monkeypatch.setattr(server, "SUPERVISOR", build_supervisor(server.HISTORY_DIR))

    request = server.ChatRequest.model_validate(
        {
            "prompt": "/doctor @tars_0",
            "conversation_id": "provider-conversation",
            "selected_robot": "spot_0",
            "attachments": [{"path": "/tmp/frame.jpg"}],
        },
    )

    async def run():
        response = await server.post_chat(request)
        chunks = [chunk async for chunk in response.body_iterator]
        text = "".join(
            chunk.decode() if isinstance(chunk, bytes) else chunk for chunk in chunks
        )
        return response, text

    response, body = asyncio.run(run())

    assert response.status_code == 200
    assert response.media_type == "text/event-stream"
    assert _sse_events(body) == [
        {"type": "init", "conversation_id": "provider-conversation"},
        {"type": "token", "delta": "checking"},
        {"type": "tool_call", "tool": "doctor", "params": {"robot": "tars_0"}},
        {"type": "tool_output", "tool": "doctor", "output": "healthy"},
        {"type": "done", "response": "checking", "status": "SUCCESS", "usage": {}},
    ]


def test_chat_rejects_empty_prompt(monkeypatch):
    response = asyncio.run(server.post_chat(server.ChatRequest(prompt="   ")))
    assert response.status_code == 400
    assert json.loads(response.body) == {"error": "Prompt cannot be empty"}


def test_status_keeps_legacy_fields_and_adds_supervisor(tmp_path, monkeypatch):
    monkeypatch.setattr(server, "HISTORY_DIR", tmp_path / "history")
    monkeypatch.setattr(server, "get_provider", lambda: FakeProvider())
    monkeypatch.setattr(server, "find_agy_binary", lambda: "/usr/local/bin/agy")
    monkeypatch.setattr(server, "query_fleet", lambda _url: [])
    monkeypatch.setenv("CORTEX_SUPERVISOR_MODE", "observe")
    monkeypatch.setenv("CORTEX_SHADOW_PLANNER", "false")
    monkeypatch.setattr(server, "SUPERVISOR", build_supervisor(server.HISTORY_DIR))

    body = asyncio.run(server.get_status())

    for field in (
        "status",
        "name",
        "version",
        "provider",
        "provider_error",
        "agy_available",
        "agy_binary",
        "workspace",
        "server_url",
        "fleet_count",
        "model",
    ):
        assert field in body
    assert body["supervisor"]["routing"] == "provider_passthrough"
