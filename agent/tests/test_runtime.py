import asyncio
import json
import sys

from agent_cortex.providers import ProviderRequest
from agent_cortex.runtime import ProviderRuntime


class JsonProcessProvider:
    name = "test"

    def __init__(self, script: str):
        self.script = script

    def status(self):
        return {"name": self.name, "available": True}

    async def start(self, _request):
        return await asyncio.create_subprocess_exec(
            sys.executable,
            "-c",
            self.script,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

    def parse_event(self, line):
        return json.loads(line)


def test_runtime_preserves_provider_event_order():
    provider = JsonProcessProvider(
        "import json; "
        "print(json.dumps({'type':'init','conversation_id':'c1'})); "
        "print(json.dumps({'type':'token','delta':'ok'})); "
        "print(json.dumps({'type':'done','response':'ok'}))"
    )

    async def collect():
        request = ProviderRequest(prompt="hello", workspace=".")
        return [event async for event in ProviderRuntime(provider).run(request)]

    assert asyncio.run(collect()) == [
        {"type": "init", "conversation_id": "c1"},
        {"type": "token", "delta": "ok"},
        {"type": "done", "response": "ok"},
    ]


def test_runtime_reports_stderr_when_provider_has_no_terminal_event():
    provider = JsonProcessProvider(
        "import sys; print('provider failed', file=sys.stderr); raise SystemExit(7)"
    )

    async def collect():
        request = ProviderRequest(prompt="hello", workspace=".")
        return [event async for event in ProviderRuntime(provider).run(request)]

    assert asyncio.run(collect()) == [
        {"type": "error", "error": "provider failed"}
    ]
