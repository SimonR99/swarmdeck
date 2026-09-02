import asyncio
import json

import httpx

from agent_cortex.contracts import PlannerRequest
from agent_cortex.models import OllamaPlanner


def test_ollama_planner_uses_schema_and_returns_typed_decision():
    observed = {}

    def handler(request: httpx.Request) -> httpx.Response:
        observed.update(json.loads(request.content))
        decision = {
            "action": "diagnose",
            "target_robots": ["tars_0"],
            "instructions": "Run the consolidated doctor check.",
            "confidence": 0.9,
            "requires_approval": False,
        }
        return httpx.Response(
            200,
            json={"message": {"role": "assistant", "content": json.dumps(decision)}},
        )

    planner = OllamaPlanner(
        base_url="http://ollama.test",
        model="qwen-test",
        transport=httpx.MockTransport(handler),
    )

    async def run():
        return await planner.plan(
            PlannerRequest(
                operator_prompt="diagnose scout",
                system_context="fleet context",
                selected_robot="tars_0",
            )
        )

    decision = asyncio.run(run())
    assert decision.action == "diagnose"
    assert decision.target_robots == ["tars_0"]
    assert observed["model"] == "qwen-test"
    assert observed["stream"] is False
    assert observed["format"]["properties"]["action"]
