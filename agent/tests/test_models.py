import asyncio
import json

import httpx

from agent_cortex.contracts import PlannerDecision, PlannerRequest
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
        context_length=4096,
        max_tokens=128,
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
    assert observed["think"] is False
    assert observed["options"]["num_ctx"] == 4096
    assert observed["options"]["num_predict"] == 128
    assert observed["format"]["properties"]["action"]
    assert planner.status()["tool_execution"] is False
    assert planner.status()["fleet_authority"] is False


def test_planner_decision_normalizes_robot_mentions():
    decision = PlannerDecision(
        action="mission",
        target_robots=["@spot_0", "spot_0", " @aslan_0 "],
        instructions="Navigate both robots.",
        confidence=0.8,
        requires_approval=True,
    )
    assert decision.target_robots == ["spot_0", "aslan_0"]


def test_planner_policy_owns_approval_and_requires_mutation_target():
    decisions = iter(
        [
            {
                "action": "respond",
                "target_robots": ["tars_0"],
                "instructions": "Answer.",
                "confidence": 0.8,
                "requires_approval": True,
            },
            {
                "action": "repair",
                "target_robots": [],
                "instructions": "Restart it.",
                "confidence": 0.8,
                "requires_approval": False,
            },
        ]
    )

    def handler(_request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            json={"message": {"content": json.dumps(next(decisions))}},
        )

    planner = OllamaPlanner(
        base_url="http://ollama.test",
        model="qwen-test",
        transport=httpx.MockTransport(handler),
    )

    async def run():
        response = await planner.plan(
            PlannerRequest(operator_prompt="hello", system_context="fleet")
        )
        ambiguous = await planner.plan(
            PlannerRequest(operator_prompt="restart the robot", system_context="fleet")
        )
        return response, ambiguous

    response, ambiguous = asyncio.run(run())
    assert response.target_robots == []
    assert response.requires_approval is False
    assert ambiguous.action == "ask"
    assert ambiguous.requires_approval is True
