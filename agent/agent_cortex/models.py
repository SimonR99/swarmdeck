"""Planner-model adapters.

Planner models only return typed decisions. They do not receive a shell, source
tree, robot network access, or credentials.
"""

from __future__ import annotations

import json
import os
import re
from typing import Any, Dict

import httpx

from .contracts import PlannerDecision, PlannerRequest


_ROBOT_ACTIONS = {"diagnose", "repair", "mission"}
_EXPLICIT_FLEET = re.compile(
    r"\b(all|fleet-wide|entire fleet|every robot|all robots)\b", re.I
)


def _apply_planner_policy(
    request: PlannerRequest, decision: PlannerDecision
) -> PlannerDecision:
    """Make authority and obvious target rules deterministic.

    The model classifies intent; it does not decide whether its own proposal is
    approved. Explicit UI selection/mentions also outrank hallucinated targets.
    """
    action = decision.action
    mentions = list(
        dict.fromkeys(re.findall(r"@([A-Za-z0-9_-]+)", request.operator_prompt))
    )
    targets = decision.target_robots
    if action in _ROBOT_ACTIONS:
        if mentions:
            targets = mentions
        elif request.selected_robot:
            targets = [request.selected_robot]
    else:
        targets = []

    if action in {"repair", "mission"} and not targets:
        if _EXPLICIT_FLEET.search(request.operator_prompt):
            targets = ["all"]
        else:
            action = "ask"
            targets = []

    requires_approval = action not in {"respond", "diagnose"}
    return decision.model_copy(
        update={
            "action": action,
            "target_robots": targets,
            "requires_approval": requires_approval,
        }
    )


class OllamaPlanner:
    name = "ollama"

    def __init__(
        self,
        *,
        base_url: str | None = None,
        model: str | None = None,
        timeout: float | None = None,
        context_length: int | None = None,
        max_tokens: int | None = None,
        transport: httpx.AsyncBaseTransport | None = None,
    ) -> None:
        self.base_url = (
            base_url or os.environ.get("CORTEX_OLLAMA_URL", "http://ollama:11434")
        ).rstrip("/")
        self.model = model or os.environ.get("CORTEX_PLANNER_MODEL", "")
        self.timeout = timeout or float(os.environ.get("CORTEX_PLANNER_TIMEOUT", "45"))
        self.context_length = context_length or int(
            os.environ.get("CORTEX_PLANNER_CONTEXT_LENGTH", "8192")
        )
        if not 1024 <= self.context_length <= 131072:
            raise ValueError("CORTEX_PLANNER_CONTEXT_LENGTH must be between 1024 and 131072")
        self.max_tokens = max_tokens or int(
            os.environ.get("CORTEX_PLANNER_MAX_TOKENS", "256")
        )
        if not 64 <= self.max_tokens <= 4096:
            raise ValueError("CORTEX_PLANNER_MAX_TOKENS must be between 64 and 4096")
        self.transport = transport

    def status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "configured": bool(self.model),
            "url": self.base_url,
            "model": self.model or "not configured",
            "context_length": self.context_length,
            "max_tokens": self.max_tokens,
            "role": "shadow planner",
            "tool_execution": False,
            "fleet_authority": False,
        }

    async def plan(self, request: PlannerRequest) -> PlannerDecision:
        if not self.model:
            raise RuntimeError("CORTEX_PLANNER_MODEL is required for the Ollama planner")
        schema = PlannerDecision.model_json_schema()
        payload = {
            "model": self.model,
            "stream": False,
            # Qwen reasoning can otherwise spend thousands of tokens before a
            # five-field routing answer and hit the request timeout.
            "think": False,
            "format": schema,
            "options": {
                "temperature": 0,
                "num_ctx": self.context_length,
                "num_predict": self.max_tokens,
            },
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "Classify the operator's requested next step and return only the "
                        "requested JSON. Use respond for text only, diagnose for a "
                        "read-only health check, repair for restart/deploy/fix, "
                        "code_change for source edits, mission for robot motion or a "
                        "multi-robot plan, and ask only when intent or a required robot "
                        "target is missing. Never use ask merely because approval has not "
                        "yet been granted: requires_approval records that gate. Set it "
                        "true for repair, code_change, mission, and ask; false for respond "
                        "and diagnose. target_robots must contain only explicitly involved "
                        "robot IDs without @; use [] for respond, code_change, and ask. "
                        "The label fleet-wide means no robot is selected, not every robot. "
                        "Do not claim that any action ran."
                    ),
                },
                {
                    "role": "user",
                    "content": (
                        f"Selected robot: {request.selected_robot or 'fleet-wide'}\n"
                        f"Operator request: {request.operator_prompt}\n\n"
                        f"Context:\n{request.system_context}"
                    ),
                },
            ],
        }
        async with httpx.AsyncClient(
            timeout=self.timeout, transport=self.transport
        ) as client:
            response = await client.post(f"{self.base_url}/api/chat", json=payload)
            response.raise_for_status()
            body = response.json()
        content = (body.get("message") or {}).get("content")
        if not isinstance(content, str):
            raise RuntimeError("Ollama returned no planner message content")
        try:
            decision = PlannerDecision.model_validate_json(content)
        except Exception as exc:
            raise RuntimeError(f"Ollama returned an invalid planner decision: {exc}") from exc
        return _apply_planner_policy(request, decision)


def get_planner() -> OllamaPlanner | None:
    name = os.environ.get("CORTEX_PLANNER_PROVIDER", "").strip().lower()
    if not name or name in {"none", "off", "disabled"}:
        return None
    if name == "ollama":
        return OllamaPlanner()
    raise ValueError(
        f"Unknown CORTEX_PLANNER_PROVIDER '{name}' (expected 'ollama' or empty)"
    )
