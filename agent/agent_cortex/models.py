"""Planner-model adapters.

Planner models only return typed decisions. They do not receive a shell, source
tree, robot network access, or credentials.
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict

import httpx

from .contracts import PlannerDecision, PlannerRequest


class OllamaPlanner:
    name = "ollama"

    def __init__(
        self,
        *,
        base_url: str | None = None,
        model: str | None = None,
        timeout: float | None = None,
        context_length: int | None = None,
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
        self.transport = transport

    def status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "configured": bool(self.model),
            "url": self.base_url,
            "model": self.model or "not configured",
            "context_length": self.context_length,
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
            "format": schema,
            "options": {"temperature": 0, "num_ctx": self.context_length},
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are a routing planner. Return only the requested JSON. "
                        "Do not claim that actions ran. Prefer ask when the target or "
                        "authority is ambiguous. Diagnosis is read-only. Robot motion, "
                        "deployment, repair, and code changes require approval."
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
            return PlannerDecision.model_validate_json(content)
        except Exception as exc:
            raise RuntimeError(f"Ollama returned an invalid planner decision: {exc}") from exc


def get_planner() -> OllamaPlanner | None:
    name = os.environ.get("CORTEX_PLANNER_PROVIDER", "").strip().lower()
    if not name or name in {"none", "off", "disabled"}:
        return None
    if name == "ollama":
        return OllamaPlanner()
    raise ValueError(
        f"Unknown CORTEX_PLANNER_PROVIDER '{name}' (expected 'ollama' or empty)"
    )
