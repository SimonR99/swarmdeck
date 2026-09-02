"""Code-worker adapters kept separate from fleet authority."""

from __future__ import annotations

import os
from typing import Any, AsyncGenerator, Dict

from .contracts import ChangeRequest
from .providers import AgyProvider, AgentProvider, OpenCodeProvider, ProviderRequest
from .runtime import ProviderRuntime


class RuntimeCodingWorker:
    """Expose an existing tool-using runtime through the CodingWorker contract."""

    def __init__(self, provider: AgentProvider) -> None:
        self.provider = provider
        self.name = provider.name

    def status(self) -> Dict[str, Any]:
        return {
            **self.provider.status(),
            "role": "coding_worker",
            "active": False,
        }

    async def run(
        self, request: ChangeRequest
    ) -> AsyncGenerator[Dict[str, Any], None]:
        provider_request = ProviderRequest(
            prompt=request.instruction,
            workspace=request.workspace,
            conversation_id=request.session_id,
        )
        async for event in ProviderRuntime(self.provider).run(provider_request):
            yield event


def get_coding_worker() -> RuntimeCodingWorker:
    """Build the configured future repair worker; no routing occurs here."""
    name = os.environ.get("CORTEX_CODING_WORKER", "agy").strip().lower()
    if name == "agy":
        return RuntimeCodingWorker(AgyProvider())
    if name in {"opencode", "open-code"}:
        return RuntimeCodingWorker(OpenCodeProvider())
    raise ValueError(
        f"Unknown CORTEX_CODING_WORKER '{name}' (expected 'agy' or 'opencode')"
    )
