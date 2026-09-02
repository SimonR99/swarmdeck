"""Transparent Cortex supervisor with durable shadow state.

In the initial rollout the supervisor never changes routing: every operator
request still runs through the selected provider.  Observe mode records enough
state to evaluate and later resume work without changing the UI or robot tools.
"""

from __future__ import annotations

import asyncio
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, AsyncGenerator, Dict, Optional

from .contracts import PlannerRequest
from .fleet_tools import RobotToolFleetTools
from .models import get_planner
from .providers import AgentProvider, ProviderRequest
from .runtime import ProviderRuntime
from .store import CortexStore, SCHEMA_VERSION
from .workers import get_coding_worker


_SENSITIVE_KEY = re.compile(
    r"(^|_)(authorization|cookie|password|passwd|secret|token|private_key)($|_)",
    re.IGNORECASE,
)
_BEARER_VALUE = re.compile(r"(?i)bearer\s+[A-Za-z0-9._~+/=-]+")
_MAX_STORED_STRING = 65536


def _safe_payload(value: Any, key: str = "") -> Any:
    if key and _SENSITIVE_KEY.search(key):
        return "[redacted]"
    if isinstance(value, dict):
        return {str(k): _safe_payload(v, str(k)) for k, v in value.items()}
    if isinstance(value, list):
        return [_safe_payload(item) for item in value]
    if isinstance(value, str):
        redacted = _BEARER_VALUE.sub("Bearer [redacted]", value)
        if len(redacted) > _MAX_STORED_STRING:
            return redacted[:_MAX_STORED_STRING] + "\n[truncated by Cortex]"
        return redacted
    return value


def classify_job(prompt: str) -> str:
    text = prompt.strip().lower()
    command = text.split(maxsplit=1)[0] if text else ""
    if command in {"/doctor", "/status"} or any(
        word in text for word in ("diagnose", "diagnostic", "why is", "health")
    ):
        return "diagnosis"
    if command in {"/deploy", "/restart"} or any(
        word in text for word in ("repair", "fix", "restart", "deploy", "start the robot")
    ):
        return "repair"
    if command == "/code" or any(
        word in text for word in ("edit the code", "modify the code", "refactor", "implement")
    ):
        return "code_change"
    if command in {"/drive", "/nav", "/stop"}:
        return "robot_action"
    return "conversation"


@dataclass(frozen=True)
class SupervisorRequest:
    provider_request: ProviderRequest
    operator_prompt: str
    provider_name: str
    selected_robot: Optional[str] = None
    planner_context: Optional[str] = None


class CortexSupervisor:
    def __init__(
        self,
        *,
        mode: str = "observe",
        store_path: str | Path,
    ) -> None:
        normalized_mode = mode.strip().lower()
        if normalized_mode not in {"legacy", "observe"}:
            raise ValueError("CORTEX_SUPERVISOR_MODE must be 'legacy' or 'observe'")
        self.mode = normalized_mode
        self.store_path = Path(store_path)
        self.store: CortexStore | None = None
        self.last_store_error: str | None = None
        self.recovered_jobs = 0
        self.shadow_planner = None
        self.last_planner_error: str | None = None
        self._background_tasks: set[asyncio.Task[None]] = set()
        self.coding_worker = None
        self.last_worker_error: str | None = None
        self.fleet_tools = RobotToolFleetTools()

        if self.mode == "observe":
            try:
                self.store = CortexStore(self.store_path)
                self.recovered_jobs = self.store.interrupt_running_jobs()
            except Exception as exc:
                self.last_store_error = str(exc)

        if os.environ.get("CORTEX_SHADOW_PLANNER", "false").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }:
            try:
                self.shadow_planner = get_planner()
            except Exception as exc:
                self.last_planner_error = str(exc)

        try:
            self.coding_worker = get_coding_worker()
        except Exception as exc:
            self.last_worker_error = str(exc)

    def status(self) -> Dict[str, Any]:
        planner_status = self.shadow_planner.status() if self.shadow_planner else None
        worker_status = self.coding_worker.status() if self.coding_worker else None
        return {
            "mode": self.mode,
            "routing": "provider_passthrough",
            "state_enabled": self.store is not None,
            "state_path": str(self.store_path),
            "schema_version": SCHEMA_VERSION,
            "state_error": self.last_store_error,
            "recovered_jobs": self.recovered_jobs,
            "shadow_planner": planner_status,
            "shadow_planner_error": self.last_planner_error,
            "shadow_plans_inflight": len(self._background_tasks),
            "coding_worker": worker_status,
            "coding_worker_error": self.last_worker_error,
            "fleet_tools": self.fleet_tools.status(),
        }

    def _store_call(self, operation: str, *args: Any, **kwargs: Any) -> Any:
        if self.store is None:
            return None
        try:
            return getattr(self.store, operation)(*args, **kwargs)
        except Exception as exc:
            # Observability must never become an outage in the compatibility phase.
            self.last_store_error = str(exc)
            return None

    async def _run_shadow_plan(
        self, request: SupervisorRequest, job_id: Optional[str]
    ) -> None:
        if self.shadow_planner is None or job_id is None:
            return
        try:
            decision = await self.shadow_planner.plan(
                PlannerRequest(
                    operator_prompt=request.operator_prompt,
                    system_context=(
                        request.planner_context or request.provider_request.prompt
                    ),
                    selected_robot=request.selected_robot,
                )
            )
            self._store_call(
                "append_event",
                job_id,
                "supervisor.shadow_plan",
                decision.model_dump(),
            )
        except Exception as exc:
            self.last_planner_error = str(exc)
            self._store_call(
                "append_event",
                job_id,
                "supervisor.shadow_plan_error",
                {"error": str(exc)},
            )

    def _start_shadow_plan(
        self, request: SupervisorRequest, job_id: Optional[str]
    ) -> None:
        """Run evaluation independently of the operator-facing provider stream.

        A local model may be slower than a short AGY response.  Keeping a strong
        reference here lets its decision reach the audit store after the UI has
        already received AGY's terminal event.  The task still has no execution
        authority and is cancelled normally when the server's event loop stops.
        """
        if self.shadow_planner is None or job_id is None:
            return
        task = asyncio.create_task(self._run_shadow_plan(request, job_id))
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.discard)

    async def run(
        self, provider: AgentProvider, request: SupervisorRequest
    ) -> AsyncGenerator[Dict[str, Any], None]:
        job_id = self._store_call(
            "create_job",
            kind=classify_job(request.operator_prompt),
            provider=request.provider_name,
            request_text=_safe_payload(request.operator_prompt),
            conversation_id=request.provider_request.conversation_id,
            selected_robot=request.selected_robot,
            metadata={"routing": "provider_passthrough"},
        )
        self._start_shadow_plan(request, job_id)
        terminal_type: str | None = None
        pending_token_text: list[str] = []
        pending_token_size = 0

        def flush_tokens() -> None:
            nonlocal pending_token_size
            if not pending_token_text:
                return
            self._store_call(
                "append_event",
                job_id,
                "provider.token",
                {"type": "token", "delta": "".join(pending_token_text)},
            )
            pending_token_text.clear()
            pending_token_size = 0

        try:
            async for event in ProviderRuntime(provider).run(request.provider_request):
                event_type = event.get("type")
                if event_type == "error":
                    terminal_type = "error"
                elif event_type == "done" and terminal_type is None:
                    terminal_type = "done"
                if event_type == "token" and isinstance(event.get("delta"), str):
                    pending_token_text.append(event["delta"])
                    pending_token_size += len(event["delta"])
                    if pending_token_size >= 4096:
                        flush_tokens()
                else:
                    flush_tokens()
                    self._store_call(
                        "append_event",
                        job_id,
                        f"provider.{event_type or 'unknown'}",
                        _safe_payload(event),
                    )
                yield event
        except asyncio.CancelledError:
            self._store_call("set_phase", job_id, "cancelled")
            raise
        except Exception as exc:
            self._store_call("set_phase", job_id, "failed", error_text=str(exc))
            raise
        else:
            flush_tokens()
            if terminal_type == "done":
                self._store_call("set_phase", job_id, "completed")
            else:
                self._store_call(
                    "set_phase",
                    job_id,
                    "failed",
                    error_text="provider stream ended with an error",
                )


def build_supervisor(history_dir: Path) -> CortexSupervisor:
    mode = os.environ.get("CORTEX_SUPERVISOR_MODE", "observe")
    state_path = Path(
        os.environ.get("CORTEX_STATE_DB", str(history_dir.parent / "cortex" / "state.db"))
    )
    try:
        return CortexSupervisor(mode=mode, store_path=state_path)
    except ValueError as exc:
        # A bad rollout flag must not take the existing operator path offline.
        supervisor = CortexSupervisor(mode="legacy", store_path=state_path)
        supervisor.last_store_error = str(exc)
        return supervisor
