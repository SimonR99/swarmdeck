"""Agent-runtime adapters for Cortex.

Cortex owns the UI/API contract and fleet context.  A provider owns the
tool-using agent loop and translates its output into Cortex's small SSE event
schema.  Keeping that seam here prevents the HTTP service from depending on a
particular model vendor or CLI event format.
"""

from __future__ import annotations

import asyncio
import json
import os
import shlex
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Protocol

from .events import NORMALIZED_EVENT_TYPES


@dataclass(frozen=True)
class ProviderRequest:
    prompt: str
    workspace: str
    conversation_id: Optional[str] = None


class AgentProvider(Protocol):
    """Runtime boundary required by the Cortex HTTP service."""

    name: str

    def status(self) -> Dict[str, Any]: ...

    async def start(self, request: ProviderRequest) -> asyncio.subprocess.Process: ...

    def parse_event(self, line: str) -> Optional[Dict[str, Any]]: ...


def find_agy_binary() -> Optional[str]:
    candidates = [
        os.environ.get("ANTIGRAVITY_AGENTAPI_EXE"),
        os.environ.get("AGY_BIN"),
        "/usr/local/bin/agy",
        os.path.expanduser("~/.local/bin/agy"),
        shutil.which("agy"),
    ]
    for candidate in candidates:
        if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


class AgyProvider:
    """Antigravity CLI adapter."""

    name = "agy"

    def __init__(self) -> None:
        self.binary = find_agy_binary()
        self.model = os.environ.get("CORTEX_MODEL") or None
        self.effort = os.environ.get("CORTEX_REASONING_EFFORT") or None

    def status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "available": self.binary is not None,
            "binary": self.binary,
            # AGY chooses its account/default model when CORTEX_MODEL is unset.
            # Reporting a guessed model here made production diagnostics lie.
            "model": self.model or "provider default",
            "effort": self.effort or "provider default",
        }

    async def start(self, request: ProviderRequest) -> asyncio.subprocess.Process:
        if not self.binary:
            raise RuntimeError("Antigravity CLI ('agy') binary was not found")

        command = [
            self.binary,
            "--add-dir",
            request.workspace,
            "-p",
            request.prompt,
            "--output-format",
            "stream-json",
            "--dangerously-skip-permissions",
        ]
        if self.model:
            command.extend(["--model", self.model])
        if self.effort:
            command.extend(["--effort", self.effort])
        if request.conversation_id:
            command.extend(["--conversation", request.conversation_id])

        return await asyncio.create_subprocess_exec(
            *command,
            cwd=request.workspace,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

    def parse_event(self, line: str) -> Optional[Dict[str, Any]]:
        try:
            payload = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            return None

        event_type = payload.get("event")
        if event_type == "init":
            return {
                "type": "init",
                "conversation_id": payload.get("conversation_id"),
            }

        if event_type == "step_update":
            update = payload.get("step_update") or {}
            step_type = update.get("step_type")
            state = update.get("state")
            if step_type == "agent_response" and update.get("text_delta"):
                return {"type": "token", "delta": update["text_delta"]}
            if step_type == "tool":
                tool_info = update.get("tool_info") or {}
                if state == "ACTIVE":
                    return {
                        "type": "tool_call",
                        "tool": update.get("tool_name"),
                        "params": tool_info.get("parameters") or {},
                    }
                if state == "DONE":
                    return {
                        "type": "tool_output",
                        "tool": update.get("tool_name"),
                        "output": tool_info.get("output", ""),
                    }
            return None

        if event_type == "result":
            result = payload.get("result") or {}
            status = result.get("status", "SUCCESS")
            response = result.get("response", "")
            if status == "ERROR" and not response:
                response = f"⚠️ {result.get('error') or 'Agent execution failed'}"
            return {
                "type": "done",
                "response": response,
                "status": status,
                "usage": result.get("usage") or {},
            }
        return None


class NdjsonBridgeProvider:
    """Adapter for any external agent runtime implementing Cortex NDJSON.

    The configured process receives one request object on stdin and emits one
    normalized Cortex event per stdout line.  The bridge can wrap another CLI,
    a local model, or a hosted-model agent; it remains responsible for its own
    tool loop and conversation storage.
    """

    name = "ndjson"

    def __init__(self) -> None:
        self.command = self._read_command()
        self.model = os.environ.get("CORTEX_MODEL") or "bridge-defined"

    @staticmethod
    def _read_command() -> list[str]:
        raw = os.environ.get("CORTEX_PROVIDER_COMMAND", "").strip()
        if not raw:
            return []
        try:
            decoded = json.loads(raw)
        except json.JSONDecodeError:
            decoded = None
        if isinstance(decoded, list) and all(isinstance(item, str) for item in decoded):
            return decoded
        return shlex.split(raw)

    def status(self) -> Dict[str, Any]:
        executable = self.command[0] if self.command else None
        available = bool(
            executable
            and (
                (Path(executable).is_file() and os.access(executable, os.X_OK))
                or shutil.which(executable)
            )
        )
        return {
            "name": self.name,
            "available": available,
            "binary": executable,
            "model": self.model,
            "protocol": "cortex-ndjson-v1",
        }

    async def start(self, request: ProviderRequest) -> asyncio.subprocess.Process:
        state = self.status()
        if not state["available"]:
            raise RuntimeError(
                "CORTEX_PROVIDER_COMMAND must name an executable NDJSON bridge"
            )
        process = await asyncio.create_subprocess_exec(
            *self.command,
            cwd=request.workspace,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        assert process.stdin is not None
        request_payload = {
            "protocol": "cortex-ndjson-v1",
            "prompt": request.prompt,
            "workspace": request.workspace,
            "conversation_id": request.conversation_id,
        }
        process.stdin.write((json.dumps(request_payload) + "\n").encode())
        await process.stdin.drain()
        process.stdin.close()
        return process

    def parse_event(self, line: str) -> Optional[Dict[str, Any]]:
        try:
            payload = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict) or payload.get("type") not in NORMALIZED_EVENT_TYPES:
            return None
        return payload


class OpenCodeProvider:
    """OpenCode CLI adapter.

    This is opt-in and primarily useful for compatibility/evaluation. The
    intended long-term shape is for the Cortex supervisor to delegate code-only
    jobs to an isolated OpenCode worker while it retains robot authority.
    """

    name = "opencode"

    def __init__(self) -> None:
        self.command = self._read_command()
        self.model = os.environ.get("CORTEX_MODEL") or None
        self.agent = os.environ.get("CORTEX_OPENCODE_AGENT") or None
        self.attach = os.environ.get("CORTEX_OPENCODE_URL") or None
        self._conversation_id: Optional[str] = None
        self._response_parts: list[str] = []
        self._emitted_init = False

    @staticmethod
    def _read_command() -> list[str]:
        raw = os.environ.get("CORTEX_OPENCODE_COMMAND", "opencode").strip()
        try:
            decoded = json.loads(raw)
        except json.JSONDecodeError:
            decoded = None
        if isinstance(decoded, list) and all(isinstance(item, str) for item in decoded):
            return decoded
        return shlex.split(raw)

    def status(self) -> Dict[str, Any]:
        executable = self.command[0] if self.command else None
        available = bool(
            executable
            and (
                (Path(executable).is_file() and os.access(executable, os.X_OK))
                or shutil.which(executable)
            )
        )
        return {
            "name": self.name,
            "available": available,
            "binary": executable,
            "model": self.model or "OpenCode default",
            "agent": self.agent or "OpenCode default",
            "server": self.attach,
            "mcp": True,
        }

    async def start(self, request: ProviderRequest) -> asyncio.subprocess.Process:
        if not self.status()["available"]:
            raise RuntimeError(
                "OpenCode is unavailable; set CORTEX_OPENCODE_COMMAND to its executable"
            )

        self._conversation_id = request.conversation_id
        self._response_parts = []
        self._emitted_init = False
        command = [*self.command, "run", "--format", "json", "--auto"]
        if self.attach:
            command.extend(["--attach", self.attach, "--dir", request.workspace])
        else:
            command.extend(["--dir", request.workspace])
        if self.model:
            command.extend(["--model", self.model])
        if self.agent:
            command.extend(["--agent", self.agent])
        if request.conversation_id:
            command.extend(["--session", request.conversation_id])
        command.append(request.prompt)

        return await asyncio.create_subprocess_exec(
            *command,
            cwd=request.workspace,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

    def parse_event(self, line: str) -> Optional[Dict[str, Any]]:
        try:
            payload = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None

        session_id = payload.get("sessionID")
        if isinstance(session_id, str):
            self._conversation_id = session_id

        event_type = payload.get("type")
        part = payload.get("part") or {}
        if event_type == "text" and isinstance(part, dict):
            text = part.get("text")
            if isinstance(text, str) and text:
                self._response_parts.append(text)
                return {"type": "token", "delta": text}
        if event_type == "tool_use" and isinstance(part, dict):
            state = part.get("state") or {}
            tool = part.get("tool") or "tool"
            if state.get("status") == "completed":
                return {
                    "type": "tool_output",
                    "tool": tool,
                    "output": state.get("output", ""),
                }
            if state.get("status") == "error":
                return {
                    "type": "tool_output",
                    "tool": tool,
                    "output": state.get("error", "OpenCode tool failed"),
                }
            return {
                "type": "tool_call",
                "tool": tool,
                "params": state.get("input") or {},
            }
        if event_type == "step_start" and not self._emitted_init:
            self._emitted_init = True
            return {"type": "init", "conversation_id": self._conversation_id}
        if event_type == "error":
            error = payload.get("error")
            if isinstance(error, dict):
                detail = error.get("message") or error.get("name") or json.dumps(error)
            else:
                detail = str(error or "OpenCode execution failed")
            return {"type": "error", "error": detail}
        return None

    def finish_event(self, return_code: int, stderr: str) -> Dict[str, Any]:
        if return_code != 0:
            return {
                "type": "error",
                "error": stderr or f"OpenCode exited with status {return_code}",
            }
        return {
            "type": "done",
            "response": "".join(self._response_parts),
            "status": "SUCCESS",
            "usage": {},
        }


def get_provider() -> AgentProvider:
    name = os.environ.get("CORTEX_PROVIDER", "agy").strip().lower()
    if name == "agy":
        return AgyProvider()
    if name in {"ndjson", "bridge"}:
        return NdjsonBridgeProvider()
    if name in {"opencode", "open-code"}:
        return OpenCodeProvider()
    raise ValueError(
        f"Unknown CORTEX_PROVIDER '{name}' "
        "(expected 'agy', 'opencode', or 'ndjson')"
    )
