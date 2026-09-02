"""Execution runtimes for tool-using agent providers.

This module owns process lifecycle details so the HTTP service and supervisor do
not need to know how AGY, OpenCode, or an NDJSON bridge is launched.
"""

from __future__ import annotations

import asyncio
from typing import Any, AsyncGenerator, Dict

from .events import TERMINAL_EVENT_TYPES
from .providers import AgentProvider, ProviderRequest


class ProviderRuntime:
    """Stream normalized events from one process-backed provider invocation."""

    def __init__(self, provider: AgentProvider) -> None:
        self.provider = provider

    async def run(
        self, request: ProviderRequest
    ) -> AsyncGenerator[Dict[str, Any], None]:
        try:
            process = await self.provider.start(request)
        except Exception as exc:
            yield {"type": "error", "error": str(exc)}
            return

        assert process.stdout is not None
        assert process.stderr is not None

        async def read_stderr() -> str:
            chunks: list[bytes] = []
            size = 0
            while True:
                chunk = await process.stderr.read(4096)
                if not chunk:
                    break
                chunks.append(chunk)
                size += len(chunk)
                while size > 65536 and chunks:
                    size -= len(chunks.pop(0))
            return b"".join(chunks).decode(errors="replace").strip()

        stderr_task = asyncio.create_task(read_stderr())
        sent_terminal_event = False

        try:
            while True:
                line = await process.stdout.readline()
                if not line:
                    break
                line_text = line.decode(errors="replace").strip()
                if not line_text:
                    continue

                event = self.provider.parse_event(line_text)
                if not event:
                    continue
                if event.get("type") in TERMINAL_EVENT_TYPES:
                    sent_terminal_event = True
                yield event

            return_code = await process.wait()
            stderr_text = await stderr_task
            if not sent_terminal_event:
                finish_event = getattr(self.provider, "finish_event", None)
                if callable(finish_event):
                    event = finish_event(return_code, stderr_text)
                    if event:
                        sent_terminal_event = event.get("type") in TERMINAL_EVENT_TYPES
                        yield event
                if not sent_terminal_event:
                    detail = stderr_text or f"provider exited with status {return_code}"
                    yield {"type": "error", "error": detail}
        except asyncio.CancelledError:
            try:
                process.kill()
                await process.wait()
            except Exception:
                pass
            raise
        except Exception as exc:
            yield {"type": "error", "error": str(exc)}
        finally:
            if not stderr_task.done():
                stderr_task.cancel()
