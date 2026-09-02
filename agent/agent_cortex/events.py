"""Stable events shared by Cortex runtimes, the supervisor, and the UI."""

from __future__ import annotations

import json
from typing import Any, Mapping


NORMALIZED_EVENT_TYPES = {
    "init",
    "token",
    "tool_call",
    "tool_output",
    "done",
    "error",
}
TERMINAL_EVENT_TYPES = {"done", "error"}


def is_normalized_event(event: object) -> bool:
    return isinstance(event, Mapping) and event.get("type") in NORMALIZED_EVENT_TYPES


def encode_sse(event: Mapping[str, Any]) -> str:
    """Encode one event without changing Cortex's existing wire format."""
    return f"data: {json.dumps(dict(event))}\n\n"
