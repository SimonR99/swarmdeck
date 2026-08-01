"""In-process async pub/sub.

Every module talks through this and nothing else, so any of them can be
extracted later and `record/` can replay a session with no adapters attached.
"""

from __future__ import annotations

import asyncio
import time
from collections import defaultdict
from typing import Any, Awaitable, Callable

Handler = Callable[[dict[str, Any]], Awaitable[None] | None]

_T0 = time.monotonic()
_SESSION_T0: float | None = None


def stamps() -> dict[str, float]:
    """Three timestamps on every record (architecture.md §8)."""
    mono = time.monotonic() - _T0
    return {
        "t_mono": round(mono, 4),
        "t_wall": round(time.time(), 4),
        "t_sess": round(mono - _SESSION_T0, 4) if _SESSION_T0 is not None else 0.0,
    }


def mark_session_start() -> None:
    global _SESSION_T0
    _SESSION_T0 = time.monotonic() - _T0


def session_elapsed() -> float:
    if _SESSION_T0 is None:
        return 0.0
    return time.monotonic() - _T0 - _SESSION_T0


class Bus:
    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)

    def on(self, topic: str, handler: Handler) -> None:
        self._subs[topic].append(handler)

    def off(self, topic: str, handler: Handler) -> None:
        if handler in self._subs[topic]:
            self._subs[topic].remove(handler)

    async def emit(self, topic: str, payload: dict[str, Any]) -> None:
        for h in list(self._subs[topic]) + list(self._subs["*"]):
            try:
                r = h(payload)
                if asyncio.iscoroutine(r):
                    await r
            except Exception as exc:  # a bad subscriber must not kill the publisher
                print(f"[bus] handler error on {topic}: {exc}")


bus = Bus()
