"""Process and session timestamps shared by commands and event records."""

from __future__ import annotations

import time

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
