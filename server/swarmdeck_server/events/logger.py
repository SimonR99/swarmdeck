"""Operator action and system event log.

One JSONL line per event, three timestamps each. This is the handoff artefact
for offline analysis, so it must never silently drop.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..bus import bus, stamps


class EventLogger:
    def __init__(self) -> None:
        self._fh = None
        self.path: Path | None = None
        self.count = 0

    def open(self, session_dir: Path) -> None:
        self.close()
        session_dir.mkdir(parents=True, exist_ok=True)
        self.path = session_dir / "events.jsonl"
        self._fh = self.path.open("a", buffering=1)  # line buffered
        self.count = 0

    def close(self) -> None:
        if self._fh:
            self._fh.flush()
            self._fh.close()
        self._fh = None

    def log(self, kind: str, payload: dict[str, Any]) -> dict[str, Any]:
        rec = {"kind": kind, **stamps(), **payload}
        if self._fh:
            self._fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self.count += 1
        return rec


events = EventLogger()
