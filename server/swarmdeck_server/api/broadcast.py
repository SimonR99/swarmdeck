"""Bounded GUI fan-out with one JSON encoding per publication."""

from __future__ import annotations

import asyncio
import json
from collections.abc import Callable, Iterable
from typing import Any

SEND_TIMEOUT_S = 1.0
CLOSE_TIMEOUT_S = 0.25


class JsonBroadcaster:
    """Ordered publications, shared encoding, and bounded socket writes.

    Publishers await delivery instead of building a background queue. The lock
    orders concurrent publishers and keeps initial snapshots ahead of updates.
    """

    def __init__(self, clients: set):
        self.clients = clients
        self._lock = asyncio.Lock()

    async def subscribe(
        self, client: Any, snapshot: Callable[[], Iterable[dict[str, Any]]]
    ) -> None:
        async with self._lock:
            # Materialize under the publication lock before the first await.
            frames = [
                json.dumps(frame, separators=(",", ":"), ensure_ascii=False)
                for frame in snapshot()
            ]

            async def send_snapshot():
                for frame in frames:
                    await client.send_text(frame)

            try:
                await asyncio.wait_for(send_snapshot(), SEND_TIMEOUT_S)
            except Exception:
                await self._retire(client)
                raise
            self.clients.add(client)

    async def _retire(self, client: Any) -> None:
        self.clients.discard(client)
        # Closing releases the GUI handler's camera-interest state.
        try:
            await asyncio.wait_for(client.close(code=1013), CLOSE_TIMEOUT_S)
        except Exception:
            pass

    async def publish(self, message: dict[str, Any]) -> None:
        async with self._lock:
            recipients = tuple(self.clients)
            if not recipients:
                return
            payload = json.dumps(message, separators=(",", ":"), ensure_ascii=False)

            async def send(client):
                try:
                    await asyncio.wait_for(client.send_text(payload), SEND_TIMEOUT_S)
                except Exception:
                    await self._retire(client)

            if len(recipients) == 1:
                await send(recipients[0])
            else:
                await asyncio.gather(*(send(client) for client in recipients))
