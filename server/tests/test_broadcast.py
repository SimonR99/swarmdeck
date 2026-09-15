import asyncio
import json
from unittest.mock import patch

import pytest

from swarmdeck_server.api import broadcast


class Client:
    def __init__(self, blocked=False):
        self.blocked = blocked
        self.frames = []
        self.closed = None
        self.received = asyncio.Event()

    async def send_text(self, text):
        if self.blocked:
            await asyncio.Event().wait()
        self.frames.append(text)
        self.received.set()

    async def close(self, code):
        self.closed = code


def test_one_encoding_for_all_clients_and_none_without_clients():
    async def run():
        clients = {Client(), Client(), Client()}
        publisher = broadcast.JsonBroadcaster(clients)
        message = {"type": "state", "name": "é", "path": [[1.5, 2.0, 0.1]]}
        with patch.object(broadcast.json, "dumps", wraps=json.dumps) as encode:
            await broadcast.JsonBroadcaster(set()).publish(message)
            encode.assert_not_called()
            await publisher.publish(message)
            assert encode.call_count == 1
        assert all(json.loads(client.frames[0]) == message for client in clients)
        assert len({id(client.frames[0]) for client in clients}) == 1

    asyncio.run(run())


def test_slow_client_does_not_hold_healthy_client_and_is_closed(monkeypatch):
    async def run():
        slow, healthy = Client(blocked=True), Client()
        clients = {slow, healthy}
        publisher = broadcast.JsonBroadcaster(clients)
        task = asyncio.create_task(publisher.publish({"type": "state"}))
        await asyncio.wait_for(healthy.received.wait(), 0.5)
        assert slow.closed is None
        await task
        assert clients == {healthy}
        assert slow.closed == 1013

    monkeypatch.setattr(broadcast, "SEND_TIMEOUT_S", 0.05)
    asyncio.run(run())


def test_concurrent_publications_and_initial_snapshot_stay_ordered():
    async def run():
        release = asyncio.Event()
        entered = asyncio.Event()

        class DelayedClient(Client):
            async def send_text(self, text):
                if json.loads(text)["seq"] == 0:
                    entered.set()
                    await release.wait()
                await super().send_text(text)

        client = DelayedClient()
        publisher = broadcast.JsonBroadcaster(set())
        subscribe = asyncio.create_task(
            publisher.subscribe(client, lambda: [{"seq": 0}])
        )
        await entered.wait()
        first = asyncio.create_task(publisher.publish({"seq": 1}))
        second = asyncio.create_task(publisher.publish({"seq": 2}))
        await asyncio.sleep(0)
        assert not client.frames
        release.set()
        await asyncio.gather(subscribe, first, second)
        assert [json.loads(frame)["seq"] for frame in client.frames] == [0, 1, 2]

    asyncio.run(run())


def test_cancellation_releases_publisher_without_retiring_clients():
    async def run():
        client = Client(blocked=True)
        clients = {client}
        publisher = broadcast.JsonBroadcaster(clients)
        task = asyncio.create_task(publisher.publish({"seq": 1}))
        await asyncio.sleep(0)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        else:
            raise AssertionError("publication swallowed cancellation")
        assert clients == {client}
        assert client.closed is None
        client.blocked = False
        await asyncio.wait_for(publisher.publish({"seq": 2}), 0.5)
        assert [json.loads(frame)["seq"] for frame in client.frames] == [2]

    asyncio.run(run())


def test_snapshot_has_one_total_deadline_and_releases_queued_publication(monkeypatch):
    async def run():
        entered = asyncio.Event()

        class SlowSnapshotClient(Client):
            async def send_text(self, text):
                entered.set()
                await asyncio.sleep(0.03)
                await super().send_text(text)

        slow, healthy = SlowSnapshotClient(), Client()
        publisher = broadcast.JsonBroadcaster({healthy})
        subscribe = asyncio.create_task(
            publisher.subscribe(slow, lambda: [{"seq": i} for i in range(10)])
        )
        await entered.wait()
        publication = asyncio.create_task(publisher.publish({"seq": 10}))
        with pytest.raises(asyncio.TimeoutError):
            await subscribe
        await asyncio.wait_for(publication, 0.5)
        assert len(slow.frames) < 10
        assert slow.closed == 1013
        assert publisher.clients == {healthy}
        assert [json.loads(frame)["seq"] for frame in healthy.frames] == [10]

    monkeypatch.setattr(broadcast, "SEND_TIMEOUT_S", 0.05)
    asyncio.run(run())
