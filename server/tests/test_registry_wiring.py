"""The composition root must wire map authority into the shared registry."""

from swarmdeck_server.api import app, map_routes
from swarmdeck_server.fleet.registry import registry


def test_registered_registry_uses_map_epoch_and_command_guard(monkeypatch):
    monkeypatch.setattr(map_routes, "cached_map_epoch", lambda rid, mission: 7)
    monkeypatch.setattr(
        map_routes, "robot_command_error", lambda rid: f"waiting for {rid}"
    )
    assert registry.epoch_store().map_epoch("r0", "mission") == 7
    assert registry.command_guard("r0") == "waiting for r0"
