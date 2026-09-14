import pytest
from autonomy.slam_status import peer_status
from swarmdeck_server.fleet.registry import Registry

MISSION = "12345678-1234-4234-8234-567812345678"


def report():
    return dict(robot_id="r0", mission_id=MISSION, keyframes=12,
                verified=3, rejected=8, by_peer={"r1": 3})


def test_reports_roundtrip_through_fleet_and_expire(monkeypatch):
    monkeypatch.setenv("SWARMDECK_MISSION_ID", MISSION)
    registry = Registry()
    robot = registry.hello({"robot_id": "r0"}, None)
    registry.update_state({"robot_id": "r0", "peer_slam": report()})
    assert robot.to_state()["peer_slam"] == report()
    robot.last_seen -= 100
    assert robot.to_state()["peer_slam"] is None
    registry.update_state({"robot_id": "r0"})
    assert robot.to_state()["peer_slam"] is None


@pytest.mark.parametrize("update", [
    {"robot_id": "r1"}, {"verified": True}, {"keyframes": -1},
    {"by_peer": {"r0": 1}}, {"by_peer": {"r1": -1}},
    {"by_peer": {str(i): 1 for i in range(257)}},
])
def test_invalid_identity_or_counter_rejected(update):
    with pytest.raises(ValueError):
        peer_status({**report(), **update}, "r0", MISSION)


def test_old_mission_rejected():
    with pytest.raises(ValueError, match="mission"):
        peer_status(report(), "r0", "92345678-1234-4234-8234-567812345678")
