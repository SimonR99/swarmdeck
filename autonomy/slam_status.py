"""Small peer SLAM diagnostics, independent of map geometry and solver authority."""

from .replication import identity
from .map_epochs import robot_run_id


def peer_status(value, robot_id, mission_id=None):
    if not isinstance(value, dict) or value.get("robot_id") != robot_id:
        raise ValueError("peer SLAM robot mismatch")
    mission = value.get("mission_id")
    identity(robot_id, mission)
    if mission_id is not None and mission != mission_id:
        raise ValueError("peer SLAM mission mismatch")
    result = {"robot_id": robot_id, "mission_id": mission}
    epoch = value["robot_map_epoch"]
    run_id = robot_run_id(mission, robot_id, epoch)
    if value["run_id"] != run_id:
        raise ValueError("peer SLAM run mismatch")
    result.update(robot_map_epoch=epoch, run_id=run_id)
    for field in ("keyframes", "verified", "rejected"):
        count = value.get(field)
        if type(count) is not int or not 0 <= count <= 2**53 - 1:
            raise ValueError("invalid peer SLAM counter")
        result[field] = count
    peers = value.get("by_peer")
    if not isinstance(peers, dict) or len(peers) > 256:
        raise ValueError("invalid peer SLAM partners")
    result["by_peer"] = {}
    for peer, count in peers.items():
        identity(peer, mission)
        if peer == robot_id or type(count) is not int or not 0 <= count <= 2**53 - 1:
            raise ValueError("invalid peer closure counter")
        result["by_peer"][peer] = count
    return result
