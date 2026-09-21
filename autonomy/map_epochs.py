"""Robot-local map lifetimes, independent of fleet missions and graph epochs."""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import tempfile
from uuid import UUID, uuid5

_ROBOT = re.compile(r"[A-Za-z0-9_.-]{1,128}\Z")


def robot_run_id(mission_id: str, robot_id: str, map_epoch: int) -> str:
    """Return the immutable keyframe namespace for one robot's map lifetime."""
    if not isinstance(mission_id, str):
        raise ValueError("mission_id must be a canonical UUID")
    mission = UUID(mission_id)
    if str(mission) != mission_id:
        raise ValueError("mission_id must be a canonical UUID")
    if (
        not isinstance(robot_id, str)
        or not _ROBOT.fullmatch(robot_id)
        or robot_id in {".", ".."}
    ):
        raise ValueError("invalid robot_id")
    if type(map_epoch) is not int or not 0 <= map_epoch < 2**63:
        raise ValueError("map_epoch must be a non-negative signed 64-bit integer")
    return str(uuid5(mission, f"swarmdeck-map:{robot_id}:{map_epoch}"))


@contextmanager
def map_epoch_lock(peer_root: str | Path):
    """Serialize lifetime changes against product/snapshot publication."""
    root = Path(peer_root)
    root.mkdir(parents=True, exist_ok=True)
    with (root / "map-epoch.lock").open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def read_map_epoch(peer_root: str | Path) -> dict | None:
    """Read a durable claim; absence is allowed, corruption is never reset to zero."""
    try:
        record = json.loads((Path(peer_root) / "map-epoch.json").read_text())
    except FileNotFoundError:
        return None
    if (
        not isinstance(record, dict)
        or record.get("version") != 1
        or record.get("run_id")
        != robot_run_id(
            record.get("mission_id"), record.get("robot_id"), record.get("map_epoch")
        )
    ):
        raise ValueError("invalid persisted map epoch")
    return record


def claim_map_epoch(
    root: str | Path, mission_id: str, robot_id: str, minimum: int = 0
) -> int:
    """Claim a new frontend lifetime before any of its processes start.

    Every launch claims once, including an automatic container restart. The
    counter is fsynced before old live publications are removed; a crash may
    skip an epoch but can never reuse keyframe identities. Immutable geometry
    is retained for content-addressed readers, not admitted to the new graph.
    """
    robot_run_id(mission_id, robot_id, minimum)
    peer_root = Path(root) / mission_id / robot_id
    with map_epoch_lock(peer_root):
        previous = read_map_epoch(peer_root)
        if previous is not None and (
            previous["mission_id"] != mission_id or previous["robot_id"] != robot_id
        ):
            raise ValueError("persisted map epoch belongs to another robot or mission")
        epoch = max(minimum, 0 if previous is None else previous["map_epoch"] + 1)
        record = {
            "version": 1,
            "mission_id": mission_id,
            "robot_id": robot_id,
            "map_epoch": epoch,
            "run_id": robot_run_id(mission_id, robot_id, epoch),
        }
        fd, name = tempfile.mkstemp(prefix=".map-epoch-", dir=peer_root)
        try:
            with os.fdopen(fd, "w") as stream:
                json.dump(record, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, peer_root / "map-epoch.json")
            geometry = peer_root / "runs" / record["run_id"] / "geometry"
            geometry.mkdir(parents=True, exist_ok=True)
            live_geometry = peer_root / "geometry"
            if live_geometry.exists() and not live_geometry.is_symlink():
                archive = Path(tempfile.mkdtemp(prefix="retired-", dir=peer_root))
                os.replace(live_geometry, archive / "geometry")
            link = peer_root / ".geometry-next"
            link.unlink(missing_ok=True)
            link.symlink_to(geometry.relative_to(peer_root), target_is_directory=True)
            os.replace(link, live_geometry)
            for relative in (
                "snapshot.json",
                "graph_solution.json",
                "status.json",
                "mola/index.json",
                "mola/source.json",
                "mola/worker.json",
            ):
                (peer_root / relative).unlink(missing_ok=True)
            directory = os.open(peer_root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            Path(name).unlink(missing_ok=True)
        return epoch


def read_peer_epochs(peer_root: str | Path) -> dict | None:
    try:
        record = json.loads((Path(peer_root) / "peer-epochs.json").read_text())
    except FileNotFoundError:
        return None
    if not isinstance(record, dict) or record.get("version") != 1:
        raise ValueError("invalid peer epoch watermark")
    epochs = record.get("robot_map_epochs")
    if not isinstance(epochs, dict) or not epochs or len(epochs) > 256:
        raise ValueError("invalid peer epoch vector")
    for robot, epoch in epochs.items():
        robot_run_id(record.get("mission_id"), robot, epoch)
    return record


def write_peer_epochs(
    peer_root: str | Path, mission_id: str, epochs: dict[str, int]
) -> None:
    root = Path(peer_root)
    for robot, epoch in epochs.items():
        robot_run_id(mission_id, robot, epoch)
    previous = read_peer_epochs(root)
    if previous is not None:
        if previous["mission_id"] != mission_id:
            raise ValueError("peer watermark belongs to another mission")
        if any(
            epochs.get(robot, -1) < epoch
            for robot, epoch in previous["robot_map_epochs"].items()
        ):
            raise ValueError("peer epoch watermark cannot move backwards")
    record = {"version": 1, "mission_id": mission_id, "robot_map_epochs": epochs}
    if record == previous:
        return
    fd, name = tempfile.mkstemp(prefix=".peer-epochs-", dir=root)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(record, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, root / "peer-epochs.json")
        directory = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        Path(name).unlink(missing_ok=True)


def assert_map_epoch_dependencies(peer_root: str | Path, dependencies) -> None:
    """Fence a product against this robot's durable accepted peer watermarks.

    Remote robots need not share a filesystem. A missing or older watermark
    cannot qualify a dependency; only unclaimed offline fixtures omit vectors.
    """
    root = Path(peer_root)
    own = read_map_epoch(root)
    if not dependencies:
        if own is not None:
            raise ValueError("claimed map product lacks epoch dependencies")
        return
    watermark = read_peer_epochs(root)
    if watermark is None:
        raise ValueError("map product has no durable peer epoch watermark")
    mission = watermark["mission_id"]
    if own is not None and mission != own["mission_id"]:
        raise ValueError("map product peer watermark mission mismatch")
    if own is not None and own["robot_id"] not in dependencies:
        raise ValueError("map product omits its owner's epoch dependency")
    for robot, epoch in dependencies.items():
        robot_run_id(mission, robot, epoch)
        if watermark["robot_map_epochs"].get(robot) != epoch:
            raise ValueError("map product dependency belongs to a retired robot epoch")
        if own is not None and robot == own["robot_id"] and own["map_epoch"] != epoch:
            raise ValueError("map product owner belongs to a retired robot epoch")


def snapshot_epoch_dependencies(snapshot) -> dict[str, int]:
    """Read explicit graph participants, never infer unknown peer lifetimes."""
    epochs = snapshot.get("robot_map_epochs")
    participants = snapshot.get("participant_robot_ids")
    if epochs is None and participants is None and "run_id" not in snapshot:
        return {}
    if (
        not isinstance(epochs, dict)
        or not isinstance(participants, list)
        or not participants
        or len(participants) > 256
        or any(not isinstance(robot, str) for robot in participants)
        or len(set(participants)) != len(participants)
        or snapshot.get("robot_id") not in participants
    ):
        raise ValueError("map product has invalid epoch dependencies")
    owner = snapshot["robot_id"]
    epoch = snapshot.get("robot_map_epoch")
    if epochs.get(owner) != epoch or snapshot.get("run_id") != robot_run_id(
        snapshot.get("mission_id"), owner, epoch
    ):
        raise ValueError("map product has inconsistent owner epoch identity")
    result = {}
    for robot in participants:
        epoch = epochs.get(robot)
        robot_run_id(snapshot.get("mission_id"), robot, epoch)
        result[robot] = epoch
    return result
