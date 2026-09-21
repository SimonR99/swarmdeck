"""Filesystem boundary to the host-owned simulation reset supervisor."""

from __future__ import annotations

import fcntl
import json
import os
import time
from contextlib import contextmanager
from pathlib import Path
from uuid import UUID, uuid4

SUPERVISOR_STALE_NS = 15_000_000_000


def reset_root() -> Path | None:
    value = os.environ.get("SWARMDECK_SIM_RESET_DIR", "").strip()
    return Path(value) if value else None


def _read(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid4().hex}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True))
    temporary.chmod(
        0o644
    )  # The host reads root-container writes across this bind mount.
    os.replace(temporary, path)


@contextmanager
def _protocol_lock(root: Path):
    descriptor = os.open(root / "protocol.lock", os.O_RDONLY | os.O_CREAT, 0o666)
    with os.fdopen(descriptor) as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def _fresh(value: dict) -> bool:
    updated = value.get("updated_at_ns", value.get("requested_at_ns"))
    return type(updated) is int and 0 <= time.time_ns() - updated <= SUPERVISOR_STALE_NS


def _unavailable(status: dict | None = None) -> dict:
    return {
        **(status or {}),
        "version": 1,
        "phase": "failed",
        "ok": False,
        "error": "simulation reset supervisor is unavailable or stale",
        "updated_at_ns": time.time_ns(),
    }


def _request_id(value: str | None) -> str:
    if value is None:
        return str(uuid4())
    try:
        parsed = UUID(value)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("simulation reset request_id must be a UUID") from exc
    if str(parsed) != value:
        raise ValueError("simulation reset request_id must be a canonical UUID")
    return str(parsed)


def request_reset(root: Path, client_request_id: str | None = None) -> dict:
    # The host supervisor creates this directory and owns its atomic files. If
    # the container creates it first, the unprivileged host process may never be
    # able to publish progress into the bind mount.
    if not root.is_dir():
        return _unavailable()
    requested_id = _request_id(client_request_id)
    with _protocol_lock(root):
        status = _read(root / "status.json")
        pending = _read(root / "request.json")
        if status.get("phase") in {"accepted", "stopping", "starting", "verifying"}:
            if _fresh(status):
                return status
            failed = _unavailable(status)
            _atomic_json(root / "status.json", failed)
            return failed

        # A POST response can be lost when the supervisor stops the backend.
        # Returning the terminal record for the caller's key makes retrying
        # that POST safe and prevents one click from becoming two resets.
        if status.get("request_id") == requested_id:
            return status

        # Do not accept work into a directory nobody is watching. Without this
        # lease the button appears to work for fifteen seconds before timing out.
        if not _fresh(_read(root / "supervisor.json")):
            failed = _unavailable(status)
            _atomic_json(root / "status.json", failed)
            return failed

        if isinstance(pending.get("request_id"), str) and pending.get(
            "request_id"
        ) != status.get("request_id"):
            return {**pending, "phase": "accepted", "ok": None}
        request = {
            "version": 1,
            "request_id": requested_id,
            "requested_at_ns": time.time_ns(),
        }
        accepted = {
            **request,
            "phase": "accepted",
            "ok": None,
            "updated_at_ns": time.time_ns(),
        }
        # Publish the durable work item first. The supervisor takes the same
        # lock while reading both files, so it cannot observe the intermediate
        # pair; if this process dies between replaces, the new request still
        # survives and is claimed against the older status.
        _atomic_json(root / "request.json", request)
        _atomic_json(root / "status.json", accepted)
        return accepted


def reset_status(root: Path) -> dict:
    status = _read(root / "status.json")
    if status.get("phase") in {"accepted", "stopping", "starting", "verifying"}:
        if not _fresh(status):
            status = _unavailable(status)
    return {
        **(status or {"version": 1, "phase": "idle", "ok": None}),
        # A completed reset describes history, not a live simulation lease.
        "supervisor_available": _fresh(_read(root / "supervisor.json")),
    }


def robot_reset_status(
    root: Path, robot_id: str, request_id: str | None = None
) -> dict:
    from autonomy.replication import identity

    heartbeat = _read(root / "supervisor.json")
    mission = heartbeat.get("mission_id") or os.environ.get("SWARMDECK_MISSION_ID")
    identity(robot_id, mission)
    directory = root / "robots" / robot_id
    status = _read(directory / "status.json")
    if request_id is not None:
        request_id = _request_id(request_id)
        if status.get("request_id") != request_id:
            status = _read(directory / "requests" / f"{request_id}.json")
    pending = _read(directory / "request.json")
    if pending and (request_id is None or pending.get("request_id") == request_id):
        if status.get("request_id") != pending.get("request_id"):
            status = {**pending, "phase": "accepted", "ok": None}
    available = _fresh(heartbeat) and robot_id in heartbeat.get(
        "supported_robot_ids", []
    )
    if status.get("phase") in {"accepted", "stopping", "starting", "verifying"}:
        deadline = status.get(
            "deadline_at_ns", status.get("requested_at_ns", 0) + 60_000_000_000
        )
        if not available or time.time_ns() > deadline:
            status = {
                **status,
                "phase": "failed",
                "ok": False,
                "error": "robot map reset supervisor is unavailable or reset deadline expired",
            }
    return {
        **(status or {"version": 1, "phase": "idle", "ok": None, "robot_id": robot_id}),
        "supervisor_available": available,
    }


def request_robot_reset(
    root: Path,
    robot_id: str,
    mission_id: str,
    client_request_id: str,
    reserve,
) -> dict:
    """Reserve a fresh lifetime and durably enqueue exactly one target restart.

    ``reserve`` runs under the target protocol lock and returns the epoch fenced
    in SQLite. A duplicate UUID never invokes it again, even after another reset.
    """
    from autonomy.map_epochs import robot_run_id
    from autonomy.replication import identity

    identity(robot_id, mission_id)
    request_id = _request_id(client_request_id)
    base = {
        "version": 1,
        "request_id": request_id,
        "robot_id": robot_id,
        "mission_id": mission_id,
    }
    directory = root / "robots" / robot_id
    # Terminal journal entries remain truthful when the supervisor is offline.
    archived = _read(directory / "requests" / f"{request_id}.json")
    if archived.get("phase") in {"done", "failed"}:
        return archived
    heartbeat = _read(root / "supervisor.json")
    if (
        not _fresh(heartbeat)
        or heartbeat.get("mission_id") != mission_id
        or robot_id not in heartbeat.get("supported_robot_ids", [])
    ):
        return {
            **base,
            "phase": "failed",
            "ok": False,
            "error": "robot map reset supervisor is unavailable or does not support this robot",
        }
    directory.mkdir(parents=True, exist_ok=True)
    with _protocol_lock(directory):
        status = _read(directory / "status.json")
        archived = _read(directory / "requests" / f"{request_id}.json")
        if status.get("request_id") == request_id:
            return robot_reset_status(root, robot_id, request_id)
        if archived:
            return archived
        pending = _read(directory / "request.json")
        if pending and pending.get("request_id") != status.get("request_id"):
            return {**pending, "phase": "accepted", "ok": None}
        if status.get("phase") in {"accepted", "stopping", "starting", "verifying"}:
            return robot_reset_status(root, robot_id)
        epoch = reserve()
        request = {
            **base,
            "map_epoch": epoch,
            "requested_at_ns": time.time_ns(),
        }
        accepted = {
            **request,
            "run_id": robot_run_id(mission_id, robot_id, epoch),
            "phase": "accepted",
            "ok": None,
            "updated_at_ns": time.time_ns(),
        }
        (directory / "requests").mkdir(exist_ok=True)
        _atomic_json(directory / "request.json", request)
        _atomic_json(directory / "requests" / f"{request_id}.json", accepted)
        _atomic_json(directory / "status.json", accepted)
        return accepted
