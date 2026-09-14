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
    return isinstance(updated, int) and time.time_ns() - updated <= SUPERVISOR_STALE_NS


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
            return _unavailable(status)
    return status or {
        "version": 1,
        "phase": "idle",
        "ok": None,
    }
