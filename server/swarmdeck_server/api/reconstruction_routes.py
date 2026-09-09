"""Read-only, operator-published reconstruction assets; training is a separate process."""

import os
from pathlib import Path
import hashlib
import json
import re
import struct
from fastapi import Request, Response
from fastapi.responses import FileResponse

MAX_BYTES = 16 + 2_000_000 * 56
_SHA256 = re.compile(r"[a-f0-9]{64}\Z")


class _PointerError(ValueError):
    """A manifest pointer exists but cannot be trusted as a model source."""


def _within(path: Path, root: Path) -> bool:
    try:
        return os.path.commonpath((str(path), str(root))) == str(root)
    except ValueError:
        return False


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source(root: Path) -> tuple[Path, dict, os.stat_result, os.stat_result | None]:
    """Resolve the atomic pointer, falling back only when it is absent."""
    pointer_path = root / "global.swgs.manifest.json"
    try:
        pointer_stat = pointer_path.stat()
    except FileNotFoundError:
        path = root / "global.swgs"
        return path, {"state": "legacy", "version": "legacy"}, path.stat(), None
    try:
        pointer = json.loads(pointer_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _PointerError("reconstruction manifest pointer is invalid") from exc
    if not isinstance(pointer, dict) or not isinstance(pointer.get("artifact"), str):
        raise _PointerError("reconstruction manifest pointer has no artifact")
    state = pointer.get("state", "ready")
    if state not in {"ready", "stale", "canceled", "failed"}:
        raise _PointerError("reconstruction pointer has an invalid state")
    if state in {"stale", "canceled", "failed"}:
        raise _PointerError(f"reconstruction pointer is {state}")
    path = Path(pointer["artifact"]).expanduser().resolve()
    root = root.resolve()
    if not _within(path, root):
        raise _PointerError("reconstruction artifact escapes its configured root")
    try:
        artifact_stat = path.stat()
    except FileNotFoundError as exc:
        raise _PointerError("reconstruction artifact from pointer is missing") from exc
    declared_size = pointer.get("artifact_size_bytes")
    if declared_size is not None and (
        type(declared_size) is not int or declared_size != artifact_stat.st_size
    ):
        raise _PointerError("reconstruction artifact size does not match its pointer")
    declared_digest = pointer.get("artifact_sha256")
    if declared_digest is not None:
        if not isinstance(declared_digest, str) or not _SHA256.fullmatch(declared_digest):
            raise _PointerError("reconstruction artifact checksum is invalid")
        if _file_digest(path) != declared_digest:
            raise _PointerError("reconstruction artifact checksum does not match its pointer")
    return path, pointer, artifact_stat, pointer_stat


async def get_gaussians(request: Request):
    root = os.environ.get("SWARMDECK_RECONSTRUCTION_DIR")
    # Published models are world-aligned. Local clouds can belong to disconnected maps.
    if not root or request.query_params.get("robot_id"):
        return Response(status_code=404)
    try:
        path, source, stat, pointer_stat = _source(Path(root))
        if stat.st_size > MAX_BYTES or stat.st_size < 16:
            return Response(status_code=422)
        with path.open("rb") as f:
            header = f.read(16)
            if len(header) != 16:
                return Response(status_code=422)
            magic, version, count, _ = struct.unpack("<4sIII", header)
        if magic != b"SWGS" or version != 1 or stat.st_size != 16 + count * 56:
            return Response(status_code=422)
    except FileNotFoundError:
        return Response(status_code=404)
    except _PointerError:
        return Response(status_code=409)
    etag = f'"{stat.st_ino:x}-{stat.st_mtime_ns:x}-{stat.st_size:x}"'
    if pointer_stat is not None:
        etag = f'{etag[:-1]}-{pointer_stat.st_mtime_ns:x}"'
    headers = {
        "ETag": etag,
        "Cache-Control": "no-cache",
        "X-Reconstruction-Frame": "world",
        "X-Reconstruction-State": str(source.get("state", "ready")),
        "X-Reconstruction-Version": str(source.get("version", source.get("job_id", "legacy"))),
    }
    if source.get("job_id"):
        headers["X-Reconstruction-Job"] = str(source["job_id"])
    if source.get("input_fingerprint"):
        headers["X-Reconstruction-Input-Fingerprint"] = str(source["input_fingerprint"])
    if request.headers.get("if-none-match") == etag:
        return Response(status_code=304, headers=headers)
    return FileResponse(path, media_type="application/octet-stream", headers=headers)
