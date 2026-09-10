"""Read-only, operator-published reconstruction assets; training is a separate process."""

import asyncio
import os
from pathlib import Path
import hashlib
import json
from functools import lru_cache
from concurrent.futures import ThreadPoolExecutor
import re
import struct
import threading
from fastapi import Request, Response
from fastapi.responses import FileResponse, JSONResponse

MAX_BYTES = 16 + 2_000_000 * 56
_SHA256 = re.compile(r"[a-f0-9]{64}\Z")
_MAX_SOURCE_TEXT = 256
_MAX_POINTER_BYTES = 64 * 1024
_INTEGRITY_CACHE_LIMIT = 32


# At most one active validation and eight queued reads. The worker starts lazily;
# cached metadata requests do not rehash unchanged immutable artifacts.
_disk_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="reconstruction")
_disk_slots = threading.BoundedSemaphore(9)


class _ArtifactError(ValueError):
    """The selected artifact is not a bounded SWGS model."""


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


def _stat_key(path: Path, stat: os.stat_result) -> tuple[str, int, int, int, int, int]:
    return (
        str(path),
        stat.st_dev,
        stat.st_ino,
        stat.st_size,
        stat.st_mtime_ns,
        stat.st_ctime_ns,
    )


@lru_cache(maxsize=_INTEGRITY_CACHE_LIMIT)
def _digest_for_key(key: tuple[str, int, int, int, int, int]) -> str:
    return _file_digest(Path(key[0]))


def _cached_digest(path: Path, stat: os.stat_result) -> str:
    key = _stat_key(path, stat)
    digest = _digest_for_key(key)
    try:
        current = path.stat()
    except OSError as exc:
        raise _PointerError(
            "reconstruction artifact changed during validation"
        ) from exc
    if _stat_key(path, current) != key:
        _digest_for_key.cache_clear()
        raise _PointerError("reconstruction artifact changed during validation")
    return digest


def _source_metadata(pointer: dict) -> dict:
    """Validate the bounded source identity emitted by the job runner."""

    source = pointer.get("source")
    if source is None:
        if pointer.get("schema_version", 1) >= 2:
            raise _PointerError("reconstruction source metadata is missing")
        return {}
    if not isinstance(source, dict):
        raise _PointerError("reconstruction source metadata is invalid")
    text_fields = (
        "capture_id",
        "robot_id",
        "session_id",
        "submap_id",
        "calibration_version",
        "optical_frame",
        "frame_id",
        "capture_frame",
    )
    frame = source.get("frame", "world")
    if not isinstance(frame, str) or frame not in {"world", "component", "local"}:
        raise _PointerError("reconstruction source frame is invalid")
    for field in text_fields:
        value = source.get(field, "")
        if (
            not isinstance(value, str)
            or len(value) > _MAX_SOURCE_TEXT
            or "\r" in value
            or "\n" in value
        ):
            raise _PointerError(f"reconstruction source {field} is invalid")
    for field in ("frame_count", "frame_bytes"):
        value = source.get(field, 0)
        if type(value) is not int or value < 0:
            raise _PointerError(f"reconstruction source {field} is invalid")
    frame_digest = source.get("frame_manifest_sha256", "")
    if not isinstance(frame_digest, str) or (
        frame_digest and not _SHA256.fullmatch(frame_digest)
    ):
        raise _PointerError("reconstruction frame manifest checksum is invalid")
    pose = source.get("pose_revision", {})
    if not isinstance(pose, dict):
        raise _PointerError("reconstruction pose revision is invalid")
    for field in (
        "graph_revision",
        "pose_revision",
        "geometry_revision",
        "component_id",
    ):
        value = pose.get(field, "")
        if (
            not isinstance(value, str)
            or len(value) > _MAX_SOURCE_TEXT
            or "\r" in value
            or "\n" in value
        ):
            raise _PointerError(f"reconstruction pose {field} is invalid")
    snapshot_digest = pose.get("snapshot_digest", "")
    if not isinstance(snapshot_digest, str) or (
        snapshot_digest and not _SHA256.fullmatch(snapshot_digest)
    ):
        raise _PointerError("reconstruction pose snapshot checksum is invalid")
    component_id = pose.get("component_id", "")
    if (frame == "component") != bool(component_id):
        raise _PointerError("reconstruction source frame and component disagree")
    frame_id = source.get("frame_id", "")
    if frame == "component" and frame_id and frame_id != component_id:
        raise _PointerError("reconstruction frame ID and component disagree")
    if frame == "world" and frame_id not in {"", "world"}:
        raise _PointerError("reconstruction world frame ID is inconsistent")
    if frame == "local" and frame_id in {"", "world"}:
        raise _PointerError("reconstruction local frame ID is missing or inconsistent")
    outer_pose = pointer.get("pose_revision")
    if outer_pose is not None and outer_pose != pose:
        raise _PointerError("reconstruction pose revision metadata is inconsistent")
    input_fingerprint = pointer.get("input_fingerprint", "")
    if input_fingerprint and (
        not isinstance(input_fingerprint, str)
        or not _SHA256.fullmatch(input_fingerprint)
    ):
        raise _PointerError("reconstruction input fingerprint is invalid")
    return {
        "frame": frame,
        **{field: source.get(field, "") for field in text_fields},
        "frame_count": source.get("frame_count", 0),
        "frame_bytes": source.get("frame_bytes", 0),
        "frame_manifest_sha256": frame_digest,
        "pose_revision": {
            field: pose.get(field, "")
            for field in (
                "graph_revision",
                "pose_revision",
                "geometry_revision",
                "component_id",
                "snapshot_digest",
            )
        },
    }


def _check_scope(source: dict, request: Request) -> dict:
    metadata = _source_metadata(source)
    if not metadata:
        metadata = {"frame": "world", "pose_revision": {}}
    if metadata["frame"] == "local":
        raise _PointerError(
            "robot-local reconstruction requires a verified map transform"
        )
    expected_input = request.query_params.get("input_fingerprint")
    actual_input = source.get("input_fingerprint", "")
    if expected_input is not None and expected_input != actual_input:
        raise _PointerError("reconstruction input revision does not match")
    expected_pose = request.query_params.get("pose_revision")
    actual_pose = metadata["pose_revision"].get("pose_revision", "")
    if expected_pose is not None and expected_pose != actual_pose:
        raise _PointerError("reconstruction pose revision does not match")
    requested_session = request.query_params.get("session_id")
    requested_component = request.query_params.get("component_id")
    if metadata["frame"] == "component":
        if not requested_session or not requested_component:
            raise _PointerError(
                "component reconstruction requires session_id and component_id"
            )
        if requested_session != metadata.get("session_id", ""):
            raise _PointerError("reconstruction session does not match")
        if requested_component != metadata["pose_revision"].get("component_id", ""):
            raise _PointerError("reconstruction component does not match")
    elif requested_session or requested_component:
        if requested_session and requested_session != metadata.get("session_id", ""):
            raise _PointerError("reconstruction session does not match")
        if requested_component and requested_component != metadata["pose_revision"].get(
            "component_id", ""
        ):
            raise _PointerError("reconstruction component does not match")
    return metadata


def _header_text(value: object) -> str | None:
    text = str(value)
    if "\r" in text or "\n" in text:
        return None
    try:
        text.encode("latin-1")
    except UnicodeEncodeError:
        return None
    return text


def _source(root: Path) -> tuple[Path, dict, os.stat_result, os.stat_result | None]:
    """Resolve the atomic pointer, falling back only when it is absent."""
    pointer_path = root / "global.swgs.manifest.json"
    try:
        pointer_stat = pointer_path.stat()
    except FileNotFoundError:
        path = root / "global.swgs"
        return path, {"state": "legacy", "version": "legacy"}, path.stat(), None
    if pointer_stat.st_size > _MAX_POINTER_BYTES:
        raise _PointerError("reconstruction manifest pointer is too large")
    try:
        with pointer_path.open("rb") as stream:
            raw = stream.read(_MAX_POINTER_BYTES + 1)
        if len(raw) > _MAX_POINTER_BYTES:
            raise _PointerError("reconstruction manifest pointer is too large")
        pointer = json.loads(raw)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _PointerError("reconstruction manifest pointer is invalid") from exc
    if not isinstance(pointer, dict) or not isinstance(pointer.get("artifact"), str):
        raise _PointerError("reconstruction manifest pointer has no artifact")
    schema_version = pointer.get("schema_version", 1)
    if type(schema_version) is not int or schema_version not in {1, 2}:
        raise _PointerError("reconstruction manifest schema version is invalid")
    state = pointer.get("state", "ready")
    if not isinstance(state, str) or state not in {
        "ready",
        "stale",
        "canceled",
        "failed",
    }:
        raise _PointerError("reconstruction pointer has an invalid state")
    if state in {"stale", "canceled", "failed"}:
        raise _PointerError(f"reconstruction pointer is {state}")
    for field in ("version", "job_id"):
        value = pointer.get(field)
        if value is not None and (
            not isinstance(value, str)
            or len(value) > _MAX_SOURCE_TEXT
            or "\r" in value
            or "\n" in value
        ):
            raise _PointerError(f"reconstruction pointer {field} is invalid")
    _source_metadata(pointer)
    path = Path(pointer["artifact"]).expanduser().resolve()
    root = root.resolve()
    if not _within(path, root):
        raise _PointerError("reconstruction artifact escapes its configured root")
    try:
        artifact_stat = path.stat()
    except FileNotFoundError as exc:
        raise _PointerError("reconstruction artifact from pointer is missing") from exc
    declared_size = pointer.get("artifact_size_bytes")
    if artifact_stat.st_size < 16 or artifact_stat.st_size > MAX_BYTES:
        raise _PointerError(
            "reconstruction artifact size is outside the serving budget"
        )
    if schema_version >= 2 and type(declared_size) is not int:
        raise _PointerError("reconstruction artifact size is missing")
    if declared_size is not None and (
        type(declared_size) is not int or declared_size != artifact_stat.st_size
    ):
        raise _PointerError("reconstruction artifact size does not match its pointer")
    declared_digest = pointer.get("artifact_sha256")
    if schema_version >= 2 and not isinstance(declared_digest, str):
        raise _PointerError("reconstruction artifact checksum is missing")
    if declared_digest is not None:
        if not isinstance(declared_digest, str) or not _SHA256.fullmatch(
            declared_digest
        ):
            raise _PointerError("reconstruction artifact checksum is invalid")
        if _cached_digest(path, artifact_stat) != declared_digest:
            raise _PointerError(
                "reconstruction artifact checksum does not match its pointer"
            )
    return path, pointer, artifact_stat, pointer_stat


def _artifact_headers(source: dict, stat: os.stat_result) -> dict[str, str]:
    source_meta = _source_metadata(source)
    if not source_meta:
        source_meta = {"frame": "world", "pose_revision": {}}
    version = _header_text(source.get("version", source.get("job_id", "legacy")))
    headers = {
        "ETag": (
            f'"{stat.st_ino:x}-{stat.st_mtime_ns:x}-{stat.st_ctime_ns:x}-'
            f'{stat.st_size:x}"'
        ),
        "Cache-Control": "no-cache",
        "X-Reconstruction-Frame": source_meta["frame"],
        "X-Reconstruction-State": str(source.get("state", "ready")),
        "X-Reconstruction-Version": version or "unknown",
    }
    for field, header in (
        ("job_id", "X-Reconstruction-Job"),
        ("input_fingerprint", "X-Reconstruction-Input-Fingerprint"),
    ):
        value = source.get(field)
        if value:
            safe = _header_text(value)
            if safe is not None:
                headers[header] = safe
    if source_meta.get("session_id"):
        safe = _header_text(source_meta["session_id"])
        if safe is not None:
            headers["X-Reconstruction-Session"] = safe
    pose = source_meta.get("pose_revision", {})
    if pose.get("component_id"):
        safe = _header_text(pose["component_id"])
        if safe is not None:
            headers["X-Reconstruction-Component"] = safe
    artifact_digest = source.get("artifact_sha256")
    if artifact_digest:
        safe = _header_text(artifact_digest)
        if safe is not None:
            headers["X-Reconstruction-Artifact-SHA256"] = safe
    if source_meta.get("frame_manifest_sha256"):
        safe = _header_text(source_meta["frame_manifest_sha256"])
        if safe is not None:
            headers["X-Reconstruction-Frame-Manifest"] = safe
    return headers


def _validated_source(root: Path):
    resolved = _source(root)
    path, _, stat, _ = resolved
    if not 16 <= stat.st_size <= MAX_BYTES:
        raise _ArtifactError("reconstruction artifact size is invalid")
    with path.open("rb") as stream:
        header = stream.read(16)
    if len(header) != 16:
        raise _ArtifactError("reconstruction artifact header is truncated")
    magic, version, count, _ = struct.unpack("<4sIII", header)
    if magic != b"SWGS" or version != 1 or stat.st_size != 16 + count * 56:
        raise _ArtifactError("reconstruction artifact format is invalid")
    return resolved


async def _source_async(root: Path):
    if not _disk_slots.acquire(blocking=False):
        raise _PointerError("reconstruction validation queue is full")
    try:
        future = _disk_executor.submit(_validated_source, root)
    except BaseException:
        _disk_slots.release()
        raise
    future.add_done_callback(lambda _: _disk_slots.release())
    # Request cancellation must not release a queue slot until its disk work
    # finishes. Otherwise canceled requests could accumulate unbounded work.
    return await asyncio.shield(asyncio.wrap_future(future))


async def get_gaussians(request: Request):
    root = os.environ.get("SWARMDECK_RECONSTRUCTION_DIR")
    # This endpoint serves world or explicitly selected component artifacts.
    if not root or request.query_params.get("robot_id"):
        return Response(status_code=404)
    try:
        path, source, stat, pointer_stat = await _source_async(Path(root))
        metadata = _check_scope(source, request)
    except FileNotFoundError:
        return Response(status_code=404)
    except _PointerError:
        return Response(status_code=409)
    except _ArtifactError:
        return Response(status_code=422)
    headers = _artifact_headers(source, stat)
    if pointer_stat is not None:
        headers["ETag"] = (
            f'{headers["ETag"][:-1]}-{pointer_stat.st_mtime_ns:x}-'
            f'{pointer_stat.st_ctime_ns:x}"'
        )
    if request.query_params.get("format") in {"status", "metadata"}:
        return JSONResponse(
            {
                "schema_version": source.get("schema_version", 1),
                "state": source.get("state", "ready"),
                "version": source.get("version", source.get("job_id", "legacy")),
                "job_id": source.get("job_id"),
                "input_fingerprint": source.get("input_fingerprint"),
                "artifact_size_bytes": stat.st_size,
                "artifact_sha256": source.get("artifact_sha256"),
                "source": metadata,
            },
            headers={"Cache-Control": "no-cache", "ETag": headers["ETag"]},
        )
    if request.headers.get("if-none-match") == headers["ETag"]:
        return Response(status_code=304, headers=headers)
    return FileResponse(path, media_type="application/octet-stream", headers=headers)
