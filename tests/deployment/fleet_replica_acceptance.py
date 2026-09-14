#!/usr/bin/env python3
"""Bounded, read-only HTTP acceptance for fleet replica components.

This observer uses only the live server catalogue, aggregate component views,
and immutable chunk GETs. It does not create fixtures, write map data, call
ROS, or send motion commands.

Example::

    python3 tests/deployment/fleet_replica_acceptance.py \
      --base-url http://127.0.0.1:8000 \
      --session-id 00000000-0000-4000-8000-000000000001 \
      --deadline 60
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import struct
import sys
import time
from typing import Any, Mapping
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urljoin
from urllib.request import Request, urlopen


XYZ_MAGIC = b"SDXYZ1\x00\x00"
XYZ_ENCODING = "application/vnd.swarmdeck.xyz-f32.v1"
XYZRGBA_MAGIC = b"SDRGB1\x00\x00"
XYZRGBA_ENCODING = "application/vnd.swarmdeck.xyzrgba-f32-u8.v1"
CHUNK_FORMATS = {
    XYZ_ENCODING: (XYZ_MAGIC, 12),
    XYZRGBA_ENCODING: (XYZRGBA_MAGIC, 16),
}
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_CHUNK_BYTES = 8 * 1024 * 1024
MAX_COMPONENTS = 128
MAX_CHUNKS = 4_096
SHA256 = re.compile(r"^[0-9a-f]{64}$")


class ObserverError(RuntimeError):
    pass


class DeadlineExceeded(ObserverError):
    pass


def remaining(deadline: float) -> float:
    value = deadline - time.monotonic()
    if value <= 0:
        raise DeadlineExceeded("observer deadline exceeded")
    return value


def endpoint(base_url: str, path: str, params: Mapping[str, str] = ()) -> str:
    query = urlencode(params)
    return urljoin(base_url.rstrip("/") + "/", path.lstrip("/")) + (
        f"?{query}" if query else ""
    )


def read_response(
    url: str,
    deadline: float,
    request_timeout: float,
    maximum: int,
    accept: str,
) -> tuple[bytes, Mapping[str, str], float]:
    started = time.monotonic()
    timeout = max(0.05, min(request_timeout, remaining(deadline)))
    request = Request(url, headers={"Accept": accept})
    try:
        with urlopen(request, timeout=timeout) as response:
            content_length = response.headers.get("Content-Length")
            if content_length is not None:
                try:
                    advertised = int(content_length)
                except ValueError as exc:
                    raise ObserverError(f"invalid Content-Length from {url}") from exc
                if advertised < 0 or advertised > maximum:
                    raise ObserverError(f"response exceeds bound from {url}")
            body = bytearray()
            while True:
                remaining(deadline)
                block = response.read(min(64 * 1024, maximum + 1 - len(body)))
                if not block:
                    break
                body.extend(block)
                if len(body) > maximum:
                    raise ObserverError(f"response exceeds bound from {url}")
            if content_length is not None and int(content_length) != len(body):
                raise ObserverError(f"Content-Length mismatch from {url}")
            return bytes(body), response.headers, time.monotonic() - started
    except HTTPError as exc:
        raise ObserverError(f"HTTP {exc.code} from {url}") from exc
    except (URLError, TimeoutError, OSError) as exc:
        raise ObserverError(f"request failed for {url}: {exc}") from exc


def json_response(
    url: str, deadline: float, request_timeout: float
) -> tuple[dict[str, Any], float]:
    body, _, latency = read_response(
        url, deadline, request_timeout, MAX_JSON_BYTES, "application/json"
    )
    try:
        value = json.loads(body)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ObserverError(f"invalid JSON from {url}") from exc
    if not isinstance(value, dict):
        raise ObserverError(f"JSON object expected from {url}")
    return value, latency


def text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ObserverError(f"{field} is invalid")
    return value


def uint(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ObserverError(f"{field} is invalid")
    return value


def validate_catalogue(value: dict[str, Any]) -> list[dict[str, Any]]:
    if value.get("version") != 1 or not isinstance(value.get("components"), list):
        raise ObserverError("catalogue envelope is invalid")
    components = value["components"]
    if len(components) > MAX_COMPONENTS:
        raise ObserverError("catalogue component budget exceeded")
    for entry in components:
        if not isinstance(entry, dict):
            raise ObserverError("catalogue component is invalid")
        text(entry.get("session_id"), "component session_id")
        text(entry.get("component_id"), "component_id")
        text(entry.get("frame_id"), "component frame_id")
        if entry.get("status") not in {"ready", "syncing", "conflict"}:
            raise ObserverError("catalogue component status is invalid")
        if not isinstance(entry.get("available"), bool):
            raise ObserverError("catalogue component availability is invalid")
        robot_ids = entry.get("robot_ids")
        if not isinstance(robot_ids, list) or any(
            not isinstance(robot_id, str) or not robot_id for robot_id in robot_ids
        ):
            raise ObserverError("catalogue robot_ids are invalid")
        for field in ("source_count", "submap_count", "point_count"):
            uint(entry.get(field), f"component {field}")
        sources = entry.get("sources")
        if not isinstance(sources, list):
            raise ObserverError("catalogue component sources are invalid")
        if len(sources) != entry["source_count"]:
            raise ObserverError("catalogue source count is inconsistent")
        for source in sources:
            if not isinstance(source, dict):
                raise ObserverError("catalogue source is invalid")
            text(source.get("robot_id"), "source robot_id")
            text(source.get("session_id"), "source session_id")
            uint(source.get("revision"), "source revision")
            snapshot_id = text(source.get("snapshot_id"), "source snapshot_id")
            if not SHA256.fullmatch(snapshot_id):
                raise ObserverError("source snapshot_id is not a SHA-256 digest")
        if set(robot_ids) != {source["robot_id"] for source in sources}:
            raise ObserverError("catalogue robot_ids do not match sources")
    return components


def committed_sources(components: list[dict[str, Any]]) -> dict[tuple[str, str], dict[str, Any]]:
    sources: dict[tuple[str, str], dict[str, Any]] = {}
    for entry in components:
        if entry["status"] != "ready" or entry["available"] is not True:
            continue
        for source in entry["sources"]:
            sources[(source["robot_id"], source["session_id"])] = source
    return sources


def validate_view(
    entry: dict[str, Any], value: dict[str, Any]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    for field, expected in (
        ("version", 1),
        ("scope", "fleet"),
        ("robot_id", "fleet"),
        ("session_id", entry["session_id"]),
        ("component_id", entry["component_id"]),
        ("revision", None),
    ):
        if value.get(field) != expected:
            raise ObserverError(f"aggregate view {field} does not match catalogue")
    snapshot_id = text(value.get("snapshot_id"), "aggregate snapshot_id")
    if not SHA256.fullmatch(snapshot_id):
        raise ObserverError("aggregate snapshot_id is not a SHA-256 digest")
    sources = value.get("sources")
    if not isinstance(sources, list) or len(sources) != entry["source_count"]:
        raise ObserverError("aggregate source count does not match catalogue")
    source_keys = set()
    source_records = set()
    for source in sources:
        if not isinstance(source, dict):
            raise ObserverError("aggregate source is invalid")
        key = (text(source.get("robot_id"), "aggregate source robot_id"),
               text(source.get("session_id"), "aggregate source session_id"))
        if key in source_keys:
            raise ObserverError("aggregate source is repeated")
        source_keys.add(key)
        revision = uint(source.get("revision"), "aggregate source revision")
        snapshot_id = text(source.get("snapshot_id"), "aggregate source snapshot_id")
        if not SHA256.fullmatch(snapshot_id):
            raise ObserverError("aggregate source snapshot_id is invalid")
        source_records.add((*key, revision, snapshot_id))
    expected_records = {
        (
            source["robot_id"], source["session_id"],
            source["revision"], source["snapshot_id"]
        )
        for source in entry["sources"]
    }
    if source_records != expected_records:
        raise ObserverError("aggregate sources do not match catalogue")
    selected = value.get("selected")
    if not isinstance(selected, dict):
        raise ObserverError("aggregate selected component is missing")
    if selected.get("component_id") != entry["component_id"]:
        raise ObserverError("aggregate selected component does not match catalogue")
    if selected.get("frame_id") != entry["frame_id"]:
        raise ObserverError("aggregate frame does not match catalogue")
    if selected.get("graph_revision") is not None:
        raise ObserverError("aggregate graph revision must be null")
    submaps = selected.get("submaps")
    chunks = value.get("chunks")
    if not isinstance(submaps, list) or len(submaps) != entry["submap_count"]:
        raise ObserverError("aggregate submap count does not match catalogue")
    if not isinstance(chunks, list) or len(chunks) > MAX_CHUNKS:
        raise ObserverError("aggregate chunk list is invalid")
    refs: dict[str, dict[str, Any]] = {}
    point_count = 0
    for submap in submaps:
        if not isinstance(submap, dict) or not isinstance(submap.get("chunks"), list):
            raise ObserverError("aggregate submap chunks are invalid")
        for ref in submap["chunks"]:
            if not isinstance(ref, dict):
                raise ObserverError("aggregate chunk reference is invalid")
            digest = text(ref.get("sha256"), "aggregate chunk sha256")
            if not SHA256.fullmatch(digest):
                raise ObserverError("aggregate chunk sha256 is invalid")
            chunk_format = CHUNK_FORMATS.get(ref.get("encoding"))
            if chunk_format is None:
                raise ObserverError("aggregate chunk encoding is invalid")
            size = uint(ref.get("size_bytes"), "aggregate chunk size")
            count = uint(ref.get("point_count"), "aggregate chunk point count")
            if size != 16 + count * chunk_format[1] or size > MAX_CHUNK_BYTES:
                raise ObserverError("aggregate chunk bounds are invalid")
            previous = refs.get(digest)
            if previous is not None and previous != ref:
                raise ObserverError("aggregate chunk descriptor conflicts")
            refs[digest] = ref
            point_count += count
    view_refs = {text(ref.get("sha256"), "view chunk sha256"): ref for ref in chunks}
    if set(view_refs) != set(refs):
        raise ObserverError("aggregate chunk list does not cover selected submaps")
    if point_count != entry["point_count"]:
        raise ObserverError("aggregate point count does not match catalogue")
    return submaps, list(refs.values()), point_count


def verify_chunk(
    base_url: str,
    digest: str,
    expected: dict[str, Any],
    deadline: float,
    request_timeout: float,
) -> tuple[int, float]:
    body, headers, latency = read_response(
        endpoint(base_url, f"/api/autonomy/chunks/{digest}"),
        deadline,
        request_timeout,
        MAX_CHUNK_BYTES,
        "application/octet-stream",
    )
    if len(body) != expected["size_bytes"] or hashlib.sha256(body).hexdigest() != digest:
        raise ObserverError(f"chunk {digest} failed size or SHA-256 validation")
    chunk_format = CHUNK_FORMATS.get(expected.get("encoding"))
    if chunk_format is None:
        raise ObserverError(f"chunk {digest} has an unsupported encoding")
    magic, stride = chunk_format
    if body[:8] != magic or len(body) < 16:
        raise ObserverError(f"chunk {digest} has an invalid format header")
    count = struct.unpack_from("<Q", body, 8)[0]
    if count != expected["point_count"] or len(body) != 16 + count * stride:
        raise ObserverError(f"chunk {digest} point header does not match descriptor")
    content_length = headers.get("Content-Length")
    if content_length is not None and int(content_length) != len(body):
        raise ObserverError(f"chunk {digest} Content-Length does not match body")
    return count, latency


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-url", default=os.environ.get("SWARMDECK_SERVER_URL", "http://127.0.0.1:8000")
    )
    parser.add_argument("--session-id", default=os.environ.get("SWARMDECK_MISSION_ID"))
    parser.add_argument("--deadline", type=float, default=60.0)
    parser.add_argument("--request-timeout", type=float, default=5.0)
    parser.add_argument("--poll", type=float, default=0.5)
    parser.add_argument("--expected-robots", type=int, default=4)
    args = parser.parse_args()
    if args.deadline <= 0 or args.request_timeout <= 0 or args.poll < 0:
        parser.error("deadline/request-timeout must be positive and poll must be non-negative")
    if args.expected_robots <= 0:
        parser.error("expected-robots must be positive")
    args.base_url = args.base_url.rstrip("/")
    return args


def run(args: argparse.Namespace) -> dict[str, Any]:
    started = time.monotonic()
    deadline = started + args.deadline
    status_errors: list[dict[str, Any]] = []
    poll_count = 0
    catalogue_latency_ms: list[float] = []
    components: list[dict[str, Any]] = []
    sources: dict[tuple[str, str], dict[str, Any]] = {}
    latest_status: list[dict[str, Any]] = []
    while True:
        try:
            params = {"session_id": args.session_id} if args.session_id else {}
            catalogue, latency = json_response(
                endpoint(args.base_url, "/api/autonomy/replicas/components", params),
                deadline,
                args.request_timeout,
            )
            poll_count += 1
            catalogue_latency_ms.append(latency * 1000)
            entries = validate_catalogue(catalogue)
            latest_status = [
                {
                    "session_id": entry["session_id"],
                    "component_id": entry["component_id"],
                    "status": entry["status"],
                    "available": entry["available"],
                    "detail": entry.get("detail", ""),
                }
                for entry in entries
                if entry["status"] != "ready" or entry["available"] is not True
            ]
            sources = committed_sources(entries)
            components = [
                entry for entry in entries
                if entry["status"] == "ready" and entry["available"] is True
            ]
            if len(sources) >= args.expected_robots and components:
                break
        except (ObserverError, HTTPError, URLError, TimeoutError) as exc:
            status_errors.append({"phase": "catalogue", "error": str(exc)})
            status_errors = status_errors[-12:]
        try:
            sleep_for = remaining(deadline)
        except DeadlineExceeded:
            break
        time.sleep(min(args.poll, sleep_for))

    if len(sources) < args.expected_robots:
        status_errors.append(
            {
                "phase": "wait",
                "error": (
                    f"committed sources {len(sources)}/{args.expected_robots}; "
                    f"ready components {len(components)}"
                ),
            }
        )
    elif not components:
        status_errors.append(
            {"phase": "wait", "error": "no ready replica components before deadline"}
        )

    source_rows = [
        {"robot_id": robot, "session_id": session}
        for robot, session in sources
    ]
    source_rows.sort(key=lambda row: (row["session_id"], row["robot_id"]))
    if len(sources) < args.expected_robots or not components:
        return {
            "ok": False,
            "session_id": args.session_id,
            "elapsed_ms": round((time.monotonic() - started) * 1000, 3),
            "polls": poll_count,
            "catalogue_ms": {
                "last": round(catalogue_latency_ms[-1], 3) if catalogue_latency_ms else None,
                "max": round(max(catalogue_latency_ms), 3) if catalogue_latency_ms else None,
            },
            "source_count": len(sources),
            "sources": source_rows,
            "ready_components": [],
            "catalogue_status": latest_status,
            "status_errors": status_errors,
        }

    result_components: list[dict[str, Any]] = []
    chunk_cache: dict[str, int] = {}
    failed_components = 0
    for entry in components:
        view_started = time.monotonic()
        try:
            view, view_latency = json_response(
                endpoint(
                    args.base_url,
                    f"/api/autonomy/replicas/components/view/{entry['session_id']}",
                    {"component_id": entry["component_id"]},
                ),
                deadline,
                args.request_timeout,
            )
            submaps, refs, point_count = validate_view(entry, view)
            chunk_latencies: list[float] = []
            for ref in refs:
                digest = ref["sha256"]
                if digest in chunk_cache:
                    continue
                count, latency = verify_chunk(
                    args.base_url, digest, ref, deadline, args.request_timeout
                )
                chunk_cache[digest] = count
                chunk_latencies.append(latency * 1000)
            result_components.append(
                {
                    "session_id": entry["session_id"],
                    "component_id": entry["component_id"],
                    "robots": entry["robot_ids"],
                    "source_count": len(view["sources"]),
                    "submap_count": len(submaps),
                    "point_count": point_count,
                    "chunk_count": len(refs),
                    "view_ms": round(view_latency * 1000, 3),
                    "chunk_ms": round(sum(chunk_latencies), 3),
                    "elapsed_ms": round((time.monotonic() - view_started) * 1000, 3),
                }
            )
        except (ObserverError, HTTPError, URLError, TimeoutError) as exc:
            failed_components += 1
            status_errors.append(
                {
                    "phase": "component",
                    "session_id": entry["session_id"],
                    "component_id": entry["component_id"],
                    "error": str(exc),
                }
            )
            status_errors = status_errors[-24:]

    return {
        "ok": failed_components == 0,
        "session_id": args.session_id,
        "elapsed_ms": round((time.monotonic() - started) * 1000, 3),
        "polls": poll_count,
        "source_count": len(sources),
        "catalogue_ms": {
            "last": round(catalogue_latency_ms[-1], 3) if catalogue_latency_ms else None,
            "max": round(max(catalogue_latency_ms), 3) if catalogue_latency_ms else None,
        },
        "sources": source_rows,
        "ready_components": result_components,
        "catalogue_status": latest_status,
        "status_errors": status_errors,
    }


def main() -> int:
    args = parse_args()
    try:
        output = run(args)
    except Exception as exc:
        output = {
            "ok": False,
            "session_id": args.session_id,
            "status_errors": [{"phase": "acceptance", "error": str(exc)}],
        }
        print(json.dumps(output, separators=(",", ":"), sort_keys=True))
        return 1
    print(json.dumps(output, separators=(",", ":"), sort_keys=True))
    return 0 if output.get("ok") is True else 1


if __name__ == "__main__":
    sys.exit(main())
