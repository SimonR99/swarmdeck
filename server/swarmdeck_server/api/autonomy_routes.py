"""Replicate onboard map artifacts without running collaborative SLAM centrally."""

import asyncio
from functools import lru_cache
import json
import logging
import os
from pathlib import Path

from fastapi import APIRouter, Request, Response
from fastapi.responses import JSONResponse

from autonomy.replication import (
    MAX_CHUNK_BYTES,
    MAX_MANIFEST_BYTES,
    MissingChunks,
    ReplicaStore,
    RevisionConflict,
    chunk_hash,
)

router = APIRouter(prefix="/api/autonomy", tags=["autonomy"])
log = logging.getLogger(__name__)


@lru_cache(maxsize=1)
def store():
    root = os.environ.get("SWARMDECK_REPLICA_DIR")
    if not root:
        root = Path(__file__).resolve().parents[3] / "sessions" / "replicas"
    replicas = ReplicaStore(
        root,
        max_bytes=int(os.environ.get("SWARMDECK_REPLICA_MAX_BYTES", 1024**3)),
        retention_s=float(os.environ.get("SWARMDECK_REPLICA_RETENTION_S", 3600)),
    )
    if os.environ.get("SWARMDECK_REPLICA_DISCARD_HISTORY", "").lower() in {
        "1",
        "true",
        "yes",
    }:
        # The simulation overlay sets this: each reset starts this server with
        # a new mission, and the maps of earlier simulated missions are never
        # read again. Real maps sharing the store are kept by naming them.
        keep = {
            value.strip()
            for value in os.environ.get("SWARMDECK_REPLICA_KEEP_SESSIONS", "").split(
                ","
            )
            if value.strip()
        }
        mission = os.environ.get("SWARMDECK_MISSION_ID", "").strip()
        if not mission:
            # Without the live mission's name there is nothing to tell it from
            # history, and a restart would discard the map being built.
            log.warning("Replica history kept: SWARMDECK_MISSION_ID is not set")
        else:
            discarded = replicas.discard_history(keep | {mission})
            log.warning(
                "Discarded %d earlier mission(s), %d bytes, from the replica store",
                len(discarded["sessions"]),
                discarded["bytes"],
            )
    return replicas


async def bounded_body(request, limit):
    chunks, size = [], 0
    async for data in request.stream():
        size += len(data)
        if size > limit:
            raise OverflowError("Upload too large")
        chunks.append(data)
    return b"".join(chunks)


@router.get("/replicas")
async def replicas():
    return {"version": 1, "replicas": await asyncio.to_thread(store().index)}


@router.get("/replicas/{robot_id}/{session_id}")
async def manifest(robot_id: str, session_id: str):
    try:
        result = await asyncio.to_thread(store().get, robot_id, session_id)
        return JSONResponse(result) if result else Response(status_code=404)
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


@router.post("/replicas")
async def publish(request: Request):
    try:
        envelope = json.loads(await bounded_body(request, MAX_MANIFEST_BYTES))
        from .map_routes import retire_robot_epoch, robot_epoch_lock

        async with robot_epoch_lock(envelope["robot_id"]):
            previous_epoch = await asyncio.to_thread(
                store().map_epoch, envelope["robot_id"], envelope["session_id"]
            )
            changed = await asyncio.to_thread(store().publish, envelope)
            if (
                changed
                and previous_epoch is not None
                and envelope["map_epoch"] > previous_epoch
                and envelope["session_id"] == os.environ.get("SWARMDECK_MISSION_ID")
            ):
                await retire_robot_epoch(
                    envelope["robot_id"], envelope["session_id"], envelope["map_epoch"]
                )
        return {"ok": True, "changed": changed, "revision": envelope["revision"]}
    except MissingChunks as exc:
        return JSONResponse({"error": str(exc), "missing": exc.hashes}, status_code=409)
    except RevisionConflict as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)
    except OverflowError as exc:
        return JSONResponse({"error": str(exc)}, status_code=413)
    except (ValueError, KeyError, TypeError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


@router.api_route("/chunks/{digest}", methods=["GET", "HEAD"])
async def chunk(digest: str, request: Request):
    try:
        chunk_hash(digest)
        if request.method == "HEAD":
            exists = await asyncio.to_thread(store().has_chunk, digest)
            return Response(status_code=200 if exists else 404)
        data = await asyncio.to_thread(store().read_chunk, digest)
        return Response(
            data,
            media_type="application/octet-stream",
            headers={
                "ETag": f'"{digest}"',
                "Cache-Control": "public, max-age=31536000, immutable",
            },
        )
    except FileNotFoundError:
        return Response(status_code=404)
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


@router.put("/chunks/{digest}")
async def put_chunk(digest: str, request: Request):
    try:
        chunk_hash(digest)
        body = await bounded_body(request, MAX_CHUNK_BYTES)
        changed = await asyncio.to_thread(store().put_chunk, digest, body)
        return {"ok": True, "changed": changed}
    except OverflowError as exc:
        # A rejected chunk stops that robot's replica from advancing, and the
        # peer records only the status code; say why on the receiving side.
        log.warning("Rejected replica chunk %s: %s", digest, exc)
        return JSONResponse({"error": str(exc)}, status_code=413)
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
