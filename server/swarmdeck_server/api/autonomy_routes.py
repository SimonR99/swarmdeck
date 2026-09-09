"""Replicate onboard map artifacts without running collaborative SLAM centrally."""

import asyncio
from functools import lru_cache
import json
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


@lru_cache(maxsize=1)
def store():
    root = os.environ.get("SWARMDECK_REPLICA_DIR")
    if not root:
        root = Path(__file__).resolve().parents[3] / "sessions" / "replicas"
    return ReplicaStore(root)


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
        changed = await asyncio.to_thread(store().publish, envelope)
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
        return JSONResponse({"error": str(exc)}, status_code=413)
    except ValueError as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
