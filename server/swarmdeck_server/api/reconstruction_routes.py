"""Read-only, operator-published reconstruction assets; training is a separate process."""

import os
from pathlib import Path
import struct
from fastapi import Request, Response
from fastapi.responses import FileResponse

MAX_BYTES = 16 + 2_000_000 * 56


async def get_gaussians(request: Request):
    root = os.environ.get("SWARMDECK_RECONSTRUCTION_DIR")
    # Published models are world-aligned. Local clouds can belong to disconnected maps.
    if not root or request.query_params.get("robot_id"):
        return Response(status_code=404)
    path = Path(root) / "global.swgs"
    try:
        stat = path.stat()
        if stat.st_size > MAX_BYTES or stat.st_size < 16:
            return Response(status_code=422)
        with path.open("rb") as f:
            magic, version, count, _ = struct.unpack("<4sIII", f.read(16))
        if magic != b"SWGS" or version != 1 or stat.st_size != 16 + count * 56:
            return Response(status_code=422)
    except FileNotFoundError:
        return Response(status_code=404)
    etag = f'"{stat.st_ino:x}-{stat.st_mtime_ns:x}-{stat.st_size:x}"'
    headers = {
        "ETag": etag,
        "Cache-Control": "no-cache",
        "X-Reconstruction-Frame": "world",
    }
    if request.headers.get("if-none-match") == etag:
        return Response(status_code=304, headers=headers)
    return FileResponse(path, media_type="application/octet-stream", headers=headers)
