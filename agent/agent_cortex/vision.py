"""Cortex Multimodal Image Understanding & Storage Module.

Handles:
- Uploading and saving user-provided images (screenshots, site maps, object photos)
- Fetching robot camera frames from SwarmDeck server
- Image analysis, metadata extraction, and multimodal vision preparation for AGY
"""

from __future__ import annotations

import io
import os
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from PIL import Image

UPLOAD_DIR = Path(os.environ.get("CORTEX_UPLOAD_DIR", "/app/sessions/cortex_images"))


def ensure_upload_dir() -> Path:
    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    return UPLOAD_DIR


def save_image_bytes(data: bytes, original_name: str = "image.png") -> Tuple[str, Path, Dict[str, Any]]:
    """Save uploaded image to disk, inspect dimensions and metadata."""
    upload_dir = ensure_upload_dir()
    image_id = f"img_{int(time.time())}_{uuid.uuid4().hex[:8]}"
    ext = Path(original_name).suffix.lower() or ".png"
    if ext not in (".jpg", ".jpeg", ".png", ".webp", ".bmp"):
        ext = ".png"

    filename = f"{image_id}{ext}"
    target_path = upload_dir / filename
    target_path.write_bytes(data)

    metadata: Dict[str, Any] = {
        "image_id": image_id,
        "filename": filename,
        "path": str(target_path),
        "size_bytes": len(data),
    }

    try:
        with Image.open(io.BytesIO(data)) as img:
            metadata["width"] = img.width
            metadata["height"] = img.height
            metadata["format"] = img.format
            metadata["mode"] = img.mode
    except Exception as exc:
        metadata["error"] = f"Failed to parse image headers: {exc}"

    return image_id, target_path, metadata


def fetch_camera_snapshot(robot_id: str, server_url: str = "http://server:8080") -> Optional[Tuple[Path, Dict[str, Any]]]:
    """Fetch live camera frame from SwarmDeck server and save to disk."""
    url = f"{server_url.rstrip('/')}/api/camera/{robot_id}"
    req = urllib.request.Request(url, headers={"User-Agent": "CortexVision/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = resp.read()
            if not data:
                return None
            image_id, path, meta = save_image_bytes(data, f"{robot_id}_live.jpg")
            meta["robot_id"] = robot_id
            meta["source"] = "robot_camera"
            return path, meta
    except Exception:
        return None
