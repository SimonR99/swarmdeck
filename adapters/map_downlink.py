"""Fetch the collaborative occupancy grid so Nav2 can plan on it.

The server warps the common-frame map into this robot's own map frame.
Adapters poll; a 404 means the fleet has not merged yet. The local costmap
must never load this product — only the global planner's static layer.
"""

from __future__ import annotations

import urllib.error
import urllib.request
import zlib
from dataclasses import dataclass

import numpy as np

MAX_BYTES = 64 * 1024 * 1024


@dataclass(frozen=True)
class DownloadedMap:
    seq: int
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    cells: np.ndarray  # (height, width) int8


class NavMapClient:
    """Poll ``GET /api/map/nav/<robot_id>``. Drop rather than block."""

    def __init__(self, http_url: str, robot_id: str, *, timeout_s: float = 5.0) -> None:
        self.http_url = http_url.rstrip("/")
        self.robot_id = robot_id
        self.timeout_s = timeout_s
        self.seq = -1
        self.last_error = ""

    def poll(self) -> DownloadedMap | None:
        url = f"{self.http_url}/api/map/nav/{self.robot_id}"
        headers = {}
        if self.seq >= 0:
            headers["If-None-Match"] = str(self.seq)
        try:
            with urllib.request.urlopen(
                urllib.request.Request(url, headers=headers),
                timeout=self.timeout_s,
            ) as response:
                seq = int(response.headers.get("X-Map-Seq", "0"))
                resolution = float(response.headers["X-Map-Resolution"])
                width = int(response.headers["X-Map-Width"])
                height = int(response.headers["X-Map-Height"])
                origin_x = float(response.headers["X-Map-Origin-X"])
                origin_y = float(response.headers["X-Map-Origin-Y"])
                raw = zlib.decompress(response.read(), bufsize=MAX_BYTES)
        except urllib.error.HTTPError as exc:
            if exc.code in (304, 404):
                self.last_error = ""
                return None
            self.last_error = str(exc)
            return None
        except (urllib.error.URLError, TimeoutError, OSError, ValueError, zlib.error, KeyError) as exc:
            self.last_error = str(exc)
            return None
        cells = np.frombuffer(raw, dtype=np.int8)
        if cells.size != width * height:
            self.last_error = "nav map size mismatch"
            return None
        if seq == self.seq:
            return None
        self.seq = seq
        self.last_error = ""
        return DownloadedMap(
            seq=seq,
            resolution=resolution,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            cells=cells.reshape(height, width),
        )


def apply_to_occupancy_grid(grid, downloaded: DownloadedMap, frame_id: str) -> None:
    """Fill a ROS OccupancyGrid in place. Stamp is left to the caller."""
    grid.header.frame_id = frame_id
    info = grid.info
    info.resolution = float(downloaded.resolution)
    info.width = int(downloaded.width)
    info.height = int(downloaded.height)
    info.origin.position.x = float(downloaded.origin_x)
    info.origin.position.y = float(downloaded.origin_y)
    info.origin.position.z = 0.0
    info.origin.orientation.x = 0.0
    info.origin.orientation.y = 0.0
    info.origin.orientation.z = 0.0
    info.origin.orientation.w = 1.0
    grid.data = downloaded.cells.astype(np.int8).reshape(-1).tolist()
