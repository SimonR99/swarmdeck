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
        self.cached: DownloadedMap | None = None

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
            if exc.code == 304:
                self.last_error = ""
                return self.cached
            if exc.code == 404:
                self.last_error = ""
                self.cached = None
                return None
            self.last_error = str(exc)
            return None
        except (
            urllib.error.URLError,
            TimeoutError,
            OSError,
            ValueError,
            zlib.error,
            KeyError,
        ) as exc:
            self.last_error = str(exc)
            return None
        cells = np.frombuffer(raw, dtype=np.int8)
        if cells.size != width * height:
            self.last_error = "nav map size mismatch"
            return None
        self.seq = seq
        self.last_error = ""
        self.cached = DownloadedMap(
            seq=seq,
            resolution=resolution,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            cells=cells.reshape(height, width),
        )
        return self.cached


def clear_robot_disc(
    cells: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x: float,
    y: float,
    radius_m: float,
) -> None:
    """Mark a disc around the live pose free so Nav2's start is not lethal.

    Collaborative occupancy is rendered from optimized keyframes. Live TF can
    sit on a rasterized wall by a cell or two, and inflation then makes the
    whole footprint lethal. The map the operator sees is unchanged; only the
    OccupancyGrid handed to Nav2 is carved.
    """
    if radius_m <= 0.0 or resolution <= 0.0:
        return
    height, width = cells.shape
    col = (x - origin_x) / resolution
    row = (y - origin_y) / resolution
    rad = radius_m / resolution
    c0 = max(0, int(np.floor(col - rad)))
    c1 = min(width, int(np.ceil(col + rad)) + 1)
    r0 = max(0, int(np.floor(row - rad)))
    r1 = min(height, int(np.ceil(row + rad)) + 1)
    if c1 <= c0 or r1 <= r0:
        return
    cols = np.arange(c0, c1, dtype=np.float64)
    rows = np.arange(r0, r1, dtype=np.float64)
    cc, rr = np.meshgrid(cols, rows)
    inside = (cc + 0.5 - col) ** 2 + (rr + 0.5 - row) ** 2 <= rad**2
    cells[r0:r1, c0:c1][inside] = np.int8(0)


def apply_to_occupancy_grid(
    grid,
    downloaded: DownloadedMap,
    frame_id: str,
    *,
    pose_xy: tuple[float, float] | None = None,
    clear_radius_m: float = 0.0,
) -> None:
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
    cells = np.array(downloaded.cells, dtype=np.int8, copy=True)
    if pose_xy is not None and clear_radius_m > 0.0:
        clear_robot_disc(
            cells,
            origin_x=downloaded.origin_x,
            origin_y=downloaded.origin_y,
            resolution=downloaded.resolution,
            x=float(pose_xy[0]),
            y=float(pose_xy[1]),
            radius_m=float(clear_radius_m),
        )
    grid.data = cells.reshape(-1).tolist()
