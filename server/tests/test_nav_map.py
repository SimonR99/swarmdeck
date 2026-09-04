"""Collaborative occupancy warped into a robot frame for Nav2."""

from __future__ import annotations

import math

import numpy as np
import pytest
from fastapi.testclient import TestClient

from swarmdeck_server.api.app import app, load_config, map_service
from swarmdeck_server.mapsvc.grid_meta import GridMeta
from swarmdeck_server.mapsvc.nav_map import OCCUPIED, UNKNOWN, warp_to_robot_frame


@pytest.fixture(autouse=True)
def _reset_maps():
    load_config()
    yield
    load_config()


def _box_grid() -> tuple[GridMeta, np.ndarray]:
    n = 20
    cells = np.full((n, n), UNKNOWN, dtype=np.int8)
    cells[8:12, 8:12] = OCCUPIED
    return GridMeta(0.1, n, n, -1.0, -1.0), cells


def _centers(meta: GridMeta, cells: np.ndarray) -> np.ndarray:
    rows, cols = np.nonzero(cells == OCCUPIED)
    xs = meta.origin_x + (cols + 0.5) * meta.resolution
    ys = meta.origin_y + (rows + 0.5) * meta.resolution
    return np.stack([xs, ys], axis=1)


def test_identity_warp_keeps_occupied_cells_in_place():
    meta, cells = _box_grid()
    out_meta, out = warp_to_robot_frame(meta, cells, (0.0, 0.0, 0.0), padding_m=0.2)
    src = _centers(meta, cells)
    dst = _centers(out_meta, out)
    assert len(dst) >= len(src) * 0.8
    for point in src:
        assert np.min(np.linalg.norm(dst - point, axis=1)) < meta.resolution


def test_rotated_warp_round_trips_occupied_cells_to_world():
    meta, cells = _box_grid()
    yaw = math.pi / 2
    tx, ty = 1.5, -0.4
    out_meta, out = warp_to_robot_frame(meta, cells, (tx, ty, yaw), padding_m=0.2)
    src = _centers(meta, cells)
    dst = _centers(out_meta, out)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world = np.stack(
        [
            tx + dst[:, 0] * cosine - dst[:, 1] * sine,
            ty + dst[:, 0] * sine + dst[:, 1] * cosine,
        ],
        axis=1,
    )
    for point in src:
        assert np.min(np.linalg.norm(world - point, axis=1)) < 2 * meta.resolution


def _merged_two_robots() -> None:
    map_service.set_mode("graph")
    meta, cells = _box_grid()
    update = {
        "graphs": {
            "robot_0": {
                "keyframes": 8,
                "in_common_frame": True,
                "inter_robot": ["robot_1"],
            },
            "robot_1": {
                "keyframes": 7,
                "in_common_frame": True,
                "inter_robot": ["robot_0"],
            },
        },
        "origins": {
            "robot_0": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "component-0"},
            "robot_1": {"x": 1.0, "y": 0.0, "yaw": 0.0, "frame": "component-0"},
        },
        "common_poses": {
            "robot_0": {"x": 0.2, "y": 0.0, "yaw": 0.0},
            "robot_1": {"x": 1.2, "y": 0.0, "yaw": 0.0},
        },
    }
    map_service.apply_slam_update(update)
    map_service.set_global_grid(meta, cells)


def test_nav_map_is_404_until_the_robot_has_merged():
    with TestClient(app) as client:
        map_service.set_mode("graph")
        missing = client.get("/api/map/nav/robot_0")
        assert missing.status_code == 404
        _merged_two_robots()
        ok = client.get("/api/map/nav/robot_0")
        assert ok.status_code == 200
        assert ok.headers["X-Map-Width"]
        replay = client.get(
            "/api/map/nav/robot_0", headers={"If-None-Match": ok.headers["ETag"]}
        )
        assert replay.status_code == 304
        outsider = client.get("/api/map/nav/robot_9")
        assert outsider.status_code == 404


def test_nav_map_client_returns_a_grid_nav2_can_load():
    from adapters.map_downlink import NavMapClient, apply_to_occupancy_grid

    class _Grid:
        def __init__(self) -> None:
            self.header = type("H", (), {"frame_id": "", "stamp": None})()
            self.info = type(
                "I",
                (),
                {
                    "resolution": 0.0,
                    "width": 0,
                    "height": 0,
                    "origin": type(
                        "O",
                        (),
                        {
                            "position": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                            "orientation": type(
                                "Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
                            )(),
                        },
                    )(),
                },
            )()
            self.data = []

    with TestClient(app) as client:
        _merged_two_robots()

        class _Client(NavMapClient):
            def poll(self):  # noqa: ANN001
                # Drive the real HTTP path through TestClient rather than urllib.
                response = client.get(f"/api/map/nav/{self.robot_id}")
                if response.status_code != 200:
                    return None
                from adapters.map_downlink import DownloadedMap
                import zlib

                cells = np.frombuffer(zlib.decompress(response.content), dtype=np.int8)
                width = int(response.headers["X-Map-Width"])
                height = int(response.headers["X-Map-Height"])
                return DownloadedMap(
                    seq=int(response.headers["X-Map-Seq"]),
                    resolution=float(response.headers["X-Map-Resolution"]),
                    width=width,
                    height=height,
                    origin_x=float(response.headers["X-Map-Origin-X"]),
                    origin_y=float(response.headers["X-Map-Origin-Y"]),
                    cells=cells.reshape(height, width),
                )

        downloaded = _Client("http://unused", "robot_1").poll()
        assert downloaded is not None
        assert int((downloaded.cells == OCCUPIED).sum()) > 0
        grid = _Grid()
        apply_to_occupancy_grid(grid, downloaded, "robot_1/map_frame")
        assert grid.header.frame_id == "robot_1/map_frame"
        assert len(grid.data) == downloaded.width * downloaded.height
        assert 100 in grid.data


def test_nav_grid_seq_and_cache():
    assert map_service.nav_grid_seq("robot_0") is None
    _merged_two_robots()
    seq = map_service.nav_grid_seq("robot_0")
    assert seq is not None and seq >= 0

    # First call computes and caches
    res1 = map_service.nav_grid("robot_0")
    assert res1 is not None
    meta1, cells1, seq1 = res1
    assert seq1 == seq

    # Second call hits cache (same cells object)
    res2 = map_service.nav_grid("robot_0")
    assert res2 is not None
    meta2, cells2, seq2 = res2
    assert cells1 is cells2

    # Reset clears cache and state
    map_service.reset()
    assert map_service.nav_grid_seq("robot_0") is None
    assert map_service.nav_grid("robot_0") is None

