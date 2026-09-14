"""Raster geometry stays paired with pixels while the map advances."""

import asyncio
from io import BytesIO

import numpy as np
import pytest
from PIL import Image

from swarmdeck_server.api import map_routes
from swarmdeck_server.mapsvc import output
from swarmdeck_server.mapsvc.service import GridMeta, MapService


def test_map_publications_capture_transform_provenance():
    service = MapService()
    meta = GridMeta(0.1, 4, 3, -2.0, -1.0)
    cells = np.zeros((meta.height, meta.width), dtype=np.int8)
    service.ingest("r", meta, cells)
    service.transforms["r"] = (2.0, 3.0, 0.4)
    service._snapshots.publish(meta, cells, dict(service.transforms))
    global_snapshot = service.map_snapshot()
    local = output.local_png_snapshot(service, "r")
    service.transforms["r"] = (20.0, 30.0, 1.4)
    assert global_snapshot.transforms["r"] == pytest.approx((2.0, 3.0, 0.4))
    assert local is not None
    assert local[1]["transforms"]["r"] == pytest.approx(
        {"x": 2.0, "y": 3.0, "yaw": 0.4}
    )


def test_global_grid_transform_snapshot_is_atomic_and_composed_into_world(monkeypatch):
    service = MapService(resolution=0.1, size_m=4.0)
    meta = GridMeta(0.1, 2, 2, 0.0, 0.0)
    cells = np.zeros((2, 2), dtype=np.int8)
    service.merge_mode = "cslam"
    service.reference = "reference"
    service.transform_priors["reference"] = (10.0, -2.0, np.pi / 2)
    service.global_grid = (meta, cells)
    service.global_grid_transforms = {"r": (2.0, 1.0, 0.25)}
    original_warp = service._warp

    def update_during_warp(*args, **kwargs):
        # A second upload arriving while this raster is rendered belongs to the
        # next publication and must not relabel the first raster.
        service.global_grid_transforms = {"r": (99.0, 99.0, 1.5)}
        return original_warp(*args, **kwargs)

    monkeypatch.setattr(service, "_warp", update_during_warp)
    service._remerge()

    pose = service.map_snapshot().transforms["r"]
    assert pose == pytest.approx((9.0, 0.0, np.pi / 2 + 0.25))


@pytest.mark.parametrize("kind", ["local", "optimized", "global"])
def test_png_headers_describe_captured_grid_not_newer_map(monkeypatch, kind):
    service = MapService()
    old = GridMeta(0.1, 4, 3, -2.0, -1.0)
    new = GridMeta(0.05, 9, 8, -3.0, -4.0)
    old_cells = np.zeros((old.height, old.width), dtype=np.int8)
    new_cells = np.zeros((new.height, new.width), dtype=np.int8)
    service.ingest("r", old, old_cells)
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(
        map_routes, "_optimized", {"robot:r": (old, old_cells, ("r",), None)}
    )
    captured = []
    encode = output.grid_png

    def update_during_encoding(meta, cells):
        captured.append(meta)
        service.ingest("r", new, new_cells)
        map_routes._optimized["robot:r"] = (new, new_cells, ("r",), None)
        return encode(meta, cells)

    monkeypatch.setattr(output, "grid_png", update_during_encoding)
    if kind == "local":
        response = asyncio.run(map_routes.get_local_map("r"))
    elif kind == "optimized":
        response = asyncio.run(map_routes.get_optimized_map("robot:r"))
    else:
        response = asyncio.run(map_routes.get_map())
    meta = captured[0]
    assert Image.open(BytesIO(response.body)).size == (meta.width, meta.height)
    assert int(response.headers["X-Map-Width"]) == meta.width
    assert int(response.headers["X-Map-Height"]) == meta.height
    assert float(response.headers["X-Map-Resolution"]) == meta.resolution
    assert float(response.headers["X-Map-Origin-X"]) == meta.origin_x
    assert float(response.headers["X-Map-Origin-Y"]) == meta.origin_y
    assert service.local_info("r")["width"] == new.width
