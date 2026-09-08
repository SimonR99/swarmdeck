"""Raster geometry stays paired with pixels while the map advances."""

import asyncio
from io import BytesIO

import numpy as np
import pytest
from PIL import Image

from swarmdeck_server.api import map_routes
from swarmdeck_server.mapsvc import output
from swarmdeck_server.mapsvc.service import GridMeta, MapService


@pytest.mark.parametrize("kind", ["local", "optimized", "global"])
def test_png_headers_describe_captured_grid_not_newer_map(monkeypatch, kind):
    service = MapService()
    old = GridMeta(0.1, 4, 3, -2.0, -1.0)
    new = GridMeta(0.05, 9, 8, -3.0, -4.0)
    old_cells = np.zeros((old.height, old.width), dtype=np.int8)
    new_cells = np.zeros((new.height, new.width), dtype=np.int8)
    service.ingest("r", old, old_cells)
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(map_routes, "_optimized", {"robot:r": (old, old_cells, ("r",))})
    captured = []
    encode = output.grid_png

    def update_during_encoding(meta, cells):
        captured.append(meta)
        service.ingest("r", new, new_cells)
        map_routes._optimized["robot:r"] = (new, new_cells, ("r",))
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
