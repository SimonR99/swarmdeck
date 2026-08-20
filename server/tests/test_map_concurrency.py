"""Regression tests for map publication while merge work runs off-thread."""

import asyncio

import numpy as np

from swarmdeck_server.mapsvc.grid_meta import GridMeta
from swarmdeck_server.mapsvc.service import MapService


def test_extent_expansion_publishes_coherent_snapshot_to_readers():
    async def exercise() -> None:
        service = MapService(resolution=0.1, size_m=2.0)
        service.set_transform("r0", 0.0, 0.0, 0.0)
        service.set_transform("r1", 1.5, 0.0, 0.0)
        cells = np.zeros((20, 20), dtype=np.int8)
        cells[8:12, 8:12] = 100
        service.ingest("r0", GridMeta(0.1, 20, 20, -1.0, -1.0), cells)

        async def merge_in_worker() -> None:
            await asyncio.to_thread(
                service.ingest,
                "r1",
                GridMeta(0.1, 20, 20, -1.0, -1.0),
                cells,
            )

        writer = asyncio.create_task(merge_in_worker())
        observations = 0
        while not writer.done():
            snapshot = service.map_snapshot()
            assert snapshot.merged.shape == (
                snapshot.meta.height,
                snapshot.meta.width,
            )
            assert snapshot.patch_prev.shape == snapshot.merged.shape
            assert snapshot.merged.flags.writeable is False
            assert snapshot.patch_prev.flags.writeable is False
            # These readers used to race extent expansion: PNG generation and
            # patch metadata must each come from one published generation.
            assert service.as_png().startswith(b"\x89PNG")
            status = service.status()
            assert status["map"]["width"] == snapshot.meta.width or status["map"]["width"] >= 20
            observations += 1
            await asyncio.sleep(0)
        await writer
        assert observations > 0

        final = service.map_snapshot()
        assert final.merged.shape == final.patch_prev.shape
        assert final.merged.shape == (final.meta.height, final.meta.width)
        patch = service.take_patch()
        assert patch is not None
        assert patch["width"] == final.meta.width
        assert patch["height"] == final.meta.height
        assert patch["origin"] == {
            "x": final.meta.origin_x,
            "y": final.meta.origin_y,
        }

    asyncio.run(exercise())
