import base64
import zlib

import numpy as np

from swarmdeck_server.mapsvc.network_grid import NO_DATA, NetworkGridAccumulator
from swarmdeck_server.mapsvc.service import MapService


def test_network_grid_averages_samples_and_leaves_unknown_transparent():
    grid = NetworkGridAccumulator(0.0, 0.0, resolution=0.5, size_m=4.0, radius_m=0.6)
    assert grid.integrate(0.0, 0.0, 20.0)
    assert grid.integrate(0.0, 0.0, 80.0)

    quality = grid.quality_grid()
    gx = int((0.0 - grid.meta.origin_x) / grid.meta.resolution)
    gy = int((0.0 - grid.meta.origin_y) / grid.meta.resolution)
    assert quality[gy, gx] == 50
    assert np.count_nonzero(quality == NO_DATA) > 0


def test_network_grid_expands_without_moving_existing_samples():
    grid = NetworkGridAccumulator(0.0, 0.0, resolution=0.5, size_m=2.0, radius_m=0.6)
    grid.integrate(0.0, 0.0, 35.0)
    grid.integrate(4.0, -3.0, 90.0)
    quality = grid.quality_grid()

    def at(x, y):
        gx = int((x - grid.meta.origin_x) / grid.meta.resolution)
        gy = int((y - grid.meta.origin_y) / grid.meta.resolution)
        return quality[gy, gx]

    assert at(0.0, 0.0) == 35
    assert at(4.0, -3.0) == 90


def test_network_grid_rejects_invalid_samples():
    grid = NetworkGridAccumulator(0.0, 0.0)
    assert not grid.integrate(0.0, 0.0, -1.0)
    assert not grid.integrate(float("nan"), 0.0, 50.0)
    assert grid.revision == 0


def test_map_service_emits_dirty_patches_and_full_snapshots():
    service = MapService(resolution=0.05, size_m=8.0)
    assert service.ingest_network_sample("r0", 1.0, -2.0, 64.0)

    snapshot = service.network_snapshot("r0")
    assert snapshot is not None
    assert snapshot["type"] == "network_patch"
    assert snapshot["robot_id"] == "r0"
    assert snapshot["w"] == snapshot["width"]
    raw = zlib.decompress(base64.b64decode(snapshot["data"]))
    values = np.frombuffer(raw, dtype=np.uint8)
    assert 64 in values
    assert NO_DATA in values

    first = service.take_network_patch("r0")
    assert first is not None
    assert service.take_network_patch("r0") is None
    service.ingest_network_sample("r0", 1.0, -2.0, 20.0)
    changed = service.take_network_patch("r0")
    assert changed is not None
    assert changed["w"] < changed["width"]


def test_map_service_network_grid_follows_map_reset():
    service = MapService(size_m=8.0)
    assert not service.ingest_network_sample("", 0.0, 0.0, 50.0)
    assert not service.ingest_network_sample("r0", float("nan"), 0.0, 50.0)
    assert service.network_robot_ids() == []

    service.ingest_network_sample("r0", 0.0, 0.0, 50.0)
    service.reset_robot("r0")
    assert service.network_snapshot("r0") is None


def test_network_grid_adapts_to_changed_quality_when_stationary():
    grid = NetworkGridAccumulator(0.0, 0.0, resolution=0.5, size_m=4.0, radius_m=0.6)
    # Simulate stationary robot receiving 100 samples at 90% quality
    for _ in range(100):
        grid.integrate(0.0, 0.0, 90.0)

    gx = int((0.0 - grid.meta.origin_x) / grid.meta.resolution)
    gy = int((0.0 - grid.meta.origin_y) / grid.meta.resolution)
    assert grid.quality_grid()[gy, gx] == 90

    # Signal degrades to 20%; thanks to weight clamping, the grid converges quickly
    for _ in range(25):
        grid.integrate(0.0, 0.0, 20.0)

    # Within 25 samples it should have updated significantly towards 20%
    assert grid.quality_grid()[gy, gx] <= 35
