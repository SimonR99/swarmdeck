"""Display history and incremental transport, independent of navigation inputs."""

import asyncio
import json
import zlib
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from starlette.requests import Request

from swarmdeck_server.mapsvc.cloud_map import CloudMap
from swarmdeck_server.mapsvc.cloud_delivery import CloudDelivery
from swarmdeck_server.mapsvc.output import merged_cloud
from swarmdeck_server.mapsvc.service import MapService


def test_history_preserves_rooms_and_color_without_changing_registration_cloud():
    service = MapService()
    first = np.array([[1, 2, 3]], np.float32)
    color = np.array([[240, 10, 20]], np.uint8)
    service.set_cloud("r", first, rgb=color, register=False)
    service.set_cloud("r", np.array([[9, 2, 3]], np.float32), register=False)
    service.set_cloud("r", first, register=False)
    first[:] = 100
    color[:] = 0
    xyz, _, _, rgb = merged_cloud(service, "r", include_rgb=True, accumulated=True)
    assert sorted(xyz[:, 0]) == [1, 9]
    assert rgb[np.argmin(xyz[:, 0])].tolist() == [240, 10, 20]
    assert service.robot_clouds["r"].tolist() == [[1, 2, 3]]
    epoch = service.cloud_epoch
    service.set_cloud("r", np.array([[1.01, 2, 3]], np.float32), register=False)
    assert service.cloud_epoch == epoch


def test_later_camera_color_upgrades_voxel_but_does_not_move_it():
    cloud = CloudMap.empty().add(np.array([[1, 2, 3]], np.float32))
    colored = cloud.add(
        np.array([[1.01, 2, 3]], np.float32), np.array([[255, 0, 0]], np.uint8)
    )
    assert colored.points.tolist() == [[1, 2, 3]]
    assert colored.colors.tolist() == [[255, 0, 0]]
    assert colored.add(np.array([[1.02, 2, 3]], np.float32)) is colored


def test_capacity_coarsens_without_losing_distant_rooms():
    points = np.column_stack(
        (np.arange(100) * 0.1, np.zeros(100), np.zeros(100))
    ).astype(np.float32)
    cloud = CloudMap.empty().add(points, max_points=10)
    assert len(cloud.points) <= 10
    assert cloud.voxel_size > 0.1
    assert np.ptp(cloud.points[:, 0]) > 8


def test_targeted_and_fleet_reset_discard_history():
    service = MapService()
    for rid in ("a", "b"):
        service.set_cloud(rid, np.array([[1, 2, 3]], np.float32), register=False)
    service.reset_robot("a")
    assert set(service.cloud_maps) == {"b"}
    service.reset_robot()
    assert not service.cloud_maps


def test_display_history_uses_current_frame_not_old_transformed_points():
    service = MapService()
    service.set_cloud("r", np.array([[1, 0, 2]], np.float32), register=False)
    service.transforms["r"] = (10, 20, np.pi / 2)
    service.cloud_z_offsets["r"] = 3
    world = merged_cloud(service, accumulated=True)[0]
    np.testing.assert_allclose(world, [[10, 21, 5]])
    np.testing.assert_allclose(
        merged_cloud(service, "r", accumulated=True)[0], [[1, 0, 2]]
    )


def request(query="", etag=""):
    return Request(
        {
            "type": "http",
            "query_string": query.encode(),
            "headers": [(b"if-none-match", etag.encode())] if etag else [],
        }
    )


def test_manifests_reuse_untouched_tiles_and_empty_scan_preserves_map(monkeypatch):
    from swarmdeck_server.api import map_routes
    from swarmdeck_server.mapsvc import graph_bridge

    service = MapService()
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(graph_bridge, "SLAM_URL", "")
    service.set_cloud("r", np.array([[1, 2, 3], [9, 2, 3]], np.float32), register=False)

    async def run():
        response = await map_routes.get_cloud(request("robot_id=r&manifest=1"))
        before = json.loads(response.body)
        assert len(before["chunks"]) == 2
        assert (
            await map_routes.get_cloud(
                request("robot_id=r&manifest=1", response.headers["etag"])
            )
        ).status_code == 304
        service.set_cloud("r", np.array([[17, 2, 3]], np.float32), register=False)
        after_response = await map_routes.get_cloud(request("robot_id=r&manifest=1"))
        after = json.loads(after_response.body)
        assert (
            len(
                {c["id"] for c in before["chunks"]} & {c["id"] for c in after["chunks"]}
            )
            == 2
        )
        points = []
        for chunk in after["chunks"]:
            part = await map_routes.get_cloud(request("chunk=" + chunk["id"]))
            assert part.status_code == 200
            assert "immutable" in part.headers["cache-control"]
            points.extend(
                np.frombuffer(
                    zlib.decompress(part.body), "<f4", count=chunk["points"] * 3
                ).reshape(-1, 3)
            )
        assert sorted(float(p[0]) for p in points) == [1, 9, 17]
        service.set_cloud("r", np.empty((0, 3), np.float32), register=False)
        assert (
            await map_routes.get_cloud(
                request("robot_id=r&manifest=1", after_response.headers["etag"])
            )
        ).status_code == 304
        service.reset_robot("r")
        empty = await map_routes.get_cloud(
            request("robot_id=r&manifest=1", after_response.headers["etag"])
        )
        assert empty.status_code == 200
        assert json.loads(empty.body)["chunks"] == []

    asyncio.run(run())


def test_cache_builds_once_for_concurrent_viewers_and_invalidates_revision():
    cache = CloudDelivery()
    builds = []

    def build():
        builds.append(1)
        return b"payload", {}

    with ThreadPoolExecutor(4) as pool:
        products = list(pool.map(lambda _: cache.prepare("r", 1, build), range(8)))
    assert len(builds) == 1
    assert all(p is products[0] for p in products)
    cache.prepare("r", 2, build)
    assert len(builds) == 2


def test_chunk_eviction_is_bounded_and_manifest_can_repopulate():
    cache = CloudDelivery(max_bytes=128)
    headers = {
        "X-Cloud-Points": "1",
        "X-Cloud-Scale": "1",
        "X-Cloud-Format": "xyz32",
        "X-Cloud-RGB": "0",
    }
    first = None
    for index in range(30):
        body = zlib.compress(
            np.array([[index * 4, 2, 3]], "<f4").tobytes() + bytes([0])
        )
        product = cache.prepare("r", index, lambda: (body, headers))
        manifest = cache.manifest(product)
        if first is None:
            first = product
        assert cache.chunk_bytes <= 128
    manifest = cache.manifest(first)
    assert cache.chunk(manifest["chunks"][0]["id"]) is not None
