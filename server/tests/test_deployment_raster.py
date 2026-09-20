"""The deployment composite rasterized for the 2D map's optimized source."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
from io import BytesIO
from unittest.mock import AsyncMock
from uuid import uuid4

import numpy as np
import pytest
from fastapi.testclient import TestClient
from PIL import Image

from autonomy.contracts import IDENTITY_SE3
from autonomy.replication import ReplicaStore
from swarmdeck_server.api import (
    autonomy_routes,
    deployment_raster,
    map_routes,
    replica_live,
    replica_views,
)
from swarmdeck_server.fleet.registry import Registry
from swarmdeck_server.mapsvc import output
from swarmdeck_server.mapsvc.grid_meta import GridMeta
from tests.test_replica_components import peer
from tests.test_replica_deployment_composite import live_payload

# robot_0 sits at the world origin; robot_1's navigation frame is 10 m east,
# 5 m north and turned 90 degrees, so its floor patch along local +x must
# appear along world +y.
TRANSFORMS = {"robot_0": (0.0, 0.0, 0.0), "robot_1": (10.0, 5.0, math.pi / 2)}


def floor_patch(x0, x1, y0, y1, step=0.05, z=0.0):
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    grid = np.array([[x, y, z] for x in xs for y in ys])
    return grid


def wall(x, y, z0=0.3, z1=1.5):
    return np.array([[x, y, z] for z in np.arange(z0, z1, 0.1)])


ROBOT_0_POINTS = np.vstack([floor_patch(0.0, 1.0, 0.0, 1.0), wall(0.5, 0.5)])
ROBOT_1_POINTS = floor_patch(0.0, 1.0, 0.0, 0.2)


class MapServiceStub:
    def __init__(self, transforms):
        self._state_lock = threading.RLock()
        self.transforms = dict(transforms)


def publish_peer(store, tmp_path, robot_id, session, points, revision=1):
    source, chunks = peer(
        tmp_path, robot_id, session, revision=revision, points=points.tolist()
    )
    for name, raw in chunks.items():
        store.put_chunk(name, raw)
    assert store.publish(source)
    return source["snapshot"]["manifests"][0]["graph_revision"]["component_id"]


@pytest.fixture
def setup(tmp_path, monkeypatch):
    session = str(uuid4())
    store = ReplicaStore(tmp_path / "replicas")
    components = {
        "robot_0": publish_peer(store, tmp_path, "robot_0", session, ROBOT_0_POINTS),
        "robot_1": publish_peer(store, tmp_path, "robot_1", session, ROBOT_1_POINTS),
    }
    monkeypatch.setattr(replica_views, "store", lambda: store)
    monkeypatch.setattr(autonomy_routes, "store", lambda: store)

    registry = Registry()
    sink = AsyncMock()
    for robot_id, component_id in components.items():
        registry.hello(
            dict(robot_id=robot_id, capabilities=["navigate", "plan_objective"]), sink
        )
        registry.update_state(
            dict(
                robot_id=robot_id,
                pose=dict(x=1, y=1),
                live_mapping=live_payload(
                    robot_id, component_id, session, IDENTITY_SE3
                ),
            )
        )
    monkeypatch.setattr(replica_views, "registry", registry)
    monkeypatch.setattr(replica_live, "registry", registry)
    map_service = MapServiceStub(TRANSFORMS)
    monkeypatch.setattr(replica_views, "map_service", map_service)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", session)
    monkeypatch.setattr(map_routes, "_optimized", {})

    calls = []

    def counting_rasterize(points, **kwargs):
        calls.append(len(points))
        return deployment_raster.rasterize_points(points, **kwargs)

    refresher = deployment_raster.DeploymentRasterRefresher(
        rasterize=counting_rasterize
    )
    yield dict(
        session=session,
        store=store,
        registry=registry,
        map_service=map_service,
        components=components,
        refresher=refresher,
        calls=calls,
        tmp_path=tmp_path,
    )
    store.close()


def scope_of(session):
    return f"deployment:{session}"


def placements(session):
    return replica_views.deployment_placements(session)


def cell_at(meta, cells, x, y):
    col = int(math.floor((x - meta.origin_x) / meta.resolution))
    row = int(math.floor((y - meta.origin_y) / meta.resolution))
    return int(cells[row, col])


def expected_transforms():
    return {
        robot_id: {"x": x, "y": y, "yaw": yaw}
        for robot_id, (x, y, yaw) in TRANSFORMS.items()
    }


def test_refresh_writes_the_scope_with_member_robots_and_world_navigation(setup):
    session, refresher = setup["session"], setup["refresher"]
    report = refresher.refresh(session, placements(session))
    assert report["status"] == "built"
    assert report["robots"] == ["robot_0", "robot_1"]
    assert report["missing_chunks"] == 0
    meta, cells, robots, transforms = map_routes._optimized[scope_of(session)]
    assert robots == ("robot_0", "robot_1")
    for robot_id, pose in expected_transforms().items():
        assert transforms[robot_id] == pytest.approx(pose)
    assert meta.resolution == 0.2
    assert cells.dtype == np.int8
    assert cells.shape == (meta.height, meta.width)
    # robot_0's floor is free, its wall occupied, the gap between robots unknown.
    assert cell_at(meta, cells, 0.1, 0.1) == 0
    assert cell_at(meta, cells, 0.5, 0.5) == 100
    assert cell_at(meta, cells, 5.0, 2.5) == -1
    # robot_1's floor along local +x lies along world +y from (10, 5): the
    # placement is T_world_navigation, the transform the 2D map applies to
    # that robot's telemetry.
    assert cell_at(meta, cells, 9.9, 5.5) == 0
    assert cell_at(meta, cells, 9.9, 5.9) == 0
    assert cell_at(meta, cells, 10.5, 5.1) == -1
    assert cell_at(meta, cells, 9.9, 4.5) == -1


def test_refresh_rebuilds_only_when_the_composite_changes(setup):
    session, refresher, calls = setup["session"], setup["refresher"], setup["calls"]
    assert refresher.refresh(session, placements(session))["status"] == "built"
    assert refresher.refresh(session, placements(session))["status"] == "unchanged"
    assert len(calls) == 1

    # A member republishes new geometry: the composite snapshot changes.
    publish_peer(
        setup["store"],
        setup["tmp_path"],
        "robot_1",
        session,
        floor_patch(0.0, 2.0, 0.0, 0.2),
        revision=2,
    )
    assert refresher.refresh(session, placements(session))["status"] == "built"
    assert len(calls) == 2
    meta, cells, _, _ = map_routes._optimized[scope_of(session)]
    assert cell_at(meta, cells, 9.9, 6.5) == 0

    # A member moves in the deployment frame: same geometry, new placement.
    setup["map_service"].transforms["robot_1"] = (10.0, 6.0, math.pi / 2)
    assert refresher.refresh(session, placements(session))["status"] == "built"
    assert len(calls) == 3
    _, _, _, transforms = map_routes._optimized[scope_of(session)]
    assert transforms["robot_1"] == pytest.approx(
        {"x": 10.0, "y": 6.0, "yaw": math.pi / 2}
    )

    # A map reset cleared the store: rebuild although nothing changed.
    map_routes.reset_optimized_maps()
    assert refresher.refresh(session, placements(session))["status"] == "built"
    assert len(calls) == 4


def test_scope_survives_a_missing_composite_and_retires_with_the_mission(setup):
    session, refresher, registry = (
        setup["session"],
        setup["refresher"],
        setup["registry"],
    )
    assert refresher.refresh(session, placements(session))["status"] == "built"
    # A member without live authority leaves fewer than two placed robots.
    registry.update_state(dict(robot_id="robot_1", pose=dict(x=1, y=1)))
    report = refresher.refresh(session, placements(session))
    assert report["status"] == "no composite"
    assert scope_of(session) in map_routes._optimized

    other = str(uuid4())
    report = refresher.refresh(other, {})
    assert report["retired"] == [scope_of(session)]
    assert scope_of(session) not in map_routes._optimized
    assert refresher.refresh(None, {})["status"] == "no mission"


def test_a_verified_merge_is_rasterized_under_its_component_id(setup, tmp_path):
    # Inter-robot closures merged the two robots: both now publish one
    # component anchored at robot_0's first keyframe. That verified component
    # is the fleet map, under the back-end's own ``component:<id>`` name (the
    # 2D view ranks it first), placed in the component frame with each
    # robot's live T_component_navigation; the composite raster is retired.
    session, refresher, store = setup["session"], setup["refresher"], setup["store"]
    registry = setup["registry"]
    first = refresher.refresh(
        session, placements(session), deployment_raster.component_frames(session)
    )
    assert first["status"] == "built" and first["scope"] == scope_of(session)

    # Both robots republish one component anchored at robot_0's first
    # keyframe with the same accepted solution order, as a merge does.
    for robot_id, points, revision in (
        ("robot_0", ROBOT_0_POINTS, 2),
        ("robot_1", ROBOT_1_POINTS, 3),
    ):
        source, chunks = peer(
            tmp_path / f"merged-{robot_id}",
            robot_id,
            session,
            anchor_robot="robot_0",
            order=[7, 0],
            revision=revision,
            points=points.tolist(),
        )
        for name, raw in chunks.items():
            store.put_chunk(name, raw)
        assert store.publish(source)
    shared = source["snapshot"]["manifests"][0]["graph_revision"]["component_id"]
    assert shared == setup["components"]["robot_0"]
    shift = [
        [1.0, 0.0, 0.0, 4.0],
        [0.0, 1.0, 0.0, -1.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for robot_id, transform in (("robot_0", IDENTITY_SE3), ("robot_1", shift)):
        payload = live_payload(robot_id, shared, session, transform)
        payload["solution_order"] = [7, 0]
        registry.update_state(
            dict(robot_id=robot_id, pose=dict(x=1, y=1), live_mapping=payload)
        )

    frames = deployment_raster.component_frames(session)
    assert frames["robot_1"][0] == shared
    report = refresher.refresh(session, placements(session), frames)
    assert report["status"] == "built"
    assert report["scope"] == f"component:{shared}"
    assert report["robots"] == ["robot_0", "robot_1"]
    assert report["retired"] == [scope_of(session)]
    meta, cells, robots, transforms = map_routes._optimized[f"component:{shared}"]
    assert robots == ("robot_0", "robot_1")
    assert transforms["robot_0"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert transforms["robot_1"] == {"x": 4.0, "y": -1.0, "yaw": 0.0}
    assert scope_of(session) not in map_routes._optimized
    # The back-end's scope list still never prunes it.
    assert map_routes._prune_optimized_maps(["robot:x"]) == []
    assert f"component:{shared}" in map_routes._optimized


def test_slam_scope_list_never_prunes_the_deployment_scope():
    meta = GridMeta(0.2, 2, 2, 0.0, 0.0)
    cells = np.zeros((2, 2), dtype=np.int8)
    scope = scope_of(str(uuid4()))
    map_routes.reset_optimized_maps()
    map_routes.publish_optimized_map(scope, meta, cells, ("robot_0",), None)
    # A verified component this server rasterized shares the back-end's
    # naming and is kept like the composite; a back-end-posted grid the
    # back-end no longer lists is pruned.
    map_routes.publish_optimized_map(
        "component:2", meta, cells, ("robot_1", "robot_2"), None
    )
    with map_routes._optimized_lock:
        map_routes._optimized["component:1"] = (meta, cells, ("robot_1",), None)
    assert map_routes._prune_optimized_maps(["robot:a"]) == ["component:1"]
    assert sorted(map_routes._optimized) == sorted([scope, "component:2"])
    assert map_routes.retire_server_scopes(keep="component:2") == [scope]
    assert map_routes.retire_deployment_scopes(keep=None) == ["component:2"]
    assert map_routes._optimized == {}
    assert map_routes.DEPLOYMENT_SCOPE_PREFIX == replica_views.DEPLOYMENT_PREFIX
    with pytest.raises(ValueError, match="outside"):
        map_routes.publish_optimized_map(
            scope, meta, cells, ("robot_0",), {"robot_9": {"x": 0, "y": 0, "yaw": 0}}
        )


def test_oversized_composite_is_skipped_with_one_warning(setup, caplog):
    session = setup["session"]
    refresher = deployment_raster.DeploymentRasterRefresher(max_points=1)
    with caplog.at_level(logging.WARNING, logger=deployment_raster.__name__):
        first = refresher.refresh(session, placements(session))
        second = refresher.refresh(session, placements(session))
    assert first["status"] == "skipped"
    assert "budget" in first["detail"]
    assert second["status"] == "skipped"
    assert scope_of(session) not in map_routes._optimized
    assert sum("skipped" in record.message for record in caplog.records) == 1


def test_tick_reads_placements_then_builds_off_loop(setup, caplog):
    session, refresher = setup["session"], setup["refresher"]
    with caplog.at_level(logging.INFO, logger=deployment_raster.__name__):
        report = asyncio.run(refresher.tick())
    assert report["status"] == "built"
    assert scope_of(session) in map_routes._optimized
    (line,) = [r.message for r in caplog.records if "Fleet raster" in r.message]
    assert "2 robots" in line and "cells" in line and "ms" in line


def test_routes_list_and_serve_the_deployment_scope(setup):
    from swarmdeck_server.api.app import app

    session, refresher = setup["session"], setup["refresher"]
    assert refresher.refresh(session, placements(session))["status"] == "built"
    meta, cells, _, _ = map_routes._optimized[scope_of(session)]
    with TestClient(app) as client:
        index = client.get("/api/map/optimized")
        assert index.status_code == 200
        (entry,) = [m for m in index.json()["maps"] if m["scope"] == scope_of(session)]
        assert entry == {
            "scope": scope_of(session),
            "robots": ["robot_0", "robot_1"],
            "resolution": meta.resolution,
            "width": meta.width,
            "height": meta.height,
            "origin": {"x": meta.origin_x, "y": meta.origin_y},
        }
        response = client.get(f"/api/map/optimized/{scope_of(session)}")
        assert response.status_code == 200
        assert response.headers["content-type"] == "image/png"
        assert int(response.headers["X-Map-Width"]) == meta.width
        assert int(response.headers["X-Map-Height"]) == meta.height
        assert float(response.headers["X-Map-Resolution"]) == meta.resolution
        published = json.loads(response.headers["X-Map-Transforms"])
        assert published.keys() == expected_transforms().keys()
        for robot_id, pose in expected_transforms().items():
            assert published[robot_id] == pytest.approx(pose)
        image = np.asarray(Image.open(BytesIO(response.content)).convert("RGB"))
        assert image.shape == (meta.height, meta.width, 3)

        def pixel(x, y):
            col = int(math.floor((x - meta.origin_x) / meta.resolution))
            row = int(math.floor((y - meta.origin_y) / meta.resolution))
            return tuple(int(v) for v in image[meta.height - 1 - row, col])

        assert pixel(0.1, 0.1) == output.FREE_RGB
        assert pixel(0.5, 0.5) == output.OCCUPIED_RGB
        assert pixel(5.0, 2.5) == output.UNKNOWN_RGB
        assert pixel(9.9, 5.5) == output.FREE_RGB
        assert cells[0, 0] == -1
