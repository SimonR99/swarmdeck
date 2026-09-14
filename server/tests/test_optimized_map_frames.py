"""Optimized raster provenance is transported with the corresponding pixels."""

import asyncio
import json
import zlib

import pytest
from starlette.requests import Request

from swarmdeck_server.api import map_routes


def upload(transforms=None):
    headers = []
    if transforms is not None:
        headers.append((b"x-map-transforms", transforms.encode("ascii")))

    async def receive():
        return {"type": "http.request", "body": zlib.compress(bytes([0, 100, 0, 0]))}

    request = Request(
        {
            "type": "http",
            "method": "POST",
            "path": "/api/slam/optimized_map",
            "headers": headers,
            "query_string": b"scope=component:0&robots=r0&resolution=0.1&width=2&height=2",
        },
        receive,
    )
    return asyncio.run(map_routes.post_optimized_map(request))


@pytest.fixture(autouse=True)
def isolated_maps(monkeypatch):
    monkeypatch.setattr(map_routes, "_optimized", {})


def test_optimized_raster_keeps_its_own_transform_after_registration_changes(
    monkeypatch,
):
    captured = {"r0": {"x": 2.0, "y": -3.0, "yaw": 0.5}}
    assert upload(json.dumps(captured))["ok"]
    monkeypatch.setattr(map_routes.map_service, "transforms", {"r0": (90.0, 10.0, 1.5)})

    response = asyncio.run(map_routes.get_optimized_map("component:0"))
    assert response.status_code == 200
    assert json.loads(response.headers["x-map-transforms"]) == captured
    assert response.headers["x-map-width"] == "2"


def test_legacy_raster_does_not_claim_current_registration(monkeypatch):
    assert upload()["ok"]
    monkeypatch.setattr(map_routes.map_service, "transforms", {"r0": (90.0, 10.0, 1.5)})
    response = asyncio.run(map_routes.get_optimized_map("component:0"))
    assert "x-map-transforms" not in response.headers


@pytest.mark.parametrize(
    "transforms",
    [
        "[]",
        '{"r0":{"x":NaN,"y":0,"yaw":0}}',
        '{"r0":{"x":0,"y":0}}',
        '{"r0":{"x":false,"y":0,"yaw":0}}',
        '{"other":{"x":0,"y":0,"yaw":0}}',
        "{" + " " * 32768 + "}",
    ],
)
def test_invalid_provenance_does_not_replace_a_valid_raster(transforms):
    captured = {"r0": {"x": 2.0, "y": -3.0, "yaw": 0.5}}
    assert upload(json.dumps(captured))["ok"]
    rejected = upload(transforms)
    assert rejected.status_code == 400
    response = asyncio.run(map_routes.get_optimized_map("component:0"))
    assert json.loads(response.headers["x-map-transforms"]) == captured
