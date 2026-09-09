"""Geometry, wire-format and export regressions without CUDA or a running ROS graph."""

import asyncio
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import zlib
import numpy as np
import pytest
from starlette.requests import Request
from adapters.reconstruction import colorize_points
from swarmdeck_server.mapsvc.service import MapService
from swarmdeck_server.mapsvc.output import merged_cloud

spec = importlib.util.spec_from_file_location(
    "umami", Path(__file__).parents[2] / "scripts/reconstruction/umami.py"
)
umami = importlib.util.module_from_spec(spec)
spec.loader.exec_module(umami)


def test_projection_rejects_occluded_behind_and_out_of_image_points():
    rgb = np.zeros((3, 3, 3), dtype=np.uint8)
    rgb[1, 1] = [255, 20, 0]
    k = np.array([[1, 0, 1], [0, 1, 1], [0, 0, 1.0]])
    points = np.array([[0, 0, 1], [0, 0, 2], [0, 0, -1], [20, 0, 1], [np.nan, 0, 1]])
    colors, mask = colorize_points(points, rgb, k, np.eye(4))
    assert mask.tolist() == [True, False, False, False, False]
    assert colors[0].tolist() == [255, 20, 0]
    _, mask = colorize_points(points, rgb, k, np.eye(4), depth=np.full((3, 3), 0.5))
    assert not mask.any()


def test_colors_follow_same_voxels_and_clear_with_cloud():
    service = MapService()
    points = np.array([[0, 0, 0], [0.01, 0, 0], [1, 0, 0]], dtype=np.float32)
    rgb = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8)
    service.set_cloud("r", points, rgb=rgb, register=False)
    rgb[:] = 0
    xyz, owners, names, colors = merged_cloud(service, robot_id="r", include_rgb=True)
    assert names == ["r"] and len(xyz) == 2
    assert colors.tolist() == [[128, 128, 0], [0, 0, 255]]
    service.set_cloud("r", points, register=False)
    assert merged_cloud(service, robot_id="r", include_rgb=True)[3] is None


def test_pose_export_inverts_camera_pose_and_preserves_metric_depth(tmp_path):
    capture = tmp_path / "capture"
    capture.mkdir()
    twc = np.eye(4)
    twc[:3, 3] = [5, 2, 1]
    np.savez(
        capture / "0.npz",
        rgb=np.full((2, 2, 3), 128, dtype=np.uint8),
        depth_m=np.ones((2, 2)),
        K=np.eye(3),
        T_world_camera=twc,
    )
    out = tmp_path / "colmap"
    assert umami.export_colmap(capture, out, stride=1) == 4
    raw = (out / "sparse/0/images.bin").read_bytes()
    pose = struct.unpack_from("<I7dI", raw, 8)
    assert pose[1:5] == (1.0, 0.0, 0.0, 0.0)
    assert pose[5:8] == (-5.0, -2.0, -1.0)
    point = struct.unpack_from(
        "<Q3d3BdQ", (out / "sparse/0/points3D.bin").read_bytes(), 8
    )
    assert point[1:4] == (5.0, 2.0, 2.0)


@pytest.mark.parametrize("angle", [0, 0.7, np.pi, -2.5])
def test_quaternion_handles_rotations_near_pi(angle):
    c, s = np.cos(angle), np.sin(angle)
    r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    q = umami.rotation_quaternion(r)
    assert np.allclose(
        abs(q @ np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])), 1
    )


def ply_file(path):
    names = [
        "x",
        "y",
        "z",
        "scale_0",
        "scale_1",
        "scale_2",
        "rot_0",
        "rot_1",
        "rot_2",
        "rot_3",
        "f_dc_0",
        "f_dc_1",
        "f_dc_2",
        "opacity",
    ]
    header = (
        "ply\nformat binary_little_endian 1.0\nelement vertex 2\n"
        + "".join(f"property float {n}\n" for n in names)
        + "end_header\n"
    )
    records = np.array(
        [
            [1, 2, 3, -2, -2, -2, 1, 0, 0, 0, 0, 0, 0, 2],
            [np.nan, 0, 0, -2, -2, -2, 1, 0, 0, 0, 0, 0, 0, 2],
        ],
        dtype="<f4",
    )
    path.write_bytes(header.encode() + records.tobytes())


def test_ply_conversion_activates_scale_opacity_and_reorders_quaternion(tmp_path):
    source = tmp_path / "in.ply"
    ply_file(source)
    output = tmp_path / "global.swgs"
    assert umami.convert_ply(source, output) == 1
    raw = output.read_bytes()
    assert struct.unpack_from("<4sIII", raw) == (b"SWGS", 1, 1, 0)
    values = np.frombuffer(raw, dtype="<f4", offset=16)
    assert np.allclose(values[3:6], np.exp(-2))
    assert values[6:10].tolist() == [0, 0, 0, 1]
    assert np.allclose(values[10:13], ((0.5 + 0.055) / 1.055) ** 2.4)
    assert np.isclose(values[13], 1 / (1 + np.exp(-2)))


@pytest.mark.parametrize("integer_type", ["int", "int32"])
def test_native_umami_creation_keyframe_metadata_preserves_vertex_alignment(
    tmp_path, integer_type
):
    source = tmp_path / "native.ply"
    ply_file(source)
    header, payload = source.read_bytes().split(b"end_header\n", 1)
    floats = np.frombuffer(payload, dtype="<f4").reshape(2, 14).copy()
    floats[1, :3] = [4, 5, 6]
    records = np.zeros(2, dtype=[("attributes", "<f4", (14,)), ("keyframe", "<i4")])
    records["attributes"] = floats
    records["keyframe"] = [-1, 123]
    source.write_bytes(
        header
        + f"property {integer_type} creation_kfid\nend_header\n".encode()
        + records.tobytes()
    )

    vertices = umami.read_gaussian_ply(source)
    assert vertices["creation_kfid"].tolist() == [-1, 123]
    assert vertices["x"].tolist() == [1, 4]
    output = tmp_path / "native.swgs"
    assert umami.convert_ply(source, output) == 2
    compact = np.frombuffer(output.read_bytes(), dtype="<f4", offset=16).reshape(2, 14)
    assert compact[:, :3].tolist() == [[1, 2, 3], [4, 5, 6]]

    source.write_bytes(source.read_bytes()[:-1])
    with pytest.raises(ValueError, match="truncated"):
        umami.read_gaussian_ply(source)


def test_gaussian_geometry_attributes_must_remain_float(tmp_path):
    source = tmp_path / "invalid.ply"
    ply_file(source)
    source.write_bytes(source.read_bytes().replace(b"property float x\n", b"property int x\n"))
    with pytest.raises(ValueError, match="unsupported Gaussian PLY property"):
        umami.read_gaussian_ply(source)


def test_reconstruction_endpoint_scope_etag_and_corruption(tmp_path, monkeypatch):
    from swarmdeck_server.api.reconstruction_routes import get_gaussians

    monkeypatch.setenv("SWARMDECK_RECONSTRUCTION_DIR", str(tmp_path))
    req = lambda query=b"", headers=[]: Request(
        {"type": "http", "query_string": query, "headers": headers}
    )
    assert asyncio.run(get_gaussians(req())).status_code == 404
    source = tmp_path / "in.ply"
    ply_file(source)
    umami.convert_ply(source, tmp_path / "global.swgs")
    response = asyncio.run(get_gaussians(req()))
    assert response.status_code == 200
    assert (
        asyncio.run(
            get_gaussians(
                req(headers=[(b"if-none-match", response.headers["etag"].encode())])
            )
        ).status_code
        == 304
    )
    assert asyncio.run(get_gaussians(req(b"robot_id=other"))).status_code == 404
    (tmp_path / "global.swgs").write_bytes(b"corrupt")
    assert asyncio.run(get_gaussians(req())).status_code == 422


def test_reconstruction_manifest_pointer_is_validated_and_served(tmp_path, monkeypatch):
    from swarmdeck_server.api.reconstruction_routes import get_gaussians

    monkeypatch.setenv("SWARMDECK_RECONSTRUCTION_DIR", str(tmp_path))
    source = tmp_path / "in.ply"
    ply_file(source)
    legacy = tmp_path / "global.swgs"
    umami.convert_ply(source, legacy)
    artifact = tmp_path / "global.swgs.job-uuid"
    artifact.write_bytes(legacy.read_bytes())
    legacy.write_bytes(b"legacy bytes that must not be served")
    pointer = tmp_path / "global.swgs.manifest.json"
    pointer.write_text(
        json.dumps(
            {
                "version": "job-uuid",
                "state": "ready",
                "job_id": "job-uuid",
                "artifact": str(artifact),
                "artifact_size_bytes": artifact.stat().st_size,
                "artifact_sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            }
        )
    )
    request = lambda: Request({"type": "http", "query_string": b"", "headers": []})
    response = asyncio.run(get_gaussians(request()))
    assert response.status_code == 200
    assert Path(response.path).resolve() == artifact.resolve()
    assert response.headers["X-Reconstruction-Version"] == "job-uuid"
    assert response.headers["X-Reconstruction-State"] == "ready"

    pointer.write_text(json.dumps({"state": "ready", "artifact": str(artifact), "artifact_size_bytes": 1}))
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(json.dumps({"state": "ready", "artifact": str(artifact), "artifact_sha256": "0" * 64}))
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(json.dumps({"state": "ready", "artifact": str(tmp_path.parent / "outside.swgs")}))
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(json.dumps({"state": "stale", "artifact": str(artifact)}))
    assert asyncio.run(get_gaussians(request())).status_code == 409


def test_cloud_float_transport_preserves_far_coordinates_and_rgb(monkeypatch):
    from swarmdeck_server.api import map_routes

    service = MapService()
    service.set_cloud(
        "r",
        np.array([[1000, 2, 3]], dtype=np.float32),
        rgb=np.array([[255, 0, 0]], dtype=np.uint8),
        register=False,
    )
    monkeypatch.setattr(map_routes, "map_service", service)
    request = Request(
        {"type": "http", "query_string": b"robot_id=r&format=xyz32", "headers": []}
    )
    response = asyncio.run(map_routes.get_cloud(request))
    assert response.status_code == 200
    assert response.headers["X-Cloud-Frame"] == "local"
    raw = zlib.decompress(response.body)
    assert np.frombuffer(raw, dtype="<f4", count=3).tolist() == [1000, 2, 3]
    assert list(raw[13:]) == [255, 0, 0]
    request = Request(
        {
            "type": "http",
            "query_string": b"robot_id=r&format=xyz32",
            "headers": [(b"if-none-match", response.headers["etag"].encode())],
        }
    )
    assert asyncio.run(map_routes.get_cloud(request)).status_code == 304


def test_rgbd_projection_preserves_observation_mask_and_rejects_occlusion():
    from types import SimpleNamespace as NS
    from adapters.reconstruction import colorize_ros_rgbd

    rgb = np.zeros((3, 3, 3), np.uint8)
    rgb[1, 1] = [240, 10, 20]
    depth = np.full((3, 3), 2.0, dtype="<f4")
    image = NS(encoding="rgb8", width=3, height=3, step=9, data=rgb.tobytes())
    depth_image = NS(
        encoding="32FC1",
        width=3,
        height=3,
        step=12,
        is_bigendian=False,
        data=depth.tobytes(),
    )
    info = NS(width=3, height=3, d=[], k=[1, 0, 1, 0, 1, 1, 0, 0, 1])
    colors = colorize_ros_rgbd(
        np.array([[0, 0, 2], [0, 0, 3], [0, 0, -2]]),
        image,
        depth_image,
        info,
        np.eye(4),
    )
    assert colors.tolist() == [
        [240, 10, 20, 255],
        [148, 148, 148, 0],
        [148, 148, 148, 0],
    ]


def test_local_slam_fallback_keeps_world_frame_metadata(monkeypatch):
    from swarmdeck_server.api import map_routes
    from swarmdeck_server.mapsvc import graph_bridge
    import urllib.request
    from io import BytesIO

    service = MapService()
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(graph_bridge, "SLAM_URL", "http://unused")
    body = zlib.compress(np.array([800, 2100, 50], dtype="<i2").tobytes() + bytes([0]))

    class Upstream(BytesIO):
        headers = {
            "X-Cloud-Points": "1",
            "X-Cloud-Robots": "r",
            "X-Cloud-Scale": "0.01",
        }

    monkeypatch.setattr(urllib.request, "urlopen", lambda *a, **kw: Upstream(body))
    response = asyncio.run(
        map_routes.get_cloud(
            Request({"type": "http", "headers": [], "query_string": b"robot_id=r"})
        )
    )
    assert response.headers["X-Cloud-Frame"] == "world"
    assert response.body == body


@pytest.mark.parametrize("source", ["optimized", "slam"])
def test_cloud_source_selects_reconstruction_even_with_live_scan(monkeypatch, source):
    from io import BytesIO
    import urllib.request
    from swarmdeck_server.api import map_routes
    from swarmdeck_server.mapsvc import graph_bridge

    service = MapService()
    service.set_cloud("r", np.array([[1, 2, 3]], dtype=np.float32), register=False)
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(graph_bridge, "SLAM_URL", "http://slam")
    body = zlib.compress(np.array([800, 2100, 50], dtype="<i2").tobytes() + bytes([0]))
    calls = []

    class Upstream(BytesIO):
        headers = {
            "X-Cloud-Points": "1",
            "X-Cloud-Robots": "r",
            "X-Cloud-Scale": "0.01",
            "X-Cloud-Frame": "world",
        }

    def upstream(url, **kwargs):
        calls.append(url)
        return Upstream(body)

    monkeypatch.setattr(urllib.request, "urlopen", upstream)
    response = asyncio.run(
        map_routes.get_cloud(
            Request(
                {
                    "type": "http",
                    "headers": [],
                    "query_string": f"robot_id=r&source={source}".encode(),
                }
            )
        )
    )
    assert response.status_code == 200
    if source == "optimized":
        assert calls == ["http://slam/cloud?robot_id=r"]
        assert response.body == body
        assert response.headers["X-Cloud-Frame"] == "world"
    else:
        assert calls == []
        assert np.frombuffer(
            zlib.decompress(response.body), dtype="<f4", count=3
        ).tolist() == [1, 2, 3]
        assert response.headers["X-Cloud-Frame"] == "local"


def test_optimized_cloud_keeps_live_fallback_when_reconstruction_is_empty(monkeypatch):
    from io import BytesIO
    import urllib.request
    from swarmdeck_server.api import map_routes
    from swarmdeck_server.mapsvc import graph_bridge

    service = MapService()
    service.set_cloud("r", np.array([[1, 2, 3]], dtype=np.float32), register=False)
    monkeypatch.setattr(map_routes, "map_service", service)
    monkeypatch.setattr(graph_bridge, "SLAM_URL", "http://slam")

    class Empty(BytesIO):
        headers = {"X-Cloud-Points": "0"}

    monkeypatch.setattr(
        urllib.request, "urlopen", lambda *a, **kw: Empty(zlib.compress(b""))
    )
    response = asyncio.run(
        map_routes.get_cloud(
            Request(
                {
                    "type": "http",
                    "headers": [],
                    "query_string": b"robot_id=r&source=optimized",
                }
            )
        )
    )
    assert response.status_code == 200
    assert response.headers["X-Cloud-Points"] == "1"
    assert response.headers["X-Cloud-Frame"] == "local"
