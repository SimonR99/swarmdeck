"""Geometry, wire-format and export regressions without CUDA or a running ROS graph."""

import asyncio
import importlib.util
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
