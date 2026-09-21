"""Geometry, wire-format and export regressions without CUDA or a running ROS graph."""

import asyncio
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import numpy as np
import pytest
from starlette.requests import Request
from adapters.reconstruction import colorize_points
from autonomy.reconstruction import (
    BackendCapabilities,
    DurableJobRunner,
    InputManifest,
    JobState,
    PoseRevision,
    ReconstructionBackend,
    ResourceBudget,
)

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
    source.write_bytes(
        source.read_bytes().replace(b"property float x\n", b"property int x\n")
    )
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

    pointer.write_text(
        json.dumps(
            {"state": "ready", "artifact": str(artifact), "artifact_size_bytes": 1}
        )
    )
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(
        json.dumps(
            {"state": "ready", "artifact": str(artifact), "artifact_sha256": "0" * 64}
        )
    )
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(
        json.dumps(
            {"state": "ready", "artifact": str(tmp_path.parent / "outside.swgs")}
        )
    )
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(json.dumps({"state": "stale", "artifact": str(artifact)}))
    assert asyncio.run(get_gaussians(request())).status_code == 409
    pointer.write_text(
        json.dumps(
            {
                "schema_version": 3,
                "state": "ready",
                "artifact": str(artifact),
            }
        )
    )
    assert asyncio.run(get_gaussians(request())).status_code == 409


def test_job_publication_is_served_with_source_metadata_and_stale_rejection(
    tmp_path, monkeypatch
):
    from swarmdeck_server.api import reconstruction_routes
    from swarmdeck_server.api.reconstruction_routes import get_gaussians

    class InertBackend(ReconstructionBackend):
        name = "inert-api-test"
        version = "test"
        capabilities = BackendCapabilities(fixed_camera_poses=True)

        def run(self, job, workdir, cancel_event, run_command):
            artifact = workdir / "artifact.swgs"
            artifact.write_bytes(struct.pack("<4sIII", b"SWGS", 1, 1, 0) + b"\0" * 56)
            return artifact

    capture = tmp_path / "capture"
    capture.mkdir()
    (capture / "capture_manifest.json").write_text(
        json.dumps(
            {
                "capture_id": "capture-api",
                "robot_id": "robot-api",
                "session_id": "session-api",
                "submap_id": "submap-api",
                "calibration_version": "cal-api",
                "optical_frame": "camera_color_optical_frame",
            }
        )
    )
    frame = capture / "frame.npz"
    frame.write_bytes(b"initial-frame")
    manifest = replace(
        InputManifest.from_capture(capture),
        pose_revision=PoseRevision(
            graph_revision="4",
            pose_revision="9",
            geometry_revision="12",
            component_id="component-api",
        ),
    )
    target = tmp_path / "published" / "global.swgs"
    runner = DurableJobRunner(tmp_path / "runner", [InertBackend()])
    first = runner.submit(
        manifest,
        backend="inert-api-test",
        artifact_target=target,
        budget=ResourceBudget(max_frames=1, max_gaussians=1),
    )
    assert runner.run(first.job_id).state is JobState.READY

    monkeypatch.setenv("SWARMDECK_RECONSTRUCTION_DIR", str(target.parent))

    def request(query: bytes = b""):
        return Request({"type": "http", "query_string": query, "headers": []})

    assert asyncio.run(get_gaussians(request())).status_code == 409
    scoped_query = b"session_id=session-api&component_id=component-api"
    response = asyncio.run(get_gaussians(request(scoped_query)))
    assert response.status_code == 200
    assert Path(response.path).read_bytes() == target.read_bytes()
    assert response.headers["X-Reconstruction-Frame"] == "component"
    assert response.headers["X-Reconstruction-Session"] == "session-api"
    assert response.headers["X-Reconstruction-Component"] == "component-api"
    assert response.headers["X-Reconstruction-Artifact-SHA256"]
    assert (
        asyncio.run(
            get_gaussians(request(scoped_query + b"&pose_revision=8"))
        ).status_code
        == 409
    )

    status = asyncio.run(
        get_gaussians(request(scoped_query + b"&format=status&pose_revision=9"))
    )
    assert status.status_code == 200
    status_body = json.loads(status.body)
    assert status_body["job_id"] == first.job_id
    assert status_body["input_fingerprint"] == first.input_manifest.fingerprint
    assert (
        status_body["source"]["frame_manifest_sha256"] == manifest.frame_manifest_digest
    )

    pointer_path = target.parent / "global.swgs.manifest.json"
    pointer_text = pointer_path.read_text()
    inconsistent = json.loads(pointer_text)
    inconsistent["source"]["frame"] = "world"
    pointer_path.write_text(json.dumps(inconsistent))
    assert asyncio.run(get_gaussians(request(scoped_query))).status_code == 409
    pointer_path.write_text(pointer_text)

    reconstruction_routes._digest_for_key.cache_clear()
    digest_calls = []
    original_digest = reconstruction_routes._file_digest

    def count_digest(path):
        digest_calls.append(path)
        return original_digest(path)

    monkeypatch.setattr(reconstruction_routes, "_file_digest", count_digest)
    status_query = scoped_query + b"&format=status"
    asyncio.run(get_gaussians(request(status_query)))
    asyncio.run(get_gaussians(request(status_query)))
    assert len(digest_calls) == 1

    pointer = json.loads(pointer_path.read_text())
    artifact_path = Path(pointer["artifact"])
    original_artifact = artifact_path.read_bytes()
    corrupted = bytearray(original_artifact)
    corrupted[-1] ^= 1
    artifact_path.write_bytes(corrupted)
    assert asyncio.run(get_gaussians(request(status_query))).status_code == 409
    artifact_path.write_bytes(original_artifact)

    second = runner.submit(
        manifest,
        backend="inert-api-test",
        artifact_target=target,
        budget=ResourceBudget(max_frames=1, max_gaussians=1),
    )
    frame.write_bytes(b"newer-frame")
    assert runner.run(second.job_id).state is JobState.STALE

    after_stale = asyncio.run(get_gaussians(request(scoped_query)))
    assert after_stale.status_code == 200
    assert after_stale.headers["X-Reconstruction-Job"] == first.job_id
    assert Path(after_stale.path).read_bytes() == target.read_bytes()


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


@pytest.mark.parametrize(
    "field,value",
    [
        ("schema_version", True),
        ("schema_version", []),
        ("state", []),
        ("source", {"frame": []}),
    ],
)
def test_reconstruction_rejects_malformed_pointer_types(
    tmp_path, monkeypatch, field, value
):
    from swarmdeck_server.api.reconstruction_routes import get_gaussians

    monkeypatch.setenv("SWARMDECK_RECONSTRUCTION_DIR", str(tmp_path))
    pointer = {"artifact": str(tmp_path / "unused.swgs"), field: value}
    (tmp_path / "global.swgs.manifest.json").write_text(json.dumps(pointer))
    request = Request({"type": "http", "query_string": b"", "headers": []})
    assert asyncio.run(get_gaussians(request)).status_code == 409


def test_raw_local_gaussians_cannot_be_mislabeled_as_global(tmp_path, monkeypatch):
    from swarmdeck_server.api.reconstruction_routes import get_gaussians

    monkeypatch.setenv("SWARMDECK_RECONSTRUCTION_DIR", str(tmp_path))
    artifact = tmp_path / "local.swgs"
    artifact.write_bytes(struct.pack("<4sIII", b"SWGS", 1, 1, 0) + b"\0" * 56)
    manifest = InputManifest(
        capture_root=str(tmp_path), metadata={"world_frame": "odom"}
    )
    (tmp_path / "global.swgs.manifest.json").write_text(
        json.dumps(
            {
                "artifact": str(artifact),
                "source": manifest.publication_source(),
            }
        )
    )
    for query in (b"", b"format=status", b"session_id=local&component_id=local"):
        request = Request({"type": "http", "query_string": query, "headers": []})
        assert asyncio.run(get_gaussians(request)).status_code == 409


def test_reconstruction_disk_queue_stays_bounded_after_request_cancellation(
    monkeypatch,
):
    from concurrent.futures import ThreadPoolExecutor
    import threading
    from swarmdeck_server.api import reconstruction_routes as routes

    gate = threading.Event()
    finished = threading.Event()
    calls = []
    executor = ThreadPoolExecutor(max_workers=1)
    monkeypatch.setattr(routes, "_disk_executor", executor)
    monkeypatch.setattr(routes, "_disk_slots", threading.BoundedSemaphore(2))

    def validate(root):
        try:
            assert gate.wait(3), "test did not release disk worker"
            calls.append(root)
            return root
        finally:
            if len(calls) == 2:
                finished.set()

    monkeypatch.setattr(routes, "_validated_source", validate)

    async def check():
        pending = [
            asyncio.create_task(routes._source_async(Path(str(i)))) for i in range(2)
        ]
        await asyncio.sleep(0.01)  # Event loop remains responsive during the disk read.
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        with pytest.raises(routes._PointerError, match="queue is full"):
            await routes._source_async(Path("overflow"))
        gate.set()
        assert await asyncio.to_thread(finished.wait, 3)
        assert await routes._source_async(Path("recovered")) == Path("recovered")

    try:
        asyncio.run(check())
    finally:
        gate.set()
        executor.shutdown(wait=True)


@pytest.mark.parametrize(
    "frame,frame_id,component",
    [
        ("component", "another-component", "component-a"),
        ("world", "odom", ""),
        ("local", "world", ""),
    ],
)
def test_reconstruction_frame_label_must_agree_with_output_identity(
    frame, frame_id, component
):
    from swarmdeck_server.api.reconstruction_routes import (
        _PointerError,
        _source_metadata,
    )

    with pytest.raises(_PointerError, match="frame"):
        _source_metadata(
            {
                "source": {
                    "frame": frame,
                    "frame_id": frame_id,
                    "pose_revision": {"component_id": component},
                }
            }
        )


@pytest.mark.parametrize("source", [{}, {"frame": "world"}])
def test_new_reconstruction_pointers_require_explicit_output_frame(source):
    from swarmdeck_server.api.reconstruction_routes import (
        _PointerError,
        _source_metadata,
    )

    with pytest.raises(_PointerError, match="frame metadata is missing"):
        _source_metadata({"schema_version": 2, "source": source})
