#!/usr/bin/env python3
"""Export posed RGB-D captures, run UMAMI's train_colmap, and publish compact Gaussians.

No CUDA or private repository dependency is imported by the SwarmDeck server.
"""

from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import numpy as np
from PIL import Image

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from autonomy.reconstruction import InputUnavailable, PoseSnapshot, _compose_se3
from autonomy.contracts import KeyframeId


def _keyframe_id(value: str) -> KeyframeId:
    parts = str(value).split("/")
    if len(parts) != 3 or not parts[0] or not parts[1]:
        raise ValueError(f"frame keyframe_id must be robot/session/seq, got {value!r}")
    try:
        sequence = int(parts[2])
    except ValueError as exc:
        raise ValueError(f"frame keyframe_id has an invalid sequence: {value!r}") from exc
    return KeyframeId(parts[0], parts[1], sequence)


def rotation_quaternion(r):
    # Eigen decomposition handles rotations near 180 degrees without division by zero.
    r = np.asarray(r)
    k = (
        np.array(
            [
                [
                    r[0, 0] - r[1, 1] - r[2, 2],
                    r[0, 1] + r[1, 0],
                    r[0, 2] + r[2, 0],
                    r[2, 1] - r[1, 2],
                ],
                [
                    r[0, 1] + r[1, 0],
                    r[1, 1] - r[0, 0] - r[2, 2],
                    r[1, 2] + r[2, 1],
                    r[0, 2] - r[2, 0],
                ],
                [
                    r[0, 2] + r[2, 0],
                    r[1, 2] + r[2, 1],
                    r[2, 2] - r[0, 0] - r[1, 1],
                    r[1, 0] - r[0, 1],
                ],
                [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], np.trace(r)],
            ]
        )
        / 3
    )
    _, v = np.linalg.eigh(k)
    q = v[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def export_colmap(
    capture: Path,
    output: Path,
    stride=8,
    max_points=200000,
    pose_snapshot: Path | None = None,
):
    """NPZ per keyframe: rgb, depth_m, K, T_world_camera, stamp. Optical camera axes.

    Depth seeds geometry; fixed camera poses preserve the fleet map frame during
    photometric training. Export again after pose-graph corrections.
    """
    frames = sorted(capture.glob("*.npz"))
    if not frames or stride < 1 or max_points < 1:
        raise ValueError("capture frames and positive export budgets required")
    capture_metadata = {}
    metadata_path = capture / "capture_manifest.json"
    if metadata_path.exists():
        capture_metadata = json.loads(metadata_path.read_text())
        if not isinstance(capture_metadata, dict):
            raise ValueError("capture_manifest.json must contain an object")
    solution = None
    if pose_snapshot is not None:
        try:
            solution = PoseSnapshot.from_file(pose_snapshot)
        except InputUnavailable as exc:
            raise ValueError(str(exc)) from exc
        capture_session = str(capture_metadata.get("session_id", ""))
        capture_robot = str(capture_metadata.get("robot_id", ""))
        capture_component = str(capture_metadata.get("component_id", ""))
        if capture_component and capture_component != solution.component_id:
            raise ValueError("pose snapshot component does not match capture metadata")
    output.mkdir(parents=True, exist_ok=False)
    sparse = output / "sparse" / "0"
    sparse.mkdir(parents=True)
    images = output / "images"
    images.mkdir()
    points = []
    colors = []
    voxel_keys = set()
    with (
        (sparse / "cameras.bin").open("wb") as cameras,
        (sparse / "images.bin").open("wb") as poses,
    ):
        cameras.write(struct.pack("<Q", len(frames)))
        poses.write(struct.pack("<Q", len(frames)))
        exported_frames = []
        for i, path in enumerate(frames, 1):
            with np.load(path, allow_pickle=False) as frame:
                rgb, depth, k, twc = (
                    frame[key] for key in ("rgb", "depth_m", "K", "T_world_camera")
                )
                optional = {
                    key: str(frame[key].item())
                    for key in (
                        "capture_id",
                        "robot_id",
                        "session_id",
                        "submap_id",
                        "calibration_version",
                        "keyframe_id",
                        "component_id",
                        "optical_frame",
                    )
                    if key in frame.files
                }
                frame_stamp = float(frame["stamp"]) if "stamp" in frame.files else None
                pose_source = "capture"
                if solution is not None:
                    if "keyframe_id" not in frame.files or "T_keyframe_camera" not in frame.files:
                        raise ValueError(
                            f"{path}: pose snapshot export requires dynamic keyframe_id and T_keyframe_camera"
                        )
                    keyframe = _keyframe_id(str(frame["keyframe_id"].item()))
                    if capture_session and keyframe.session_id != capture_session:
                        raise ValueError(f"{path}: keyframe session does not match capture metadata")
                    if capture_robot and keyframe.robot_id != capture_robot:
                        raise ValueError(f"{path}: keyframe robot does not match capture metadata")
                    if "component_id" in frame.files and str(frame["component_id"].item()) != solution.component_id:
                        raise ValueError(f"{path}: frame component does not match pose snapshot")
                    twc = np.asarray(
                        _compose_se3(solution.pose_for(keyframe), frame["T_keyframe_camera"]),
                        dtype=np.float64,
                    )
                    pose_source = f"graph-solution:{solution.component_id}:{solution.solution.revision.revision}"
                calibration = {
                    "intrinsics": np.asarray(k).tolist(),
                    "distortion": (
                        np.asarray(frame["D"]).tolist() if "D" in frame.files else []
                    ),
                    "distortion_model": (
                        str(frame["distortion_model"].item())
                        if "distortion_model" in frame.files
                        else "rectified_pinhole"
                    ),
                    "depth_units": (
                        str(frame["depth_units"].item())
                        if "depth_units" in frame.files
                        else "metres"
                    ),
                    "has_validity_mask": "depth_valid_mask" in frame.files,
                    "has_keyframe_to_camera": "T_keyframe_camera" in frame.files,
                }
            h, w = depth.shape
            if (
                rgb.shape != (h, w, 3)
                or rgb.dtype != np.uint8
                or k.shape != (3, 3)
                or twc.shape != (4, 4)
            ):
                raise ValueError(f"{path}: invalid aligned RGB-D frame")
            if (
                not np.isfinite(twc).all()
                or not np.allclose(twc[3], [0, 0, 0, 1])
                or not np.allclose(twc[:3, :3].T @ twc[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(twc[:3, :3]), 1)
            ):
                raise ValueError(f"{path}: pose must be rigid T_world_camera")
            if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
                raise ValueError(f"{path}: invalid intrinsics")
            cameras.write(
                struct.pack("<IiQQ4d", i, 1, w, h, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
            )
            tcw = np.linalg.inv(twc)
            q = rotation_quaternion(tcw[:3, :3])
            name = f"{i:08d}.png"
            Image.fromarray(rgb).save(images / name)
            poses.write(
                struct.pack("<I7dI", i, *q, *tcw[:3, 3], i)
                + name.encode()
                + b"\0"
                + struct.pack("<Q", 0)
            )
            yy, xx = np.mgrid[0:h:stride, 0:w:stride]
            z = depth[yy, xx]
            valid = np.isfinite(z) & (z > 0.1) & (z < 30)
            z, x, y = z[valid], xx[valid], yy[valid]
            xyz = np.column_stack(
                ((x - k[0, 2]) * z / k[0, 0], (y - k[1, 2]) * z / k[1, 1], z)
            )
            xyz = xyz @ twc[:3, :3].T + twc[:3, 3]
            # Per-frame allowance prevents the first camera from consuming the seed budget.
            allowance = max(1, max_points // len(frames))
            keep = np.linspace(0, len(xyz) - 1, min(len(xyz), allowance), dtype=int)
            for j in keep:
                key = tuple(np.floor(xyz[j] / 0.04).astype(np.int64))
                if key in voxel_keys or len(points) >= max_points:
                    continue
                voxel_keys.add(key)
                points.append(xyz[j])
                colors.append(rgb[y[j], x[j]])
            exported_frame = {
                "source": path.name,
                "image": name,
                "stamp": frame_stamp,
                "pose_source": pose_source,
                "pose_snapshot_digest": solution.digest if solution is not None else "",
                "calibration": calibration,
                **optional,
            }
            if solution is not None:
                exported_frame["T_component_camera"] = np.asarray(twc).tolist()
            exported_frames.append(exported_frame)
    if not points:
        raise ValueError("capture contains no valid metric depth")
    with (sparse / "points3D.bin").open("wb") as f:
        f.write(struct.pack("<Q", len(points)))
        for i, (p, c) in enumerate(zip(points, colors), 1):
            f.write(struct.pack("<Q3d3BdQ", i, *p, *c, 0.0, 0))
    (output / "swarmdeck.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "frame": "world" if solution is None else f"component:{solution.component_id}",
                "pose_frame": "world" if solution is None else solution.component_id,
                "units": "metres",
                "up": "z",
                "frames": len(frames),
                "seed_points": len(points),
                "capture": capture_metadata,
                "frame_records": exported_frames,
            },
            indent=2,
        )
    )
    return len(points)


def read_gaussian_ply(path):
    """Read UMAMI's little-endian attributes and optional creation-keyframe IDs."""
    with Path(path).open("rb") as f:
        properties = []
        count = None
        in_vertex = False
        fmt = None
        header_bytes = 0
        if f.readline() != b"ply\n":
            raise ValueError("not a PLY file")
        while True:
            line = f.readline(4096)
            header_bytes += len(line)
            if not line or header_bytes > 65536:
                raise ValueError("invalid PLY header")
            parts = line.decode("ascii").split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = parts[1]
            if parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    count = int(parts[2])
                elif count is None:
                    raise ValueError("vertex must be the first PLY element")
            if parts[0] == "property" and in_vertex:
                if len(parts) != 3:
                    raise ValueError("expected scalar Gaussian attributes")
                if parts[1] in ("float", "float32"):
                    scalar = "<f4"
                elif parts[1] in ("int", "int32") and parts[2] == "creation_kfid":
                    # UMAMI appends this metadata to each vertex. Preserve its
                    # width so subsequent rows remain aligned; rendering ignores it.
                    scalar = "<i4"
                else:
                    raise ValueError("unsupported Gaussian PLY property")
                properties.append((parts[2], scalar))
            if parts[0] == "end_header":
                break
        required = [
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
        names = [name for name, _scalar in properties]
        if (
            fmt != "binary_little_endian"
            or count is None
            or not 0 < count <= 10000000
            or not set(required) <= set(names)
            or len(set(names)) != len(names)
        ):
            raise ValueError("unsupported or incomplete Gaussian PLY")
        offset = f.tell()
    dtype = np.dtype(properties)
    if Path(path).stat().st_size < offset + count * dtype.itemsize:
        raise ValueError("truncated PLY")
    return np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,))


def convert_ply(source: Path, destination: Path, budget=150000):
    if not 1 <= budget <= 2000000:
        raise ValueError("Gaussian budget must be 1..2000000")
    vertices = read_gaussian_ply(source)
    # Opacity/volume importance selection; bounded display omits view-dependent SH terms.
    opacity = 1 / (1 + np.exp(-np.clip(vertices["opacity"], -30, 30)))
    scales = np.column_stack([vertices[f"scale_{i}"] for i in range(3)])
    xyz = np.column_stack([vertices[k] for k in ("x", "y", "z")])
    q = np.column_stack([vertices[f"rot_{i}"] for i in (1, 2, 3, 0)])
    rgb = 0.5 + 0.28209479177387814 * np.column_stack(
        [vertices[f"f_dc_{i}"] for i in range(3)]
    )
    valid = (
        np.isfinite(xyz).all(axis=1)
        & np.isfinite(scales).all(axis=1)
        & np.isfinite(q).all(axis=1)
        & np.isfinite(rgb).all(axis=1)
        & np.isfinite(opacity)
        & (opacity > 0.01)
        & (np.linalg.norm(q, axis=1) > 1e-8)
    )
    indices = np.flatnonzero(valid)
    if not len(indices):
        raise ValueError("no finite, visible Gaussians")
    score = opacity[indices] * np.exp(np.clip(scales[indices].sum(axis=1), -30, 20))
    if len(indices) > budget:
        indices = indices[np.argpartition(score, -budget)[-budget:]]
    q = q[indices]
    q /= np.linalg.norm(q, axis=1)[:, None]
    # UMAMI trains against normalized camera pixels. Three.js blends in linear RGB.
    rgb = np.clip(rgb, 0, 1)
    rgb = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
    records = np.column_stack(
        (
            xyz[indices],
            np.exp(np.clip(scales[indices], -12, 4)),
            q,
            np.clip(rgb[indices], 0, 1),
            opacity[indices],
        )
    ).astype("<f4")
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Replace atomically so the HTTP server never observes a partial reconstruction.
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as f:
        tmp = Path(f.name)
        f.write(struct.pack("<4sIII", b"SWGS", 1, len(records), 0))
        f.write(records.tobytes())
    os.replace(tmp, destination)
    return len(records)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    e = sub.add_parser("export")
    e.add_argument("capture", type=Path)
    e.add_argument("output", type=Path)
    e.add_argument("--stride", type=int, default=8)
    e.add_argument("--max-points", type=int, default=200000)
    e.add_argument("--pose-snapshot", type=Path)
    c = sub.add_parser("convert")
    c.add_argument("ply", type=Path)
    c.add_argument("output", type=Path)
    c.add_argument("--budget", type=int, default=150000)
    t = sub.add_parser("train")
    t.add_argument("--umami", type=Path, required=True)
    t.add_argument("--config", type=Path, required=True)
    t.add_argument("--dataset", type=Path, required=True)
    t.add_argument("--output", type=Path, required=True)
    t.add_argument("--publish", type=Path, required=True)
    t.add_argument("--budget", type=int, default=150000)
    a = p.parse_args()
    if a.command == "export":
        print(
            f"Exported {export_colmap(a.capture, a.output, a.stride, a.max_points, a.pose_snapshot)} seed points"
        )
    elif a.command == "convert":
        print(f"Published {convert_ply(a.ply,a.output,a.budget)} Gaussians")
    else:
        metadata = json.loads((a.dataset / "swarmdeck.json").read_text())
        frame = str(metadata.get("frame", ""))
        if frame != "world" and not frame.startswith("component:"):
            raise ValueError("world- or component-aligned SwarmDeck dataset required")
        a.output.mkdir(parents=True, exist_ok=False)
        subprocess.run(
            [
                str((a.umami / "bin/train_colmap").resolve()),
                str(a.config.resolve()),
                str(a.dataset.resolve()),
                str(a.output.resolve()),
                "no_viewer",
            ],
            check=True,
            cwd=a.umami,
        )
        candidates = list(a.output.rglob("point_cloud.ply"))
        if not candidates:
            raise RuntimeError("UMAMI produced no Gaussian PLY")
        final = max(candidates, key=lambda p: p.stat().st_mtime_ns)
        print(f"Published {convert_ply(final,a.publish,a.budget)} Gaussians from {final}")


if __name__ == "__main__":
    main()
