#!/usr/bin/env python3
"""Create a deterministic tiny COLMAP dataset for the native UMAMI smoke.

The fixture contains three calibrated 64x48 RGB-D views and is deliberately
small. It exercises SwarmDeck's COLMAP binary export and the pinned native
``train_colmap`` loader; it is not a quality or convergence benchmark.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import uuid

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from scripts.reconstruction.umami import export_colmap


def make_fixture(root: Path) -> tuple[Path, Path]:
    root = root.resolve()
    if root.exists():
        raise FileExistsError(f"refusing to replace existing fixture directory: {root}")
    capture = root / "capture"
    dataset = root / "dataset"
    capture.mkdir(parents=True)

    width, height = 64, 48
    session_id = str(uuid.UUID("00000000-0000-4000-8000-000000000001"))
    fx = fy = 52.0
    k = np.array(
        [[fx, 0, 0.5 * width], [0, fy, 0.5 * height], [0, 0, 1]], dtype=np.float64
    )
    # A shallow fronto-parallel surface gives every camera a stable metric
    # seed while the translation changes the projected colour pattern.
    yy, xx = np.mgrid[:height, :width]
    depth = np.full((height, width), 2.5, dtype=np.float32)
    for index, tx in enumerate((0.0, 0.08, 0.16)):
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[..., 0] = (xx * 4 + index * 17) % 256
        rgb[..., 1] = (yy * 5 + index * 29) % 256
        rgb[..., 2] = ((xx + yy) * 3 + index * 11) % 256
        twc = np.eye(4, dtype=np.float64)
        twc[0, 3] = tx
        np.savez_compressed(
            capture / f"frame-{index:04d}.npz",
            rgb=rgb,
            depth_m=depth,
            K=k,
            T_world_camera=twc,
            stamp=float(index) * 0.1,
            capture_id=f"native-smoke-{index}",
            robot_id="smoke",
            session_id=session_id,
            calibration_version="smoke-v1",
            optical_frame="smoke/camera_optical",
        )
    (capture / "capture_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "capture_id": f"native-umami-smoke-{session_id}",
                "robot_id": "smoke",
                "session_id": session_id,
                "component_id": "smoke-component",
                "calibration_version": "smoke-v1",
            },
            indent=2,
        )
    )
    export_colmap(capture, dataset, stride=4, max_points=5000)
    config = root / "umami_smoke.yaml"
    shutil.copy2(Path(__file__).with_name("umami_smoke.yaml"), config)
    return dataset, config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="new directory for fixture, dataset, and config")
    args = parser.parse_args()
    dataset, config = make_fixture(args.root)
    print(json.dumps({"dataset": str(dataset), "config": str(config)}))


if __name__ == "__main__":
    main()
