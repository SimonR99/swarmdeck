#!/usr/bin/env python3
"""Compare persistent corrections with full imports using a deterministic cloud.

Run with repository PYTHONPATH and the built native importer. Both paths include
metric-map serialization, so this measures the artifact worker's cost rather
than just an in-memory pose update. No ROS, server, or robot motion is required.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import tempfile
import time
import uuid

import numpy as np

from autonomy.contracts import (
    IDENTITY_SE3, ZERO_COVARIANCE, Calibration, CalibratedCapture,
    DeskewStatus, KeyframeId,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from deploy.autonomy.mola_process import PersistentImporter


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--points", type=int, default=100_000)
    parser.add_argument("--revisions", type=int, default=20)
    args = parser.parse_args()
    if not 1 <= args.points <= 1_000_000 or not 3 <= args.revisions <= 100:
        parser.error("points must be 1..1000000 and revisions 3..100")

    with tempfile.TemporaryDirectory(prefix="mola-benchmark-") as temporary:
        root = Path(temporary)
        mapper = CorrectionAwareMapper(SubmapStore(root))
        calibration = Calibration(
            "benchmark-v1", "benchmark/lidar", "x-forward/y-left/z-up", (),
            "none", (), IDENTITY_SE3,
        )
        capture = CalibratedCapture(
            KeyframeId("benchmark", str(uuid.UUID(int=1)), 0), 0, 1,
            calibration.sensor_frame, calibration.version, IDENTITY_SE3,
            ZERO_COVARIANCE, DeskewStatus.DESKEWED,
        )
        points = np.random.default_rng(1).uniform(-10, 10, (args.points, 3))
        mapper.add_capture(capture, calibration, points)
        snapshot = mapper.snapshot().to_dict()
        native = PersistentImporter(
            args.binary, 30, max_points_per_map=2_000_000,
            max_resident_points=8_000_000, max_maps=256,
            max_output_bytes=268_435_456,
        )
        persistent, oneshot, resident_kib = [], [], []
        try:
            for revision in range(args.revisions + 1):
                manifest = snapshot["manifests"][0]
                manifest["graph_revision"]["revision"] = revision
                for submap in manifest["submaps"]:
                    submap["pose_revision"] = dict(manifest["graph_revision"])
                    submap["T_component_submap"][0][3] = revision / 10
                raw = json.dumps(snapshot, sort_keys=True, separators=(",", ":")).encode()
                snapshot["snapshot_id"] = hashlib.sha256(raw).hexdigest()
                raw = json.dumps(snapshot, sort_keys=True, separators=(",", ":")).encode()
                source = root / "snapshot.json"
                source.write_bytes(raw)
                output = root / f"persistent-{revision}.metricmap"
                start = time.perf_counter()
                response = native.apply({
                    "mode": "replace" if revision == 0 else "pose_only",
                    "map_id": "benchmark", "snapshot_path": str(source),
                    "snapshot_sha256": hashlib.sha256(raw).hexdigest(),
                    "chunks_dir": str(root / "chunks"), "output_path": str(output),
                })
                elapsed = time.perf_counter() - start
                assert response["points"] == args.points
                if revision:
                    persistent.append(elapsed)
                # Test-only process inspection; Linux workstation/container.
                assert native._process is not None
                for line in Path(f"/proc/{native._process.pid}/status").read_text().splitlines():
                    if line.startswith("VmRSS:"):
                        resident_kib.append(int(line.split()[1]))
                output.unlink()
                if 0 < revision <= 5:
                    output = root / f"oneshot-{revision}.metricmap"
                    start = time.perf_counter()
                    subprocess.run(
                        (str(args.binary), str(source), str(root / "chunks"), str(output)),
                        check=True, timeout=30, stdout=subprocess.DEVNULL,
                    )
                    oneshot.append(time.perf_counter() - start)
                    output.unlink()
        finally:
            native.close()
        print(json.dumps({
            "points": args.points, "pose_revisions": len(persistent),
            "persistent_median_ms": 1000 * statistics.median(persistent),
            "persistent_max_ms": 1000 * max(persistent),
            "oneshot_samples": len(oneshot),
            "oneshot_median_ms": 1000 * statistics.median(oneshot),
            "native_rss_first_kib": resident_kib[0],
            "native_rss_last_kib": resident_kib[-1],
            "native_rss_max_kib": max(resident_kib),
        }, sort_keys=True))


if __name__ == "__main__":
    main()
