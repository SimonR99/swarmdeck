#!/usr/bin/env python3
"""Generate real MOLA worker publications for the native MGG map probe.

Run in the mapping image with an empty writable --output directory. The MGG
image can then read each case without a mapper, ROS graph or motion controller.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import time
from pathlib import Path

import mola_planner_acceptance as fixture


def generate(output: Path, binary: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("fixture output must be empty")
    mapper, members, wall = fixture._fixture(output / "store")
    peer = output / "maps" / fixture.MISSION / "planner"
    component = fixture.component_id_for_anchor(wall)
    worker = fixture.MolaWorker(
        output / "maps",
        importer=binary,
        timeout_s=10.0,
        retry_s=0.0,
        planner_maps=True,
        mission_id=fixture.MISSION,
        max_output_bytes=64 * 1024 * 1024,
    )
    source = fixture.MolaDirectorySource(peer)
    view = fixture.IndexedMapView(resolution_m=0.2, max_query_s=0.5)
    deadline = time.monotonic() + 45.0
    cases = []

    def save_case(name, points):
        snapshot = json.loads((peer / "snapshot.json").read_text())
        manifest = next(
            m for m in snapshot["manifests"]
            if m["graph_revision"]["component_id"] == component
        )
        revision = manifest["graph_revision"]
        target = output / name
        shutil.copytree(peer, target)
        request = {
            "component_id": component,
            "epoch": revision["epoch"],
            "graph_revision": revision["revision"],
            "geometry_revision": manifest["geometry_revision"],
            "source_stamp_ns": max(s["observed_at_ns"] for s in manifest["submaps"]),
            "T_component_navigation": fixture.IDENTITY_SE3,
            "points": points,
            "expected_voxels": ["free", "occupied", "unknown"],
        }
        (target / "probe.json").write_text(json.dumps(request, allow_nan=False))
        cases.append(name)

    try:
        fixture._publish_and_refresh(worker, mapper, peer, source, view, component, deadline)
        # Sample inside a known occupied cell; arange's nominal y=0 is slightly
        # negative, so its wall return belongs to the adjacent negative cell.
        points = ((2.0, 0.0, 0.6), (3.1, 0.3, 0.7), (4.0, 0.0, 0.6))
        save_case("initial", points)
        first = json.loads((peer / "mola/index.json").read_text())

        # A new snapshot heartbeat must reuse the unchanged immutable grid.
        # Its original source-byte digest differs from the new index digest.
        fixture._publish_and_refresh(worker, mapper, peer, source, view, component, deadline)
        second = json.loads((peer / "mola/index.json").read_text())
        assert first["source_sha256"] != second["source_sha256"]
        assert first["artifacts"][0]["planner"] == second["artifacts"][0]["planner"]
        save_case("reused", points)

        corrected = fixture._yaw_pose(math.pi / 2, 1.0, 2.0)
        mapper.apply_solution(
            fixture.GraphSolution(
                fixture.ComponentRevision(component, 0, 2),
                wall,
                members,
                {key: corrected for key in members},
            )
        )
        fixture._publish_and_refresh(worker, mapper, peer, source, view, component, deadline)
        save_case("corrected", ((1.0, 3.5, 0.6), (1.0, 5.0, 0.6), (3.0, 0.0, 0.6)))
        return {"cases": cases, "component": component}
    finally:
        worker.close()
        mapper.store.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--binary", type=Path, default=Path(
        "/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import"
    ))
    args = parser.parse_args()
    print(json.dumps(generate(args.output, args.binary)))
