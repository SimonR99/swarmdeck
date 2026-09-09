#!/usr/bin/env python3
"""Executable capture -> correction -> canonical manifest example."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import uuid

from autonomy.contracts import (
    IDENTITY_SE3,
    ZERO_COVARIANCE,
    Calibration,
    CalibratedCapture,
    ComponentRevision,
    DeskewStatus,
    GraphSolution,
    KeyframeId,
    component_id_for_anchor,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore


def translated(x: float):
    matrix = [list(row) for row in IDENTITY_SE3]
    matrix[0][3] = x
    return tuple(tuple(row) for row in matrix)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "store", type=Path, help="new or existing durable map-store directory"
    )
    args = parser.parse_args()
    session = str(uuid.uuid4())
    keyframe = KeyframeId("robot_0", session, 0)
    calibration = Calibration(
        "lidar-v1",
        "robot_0/lidar",
        "x-forward/y-left/z-up",
        (),
        "none",
        (),
        IDENTITY_SE3,
    )
    capture = CalibratedCapture(
        keyframe,
        1_000,
        1_010,
        calibration.sensor_frame,
        calibration.version,
        translated(1.0),
        ZERO_COVARIANCE,
        DeskewStatus.DESKEWED,
    )
    mapper = CorrectionAwareMapper(SubmapStore(args.store))
    mapper.add_capture(capture, calibration, [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    component = component_id_for_anchor(keyframe)
    mapper.apply_solution(
        GraphSolution(
            ComponentRevision(component, 0, 1),
            keyframe,
            (keyframe,),
            {keyframe: translated(2.0)},
        )
    )
    snapshot = mapper.snapshot_dict()
    (args.store / "snapshot.json").write_text(
        json.dumps(snapshot, sort_keys=True, separators=(",", ":")) + "\n"
    )
    print(json.dumps(snapshot, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
