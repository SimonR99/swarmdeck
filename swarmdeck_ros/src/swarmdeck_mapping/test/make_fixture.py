#!/usr/bin/env python3
"""Create a real Python-store snapshot for the native importer smoke test."""

from pathlib import Path
import sys
import uuid

from autonomy.contracts import (
    IDENTITY_SE3,
    ZERO_COVARIANCE,
    Calibration,
    CalibratedCapture,
    DeskewStatus,
    KeyframeId,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore


def main() -> None:
    destination = Path(sys.argv[1])
    store = SubmapStore(destination)
    mapper = CorrectionAwareMapper(store)
    session = str(uuid.UUID("17df1d89-9b34-4be9-8842-d53ea4f0e40e"))
    keyframe = KeyframeId("smoke", session, 0)
    calibration = Calibration(
        "lidar-v1", "smoke/lidar", "x-forward/y-left/z-up", (), "none", (), IDENTITY_SE3
    )
    capture = CalibratedCapture(
        keyframe,
        0,
        1,
        calibration.sensor_frame,
        calibration.version,
        IDENTITY_SE3,
        ZERO_COVARIANCE,
        DeskewStatus.DESKEWED,
    )
    mapper.add_capture(
        capture,
        calibration,
        [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0.5, 0.5, 0.1]],
        colors_rgba=(
            [
                [255, 0, 0, 255],
                [0, 255, 0, 255],
                [0, 0, 255, 255],
                [255, 255, 255, 0],
                [128, 128, 128, 255],
            ]
            if "--color" in sys.argv[2:]
            else None
        ),
    )
    (destination / "snapshot.json").write_text(mapper.snapshot().to_json())


if __name__ == "__main__":
    main()
