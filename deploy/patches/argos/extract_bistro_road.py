"""Extract the manhole contact regression from the locally installed Bistro asset."""

import argparse
from pathlib import Path
import sys

import numpy as np


def extract_road(asset: Path, output: Path) -> None:
    """Write intersecting road triangles as packed float32 XYZ coordinates."""
    # Reuse the scenario loader's glTF transforms without requiring ROS.
    scenario = (
        Path(__file__).resolve().parents[3] / "swarmdeck_ros/src/swarmdeck_sim/scenario"
    )
    sys.path.insert(0, str(scenario))
    try:
        from mesh_surface import GlbGeometry
    finally:
        sys.path.pop(0)

    lower = np.array([-13.5, 0.5, -0.2])
    upper = np.array([-10.5, 4.5, 0.25])
    selected = []
    for triangles in GlbGeometry(str(asset)).triangles():
        triangles[:, :, 2] -= 0.3  # Bistro scene translation.
        overlaps = np.all(triangles.min(axis=1) < upper, axis=1) & np.all(
            triangles.max(axis=1) > lower, axis=1
        )
        if overlaps.any():
            selected.append(triangles[overlaps])
    if not selected:
        raise ValueError(f"No road triangles at the regression site in {asset}")
    np.concatenate(selected).astype("float32").tofile(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    extract_road(args.asset, args.output)


if __name__ == "__main__":
    main()
