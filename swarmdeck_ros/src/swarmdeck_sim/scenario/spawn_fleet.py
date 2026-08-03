#!/usr/bin/env python3
"""Render N robot SDFs and spawn them into a running Gazebo world.

Start poses come from the study config, so `static` map merging has known
transforms and `auto` merging has ground truth to be scored against.

    python3 spawn_fleet.py --config ../../../../study/4robot.yaml
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

COLORS = [
    "0.22 0.74 0.97",  # robot_0
    "0.65 0.55 0.98",
    "0.20 0.83 0.60",
    "0.98 0.57 0.18",
    "0.96 0.45 0.71",
]

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE.parent.parent / "swarmdeck_description" / "urdf" / "robot.sdf.jinja"

# Vertical spread of a multi-ring mapping lidar, radians. Only used when
# lidar_rings > 1; a single ring is horizontal by definition.
LIDAR_VFOV = 0.26


def lidar_scan_fields(rings: int) -> dict[str, str]:
    """Vertical scan block for the mapping lidar.

    Gazebo distributes N samples evenly across [min_angle, max_angle] inclusive,
    so an even N leaves no ring at elevation 0 — every ring is then tilted, and a
    tilted ring vanishes beyond (band height / sin elevation). That is what turned
    the merged map into a hatched wedge (docs/KNOWN_ISSUES.md), so refuse an even
    count rather than emit a model that silently maps badly.
    """
    if rings < 1:
        raise ValueError(f"lidar_rings must be >= 1, got {rings}")
    if rings > 1 and rings % 2 == 0:
        raise ValueError(
            f"lidar_rings={rings} is even, so no ring sits at elevation 0 and any "
            f"height band truncates the 2D scan at short range. Use {rings + 1}."
        )
    if rings == 1:
        return {"LIDAR_RINGS": "1", "LIDAR_VMIN": "0", "LIDAR_VMAX": "0"}
    return {
        "LIDAR_RINGS": str(rings),
        "LIDAR_VMIN": f"{-LIDAR_VFOV:.5f}",
        "LIDAR_VMAX": f"{LIDAR_VFOV:.5f}",
    }


def render(name: str, color: str, rings: int = 1) -> str:
    sdf = TEMPLATE.read_text().replace("{{NAME}}", name).replace("{{COLOR}}", color)
    for key, value in lidar_scan_fields(rings).items():
        sdf = sdf.replace("{{" + key + "}}", value)
    return sdf


def yaw_quaternion(yaw: float) -> tuple[float, float]:
    """Return the normalized planar quaternion components (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def spawn(world: str, name: str, sdf: str, x: float, y: float, yaw: float) -> bool:
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as fh:
        fh.write(sdf)
        path = fh.name
    qz, qw = yaw_quaternion(yaw)
    cmd = [
        "gz", "service", "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req",
        f'sdf_filename: "{path}", name: "{name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: 0.15}}, '
        f'orientation: {{z: {qz:.9f}, w: {qw:.9f}}}}}',
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    ok = "true" in r.stdout.lower()
    print(f"[spawn] {name} at ({x}, {y}) -> {'ok' if ok else 'FAILED: ' + r.stderr.strip()}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--robots", type=int, default=None,
                    help="override the fleet count from the study config")
    ap.add_argument("--world", default="swarmdeck_indoor")
    ap.add_argument("--dry-run", action="store_true", help="render SDFs without spawning")
    ap.add_argument("--outdir", default=None, help="write rendered SDFs here")
    ap.add_argument("--lidar-rings", type=int, default=None,
                    help="vertical rings on the mapping lidar; 1 is 2D-only, "
                         "odd values > 1 also publish a usable 3D cloud")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    configured_count = int(cfg.get("fleet", {}).get("robot_count", 4))
    count = configured_count if args.robots is None else max(1, min(args.robots, 5))
    prefix = cfg.get("fleet", {}).get("robot_prefix", "robot_")
    starts = cfg.get("map", {}).get("start_poses", {})
    rings = (
        int(cfg.get("fleet", {}).get("lidar_rings", 1))
        if args.lidar_rings is None
        else args.lidar_rings
    )

    ok = True
    for i in range(min(count, 5)):
        name = f"{prefix}{i}"
        pose = starts.get(name, {"x": i * 3.0, "y": 0.0, "yaw": 0.0})
        sdf = render(name, COLORS[i % len(COLORS)], rings)

        if args.outdir:
            out = Path(args.outdir) / f"{name}.sdf"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(sdf)
            print(f"[render] {out}")

        if not args.dry_run:
            ok &= spawn(args.world, name, sdf, pose["x"], pose["y"], pose.get("yaw", 0.0))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
