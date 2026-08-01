#!/usr/bin/env python3
"""Render N robot SDFs and spawn them into a running Gazebo world.

Start poses come from the study config, so `static` map merging has known
transforms and `auto` merging has ground truth to be scored against.

    python3 spawn_fleet.py --config ../../../../study/4robot.yaml
"""

from __future__ import annotations

import argparse
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


def render(name: str, color: str) -> str:
    return TEMPLATE.read_text().replace("{{NAME}}", name).replace("{{COLOR}}", color)


def spawn(world: str, name: str, sdf: str, x: float, y: float, yaw: float) -> bool:
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as fh:
        fh.write(sdf)
        path = fh.name
    cmd = [
        "gz", "service", "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req",
        f'sdf_filename: "{path}", name: "{name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: 0.15}}, '
        f'orientation: {{z: {yaw / 2:.6f}, w: 1.0}}}}',
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    ok = "true" in r.stdout.lower()
    print(f"[spawn] {name} at ({x}, {y}) -> {'ok' if ok else 'FAILED: ' + r.stderr.strip()}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--world", default="swarmdeck_indoor")
    ap.add_argument("--dry-run", action="store_true", help="render SDFs without spawning")
    ap.add_argument("--outdir", default=None, help="write rendered SDFs here")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    count = int(cfg.get("fleet", {}).get("robot_count", 4))
    prefix = cfg.get("fleet", {}).get("robot_prefix", "robot_")
    starts = cfg.get("map", {}).get("start_poses", {})

    ok = True
    for i in range(min(count, 5)):
        name = f"{prefix}{i}"
        pose = starts.get(name, {"x": i * 3.0, "y": 0.0, "yaw": 0.0})
        sdf = render(name, COLORS[i % len(COLORS)])

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
