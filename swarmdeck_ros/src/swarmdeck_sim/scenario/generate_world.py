#!/usr/bin/env python3
"""Deterministic world generator.

One seed fixes wall layout, target placement and robot start poses, so two runs
with the same config are byte-identical (FR-S6, NFR-5, acceptance criterion 11).

    python3 generate_world.py --seed 20260801 --targets 8 -o ../worlds/indoor.sdf
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

WALL_H = 1.6
WALL_T = 0.15

# Fixed floor plan: (x0, y0, x1, y1) in metres. Rooms and corridors.
LAYOUT = [
    (-20, -20, 20, -19.85),   # outer
    (-20, 19.85, 20, 20),
    (-20, -20, -19.85, 20),
    (19.85, -20, 20, 20),
    (-8, -20, -7.85, -4),     # interior partitions
    (-8, 4, -7.85, 20),
    (6, -20, 6.15, -6),
    (6, 2, 6.15, 20),
    (-7.85, 2, 6, 2.15),
    (6.15, 9, 20, 9.15),
    (-20, -6, -14, -5.85),
    (12, -12, 12.15, -2),
]


def wall_model(i: int, x0: float, y0: float, x1: float, y1: float) -> str:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    sx, sy = max(abs(x1 - x0), WALL_T), max(abs(y1 - y0), WALL_T)
    return f"""    <model name="wall_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {WALL_H / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
          <material>
            <ambient>0.30 0.34 0.40 1</ambient>
            <diffuse>0.42 0.47 0.55 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def target_model(i: int, x: float, y: float) -> str:
    """Search target — a small yellow cylinder ('duck')."""
    return f"""    <model name="target_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.12 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.11</radius><length>0.24</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.11</radius><length>0.24</length></cylinder></geometry>
          <material>
            <ambient>0.9 0.7 0.05 1</ambient>
            <diffuse>1.0 0.78 0.1 1</diffuse>
            <emissive>0.35 0.26 0.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def blocked(x: float, y: float, margin: float = 0.8) -> bool:
    for x0, y0, x1, y1 in LAYOUT:
        if x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin:
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=20260801)
    ap.add_argument("--targets", type=int, default=8)
    ap.add_argument("-o", "--output", default="indoor.sdf")
    args = ap.parse_args()

    rng = random.Random(args.seed)

    walls = "\n".join(wall_model(i, *w) for i, w in enumerate(LAYOUT))

    placed: list[tuple[float, float]] = []
    guard = 0
    while len(placed) < args.targets and guard < 10000:
        guard += 1
        x, y = rng.uniform(-18, 18), rng.uniform(-18, 18)
        if blocked(x, y):
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < 9 for px, py in placed):
            continue
        placed.append((x, y))
    targets = "\n".join(target_model(i, x, y) for i, (x, y) in enumerate(placed))

    tpl = (Path(__file__).parent.parent / "worlds" / "indoor.sdf.jinja").read_text()
    out = Path(args.output)
    out.write_text(tpl.replace("{{WALLS}}", walls).replace("{{TARGETS}}", targets))
    print(f"[world] seed={args.seed} walls={len(LAYOUT)} targets={len(placed)} -> {out}")


if __name__ == "__main__":
    main()
