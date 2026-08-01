#!/usr/bin/env python3
"""Deterministic indoor world generator.

One seed fixes wall layout, target placement and robot start poses, so two runs
with the same config are byte-identical (FR-S6, NFR-5, acceptance criterion 11).

    python3 generate_world.py --seed 20260801 --targets 8 -o ../worlds/indoor.sdf

Layout note: the building is deliberately COMPACT and ENCLOSED — 24 x 24 m with
rooms 6 m across, so every wall sits well inside lidar range. An earlier 40 x 40 m
world with a handful of thin partitions produced a useless map: from most poses
there was nothing within sensor range, so SLAM carved a giant free-space starburst
and had no structure to scan-match against. Enclosure is what makes 2D SLAM and
map registration work.
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

WALL_H = 2.4
WALL_T = 0.15

HALF = 12.0          # building half-extent, metres
CORR = 1.8           # corridor half-width
DOOR = 1.1           # door half-width

# Room dividers are deliberately IRREGULAR, and differ above vs below the
# corridor. A symmetric grid of identical rooms is pathological for map
# registration: shifting by one room width scores almost as well as the true
# alignment, so the merge locks onto the wrong offset. Asymmetry is what makes
# the correlation peak unique.
DIVIDERS_N = (-7.0, -1.0, 4.5)    # rooms above: 5.0, 6.0, 5.5, 7.5 m wide
DIVIDERS_S = (-4.5, 2.0, 7.0)     # rooms below: 7.5, 6.5, 5.0, 5.0 m wide


def _segments() -> list[tuple[float, float, float, float]]:
    """Axis-aligned wall rectangles (x0, y0, x1, y1)."""
    S: list[tuple[float, float, float, float]] = []
    h, t = HALF, WALL_T

    # Outer shell
    S += [
        (-h, -h, h, -h + t),
        (-h, h - t, h, h),
        (-h, -h, -h + t, h),
        (h - t, -h, h, h),
    ]

    def h_wall_with_doors(y: float, x0: float, x1: float, doors: list[float]) -> None:
        """Horizontal wall from x0..x1 at height y, with gaps centred on `doors`."""
        cuts: list[tuple[float, float]] = []
        for d in sorted(doors):
            cuts.append((d - DOOR, d + DOOR))
        cursor = x0
        for a, b in cuts:
            if a > cursor:
                S.append((cursor, y, a, y + t))
            cursor = max(cursor, b)
        if cursor < x1:
            S.append((cursor, y, x1, y + t))

    # Corridor walls, with a door into every room
    room_centres = [-9.0, -3.0, 3.0, 9.0]
    h_wall_with_doors(CORR, -h, h, room_centres)
    h_wall_with_doors(-CORR - t, -h, h, room_centres)

    # Room dividers above and below the corridor, at different x positions
    for x in DIVIDERS_N:
        S.append((x, CORR, x + t, h))
    for x in DIVIDERS_S:
        S.append((x, -h, x + t, -CORR))

    # Distinctive interior features, all different, so no two regions of the
    # building look alike to the registration correlator.
    S.append((-10.5, 6.5, -7.0, 6.5 + t))      # alcove, NW room
    S.append((-1.0, 8.5, 3.0, 8.5 + t))        # partial wall, N room
    S.append((6.0, 4.0, 6.0 + t, 9.0))         # stub, NE room
    S.append((-9.0, -6.0, -9.0 + t, -1.8))     # stub, SW room
    S.append((-2.0, -9.5, 2.0, -9.5 + t))      # partial wall, S room
    S.append((7.0, -5.0, 11.0, -5.0 + t))      # alcove, SE room
    S.append((9.0, -5.0, 9.0 + t, -1.8))       # SE subdivision

    return S


LAYOUT = _segments()

# Robot start poses live in the corridor, spaced along it.
START_POSES = [(-9.0, 0.0, 0.0), (-3.0, 0.0, 0.0), (3.0, 0.0, 3.1416), (9.0, 0.0, 3.1416), (0.0, 0.0, 1.5708)]


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


def blocked(x: float, y: float, margin: float = 0.7) -> bool:
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
    while len(placed) < args.targets and guard < 20000:
        guard += 1
        x, y = rng.uniform(-HALF + 1, HALF - 1), rng.uniform(-HALF + 1, HALF - 1)
        if blocked(x, y):
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < 6 for px, py in placed):
            continue
        placed.append((x, y))
    targets = "\n".join(target_model(i, x, y) for i, (x, y) in enumerate(placed))

    tpl = (Path(__file__).parent.parent / "worlds" / "indoor.sdf.jinja").read_text()
    out = Path(args.output)
    out.write_text(tpl.replace("{{WALLS}}", walls).replace("{{TARGETS}}", targets))
    print(f"[world] seed={args.seed} walls={len(LAYOUT)} targets={len(placed)} -> {out}")


if __name__ == "__main__":
    main()
