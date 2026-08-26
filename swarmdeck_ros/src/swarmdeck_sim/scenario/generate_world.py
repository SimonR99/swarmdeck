#!/usr/bin/env python3
"""Deterministic indoor world generator.

One seed fixes wall layout, target placement and robot start poses, so two runs
with the same config are byte-identical (FR-S6, NFR-5, acceptance criterion 11).

    python3 generate_world.py --seed 20260801 --targets 5 -o ../worlds/indoor.sdf

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


def target_model(i: int, x: float, y: float, yaw: float) -> str:
    """Recognisable yellow rubber duck, built from portable SDF primitives."""
    return f"""    <model name="rubber_duck_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.16 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="body_collision">
          <geometry><sphere><radius>0.17</radius></sphere></geometry>
        </collision>
        <visual name="body">
          <geometry><sphere><radius>0.17</radius></sphere></geometry>
          <material>
            <ambient>0.95 0.68 0.01 1</ambient>
            <diffuse>1.0 0.82 0.03 1</diffuse>
            <emissive>0.08 0.05 0.0 1</emissive>
          </material>
        </visual>
        <visual name="head">
          <pose>0.105 0 0.185 0 0 0</pose>
          <geometry><sphere><radius>0.115</radius></sphere></geometry>
          <material><ambient>0.95 0.68 0.01 1</ambient><diffuse>1.0 0.82 0.03 1</diffuse></material>
        </visual>
        <visual name="beak">
          <pose>0.215 0 0.165 0 0 0</pose>
          <geometry><box><size>0.10 0.105 0.045</size></box></geometry>
          <material><ambient>0.95 0.28 0.01 1</ambient><diffuse>1.0 0.38 0.02 1</diffuse></material>
        </visual>
        <visual name="eye_left">
          <pose>0.18 0.078 0.218 0 0 0</pose>
          <geometry><sphere><radius>0.018</radius></sphere></geometry>
          <material><ambient>0.01 0.01 0.015 1</ambient><diffuse>0.02 0.02 0.025 1</diffuse></material>
        </visual>
        <visual name="eye_right">
          <pose>0.18 -0.078 0.218 0 0 0</pose>
          <geometry><sphere><radius>0.018</radius></sphere></geometry>
          <material><ambient>0.01 0.01 0.015 1</ambient><diffuse>0.02 0.02 0.025 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def table_model(i: int, x: float, y: float, yaw: float = 0.0) -> str:
    parts = []
    for n, lx, ly in ((0, -0.55, -0.35), (1, -0.55, 0.35), (2, 0.55, -0.35), (3, 0.55, 0.35)):
        parts.append(f"""
        <collision name="leg_{n}_collision"><pose>{lx} {ly} 0.36 0 0 0</pose><geometry><box><size>0.08 0.08 0.72</size></box></geometry></collision>
        <visual name="leg_{n}"><pose>{lx} {ly} 0.36 0 0 0</pose><geometry><box><size>0.08 0.08 0.72</size></box></geometry><material><ambient>0.25 0.13 0.06 1</ambient><diffuse>0.42 0.22 0.09 1</diffuse></material></visual>""")
    return f"""    <model name="table_{i}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose><link name="link">
        <collision name="top_collision"><pose>0 0 0.76 0 0 0</pose><geometry><box><size>1.3 0.85 0.10</size></box></geometry></collision>
        <visual name="top"><pose>0 0 0.76 0 0 0</pose><geometry><box><size>1.3 0.85 0.10</size></box></geometry><material><ambient>0.34 0.18 0.08 1</ambient><diffuse>0.55 0.30 0.12 1</diffuse></material></visual>{''.join(parts)}
      </link></model>"""


def chair_model(i: int, x: float, y: float, yaw: float) -> str:
    return f"""    <model name="chair_{i}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose><link name="link">
        <collision name="seat_collision"><pose>0 0 0.45 0 0 0</pose><geometry><box><size>0.48 0.48 0.09</size></box></geometry></collision>
        <visual name="seat"><pose>0 0 0.45 0 0 0</pose><geometry><box><size>0.48 0.48 0.09</size></box></geometry><material><ambient>0.10 0.20 0.30 1</ambient><diffuse>0.16 0.36 0.54 1</diffuse></material></visual>
        <collision name="back_collision"><pose>-0.20 0 0.78 0 0 0</pose><geometry><box><size>0.08 0.48 0.62</size></box></geometry></collision>
        <visual name="back"><pose>-0.20 0 0.78 0 0 0</pose><geometry><box><size>0.08 0.48 0.62</size></box></geometry><material><ambient>0.10 0.20 0.30 1</ambient><diffuse>0.16 0.36 0.54 1</diffuse></material></visual>
        <visual name="legs"><pose>0 0 0.21 0 0 0</pose><geometry><box><size>0.38 0.38 0.42</size></box></geometry><material><ambient>0.15 0.16 0.18 1</ambient><diffuse>0.22 0.24 0.27 1</diffuse></material></visual>
      </link></model>"""


def painting_model(i: int, x: float, y: float, z: float, yaw: float, color: str) -> str:
    return f"""    <model name="painting_{i}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose><link name="link">
        <visual name="frame"><geometry><box><size>1.15 0.055 0.78</size></box></geometry><material><ambient>0.07 0.045 0.025 1</ambient><diffuse>0.12 0.07 0.03 1</diffuse></material></visual>
        <visual name="canvas"><pose>0 -0.032 0 0 0 0</pose><geometry><box><size>0.98 0.018 0.61</size></box></geometry><material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link></model>"""


def plant_model(i: int, x: float, y: float) -> str:
    return f"""    <model name="plant_{i}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="link">
        <collision name="pot_collision"><pose>0 0 0.20 0 0 0</pose><geometry><cylinder><radius>0.20</radius><length>0.40</length></cylinder></geometry></collision>
        <visual name="pot"><pose>0 0 0.20 0 0 0</pose><geometry><cylinder><radius>0.20</radius><length>0.40</length></cylinder></geometry><material><ambient>0.36 0.12 0.06 1</ambient><diffuse>0.58 0.20 0.09 1</diffuse></material></visual>
        <visual name="leaves"><pose>0 0 0.62 0 0 0</pose><geometry><sphere><radius>0.32</radius></sphere></geometry><material><ambient>0.05 0.28 0.12 1</ambient><diffuse>0.08 0.48 0.20 1</diffuse></material></visual>
      </link></model>"""


_FURNITURE_EMITTERS = {
    "table": lambda i, *a: table_model(i, *a),
    "chair": lambda i, *a: chair_model(i, *a),
    "painting": lambda i, *a: painting_model(i, *a),
    "plant": lambda i, *a: plant_model(i, *a),
}


# Deterministic furniture, deep inside rooms so the central corridor and the
# robot start poses stay clear. It gives the cameras realistic visual structure
# and gives SLAM non-repetitive landmarks.
#
# A table of (kind, index, args) rather than a list of rendered SDF, because
# the ARGoS backend builds the same furniture as glTF geometry
# (scenario/make_argos_world.py). Two hand-maintained copies of a floor plan is
# how the two backends stop being comparable.
FURNITURE: list[tuple[str, int, tuple]] = [
    ("table", 0, (-9.2, 8.3, 0.15)),
    ("table", 1, (-1.8, -7.1, -0.2)),
    ("table", 2, (8.2, 7.4, 0.4)),
    ("chair", 0, (-9.2, 7.25, 1.57)),
    ("chair", 1, (-9.2, 9.35, -1.57)),
    ("chair", 2, (-2.8, -7.1, 0.0)),
    ("chair", 3, (-0.8, -7.1, 3.14159)),
    ("chair", 4, (8.2, 6.35, 1.57)),
    ("chair", 5, (8.2, 8.45, -1.57)),
    ("painting", 0, (-9.0, 11.90, 1.35, 0.0, "0.10 0.42 0.72")),
    ("painting", 1, (1.2, -11.90, 1.40, 0.0, "0.78 0.24 0.18")),
    ("painting", 2, (11.90, 7.0, 1.30, 1.5708, "0.24 0.62 0.40")),
    ("plant", 0, (-5.7, 10.7)),
    ("plant", 1, (4.9, -10.5)),
    ("plant", 2, (10.6, 2.8)),
]


def place_targets(seed: int, count: int) -> list[tuple[float, float, float]]:
    """Seeded detection-target poses, as (x, y, yaw).

    The first one is deliberately in robot_0's opening field of view, which is
    what makes the whole camera -> detector -> adapter -> UI path testable
    without driving anywhere; the rest are scattered through the rooms so that
    finding them requires exploring.

    Shared with the ARGoS backend, which places the same positions as
    photorealism props. Both backends must agree, or a detection scored
    against one world is being compared with a run of a different one.
    """
    rng = random.Random(seed)
    placed: list[tuple[float, float]] = [(-7.45, 0.45)] if count > 0 else []
    guard = 0
    while len(placed) < count and guard < 20000:
        guard += 1
        x, y = rng.uniform(-HALF + 1, HALF - 1), rng.uniform(-HALF + 1, HALF - 1)
        if blocked(x, y):
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < 6 for px, py in placed):
            continue
        placed.append((x, y))
    return [
        (x, y, 3.14159 if i == 0 else rng.uniform(-3.14159, 3.14159))
        for i, (x, y) in enumerate(placed)
    ]


def blocked(x: float, y: float, margin: float = 0.7) -> bool:
    for x0, y0, x1, y1 in LAYOUT:
        if x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin:
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=20260801)
    ap.add_argument("--targets", type=int, default=5)
    ap.add_argument("-o", "--output", default="indoor.sdf")
    args = ap.parse_args()

    walls = "\n".join(wall_model(i, *w) for i, w in enumerate(LAYOUT))
    placed = place_targets(args.seed, args.targets)
    targets = "\n".join(
        target_model(i, x, y, yaw) for i, (x, y, yaw) in enumerate(placed)
    )

    furniture = "\n".join(
        _FURNITURE_EMITTERS[kind](i, *args) for kind, i, args in FURNITURE
    )

    tpl = (Path(__file__).parent.parent / "worlds" / "indoor.sdf.jinja").read_text()
    out = Path(args.output)
    out.write_text(
        tpl.replace("{{WALLS}}", walls)
        .replace("{{FURNITURE}}", furniture)
        .replace("{{TARGETS}}", targets)
    )
    print(f"[world] seed={args.seed} walls={len(LAYOUT)} targets={len(placed)} -> {out}")


if __name__ == "__main__":
    main()
