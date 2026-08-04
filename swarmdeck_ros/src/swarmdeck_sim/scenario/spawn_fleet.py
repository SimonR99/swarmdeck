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
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any, Mapping

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

# +/- 22.5 deg, the vertical spread of a typical 32-beam scanning lidar.
LIDAR_VFOV_DEFAULT = 0.3927


@dataclass(frozen=True)
class LidarSpec:
    """Mapping-lidar geometry, in the form the SDF template needs.

    Every field is validated on construction, because each of these has a
    failure mode that is silent in Gazebo: the model spawns, the sensor
    publishes, and the map is quietly worse.
    """

    h_samples: int = 1800
    rings: int = 1
    vfov: float = 0.0        # half-angle, radians; 0 for a single horizontal ring
    range_max: float = 30.0
    rate: float = 10.0

    def __post_init__(self) -> None:
        if self.h_samples < 8:
            raise ValueError(f"lidar h_samples must be >= 8, got {self.h_samples}")
        if self.rings < 1:
            raise ValueError(f"lidar rings must be >= 1, got {self.rings}")
        if self.rings > 1 and self.rings % 2 == 0:
            # Gazebo distributes N samples evenly across [min_angle, max_angle]
            # INCLUSIVE, so an even N leaves no ring at elevation 0 — every ring
            # is tilted, and a tilted ring at elevation e vanishes beyond
            # (band half-height / sin e) once a height band slices it. That is
            # what turned the merged map into a hatched wedge; see
            # docs/KNOWN_ISSUES.md. Refuse rather than map badly in silence.
            raise ValueError(
                f"lidar rings={self.rings} is even, so no ring sits at elevation 0 "
                f"and any height band truncates the 2D scan at short range. "
                f"Use {self.rings + 1}."
            )
        if self.rings == 1 and self.vfov:
            raise ValueError(
                f"a single-ring lidar is horizontal by definition, but vfov is "
                f"{self.vfov}. Set vfov to 0, or raise rings to an odd value > 1."
            )
        if self.rings > 1 and self.vfov <= 0:
            raise ValueError(f"rings={self.rings} needs a vfov > 0, got {self.vfov}")
        if self.range_max <= 0:
            raise ValueError(f"lidar range_max must be > 0, got {self.range_max}")
        if self.rate <= 0:
            raise ValueError(f"lidar rate must be > 0, got {self.rate}")

    @property
    def h_step_deg(self) -> float:
        """Angular step between adjacent rays.

        This is the number that decides whether distant walls come out
        continuous or dotted: two adjacent returns land `range * radians(step)`
        apart, so against a 5 cm grid cell a 1.003 deg step (the 360-sample
        lidar SwarmDeck shipped) separates them by one cell at 2.9 m and three
        cells at 8.6 m. Real units are 0.1-0.4 deg.

        Divided by N-1, not N, for the same reason the ring count must be odd:
        Gazebo spreads samples across [min_angle, max_angle] INCLUSIVE. On a full
        revolution that makes the first and last ray the same bearing, so 360
        samples buy 359 distinct ones — which is where the 1.003 deg in
        docs/KNOWN_ISSUES.md comes from rather than a round 1.000.
        """
        return 360.0 / (self.h_samples - 1)

    def fields(self) -> dict[str, str]:
        vmin, vmax = (-self.vfov, self.vfov) if self.rings > 1 else (0.0, 0.0)
        return {
            "LIDAR_HSAMPLES": str(self.h_samples),
            "LIDAR_RINGS": str(self.rings),
            "LIDAR_VMIN": f"{vmin:.5f}",
            "LIDAR_VMAX": f"{vmax:.5f}",
            "LIDAR_RANGE_MAX": f"{self.range_max:g}",
            "LIDAR_RATE": f"{self.rate:g}",
        }


# Named sensor profiles. The point of the table is that swapping the fleet's
# lidar is a one-line config change rather than an SDF edit, so what gets tuned
# in simulation can be re-pointed at whatever hardware is actually bought.
#
# `vlp16` and `os1_32` are approximations of uniformly spaced spinning units and
# are honest as ring models. A Livox Mid-360 deliberately has NO profile here: it
# scans a non-repetitive rosette, which a fixed ring count does not represent at
# all, and pretending otherwise would make simulated results transfer badly.
LIDAR_PROFILES: dict[str, LidarSpec] = {
    # Exactly what SwarmDeck shipped. Kept as the A/B control for measuring any
    # of the changes below — not as a thing to run.
    "legacy_360": LidarSpec(h_samples=360, rings=1, vfov=0.0, range_max=16.0),
    # 0.2 deg planar scan: same 2D pipeline, walls that stay continuous to the
    # far end of a 24 m building.
    "generic_2d": LidarSpec(h_samples=1800, rings=1, vfov=0.0, range_max=30.0),
    # Generic 32-beam 3D unit, the target for the IMU + 3D lidar stack.
    # 1024 columns rather than the 2D profiles' 1800: this is what a real
    # 32-beam unit does (an OS1-32 is 32x1024 at 10 Hz), and the product is what
    # costs. At 1800 the fleet emits 4 x 594k points/s, which starved
    # ros_gz_bridge badly enough that clouds arrived at 2-3 Hz instead of 10 and
    # lidar odometry could not keep a lock. Raise `rings` before `h_samples` if
    # more fidelity is needed: vertical structure is what ICP registers on.
    "generic_32": LidarSpec(
        h_samples=1024, rings=33, vfov=LIDAR_VFOV_DEFAULT, range_max=30.0
    ),
    "vlp16": LidarSpec(h_samples=1800, rings=17, vfov=0.2618, range_max=100.0),
    "os1_32": LidarSpec(
        h_samples=1024, rings=33, vfov=LIDAR_VFOV_DEFAULT, range_max=100.0
    ),
}

DEFAULT_LIDAR_PROFILE = "generic_2d"

_INT_FIELDS = {"h_samples", "rings"}
_SPEC_FIELDS = {f.name for f in fields(LidarSpec)}


def lidar_spec(
    fleet_cfg: Mapping[str, Any] | None, rings_override: int | None = None
) -> LidarSpec:
    """Resolve `fleet.lidar` — a profile name plus per-field overrides.

    ```yaml
    fleet:
      lidar:
        profile: generic_32     # see LIDAR_PROFILES
        h_samples: 2048         # any field may be overridden individually
    ```

    `fleet.lidar_rings` is the pre-profile spelling and still works: it sets the
    ring count on top of whichever profile is in force.
    """
    fleet_cfg = fleet_cfg or {}
    block = dict(fleet_cfg.get("lidar") or {})
    name = block.pop("profile", DEFAULT_LIDAR_PROFILE)
    if name not in LIDAR_PROFILES:
        raise ValueError(
            f"unknown lidar profile {name!r}; available: {sorted(LIDAR_PROFILES)}"
        )
    spec = LIDAR_PROFILES[name]

    legacy_rings = fleet_cfg.get("lidar_rings")
    if legacy_rings is not None and "rings" not in block:
        block["rings"] = legacy_rings
    if rings_override is not None:
        block["rings"] = rings_override

    unknown = set(block) - _SPEC_FIELDS
    if unknown:
        raise ValueError(
            f"unknown fleet.lidar keys {sorted(unknown)}; expected "
            f"'profile' or any of {sorted(_SPEC_FIELDS)}"
        )
    block = {
        key: (int(value) if key in _INT_FIELDS else float(value))
        for key, value in block.items()
    }

    # Ring count and vfov are not independent, and LidarSpec (correctly) refuses
    # the two contradictory combinations. Rather than make every config restate
    # both, fill in the one that was not asked for.
    rings = block.get("rings", spec.rings)
    if "vfov" not in block:
        if rings == 1:
            block["vfov"] = 0.0
        elif not spec.vfov:
            # Raising a planar profile to several rings — the `lidar_rings: 9`
            # spelling does exactly this — has to pick a spread from somewhere.
            block["vfov"] = LIDAR_VFOV_DEFAULT
    return replace(spec, **block)


def render(name: str, color: str, spec: LidarSpec | None = None) -> str:
    sdf = TEMPLATE.read_text().replace("{{NAME}}", name).replace("{{COLOR}}", color)
    for key, value in (spec or LidarSpec()).fields().items():
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
                    help="override the ring count from the config; 1 is 2D-only, "
                         "odd values > 1 also publish a usable 3D cloud")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    fleet_cfg = cfg.get("fleet", {})
    configured_count = int(fleet_cfg.get("robot_count", 4))
    count = configured_count if args.robots is None else max(1, min(args.robots, 5))
    prefix = fleet_cfg.get("robot_prefix", "robot_")
    starts = cfg.get("map", {}).get("start_poses", {})
    spec = lidar_spec(fleet_cfg, args.lidar_rings)
    print(
        f"[spawn] lidar {spec.h_samples} samples/rev ({spec.h_step_deg:.3f} deg), "
        f"{spec.rings} ring(s), {spec.range_max:g} m, {spec.rate:g} Hz"
    )

    ok = True
    for i in range(min(count, 5)):
        name = f"{prefix}{i}"
        pose = starts.get(name, {"x": i * 3.0, "y": 0.0, "yaw": 0.0})
        sdf = render(name, COLORS[i % len(COLORS)], spec)

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
