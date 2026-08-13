#!/usr/bin/env python3
"""Render N robot SDFs and spawn them into a running Gazebo world.

Start poses come from the study config, so `static` map merging has known
transforms and `auto` merging has ground truth to be scored against.

    python3 spawn_fleet.py --config ../../../../study/4robot.yaml
"""

from __future__ import annotations

import argparse
import math
import re
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
# Named hardware robots (and Spot's chassis) keep a stable Gazebo colour.
IDENTITY_COLORS = {
    "spot": "0.79 0.63 0.00",    # yellow
    "botman": "0.00 0.48 1.00",  # blue
    "aslan": "0.88 0.44 0.00",   # orange
}
TYPE_COLORS = {
    "spot": IDENTITY_COLORS["spot"],
}


def color_for(name: str, index: int, profile: str | None = None) -> str:
    """Gazebo body colour: named robots first, then chassis, then list order."""
    stem = re.sub(r"_\d+$", "", name)
    if stem in IDENTITY_COLORS:
        return IDENTITY_COLORS[stem]
    if profile in TYPE_COLORS:
        return TYPE_COLORS[profile]
    return COLORS[index % len(COLORS)]

HERE = Path(__file__).resolve().parent
DESCRIPTION = HERE.parent.parent / "swarmdeck_description" / "urdf"
TEMPLATE = DESCRIPTION / "robot.sdf.jinja"
CHASSIS_DIR = DESCRIPTION / "chassis"

# Height above the floor at which every robot's bumper scan is taken, metres.
#
# One number for the whole fleet, and it has to clear the SHORTEST robot's roof
# from the TALLEST robot's point of view. A Scout Mini is 0.245 m tall overall;
# a Spot's body floats at 0.40-0.60 m. Put the scan at Spot's body height and it
# misses the Scout entirely; put it at 0.05 m and it grazes the floor on a ramp.
# 0.15 m sits inside every platform's collision volume (Spot's via the leg
# envelope in chassis/spot.xml) with margin on both sides.
PROXIMITY_SCAN_HEIGHT = 0.15

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


@dataclass(frozen=True)
class RobotSpec:
    """One platform's physical facts, in the form the SDF and Nav2 both need.

    Everything here is a nominal datasheet figure or a mount choice, NOT a
    calibration. On hardware these come from the unit's URDF; the numbers below
    describe the simulated stand-in and nothing more.

    `base_height` is what ties the file together: it is how far base_link floats
    above the floor, set by the drive geometry in the chassis fragment. The
    proximity sensor's z is derived from it so that every robot scans at the
    same absolute height (PROXIMITY_SCAN_HEIGHT) no matter how tall it is —
    which is the only way a tall robot and a short one can see each other.
    """

    chassis: str                # chassis/<name>.xml
    robot_type: str             # reported to the backend at `hello`
    length: float               # metres, for the record and for footprint maths
    width: float
    base_height: float          # base_link height above the floor
    lidar_x: float              # mapping lidar mount, relative to base_link
    lidar_z: float
    camera_x: float
    camera_z: float
    prox_x: float               # bumper scan, forward of base_link
    # The highest thing on the robot's own back that the mapping lidar has to
    # see OVER, and how far it reaches from base_link. A multi-ring lidar sweeps
    # downwards as well as outwards, so a mount that clears the deck at zero
    # range still buries the lower rings in it further out.
    deck_top: float = 0.0
    deck_half_length: float = 0.0
    prox_range_max: float = 6.0
    prox_samples: int = 181
    prox_fov: float = math.pi   # total horizontal sweep, radians

    @property
    def footprint_radius(self) -> float:
        """Circumscribed radius of the chassis rectangle.

        Nav2 is configured with a circular footprint, so this must circumscribe
        rather than inscribe: an inscribed radius lets a corner of a 1.02 m
        Bunker clip a wall the planner believed was clear.
        """
        return math.hypot(self.length, self.width) / 2.0

    @property
    def prox_z(self) -> float:
        return PROXIMITY_SCAN_HEIGHT - self.base_height

    def min_lidar_z(self, vfov: float) -> float:
        """Lowest mount height at which no ring lands on the robot's own deck.

        Gazebo's `gpu_lidar` raytraces the RENDER scene, so it hits visuals —
        the decks, masts and (on Spot) legs, none of which are collision
        geometry. The steepest downward ring falls `tan(vfov)` per metre of
        range, so it meets deck height at `(lidar_z - deck_top)/tan(vfov)`; the
        mount has to be high enough that this happens beyond the deck's far
        edge.

        Getting this wrong does not look like a sensor fault. The robot simply
        reports a ring of obstacles at its own radius, every direction is
        blocked, and it sits still having "found nowhere to go" — which is what
        a Spot with its lidar 0.17 m too low actually did.
        """
        if not self.deck_top or vfov <= 0.0:
            return 0.0
        reach = self.deck_half_length + abs(self.lidar_x)
        return self.deck_top + math.tan(vfov) * reach

    @property
    def footprint(self) -> str:
        """The chassis rectangle, as Nav2's `footprint` polygon parameter.

        Nav2 accepts either `robot_radius` (a circle) or `footprint` (a polygon),
        and the difference decides whether these robots can plan through a door.
        A circle has to circumscribe, so an AgileX Bunker that is 0.778 m wide
        becomes a 1.285 m disc, and every cell within 0.643 m of a wall is
        lethal. Given the polygon, Nav2 computes an inscribed radius of 0.389 m
        — the actual half-width — and the same door is passable.

        Rectangles rather than the true silhouette on purpose: the inscribed and
        circumscribed radii are what the costmap actually uses, and a rectangle
        gets both right without paying for a polygon collision check per cell.
        """
        half_l, half_w = self.length / 2.0, self.width / 2.0
        corners = (
            (half_l, half_w), (half_l, -half_w),
            (-half_l, -half_w), (-half_l, half_w),
        )
        return "[" + ",".join(f"[{x:.3f},{y:.3f}]" for x, y in corners) + "]"

    @property
    def spawn_z(self) -> float:
        """Height to create the model at, with clearance to settle rather than
        interpenetrate the floor on the first physics step."""
        return self.base_height + 0.03

    def fields(self) -> dict[str, str]:
        half = self.prox_fov / 2.0
        return {
            "SPAWN_Z": f"{self.base_height:.4f}",
            "LIDAR_X": f"{self.lidar_x:.4f}",
            "LIDAR_Z": f"{self.lidar_z:.4f}",
            "CAM_X": f"{self.camera_x:.4f}",
            "CAM_Z": f"{self.camera_z:.4f}",
            "PROX_X": f"{self.prox_x:.4f}",
            "PROX_Z": f"{self.prox_z:.4f}",
            "PROX_RANGE_MAX": f"{self.prox_range_max:g}",
            "PROX_SAMPLES": str(self.prox_samples),
            "PROX_MIN_ANGLE": f"{-half:.5f}",
            "PROX_MAX_ANGLE": f"{half:.5f}",
        }


# The simulated fleet's platforms. Adding one means adding a chassis fragment
# and a row here; nothing else in the stack needs to know.
#
# base_height is DERIVED from each fragment's drive geometry and must match it:
#   bunker  track wheel centre -0.100, radius 0.100 -> 0.200
#   scout   wheel centre       -0.035, radius 0.0875 -> 0.1225
#   spot    hidden wheel       -0.340, radius 0.160 -> 0.500
# Get it wrong and the robot spawns inside the floor or drops on start.
ROBOT_PROFILES: dict[str, RobotSpec] = {
    # lidar_z on every platform clears min_lidar_z(0.3927) — the steepest ring
    # of the generic_32 profile — with roughly 0.09 m to spare. They were all
    # originally set just below it, which cost a Spot every direction it could
    # have driven in.
    "bunker": RobotSpec(
        chassis="bunker",
        robot_type="agilex_bunker",
        length=1.023, width=0.778, base_height=0.200,
        lidar_x=-0.150, lidar_z=0.520,
        camera_x=0.515, camera_z=0.100,
        prox_x=0.530, prox_range_max=8.0,
        deck_top=0.180, deck_half_length=0.450,
    ),
    "scout_mini": RobotSpec(
        chassis="scout_mini",
        robot_type="agilex_scout_mini",
        length=0.612, width=0.580, base_height=0.1225,
        lidar_x=-0.080, lidar_z=0.330,
        camera_x=0.322, camera_z=0.090,
        prox_x=0.320, prox_range_max=6.0,
        deck_top=0.121, deck_half_length=0.250,
    ),
    "spot": RobotSpec(
        chassis="spot",
        robot_type="boston_dynamics_spot",
        length=1.100, width=0.500, base_height=0.500,
        lidar_x=-0.180, lidar_z=0.470,
        camera_x=0.598, camera_z=0.020,
        prox_x=0.580, prox_range_max=8.0,
        deck_top=0.120, deck_half_length=0.450,
    ),
}

DEFAULT_ROBOT_PROFILE = "scout_mini"

# The three sections a chassis fragment is split into, in file order.
CHASSIS_SECTIONS = ("CHASSIS", "LINKS", "DRIVE")


def robot_spec(name: str) -> RobotSpec:
    if name not in ROBOT_PROFILES:
        raise ValueError(
            f"unknown robot profile {name!r}; available: {sorted(ROBOT_PROFILES)}"
        )
    return ROBOT_PROFILES[name]


def robot_types(fleet_cfg: Mapping[str, Any] | None, count: int, prefix: str) -> list[str]:
    """Resolve which platform each robot is.

    ```yaml
    fleet:
      robot_type: scout_mini        # fleet-wide default
      robot_types:                  # per-robot override, by id
        robot_0: bunker
        robot_3: spot
    ```

    A mixed fleet is the point — the real deployment is two Bunkers, a Scout
    Mini and a Spot, and a merged map built from robots with different lidar
    heights and footprints is a different problem than one built from four
    identical ones.
    """
    fleet_cfg = fleet_cfg or {}
    default = fleet_cfg.get("robot_type", DEFAULT_ROBOT_PROFILE)
    per_robot = fleet_cfg.get("robot_types") or {}
    unknown = set(per_robot) - {f"{prefix}{i}" for i in range(count)}
    if unknown:
        raise ValueError(
            f"fleet.robot_types names robots that are not in this fleet: "
            f"{sorted(unknown)}"
        )
    return [per_robot.get(f"{prefix}{i}", default) for i in range(count)]


def chassis_sections(name: str) -> dict[str, str]:
    """Split a chassis fragment on its @SECTION markers."""
    path = CHASSIS_DIR / f"{name}.xml"
    if not path.exists():
        raise FileNotFoundError(f"no chassis fragment for {name!r}: {path}")
    text = path.read_text()
    out: dict[str, str] = {}
    for section in CHASSIS_SECTIONS:
        marker = f"<!-- @{section} -->"
        if marker not in text:
            raise ValueError(f"{path.name} is missing the {marker} marker")
        body = text.split(marker, 1)[1]
        for other in CHASSIS_SECTIONS:
            body = body.split(f"<!-- @{other} -->", 1)[0]
        out[section] = body.rstrip()
    return out

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


def render(
    name: str,
    color: str,
    spec: LidarSpec | None = None,
    robot: RobotSpec | None = None,
) -> str:
    """Assemble one robot's SDF from the shared shell plus its chassis fragment."""
    robot = robot or robot_spec(DEFAULT_ROBOT_PROFILE)
    sections = chassis_sections(robot.chassis)
    sdf = TEMPLATE.read_text()
    sdf = sdf.replace("{{CHASSIS}}", sections["CHASSIS"])
    sdf = sdf.replace("{{EXTRA_LINKS}}", sections["LINKS"])
    sdf = sdf.replace("{{DRIVE_PLUGIN}}", sections["DRIVE"])
    for key, value in {**robot.fields(), **(spec or LidarSpec()).fields()}.items():
        sdf = sdf.replace("{{" + key + "}}", value)
    # NAME and COLOR last: the chassis fragments use them too, and they are only
    # in the document after the sections above have been spliced in.
    sdf = sdf.replace("{{NAME}}", name).replace("{{COLOR}}", color)

    leftover = sorted(set(re.findall(r"\{\{([A-Z_]+)\}\}", sdf)))
    if leftover:
        # An unreplaced placeholder is not a cosmetic problem: Gazebo will
        # refuse the SDF, or worse parse `{{LIDAR_Z}}` as 0 and mount the
        # mapping lidar inside the chassis.
        raise ValueError(f"{name}: unfilled SDF placeholders {leftover}")
    return sdf


def yaw_quaternion(yaw: float) -> tuple[float, float]:
    """Return the normalized planar quaternion components (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def spawn(
    world: str, name: str, sdf: str, x: float, y: float, yaw: float, z: float
) -> bool:
    """Create the model in a running world.

    `z` is per platform and must be supplied. EntityFactory's pose REPLACES the
    model's own `<pose>`, so the SPAWN_Z baked into the SDF does not apply here
    — a single hardcoded height buries a Spot's hidden drive wheels below the
    floor while leaving a Scout Mini hanging in the air.
    """
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
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
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
    types = robot_types(fleet_cfg, min(count, 5), prefix)

    ok = True
    for i in range(min(count, 5)):
        name = f"{prefix}{i}"
        robot = robot_spec(types[i])
        pose = starts.get(name, {"x": i * 3.0, "y": 0.0, "yaw": 0.0})
        sdf = render(name, color_for(name, i, types[i]), spec, robot)
        print(
            f"[spawn] {name}: {types[i]} "
            f"({robot.length:.2f}x{robot.width:.2f} m, r={robot.footprint_radius:.2f} m, "
            f"lidar z={robot.lidar_z:.3f})"
        )

        if args.outdir:
            out = Path(args.outdir) / f"{name}.sdf"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(sdf)
            print(f"[render] {out}")

        if not args.dry_run:
            ok &= spawn(
                args.world, name, sdf,
                pose["x"], pose["y"], pose.get("yaw", 0.0),
                z=robot.spawn_z,
            )

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
