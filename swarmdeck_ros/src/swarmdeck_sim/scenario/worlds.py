"""The worlds the ARGoS backend can load from prebuilt glTF assets.

A session config selects one with `world: <name>` (or `world: {name: <name>}`).
Anything not listed here is the procedural indoor world that
`make_argos_world.py` writes from the session seed.

Each mesh world is a pair of glTF files (what the cameras and the
photorealistic lidar see, what the robots collide with), the lamps that make
it readable, the exposure those lamps were tuned against, and where the fleet
deploys. `make_argos_session.py` turns this table into XML; `session.launch.py`
and the visual test only ask whether a config names one of them.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
# scenario -> swarmdeck_sim -> src -> swarmdeck_ros -> repo root.
REPO = HERE.parents[3]


@dataclass(frozen=True)
class MeshWorld:
    name: str
    title: str
    # Where the assets live: environment variables to consult, then directories
    # to try, on the host and at the path the compose files mount them to.
    env_vars: tuple[str, ...]
    candidates: tuple[Path, ...]
    visual_glb: str  # the marker file too: the directory must hold it
    collision_glb: str
    lights_file: str | None
    ibl_file: str | None
    # Both glTF files are y-up and are placed with the same transform:
    # orientation 0,0,90 and this z translation (see make_argos_session).
    z_offset: float
    arena_size: str
    arena_center: str
    # Lighting the lamps were tuned against; the renderer is physically based
    # so lamps and exposure are one setting made in two places.
    sky_color: str
    sky_lux: float
    sun_direction: str
    sun_lux: float
    aperture: float
    shutter: float
    iso: float
    # Ground for detection targets: only triangles whose glTF material starts
    # with this prefix (None = any), and only surfaces at or below this height
    # (None = any), so a multi-level world does not put a target on a ceiling.
    ground_material_prefix: str | None
    ground_below_z: float | None
    # Where the fleet deploys when the config does not say (x, y, z, yaw).
    default_start_poses: dict[str, dict[str, float]]
    # (x, y, yaw) of the detection targets, on ground the robots can reach.
    target_placements: tuple[tuple[float, float, float], ...]
    # The interactive viewer's opening shot.
    viewer_position: str
    viewer_look_at: str
    viewer_far: str
    viewer_flashlight: bool = False
    robot_floodlights: bool = False
    # Scatter the detection targets over the floor the fleet can reach from
    # its start (walkable.py) instead of using `target_placements`: one in the
    # far end of the network, none near the start.
    scatter_targets: bool = False

    def assets_dir(self, custom_path: Path | str | None = None) -> Path:
        candidates: list[Path] = []
        if custom_path:
            candidates.append(Path(custom_path))
        for env_var in self.env_vars:
            val = os.environ.get(env_var)
            if val:
                candidates.append(Path(val))
        candidates.extend(self.candidates)
        for c in candidates:
            if c.is_dir() and (c / self.visual_glb).exists():
                return c.resolve()
            if c.is_dir() and (c / "assets" / self.visual_glb).exists():
                return (c / "assets").resolve()
        searched = "\n  - ".join(str(p) for p in candidates)
        raise FileNotFoundError(
            f"Cannot find the {self.title} assets ({self.visual_glb}). Searched:\n"
            f"  - {searched}\nSet {self.env_vars[0]} or pass --world-dir."
        )


# Amazon Lumberyard Bistro: a Paris street at night, lit by its own lamps.
# Assets (2.6 GB) live in the sibling argos3-examples checkout.
BISTRO = MeshWorld(
    name="bistro",
    title="Amazon Lumberyard Bistro",
    env_vars=("SWARMDECK_BISTRO_DIR", "BISTRO_ASSETS_DIR", "BISTRO_DIR"),
    candidates=(
        REPO.parent
        / "argos3-examples"
        / "experiments"
        / "bistro_exploration"
        / "assets",
        REPO / "argos" / "assets" / "bistro",
        Path("/app/argos3-examples/experiments/bistro_exploration/assets"),
        Path("/argos3-examples/experiments/bistro_exploration/assets"),
    ),
    visual_glb="bistro_exterior.glb",
    collision_glb="bistro_exterior.glb",
    lights_file="bistro_lamps.inc",
    ibl_file="san_giuseppe_ibl.ktx",
    # The street surface lies at z 0.21-0.45 in asset coordinates; lowered so
    # the robots' z=0.15 spawn clears the cobblestones.
    z_offset=-0.3,
    arena_size="200,210,70",
    arena_center="24,-4,25",
    sky_color="0.045,0.055,0.085",
    sky_lux=8.0,
    sun_direction="0.85,0.35,-0.22",
    sun_lux=0.0,
    aperture=2.0,
    shutter=0.02,
    iso=400.0,
    ground_material_prefix="pavement",
    ground_below_z=None,
    # Shared deployment on the crown of the street; z clears the local
    # cobblestones. The road is cambered into a gutter along the west kerb, and
    # a light robot spawned at x = -14 slides into it, so both columns keep
    # 1.2 m or more from a kerb.
    default_start_poses={
        f"robot_{i}": {"x": x, "y": y, "z": 0.15, "yaw": -math.pi / 2}
        for i, (x, y) in enumerate(((-11.2, 4), (-11.2, 6), (-13.2, 4), (-13.2, 6)))
    },
    # Safe street / sidewalk coordinates for detection targets.
    target_placements=(
        (-8.0, 5.0, -1.5),
        (-7.0, -10.0, -1.5),
        (-5.0, -13.0, 0.0),
        (5.0, -15.0, 0.0),
        (25.0, -20.0, -0.3),
        (35.0, -24.0, 0.0),
        (18.0, -17.0, 3.14),
        (8.0, -14.0, 3.14),
        (-2.0, -14.0, 3.14),
        (-8.5, -3.0, 1.57),
    ),
    viewer_position="-7.25,-12.75,3.0",
    viewer_look_at="-9.5,0.5,1.2",
    viewer_far="400",
)

# DARPA SubT Finals Prize Round World 01: the Finals staging hangar and the
# tunnel circuit behind it. Imported from Gazebo Fuel by the argos3 fork's
# photorealism/environments/import_fuel_world.py, which also writes the lamp
# list and the exposure this table repeats. Measured through the collision
# mesh (Jolt ray casts): hangar floor z = -0.01 for x in [-20, -11], y within
# +-5.5; the gate to the tunnel is 3.3 m wide at x = -10.5, and the first tunnel
# tile runs from there to x = 15 at +-2.6 m, floor z = 0, ceiling 3.5 m.
SUBT_FINALS = MeshWorld(
    name="subt_finals",
    title="DARPA SubT Finals Prize Round World 01",
    env_vars=("SWARMDECK_SUBT_FINALS_DIR",),
    candidates=(
        REPO.parent
        / "argos3"
        / "src"
        / "plugins"
        / "simulator"
        / "photorealism"
        / "environments"
        / "finals_prize_round_world_01",
        REPO / "argos" / "assets" / "finals_prize_round_world_01",
        Path("/app/argos3-environments/finals_prize_round_world_01"),
    ),
    visual_glb="finals_prize_round_world_01.glb",
    collision_glb="finals_prize_round_world_01.collision.glb",
    lights_file="finals_prize_round_world_01.lights.xml",
    ibl_file=None,
    z_offset=0.0,
    # The importer's bounds plus a 2 m margin.
    arena_size="453.5,213.5,36.5",
    arena_center="191.9,10,-1.7",
    # Underground: no sky, no sun, only the SDF lamps (staging area and the
    # lit tiles); the rest of the map is dark, as in the competition.
    sky_color="0,0,0",
    sky_lux=0.0,
    sun_direction="0,0,-1",
    sun_lux=0.0,
    aperture=2.0,
    shutter=0.04,
    iso=400.0,
    ground_material_prefix=None,
    # Only used for fixed placements; scattered targets carry their floor height.
    ground_below_z=1.0,
    # Two columns in the staging hangar, lined up with the gate at y = 0 and
    # facing the tunnel (+x). robot_0 is at the head of the group, which is
    # the one fleet-wide Explore releases first.
    default_start_poses={
        "robot_0": {"x": -14.5, "y": 1.0, "z": 0.15, "yaw": 0.0},
        "robot_1": {"x": -14.5, "y": -1.0, "z": 0.15, "yaw": 0.0},
        "robot_2": {"x": -16.5, "y": 1.0, "z": 0.15, "yaw": 0.0},
        "robot_3": {"x": -16.5, "y": -1.0, "z": 0.15, "yaw": 0.0},
    },
    # Scattered through the whole tunnel network instead (scatter_targets).
    target_placements=(),
    # In the hangar behind the fleet, looking through the gate into the tunnel.
    viewer_position="-19,0,2.2",
    viewer_look_at="-8,0,0.8",
    viewer_far="300",
    viewer_flashlight=True,
    robot_floodlights=True,
    scatter_targets=True,
)

MESH_WORLDS: dict[str, MeshWorld] = {w.name: w for w in (BISTRO, SUBT_FINALS)}


def world_name(cfg: dict) -> str:
    """The world a session config names, lower-cased; "procedural" when none.

    Accepts the mapping form (`world: {name: bistro}`) because the scenario
    configs have carried both.
    """
    world_cfg = cfg.get("world") or cfg.get("environment") or "procedural"
    if isinstance(world_cfg, dict):
        name = world_cfg.get("name") or world_cfg.get("type", "procedural")
    else:
        name = world_cfg
    return str(name).lower()


def mesh_world(cfg: dict) -> MeshWorld | None:
    """The prebuilt world a config selects, or None for the procedural one."""
    return MESH_WORLDS.get(world_name(cfg))
