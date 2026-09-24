#!/usr/bin/env python3
"""Generate the ARGoS experiment file for a SwarmDeck session config.

    python3 make_argos_session.py --config configs/4robot.yaml -o session.argos

Everything physical in the output comes from `spawn_fleet.py`, which
`session.launch.py` and `adapter_sim` all read too. That is deliberate:
chassis footprints and sensor mounts appear in Nav2, the adapter protocol,
and this XML, and a second table would drift silently.

The procedural geometry is shared with the ARGoS mesh builder in
`indoor_geometry.py`.

Two things in the output are worth understanding before editing it.

**The world appears twice, from two files.** The photorealism `<prop>` that the
cameras and the photorealistic lidar raytrace draws `indoor.gltf`; the Jolt
`<mesh>` the robots collide with is `indoor_collision.gltf`. Position,
orientation and scale are identical on both and must stay identical: if they
disagree, robots collide with a building that is not where it is drawn, and
nothing says so.

They are separate files for exactly one reason. The collision mesh has no floor
slab, because `<physics_engines>` below already provides the ground as a
`<floor height="0">` plane. A slab whose top face is also at z=0 leaves every
robot resting on two coincident surfaces, and the degenerate contacts cost
60-100% of the commanded turn rate (position-dependent, and measured on all
three platforms) while barely touching translation: a robot that drives but
will not turn. `make_argos_world.build_indoor_world` writes both.

**The odometry is not ground truth.** `<odometry implementation="external"
medium="uf"/>` reports whatever Ultra-Fusion estimated from the simulated IMU,
lidar and wheel encoders, so it drifts the way a real front-end drifts, and
`swarmdeck-slam` has a real problem to solve. Switching this to the
`positioning` sensor would make the collaborative merge trivially correct and
measure nothing.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from pathlib import Path
from xml.etree import ElementTree
from xml.sax.saxutils import quoteattr

import yaml

HERE = Path(__file__).resolve().parent
# scenario -> swarmdeck_sim -> src -> swarmdeck_ros -> repo root.
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE))

from indoor_geometry import place_targets  # noqa: E402
from make_argos_world import collision_path, target_classes  # noqa: E402
from spawn_fleet import (  # noqa: E402
    lidar_spec,
    odometry_spec,
    odometry_types,
    robot_spec,
    robot_types,
)
from worlds import MESH_WORLDS, MeshWorld, world_name  # noqa: E402

# Small detection props are visible sensor targets, not infinite-mass barriers.
NONBLOCKING_TARGET_CLASSES = frozenset(
    {"wooden_block", "filament_spool", "disc_cone", "pool_noodle"}
)


def world_lights(path: Path) -> list[str]:
    """The <point>/<spot> lamps of a world's light file, one XML line each.

    Bistro ships a bare list of <point> elements under a comment that holds
    "--" (a command line), which no XML parser accepts; the Fuel importer
    writes a <lights> root with a comment per lamp. Comments are dropped
    before parsing, so the output carries the lamps and only the lamps.
    """
    text = re.sub(r"<!--.*?-->", "", path.read_text(), flags=re.DOTALL)
    root = ElementTree.fromstring(f"<lights>{text}</lights>")
    return [
        ElementTree.tostring(lamp, encoding="unicode").strip()
        for lamp in root.iter()
        if lamp.tag in ("point", "spot")
    ]


def robot_floodlights(
    robot_ids: list[str],
    types: list[str],
    cfg: bool | dict[str, Any],
    indent: str = "        ",
) -> list[str]:
    """Mounts a forward-facing floodlight on each robot entity for camera navigation.

    ARGoS's photorealism plugin attaches any <spot> with an `entity` attribute to that
    robot's origin anchor, tracking its pose every tick in the body frame (+x forward).
    A wide beam (inner 35 deg, outer 55 deg) covers the camera's 60 deg FOV, with a
    slight downward pitch (-0.05 z component) to illuminate the floor ahead as well as
    walls and ceiling in dark tunnels.
    """
    params = cfg if isinstance(cfg, dict) else {}
    intensity = params.get("intensity", 4000)
    falloff = params.get("falloff", 25)
    inner = params.get("inner_angle", 35)
    outer = params.get("outer_angle", 55)
    color = params.get("color", "1.0,0.95,0.85")
    direction = params.get("direction", "1,0,-0.05")
    lines: list[str] = []
    for rid, profile in zip(robot_ids, types):
        spec = robot_spec(profile)
        camera_z = spec.base_height + spec.camera_z
        pos = _vec(spec.camera_x, 0.0, camera_z)
        lines.append(
            f"{indent}<spot entity={_attr(rid)} position={_attr(pos)} "
            f'direction={_attr(direction)} intensity="{intensity:g}" '
            f'falloff="{falloff:g}" inner_angle="{inner:g}" '
            f'outer_angle="{outer:g}" color={_attr(color)} />'
        )
    return lines


# Ultra-Fusion's tooling is built around 100 Hz. `make_profile.py` derives the
# estimator's IMU noise densities from the sample rate, and those scale by
# sqrt(rate), so running the simulation ten times slower than the estimator was
# told to expect is a 3.16x noise error it cannot see. The sensors below divide
# this back down to their own real rates, so raising it does not multiply the
# rendering cost.
TICKS_PER_SECOND = 100

# Sensor rates, as divisors of the tick rate.
CAMERA_HZ = 5.0  # matches the adapter's JPEG preview budget
EXCHANGE_HZ = 10.0  # how often ROS sees an observation, and the /clock rate

# The photorealistic camera's resolution is the single biggest render-cost dial
# in the experiment, ahead of the lidar. 320x240 is the reference figure from
# argos3-examples' Ultra-Fusion benchmarks.
# Overridable so a visual estimator can be given something to work with:
# FAST-LIVO2 aligns image patches photometrically, and 320x240 of a sparse
# scene gives it very little. SWARMDECK_CAMERA_RESOLUTION=640,480
CAMERA_RESOLUTION = tuple(
    int(v) for v in os.environ.get("SWARMDECK_CAMERA_RESOLUTION", "320,240").split(",")
)
CAMERA_FOV_DEG = 60.0

# Ultra-Fusion's lidar-inertial modes register on the vertical structure of the
# cloud. A planar scan has none, so a fleet configured with one produces an
# estimator that never converges and robots that never move, which presents as
# a bridge problem rather than a sensor one. Refuse instead.
MIN_LIDAR_RINGS = 9

# The controller and the loop function live in ONE ARGoS module; see
# argos/CMakeLists.txt for why they cannot be two.
# The "lib" prefix is part of the name ARGoS is given: it appends ".so"
# when resolving a library against ARGOS_PLUGIN_PATH but does not add a
# prefix, so "swarmdeck_argos" resolves to swarmdeck_argos.so and finds
# nothing.
LIBRARY = "libswarmdeck_argos"

# Where the socket volume is mounted in every container. Kept short on
# purpose: a Unix socket path over 107 bytes fails to bind, and the error
# ("AF_UNIX path too long") surfaces in whichever process binds first.
# Where the socket volume is mounted in every container.
RUNTIME_DIR = "/run/swarmdeck"


def _attr(value) -> str:
    return quoteattr(str(value))


def _fmt(value: float) -> str:
    """Fixed precision, so the same config always renders the same bytes."""
    return f"{value:.4f}".rstrip("0").rstrip(".") or "0"


def _vec(*values: float) -> str:
    return ",".join(_fmt(v) for v in values)


def _divider(hz: float) -> int:
    return max(1, int(round(TICKS_PER_SECOND / hz)))


def controller_block(
    rid: str,
    profile: str,
    spec,
    lidar,
    odometry: Any = True,
    indent: str = "    ",
    *,
    parked_lidar_divider: int = 0,
) -> list[str]:
    """One robot's controller, with its sensors mounted from `RobotSpec`.

    Sensors hang off the `origin` anchor with explicit positions rather than
    off the entity's named `lidar`/`camera` anchors. The entity plugins do
    define those anchors, but they hardcode one platform's mounts in C++,
    whereas these numbers have to follow the fleet config that Nav2 and SLAM
    are configured from at the same time.

    `RobotSpec` mounts are relative to base_link, which floats `base_height`
    above the floor; the ARGoS origin anchor is ON the floor. Hence the sum.
    """
    if isinstance(odometry, bool):
        spec_obj = odometry_spec("fast_livo2" if odometry else "drift")
    elif isinstance(odometry, str):
        spec_name = "fast_livo2" if odometry == "external" else odometry
        spec_obj = odometry_spec(spec_name)
    else:
        spec_obj = odometry

    if spec_obj.implementation == "external" and spec_obj.medium:
        odometry_attrs = (
            f'implementation="{spec_obj.implementation}" medium="{spec_obj.medium}"'
        )
    else:
        odometry_attrs = f'implementation="{spec_obj.implementation}"'
    lidar_z = spec.base_height + spec.lidar_z
    camera_z = spec.base_height + spec.camera_z
    vfov_deg = math.degrees(lidar.vfov)
    h_res = 360.0 / lidar.h_samples
    # Four 90-degree faces must cover the extreme elevations at their
    # corners too, where the forward projection is reduced by cos(45°).
    # Keep one pixel of margin for the serialized angle and edge sampling.
    face_width = 512
    face_height = max(
        192, math.ceil(face_width * math.sqrt(2.0) * math.tan(lidar.vfov)) + 1
    )
    lines = [
        f'{indent}<swarmdeck_robot_controller id={_attr(rid + "_ctrl")}',
        f"{indent}    library={_attr(LIBRARY)}>",
        f"{indent}  <actuators>",
        f'{indent}    <differential_steering implementation="default" />',
        f"{indent}  </actuators>",
        f"{indent}  <sensors>",
        f'{indent}    <positioning implementation="default" />',
        f'{indent}    <photorealistic_lidar implementation="default" medium="pr"',
        f'{indent}                          anchor="origin"',
        f"{indent}                          position={_attr(_vec(spec.lidar_x, 0.0, lidar_z))}",
        f'{indent}                          orientation="0,0,0"',
        f"{indent}                          rings={_attr(lidar.rings)}",
        f"{indent}                          vertical_fov={_attr(_vec(-vfov_deg, vfov_deg))}",
        f'{indent}                          horizontal_resolution={_attr(f"{h_res:.4f}")}',
        f'{indent}                          faces="4"',
        f"{indent}                          face_resolution={_attr(_vec(face_width, face_height))}",
        f"{indent}                          max_range={_attr(_fmt(lidar.range_max))}",
        # A real time-of-flight unit is specified around +/-3 cm. Zero noise
        # hands the scan matcher an accuracy it will never have on hardware.
        f'{indent}                          range_noise_std_dev="0.03"',
        *(
            [
                f"{indent}                          parked_framerate_divider={_attr(parked_lidar_divider)}",
                f"{indent}                          parked_after_ticks={_attr(TICKS_PER_SECOND)}",
            ]
            if parked_lidar_divider
            else []
        ),
        f"{indent}                          framerate_divider={_attr(_divider(lidar.rate))} />",
        f'{indent}    <photorealistic_camera implementation="default" medium="pr"',
        f'{indent}                           anchor="origin"',
        f"{indent}                           position={_attr(_vec(spec.camera_x, 0.0, camera_z))}",
        f'{indent}                           orientation="0,0,0"',
        f"{indent}                           resolution={_attr(_vec(*CAMERA_RESOLUTION))}",
        f"{indent}                           fov={_attr(_fmt(CAMERA_FOV_DEG))}",
        f'{indent}                           near="0.05" far="40"',
        f'{indent}                           modalities="rgb,depth"',
        f"{indent}                           framerate_divider={_attr(_divider(CAMERA_HZ))} />",
        f'{indent}    <imu implementation="default"',
        f'{indent}         gyro_noise_std_dev="0.002" accel_noise_std_dev="0.02"',
        f'{indent}         gyro_bias_walk_std_dev="0.0002" accel_bias_walk_std_dev="0.002" />',
        # The encoder noise is what makes wheel odometry worth fusing rather
        # than trusting. The medium dead-reckons Ultra-Fusion's wheel channel
        # from the covered distances this reports.
        f'{indent}    <differential_steering implementation="default"',
        f'{indent}                           vel_noise_range="-0.2:0.2"',
        f'{indent}                           dist_noise_range="-0.02:0.02" />',
        # The pose the whole ROS stack navigates on. See the module docstring.
        f"{indent}    <odometry {odometry_attrs} />",
        f"{indent}  </sensors>",
        f"{indent}  <params robot_id={_attr(rid)}",
        f"{indent}          track_gauge={_attr(_fmt(spec.track_gauge))}",
        f'{indent}          max_speed="150.0" />',
        f"{indent}</swarmdeck_robot_controller>",
    ]
    return lines


# Scattered targets (MeshWorld.scatter_targets): how far from the fleet's start
# they have to be along the floor, and how far apart from each other.
SCATTER_MIN_DISTANCE_M = 25.0
SCATTER_SPACING_M = 15.0


def scattered_targets(
    surface, start: dict, seed: int, count: int
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """(x, y, yaw) and floor height of `count` targets over the reachable floor.

    Seeded, so a scenario keeps its layout; the first is in the far end of
    the network, and none is within SCATTER_MIN_DISTANCE_M of the start.
    """
    import random

    from walkable import reachable_floor, scatter

    floor = reachable_floor(
        surface.triangles,
        (float(start["x"]), float(start["y"]), float(start.get("z", 0.0))),
    )
    rng = random.Random(seed)
    picks = scatter(
        floor,
        count,
        rng,
        min_distance=SCATTER_MIN_DISTANCE_M,
        spacing=SCATTER_SPACING_M,
    )
    placements = [
        (float(floor.x[n]), float(floor.y[n]), rng.uniform(-math.pi, math.pi))
        for n in picks
    ]
    return placements, [float(floor.z[n]) for n in picks]


def generate_argos_xml(
    config_path: Path,
    robot_count: int | None = None,
    world_gltf: str = "indoor.gltf",
    world_dir: Path | str | None = None,
    props_dir: str = "props",
    socket_path: str = f"{RUNTIME_DIR}/argos.sock",
    uf_socket_path: str = f"{RUNTIME_DIR}/uf.sock",
    targets: int = 10,
    headless: bool = True,
    estimator: bool | None = None,
    odometry: str | None = None,
    threads: int | None = None,
) -> str:
    world_collision = str(collision_path(Path(world_gltf)))
    cfg = yaml.safe_load(config_path.read_text())
    fleet_cfg = cfg.get("fleet", {}) or {}
    count = int(fleet_cfg.get("robot_count", 4)) if robot_count is None else robot_count
    count = max(1, min(count, 5))
    prefix = fleet_cfg.get("robot_prefix", "robot_")
    seed = int(cfg.get("seed", 20260801))
    starts = (cfg.get("map", {}) or {}).get("start_poses", {}) or {}

    world_cfg = cfg.get("world") or cfg.get("environment") or "procedural"
    if isinstance(world_cfg, dict) and not world_dir:
        world_dir = world_cfg.get("dir") or world_cfg.get("assets_dir")
    # A prebuilt world is selected by the config, or by --world naming it
    # (session.launch.py passes the name through rather than a glTF path).
    world: MeshWorld | None = MESH_WORLDS.get(world_gltf) or MESH_WORLDS.get(
        world_name(cfg)
    )

    lidar = lidar_spec(fleet_cfg)
    if lidar.rings < MIN_LIDAR_RINGS:
        raise ValueError(
            f"the ARGoS backend fuses odometry with Fast-LIVO2, which is "
            f"lidar-inertial-visual, but the lidar resolves to {lidar.rings} ring(s) "
            f"in {config_path.name}. A planar scan carries no vertical "
            f"structure to register on, so the estimator never converges and "
            f"every robot stands still with no odometry at all. Set "
            f"fleet.lidar.profile to vlp16 or generic_32."
        )

    simulation = cfg.get("simulation") or {}
    realtime_factor = simulation.get("realtime_factor", 1)
    if (
        isinstance(realtime_factor, bool)
        or not isinstance(realtime_factor, (int, float))
        or not math.isfinite(realtime_factor)
        or realtime_factor < 0
    ):
        raise ValueError("simulation.realtime_factor must be finite and nonnegative")
    parked_rate = simulation.get("parked_lidar_rate")
    parked_divider = 0
    if parked_rate is not None:
        if (
            isinstance(parked_rate, bool)
            or not isinstance(parked_rate, (int, float))
            or not math.isfinite(parked_rate)
            or not 1 <= parked_rate <= lidar.rate
            or not math.isclose(_divider(parked_rate) * parked_rate, TICKS_PER_SECOND)
        ):
            raise ValueError(
                "simulation.parked_lidar_rate must be at least 1 Hz, no faster than "
                "fleet.lidar.rate, and divide the 100 Hz physics tick rate exactly"
            )
        parked_divider = _divider(parked_rate)

    types = robot_types(fleet_cfg, count, prefix)
    robot_ids = [f"{prefix}{i}" for i in range(count)]

    default_odom = odometry
    if default_odom is None and estimator is not None:
        default_odom = "fast_livo2" if estimator else "drift"
    elif default_odom == "external":
        default_odom = "fast_livo2"

    odom_types_list = odometry_types(
        fleet_cfg, count, prefix, default_override=default_odom
    )
    odom_specs = [odometry_spec(o) for o in odom_types_list]
    floodlight_setting = fleet_cfg.get("floodlight")
    if floodlight_setting is None:
        floodlight_setting = fleet_cfg.get("floodlights")
    if floodlight_setting is None:
        floodlight_setting = fleet_cfg.get("light")
    if floodlight_setting is None and world is not None:
        floodlight_setting = getattr(world, "robot_floodlights", False)

    system_threads = 0 if threads is None else threads

    lines: list[str] = [
        '<?xml version="1.0" ?>',
        "<!-- GENERATED by swarmdeck_sim/scenario/make_argos_session.py.",
        f"     Config: {config_path.name}. Do not edit by hand: the next",
        "     session launch overwrites it. -->",
        "<argos-configuration>",
        "",
        "  <framework>",
        f'    <system threads="{system_threads}" />',
        f'    <experiment length="0" ticks_per_second="{TICKS_PER_SECOND}"',
        f'                random_seed="{seed}" />',
        "  </framework>",
        "",
        "  <controllers>",
    ]

    for rid, profile, odom_s in zip(robot_ids, types, odom_specs):
        lines.extend(
            controller_block(
                rid,
                profile,
                robot_spec(profile),
                lidar,
                odom_s,
                parked_lidar_divider=parked_divider,
            )
        )

    lines.extend(
        [
            "  </controllers>",
            "",
            "  <!-- The single loop-function slot is the ROS boundary. Ultra-Fusion",
            "       is a <media> below precisely because this slot is taken. -->",
            "  <loop_functions",
            f"      library={_attr(LIBRARY)}",
            '      label="swarmdeck_bridge"',
            f"      socket={_attr(socket_path)}",
            f'      robots={_attr(",".join(robot_ids))}',
            f"      exchange_period={_attr(_divider(EXCHANGE_HZ))}",
            f"      realtime_factor={_attr(realtime_factor)}",
            '      connect_timeout="180" />',
            "",
        ]
    )

    # Props need a +90-degree roll; Jolt must not also convert Y-up internally.
    # Set y_up=false on every mesh below to preserve identical world transforms.
    surface = None
    if world is not None:
        assets = world.assets_dir(world_dir)
        visual_glb = (assets / world.visual_glb).resolve()
        collision_glb = (assets / world.collision_glb).resolve()
        world_pose = f'position={_attr(_vec(0.0, 0.0, world.z_offset))} orientation="0,0,90" scale="1.0"'
        lines.extend(
            [
                f'  <arena size="{world.arena_size}" center="{world.arena_center}">',
                "",
                f"    <!-- {world.title}: collision geometry straight from the glTF,",
                "         at the SAME transform as the photorealism <prop> below. -->",
                f'    <mesh id="world_mesh" y_up="false" file={_attr(str(collision_glb))}',
                f"          {world_pose} />",
            ]
        )
        placements = list(world.target_placements[:targets]) if targets > 0 else []
        if world.scatter_targets and targets > 0:
            from mesh_surface import MeshSurface

            surface = MeshSurface(
                collision_glb,
                material_prefix=world.ground_material_prefix,
                z_offset=world.z_offset,
                below=world.ground_below_z,
            )
            placements, target_floor = scattered_targets(
                surface,
                starts.get(robot_ids[0]) or world.default_start_poses[robot_ids[0]],
                seed,
                targets,
            )
    else:
        # 26 m of building plus clearance; nothing in the world reaches 6 m.
        lines.extend(
            [
                '  <arena size="30,30,6" center="0,0,3">',
                "",
                "    <!-- Collision geometry, at the SAME transform as the photorealism",
                "         <prop> at the bottom of this file, but NOT the same file.",
                "         The collision mesh carries no floor slab: the <jolt> engine",
                "         below already provides the ground as a plane at z=0, and a",
                "         slab whose top face is also at z=0 puts every robot in",
                "         contact with two coincident surfaces. Measured cost of that:",
                "         60-100% of the commanded turn rate, varying with position,",
                "         while translation is almost unaffected. A robot that drives",
                "         but will not turn. See make_argos_world.build_indoor_world. -->",
                f'    <mesh id="world_mesh" y_up="false" file={_attr(world_collision)}',
                '          position="0,0,0" orientation="0,0,90" scale="1.0" />',
            ]
        )
        placements = place_targets(seed, targets)

    classes = target_classes(len(placements))
    target_z = [0.0] * len(placements)
    if world is not None and placements:
        from mesh_surface import MeshSurface

        if surface is None:
            surface = MeshSurface(
                collision_glb,
                material_prefix=world.ground_material_prefix,
                z_offset=world.z_offset,
                below=world.ground_below_z,
            )
        floors = target_floor if world.scatter_targets else [None] * len(placements)
        # Default relative prop paths refer to generated runtime copies. Use
        # their checked-in originals when generating outside that directory.
        models = Path(props_dir)
        if props_dir == "props" and not models.is_dir():
            models = REPO / "argos/assets/props"
        target_z = [
            surface.place(models / f"{name}.glb", x, y, yaw, near_z=floor)
            for (x, y, yaw), name, floor in zip(placements, classes, floors)
        ]
    if placements:
        lines.append("")
        lines.append("    <!-- Large detection targets: collision meshes drawn by")
        lines.append("         the matching <prop>. Classes come from")
        lines.append("         adapters/perception/catalog.py. -->")
        for i, ((x, y, yaw), name, z) in enumerate(zip(placements, classes, target_z)):
            if name in NONBLOCKING_TARGET_CLASSES:
                continue
            model = f"{props_dir}/{name}.glb"
            # ARGoS composes Rx * Ry * Rz. After the glTF-to-Z-up roll,
            # its Y angle becomes world yaw; using the Z angle tips the prop.
            orientation = _vec(0.0, math.degrees(yaw), 90.0)
            lines.append(
                f'    <mesh id={_attr(f"target_{i}_{name}")} y_up="false" file={_attr(model)}'
            )
            lines.append(
                f"          position={_attr(_vec(x, y, z))} "
                f'orientation={_attr(orientation)} scale="1.0" />'
            )

    lines.append("")
    for i, (rid, profile) in enumerate(zip(robot_ids, types)):
        entity = profile
        pose = starts.get(rid)
        if pose is None and world is not None:
            pose = world.default_start_poses.get(rid)
        if pose is None:
            x = (i - count / 2.0) * 3.0
            y = 0.0
            yaw_deg = 0.0
        else:
            x = float(pose.get("x", (i - count / 2.0) * 3.0))
            y = float(pose.get("y", 0.0))
            yaw_deg = math.degrees(float(pose.get("yaw", 0.0)))
        # Mesh worlds have uneven ground. Honour explicit spawn clearance.
        z = float((pose or {}).get("z", 0.15 if world is not None else 0.02))
        lines.extend(
            [
                f"    <{entity} id={_attr(rid)}>",
                f"      <body position={_attr(_vec(x, y, z))} "
                f"orientation={_attr(_vec(yaw_deg, 0.0, 0.0))} />",
                f'      <controller config={_attr(rid + "_ctrl")} />',
                f"    </{entity}>",
            ]
        )

    lines.extend(
        [
            "  </arena>",
            "",
            "  <physics_engines>",
            '    <jolt id="jolt" iterations="10" threads="1">',
            *([] if world is not None else ['      <floor height="0" />']),
            '      <gravity g="9.81" />',
            "    </jolt>",
            "  </physics_engines>",
            "",
            "  <media>",
        ]
    )

    uf_robots = [
        rid for rid, odom_s in zip(robot_ids, odom_specs) if odom_s.medium == "uf"
    ]
    if uf_robots:
        lines.extend(
            [
                "    <!-- Fast-LIVO2. `channels` names only what the estimator reads:",
                "         the camera is declared on every robot but not streamed here,",
                "         and an unread VLP-16 revolution is ~630 KB per robot per",
                "         100 ms of traffic that only starves the channels that matter.",
                "",
                '         alignment="none" on purpose. Each robot\'s estimate starts at',
                "         its own origin, exactly as a real robot's does, which is the",
                "         premise every merge_mode in configs/ is written against.",
                '         alignment="ground_truth" would hand every robot the shared',
                "         frame that swarmdeck-slam exists to recover. -->",
                f'    <external_estimator id="uf" socket={_attr(uf_socket_path)}',
                f'                        robots={_attr(",".join(uf_robots))}',
                '                        lockstep_pose="false"',
                # Ultra-Fusion ran lwio and never read the camera, so withholding it was
                # right: an unread frame is pure bandwidth. FAST-LIVO2 is
                # LiDAR-inertial-VISUAL, and with the camera withheld its sync_packages
                # never completes, so it emits no pose at all and logs nothing about it.
                # The camera is declared on every robot either way, which is why it
                # still reaches the operator UI; this attribute only governs what is
                # streamed to the estimator socket.
                # Override: SWARMDECK_ESTIMATOR_CHANNELS=imu,lidar,wheels,camera
                f'                        channels={_attr(os.environ.get("SWARMDECK_ESTIMATOR_CHANNELS", "imu,lidar,wheels"))}',
                '                        alignment="none"',
                '                        connect_timeout="180" timeout="120" />',
                "",
            ]
        )

    if world is not None:
        lines.extend(
            [
                f'    <photorealism id="pr" backend="vulkan" asset_path={_attr(str(REPO / "argos/assets/robots"))} draw_floor="false">',
            ]
        )
        ibl_path = assets / world.ibl_file if world.ibl_file else None
        if ibl_path is not None and ibl_path.exists():
            lines.append(
                f'      <environment ibl={_attr(str(ibl_path))} intensity="{world.sky_lux:g}" />'
            )
        lines.extend(
            [
                f'      <skybox color="{world.sky_color}" />',
                f'      <sun direction="{world.sun_direction}" intensity="{world.sun_lux:g}" cast_shadows="false" />',
                f'      <exposure aperture="{world.aperture:g}" shutter_speed="{world.shutter:g}" sensitivity="{world.iso:g}" />',
                "      <lights>",
            ]
        )
        lights_path = assets / world.lights_file if world.lights_file else None
        if lights_path is not None and lights_path.exists():
            lines.extend(f"        {lamp}" for lamp in world_lights(lights_path))
        if floodlight_setting:
            lines.extend(
                robot_floodlights(
                    robot_ids, types, floodlight_setting, indent="        "
                )
            )
        lines.extend(
            [
                "      </lights>",
                "      <scenery>",
                f"        <prop model={_attr(str(visual_glb))} {world_pose} />",
            ]
        )
        for (x, y, yaw), name, z in zip(placements, classes, target_z):
            model = f"{props_dir}/{name}.glb"
            # ARGoS composes Rx * Ry * Rz. After the glTF-to-Z-up roll,
            # its Y angle becomes world yaw; using the Z angle tips the prop.
            orientation = _vec(0.0, math.degrees(yaw), 90.0)
            lines.append(
                f"        <prop model={_attr(model)} "
                f"position={_attr(_vec(x, y, z))} "
                f'orientation={_attr(orientation)} scale="1.0" />'
            )
        lines.extend(
            [
                "      </scenery>",
                "    </photorealism>",
                "  </media>",
            ]
        )
    else:
        lines.extend(
            [
                "    <!-- draw_floor is false because indoor.gltf carries its own floor",
                "         slab; the built-in one would z-fight with it. -->",
                f'    <photorealism id="pr" backend="vulkan" asset_path={_attr(str(REPO / "argos/assets/robots"))} draw_floor="false">',
                '      <skybox color="0.53,0.71,0.92" />',
                "      <!-- Bright overcast, not direct sun. The building has walls but",
                "           no ceiling, so whatever is in the sky lights the rooms, and",
                "           the exposure has to be set for it: the renderer is",
                "           physically based, so illuminance and exposure are one",
                "           setting made in two places. Filament exposes EV =",
                "           log2(N^2/t * 100/S), which at f/4, 1/250 s, ISO 100 is",
                "           EV 12, the right value for roughly 15 klux. The first",
                "           version of this file paired 70 klux with EV 10.9 and every",
                "           camera returned a white frame with the geometry burned out",
                "           of it, while depth and segmentation looked perfect: nothing",
                "           downstream of a physically based renderer notices that the",
                "           photograph is unusable. -->",
                '      <sun direction="0.35,0.25,-0.90" intensity="15000"',
                '           cast_shadows="true" />',
                '      <exposure aperture="4" shutter_speed="0.004" sensitivity="100" />',
                "      <lights>",
                '        <point position="-9,7,2.2" intensity="6000" falloff="9" color="1.0,0.96,0.90" />',
                '        <point position="-3,7,2.2" intensity="6000" falloff="9" color="1.0,0.96,0.90" />',
                '        <point position="3,7,2.2" intensity="6000" falloff="9" color="0.95,0.97,1.0" />',
                '        <point position="9,7,2.2" intensity="6000" falloff="9" color="0.95,0.97,1.0" />',
                '        <point position="-9,-7,2.2" intensity="6000" falloff="9" color="0.95,0.97,1.0" />',
                '        <point position="-3,-7,2.2" intensity="6000" falloff="9" color="1.0,0.96,0.90" />',
                '        <point position="3,-7,2.2" intensity="6000" falloff="9" color="1.0,0.96,0.90" />',
                '        <point position="9,-7,2.2" intensity="6000" falloff="9" color="0.95,0.97,1.0" />',
                '        <point position="0,0,2.3" intensity="8000" falloff="14" color="1.0,0.98,0.95" />',
            ]
        )
        if floodlight_setting:
            lines.extend(
                robot_floodlights(
                    robot_ids, types, floodlight_setting, indent="        "
                )
            )
        lines.extend(
            [
                "      </lights>",
                "      <scenery>",
                f'        <prop model={_attr(world_gltf)} position="0,0,0"',
                '              orientation="0,0,90" scale="1.0" />',
            ]
        )

        for (x, y, yaw), name, z in zip(placements, classes, target_z):
            model = f"{props_dir}/{name}.glb"
            # ARGoS composes Rx * Ry * Rz. After the glTF-to-Z-up roll,
            # its Y angle becomes world yaw; using the Z angle tips the prop.
            orientation = _vec(0.0, math.degrees(yaw), 90.0)
            lines.append(
                f"        <prop model={_attr(model)} "
                f"position={_attr(_vec(x, y, z))} "
                f'orientation={_attr(orientation)} scale="1.0" />'
            )

        lines.extend(
            [
                "      </scenery>",
                "    </photorealism>",
                "  </media>",
            ]
        )

    if not headless:
        if world is not None:
            lines.extend(
                [
                    "",
                    "  <visualization>",
                    '    <filament medium="pr" resolution="1280,720" speed="1"',
                    f'              near="0.3" far="{world.viewer_far}"'
                    + (' flashlight="true"' if world.viewer_flashlight else ""),
                    f'              position="{world.viewer_position}" look_at="{world.viewer_look_at}" />',
                    "  </visualization>",
                ]
            )
        else:
            lines.extend(
                [
                    "",
                    "  <visualization>",
                    '    <filament medium="pr" resolution="1280,720" speed="1"',
                    '              near="0.3" far="80"',
                    '              position="0,-20,14" look_at="0,0,1" />',
                    "  </visualization>",
                ]
            )

    lines.extend(["", "</argos-configuration>", ""])
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, help="SwarmDeck session YAML")
    ap.add_argument("-o", "--output", default="session.argos")
    ap.add_argument("--robots", type=int, default=None, help="Override robot count")
    ap.add_argument(
        "--world",
        default="indoor.gltf",
        help="Path to the generated world glTF as ARGoS will resolve it "
        "(relative paths are relative to the working directory argos3 runs "
        f"in), or the name of a prebuilt world: {', '.join(MESH_WORLDS)}",
    )
    ap.add_argument(
        "--world-dir",
        default=None,
        help="Directory holding the prebuilt world's assets (overrides the "
        "per-world environment variable and the default locations)",
    )
    ap.add_argument(
        "--props-dir",
        default="props",
        help="Directory holding the detection-target models",
    )
    ap.add_argument(
        "--targets",
        type=int,
        default=10,
        help="How many detection targets to place; classes are "
        "assigned round robin from the catalog",
    )
    ap.add_argument("--socket", default=f"{RUNTIME_DIR}/argos.sock")
    ap.add_argument("--uf-socket", default=f"{RUNTIME_DIR}/uf.sock")
    ap.add_argument("--headless", action="store_true", default=True)
    ap.add_argument(
        "--odometry",
        default=None,
        help="Default odometry profile (e.g. fast_livo2, drift, ekf). "
        "Overrides fleet.odometry default in session config.",
    )
    ap.add_argument(
        "--threads",
        type=int,
        default=None,
        help="Worker threads for ARGoS simulation (defaults to robot count)",
    )
    ap.add_argument(
        "--gui",
        dest="headless",
        action="store_false",
        help="Add the interactive Filament viewer",
    )
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = REPO / args.config

    xml = generate_argos_xml(
        config_path=cfg_path,
        robot_count=args.robots,
        world_gltf=args.world,
        world_dir=args.world_dir,
        props_dir=args.props_dir,
        socket_path=args.socket,
        uf_socket_path=args.uf_socket,
        targets=args.targets,
        headless=args.headless,
        odometry=args.odometry,
        threads=args.threads,
    )

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Rename into place rather than truncating in place. The argos container
    # polls this path on a shared volume and reads it as soon as it looks
    # ready, and an in-place write gives it a window in which the file exists
    # but holds half an experiment. It also moves the mtime at the START of the
    # write, and argos-entrypoint.sh uses that mtime to tell this run's
    # experiment from the previous one's. os.replace is atomic within a
    # filesystem, so a reader sees either the old file or a complete new one,
    # and the mtime it checks belongs to a file that is fully written.
    tmp = out.with_name(f".{out.name}.tmp")
    tmp.write_text(xml)
    os.replace(tmp, out)
    print(f"[make_argos_session] {cfg_path.name} -> {out} ({len(xml)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
