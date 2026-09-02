#!/usr/bin/env python3
"""Generate the ARGoS experiment file for a SwarmDeck session config.

    python3 make_argos_session.py --config configs/4robot.yaml -o session.argos

Everything physical in the output comes from `spawn_fleet.py`, which the Gazebo
backend, `session.launch.py` and `adapters/adapter_sim/adapter_sim.py` all read
too. That is deliberate: chassis footprints and sensor mounts appear in Nav2's
costmap parameters, in the SLAM static transforms, in the adapter's `hello`, and
now in this XML, and a second table of them would drift silently. A lidar
mounted 0.25 m lower than SLAM believes tilts and offsets every scan, and no
part of the stack reports an error.

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
from xml.sax.saxutils import quoteattr

import yaml

HERE = Path(__file__).resolve().parent
# scenario -> swarmdeck_sim -> src -> swarmdeck_ros -> repo root.
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE))

from generate_world import place_targets  # noqa: E402
from make_argos_world import collision_path, target_classes  # noqa: E402
from spawn_fleet import (  # noqa: E402
    lidar_spec,
    odometry_spec,
    odometry_types,
    robot_spec,
    robot_types,
)

# Bistro scene constants
BISTRO_ARENA_SIZE = "200,210,70"
BISTRO_ARENA_CENTER = "24,-4,25"

# Night lighting for Bistro (Amazon Lumberyard Bistro scene)
BISTRO_SKY_LUX = 8.0
BISTRO_SUN_LUX = 0.0
BISTRO_SKY_COLOR = "0.045,0.055,0.085"
BISTRO_APERTURE = 2.0
BISTRO_SHUTTER = 0.02
BISTRO_ISO = 400.0

# 4 default start poses (x, y, yaw in radians) along the 141m closed road loop
BISTRO_DEFAULT_START_POSES = {
    "robot_0": {"x": -10.75, "y": 8.25, "yaw": -1.4259},   # -81.7 deg
    "robot_1": {"x": 14.25, "y": -15.75, "yaw": -0.3665},  # -21.0 deg
    "robot_2": {"x": 40.25, "y": -25.75, "yaw": 2.8047},   # 160.7 deg
    "robot_3": {"x": 11.25, "y": -14.75, "yaw": 3.1416},   # 180.0 deg
}

# Safe street / sidewalk coordinates for detection targets in Bistro
BISTRO_TARGET_PLACEMENTS = [
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
]


def find_bistro_assets_dir(custom_path: Path | str | None = None) -> Path:
    candidates: list[Path] = []
    if custom_path:
        candidates.append(Path(custom_path))
    for env_var in ("SWARMDECK_BISTRO_DIR", "BISTRO_ASSETS_DIR", "BISTRO_DIR"):
        val = os.environ.get(env_var)
        if val:
            candidates.append(Path(val))
    candidates.extend([
        REPO.parent / "argos3-examples" / "experiments" / "bistro_exploration" / "assets",
        REPO / "argos" / "assets" / "bistro",
        Path("/app/argos3-examples/experiments/bistro_exploration/assets"),
        Path("/argos3-examples/experiments/bistro_exploration/assets"),
    ])
    for c in candidates:
        if c.is_dir() and (c / "bistro_exterior.glb").exists():
            return c.resolve()
        if c.is_dir() and (c / "assets" / "bistro_exterior.glb").exists():
            return (c / "assets").resolve()
    searched = "\n  - ".join(str(p) for p in candidates)
    raise FileNotFoundError(
        f"Cannot find Bistro assets (bistro_exterior.glb). Searched:\n  - {searched}\n"
        f"Set SWARMDECK_BISTRO_DIR or pass --bistro-dir."
    )


def _sanitize_xml_comments(xml_text: str) -> str:
    """XML 1.0 disallows '--' inside comments."""
    def fix_comment(match: re.Match) -> str:
        content = match.group(1).replace("--", "- -")
        return f"<!--{content}-->"
    return re.sub(r"<!--(.*?)-->", fix_comment, xml_text, flags=re.DOTALL)

# Ultra-Fusion's tooling is built around 100 Hz. `make_profile.py` derives the
# estimator's IMU noise densities from the sample rate, and those scale by
# sqrt(rate), so running the simulation ten times slower than the estimator was
# told to expect is a 3.16x noise error it cannot see. The sensors below divide
# this back down to their own real rates, so raising it does not multiply the
# rendering cost.
TICKS_PER_SECOND = 100

# Sensor rates, as divisors of the tick rate.
LIDAR_HZ = 10.0     # what a VLP-16 actually spins at
CAMERA_HZ = 5.0     # matches the adapter's JPEG preview budget
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


def controller_block(rid: str, profile: str, spec, lidar,
                     odometry: Any = True, indent: str = "    ") -> list[str]:
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
        odometry_attrs = f'implementation="{spec_obj.implementation}" medium="{spec_obj.medium}"'
    else:
        odometry_attrs = f'implementation="{spec_obj.implementation}"'
    lidar_z = spec.base_height + spec.lidar_z
    camera_z = spec.base_height + spec.camera_z
    vfov_deg = math.degrees(lidar.vfov)
    h_res = 360.0 / lidar.h_samples
    lines = [
        f'{indent}<swarmdeck_robot_controller id={_attr(rid + "_ctrl")}',
        f'{indent}    library={_attr(LIBRARY)}>',
        f'{indent}  <actuators>',
        f'{indent}    <differential_steering implementation="default" />',
        f'{indent}  </actuators>',
        f'{indent}  <sensors>',
        f'{indent}    <positioning implementation="default" />',
        f'{indent}    <photorealistic_lidar implementation="default" medium="pr"',
        f'{indent}                          anchor="origin"',
        f'{indent}                          position={_attr(_vec(spec.lidar_x, 0.0, lidar_z))}',
        f'{indent}                          orientation="0,0,0"',
        f'{indent}                          rings={_attr(lidar.rings)}',
        f'{indent}                          vertical_fov={_attr(_vec(-vfov_deg, vfov_deg))}',
        f'{indent}                          horizontal_resolution={_attr(f"{h_res:.4f}")}',
        f'{indent}                          max_range={_attr(_fmt(lidar.range_max))}',
        # A real time-of-flight unit is specified around +/-3 cm. Zero noise
        # hands the scan matcher an accuracy it will never have on hardware.
        f'{indent}                          range_noise_std_dev="0.03"',
        f'{indent}                          framerate_divider={_attr(_divider(LIDAR_HZ))} />',
        f'{indent}    <photorealistic_camera implementation="default" medium="pr"',
        f'{indent}                           anchor="origin"',
        f'{indent}                           position={_attr(_vec(spec.camera_x, 0.0, camera_z))}',
        f'{indent}                           orientation="0,0,0"',
        f'{indent}                           resolution={_attr(_vec(*CAMERA_RESOLUTION))}',
        f'{indent}                           fov={_attr(_fmt(CAMERA_FOV_DEG))}',
        f'{indent}                           near="0.05" far="40"',
        f'{indent}                           modalities="rgb,depth"',
        f'{indent}                           framerate_divider={_attr(_divider(CAMERA_HZ))} />',
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
        f'{indent}    <odometry {odometry_attrs} />',
        f'{indent}  </sensors>',
        f'{indent}  <params robot_id={_attr(rid)}',
        f'{indent}          track_gauge={_attr(_fmt(spec.track_gauge))}',
        f'{indent}          max_speed="150.0" />',
        f'{indent}</swarmdeck_robot_controller>',
    ]
    return lines


def generate_argos_xml(
    config_path: Path,
    robot_count: int | None = None,
    world_gltf: str = "indoor.gltf",
    bistro_dir: Path | str | None = None,
    props_dir: str = "props",
    socket_path: str = f"{RUNTIME_DIR}/argos.sock",
    uf_socket_path: str = f"{RUNTIME_DIR}/uf.sock",
    targets: int = 10,
    headless: bool = True,
    estimator: bool | None = None,
    odometry: str | None = None,
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
    if isinstance(world_cfg, dict):
        world_type = str(world_cfg.get("name") or world_cfg.get("type", "procedural")).lower()
        if not bistro_dir and (world_cfg.get("dir") or world_cfg.get("assets_dir")):
            bistro_dir = world_cfg.get("dir") or world_cfg.get("assets_dir")
    else:
        world_type = str(world_cfg).lower()

    is_bistro = (world_type == "bistro") or (world_gltf == "bistro")

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

    types = robot_types(fleet_cfg, count, prefix)
    robot_ids = [f"{prefix}{i}" for i in range(count)]

    default_odom = odometry
    if default_odom is None and estimator is not None:
        default_odom = "fast_livo2" if estimator else "drift"
    elif default_odom == "external":
        default_odom = "fast_livo2"

    odom_types_list = odometry_types(fleet_cfg, count, prefix, default_override=default_odom)
    odom_specs = [odometry_spec(o) for o in odom_types_list]

    lines: list[str] = [
        '<?xml version="1.0" ?>',
        "<!-- GENERATED by swarmdeck_sim/scenario/make_argos_session.py.",
        f"     Config: {config_path.name}. Do not edit by hand: the next",
        "     session launch overwrites it. -->",
        "<argos-configuration>",
        "",
        "  <framework>",
        '    <system threads="0" />',
        f'    <experiment length="0" ticks_per_second="{TICKS_PER_SECOND}"',
        f'                random_seed="{seed}" />',
        "  </framework>",
        "",
        "  <controllers>",
    ]

    for rid, profile, odom_s in zip(robot_ids, types, odom_specs):
        lines.extend(
            controller_block(rid, profile, robot_spec(profile), lidar, odom_s)
        )

    lines.extend([
        "  </controllers>",
        "",
        "  <!-- The single loop-function slot is the ROS boundary. Ultra-Fusion",
        "       is a <media> below precisely because this slot is taken. -->",
        "  <loop_functions",
        f'      library={_attr(LIBRARY)}',
        '      label="swarmdeck_bridge"',
        f'      socket={_attr(socket_path)}',
        f'      robots={_attr(",".join(robot_ids))}',
        f'      exchange_period={_attr(_divider(EXCHANGE_HZ))}',
        '      realtime_factor="1"',
        '      connect_timeout="180" />',
        "",
    ])

    if is_bistro:
        b_dir = find_bistro_assets_dir(bistro_dir)
        bistro_glb_path = (b_dir / "bistro_exterior.glb").resolve()
        if world_gltf and world_gltf not in ("indoor.gltf", "bistro"):
            bistro_glb_path = Path(world_gltf)
        lamps_inc_path = b_dir / "bistro_lamps.inc"
        ibl_path = b_dir / "san_giuseppe_ibl.ktx"
        lamps_xml = _sanitize_xml_comments(lamps_inc_path.read_text().rstrip()) if lamps_inc_path.exists() else ""

        lines.extend([
            f'  <arena size="{BISTRO_ARENA_SIZE}" center="{BISTRO_ARENA_CENTER}">',
            "",
            "    <!-- Collision geometry: Jolt physics mesh directly from the glTF model. -->",
            f'    <mesh id="world_mesh" file={_attr(str(bistro_glb_path))}',
            '          position="0,0,-0.3" orientation="0,0,90" scale="1.0" />',
        ])
        placements = BISTRO_TARGET_PLACEMENTS[:targets] if targets > 0 else []
    else:
        # 26 m of building plus clearance; nothing in the world reaches 6 m.
        lines.extend([
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
            f'    <mesh id="world_mesh" file={_attr(world_collision)}',
            '          position="0,0,0" orientation="0,0,90" scale="1.0" />',
        ])
        placements = place_targets(seed, targets)

    classes = target_classes(len(placements))
    if placements:
        lines.append("")
        lines.append("    <!-- Detection targets: one collision mesh each, drawn by")
        lines.append("         the matching <prop>. Classes come from")
        lines.append("         adapters/perception/catalog.py. -->")
        for i, ((x, y, yaw), name) in enumerate(zip(placements, classes)):
            model = f"{props_dir}/{name}.glb"
            orientation = _vec(math.degrees(yaw), 0.0, 90.0)
            lines.append(
                f'    <mesh id={_attr(f"target_{i}_{name}")} file={_attr(model)}'
            )
            lines.append(
                f'          position={_attr(_vec(x, y, 0.0))} '
                f'orientation={_attr(orientation)} scale="1.0" />'
            )

    lines.append("")
    for i, (rid, profile) in enumerate(zip(robot_ids, types)):
        entity = profile
        pose = starts.get(rid)
        if pose is None and is_bistro:
            pose = BISTRO_DEFAULT_START_POSES.get(rid)
        if pose is None:
            x = (i - count / 2.0) * 3.0
            y = 0.0
            yaw_deg = 0.0
        else:
            x = float(pose.get("x", (i - count / 2.0) * 3.0))
            y = float(pose.get("y", 0.0))
            yaw_deg = math.degrees(float(pose.get("yaw", 0.0)))
        # The origin anchor is on the floor and the body stands on it, so z is
        # zero plus a hair of clearance to settle rather than interpenetrate
        # the floor on the first physics step.
        lines.extend([
            f'    <{entity} id={_attr(rid)}>',
            f'      <body position={_attr(_vec(x, y, 0.02))} '
            f'orientation={_attr(_vec(yaw_deg, 0.0, 0.0))} />',
            f'      <controller config={_attr(rid + "_ctrl")} />',
            f'    </{entity}>',
        ])

    lines.extend([
        "  </arena>",
        "",
        "  <physics_engines>",
        '    <jolt id="jolt" iterations="10" threads="1">',
        '      <floor height="0" />',
        '      <gravity g="9.81" />',
        "    </jolt>",
        "  </physics_engines>",
        "",
        "  <media>",
    ])

    uf_robots = [rid for rid, odom_s in zip(robot_ids, odom_specs) if odom_s.medium == "uf"]
    if uf_robots:
        lines.extend([
        "    <!-- Fast-LIVO2. `channels` names only what the estimator reads:",
        "         the camera is declared on every robot but not streamed here,",
        "         and an unread VLP-16 revolution is ~630 KB per robot per",
        "         100 ms of traffic that only starves the channels that matter.",
        "",
        "         alignment=\"none\" on purpose. Each robot's estimate starts at",
        "         its own origin, exactly as a real robot's does, which is the",
        "         premise every merge_mode in configs/ is written against.",
        "         alignment=\"ground_truth\" would hand every robot the shared",
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
        ])

    if is_bistro:
        lines.extend([
            '    <photorealism id="pr" backend="vulkan" draw_floor="false">',
        ])
        if ibl_path.exists():
            lines.append(f'      <environment ibl={_attr(str(ibl_path))} intensity="{BISTRO_SKY_LUX:g}" />')
        lines.extend([
            f'      <skybox color="{BISTRO_SKY_COLOR}" />',
            f'      <sun direction="0.85,0.35,-0.22" intensity="{BISTRO_SUN_LUX:g}" cast_shadows="false" />',
            f'      <exposure aperture="{BISTRO_APERTURE:g}" shutter_speed="{BISTRO_SHUTTER:g}" sensitivity="{BISTRO_ISO:g}" />',
            '      <lights>',
        ])
        if lamps_xml:
            for lamp_line in lamps_xml.splitlines():
                lines.append(f"        {lamp_line}" if lamp_line.startswith("<") else f"      {lamp_line}")
        lines.extend([
            '      </lights>',
            '      <scenery>',
            f'        <prop model={_attr(str(bistro_glb_path))} position="0,0,-0.3"',
            '              orientation="0,0,90" scale="1.0" />',
        ])
        for (x, y, yaw), name in zip(placements, classes):
            model = f"{props_dir}/{name}.glb"
            orientation = _vec(math.degrees(yaw), 0.0, 90.0)
            lines.append(
                f'        <prop model={_attr(model)} '
                f'position={_attr(_vec(x, y, 0.0))} '
                f'orientation={_attr(orientation)} scale="1.0" />'
            )
        lines.extend([
            '      </scenery>',
            '    </photorealism>',
            '  </media>',
        ])
    else:
        lines.extend([
            "    <!-- draw_floor is false because indoor.gltf carries its own floor",
            "         slab; the built-in one would z-fight with it. -->",
            '    <photorealism id="pr" backend="vulkan" draw_floor="false">',
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
            "      </lights>",
            "      <scenery>",
            f'        <prop model={_attr(world_gltf)} position="0,0,0"',
            '              orientation="0,0,90" scale="1.0" />',
        ])

        for (x, y, yaw), name in zip(placements, classes):
            model = f"{props_dir}/{name}.glb"
            orientation = _vec(math.degrees(yaw), 0.0, 90.0)
            lines.append(
                f'        <prop model={_attr(model)} '
                f'position={_attr(_vec(x, y, 0.0))} '
                f'orientation={_attr(orientation)} scale="1.0" />'
            )

        lines.extend([
            "      </scenery>",
            "    </photorealism>",
            "  </media>",
        ])

    if not headless:
        if is_bistro:
            lines.extend([
                "",
                "  <visualization>",
                '    <filament medium="pr" resolution="1280,720" speed="1"',
                '              near="0.3" far="400"',
                '              position="-7.25,-12.75,3.0" look_at="-9.5,0.5,1.2" />',
                "  </visualization>",
            ])
        else:
            lines.extend([
                "",
                "  <visualization>",
                '    <filament medium="pr" resolution="1280,720" speed="1"',
                '              near="0.3" far="80"',
                '              position="0,-20,14" look_at="0,0,1" />',
                "  </visualization>",
            ])

    lines.extend(["", "</argos-configuration>", ""])
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, help="SwarmDeck session YAML")
    ap.add_argument("-o", "--output", default="session.argos")
    ap.add_argument("--robots", type=int, default=None, help="Override robot count")
    ap.add_argument("--world", default="indoor.gltf",
                    help="Path to the generated world glTF, or world name, as ARGoS will "
                         "resolve it (relative paths are relative to the "
                         "working directory argos3 runs in)")
    ap.add_argument("--bistro-dir", default=None,
                    help="Directory holding Bistro assets (bistro_exterior.glb, etc.)")
    ap.add_argument("--props-dir", default="props",
                    help="Directory holding the detection-target models")
    ap.add_argument("--targets", type=int, default=10,
                    help="How many detection targets to place; classes are "
                         "assigned round robin from the catalog")
    ap.add_argument("--socket", default=f"{RUNTIME_DIR}/argos.sock")
    ap.add_argument("--uf-socket", default=f"{RUNTIME_DIR}/uf.sock")
    ap.add_argument("--headless", action="store_true", default=True)
    ap.add_argument("--odometry", default=None,
                    help="Default odometry profile (e.g. fast_livo2, drift, ekf). "
                         "Overrides fleet.odometry default in session config.")
    ap.add_argument("--no-estimator", dest="odometry", action="store_const",
                    const="drift", help="Deprecated alias for --odometry drift.")
    ap.add_argument("--gui", dest="headless", action="store_false",
                    help="Add the interactive Filament viewer")
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = REPO / args.config

    xml = generate_argos_xml(
        config_path=cfg_path,
        robot_count=args.robots,
        world_gltf=args.world,
        bistro_dir=args.bistro_dir,
        props_dir=args.props_dir,
        socket_path=args.socket,
        uf_socket_path=args.uf_socket,
        targets=args.targets,
        headless=args.headless,
        odometry=args.odometry,
    )

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml)
    print(f"[make_argos_session] {cfg_path.name} -> {out} ({len(xml)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
