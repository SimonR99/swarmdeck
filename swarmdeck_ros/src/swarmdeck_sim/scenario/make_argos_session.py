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

**The world appears twice.** `indoor.gltf` is both the Jolt `<mesh>` the robots
collide with and the photorealism `<prop>` the cameras and the photorealistic
lidar raytrace, with the same position, orientation and scale on both. If the
two ever disagree, robots collide with a building that is not where it is
drawn, and nothing says so.

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
import sys
from pathlib import Path
from xml.sax.saxutils import quoteattr

import yaml

HERE = Path(__file__).resolve().parent
# scenario -> swarmdeck_sim -> src -> swarmdeck_ros -> repo root.
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE))

from generate_world import place_targets  # noqa: E402
from make_argos_world import target_classes  # noqa: E402
from spawn_fleet import lidar_spec, robot_spec, robot_types  # noqa: E402

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
CAMERA_RESOLUTION = (320, 240)
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
                     estimator: bool = True, indent: str = "    ") -> list[str]:
    """One robot's controller, with its sensors mounted from `RobotSpec`.

    Sensors hang off the `origin` anchor with explicit positions rather than
    off the entity's named `lidar`/`camera` anchors. The entity plugins do
    define those anchors, but they hardcode one platform's mounts in C++,
    whereas these numbers have to follow the fleet config that Nav2 and SLAM
    are configured from at the same time.

    `RobotSpec` mounts are relative to base_link, which floats `base_height`
    above the floor; the ARGoS origin anchor is ON the floor. Hence the sum.
    """
    odometry_attrs = ('implementation="external" medium="uf"' if estimator
                      else 'implementation="drift"')
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
    props_dir: str = "props",
    socket_path: str = f"{RUNTIME_DIR}/argos.sock",
    uf_socket_path: str = f"{RUNTIME_DIR}/uf.sock",
    targets: int = 10,
    headless: bool = True,
    estimator: bool = True,
) -> str:
    cfg = yaml.safe_load(config_path.read_text())
    fleet_cfg = cfg.get("fleet", {}) or {}
    count = int(fleet_cfg.get("robot_count", 4)) if robot_count is None else robot_count
    count = max(1, min(count, 5))
    prefix = fleet_cfg.get("robot_prefix", "robot_")
    seed = int(cfg.get("seed", 20260801))
    starts = (cfg.get("map", {}) or {}).get("start_poses", {}) or {}

    lidar = lidar_spec(fleet_cfg)
    if lidar.rings < MIN_LIDAR_RINGS:
        raise ValueError(
            f"the ARGoS backend fuses odometry with Ultra-Fusion, which is "
            f"lidar-inertial, but the lidar resolves to {lidar.rings} ring(s) "
            f"in {config_path.name}. A planar scan carries no vertical "
            f"structure to register on, so the estimator never converges and "
            f"every robot stands still with no odometry at all. Set "
            f"fleet.lidar.profile to vlp16 or generic_32."
        )

    types = robot_types(fleet_cfg, count, prefix)
    robot_ids = [f"{prefix}{i}" for i in range(count)]

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

    for rid, profile in zip(robot_ids, types):
        lines.extend(
            controller_block(rid, profile, robot_spec(profile), lidar, estimator)
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
        # 26 m of building plus clearance; nothing in the world reaches 6 m.
        '  <arena size="30,30,6" center="0,0,3">',
        "",
        "    <!-- Collision geometry. The photorealism <prop> at the bottom of",
        "         this file draws the SAME file with the SAME transform. -->",
        f'    <mesh id="world_mesh" file={_attr(world_gltf)}',
        '          position="0,0,0" orientation="0,0,90" scale="1.0" />',
    ])

    placements = place_targets(seed, targets)
    classes = target_classes(len(placements))
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
        pose = starts.get(rid) or {}
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

    if estimator:
        lines.extend([
        "    <!-- Ultra-Fusion. `channels` names only what the estimator reads:",
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
        f'                        robots={_attr(",".join(robot_ids))}',
        '                        lockstep_pose="false"',
        '                        channels="imu,lidar,wheels"',
        '                        alignment="none"',
        '                        connect_timeout="180" timeout="120" />',
        "",
        ])

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
                    help="Path to the generated world glTF, as ARGoS will "
                         "resolve it (relative paths are relative to the "
                         "working directory argos3 runs in)")
    ap.add_argument("--props-dir", default="props",
                    help="Directory holding the detection-target models")
    ap.add_argument("--targets", type=int, default=10,
                    help="How many detection targets to place; classes are "
                         "assigned round robin from the catalog")
    ap.add_argument("--socket", default=f"{RUNTIME_DIR}/argos.sock")
    ap.add_argument("--uf-socket", default=f"{RUNTIME_DIR}/uf.sock")
    ap.add_argument("--headless", action="store_true", default=True)
    ap.add_argument("--no-estimator", dest="estimator", action="store_false",
                    default=True,
                    help="DIAGNOSTICS ONLY. Drop the Ultra-Fusion medium and "
                         "fall back to ARGoS's synthetic <odometry "
                         "implementation=\"drift\"> so the render, physics and "
                         "bridge can be exercised without the estimator "
                         "sidecar. The launch path never passes this: a drift "
                         "model cannot slip a wheel or lose a scan, which is "
                         "the entire reason the estimator is there.")
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
        props_dir=args.props_dir,
        socket_path=args.socket,
        uf_socket_path=args.uf_socket,
        targets=args.targets,
        headless=args.headless,
        estimator=args.estimator,
    )

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml)
    print(f"[make_argos_session] {cfg_path.name} -> {out} ({len(xml)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
