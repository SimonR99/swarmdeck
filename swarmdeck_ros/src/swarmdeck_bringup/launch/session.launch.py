"""One-command launch for the ARGoS simulated stack."""

import json
import os
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REPO = Path(__file__).resolve().parents[4]

BRINGUP_DELAY = 20.0
ROBOT_STAGGER = 10.0
INFLATION_MARGIN = 0.25
def prebuilt_world(cfg: dict) -> str | None:
    """Which building the ARGoS backend loads.

    `world: bistro` selects the Amazon Lumberyard Bistro scenario, whose assets
    are mounted rather than generated; the name is returned. Anything else
    (including no key at all) is the procedural indoor world, which
    make_argos_world.py writes from the session seed, and None is returned.
    Accepts the mapping form (`world: {name: bistro}`) as well.
    """
    world_cfg = cfg.get("world") or cfg.get("environment") or "procedural"
    if isinstance(world_cfg, dict):
        name = world_cfg.get("name") or world_cfg.get("type", "procedural")
    else:
        name = world_cfg
    return "bistro" if str(name).lower() == "bistro" else None


def argos_actions(
    cfg,
    cfg_path,
    count,
    seed,
    runtime_dir,
    headless,
    launch_argos,
    targets,
    odometry,
):
    """Generate the ARGoS world and experiment, then start the bridge.

    Order matters. The bridge BINDS the socket and ARGoS DIALS it, so the
    bridge has to exist first; the loop function waits (connect_timeout in the
    generated file) rather than failing, but only for as long as that timeout.

    The world and the experiment are generated here, in the ROS container,
    because this is where the session config lives. ARGoS reads both from the
    shared runtime directory. Paths handed to the generator are absolute, so
    the working directory argos3 happens to start in cannot change which
    building it loads.
    """
    sim_share = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim"
    scenario = sim_share / "scenario"
    nodes_dir = sim_share / "nodes"
    props = REPO / "argos" / "assets" / "props"
    world = Path(runtime_dir) / "indoor.gltf"
    experiment = Path(runtime_dir) / "session.argos"
    socket = Path(runtime_dir) / "argos.sock"
    uf_socket = Path(runtime_dir) / "uf.sock"

    Path(runtime_dir).mkdir(parents=True, exist_ok=True)

    prebuilt = prebuilt_world(cfg)

    actions = []
    if prebuilt is None:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(scenario / "make_argos_world.py"),
                    "--seed",
                    str(seed),
                    "-o",
                    str(world),
                ],
                output="screen",
            )
        )
    actions.append(
        ExecuteProcess(
            cmd=[
                "python3",
                str(scenario / "make_argos_session.py"),
                "--config",
                str(cfg_path),
                "-o",
                str(experiment),
                "--robots",
                str(count),
                "--world",
                prebuilt or str(world),
                "--props-dir",
                str(props),
                "--targets",
                str(targets),
                "--socket",
                str(socket),
                "--uf-socket",
                str(uf_socket),
                "--odometry",
                odometry,
                "--threads",
                str(count),
            ]
            + ([] if headless else ["--gui"]),
            output="screen",
        )
    )
    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "python3",
                        str(nodes_dir / "swarmdeck_argos_bridge.py"),
                        "--socket",
                        str(socket),
                        "--config",
                        str(cfg_path),
                    ],
                    output="screen",
                )
            ],
        )
    )

    if launch_argos:
        # Host development runs the simulator here. Under Compose the `argos`
        # service owns it, because that container is the one with Vulkan, the
        # Filament runtime and no ROS at all.
        actions.append(
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=["argos3", "-c", str(experiment)],
                        cwd=str(runtime_dir),
                        output="screen",
                    )
                ],
            )
        )
    return actions


def sensor_mount_transforms(ns: str, robot) -> list[Node]:
    """Static ``base_link -> sensor`` edges for one simulated robot.

    The ARGoS bridge stamps sensor messages ``<ns>/base_link/<sensor>``, which
    nothing else publishes. The peer bridge needs the lidar mount to place a
    capture at its capture-time pose, and the local costmap needs both scan
    frames. The numbers come from RobotSpec, relative to base_link; the
    generated experiment adds base_height itself. Hardware profiles get these
    from their URDF instead.
    """
    mounts = (
        ("lidar_tf", "lidar", f"{robot.lidar_x:.4f}", f"{robot.lidar_z:.4f}"),
        ("imu_tf", "imu", "0", "0"),
        ("proximity_lidar_tf", "proximity_lidar", "0.24", "0.05"),
        ("camera_tf", "camera", f"{robot.camera_x:.4f}", f"{robot.camera_z:.4f}"),
    )
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            namespace=ns,
            arguments=[
                "--x", x, "--z", z,
                "--frame-id", f"{ns}/base_link",
                "--child-frame-id", f"{ns}/base_link/{sensor}",
            ],
            parameters=[{"use_sim_time": True}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
        for name, sensor, x, z in mounts
    ]


def setup(context, *args, **kwargs):
    cfg_arg = LaunchConfiguration("config").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    runtime_dir = LaunchConfiguration("runtime_dir").perform(context)
    launch_argos = (
        LaunchConfiguration("launch_argos").perform(context).lower() == "true"
    )
    targets = int(LaunchConfiguration("targets").perform(context))
    odometry = LaunchConfiguration("odometry").perform(context).lower()

    cfg_path = Path(cfg_arg)
    if not cfg_path.is_absolute():
        cfg_path = REPO / cfg_arg
    cfg = yaml.safe_load(cfg_path.read_text())
    count = int(cfg.get("fleet", {}).get("robot_count", 4))
    env_count = os.environ.get("SWARMDECK_ROBOT_COUNT", "").strip()
    if env_count:
        count = int(env_count)
    else:
        try:
            persisted = json.loads((REPO / "sessions" / "settings.json").read_text())
            count = int(persisted.get("robot_count", count))
        except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError, OSError):
            pass
    count = max(1, min(count, 5))
    prefix = cfg.get("fleet", {}).get("robot_prefix", "robot_")
    seed = cfg.get("seed", 20260801)

    scenario = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim" / "scenario"
    sys.path.insert(0, str(scenario))
    from spawn_fleet import robot_spec, robot_types  # noqa: E402

    types = robot_types(cfg.get("fleet", {}), count, prefix)
    actions = argos_actions(
        cfg,
        cfg_path,
        count,
        seed,
        runtime_dir,
        headless,
        launch_argos,
        targets,
        odometry,
    )

    for i in range(count):
        ns = f"{prefix}{i}"
        robot = robot_spec(types[i])
        actions.append(
            TimerAction(
                period=BRINGUP_DELAY + i * ROBOT_STAGGER,
                actions=sensor_mount_transforms(ns, robot)
                + [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("swarmdeck_nav"), "/launch/nav.launch.py"]
                        ),
                        launch_arguments={
                            "namespace": ns,
                            "use_sim_time": "true",
                            "robot_base_frame": f"{ns}/base_link",
                            "bounded_startup": "true",
                            "robot_radius": f"{robot.footprint_radius:.3f}",
                            "footprint": robot.footprint,
                            "inflation_radius": f"{robot.footprint_radius + INFLATION_MARGIN:.3f}",
                        }.items(),
                    )
                ],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value="configs/4robot.yaml"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("runtime_dir", default_value="/run/swarmdeck"),
            DeclareLaunchArgument("launch_argos", default_value="false"),
            DeclareLaunchArgument("odometry", default_value="fast_livo2"),
            DeclareLaunchArgument("targets", default_value="10"),
            OpaqueFunction(function=setup),
        ]
    )
