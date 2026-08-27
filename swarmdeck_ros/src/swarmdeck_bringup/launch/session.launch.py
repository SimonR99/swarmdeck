"""One-command launch for the whole simulated stack (acceptance criterion 13).

    ros2 launch swarmdeck_bringup session.launch.py config:=configs/4robot.yaml

Two simulation backends, selected with `sim_backend`.

**`argos`** (the default). Generates the world glTF and the ARGoS experiment
from the session config, starts the ROS bridge, and brings up per-robot SLAM
and Nav2. The simulator itself normally runs in its own container and dials the
bridge's socket; `launch_argos:=true` starts it here instead, for host
development. Odometry comes from Ultra-Fusion through the bridge, so no EKF is
launched, and `<ns>/proximity_scan` is derived from the 3D cloud because an
ARGoS robot carries one lidar rather than two.

**`gazebo`** (legacy, kept as an A/B control). World SDF -> `gz sim` -> fleet
spawn -> `ros_gz` bridges -> EKF + SLAM + Nav2, exactly as before.

The backend and UI run separately (`make server`, `make ui`) because they are
ROS-free by design.

Arguments worth knowing:

`slam_backend:=rtabmap` swaps 2D SLAM Toolbox for RTAB-Map on the 3D cloud, which
needs a multi-ring lidar — set `fleet.lidar.profile: generic_32` in the study
config (see spawn_fleet.py's LIDAR_PROFILES).

`fuse_imu:=false` reverts to the drive plugin's raw wheel odometry. Gazebo
backend only, and only useful for reproducing what unfused odometry does to a
map; wheel odometry alone was measured 8.8-30.5 m and up to 244 deg wrong on a
24 m floor plan.

Exploration is not launched here. `adapter_sim` owns that process so the
dashboard can start and stop it, and bootstraps one run at startup when its
`EXPLORE_SECONDS` environment variable is positive. Nothing moves without
either that or an operator goal, and issuing goals against an empty map is how
robots end up jammed against walls.
"""

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

# Seconds after launch before the first robot's bringup, leaving Gazebo and the
# fleet spawn time to settle.
BRINGUP_DELAY = 20.0
# Gap between successive robots' stacks. See the comment at the loop below: they
# lose a startup race against each other if brought up simultaneously.
ROBOT_STAGGER = 6.0
# Exploration starts after the last robot's stack is up, plus lifecycle settling.
EXPLORE_LEAD_IN = 25.0
# Costmap inflation beyond the chassis radius, metres. This is the knob that
# decides how far robots stay off walls AND off each other, since the bumper
# scan writes neighbours into the local costmap as ordinary obstacles.
INFLATION_MARGIN = 0.25


def bridge_args(ns: str, lidar_rings: int = 1, fuse_imu: bool = True) -> list[str]:
    """gz <-> ROS 2 topic bridges for one robot.

    With one ring, Gazebo's own LaserScan on `<ns>/scan` is exactly the planar
    scan SLAM wants, so bridge it. With several rings that message is no longer a
    planar slice, and `pointcloud_to_laserscan` publishes `<ns>/scan` instead —
    bridging it too would put two publishers on one topic and let SLAM alternate
    between them.

    The drive plugin's `<ns>/tf` carries odom -> base_link. When the EKF is fusing
    the gyro it publishes that same transform, so the bridge is dropped: two
    publishers of one transform produce a TF tree that flickers between a
    slip-corrupted estimate and a corrected one, which is worse than either.
    """
    args = [
        f"/{ns}/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        f"/{ns}/proximity_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        # Three of the four streams an `rgbd_camera` publishes. The colour image
        # is the operator's video and the detector's input; the depth image and
        # the intrinsics are what let the adapter turn a duck detection into a
        # point on the map (adapter_sim._depth_map_position). The fourth,
        # `<ns>/camera/points`, is the same geometry as an organised cloud at
        # about 70 MB/s per robot and is deliberately left inside Gazebo.
        f"/{ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        f"/{ns}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        f"/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        f"/{ns}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]
    if not fuse_imu:
        args.append(f"/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
    if lidar_rings == 1:
        args.insert(0, f"/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
    return args


def argos_actions(context, cfg_path, count, prefix, seed, runtime_dir,
                  headless, launch_argos, targets, odometry):
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
    scenario = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim" / "scenario"
    nodes_dir = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim" / "nodes"
    props = REPO / "argos" / "assets" / "props"
    world = Path(runtime_dir) / "indoor.gltf"
    experiment = Path(runtime_dir) / "session.argos"
    socket = Path(runtime_dir) / "argos.sock"
    uf_socket = Path(runtime_dir) / "uf.sock"

    Path(runtime_dir).mkdir(parents=True, exist_ok=True)

    actions = [
        ExecuteProcess(
            cmd=["python3", str(scenario / "make_argos_world.py"),
                 "--seed", str(seed), "-o", str(world)],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(scenario / "make_argos_session.py"),
                 "--config", str(cfg_path),
                 "-o", str(experiment),
                 "--robots", str(count),
                 "--world", str(world),
                 "--props-dir", str(props),
                 "--targets", str(targets),
                 "--socket", str(socket),
                 "--uf-socket", str(uf_socket),
                 "--odometry", odometry]
                + ([] if headless else ["--gui"]),
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", str(nodes_dir / "swarmdeck_argos_bridge.py"),
                         "--socket", str(socket)],
                    output="screen",
                )
            ],
        ),
    ]

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


def gazebo_actions(context, cfg_path, count, seed, headless, lidar_rings):
    """The legacy backend, unchanged: world SDF, gz sim, fleet spawn, clock."""
    sim_share = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim"
    world = sim_share / "worlds" / "indoor.sdf"
    scenario = sim_share / "scenario"
    return [
        # Regenerate the world from the seed so the run is reproducible.
        ExecuteProcess(
            cmd=["python3", str(scenario / "generate_world.py"),
                 "--seed", str(seed), "-o", str(world)],
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["gz", "sim", "-s" if headless else "", "-r",
                         "--headless-rendering" if headless else "", str(world)],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", str(scenario / "spawn_fleet.py"),
                         "--config", str(cfg_path), "--robots", str(count),
                         "--lidar-rings", str(lidar_rings)],
                    output="screen",
                )
            ],
        ),
        # Gazebo sensor and TF stamps use simulation time. One global clock
        # bridge serves every namespaced robot stack. The ARGoS bridge
        # publishes /clock itself and needs no equivalent.
        TimerAction(
            period=BRINGUP_DELAY,
            actions=[
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="bridge_clock",
                    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                    output="screen",
                )
            ],
        ),
    ]


def setup(context, *args, **kwargs):
    cfg_arg = LaunchConfiguration("config").perform(context)
    backend = LaunchConfiguration("sim_backend").perform(context).lower()
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    slam_backend = LaunchConfiguration("slam_backend").perform(context).lower()
    fuse_imu = LaunchConfiguration("fuse_imu").perform(context).lower() == "true"
    fuse_cov = LaunchConfiguration("fuse_covariance").perform(context).lower() == "true"
    grid_3d = LaunchConfiguration("grid_3d").perform(context).lower() == "true"
    runtime_dir = LaunchConfiguration("runtime_dir").perform(context)
    launch_argos = LaunchConfiguration("launch_argos").perform(context).lower() == "true"
    targets = int(LaunchConfiguration("targets").perform(context))
    odometry = LaunchConfiguration("odometry").perform(context).lower()

    cfg_path = Path(cfg_arg)
    if not cfg_path.is_absolute():
        cfg_path = REPO / cfg_arg
    cfg = yaml.safe_load(cfg_path.read_text())

    count = int(cfg.get("fleet", {}).get("robot_count", 4))
    # Operator settings are deliberately ROS-independent, but the simulated
    # fleet consumes the persisted count on its next launch. An explicit
    # SWARMDECK_ROBOT_COUNT wins so a 2-robot graph test cannot be inflated
    # by a dashboard left at 7 from a hardware session.
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

    # Resolve the lidar profile through the spawner's own code rather than
    # re-reading the YAML here, so the ring count this file bridges on and the
    # geometry the simulator is actually given can never disagree. Imported
    # lazily: test_session_launch.py imports this module for `bridge_args`
    # alone and should not need the sim package on its path.
    sys.path.insert(0, str(scenario))
    from spawn_fleet import lidar_spec, robot_spec, robot_types  # noqa: E402

    spec = lidar_spec(cfg.get("fleet", {}))
    types = robot_types(cfg.get("fleet", {}), min(count, 5), prefix)
    lidar_rings = spec.rings

    # RTAB-Map registers against the cloud's vertical structure, which a
    # single-ring lidar does not have. Fail here rather than start a stack that
    # comes up healthy and maps badly.
    if slam_backend == "rtabmap" and lidar_rings < 3:
        raise RuntimeError(
            f"slam_backend:=rtabmap needs a 3D cloud, but the lidar resolves to "
            f"{lidar_rings} ring(s) in {cfg_path.name}. Set fleet.lidar.profile "
            f"to generic_32, or fleet.lidar.rings to an odd value >= 9."
        )

    argos = backend == "argos"
    if argos:
        actions = argos_actions(context, cfg_path, count, prefix, seed,
                                runtime_dir, headless, launch_argos, targets, odometry)
    else:
        actions = gazebo_actions(context, cfg_path, count, seed, headless,
                                 lidar_rings)

    # Each robot's stack starts on its own offset. Launching four lifecycle
    # managers, four SLAM nodes and four Nav2 stacks at once on a CPU-starved
    # container loses the race: `lifecycle_manager_slam` times out on
    # `slam_toolbox/get_state`, logs "Failed to bring up all requested nodes", and
    # gives up. The process stays alive but sits in `unconfigured` with no
    # subscribers, so that robot produces no map and nothing else complains.
    for i in range(min(count, 5)):
        ns = f"{prefix}{i}"
        # The platform this robot actually is. Its lidar mount has to reach SLAM,
        # because slam.launch.py publishes base_link -> lidar from these numbers
        # and the fleet is no longer uniform: a Scout Mini carries its lidar at
        # 0.245 m and a Spot at 0.500 m. Leaving the old single default in place
        # would put every Spot scan a quarter of a metre below where it was
        # taken, which is exactly the wrong-extrinsic failure adapter_ros2's
        # notes warn about — it tilts and offsets every scan and SLAM cannot
        # recover from it.
        robot = robot_spec(types[i])
        # ARGoS mounts every sensor on the origin anchor, which sits on the
        # floor; RobotSpec's numbers are relative to base_link. SLAM's static
        # transform is base_link -> lidar either way, so it takes RobotSpec
        # unchanged and the generated experiment adds base_height itself.
        if slam_backend == "rtabmap":
            slam_args = {
                "namespace": ns,
                "use_sim_time": "true",
                "range_max": str(spec.range_max),
                "grid_3d": str(grid_3d).lower(),
                "lidar_x": f"{robot.lidar_x:.4f}",
                "lidar_z": f"{robot.lidar_z:.4f}",
            }
            slam_launch = "/launch/slam_rtabmap.launch.py"
        else:
            slam_args = {
                "namespace": ns,
                "use_sim_time": "true",
                "lidar_rings": str(lidar_rings),
                "fuse_imu": str(fuse_imu).lower(),
                "fuse_covariance": str(fuse_cov).lower(),
                "range_max": str(spec.range_max),
                "lidar_x": f"{robot.lidar_x:.4f}",
                "lidar_z": f"{robot.lidar_z:.4f}",
                # ARGoS robots carry one lidar and get their fused pose from
                # Ultra-Fusion; Gazebo robots carry a bumper lidar and need the
                # EKF. See slam.launch.py.
                "odometry_source": "external" if argos else "ekf",
                "proximity_from_cloud": "true" if argos else "false",
                "proximity_range_max": f"{robot.prox_range_max:.1f}",
                "base_height": f"{robot.base_height:.4f}",
            }
            slam_launch = "/launch/slam.launch.py"

        stack = []
        if not argos:
            stack.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"bridge_{ns}",
                    # `owns_odom_tf` rather than `fuse_imu`: with the rtabmap
                    # backend it is icp_odometry that publishes
                    # odom -> base_link, so the drive plugin's TF must stay
                    # unbridged there too, whatever fuse_imu says. Two
                    # publishers of one transform give a TF tree that
                    # flickers between estimates, which is worse than either.
                    arguments=bridge_args(
                        ns, lidar_rings, fuse_imu or slam_backend == "rtabmap"
                    ),
                    output="screen",
                )
            )
        stack.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_slam"), slam_launch]
                ),
                launch_arguments=slam_args.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_nav"), "/launch/nav.launch.py"]
                ),
                launch_arguments={
                    "namespace": ns,
                    "use_sim_time": "true",
                    # Nav2 plans with a circular footprint, so this is
                    # the circumscribed radius of the chassis — an
                    # inscribed one lets a 1.02 m Bunker's corner clip a
                    # wall the planner believed was clear. It also sets
                    # how far robots keep off EACH OTHER, since the
                    # bumper scan writes them into the local costmap.
                    "robot_radius": f"{robot.footprint_radius:.3f}",
                    # The real chassis rectangle. Without it Nav2 models
                    # a 0.778 m wide Bunker as a 1.285 m disc and refuses
                    # gaps it fits through comfortably.
                    "footprint": robot.footprint,
                    # Clearance beyond the chassis before cost starts
                    # decaying. Constant margin rather than a scale
                    # factor: what it buys is room to manoeuvre next to
                    # an obstacle, and that is set by the corridor, not
                    # by how big the robot is. INFLATION_MARGIN is where
                    # inter-robot spacing is actually tuned.
                    "inflation_radius": f"{robot.footprint_radius + INFLATION_MARGIN:.3f}",
                }.items(),
            ),
        ])

        actions.append(
            TimerAction(period=BRINGUP_DELAY + i * ROBOT_STAGGER, actions=stack)
        )

    # Exploration is NOT started here. adapter_sim owns the process, because an
    # operator can now start and stop it from the dashboard, and two owners
    # means two ways to end up with two copies running. `explore.py` publishes
    # cmd_vel directly and yields it per robot to Nav2; a second instance would
    # reintroduce exactly the contention that yielding exists to prevent.
    #
    # The startup bootstrap is now the adapter's `EXPLORE_SECONDS` environment
    # variable, which sim-entrypoint.sh already exports to both processes.
    # Setting it from here would not work anyway: the adapter is this launch
    # file's SIBLING, not its child, and inherits nothing it sets.

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value="configs/4robot.yaml"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "sim_backend",
                default_value="argos",
                choices=["argos", "gazebo"],
                description="argos = ARGoS3 with the Filament photorealism "
                            "medium, Jolt physics and Ultra-Fusion odometry; "
                            "gazebo = the legacy Gazebo Harmonic path, kept "
                            "runnable as an A/B control.",
            ),
            DeclareLaunchArgument(
                "runtime_dir",
                default_value="/run/swarmdeck",
                description="ARGoS backend only. Holds the generated world and "
                            "experiment and the two Unix sockets, and is the "
                            "volume shared with the argos and ultrafusion "
                            "services. Keep it SHORT: a Unix socket path over "
                            "107 bytes fails to bind.",
            ),
            DeclareLaunchArgument(
                "launch_argos",
                default_value="false",
                description="ARGoS backend only. Start argos3 from this launch "
                            "file, for host development. Under Compose the "
                            "`argos` service owns the simulator, because it is "
                            "the container with Vulkan and no ROS.",
            ),
            DeclareLaunchArgument(
                "odometry",
                default_value="external",
                choices=["external", "drift"],
                description="ARGoS backend only. external = Ultra-Fusion, a "
                            "real lidar-inertial front-end running outside the "
                            "simulator. drift = ARGoS's synthetic drift model, "
                            "roughly 4x faster because it takes the estimator "
                            "out of the lockstep exchange, and correspondingly "
                            "less faithful: a Gaussian cannot slip a wheel or "
                            "lose a scan. For development, not for judging "
                            "mapping quality.",
            ),
            DeclareLaunchArgument(
                "targets",
                default_value="10",
                description="ARGoS backend only. Detection targets to scatter; "
                            "classes are assigned round robin from "
                            "adapters/perception/catalog.py.",
            ),
            DeclareLaunchArgument(
                "slam_backend",
                default_value="toolbox",
                choices=["toolbox", "rtabmap"],
                description="toolbox = 2D SLAM Toolbox; rtabmap = 3D cloud + "
                            "optional visual loop closure (needs a multi-ring "
                            "fleet.lidar profile)",
            ),
            DeclareLaunchArgument(
                "fuse_imu",
                default_value="true",
                description="Gazebo backend only. EKF-fuse wheel odometry with "
                            "the gyro and let it own odom -> base_link, instead "
                            "of trusting the drive plugin's slip-corrupted "
                            "heading. The ARGoS backend's pose arrives already "
                            "fused and launches no filter.",
            ),
            DeclareLaunchArgument(
                "fuse_covariance",
                default_value="false",
                description="Gazebo backend only. Feed the EKF real "
                            "per-measurement covariance from covariance_relay.py. "
                            "False reproduces the filter as it behaved on "
                            "Gazebo's all-zero covariance.",
            ),
            DeclareLaunchArgument(
                "grid_3d",
                default_value="false",
                description="rtabmap backend only: keep occupancy in 3D so the "
                            "GUI's 3D view shows real structure instead of a flat "
                            "plane. Halves the real-time factor.",
            ),
            OpaqueFunction(function=setup),
        ]
    )
