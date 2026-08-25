"""One-command launch for the whole simulated stack (acceptance criterion 13).

    ros2 launch swarmdeck_bringup session.launch.py config:=configs/4robot.yaml

Brings up: Gazebo world -> fleet spawn -> ros_gz bridges -> per-robot odometry
fusion + SLAM + Nav2. The backend and UI run separately (`make server`, `make ui`)
because they are ROS-free by design.

Arguments worth knowing:

`slam_backend:=rtabmap` swaps 2D SLAM Toolbox for RTAB-Map on the 3D cloud, which
needs a multi-ring lidar — set `fleet.lidar.profile: generic_32` in the study
config (see spawn_fleet.py's LIDAR_PROFILES).

`fuse_imu:=false` reverts to the drive plugin's raw wheel odometry. Only useful for
reproducing what unfused odometry does to a map; wheel odometry alone was measured
8.8-30.5 m and up to 244 deg wrong on a 24 m floor plan.

`explore_seconds:=N` drives the fleet reactively for N seconds to bootstrap the
maps, then stops. Nothing moves without either this or an operator goal, and
issuing goals against an empty map is how robots end up jammed against walls.
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


def setup(context, *args, **kwargs):
    cfg_arg = LaunchConfiguration("config").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    slam_backend = LaunchConfiguration("slam_backend").perform(context).lower()
    fuse_imu = LaunchConfiguration("fuse_imu").perform(context).lower() == "true"
    fuse_cov = LaunchConfiguration("fuse_covariance").perform(context).lower() == "true"
    grid_3d = LaunchConfiguration("grid_3d").perform(context).lower() == "true"
    explore_seconds = float(LaunchConfiguration("explore_seconds").perform(context))

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

    sim_share = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim"
    world = sim_share / "worlds" / "indoor.sdf"
    scenario = sim_share / "scenario"

    # Resolve the lidar profile through the spawner's own code rather than
    # re-reading the YAML here, so the ring count this file bridges on and the
    # geometry Gazebo is actually given can never disagree. Imported lazily:
    # test_session_launch.py imports this module for `bridge_args` alone and
    # should not need the sim package on its path.
    sys.path.insert(0, str(scenario))
    # noqa: E402 — path set immediately above. Resolved through the spawner's own
    # code so the geometry this file configures SLAM and Nav2 with is the same
    # geometry Gazebo was handed; two tables would drift.
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

    actions = [
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
    ]

    # Gazebo sensor and TF stamps use simulation time. One global clock bridge
    # serves every namespaced robot stack.
    actions.append(
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
        )
    )

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
            }
            slam_launch = "/launch/slam.launch.py"

        actions.append(
            TimerAction(
                period=BRINGUP_DELAY + i * ROBOT_STAGGER,
                actions=[
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
                    ),
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
                ],
            )
        )

    # Reactive exploration, for a bounded period, then the fleet stops and the
    # operator drives it through Nav2. Without this nothing moves until someone
    # sends a goal, and Nav2 sending goals against an empty map drives robots into
    # walls, where a differential drive spins its wheels and destroys its own
    # odometry — see scenario/explore.py. `explore.py` publishes cmd_vel directly,
    # so it fights Nav2 while it runs; that is why it stops rather than persisting.
    if explore_seconds > 0:
        actions.append(
            TimerAction(
                period=(BRINGUP_DELAY + min(count, 5) * ROBOT_STAGGER
                        + EXPLORE_LEAD_IN),
                actions=[
                    ExecuteProcess(
                        cmd=["python3", str(scenario / "explore.py"),
                             "--robots", str(min(count, 5)),
                             "--prefix", prefix,
                             "--seconds", str(explore_seconds),
                             "--seed", str(seed),
                             # Configured spawn poses, so the explorer can send
                             # scheduled pairs to a shared meeting point. Every
                             # robot steers in its own odom frame, so without
                             # these a common rendezvous cannot be expressed and
                             # inter-robot encounters stay a matter of luck.
                             "--start-poses", json.dumps(
                                 cfg.get("map", {}).get("start_poses", {})
                             ),
                             # Per-robot chassis size. The explorer's clearances
                             # were absolute constants tuned on a 0.27 m
                             # Duckiebot; on a fleet whose largest member is
                             # 0.64 m they let a Bunker drive to within 0.9 m of
                             # a neighbour and then rotate into it.
                             "--radii", json.dumps({
                                 f"{prefix}{i}": round(
                                     robot_spec(types[i]).footprint_radius, 3
                                 )
                                 for i in range(min(count, 5))
                             })],
                        output="screen",
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
                description="EKF-fuse wheel odometry with the gyro and let it own "
                            "odom -> base_link, instead of trusting the drive "
                            "plugin's slip-corrupted heading.",
            ),
            DeclareLaunchArgument(
                "fuse_covariance",
                default_value="false",
                description="Feed the EKF real per-measurement covariance from "
                            "covariance_relay.py. False reproduces the filter as "
                            "it behaved on Gazebo's all-zero covariance.",
            ),
            DeclareLaunchArgument(
                "grid_3d",
                default_value="false",
                description="rtabmap backend only: keep occupancy in 3D so the "
                            "GUI's 3D view shows real structure instead of a flat "
                            "plane. Halves the real-time factor.",
            ),
            DeclareLaunchArgument(
                "explore_seconds",
                default_value="0",
                description="Run reactive exploration for this many seconds after "
                            "startup to bootstrap the maps, then hand control back "
                            "to Nav2. 0 disables it.",
            ),
            OpaqueFunction(function=setup),
        ]
    )
