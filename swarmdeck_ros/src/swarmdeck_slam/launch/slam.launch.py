"""Per-robot 2D SLAM: planar lidar -> SLAM Toolbox -> OccupancyGrid.

`odometry_source` decides who owns `odom -> base_link`, and the two backends
answer differently.

**`external`** (the default, and the only thing the ARGoS backend uses). The
pose arrives already fused, from Ultra-Fusion running outside the simulator,
and `swarmdeck_argos_bridge.py` publishes both the topic and the transform.
Nothing is launched here. A filter on top of an already-fused estimate adds
latency and double-counts the IMU.

**`ekf`** (the legacy Gazebo backend). Gazebo's drive plugin publishes raw
wheel odometry, which was measured 8.8-30.5 m and up to 244 deg wrong on a 24 m
floor plan, so `robot_localization` fuses it with the gyro and owns the
transform instead. `fuse_imu` and `fuse_covariance` apply only here. See
ekf.yaml for what is fused and why, and note that session.launch.py must then
stop bridging the drive plugin's own TF, or two publishers fight over the same
transform.

Four things here are non-obvious and were each found the hard way, because all
four fail *silently* — no error, no log line, node looks healthy:

0. A parameter file keyed by bare node name (`slam_toolbox:`) matches nothing for
   a namespaced node, so every parameter falls back to its default. Both YAMLs in
   this package are therefore keyed `/**`.

1. `async_slam_toolbox_node` is a LIFECYCLE node. On Jazzy it sits in
   `unconfigured` with zero subscribers and logs nothing at all, so it presents
   as a hang. `nav2_lifecycle_manager` with autostart drives it to `active`.
2. The scan topic must be REMAPPED (`scan:=...`). Setting the `scan_topic`
   parameter has no effect and leaves subscription count at 0.
3. Everything needs `use_sim_time:=true` and a bridged `/clock`. Gazebo stamps
   sensors with sim time; without it TF lookups never resolve and SLAM stalls.

With `lidar_rings:=1` (the default) Gazebo publishes a usable LaserScan directly
and nothing converts anything. With an odd `lidar_rings` > 1 the lidar also
publishes a 3D cloud, and `pointcloud_to_laserscan` slices the planar scan back
out of it — see the node below for why the band is as tight as it is, and
robot.sdf.jinja for why the ring count must be odd.

`proximity_from_cloud:=true` derives `<ns>/proximity_scan` from that same cloud
as well. Gazebo carried a second, dedicated bumper lidar for it; an ARGoS robot
has one lidar, and `nav2_params.yaml` still names both sources.

This is still 2D SLAM either way. It cannot use the 3D structure, correct z/roll/
pitch drift, or close loops visually; see docs/architecture/collaborative-slam.md for what a
3D-capable per-robot SLAM would change.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_z = LaunchConfiguration("lidar_z")
    floor_z = LaunchConfiguration("floor_z")
    rings = LaunchConfiguration("lidar_rings")
    multi_ring = IfCondition(PythonExpression(['"', rings, '" != "1"']))
    fuse_imu = LaunchConfiguration("fuse_imu")
    odom_source = LaunchConfiguration("odometry_source")
    proximity = LaunchConfiguration("proximity_from_cloud")
    prox_range = LaunchConfiguration("proximity_range_max")
    fuse_cov = LaunchConfiguration("fuse_covariance")
    range_max = LaunchConfiguration("range_max")

    slam_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_slam"), "config", "slam_toolbox.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_slam"), "config", "ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            # Lidar mount, metres, relative to base_link. Arguments rather than
            # constants because these are the simulated robot's numbers: on
            # hardware they come from that unit's URDF or a calibration, and a
            # wrong extrinsic tilts every scan in a way SLAM cannot recover
            # from. See docs/operations/hardware-bringup.md.
            DeclareLaunchArgument("lidar_x", default_value="-0.07"),
            DeclareLaunchArgument("lidar_z", default_value="0.402"),
            DeclareLaunchArgument(
                "floor_z",
                default_value="0.0",
                description="Ground height in base_link for the physical "
                "0.15..1.80 m obstacle band.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("lidar_rings", default_value="1"),
            DeclareLaunchArgument(
                "range_max",
                default_value="30.0",
                description="Mapping lidar's maximum range. Passed down from the "
                "fleet's lidar profile so SLAM and the sensor agree.",
            ),
            DeclareLaunchArgument(
                "fuse_covariance",
                default_value="false",
                description="Feed the EKF covariance_relay.py's restamped topics "
                "instead of Gazebo's all-zero-covariance ones. OFF by "
                "default because it was measured to make this filter "
                "10x worse; see ekf.yaml.",
            ),
            DeclareLaunchArgument(
                "odometry_source",
                default_value="external",
                choices=["external", "ekf"],
                description="external = the bridge publishes an already-fused "
                "pose and owns odom -> base_link (ARGoS + "
                "Ultra-Fusion); ekf = robot_localization fuses "
                "wheel odometry with the gyro (legacy Gazebo).",
            ),
            DeclareLaunchArgument(
                "proximity_from_cloud",
                default_value="false",
                description="Derive <ns>/proximity_scan from the 3D cloud as a "
                "second flattened projection, for fleets with no "
                "dedicated bumper lidar.",
            ),
            DeclareLaunchArgument(
                "proximity_range_max",
                default_value="8.0",
                description="Range limit for the derived bumper scan; the "
                "costmap raytraces it to 4 m either way.",
            ),
            DeclareLaunchArgument(
                "fuse_imu",
                default_value="true",
                description="Fuse wheel odometry with the gyro and publish "
                "odom -> base_link from the EKF instead of the drive "
                "plugin. Wheel odometry alone was measured up to 30 m "
                "and 244 deg wrong.",
            ),
            # Only started when something on this path actually consumes it.
            # `fuse_covariance` is off by default (see ekf.yaml for the
            # measurement), so on the 2D path this normally does not run at all
            # rather than republishing a 200 Hz IMU for no reader. The RTAB-Map
            # path launches its own instance, because icp_odometry does want it.
            Node(
                package="swarmdeck_slam",
                executable="covariance_relay.py",
                name="covariance_relay",
                namespace=ns,
                condition=IfCondition(
                    PythonExpression(
                        ['"', odom_source, '" == "ekf" and "', fuse_cov, '" == "true"']
                    )
                ),
                parameters=[{"use_sim_time": use_sim}],
                output="screen",
            ),
            # Owns odom -> base_link when enabled. Must start before SLAM, which
            # cannot transform a scan without it.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                namespace=ns,
                # Both gates: `ekf` selects the legacy Gazebo path at all,
                # and `fuse_imu` is that path's own switch for reproducing what
                # unfused wheel odometry does to a map.
                condition=IfCondition(
                    PythonExpression(
                        ['"', odom_source, '" == "ekf" and "', fuse_imu, '" == "true"']
                    )
                ),
                parameters=[
                    ekf_params,
                    {
                        "use_sim_time": use_sim,
                        "odom_frame": [ns, "/odom"],
                        "base_link_frame": [ns, "/base_link"],
                        "world_frame": [ns, "/odom"],
                        "map_frame": [ns, "/map_frame"],
                        # Which topics the filter reads, overriding ekf.yaml.
                        # `fuse_covariance:=false` points it back at Gazebo's raw
                        # all-zero-covariance messages, which is what the filter
                        # was tuned against before covariance_relay.py existed.
                        # Kept switchable because "is the filter better with real
                        # covariance?" is a measurement, not an assumption — and
                        # the answer depends on process_noise_covariance, which
                        # was calibrated for the old behaviour.
                        "odom0": PythonExpression(
                            ['"odom_cov" if "', fuse_cov, '" == "true" else "odom"']
                        ),
                        "imu0": PythonExpression(
                            ['"imu_cov" if "', fuse_cov, '" == "true" else "imu"']
                        ),
                    },
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
            # Lidar mount transform. Gazebo names the sensor frame
            # <model>/<link>/<sensor>, which nothing else publishes.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                namespace=ns,
                arguments=[
                    "--x",
                    lidar_x,
                    "--z",
                    lidar_z,
                    "--frame-id",
                    [ns, "/base_link"],
                    "--child-frame-id",
                    [ns, "/base_link/lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            # The IMU stamps its messages `<ns>/base_link/imu`. Without this frame
            # robot_localization cannot rotate the gyro into base_link and drops
            # every sample — silently, and since the gyro is the filter's only yaw
            # source the estimate then has no heading information at all. Measured
            # cost of omitting it: 8-17 m and up to 175 deg of drift in 90 s, far
            # worse than the unfused wheel odometry it was meant to improve.
            # The sensor has no <pose> in robot.sdf.jinja, so this is identity.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_tf",
                namespace=ns,
                arguments=[
                    "--frame-id",
                    [ns, "/base_link"],
                    "--child-frame-id",
                    [ns, "/base_link/imu"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="proximity_lidar_tf",
                namespace=ns,
                arguments=[
                    "--x",
                    "0.24",
                    "--z",
                    "0.05",
                    "--frame-id",
                    [ns, "/base_link"],
                    "--child-frame-id",
                    [ns, "/base_link/proximity_lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            # The bumper scan, where the fleet has no dedicated bumper lidar.
            # `flatten` rather than `slice`: what a costmap wants is the nearest
            # obstacle per bearing at any height a robot can hit, and the low
            # obstacles this exists to catch (a duck at 0.33 m, a neighbour's
            # chassis at 0.28 m) are exactly what a horizontal ring misses.
            # scoped=True, and forwarding left at its default True.
            # `scoped` pops the configuration scope when the group ends, so
            # what the include sets does not escape; `forwarding=True` still
            # lets the parent's configurations IN, which the include's own
            # condition needs (forwarding=False fails with "launch
            # configuration 'proximity_from_cloud' does not exist").
            #
            # This is load-bearing, not tidiness. `launch_arguments` on an
            # IncludeLaunchDescription are implemented as SetLaunchConfiguration
            # in the CURRENT scope, so without a scoped group they leak into
            # every later include: this one's mode=flatten and
            # node_name=cloud_to_proximity_scan overrode the slice include
            # below, which then started a SECOND flatten node under the same
            # name and published no <ns>/scan at all. 2D SLAM and both costmaps
            # sit waiting on a topic with no publisher, and the only symptom is
            # a fleet that never moves.
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("swarmdeck_slam"),
                                "/launch/cloud_to_scan.launch.py",
                            ]
                        ),
                        condition=IfCondition(proximity),
                        launch_arguments={
                            "namespace": ns,
                            "use_sim_time": use_sim,
                            "mode": "flatten",
                            "range_max": prox_range,
                            "output_topic": "proximity_scan",
                            "node_name": "cloud_to_proximity_scan",
                            # The band is referred back to the floor, because
                            # pointcloud_to_laserscan filters in `target_frame` and
                            # that frame is base_link, which floats. Left
                            # base_link-relative, Spot's band would start 0.62 m above
                            # the floor (its base_link is at 0.50) and miss a Scout
                            # Mini, which is 0.245 m tall, entirely. That is precisely
                            # the failure PROXIMITY_SCAN_HEIGHT existed to prevent on
                            # the Gazebo fleet: a tall robot has to be able to see a
                            # short one. cloud_to_scan.launch.py applies the band to
                            # `floor_z` itself.
                            "floor_z": floor_z,
                        }.items(),
                    ),
                ],
            ),
            # A multi-ring lidar's LaserScan is not a planar slice, so derive one.
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("swarmdeck_slam"),
                                "/launch/cloud_to_scan.launch.py",
                            ]
                        ),
                        condition=multi_ring,
                        launch_arguments={
                            "namespace": ns,
                            "use_sim_time": use_sim,
                            # 2D SLAM needs the horizontal ring itself, not a projection.
                            "mode": "slice",
                            "output_topic": "scan",
                            "node_name": "cloud_to_scan",
                            "range_max": range_max,
                            "floor_z": floor_z,
                        }.items(),
                    ),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                namespace=ns,
                # Remap, do NOT use the scan_topic parameter — see docstring.
                remappings=[
                    ("/scan", "scan"),
                    ("/map", "map"),
                    ("/map_metadata", "map_metadata"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                parameters=[
                    slam_params,
                    {
                        "use_sim_time": use_sim,
                        "odom_frame": [ns, "/odom"],
                        "base_frame": [ns, "/base_link"],
                        "map_frame": [ns, "/map_frame"],
                        # Follows the sensor. Left at the file's old 16.0 while
                        # the lidar profile reaches 30 m, SLAM Toolbox would
                        # discard the far half of every scan and raytrace free
                        # space only as far as it was told to look.
                        "max_laser_range": range_max,
                    },
                ],
                output="screen",
            ),
            # Drives slam_toolbox unconfigured -> configure -> activate.
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                namespace=ns,
                parameters=[
                    {
                        "use_sim_time": use_sim,
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                        "bond_timeout": 0.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
