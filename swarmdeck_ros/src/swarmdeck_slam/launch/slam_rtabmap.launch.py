"""Per-robot 3D SLAM on a lidar + IMU, as a drop-in alternative to SLAM Toolbox.

    ros2 launch swarmdeck_bringup session.launch.py slam_backend:=rtabmap \\
         config:=study/4robot_3d.yaml

Two nodes, and the split matters:

  icp_odometry   owns `odom -> base_link`. Registers each cloud against a local
                 map of recent ones, de-skewed with the IMU. This REPLACES wheel
                 odometry as the motion source — it does not merely filter it.
  rtabmap        owns `map_frame -> odom`. Pose graph, loop closure, and the 2D
                 `OccupancyGrid` republished on `<ns>/map`.

Why replace the wheel+gyro EKF rather than fuse with it. The EKF's measured
weakness is not noise, it is that wheel odometry cannot observe slip at all: a
jammed differential drive keeps turning its wheels and the plugin integrates
motion that never happened, which no filter downstream can undo (8.8-30.5 m of
error, docs/KNOWN_ISSUES.md). Lidar odometry observes the world directly and is
simply immune to that. Running both would also put two publishers on
`odom -> base_link`, and a TF tree that flickers between two estimates is worse
than either — which is exactly why session.launch.py already drops the drive
plugin's TF bridge when the EKF is active.

Requires a multi-ring lidar (`fleet.lidar.profile: generic_32`): a single ring
gives a cloud with no vertical structure, and point-to-plane ICP has no normals
to work with. session.launch.py refuses the combination rather than start a stack
that comes up healthy and drifts.

Downstream is untouched. `grid_map` is remapped to `<ns>/map`, so the adapter,
the backend map service, Nav2's static layer and the GUI cannot tell which
backend produced it. That is the property that makes this swap cheap.

What it still does NOT do: share anything between robots. Each robot keeps a
private pose graph and its own map frame. See docs/collaborative-slam.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Downsampling applied before registration. 0.1 m keeps enough structure for
# point-to-plane normals in a 24 m building while holding the per-cloud cost low
# enough to run four robots in real time.
ICP_VOXEL = "0.1"


def rtab(value):
    """A RTAB-Map tuning parameter carrying a launch substitution.

    Every `Foo/Bar` parameter RTAB-Map exposes is a **string** that it parses
    itself, but `launch_ros` infers a ROS parameter type from the substituted
    text: "true" becomes a bool, "30.0" a double, "1" an int. The node then
    aborts on startup with `InvalidParameterTypeException` — SIGABRT, no map, and
    a message that reads as though the value were wrong rather than its type.
    Literal strings in the dicts below are unaffected; only substitutions need
    this.
    """
    return ParameterValue(value, value_type=str)


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_z = LaunchConfiguration("lidar_z")
    deskew = LaunchConfiguration("deskew")
    use_camera = LaunchConfiguration("use_camera")
    force_3dof = LaunchConfiguration("force_3dof")
    range_max = LaunchConfiguration("range_max")
    grid_3d = LaunchConfiguration("grid_3d")
    visual = IfCondition(PythonExpression(['"', use_camera, '" == "true"']))

    # Shared registration settings. Keeping odometry and the pose graph on the
    # same ICP parameters means a loop closure is verified by the same measure
    # that produced the drift, which makes disagreements informative.
    icp = {
        "Icp/PointToPlane": "true",
        "Icp/VoxelSize": ICP_VOXEL,
        "Icp/Epsilon": "0.001",
        "Icp/MaxTranslation": "2.0",
        "Icp/PointToPlaneK": "20",
        "Icp/PointToPlaneRadius": "0.0",
        "Icp/MaxCorrespondenceDistance": "0.5",
    }

    odom_params = {
        "use_sim_time": use_sim,
        "frame_id": [ns, "/base_link"],
        "odom_frame_id": [ns, "/odom"],
        "publish_tf": True,
        "subscribe_scan_cloud": True,
        "approx_sync": True,
        "wait_for_transform": 0.3,
        # De-skewing is OFF here, and only because the simulator cannot support
        # it: Gazebo's cloud carries `x y z intensity ring` and no per-point
        # timestamp, so there is nothing to say when within the sweep a return
        # was taken. Enabled anyway, icp_odometry processed exactly two clouds
        # and then stopped publishing `odom -> base_link` at all, which presents
        # downstream as rtabmap complaining that TF is stuck in the past.
        #
        # On real hardware this should be TRUE. Every driver worth using stamps
        # points (Ouster `t`, Velodyne `time`, Livox `offset_time`), and the
        # smear it removes is real: a 10 Hz scan taken while turning at 0.8 rad/s
        # sweeps 4.6 deg during one revolution, which at 10 m is 0.8 m of
        # distortion registered as though it were structure.
        "deskewing": PythonExpression(["'", deskew, "' == 'true'"]),
        # Hold initialisation until the IMU has given a gravity direction, so
        # roll and pitch start observed rather than assumed.
        "wait_imu_to_init": True,
        # Frame-to-local-map. Frame-to-frame drifts faster on sparse indoor
        # clouds because each registration sees only one previous sweep.
        "Odom/Strategy": "0",
        "Odom/ScanKeyFrameThr": "0.6",
        "OdomF2M/ScanSubtractRadius": ICP_VOXEL,
        "OdomF2M/ScanMaxSize": "15000",
        # Odometry must accept a weaker match than loop closure does: refusing
        # here means losing the pose entirely, refusing there means waiting.
        "Icp/CorrespondenceRatio": "0.01",
        "Icp/Iterations": "10",
        "Reg/Force3DoF": rtab(force_3dof),
        **icp,
    }

    slam_params = {
        "use_sim_time": use_sim,
        "frame_id": [ns, "/base_link"],
        "odom_frame_id": [ns, "/odom"],
        "map_frame_id": [ns, "/map_frame"],
        # Gazebo does not hardware-sync the cloud, image and odometry, so exact
        # stamp matching would drop nearly every message.
        "approx_sync": True,
        "wait_for_transform": 0.3,
        "subscribe_depth": False,
        "subscribe_scan_cloud": True,
        "subscribe_rgb": use_camera,
        # `subscribe_odom_info` is deliberately OFF. It adds per-step matching
        # quality to the graph, which is genuinely useful, but it also puts
        # `odom_info` into the approximate-time synchroniser alongside the cloud —
        # and that pair never matched here, so rtabmap sat logging "Did not
        # receive data since 5 seconds" forever while icp_odometry ran perfectly.
        # Odometry itself reaches rtabmap through TF (odom_frame_id is set), not
        # through a topic, so nothing is lost but the diagnostic. Re-enable only
        # with the sync verified.
        "publish_tf": True,
        "database_path": "",  # in-memory: a session is a fresh map, like SLAM Toolbox
        "Reg/Strategy": rtab(
            PythonExpression(['"2" if "', use_camera, '" == "true" else "1"'])
        ),
        "Reg/Force3DoF": rtab(force_3dof),
        "Icp/Iterations": "10",
        "Icp/CorrespondenceRatio": "0.3",
        # Loop closure. ProximityBySpace is the lidar equivalent of revisiting a
        # place; the visual detector below adds appearance-based closure, which
        # is the one thing a camera gives that a lidar cannot.
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "RGBD/OptimizeFromGraphEnd": "false",
        "Kp/DetectorStrategy": "6",  # GFTT/BRIEF: no patented features
        "Vis/MinInliers": "15",
        "Mem/NotLinkedNodesKept": "false",
        "Optimizer/Strategy": "1",  # g2o
        "Optimizer/Robust": "true",
        # 2D occupancy grid from the 3D cloud. This is the part a height band can
        # never do properly: RTAB-Map segments ground from obstacles by height
        # relative to the robot's own estimated plane, so a sloped floor or a
        # pitching robot does not paint the floor into the map as a wall.
        "Grid/Sensor": "0",  # build the grid from the scan cloud, not the camera
        # Whether the occupancy representation is 3D. The 2D `grid_map` that the
        # adapter, Nav2 and the GUI consume is identical either way — this only
        # decides what `cloud_map` contains, and it is not a free choice.
        # Measured over matched 330 s four-robot runs:
        #
        #            cloud_map            real-time factor   known cells
        #   false    flat, z == 0.00      0.54               176k
        #   true     z -1.68 .. 1.80      0.25               121k
        #
        # So `true` is the only way the GUI's 3D view shows actual structure
        # rather than a plane, and it costs half the simulation speed — which in
        # a fixed wall-clock window means noticeably less of the building gets
        # mapped. Off by default: the 3D view is optional, the map is not.
        "Grid/3D": rtab(grid_3d),
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": rtab(range_max),
        "Grid/RayTracing": "true",
        "Grid/MaxGroundHeight": "0.08",
        "Grid/MaxObstacleHeight": "1.80",
        "Grid/NormalsSegmentation": "false",  # plain height split; the robot is planar
        "GridGlobal/MinSize": "30.0",
        # Erode obstacle cells that no longer have obstacle neighbours when the
        # global grid is reassembled. This is what removes the ghost a robot
        # leaves behind after another robot drives past it: RTAB-Map assembles
        # the map from per-keyframe local grids and never edits an old
        # keyframe, so a robot recorded once stays recorded, as a small blob
        # sitting in open floor.
        #
        # Safe for structure and not for clutter, which is the trade being made:
        # a wall is a connected run of occupied cells and survives erosion, while
        # an isolated cell or two does not. A genuinely thin, isolated obstacle
        # -- a chair leg seen from one angle -- can be eroded with the ghosts.
        # Ray tracing above already clears free space along each beam; this
        # handles what ray tracing cannot, namely cells no beam has revisited.
        "GridGlobal/Eroded": "true",
        **icp,
    }

    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            # Lidar mount, metres, relative to base_link. Arguments rather than
            # constants because these are the simulated robot's numbers: on
            # hardware they come from that unit's URDF or a calibration, and a
            # wrong extrinsic tilts every scan in a way SLAM cannot recover
            # from. See docs/hardware-readiness.md.
            # De-skewing. FALSE by default only because Gazebo's cloud carries
            # no per-point timestamps (verified: fields are x y z intensity
            # ring), so there is nothing to interpolate against and enabling it
            # made icp_odometry stop publishing after two clouds.
            #
            # SET TRUE ON HARDWARE. Every driver worth using stamps points, and
            # the distortion this removes is real: a 10 Hz sweep taken while
            # turning at 0.8 rad/s smears 4.6 deg, which at 10 m is 0.8 m of
            # structure that never existed.
            DeclareLaunchArgument("deskew", default_value="false"),
            DeclareLaunchArgument("lidar_x", default_value="-0.07"),
            DeclareLaunchArgument("lidar_z", default_value="0.402"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="add appearance-based loop closure from the RGB camera",
            ),
            DeclareLaunchArgument(
                "force_3dof",
                default_value="true",
                description="constrain the estimate to x/y/yaw. Correct for this "
                            "fleet, which drives on a flat floor; set false for a "
                            "platform that genuinely leaves the plane.",
            ),
            DeclareLaunchArgument(
                "grid_3d",
                default_value="false",
                description="Keep RTAB-Map's occupancy in 3D so cloud_map carries "
                            "real structure for the GUI's 3D view. Halves the "
                            "real-time factor (0.54 -> 0.25 measured); the 2D map "
                            "is unaffected either way.",
            ),
            DeclareLaunchArgument("range_max", default_value="30.0"),
            # Nav2's costmap observation sources are LaserScan topics, so
            # <ns>/scan must exist even though SLAM works from the full cloud.
            # `flatten`, not `slice`: nothing here needs a planar slice any more,
            # and a costmap wants the nearest obstacle at any height.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_slam"), "/launch/cloud_to_scan.launch.py"]
                ),
                launch_arguments={
                    "namespace": ns,
                    "use_sim_time": use_sim,
                    "mode": "flatten",
                    "range_max": range_max,
                }.items(),
            ),
            # Gazebo names the sensor frame <model>/<link>/<sensor>, which
            # nothing else publishes.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                namespace=ns,
                arguments=[
                    "--x", lidar_x, "--z", lidar_z,
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=tf_remap,
            ),
            # icp_odometry rotates the IMU into base_link and drops every sample
            # without this frame — silently, exactly as robot_localization did.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_tf",
                namespace=ns,
                arguments=[
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/imu"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=tf_remap,
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_tf",
                namespace=ns,
                condition=visual,
                arguments=[
                    "--x", "0.162", "--z", "0.292",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/camera"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=tf_remap,
            ),
            # Gazebo ships all-zero covariance on the IMU; the relay restamps it
            # with the noise robot.sdf.jinja injects. ICP odometry weighs the
            # inertial prior by that covariance, so a zero is not a harmless
            # placeholder here. See covariance_relay.py.
            Node(
                package="swarmdeck_slam",
                executable="covariance_relay.py",
                name="covariance_relay",
                namespace=ns,
                parameters=[{"use_sim_time": use_sim}],
                output="screen",
            ),
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                name="icp_odometry",
                namespace=ns,
                parameters=[odom_params],
                remappings=[
                    ("scan_cloud", "scan/points"),
                    ("imu", "imu_cov"),
                    ("odom", "odom_icp"),
                    *tf_remap,
                ],
                output="screen",
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                namespace=ns,
                parameters=[slam_params],
                remappings=[
                    ("scan_cloud", "scan/points"),
                    # The lidar odometry, NOT the wheel topic on <ns>/odom.
                    ("odom", "odom_icp"),
                    ("rgb/image", "camera/image_raw"),
                    ("rgb/camera_info", "camera/camera_info"),
                    # The whole point: downstream keeps consuming <ns>/map.
                    ("grid_map", "map"),
                    *tf_remap,
                ],
                # A stale database would silently relocalise into an old session.
                arguments=["--delete_db_on_start"],
                output="screen",
            ),
        ]
    )
