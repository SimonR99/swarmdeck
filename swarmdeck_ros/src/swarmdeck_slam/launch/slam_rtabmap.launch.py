"""Per-robot SLAM on a 3D cloud, as a drop-in alternative to SLAM Toolbox.

    ros2 launch swarmdeck_bringup session.launch.py slam_backend:=rtabmap

Why this exists. SLAM Toolbox is 2D: it consumes one planar scan, so on a robot
whose real sensors are odometry + a point cloud + a camera it throws away almost
everything, and the only thing correcting odometry drift is planar scan matching.
It cannot use the cloud's vertical structure, cannot observe z/roll/pitch drift,
and cannot close a loop from the camera.

RTAB-Map fits that sensor set directly: it takes external odometry as its motion
guess, refines it with ICP against the cloud, optionally closes loops visually
from the RGB image, optimises a pose graph, and still publishes an ordinary 2D
`nav_msgs/OccupancyGrid`. That last part matters — `grid_map` is remapped to
`<ns>/map`, so the adapter, the backend map service, Nav2's static layer, and the
GUI are all unchanged. The swap is contained to this file.

What it does NOT do: share anything between robots. Each robot still has a
private pose graph and its own map frame, and the backend still stitches finished
grids afterwards. No robot's drift is corrected by another robot's observations.
That needs inter-robot loop closures — see docs/collaborative-slam.md.

Requires `lidar_rings` > 1 (an odd value), because a single-ring lidar produces a
cloud with no vertical structure for ICP to use. Requires ros-jazzy-rtabmap-ros.

Unverified here: `use_camera:=true` needs Gazebo's camera_info topic, whose exact
name under a namespaced sensor should be confirmed with `gz topic -l` on the
first run. The lidar path has no such dependency, so it is the default.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    use_camera = LaunchConfiguration("use_camera")
    visual = PythonExpression(['"true" if "', use_camera, '" == "true" else "false"'])

    parameters = {
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
        "publish_tf": True,
        "database_path": "",  # in-memory: a session is a fresh map, like SLAM Toolbox
        # Registration. ICP against the cloud is what corrects the drift that
        # wheel odometry accumulates; Force3DoF holds it to the plane the robot
        # actually drives on, which keeps the pose graph consistent with a 2D grid.
        "Reg/Strategy": PythonExpression(['"2" if "', use_camera, '" == "true" else "1"']),
        "Reg/Force3DoF": "true",
        "Icp/PointToPlane": "true",
        "Icp/Iterations": "10",
        "Icp/VoxelSize": "0.05",
        "Icp/MaxCorrespondenceDistance": "0.5",
        "Icp/CorrespondenceRatio": "0.3",
        "Icp/Epsilon": "0.001",
        # Loop closure. ProximityBySpace is the lidar equivalent of revisiting a
        # place; the visual detector below adds appearance-based closure, which is
        # the one thing a camera gives that a lidar cannot.
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
        "Grid/3D": "false",
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "16.0",
        "Grid/RayTracing": "true",
        "Grid/MaxGroundHeight": "0.08",
        "Grid/MaxObstacleHeight": "1.80",
        "Grid/NormalsSegmentation": "false",  # plain height split; the robot is planar
        "GridGlobal/MinSize": "30.0",
        "GridGlobal/Eroded": "false",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="add appearance-based loop closure from the RGB camera",
            ),
            # RTAB-Map works from the full cloud, but Nav2's costmap observation
            # sources are LaserScan topics, so <ns>/scan still has to exist.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_slam"), "/launch/cloud_to_scan.launch.py"]
                ),
                launch_arguments={"namespace": ns, "use_sim_time": use_sim}.items(),
            ),
            # Same lidar mount transform SLAM Toolbox needs; Gazebo names the
            # sensor frame <model>/<link>/<sensor> and nothing else publishes it.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                namespace=ns,
                arguments=[
                    "--x", "-0.07", "--z", "0.402",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_tf",
                namespace=ns,
                condition=IfCondition(visual),
                arguments=[
                    "--x", "0.162", "--z", "0.292",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/camera"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                namespace=ns,
                parameters=[parameters],
                remappings=[
                    ("scan_cloud", "scan/points"),
                    ("odom", "odom"),
                    ("rgb/image", "camera/image_raw"),
                    ("rgb/camera_info", "camera/camera_info"),
                    # The whole point: downstream keeps consuming <ns>/map.
                    ("grid_map", "map"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                # A stale database would silently relocalise into an old session.
                arguments=["--delete_db_on_start"],
                output="screen",
            ),
        ]
    )
