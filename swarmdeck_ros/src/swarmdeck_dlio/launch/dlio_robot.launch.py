"""Direct LiDAR-Inertial Odometry for one SwarmDeck robot.

    ros2 launch swarmdeck_dlio dlio_robot.launch.py namespace:=robot_0

An alternative source of `odom -> base_link`, replacing RTAB-Map's
`icp_odometry`. Both register clouds; the difference is that DLIO builds a
continuous-time motion model from the IMU and de-skews every point against it,
rather than treating a sweep as instantaneous.

**Read this before drawing conclusions in simulation.** Gazebo's cloud carries
`x y z intensity ring` and no per-point timestamp — verified on a live topic —
so there is nothing for DLIO to de-skew here and its main advantage cannot
appear. On hardware every driver worth using stamps points (Ouster `t`,
Velodyne `time`, Livox `offset_time`) and the advantage is real: a 10 Hz scan
taken while turning at 0.8 rad/s sweeps 4.6 deg during one revolution, which at
10 m is 0.8 m of distortion registered as though it were structure. Simulation
numbers here are a floor, not a prediction of hardware behaviour.

Runs in its own container image (docker/Dockerfile.dlio) because DLIO's ROS 2
support is a community branch (`feature/ros2`), pinned by commit, and does not
belong in the main Gazebo image.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="direct_lidar_inertial_odometry",
                executable="dlio_odom_node",
                name="dlio_odom",
                namespace=ns,
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("swarmdeck_dlio"), "config", "dlio.yaml"]
                    ),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                # RELATIVE names: the node already runs in `ns`, so prefixing
                # the namespace again yields /robot_0/robot_0/dlio/odom — which
                # every node happily publishes to and nothing subscribes to.
                remappings=[
                    ("pointcloud", "scan/points"),
                    # The RAW IMU, not covariance_relay's restamped copy: DLIO
                    # uses the measurement for its motion model, and the relay
                    # only rewrites covariance. Taking the relayed topic would
                    # add a Python hop in front of a 200 Hz stream for nothing.
                    ("imu", "imu"),
                    ("odom", "dlio/odom"),
                    ("pose", "dlio/pose"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
        ]
    )
