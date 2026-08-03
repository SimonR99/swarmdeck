"""Slice a planar `<ns>/scan` out of a multi-ring lidar's `<ns>/scan/points`.

Included by both SLAM backends, for different reasons:

* `slam.launch.py` needs it because SLAM Toolbox consumes only a planar scan.
* `slam_rtabmap.launch.py` needs it because Nav2's costmap observation sources
  are LaserScan topics, so `<ns>/scan` must exist even when SLAM itself is
  working from the full cloud.

The height band's job here is to SELECT THE HORIZONTAL RING, not to squash 3D
structure into 2D. Keep it tight. A ring at elevation e leaves a band of
half-height h at range h/sin(e), so widening the band buys no range — it only
admits the downward rings' floor returns near the robot (from a 0.40 m mount they
strike the floor around 1.6 m out) and writes the floor into the map as a wall.
This is also why the ring count must be odd: an even count puts no ring at
elevation 0, so no ring survives the band at range. See robot.sdf.jinja.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="cloud_to_scan",
                namespace=ns,
                remappings=[("cloud_in", "scan/points"), ("scan", "scan")],
                parameters=[
                    {
                        "use_sim_time": use_sim,
                        # Empty keeps the cloud in the lidar frame, where the
                        # horizontal ring is exactly z=0 and no TF lookup can fail
                        # or lag. A gravity-aligned frame would not help: a tilted
                        # ring rises with range in every frame, so the truncation
                        # is geometric, not a frame choice.
                        "target_frame": "",
                        "min_height": -0.05,
                        "max_height": 0.05,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0174533,  # 360 bins, matching the lidar
                        "scan_time": 0.1,
                        "range_min": 0.15,
                        "range_max": 16.0,
                        "use_inf": True,
                        "transform_tolerance": 0.05,
                    }
                ],
                output="screen",
            ),
        ]
    )
