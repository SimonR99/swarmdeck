"""Derive a `<ns>/scan` LaserScan from a multi-ring lidar's `<ns>/scan/points`.

Two modes, because the two SLAM backends want opposite things from this node.

**`mode:=slice`** (SLAM Toolbox). 2D SLAM consumes one planar scan and treats it
as ground truth about a horizontal plane, so the band's job is to SELECT THE
HORIZONTAL RING and nothing else. Keep it tight, and filter in the LIDAR frame
where that ring is exactly z=0. A ring at elevation `e` leaves a band of
half-height `h` at range `h/sin(e)`, so widening the band buys no range — it only
admits the downward rings' floor returns, which get written into the map as
walls. This is also why the ring count must be odd: an even count puts no ring at
elevation 0, so nothing survives the band at range. See robot.sdf.jinja.

**`mode:=flatten`** (RTAB-Map). Here SLAM itself consumes the full cloud, and
`<ns>/scan` exists only as an observation source for Nav2's costmaps. What a
costmap wants is the nearest obstacle per bearing at any height a robot can hit —
a 2.5D projection, not a slice. `pointcloud_to_laserscan` already keeps the
closest return per angular bin, so that is precisely what a tall band gives,
filtered in a GRAVITY-ALIGNED frame so "height" means height above the floor.
It does not truncate with range, because it is not trying to follow one ring.

Those two facts are easy to conflate. The truncation argument above is about
recovering a *planar slice* from tilted rings, and it is correct: no band and no
choice of frame can do that. It says nothing about flattening 3D structure into
an obstacle scan, which is a different operation with a different answer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Nothing below this is an obstacle a robot can hit; it is the floor. Above the
# lidar's own mount height there is nothing to see either, but a doorframe or a
# low ceiling fixture is worth marking, so the top of the band is generous.
FLATTEN_MIN_HEIGHT = 0.12
FLATTEN_MAX_HEIGHT = 1.60


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    range_max = LaunchConfiguration("range_max")
    flatten = IfCondition(PythonExpression(['"', mode, '" == "flatten"']))
    slice_mode = UnlessCondition(PythonExpression(['"', mode, '" == "flatten"']))

    common = {
        "use_sim_time": use_sim,
        "angle_min": -3.14159,
        "angle_max": 3.14159,
        "angle_increment": 0.0174533,   # 360 bins; a costmap needs no more
        "scan_time": 0.1,
        "range_min": 0.15,
        "range_max": range_max,
        "use_inf": True,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "mode",
                default_value="slice",
                choices=["slice", "flatten"],
                description="slice = select the horizontal ring for 2D SLAM; "
                            "flatten = project all obstacle heights down for Nav2",
            ),
            DeclareLaunchArgument("range_max", default_value="30.0"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="cloud_to_scan",
                namespace=ns,
                condition=slice_mode,
                # The /tf remaps are load-bearing in `flatten` mode and inert in
                # `slice` mode, which is precisely why they were easy to omit:
                # with target_frame empty nothing is ever looked up, so a node
                # listening on the global /tf appears to work. Set a target frame
                # and it silently drops every cloud instead — the message filter
                # reports only "queue is full", never "no such transform".
                remappings=[
                    ("cloud_in", "scan/points"),
                    ("scan", "scan"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                parameters=[
                    dict(
                        common,
                        # Empty keeps the cloud in the lidar frame, where the
                        # horizontal ring is exactly z=0 and no TF lookup can
                        # fail or lag.
                        target_frame="",
                        min_height=-0.05,
                        max_height=0.05,
                        transform_tolerance=0.05,
                    )
                ],
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="cloud_to_scan",
                namespace=ns,
                condition=flatten,
                # The /tf remaps are load-bearing in `flatten` mode and inert in
                # `slice` mode, which is precisely why they were easy to omit:
                # with target_frame empty nothing is ever looked up, so a node
                # listening on the global /tf appears to work. Set a target frame
                # and it silently drops every cloud instead — the message filter
                # reports only "queue is full", never "no such transform".
                remappings=[
                    ("cloud_in", "scan/points"),
                    ("scan", "scan"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                parameters=[
                    dict(
                        common,
                        # Gravity-aligned, so the band means height above the
                        # floor rather than height above a tilting sensor.
                        target_frame=[ns, "/base_link"],
                        min_height=FLATTEN_MIN_HEIGHT,
                        max_height=FLATTEN_MAX_HEIGHT,
                        # A real TF lookup now happens per cloud, so this cannot
                        # be as tight as the in-frame slice above.
                        transform_tolerance=0.1,
                    )
                ],
                output="screen",
            ),
        ]
    )
