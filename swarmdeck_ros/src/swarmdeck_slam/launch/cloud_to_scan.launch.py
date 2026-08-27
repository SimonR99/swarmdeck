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

`output_topic` exists because the ARGoS backend needs `flatten` twice. Gazebo
carried a second, dedicated bumper lidar at a fixed 0.15 m for
`<ns>/proximity_scan`; ARGoS robots have one lidar each, so the bumper scan is
a second flattened projection of the same cloud with a short range instead. The
costmap parameters in `swarmdeck_nav/config/nav2_params.yaml` are unchanged and
still name both sources.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The flattening node targets `base_link`, not the lidar frame. `floor_z` is
# therefore the ground height in that target frame and is supplied per robot
# by session.launch.py (`-base_height` in simulation). Keep this physical band
# identical to the adapter/keyframe map path: 15 cm above ground through 1.8 m.
FLATTEN_MIN_HEIGHT = 0.15
FLATTEN_MAX_HEIGHT = 1.80


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    range_max = LaunchConfiguration("range_max")
    output_topic = LaunchConfiguration("output_topic")
    node_name = LaunchConfiguration("node_name")
    floor_z = LaunchConfiguration("floor_z")
    flatten = IfCondition(PythonExpression(['"', mode, '" == "flatten"']))
    slice_mode = UnlessCondition(PythonExpression(['"', mode, '" == "flatten"']))

    common = {
        "use_sim_time": use_sim,
        "angle_min": -3.14159,
        "angle_max": 3.14159,
        "angle_increment": 0.0174533,  # 360 bins; a costmap needs no more
        "scan_time": 0.1,
        # Outside the robot's own footprint (0.35 m radius). A 3D lidar mounted
        # on the deck sees the robot it is standing on, and 0.15 m let those
        # self-returns straight through: every flattened scan reported an
        # obstacle 0.15 m ahead, so `explore.py` sat permanently in its blocked
        # escape — reversing and spinning in place for the whole run while the
        # bumper scan showed 2.6-3.3 m of clear floor. The fleet crawled at
        # ~1.4 cm/s and inter-robot encounters never happened, which read as a
        # collaborative-SLAM problem and was not one.
        "range_min": 0.45,
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
            DeclareLaunchArgument(
                "output_topic",
                default_value="scan",
                description="Where the derived LaserScan is published, "
                "relative to the namespace. The ARGoS backend "
                "runs a second flatten instance on "
                "proximity_scan.",
            ),
            DeclareLaunchArgument("node_name", default_value="cloud_to_scan"),
            DeclareLaunchArgument(
                "floor_z",
                default_value="0.0",
                description="Ground height in target_frame. For a simulated "
                "base_link this is -base_height.",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name=node_name,
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
                    ("scan", output_topic),
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
                name=node_name,
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
                    ("scan", output_topic),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                parameters=[
                    dict(
                        common,
                        # Gravity-aligned, so the band means height above the
                        # floor rather than height above a tilting sensor. The
                        # target frame's ground offset makes this physical.
                        target_frame=[ns, "/base_link"],
                        min_height=ParameterValue(
                            PythonExpression(
                                [floor_z, " + ", str(FLATTEN_MIN_HEIGHT)]
                            ),
                            value_type=float,
                        ),
                        max_height=ParameterValue(
                            PythonExpression(
                                [floor_z, " + ", str(FLATTEN_MAX_HEIGHT)]
                            ),
                            value_type=float,
                        ),
                        # A real TF lookup now happens per cloud, so this cannot
                        # be as tight as the in-frame slice above.
                        transform_tolerance=0.1,
                    )
                ],
                output="screen",
            ),
        ]
    )
