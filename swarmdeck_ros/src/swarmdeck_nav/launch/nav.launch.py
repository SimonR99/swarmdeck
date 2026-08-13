"""Launch one namespaced Nav2 stack against that robot's live SLAM map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_nav"), "config", "nav2_params.yaml"]
    )
    source_params = LaunchConfiguration("params_file")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")
    controller_cmd_vel_topic = LaunchConfiguration("controller_cmd_vel_topic")
    output_cmd_vel_topic = LaunchConfiguration("output_cmd_vel_topic")

    # TF frame IDs are strings and are not affected by a ROS namespace. Replace
    # the marker before putting the otherwise standard Nav2 YAML under the
    # robot's namespace.
    namespaced_params = ReplaceString(
        source_file=source_params,
        replacements={"<robot_namespace>": namespace},
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=namespaced_params,
            root_key=namespace,
            # The fleet is no longer uniform, so footprint cannot live in the
            # shared YAML: a Scout Mini is 0.42 m circumscribed and a Bunker
            # 0.64 m. RewrittenYaml rewrites a bare key wherever it appears,
            # which reaches both the global and local costmap copies.
            #
            # Inflation is passed alongside rather than derived here, because a
            # launch substitution cannot do arithmetic — session.launch.py owns
            # the relationship between the two.
            param_rewrites={
                # `footprint` wins over `robot_radius` wherever it parses to a
                # valid polygon, and that is the point: a circle has to
                # circumscribe, which makes a 0.778 m wide Bunker into a 1.285 m
                # disc and every cell within 0.643 m of a wall lethal. The
                # rectangle gives Nav2 the real 0.389 m inscribed radius.
                # robot_radius is still passed as the fallback for anything that
                # cannot parse the polygon.
                "footprint": LaunchConfiguration("footprint"),
                "robot_radius": LaunchConfiguration("robot_radius"),
                "inflation_radius": LaunchConfiguration("inflation_radius"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [configured_params, {"use_sim_time": use_sim_time}],
        "remappings": [("/tf", tf_topic), ("/tf_static", tf_static_topic)],
    }

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        **{
            **common,
            "remappings": common["remappings"]
            + [("cmd_vel", controller_cmd_vel_topic)],
        },
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        **common,
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        **{
            **common,
            "remappings": common["remappings"]
            + [("cmd_vel", controller_cmd_vel_topic)],
        },
    )
    navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        **common,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        **{
            **common,
            "remappings": common["remappings"]
            + [
                ("cmd_vel", controller_cmd_vel_topic),
                ("cmd_vel_smoothed", output_cmd_vel_topic),
            ],
        },
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "velocity_smoother",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            # Circumscribed chassis radius, and the inflation built on it.
            # RewrittenYaml always overwrites the YAML copies of these keys, so
            # a caller that omits them does not "keep the YAML values" — it
            # gets these defaults. They are the smallest platform (Scout Mini)
            # so a forgotten override plans too tightly rather than refusing to
            # plan. On a Bunker that is fatal in a different way: the lidar
            # sees the deck as an obstacle and the only recovery is reverse.
            # Hardware launches MUST pass robot_radius / footprint.
            DeclareLaunchArgument("robot_radius", default_value="0.422"),
            DeclareLaunchArgument("inflation_radius", default_value="0.70"),
            # Chassis rectangle. Empty means "fall back to robot_radius".
            DeclareLaunchArgument("footprint", default_value="[]"),
            # Simulation keeps TF namespaced. Hardware can opt into the
            # machine-wide TF graph without duplicating this Nav2 bring-up.
            DeclareLaunchArgument("tf_topic", default_value="tf"),
            DeclareLaunchArgument("tf_static_topic", default_value="tf_static"),
            # Keep the controller -> smoother -> driver chain configurable so
            # hardware can put the final velocity behind an external safety
            # arbiter. The simulation defaults remain unchanged.
            DeclareLaunchArgument(
                "controller_cmd_vel_topic", default_value="cmd_vel_nav"
            ),
            DeclareLaunchArgument("output_cmd_vel_topic", default_value="cmd_vel"),
            controller,
            planner,
            behaviors,
            navigator,
            velocity_smoother,
            lifecycle,
        ]
    )
