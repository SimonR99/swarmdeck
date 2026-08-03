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
    source_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_nav"), "config", "nav2_params.yaml"]
    )

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
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [configured_params, {"use_sim_time": use_sim_time}],
        "remappings": [("/tf", "tf"), ("/tf_static", "tf_static")],
    }

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        **{**common, "remappings": common["remappings"] + [("cmd_vel", "cmd_vel_nav")]},
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
        **{**common, "remappings": common["remappings"] + [("cmd_vel", "cmd_vel_nav")]},
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
            + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
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
            controller,
            planner,
            behaviors,
            navigator,
            velocity_smoother,
            lifecycle,
        ]
    )
