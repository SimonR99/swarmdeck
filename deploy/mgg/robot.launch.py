"""One MGG planner/PCI connected to an existing SwarmDeck robot's ROS topics."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_nodes(
    robot,
    frame,
    odom,
    cloud,
    tf,
    tf_static,
    params,
    sim_time,
    robot_index=1,
    size=None,
    planner_overrides=None,
    sim_depth=False,
    camera_offset=(0.0, 0.0),
):
    ns = f"{robot}/mgg"
    common = {"use_sim_time": sim_time}
    tf_remaps = [("/tf", tf), ("/tf_static", tf_static)]
    overrides = {
        "PlanningParams.global_frame_id": frame,
        "PlanningParams.robot_id": robot_index,
    }
    overrides.update(planner_overrides or {})
    if size:
        overrides["RobotParams.size"] = size
    return [
        ExecuteProcess(
            cmd=[
                "python3",
                str(Path(__file__).with_name("inputs.py")),
                "--ros-args",
                "-r",
                f"__ns:=/{ns}",
                "-p",
                f"map_frame:={frame}",
                "-p",
                f"use_sim_time:={str(sim_time).lower()}",
                "-p",
                f"sim_depth:={str(sim_depth).lower()}",
                "-p",
                f"camera_x:={camera_offset[0]}",
                "-p",
                f"camera_z:={camera_offset[1]}",
                "-p",
                f"base_frame:={robot}/base_link",
                "-r",
                f"depth:=/{robot}/camera/depth_image",
                "-r",
                f"camera_info:=/{robot}/camera/camera_info",
                "-r",
                f"input_odometry:={odom}",
                "-r",
                f"input_cloud:={cloud}",
                "-r",
                f"/tf:={tf}",
                "-r",
                f"/tf_static:={tf_static}",
            ],
            output="screen",
        ),
        Node(
            package="mgg_ros",
            executable="mggplanner_node",
            namespace=ns,
            parameters=[params, common, overrides],
            remappings=tf_remaps
            + [("odometry", "map_odometry"), ("pointcloud", "mapping_cloud")],
        ),
        Node(
            package="mgg_pci",
            executable="mgg_pci_node",
            namespace=ns,
            parameters=[common, {"world_frame": frame, "bootstrap_distance": 0.0}],
            remappings=[("odometry", "map_odometry")],
        ),
    ]


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    return robot_nodes(
        get("robot"),
        get("map_frame"),
        get("odom"),
        get("cloud"),
        get("tf"),
        get("tf_static"),
        get("params"),
        get("use_sim_time") == "true",
    )


def generate_launch_description():
    defaults = {
        "robot": "robot_0",
        "map_frame": "map",
        "odom": "/odom",
        "cloud": "/points",
        "tf": "/tf",
        "tf_static": "/tf_static",
        "use_sim_time": "false",
    }
    return LaunchDescription(
        [DeclareLaunchArgument("params")]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
