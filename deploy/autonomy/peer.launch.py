"""One onboard peer. No dashboard service participates in discovery or solving.

Required environment: SWARMDECK_MISSION_ID (fresh UUID), ROS_DOMAIN_ID (shared
peer domain), SWARMDECK_PEER_NAMES (JSON array), SWARMDECK_PEER_INDEX.
SWARMDECK_SENSOR_DOMAIN_ID may select a separate robot-local sensor domain.
Robot-specific sensor topics and frames are explicit overrides for hardware.
"""

import os
import json
from pathlib import Path
from uuid import UUID

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    mission = str(UUID(os.environ["SWARMDECK_MISSION_ID"]))
    names = json.loads(os.environ["SWARMDECK_PEER_NAMES"])
    index = int(os.environ["SWARMDECK_PEER_INDEX"])
    robot = names[index]
    peer_domain = int(os.environ["ROS_DOMAIN_ID"])
    sensor_domain = int(os.environ.get("SWARMDECK_SENSOR_DOMAIN_ID", peer_domain))
    if not 1 <= peer_domain <= 232:
        raise ValueError("Use a dedicated nonzero ROS_DOMAIN_ID for each mission")
    if not 0 <= sensor_domain <= 232:
        raise ValueError("SWARMDECK_SENSOR_DOMAIN_ID must be between 0 and 232")
    ns = os.environ.get("SWARMDECK_SENSOR_NAMESPACE", robot).strip("/")
    sim_time = os.environ.get("SWARMDECK_USE_SIM_TIME", "false").lower() == "true"
    config = "/cslam_ws/install/swarmdeck_cslam/share/swarmdeck_cslam/config/cslam_lidar.yaml"
    common = [
        config,
        {
            "robot_id": index,
            "max_nb_robots": len(names),
            "use_sim_time": sim_time,
            "swarmdeck.mission_id": mission,
            "frontend.odom_topic": f"/r{index}/normalized_odom",
            "frontend.pointcloud_topic": f"/r{index}/normalized_cloud",
            "frontend.keyframe_min_subscribers": 2,
            "backend.max_waiting_time_sec": 30,
            # Correction TF is owned by the downstream solution adapter. The
            # experimental upstream reference-frame TF is never a second authority.
            "backend.enable_broadcast_tf_frames": False,
            "visualization.enable": False,
        },
    ]
    bridge = Node(
        executable="/usr/bin/python3",
        name="onboard_mapper",
        namespace=f"r{index}",
        arguments=[str(Path(__file__).with_name("cslam_bridge.py"))],
        parameters=[
            {
                "robot_id": robot,
                "robot_index": index,
                "robot_names": json.dumps(names),
                "mission_id": mission,
                "sensor_namespace": ns,
                "sensor_domain_id": sensor_domain,
                "use_sim_time": sim_time,
                "navigation_frame": os.environ.get(
                    "SWARMDECK_NAVIGATION_FRAME", f"{ns}/map_frame"
                ),
                "base_frame": os.environ.get("SWARMDECK_BASE_FRAME", f"{ns}/base_link"),
                "odom_frame": os.environ.get("SWARMDECK_ODOM_FRAME", f"{ns}/odom"),
                "cloud_topic": os.environ.get(
                    "SWARMDECK_CLOUD_TOPIC", f"/{ns}/scan/points"
                ),
                "tf_topic": os.environ.get("SWARMDECK_TF_TOPIC", f"/{ns}/tf"),
                "tf_static_topic": os.environ.get(
                    "SWARMDECK_TF_STATIC_TOPIC", f"/{ns}/tf_static"
                ),
                "server_url": os.environ.get("SWARMDECK_SERVER_URL", ""),
                "store_root": os.environ.get("SWARMDECK_MAP_STORE", "/maps"),
            }
        ],
        remappings=[
            ("/tf", os.environ.get("SWARMDECK_TF_TOPIC", f"/{ns}/tf")),
            (
                "/tf_static",
                os.environ.get("SWARMDECK_TF_STATIC_TOPIC", f"/{ns}/tf_static"),
            ),
        ],
        output="screen",
    )
    frontends = [
        Node(
            package="cslam",
            executable=executable,
            namespace=f"r{index}",
            parameters=common,
            output="screen",
        )
        for executable in (
            "lidar_handler_node.py",
            "loop_closure_detection_node.py",
            "pose_graph_manager",
        )
    ]
    nodes = [bridge, *frontends]
    # Exit the entire peer if any authority dies. Auto-respawning only a
    # frontend would reuse keyframe IDs in the live distributed graph.
    exits = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="peer authority exited; new mission required"
                        )
                    )
                ],
            )
        )
        for node in nodes
    ]
    return LaunchDescription(
        [*exits, bridge, TimerAction(period=3.0, actions=frontends)]
    )
