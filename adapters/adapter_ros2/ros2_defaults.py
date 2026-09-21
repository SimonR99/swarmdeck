"""Default ROS 2 adapter profile."""

from __future__ import annotations

from typing import Any

from adapters.runtime import TRANSPORT_DEFAULTS, deep_merge

DEFAULTS: dict[str, Any] = deep_merge(
    TRANSPORT_DEFAULTS,
    {
        "robot_type": "generic",
        "ros_distro": "jazzy",
        "footprint_radius": 0.35,
        "footprint": [],
        "network_iface": "",
        "navigation_frame": "odom",
        "base_frame": "base_link",
        "topics": {
            "odom": "odom",
            "plan": "plan",
            "local_plan": "",
            "local_costmap": "",
            "cmd_vel": "cmd_vel",
            "battery": "",
            "camera": "",
            "camera_compressed": "",
            "camera_depth": "",
            "camera_info": "",
            "camera_color_info": "",
            "camera_depth_points": "",
            "nav_cmd_vel": "",
        },
        "retain_free_space": False,
        "actions": {"trajectory": ""},
        "trajectory": {
            "frame": "body",
            "duration_s": 30.0,
            "precise_positioning": True,
            "disable_obstacle_avoidance": False,
            "control_mode": "",
            "progress_frame": "",
            "velocity_limit": {},
        },
        "services": {
            "claim": "",
            "release": "",
            "sit": "",
            "stand": "",
            "power_on": "",
            "stop": "",
            "estop_release": "",
            "clear_keepalive": "",
            "max_velocity": "",
        },
        "perception": {
            "enabled": True,
            "period_s": 0.2,
            "sensitivity": 0.55,
            "classes": [],
            "detector_url": "",
            "depth_min_m": 0.15,
            "depth_max_m": 8.0,
            "depth_max_age_s": 0.35,
        },
    },
)
