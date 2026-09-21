"""Default ROS 1 adapter profile."""

from __future__ import annotations

from typing import Any

from adapters.runtime import TRANSPORT_DEFAULTS, deep_merge

DEFAULTS: dict[str, Any] = deep_merge(
    TRANSPORT_DEFAULTS,
    {
        "robot_type": "generic",
        "ros_distro": "noetic",
        "footprint_radius": 0.35,
        "footprint": [],
        "network_iface": "",
        "navigation_frame": "odom",
        "base_frame": "base_link",
        "topics": {
            "odom": "odom",
            "plan": "plan",
            "cmd_vel": "cmd_vel",
            "battery": "",
            "camera": "",
            "camera_compressed": "",
            "camera_depth": "",
            "camera_info": "",
            "camera_color_info": "",
            "camera_depth_points": "",
            "nav_goal": "",
            "nav_stop": "",
            "nav_cmd_vel": "",
            "nav_joy": "",
            "local_costmap": "",
        },
        "nav_joy_throttle": 0.5,
        "nav_joy_reverse_steering": False,
        "retain_free_space": False,
        "nav_goal_tolerance_m": 0.5,
        "actions": {},
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
