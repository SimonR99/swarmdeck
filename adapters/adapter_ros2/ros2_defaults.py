"""Default YAML overlay for this adapter. Transport lives in runtime."""

from __future__ import annotations

from typing import Any

from adapters.runtime import TRANSPORT_DEFAULTS, deep_merge

DEFAULTS: dict[str, Any] = deep_merge(TRANSPORT_DEFAULTS, {
    "robot_type": "generic",
    "ros_distro": "jazzy",
    "footprint_radius": 0.35,
    # Optional polygon in base_frame coordinates, x forward / y left. The
    # radius remains the conservative fallback for map filtering and older
    # protocol peers.
    "footprint": [],
    # Linux wireless interface to sample into the per-robot network heatmap.
    # "auto" uses the first row in /proc/net/wireless; empty disables it.
    "network_iface": "",
    # Frames. `map` and `base_link` are REP-105 names and the only two that must
    # exist; the pose is a tf2 lookup between them rather than a composition of
    # transforms we recognise, because a real TF tree has links we do not know
    # about (base_footprint, odom_combined, per-vendor intermediates).
    "map_frame": "map",
    "base_frame": "base_link",
    "topics": {
        "odom": "odom",
        "map": "map",
        # OccupancyGrid the collaborative back-end warps into this robot's
        # map frame. Nav2's global static layer subscribes here. Local
        # costmaps must not.
        "nav_map": "/global_map",
        # Registered PointCloud2 for a 3D SLAM stack that does not publish an
        # OccupancyGrid. The backend raytraces a height-filtered XY view into
        # a grid and keeps a coarser XYZ view for the optional 3D panel.
        "map_cloud": "",
        "plan": "plan",
        "cmd_vel": "cmd_vel",
        "battery": "",  # empty disables the capability
        "camera": "",
        "camera_compressed": "",
        "camera_depth": "",
        "camera_info": "",
        # Colour CameraInfo. Set this when depth is *not* RGB-aligned so a
        # detection box in the operator image can be joined to a slower,
        # independently-published depth stream. Leave empty when `camera_info`
        # already describes aligned depth (the usual RGB-D case).
        "camera_color_info": "",
        # Organised PointCloud2 aligned pixel-for-pixel with the RGB image.
        # Optional: bbox-only detection still works when it is absent.
        "camera_depth_points": "",
        # Isolated output from a navigation stack. If set, the adapter relays
        # it to the real driver only while an action goal is active, keeping
        # teleop, cancellation and e-stop authoritative.
        "nav_cmd_vel": "",
        # Nav2's DWB controller trajectory. This is preferred over the global
        # planner path for the dashboard because it is the route selected for
        # the next control cycle. Empty disables the optional local route.
        "local_plan": "",
        # Read-only Nav2 planner products for the dashboard. These are never
        # fed back into the collaborative occupancy map or navigation stack.
        "global_costmap": "",
        "local_costmap": "",
    },
    # min_z/max_z are map-frame limits by default. A profile may add floor_z;
    # then they mean physical heights above the floor and the adapter adds that
    # map-frame floor reference before filtering.
    "map_cloud_height_band": {"min_z": -0.3, "max_z": 0.5},
    # Keep ray-traced known-free cells white after the lidar moves on. Unknown
    # cells remain unknown; this only controls retention of observed free space.
    "retain_free_space": False,
    "actions": {
        "navigate_to_pose": "navigate_to_pose",
        # Spot: Clearpath `spot_msgs/Trajectory`. Empty on every other robot.
        "trajectory": "",
    },
    # Spot Trajectory goals are in `body`. duration_s must be > 0 or the
    # driver aborts. The high-level controller uses 30 s.
    "trajectory": {
        "frame": "body",
        "duration_s": 30.0,
        "precise_positioning": True,
        "disable_obstacle_avoidance": False,
        # Optional Spot SDK mobility limit, applied through /max_velocity
        # immediately before each trajectory. `duration_s` is only a timeout;
        # it does not control how quickly Spot walks to the target. linear_y
        # may be zero to prohibit lateral walking.
        "velocity_limit": {},
    },
    # Empty disables the `body` capability. Spot's Clearpath driver exposes
    # these as std_srvs/Trigger; a robot without them leaves them blank.
    "services": {
        "claim": "",
        "release": "",
        "sit": "",
        "stand": "",
        "power_on": "",
        "stop": "",
        "estop_release": "",
        "clear_keepalive": "",
        # Spot SDK mobility limit service (spot_msgs/SetVelocity).
        "max_velocity": "",
    },
    "perception": {
        "enabled": True,
        "period_s": 0.2,
        "sensitivity": 0.55,
        # Catalog classes this robot looks for (adapters/perception/catalog.py).
        # Empty means all of them; the dashboard's own selection overrides this
        # on the next settings refresh either way.
        "classes": [],
        # Empty uses SWARMDECK_DETECTOR_URL, then localhost:8091.
        "detector_url": "",
        "depth_min_m": 0.15,
        "depth_max_m": 8.0,
        "depth_max_age_s": 0.35,
    },
    # Ping, rates, and upload timeouts: TRANSPORT_DEFAULTS.
})
