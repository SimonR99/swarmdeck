"""Default YAML overlay for this adapter. Transport lives in runtime."""

from __future__ import annotations

from typing import Any

from adapters.runtime import TRANSPORT_DEFAULTS, deep_merge

DEFAULTS: dict[str, Any] = deep_merge(TRANSPORT_DEFAULTS, {
    "robot_type": "generic",
    "ros_distro": "noetic",
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
        # map frame. Nav2 / move_base global planning subscribes here.
        "nav_map": "/global_map",
        # PointCloud2, for a robot whose SLAM stack registers a cloud but never
        # projects one to a 2D OccupancyGrid (LVI-SAM, LOAM-family stacks in
        # general). Mutually additive with `map`, not a fallback for it: the
        # backend raytraces this into a grid itself (mapsvc/scan_grid.py)
        # rather than the adapter needing its own occupancy-grid mapper.
        "map_cloud": "",
        # Accumulated/global PointCloud2 from a 3D SLAM stack. Unlike
        # `map_cloud`, this is already the whole map, so the adapter projects it
        # directly to an occupied-only OccupancyGrid and must not raytrace it as
        # one scan. Unknown cells stay unknown because the cloud has no per-point
        # sensor origins from which safe free-space evidence could be recovered.
        "map_cloud_global": "",
        # Optional Nav2 costmap topics for the read-only dashboard overlay.
        # They are not map inputs and are intentionally empty on ROS 1 robots
        # that do not run Nav2.
        "global_costmap": "",
        "local_costmap": "",
        "plan": "plan",
        "cmd_vel": "cmd_vel",
        "battery": "",  # empty disables the capability
        "camera": "",
        "camera_compressed": "",
        # RGB-aligned depth image and its CameraInfo. Preferred over a point
        # cloud because many depth drivers publish unordered clouds by default.
        "camera_depth": "",
        "camera_info": "",
        # Colour CameraInfo. Set this when depth is *not* RGB-aligned so a
        # detection box in the operator image can be joined to a slower,
        # independently-published depth stream. Leave empty when `camera_info`
        # already describes aligned depth (the usual RGB-D case).
        "camera_color_info": "",
        # Organised PointCloud2 whose pixels are aligned with the RGB image.
        # When present, detections gain a map_position; otherwise bbox-only
        # detection continues unchanged.
        "camera_depth_points": "",
        # `move_base_simple/goal`-style single-goal topic (geometry_msgs/PoseStamped) —
        # the interface stacks like CMU's local_planner use instead of an
        # actionlib server. Takes priority over `actions.navigate_to_pose`
        # when both are configured: a robot only ever has one real navigation
        # stack, and this is the more specific of the two settings.
        "nav_goal": "",
        # std_msgs/Int8 safety-stop local_planner's pathFollower listens on
        # (1 = halt, 0 = resume). Optional even when `nav_goal` is set — a
        # stack without one just can't be preempted as cleanly by teleop or
        # cancel_goal, which then fall back to zeroing cmd_vel once.
        "nav_stop": "",
        # Where a topic-based nav stack's OWN cmd_vel output lands when it has
        # been remapped away from the real `/cmd_vel` (see
        # launch/local_planner_muxed.launch). If set, this adapter relays it
        # to the real `cmd_vel` topic ONLY while nav_status is "active" — the
        # adapter becomes the sole arbiter of the real topic, so a nav stack
        # that publishes continuously even when idle (confirmed true of
        # local_planner's pathFollower) never gets to race teleop for it.
        # Leave empty for a nav stack that already respects nav_stop/cancel
        # cleanly on its own.
        "nav_cmd_vel": "",
        # sensor_msgs/Joy, for a nav stack whose speed AND path direction both
        # come from a joystick with no software override (confirmed true of
        # local_planner: pathFollower.joystickHandler sets joySpeed = |axes[1]|
        # and localPlanner.joystickHandler sets joyDir = atan2(axes[2], axes[1])
        # — neither has an autonomyMode gate, unlike every other speed/direction
        # source either node has). If set, this adapter fakes both axes every
        # tick from the real bearing to the goal (see _pump_nav_joy) while
        # nav_status is "active", zero otherwise. Real safety still comes from
        # nav_cmd_vel only being relayed while nav_status is "active"
        # (_on_nav_cmd_vel) — this does not need to be itself trusted as a
        # safety mechanism, only a plausible one.
        "nav_joy": "",
    },
    # Magnitude of the fake nav_joy vector, 0..1 — NOT a speed by itself, see
    # _pump_nav_joy: axes[1]/axes[2] together encode the bearing to the goal,
    # and this is that vector's length. The stack rescales its own output to
    # maxSpeed regardless (confirmed live), so this mostly just needs to stay
    # comfortably nonzero after being split into cos/sin components.
    "nav_joy_throttle": 0.5,
    # Height band for `map_cloud` points, metres, in the map_frame (NOT
    # relative to the robot — this stack has no live z estimate to be relative
    # to, since the EKF publishing map_frame runs `two_d_mode: true`). Points
    # outside are dropped before upload: ground and ceiling returns would
    # otherwise flood the grid with false occupied cells. The default is a
    # starting guess, not a calibration — verify against the actual map that
    # comes out, per docs/operations/hardware-bringup.md.
    # min_z/max_z are map-frame limits by default. A profile may add floor_z;
    # then they mean physical heights above the floor and the adapter adds that
    # map-frame floor reference before filtering.
    "map_cloud_height_band": {"min_z": -0.3, "max_z": 0.5},
    # Native accumulated-cloud projection settings. The resulting grid is
    # occupied-only; free cells can only be asserted by a native OccupancyGrid
    # or the separate per-scan raytracing path.
    "native_map_resolution": 0.05,
    "native_map_padding_m": 1.0,
    "native_map_max_cells": 8_000_000,
    # Keep ray-traced known-free cells white after the lidar moves on. Unknown
    # cells remain unknown; this only controls retention of observed free space.
    "retain_free_space": False,
    # How close counts as "arrived" for a `nav_goal`-topic navigation stack,
    # metres. Unlike actionlib there is no explicit success signal to wait
    # for, so this adapter watches its own tf2 pose against the goal.
    "nav_goal_tolerance_m": 0.5,
    # `move_base`'s actionlib namespace — the ROS 1 convention, the way
    # `navigate_to_pose` is the ROS 2/Nav2 one.
    "actions": {"navigate_to_pose": "move_base"},
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

