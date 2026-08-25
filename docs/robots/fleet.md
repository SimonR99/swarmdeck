# Physical fleet

| Robot ID | Platform | Compute / ROS | Localization / Odom | Sensors | Deploy target |
|---|---|---|---|---|---|
| `tars_0` | AgileX Scout Mini | Jetson AGX Xavier, ROS 1 Noetic | LVI-SAM / filtered odometry | Ouster OS1, VectorNav, RealSense D435 | `scout` (`192.168.1.230`) |
| `botman_0` | AgileX Bunker | Jetson AGX Orin, ROS 2 Humble | SuperOdometry | Ouster OS1-128, OAK-D Pro RGB-D | `botman@botman.local` |
| `aslan_0` | AgileX Bunker | Jetson AGX Orin, ROS 2 Humble | SuperOdometry | Ouster OS0-64, VectorNav, OAK-D | `aslan@aslan.local` |
| `spot_0` | Boston Dynamics Spot | Jetson AGX Orin, ROS 2 Humble | LIO-SAM | Ouster, VectorNav, RealSense D435 | `spot` (`192.168.1.192`) |

## Robot Contract & Data Flow

Each robot can run its own local SLAM (LVI-SAM, SuperOdometry, LIO-SAM) or filtered odometry to produce a good local pose. To integrate with SwarmDeck, the robot only needs to publish:

1. **Odometry & Pose**: 5 Hz continuous pose stream (`robot_state` via `WS /adapter`).
2. **3D Spatial Information**: Voxelized 3D keyframe packets (`POST /api/adapter/keyframe` @ ~0.5 Hz / distance trigger) for server-side pose-graph SLAM, plus optional registered 3D cloud (`POST /api/adapter/cloud`) for the WebGL viewer.
3. **Video Stream**: H.264 camera video stream ingested via RTSP (`:8554/<robot_id>`).
4. **Operational Metadata**: Battery percentage, Wi-Fi link quality, object detections, nav status, and planned paths.

The server's collaborative SLAM backend (`swarmdeck-slam`) optimizes trajectories across the fleet, renders the global occupancy map, and feeds it back to each robot's Nav2 planner via the navigation map downlink (`GET /api/map/nav/<robot_id>` $\rightarrow$ `/global_map`). Robots do **not** need to rasterize or upload 2D occupancy grids on board.

The operator/server address is `BACKEND_HOST` in `deploy/fleet.env`. Hardware
profiles keep their documented ROS domains separate. Video uses RTSP ingest on
8554 and WHEP/WebRTC on 8889; a Zenoh router may use TCP 7447.

See [hardware bring-up](../operations/hardware-bringup.md) for the common
procedure and the per-robot pages for prerequisites and safety exceptions.
