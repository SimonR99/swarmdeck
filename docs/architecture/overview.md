# SwarmDeck Architecture

SwarmDeck connects heterogeneous robots to a web dashboard without ROS on the
server or browser.

```mermaid
flowchart TB
    UI["Browser UI (:5173)<br/>Svelte 5 · Canvas 2D · WebGL2 3D"]
    Server["FastAPI server (:8080)<br/>Fleet · Maps · Events · API"]
    SLAM["Collaborative SLAM (:8090)<br/>Python 3.12 · GTSAM · GICP · PCM"]
    ROS2["ROS 2 adapter<br/>Botman · Aslan · Spot"]
    ROS1["ROS 1 adapter<br/>Scout Mini"]
    Sim["Simulation adapter"]
    Mock["Synthetic adapter"]
    Detector["YOLOE perception sidecar"]
    Media["MediaMTX (:8554 / :8889)"]

    UI <-->|REST / WebSocket| Server
    Server <-->|Forward keyframes / optimized maps| SLAM
    ROS2 <-->|Adapter protocol + keyframes| Server
    ROS1 <-->|Adapter protocol + keyframes| Server
    Sim <-->|Adapter protocol + keyframes| Server
    Mock <-->|Adapter protocol| Server
    ROS2 <-->|Inference| Detector
    ROS1 <-->|Inference| Detector
    Sim <-->|Inference| Detector
    ROS2 -->|RTSP| Media
    ROS1 -->|RTSP| Media
    Sim -->|RTSP| Media
    Media -->|WHEP / WebRTC| UI
```

## 1. System Components

### A. SwarmDeck Server (`server/`)

- FastAPI, WebSockets, and NumPy; no `rclpy` or `rospy`.
- Fleet registry for identity, capabilities, telemetry, liveness, and commands.
- Map service for scan accumulation, dynamic bounds, registration, merging, and
  network-quality grids.
- Detection review, persistent settings, and timestamped event logging.
- Runs in its own virtual environment (Python 3.10+, NumPy 2.x).

### B. User Interface (`ui/`)

Svelte 5 renders the 2D occupancy map and a raw-WebGL2 point cloud. It consumes
REST/WebSocket state and WHEP/WebRTC video, with JPEG fallback and stream/link
diagnostics.

### C. Protocol Adapters (`adapters/`)

- `adapter_ros1`: ROS 1 Noetic hardware, including Scout/LVI-SAM.
- `adapter_ros2`: ROS 2 Humble/Jazzy hardware, including Bunker and Spot.
- `adapter_sim`: simulated fleet with planar/3D keyframe extraction. Backed by
  ARGoS by default; see [simulation](simulation.md).
- `adapter_mock`: synthetic fleet without ROS or a GPU.

All use the same [wire protocol](../../adapters/protocol/README.md).

### D. Collaborative SLAM Back-end (`slam/`)

- Dedicated service (`swarmdeck-slam`, port 8090) implementing trajectory-based
  collaborative SLAM.
- Ingests keyframe packets (voxel-downsampled base-frame point cloud + odometry pose).
- Generates Scan Context descriptors, retrieves candidate loop closures via KD-tree,
  and verifies them geometrically using GICP.
- Rejects false loop closures via Pairwise Consistency Maximization (PCM, minimum clique size 2)
  and Graduated Non-Convexity (GNC).
- Optimizes a joint pose graph in GTSAM and renders consistent 2D occupancy grids per
  multi-robot connected component directly from optimized trajectories.
- **Python Environment Isolation**: Strictly pinned to Python 3.12 and NumPy < 2.
  `gtsam==4.2.2` segfaults under NumPy 2.x, which is why it runs in its own isolated
  distribution (`slam/.venv`) separate from `server/.venv`.

## 2. Coordinate Frames & Transforms

SwarmDeck standardizes coordinate frames across heterogeneous robots:

| Frame | Scope | Description |
|---|---|---|
| shared/world | Deployment | Display-only placement for replicas. |
| `robot/odom` | Robot | Continuous navigation and planning frame. |
| `base_link` | Robot | Chassis frame. |
| sensor frames | Robot | Camera, lidar, and IMU frames. |

### Transform and correction conventions

- **Direction**: Every transform `T_a_b` maps coordinates in frame `b` into
  frame `a` ($p_a = T_{a\_b} \cdot p_b$).
- **Navigation**: MGG plans in each robot's continuous odometry frame.
- **Corrections**: Peer Swarm-SLAM publishes `T_component_navigation` as data;
  it never creates a competing map-to-odometry TF edge.
- **Products**: MOLA consumes peer snapshots and publishes the occupancy
  products queried by MGG. `SWARMDECK_SLAM_BACKEND=cslam` is the only backend.


### Safety boundary

The `reset` capability is strictly simulation-only (`adapter_sim`, `mock_adapter`). Hardware
adapters must never advertise or implement `reset`.
