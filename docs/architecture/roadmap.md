# Roadmap

This file records current implementation status and the remaining work. Product
requirements are defined in [requirements.md](requirements.md).

## Implemented

| Area | Current state |
|---|---|
| Adapter contract | Versioned WebSocket/HTTP protocol; backend remains ROS-free. |
| Adapters | Mock, Gazebo, configurable ROS 1, and configurable ROS 2 adapters (with keyframe streaming). |
| Fleet UI | Registration, status, selection, alerts, manual drive, navigation, and stop-all. |
| Mapping | Per-robot views, dynamic grids, `static`, guarded `auto`, and trajectory-based `graph` modes. |
| 3D mapping | Optional compressed point-cloud upload and WebGL2 viewer. |
| Collaborative SLAM | Joint GTSAM pose-graph back-end (`slam/`), Scan Context loop retrieval, GICP geometric verification, PCM outlier rejection, and trajectory-rendered occupancy grids (`merge_mode: graph`). Legacy Swarm-SLAM `cslam` mode retained for comparison. |
| Video | RTSP ingest through MediaMTX, WHEP/WebRTC display, JPEG fallback. |
| Perception | YOLOE sidecar, depth projection, operator review, deduplication, persistence. |
| Simulation | Seeded Gazebo fleet, SLAM, Nav2, reactive exploration, keyframe extraction, headless integration test. |
| Hardware | Scout, Botman, Aslan, and Spot profiles with centralized deployment and verification. |
| Session events | Session manifest and timestamped JSONL operator/system event log. |

## Priority work

### 1. Multi-robot dataset recording & ground truth validation

- Record multi-robot time-aligned bags/MCAP on physical hardware (Ouster lidar + IMU).
- Survey ground-truth control points to calibrate sensor noise models and information matrices.
- Validate trajectory optimization against surveyed control points (ATE / RPE).

Exit: reproducible multi-robot replay harness driven by physical bags with surveyed ground truth.

### 2. Robot-side front-end unification

- Unify front-end lidar-inertial odometry across the fleet using time-synced Ouster lidar + IMU (FAST-LIO2 or DLIO).
- Gravity-align all front-ends to eliminate extrinsic pitch/tilt errors.
- Preserve ROS 1 control on Scout Mini while running modernized lidar-inertial front-end in isolated container.

Exit: consistent pointcloud and odometry estimates across all hardware platforms.

### 3. Complete session recording and replay

- Record fleet state, maps, detections, commands, and stream health in a durable
  format such as MCAP.
- Add session validation, configuration/version snapshots, and replay without
  connected adapters.
- Keep JSONL operator events as the human-readable action index.

Exit: a stopped session validates and reproduces the operator view offline.

### 4. Hardening and security

- Add authentication and authorization before exposing hardware controls.
- Exercise adapter loss, stream loss, disk exhaustion, browser reload, and robot
  restart during active navigation.
- Run repeated four-robot soak tests and verify deterministic simulation seeds.
- Freeze tested deployment configurations for data collection.

Exit: two consecutive soak runs complete without silent data loss, unsafe command
replay, or orphaned processes.

## Design constraints

- Planner and middleware details stay inside adapters.
- The backend and browser remain ROS-free.
- Hardware adapters never advertise the simulation-only `reset` capability.
- `static` map transforms remain the reliable fallback when automatic alignment
  lacks evidence.
- New robot support may require adapter/configuration and deployment packaging,
  but must not require backend or UI code changes.
