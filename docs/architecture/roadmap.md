# Roadmap

This file records current implementation status and the remaining work. Product
requirements are defined in [requirements.md](requirements.md).

## Implemented

| Area | Current state |
|---|---|
| Adapter contract | Versioned WebSocket/HTTP protocol; backend remains ROS-free. |
| Adapters | Mock, Gazebo, configurable ROS 1, and configurable ROS 2 adapters. |
| Fleet UI | Registration, status, selection, alerts, manual drive, navigation, and stop-all. |
| Mapping | Per-robot views, dynamic grids, `static` and guarded `auto` merging. |
| 3D mapping | Optional compressed point-cloud upload and WebGL2 viewer. |
| Collaborative SLAM | Experimental Swarm-SLAM graph reporting and `cslam` merge mode. |
| Video | RTSP ingest through MediaMTX, WHEP/WebRTC display, JPEG fallback. |
| Perception | YOLOE sidecar, depth projection, operator review, deduplication, persistence. |
| Simulation | Seeded Gazebo fleet, SLAM, Nav2, reactive exploration, headless integration test. |
| Hardware | Scout, Botman, Aslan, and Spot profiles with centralized deployment. |
| Session events | Session manifest and timestamped JSONL operator/system event log. |

## Priority work

### 1. Hardware readiness

- Give every robot the same functional post-deploy checks for backend
  registration, telemetry, map, camera, and advertised navigation interfaces.
- Add offline deployment validation and a new-robot scaffold.
- Record calibration and image/workspace prerequisites without embedding secrets.
- Run repeatable motion, stop, reconnect, and degraded-sensor tests on each robot.

Exit: one documented command deploys each provisioned robot and reports functional
readiness, not only running containers.

### 2. Consistent collaborative mapping

- Make occupancy grids and inter-robot transforms originate from one optimized
  trajectory estimate.
- Add loop-consistency checks, disconnected-component handling, and reference
  recovery.
- Validate transforms against ground truth and physical surveyed landmarks.

Exit: collaborative mode improves or matches guarded grid registration and never
places disconnected robots in an assumed common frame.

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
