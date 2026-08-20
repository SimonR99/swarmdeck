# SwarmDeck — Requirements

## 1. Purpose

SwarmDeck is a **multi-robot supervision stack**: lidar-equipped robots, merged
maps, and a browser GUI for one operator. This document is the target product
contract, not an implementation-status report; see the root README and roadmap.

Built and validated in simulation. **Robots connect through a version-agnostic adapter
contract**, so ROS 2 robots, ROS 1 robots, and Gazebo can coexist in one fleet.

## 2. Scope

**In scope**

- Gazebo simulation: indoor world, 4 differential-drive robots with lidar, IMU, wheel
  odometry, RGB camera.
- Per-robot 2D SLAM producing an occupancy grid.
- Merged 2D occupancy grid across robots, supporting unknown relative start poses.
- 3D point cloud visualization layer with voxel downsampling and detection volumes.
- Autonomous navigation and teleoperation per robot.
- Web GUI: 2D map canvas, 3D scene, per-robot cameras, status, click-to-navigate, alerts.
- Low-latency video via WebRTC (WHEP) for simultaneous camera feeds.
- Open-vocabulary object detection (YOLOE) projected onto the global map.
- Synchronized recording and offline replay of a full session.
- **Heterogeneous fleet support** via per-robot adapters (ROS 2, ROS 1, simulation).

**Out of scope**

- Long-range or lossy-link networking. LAN and localhost only.
- Authentication, multi-user, cloud, high availability.
- More than one operator at a time.
- Production hardening. Clear and reproducible beats bulletproof.

## 3. Actors

| Actor | Needs |
|---|---|
| **Operator** | Supervise 1–4 robots from one screen. No ROS knowledge. |
| **Session operator** | Configure a run, start/stop, verify recording is complete. |
| **Analyst** | Open recorded sessions with standard tools; replay them. |
| **Robot integrator** | Add a robot without changing backend or UI code. |

## 4. Functional requirements

### 4.1 Fleet integration (`FR-A`)

- **FR-A1** Robots connect to the backend through a documented **adapter contract**.
  The backend has **no ROS dependency** of any kind.
- **FR-A2** Ship ROS 1, ROS 2, simulation, and ROS-free reference adapters; allow
  vendor SDK adapters through the same contract.
- **FR-A3** An adapter runs in its own environment — its own OS, ROS distro, or
  container — and never requires matching the backend host.
- **FR-A4** Robots of different types coexist in one fleet with no backend change.
- **FR-A5** Robot identity, type, and capabilities are declared at adapter connect;
  the GUI adapts to declared capabilities.
- **FR-A6** Adapter disconnect is surfaced in the GUI and logged, never silent.

### 4.2 Simulation (`FR-S`)

- **FR-S1** Indoor world: rooms, corridors, static obstacles, a few dynamic ones.
- **FR-S2** Four identical differential-drive robots, namespaced `/robot_0` … `/robot_3`.
- **FR-S3** Each robot carries lidar, IMU, wheel odometry, RGB camera. 3D lidar is
  permitted; it is reduced to a 2D scan before mapping.
- **FR-S4** Robots start at **unknown relative positions**.
- **FR-S5** Detectable target objects placed throughout the world.
- **FR-S6** One seed fixes target placement, dynamic-obstacle paths, and start poses.
- **FR-S7** Headless mode with no GUI process, for CI and automated testing.
- **FR-S8** Active robot count is set by config (1, 2, or 4), not by the operator.

### 4.3 Mapping (`FR-M`)

- **FR-M1** Per-robot 2D SLAM producing `nav_msgs/OccupancyGrid`.
- **FR-M2** Reduce 3D lidar to a 2D scan by configurable height band.
- **FR-M3** A map service merging all robots' grids into one world-frame grid.
- **FR-M4** Two merge modes, selectable by config:
  - `static` — known start poses, fixed transforms (default, always available).
  - `auto` — unknown starts, estimated by grid feature matching.
- **FR-M5** Serve the merged map to the GUI as a full grid on connect and incremental
  patches thereafter.
- **FR-M6** Report merge accuracy against Gazebo ground truth, in meters.
- **FR-M7** Map state survives GUI reload and reconnection.
- **FR-M8** Merged map feeds each robot's navigation costmap.

### 4.4 Navigation and control (`FR-N`)

- **FR-N1** Autonomous point-to-point navigation per robot.
- **FR-N2** Click-to-navigate from the GUI to any selected robot.
- **FR-N3** Reject assigning the same goal to two robots.
- **FR-N4** Select one or several robots; commands apply to the selection.
- **FR-N5** Cancel an active goal.
- **FR-N6** Robots avoid static obstacles, dynamic obstacles, and each other.
- **FR-N7** Stop-all control that halts every robot immediately.
- **FR-N8** Navigation commands are expressed in adapter-contract terms, not in terms
  of any specific planner, so ROS 1 and ROS 2 robots accept the same command.

### 4.5 Perception (`FR-P`)

- **FR-P1** Object detection on each robot's camera stream.
- **FR-P2** Detections attributed to robot and camera, with class, score, bbox.
- **FR-P3** Project detections onto map coordinates.
- **FR-P4** Deduplicate: the same object seen by two robots is one map entity.
- **FR-P5** Detections persist as map markers after the robot moves on.

### 4.6 GUI (`FR-G`)

- **FR-G1** 2D map view with all robot poses, headings, and trails.
- **FR-G2** Per-robot status card: pose, battery, mode, nav state, link health.
- **FR-G3** Live camera view, switchable per robot, with detection overlay.
- **FR-G4** Alert when a robot has been unattended past a configurable threshold.
- **FR-G5** Alert on navigation failure, robot fault, adapter disconnect, stream loss.
- **FR-G6** Tablet-friendly responsive layout; touch targets usable without a mouse.
- **FR-G7** Reconnect after interruption or reload without losing map state.

### 4.7 Recording (`FR-R`)

- **FR-R1** Record all fleet state and map updates to MCAP.
- **FR-R2** Log every operator action to JSONL: robot selection, goal issued, goal
  cancelled, camera switch, alert acknowledged, target reported.
- **FR-R3** Every record carries monotonic, wall-clock, and session-relative timestamps.
- **FR-R4** One self-contained directory per session, with config snapshot and versions.
- **FR-R5** Replay a session into the GUI with no simulation and no robots running.
- **FR-R6** Validation command reporting session completeness and logging gaps.

## 5. Non-functional requirements

| ID | Requirement |
|---|---|
| **NFR-1** | Camera glass-to-glass latency < 300 ms; GUI interaction feedback < 100 ms. |
| **NFR-2** | 4 robots + 4 camera feeds + live map for 15 min on one workstation, no frame drops or logging gaps. |
| **NFR-3** | Full stack starts with one command and stops cleanly, leaving no orphan processes. |
| **NFR-4** | A user with no ROS knowledge can run a session from the README. |
| **NFR-5** | Same config + same seed ⇒ identical simulation run. |
| **NFR-6** | Map updates reach the GUI within 1 s. |
| **NFR-7** | Stream or adapter loss is surfaced in the GUI, never silently dropped. |
| **NFR-8** | Every module runs and is testable in isolation, with the others mocked. |
| **NFR-9** | Adding a robot type requires no backend or UI code change; physical deployment may add adapter configuration, packaging, and calibration. |

## 6. Acceptance criteria

1. Four robots appear on one merged 2D map with correct poses.
2. Robots start at unknown relative positions; the map aligns after they observe
   shared areas. Merge error is reported in meters against ground truth.
3. Navigation goals can be issued from the GUI; robots reach them while avoiding
   obstacles and each other.
4. Duplicate goal assignment is rejected with a visible reason.
5. Detections appear with correct robot and camera attribution, and persist as markers.
6. Four simultaneous camera feeds sustain < 300 ms latency.
7. Inactivity, failure, and adapter-disconnect alerts fire at their thresholds.
8. Reloading the browser restores the full map without re-running the simulation.
9. A recorded session replays into the GUI and reproduces the operator's view.
10. Session validation passes; MCAP opens in Foxglove, JSONL and CSV in Python/R.
11. Two runs with the same seed produce identical start poses, target placement, and
    dynamic-obstacle behaviour.
12. **The backend runs with zero ROS packages installed**, driven by a mock adapter.
13. The Docker simulation stack starts with one command:
    ```bash
    make docker-up-gpu
    ```

## 7. Prerequisites

Docker is the supported setup. Host simulation development uses ROS 2 Jazzy,
Gazebo Harmonic, Nav2, SLAM Toolbox, `robot_localization`, and optionally
RTAB-Map. MediaMTX and perception runtimes are included in Compose images.

ROS 1 Noetic is EOL and must run in the robot's own environment or container;
the ROS-free backend does not require a ROS 1 bridge. See the root README for
commands and the hardware guide for physical deployment.
