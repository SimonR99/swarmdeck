# SwarmDeck — Requirements

## 1. Purpose

SwarmDeck is a **multi-robot supervision stack**: lidar-equipped robots, merged
maps, and a browser GUI for one operator. This document is the target product
contract, not an implementation-status report; see the root README and roadmap.

ARGoS is the default simulation environment. Robots connect through a versioned
adapter contract, so ROS 2, ROS 1, simulated, and custom robots can coexist.
Requirements below are acceptance targets: recording/replay, measured performance,
and access controls are not all implemented or validated. See the
[development objectives](roadmap.md) for the current gaps and priorities.

## 2. Scope

**In scope**

- ARGoS simulation: seeded indoor/Bistro scenarios, configurable fleet profiles,
  LiDAR, IMU, wheel data, and RGB-D cameras. Gazebo is a legacy comparison path.
- Per-robot 2D SLAM producing an occupancy grid.
- Collaborative pose-graph alignment and shared 2D/3D maps, supporting initially
  unaligned robots when sufficient overlapping observations exist.
- Tactical 3D point, voxel, and surface views; optional externally trained
  Gaussian reconstructions.
- Autonomous navigation and teleoperation per robot.
- Web GUI: 2D map canvas, 3D scene, per-robot cameras, status, click-to-navigate, alerts.
- Low-latency video via WebRTC (WHEP), per-robot selection, and JPEG fallback.
- Open-vocabulary object detection (YOLOE) projected onto the global map.
- Synchronized recording and offline replay of a full session.
- Heterogeneous fleet support via per-robot adapters and capability-aware controls.
- Recoverable command/connection behavior and access controls before broader deployment.
- Optional Cortex assistance with explicit provider and fleet-authority boundaries.

**Out of scope**

- Long-range or lossy-link networking. LAN and localhost only.
- Cloud hosting, high availability, and collaborative multi-operator workflows.
  Authentication/authorization is a hardening objective, not an excluded requirement.
- More than one operator at a time.
- Certifying robot safety or replacing onboard emergency stops and collision control.

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
- **FR-S2** Configurable robot profiles and namespaced IDs; the default scenario
  uses four Bunkers and the three-robot development scenario exercises mixed platforms.
- **FR-S3** Simulated LiDAR, IMU, wheel data, and RGB-D feed the selected odometry
  and mapping pipeline. Preserve 3D returns for keyframes and derive planar scans
  separately for onboard navigation.
- **FR-S4** Test initially unaligned robot maps. Ground-truth spawn poses are
  evaluation data; they must not count as geometrically verified loop closures.
- **FR-S5** Detectable target objects placed throughout the world.
- **FR-S6** Config and seed determine scene generation and configured starts.
  Record external assets and software versions needed to reproduce a run.
- **FR-S7** Headless mode with no GUI process, for CI and automated testing.
- **FR-S8** Spawn count is set by scenario config (including 1, 2, 3, and 4);
  operator enable/disable controls do not change the simulator spawn count.

### 4.3 Mapping (`FR-M`)

- **FR-M1** Per-robot 2D SLAM producing `nav_msgs/OccupancyGrid`.
- **FR-M2** Reduce 3D lidar to a 2D scan by configurable height band.
- **FR-M3** Render shared occupancy and clouds from aligned robot trajectories;
  represent unmerged components explicitly.
- **FR-M4** Mapping mode is selected by configuration:
  - `graph` — collaborative pose-graph alignment, the main fleet default.
  - `static` — configured transforms for surveyed starts or diagnostic fallback.
  - `auto` / `cslam` — legacy comparison modes, not the development default.
- **FR-M5** Serve the merged map to the GUI as a full grid on connect and incremental
  patches thereafter.
- **FR-M6** Report trajectory and map error against simulation or surveyed hardware
  ground truth, including failed/false merges and dataset/version information.
- **FR-M7** Map state survives GUI reload and reconnection.
- **FR-M8** Shared maps may inform global planning through verified frame
  alignment. Live onboard sensors retain local collision-avoidance authority.

### 4.4 Navigation and control (`FR-N`)

- **FR-N1** Autonomous point-to-point navigation per robot.
- **FR-N2** Click-to-navigate from the GUI to any selected robot.
- **FR-N3** Reject assigning the same goal to two robots.
- **FR-N4** Select one or several robots; commands apply to the selection.
- **FR-N5** Cancel an active goal.
- **FR-N6** Robots avoid static obstacles, dynamic obstacles, and each other.
- **FR-N7** Stop-all cancels active navigation/exploration and requests zero velocity
  from connected capable robots. Measure delivery and stopping latency; retain
  robot-side deadman/emergency-stop behavior when the network is unavailable.
- **FR-N8** Navigation commands are expressed in adapter-contract terms, not in terms
  of any specific planner, so ROS 1 and ROS 2 robots accept the same command.
- **FR-N9** Fleet exploration is explicitly started/stopped; launching the normal
  ARGoS stack leaves robots stationary unless startup exploration is requested.

### 4.5 Perception (`FR-P`)

- **FR-P1** Object detection on each robot's camera stream.
- **FR-P2** Detections attributed to robot and camera, with class, score, bbox.
- **FR-P3** Project detections onto map coordinates.
- **FR-P4** Deduplicate: the same object seen by two robots is one map entity.
- **FR-P5** Operator-reviewed objects persist as map entities; transient camera
  tracks and accepted objects have distinct lifecycles.

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

### 4.8 Access and optional assistant (`FR-X`)

- **FR-X1** Authenticate callers and authorize control/assistant actions before
  exposing them beyond a trusted operator network. This is not yet implemented.
- **FR-X2** Keep Cortex opt-in; preserve the fleet UI/API without a model provider.
- **FR-X3** Separate proposal, coding access, and fleet execution authority, with
  auditable tool outcomes and explicit failure reporting.

## 5. Non-functional requirements

Performance values are targets requiring measurements on a named machine and
configuration; unit tests and historical benchmarks do not establish compliance.

| ID | Requirement |
|---|---|
| **NFR-1** | Camera glass-to-glass latency < 300 ms; GUI interaction feedback < 100 ms. |
| **NFR-2** | Four robots and a live map for 15 min on one documented workstation; measure stream throughput, latency, drops, and recording gaps. Report bounded-queue loss explicitly. |
| **NFR-3** | Full stack starts with one command and stops cleanly, leaving no orphan processes. |
| **NFR-4** | A user with no ROS knowledge can run a session from the README. |
| **NFR-5** | Same config, seed, and asset versions produce identical scene inputs. Assess full-run repeatability statistically; asynchronous estimators/rendering need not be bit-identical. |
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
6. Selected camera feeds meet the latency target on a documented network and
   machine; report capture-to-display measurements rather than transport availability.
7. Inactivity, failure, and adapter-disconnect alerts fire at their thresholds.
8. Reloading the browser restores the full map without re-running the simulation.
9. A recorded session replays into the GUI and reproduces the operator's view.
10. Session validation passes; MCAP opens in Foxglove, JSONL and CSV in Python/R.
11. Matching config, seed, and asset versions reproduce scene inputs; repeated
    runs report trajectory/coverage variation and any unexplained differences.
12. **The backend runs with zero ROS packages installed**, driven by a mock adapter.
13. The Docker simulation stack starts with one command:
    ```bash
    make up-sim RENDER=gpu
    ```

## 7. Prerequisites

Docker Compose with ARGoS is the supported simulation setup. Host development
requires the pinned ARGoS fork, rendering SDK/plugins, ROS 2 Jazzy, Nav2, and
onboard SLAM; external odometry runs in its separate environment. Gazebo Harmonic,
wheel/IMU EKF, and RTAB-Map apply to legacy comparisons. See
[simulation setup](simulation.md) for prerequisites and current commands.

ROS 1 Noetic is EOL and must run in the robot's own environment or container;
the ROS-free backend does not require a ROS 1 bridge. See the root README for
commands and the hardware guide for physical deployment.
