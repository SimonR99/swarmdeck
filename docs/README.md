# Documentation

Start with the [plan](plan.md): architecture, invariants, status, open work and
validation in one page. Then the root [README](../README.md) for setup and
package structure.

The shortest complete local run is `./scripts/sim-up --drift`; it selects the
peer Swarm-SLAM, native MOLA, MGG path and starts the matching reset supervisor.

## Plan and evidence

| Topic | Document |
|---|---|
| Architecture, invariants, status, open work, validation | [Plan](plan.md) |
| Every dated trial, with its numbers and its limits | [Acceptance log](operations/acceptance-log.md) |
| Superseded design, tracker and chronology documents | [Archive](archive/README.md) |

## Operating the stack

| Topic | Document |
|---|---|
| Modes, launcher settings, frames and ownership, validation scope | [Current stack](operations/current-stack.md) |
| Autonomous exploration for simulation and ROS 2 hardware | [MGG exploration](operations/mgg-exploration.md) |
| Native MOLA runtime, products, and query contract | [MOLA runtime](operations/mola-runtime.md) |
| Fleet component catalogue, compatibility and freshness rules | [Replica components](operations/replica-components.md) |
| Tactical 3D maps and RGB-D reconstruction | [3D map workflow](operations/tactical-3d-map.md) |
| Bounded fixed-pose Gaussian training | [Reconstruction jobs](operations/reconstruction-jobs.md) |
| Route display and tracking | [Route tracking](operations/route-tracking.md) |
| Camera ingest and playback | [Camera streams](operations/camera-streams.md) |
| Simulation timestamps, adapter costs, and tuning | [Simulation performance](operations/simulation-performance.md) |
| Simulation reset lifecycle and the host supervisor | [Simulation reset](operations/simulation-reset.md) |
| Capture-time poses, guards, and rotated duplicates | [Keyframe timing](operations/keyframe-yaw.md) |
| Active limitations and traps | [Known issues](operations/known-issues.md) |
| Deployment and safety checks | [Hardware bring-up](operations/hardware-bringup.md) |

## Reference

| Topic | Document |
|---|---|
| Component and frame reference, transform conventions, merge modes | [Architecture overview](architecture/overview.md) |
| Product scope and acceptance criteria | [Requirements](architecture/requirements.md) |
| ARGoS rendering, sensors, and Fast-LIVO2 odometry | [Simulation](architecture/simulation.md) |
| Detection and operator review | [Perception](architecture/perception.md) |
| Low-odometry reconstruction and measured accuracy | [Odometry-free reconstruction](architecture/odometry-free-keyframe-reconstruction.md) |
| Ultra-Fusion native ARM investigation | [Ultra-Fusion native ARM](architecture/ultra-fusion-native-arm.md) |
| Adapter messages and binary payloads | [Adapter protocol](../adapters/protocol/README.md) |
| Development workflow and focused tests | [Contributing](../CONTRIBUTING.md) |

## Physical fleet

| Topic | Document |
|---|---|
| Fleet hardware matrix | [Fleet](robots/fleet.md) |
| Per-robot prerequisites and shutdown | [Scout](robots/scout.md), [Botman](robots/botman.md), [Aslan](robots/aslan.md), [Spot](robots/spot.md) |
