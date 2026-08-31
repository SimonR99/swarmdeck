# Documentation

Start with the root [README](../README.md) for setup and package structure.

| Topic | Document |
|---|---|
| Components, data flow, and frames | [Architecture](architecture/overview.md) |
- [Simulation](architecture/simulation.md) — the ARGoS backend: photorealistic rendering, Jolt physics, and Ultra-Fusion odometry.
| Pose-graph collaborative SLAM design | [Collaborative mapping plan](architecture/collaborative-mapping-plan.md) |
| Low-odometry reconstruction and measured accuracy | [Odometry-free reconstruction](architecture/odometry-free-keyframe-reconstruction.md) |
| Joint frontier allocation and measured exploration | [Coordinated exploration](architecture/coordinated-exploration.md) |
| Legacy grid-registration & Swarm-SLAM analysis | [Collaborative SLAM](architecture/collaborative-slam.md) |
| Detection and operator review | [Perception](architecture/perception.md) |
| Product scope and acceptance criteria | [Requirements](architecture/requirements.md) |
| Implemented and remaining work | [Roadmap](architecture/roadmap.md) |
| Adapter messages and binary payloads | [Adapter protocol](../adapters/protocol/README.md) |
| Physical fleet hardware | [Fleet matrix](robots/fleet.md) |
| Deployment and safety checks | [Hardware bring-up](operations/hardware-bringup.md) |
| Active limitations and traps | [Known issues](operations/known-issues.md) |

Robot-specific prerequisites and shutdown commands: [Scout](robots/scout.md),
[Botman](robots/botman.md), [Aslan](robots/aslan.md), and [Spot](robots/spot.md).
