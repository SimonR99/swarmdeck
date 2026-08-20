# SwarmDeck Documentation

SwarmDeck is a web-first multi-robot operations platform providing live telemetry, 2D/3D map merging, real-time WebRTC camera streams, open-vocabulary object detection, and coordinated teleoperation.

## Directory Guide

### 📐 Architecture
- [System Architecture](architecture/overview.md) — Core backend, UI, adapters, map engine, and coordinate frame conventions.
- [Perception Pipeline](architecture/perception.md) — Open-vocabulary object detector (YOLOE), depth projection, and detection review.
- [Collaborative SLAM](architecture/collaborative-slam.md) — Swarm-SLAM multi-robot mapping and RTAB-Map integration.
- [Requirements & Protocol](architecture/requirements.md) — Protocol specifications, schemas, and non-functional targets.

### 🤖 Robot Platforms
- [Fleet Overview](robots/fleet.md) — Hardware summary, network matrix, and platform support.
- [Scout Mini (TARS)](robots/scout.md) — AgileX Scout Mini running ROS 1 Noetic & LVI-SAM.
- [Botman](robots/botman.md) — AgileX Bunker running ROS 2 Humble & SuperOdometry with OAK-D Pro RGB-D.
- [Aslan](robots/aslan.md) — AgileX Bunker running ROS 2 Humble & SuperOdometry.
- [Spot](robots/spot.md) — Boston Dynamics Spot payload running ROS 2 Humble & LIO-SAM.

### 🚀 Operations & Deployment
- [Hardware Bring-up & Deployment](operations/hardware-bringup.md) — Deploying adapters to physical robots, Zenoh routing, and operator setup.
- [Known Issues & Troubleshooting](operations/known-issues.md) — Active issues, diagnostic tips, and performance guidelines.
