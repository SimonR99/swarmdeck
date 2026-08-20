# Fleet Overview & Hardware Matrix

SwarmDeck supports a heterogeneous fleet of ground rovers, tracked platforms, and quadrupeds.

## Platform Summary

| Robot | Hostname / IP | Platform Type | Compute | ROS Stack | SLAM / Odometry | Camera & Sensors |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Scout (TARS)** | `scout` (`192.168.1.230`) | AgileX Scout Mini (Wheeled) | Jetson AGX Xavier | ROS 1 Noetic | LVI-SAM (Lidar-Visual-Inertial) | RealSense D435 + Ouster 64-ch Lidar + VectorNav IMU |
| **Botman** | `botman` (`192.168.1.49`) | AgileX Bunker (Tracked) | Jetson AGX Orin | ROS 2 Humble (Docker) | SuperOdometry | OAK-D Pro RGB-D + Ouster 128-ch Lidar |
| **Aslan** | `aslan` (`192.168.1.139`) | AgileX Bunker (Tracked) | Jetson AGX Orin | ROS 2 Humble (Docker) | SuperOdometry | Ouster 128-ch Lidar + IMU |
| **Spot** | `spot` (`192.168.1.192`) | Boston Dynamics Spot | Jetson AGX Orin Payload | ROS 2 Humble (Docker) | LIO-SAM | RealSense D435 + Velodyne VLP-16 |

---

## Network Architecture

- **Subnet**: Standard operational subnet `192.168.1.0/24`.
- **Zenoh Bridge**: When multi-machine ROS 2 communication is required, Zenoh router operates on port `7447`.
- **WebRTC / RTSP**: MediaMTX bridges RTSP video feeds to WHEP WebRTC on port `8554` / `8889`.
- **SwarmDeck Backend**: WebSocket API server runs on port `8080` (or behind nginx on port `5173`).
