# Botman Bring-up

Botman is an AgileX Bunker tracked rover running ROS 2 Humble in Docker on a Jetson AGX Orin.

## Hardware Specifications
- **Base**: AgileX Bunker (`1.02 x 0.78 m`, circumscribed radius 0.65 m).
- **Compute**: Jetson AGX Orin (Ubuntu 22.04 / Docker).
- **Sensors**: Ouster OS1-128 lidar, Luxonis OAK-D Pro RGB-D camera.
- **SLAM**: SuperOdometry publishing `/laser_odometry` and registered lidar scans.

## Bring-up Instructions

1. **Start SwarmDeck Server**:
   ```bash
   cd server
   .venv/bin/python -m swarmdeck_server --config ../configs/hardware_botman.yaml
   ```

2. **Start Robot-Side Services (on Botman)**:
   From the SwarmDeck checkout on Botman:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> docker compose -f docker-compose.robot-botman.yml up -d
   ```
   This launches:
   - Bunker base driver & SuperOdometry mapping stack
   - OAK-D Pro RGB-D camera publisher (`botman_oak_rgbd.yaml`)
   - OAK mount TF publisher (`oak_mount_tf`)
   - Duck detection sidecar (`duck_detector`)
   - SwarmDeck ROS 2 adapter (`adapter`)
   - Low-latency media streaming bridge (`media`)

3. **Shutdown**:
   ```bash
   docker compose -f docker-compose.robot-botman.yml down
   ```
