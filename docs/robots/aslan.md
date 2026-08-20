# Aslan Bring-up

Aslan is an AgileX Bunker tracked rover running ROS 2 Humble in Docker on a Jetson AGX Orin.

## Hardware Specifications
- **Base**: AgileX Bunker tracked platform.
- **Compute**: Jetson AGX Orin.
- **Sensors**: Ouster OS1-128 lidar + IMU.
- **SLAM**: SuperOdometry.

## Bring-up Instructions

1. **Start SwarmDeck Server**:
   ```bash
   cd server
   .venv/bin/python -m swarmdeck_server --config ../configs/hardware_aslan.yaml
   ```

2. **Start Robot-Side Services (on Aslan)**:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> docker compose -f docker-compose.robot-aslan.yml up -d
   ```

3. **Shutdown**:
   ```bash
   docker compose -f docker-compose.robot-aslan.yml down
   ```
