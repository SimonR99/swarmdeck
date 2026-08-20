# Spot Bring-up

Spot is a Boston Dynamics quadruped equipped with an onboard Jetson AGX Orin payload running ROS 2 Humble.

## Hardware Specifications
- **Base**: Boston Dynamics Spot.
- **Payload Compute**: Jetson AGX Orin.
- **Sensors**: Velodyne VLP-16 lidar + RealSense D435 camera.
- **SLAM**: LIO-SAM.

## Bring-up Instructions

1. **Start SwarmDeck Server**:
   ```bash
   cd server
   .venv/bin/python -m swarmdeck_server --config ../configs/hardware_spot.yaml
   ```

2. **Start Robot-Side Services (on Spot payload)**:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> docker compose -f docker-compose.robot-spot.yml up -d
   ```

3. **Shutdown**:
   ```bash
   docker compose -f docker-compose.robot-spot.yml down
   ```
