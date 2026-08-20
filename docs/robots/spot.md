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

2. **Deploy and start Robot-Side Services**:
   From the operator workstation:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> make deploy ROBOT=spot
   ```

3. **Manual shutdown (if needed)**:
   ```bash
   ssh spot 'cd /home/indro/swarmdeck && docker compose -f deploy/compose/docker-compose.robot-spot.yml down'
   ```
