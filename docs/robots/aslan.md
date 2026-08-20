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

2. **Deploy and start Robot-Side Services**:
   From the operator workstation:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> make deploy ROBOT=aslan
   ```
   This also builds the Aslan ROS overlay before building and starting the
   robot-side Compose services.
   The base driver remains opt-in for safety; once the physical e-stop and
   CAN interface are ready, use `DEPLOY_COMPOSE_PROFILES=base` with the same
   command to start it.

3. **Manual shutdown (if needed)**:
   ```bash
   ssh aslan 'cd /ssd/swarmdeck && docker compose -f deploy/compose/docker-compose.robot-aslan.yml down'
   ```
