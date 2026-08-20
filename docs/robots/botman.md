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

2. **Deploy and start Robot-Side Services**:
   From the operator workstation:
   ```bash
   BACKEND_HOST=<OPERATOR_IP> \
   BOTMAN_OAK_X=<measured> BOTMAN_OAK_Y=<measured> BOTMAN_OAK_Z=<measured> \
   BOTMAN_OAK_ROLL=<measured> BOTMAN_OAK_PITCH=<measured> BOTMAN_OAK_YAW=<measured> \
   make deploy ROBOT=botman
   ```
   The operator command syncs the checkout, writes the overrides on Botman,
   builds the local images, resets the old Compose stack, and starts it.
   This launches:
   - Bunker base driver & SuperOdometry mapping stack
   - OAK-D Pro RGB-D camera publisher (`botman_oak_rgbd.yaml`)
   - OAK mount TF publisher (`oak_mount_tf`)
   - Duck detection sidecar (`duck_detector`)
   - SwarmDeck ROS 2 adapter (`adapter`)
   - Low-latency media streaming bridge (`media`)

3. **Manual shutdown (if needed)**:
   ```bash
   ssh botman 'cd /ssd/swarmdeck && docker compose -f deploy/compose/docker-compose.robot-botman.yml down'
   ```
