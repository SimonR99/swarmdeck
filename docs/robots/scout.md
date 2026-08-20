# Scout Mini (TARS) Bring-up

The Scout Mini is an AgileX 4WD skid-steer rover running native ROS 1 Noetic on a Jetson AGX Xavier.

## Hardware Specifications
- **Base**: AgileX Scout Mini (`0.62 x 0.585 m`).
- **Compute**: Jetson AGX Xavier (Ubuntu 20.04 / ROS 1 Noetic).
- **Sensors**: Ouster OS1 lidar, VectorNav VN-100 IMU, Intel RealSense D435 camera.
- **SLAM**: LVI-SAM producing `/lvi_sam/lidar/mapping/odometry`.

## Bring-up Instructions

1. **Deploy and launch the Autonomous Stack & SwarmDeck Bridge**:
   From the operator machine, execute:
   ```bash
   make deploy ROBOT=scout
   ```
   This syncs/builds the checkout, SSHs into `scout`, configures the ROS 1 master and network interfaces, and launches:
   - Base drivers and sensors (`rover_agx.launch`)
   - Low-latency RealSense camera publisher (`scout_camera_low_latency.launch`, 10 FPS / 700 kbps)
   - Media streaming bridge (`ros1_rtsp.py`)
   - ROS 1 SwarmDeck adapter (`adapters/adapter_ros1/adapter_ros1.py`)

   `./scripts/scout-up` remains available as the low-level bring-up/check command
   when the images are already deployed.

2. **Operator Station Server**:
   ```bash
   .venv/bin/python -m swarmdeck_server --config configs/hardware_tars.yaml
   ```
