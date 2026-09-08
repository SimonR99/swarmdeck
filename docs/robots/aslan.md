# Aslan

Aslan is an AgileX Bunker on ROS 2 Humble with Ouster/VectorNav sensing,
SuperOdometry, Nav2, OAK video, and the ROS 2 adapter.

Prerequisites on the robot:

- SSH at `aslan@aslan.local`; checkout `/ssd/swarmdeck`.
- Read-only MIST workspace `/ssd/mist_ws_ros2` and image `bunker:dev`.
- Writable SwarmDeck overlay directory and one-time `.aslan_pip` dependencies.
- OAK-D RGB-D is enabled by default for map-projected detections. The current
  approximate mount is `x=0.03`, `y=0`, `z=-0.04` m from `os_lidar`, with zero
  roll/pitch/yaw; replace it with a measured transform when available.
- ROS domain 49. The base additionally requires a ready `can2` and physical
  e-stop supervision.

```bash
make deploy ROBOT=aslan                 # full stack (base driver, sensing, SLAM, Nav2)
```

The deployment builds the Aslan overlay. The Bunker base driver (`robot_stack`) starts
up by default alongside sensing and Nav2. Complete the common [pre-flight checks](../operations/hardware-bringup.md)
before commanding motion.

```bash
ssh aslan 'cd /ssd/swarmdeck && docker compose --env-file .deploy/aslan.env \
  -f deploy/compose/docker-compose.robot-aslan.yml --profile "*" down'
```
