# Aslan

Aslan is an AgileX Bunker on ROS 2 Humble with Ouster/VectorNav sensing,
SuperOdometry, Nav2, OAK video, and the ROS 2 adapter.

Prerequisites on the robot:

- SSH at `aslan@192.168.1.139`; checkout `/ssd/swarmdeck`.
- Read-only MIST workspace `/ssd/mist_ws_ros2` and image `bunker:dev`.
- Writable SwarmDeck overlay directory and one-time `.aslan_pip` dependencies.
- ROS domain 49. The base additionally requires a ready `can2` and physical
  e-stop supervision.

```bash
make deploy ROBOT=aslan                 # sensing, mapping, UI bridge
DEPLOY_COMPOSE_PROFILES=base make deploy ROBOT=aslan  # include base driver
```

The deployment builds the Aslan overlay. The base is intentionally excluded by
default. Complete the common [pre-flight checks](../operations/hardware-bringup.md)
before enabling its profile.

```bash
ssh aslan 'cd /ssd/swarmdeck && docker compose --env-file .deploy/aslan.env \
  -f deploy/compose/docker-compose.robot-aslan.yml --profile "*" down'
```
