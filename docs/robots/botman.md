# Botman

Botman is an AgileX Bunker on ROS 2 Humble with Ouster OS1-128,
SuperOdometry, Nav2, and an OAK-D Pro RGB-D camera.

Prerequisites on the robot:

- SSH at `botman@192.168.1.49`; checkout `/ssd/swarmdeck`.
- Read-only MIST workspace `/ssd/mist_ws` and image `bunker_super_odom:dev`.
- ROS domain 17.
- Measured transform from `os_lidar` to `oak-d-base-frame`.

The profile currently has coarse camera extrinsic defaults. Replace them with
measured values when available:

```bash
BOTMAN_OAK_X=<m> BOTMAN_OAK_Y=<m> BOTMAN_OAK_Z=<m> \
BOTMAN_OAK_ROLL=<rad> BOTMAN_OAK_PITCH=<rad> BOTMAN_OAK_YAW=<rad> \
make deploy ROBOT=botman
```

This starts the base, lidar, SuperOdometry, camera/TF, Nav2, detection, adapter,
and media services. Complete the common [pre-flight checks](../operations/hardware-bringup.md).

Botman navigation uses the Scout-like responsive Nav2 tuning: a `0.4 m/s`
linear cap and `0.50 m` obstacle-inflation margin. The physical Bunker footprint
is still retained in Nav2; only the extra clearance around obstacles is reduced.

```bash
ssh botman 'cd /ssd/swarmdeck && docker compose --env-file .deploy/botman.env \
  -f deploy/compose/docker-compose.robot-botman.yml --profile "*" down'
```
