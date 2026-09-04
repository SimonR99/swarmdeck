# Botman

Botman is an AgileX Bunker on ROS 2 Humble with Ouster OS-0-64,
VectorNav VN-100, SuperOdometry, Nav2, and an OAK-D Pro RGB-D camera.

Prerequisites on the robot:

- SSH at `botman@192.168.1.49`; checkout `/ssd/swarmdeck`.
- Read-only MIST workspace `/ssd/mist_ws` and image `bunker_super_odom:dev`.
- ROS domain 17.
- Measured transform from `os_lidar` to `oak-d-base-frame`.
- Measured calibration for `os_lidar` -> `vectornav` (kept in `botman_superodom_calibration.yaml`).

```bash
make deploy ROBOT=botman                 # full stack (base driver, sensing, SLAM, Nav2)
```

The Bunker CAN driver (`robot_stack`) starts up by default alongside sensing and Nav2.
Complete the common [pre-flight checks](../operations/hardware-bringup.md) with a
person at the physical e-stop before commanding motion.

Botman navigation uses the Scout-like responsive Nav2 tuning: a `0.4 m/s`
linear cap and `0.50 m` obstacle-inflation margin. The physical Bunker footprint
is still retained in Nav2; only the extra clearance around obstacles is reduced.

```bash
ssh botman 'cd /ssd/swarmdeck && docker compose --env-file .deploy/botman.env \
  -f deploy/compose/docker-compose.robot-botman.yml --profile "*" down'
```
