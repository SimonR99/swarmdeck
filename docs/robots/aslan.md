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

## IMU

SuperOdometry fuses the VectorNav VN-100 on `/vectornav/imu`. It is the quieter
part, and SuperOdometry is IMU-first (it falls back to IMU-only propagation
whenever a scan is late), so the IMU sets the floor on odometry quality. The
`os_lidar -> vectornav` rotation was measured 2026-09-01 with
`scripts/calibration/run_aslan_calibration.py`.

Four settings in `deploy/robots/aslan.env` select the IMU and must move
together (`ASLAN_IMU_TOPIC`, `ASLAN_START_VECTORNAV`, `ASLAN_SUPERODOM_CONFIG`,
`ASLAN_SUPERODOM_CALIB`); a config naming one IMU while the topic carries
another diverges with no error message. To fall back to the Ouster's internal
IMU, whose extrinsic is factory-exact, override all four.

The IMU's noise densities and `g_norm` were measured on this unit 2026-09-04
over a 600 s static log (59908 samples at 100.16 Hz, 0.19 % dropped). `g_norm`
must equal what this specific accelerometer reads when static, not true local
gravity and not another unit's value: Aslan's VN-100 reads 9.7666 where
Botman's reads 9.8719, and a wrong `g_norm` makes `imu_preintegration` reset
repeatedly and drift rather than fail. `scripts/aslan-build-overlay` refuses to
deploy a config that reverts to the upstream placeholder.

To re-measure, with the robot powered, level and completely still:

```bash
python3 scripts/calibration/measure_imu_static.py \
    --topic /vectornav/imu --duration 600
```

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
