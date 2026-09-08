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

## VectorNav odometry

Deployment uses the VectorNav VN-100 on `/vectornav/imu`. The profile and Compose
select its driver, `aslan_superodom.yaml`, and `aslan_superodom_calibration.yaml`
together. The Ouster IMU remains an explicit fallback: override all four
`ASLAN_IMU_TOPIC`, `ASLAN_START_VECTORNAV`, `ASLAN_SUPERODOM_CONFIG`, and
`ASLAN_SUPERODOM_CALIB` settings together.

The VN-100 occupies the same baseplate position as Botman's. The existing TF
accounts for Aslan's lower lidar mount, and the estimator uses Aslan's measured
rotation (approximately -90.65 degrees yaw). Its unit-specific calibration was
measured on 2026-09-04: `g_norm=9.7666`, `acc_n=1.422605e-3`, and
`gyr_n=6.969057e-5`. Botman's VN-100 reads a different static acceleration norm
(9.8719), so matching the mount does not justify copying its bias calibration.
These validated values had remained in `worktree-aslan-vn100` while deployments
from the main checkout still selected the Ouster IMU.

On 2026-09-08, the live Ouster-configured IMU preintegration process had aborted
on an invalid assertion that rotating acceleration must change its X component.
A zero or rotation-invariant component is legitimate. The SwarmDeck overlay
patch removes that assertion; the SLAM health check now also requires the IMU
preintegration process, so surviving lidar nodes cannot hide this crash.

After deploying the fix on 2026-09-08, a passive 120-second stationary capture
received 11,927 IMU samples and 1,151 odometry samples. Excluding the first
30 seconds, fitted drift was 0.0034 m/min in position and 0.0149 degrees/min
in yaw. The median IMU rate was 100.15 Hz, odometry averaged 9.68 Hz, and no
IMU timestamps repeated or went backwards; the largest observed IMU gap was
91 ms. Startup included one bias reset; occasional stationary iSAM2
underconstraint warnings remained, without further bias resets or scan
synchronization failures in the subsequent log check. This short static
capture does not establish accuracy while driving.

Changing the IMU selection requires restarting SLAM and begins a new local map.
The previous calibration's static drift measurements are not a guarantee of
performance while driving; check live IMU rate, timestamp continuity, and scan
synchronization after deployment.

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
