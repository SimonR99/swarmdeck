# Spot

Spot uses a Jetson AGX Orin ROS 2 payload with the Clearpath Spot driver,
Ouster lidar, VectorNav/LIO-SAM, RealSense D435, and the ROS 2 adapter.

Prerequisites on the payload:

- SSH alias `spot`; checkout `/home/indro/swarmdeck`.
- Read-only workspace `/home/indro/mist_ws_ros2`.
- Prebuilt `spot:dev` and `spot_lio_sam:dev` images.
- ROS domain 0 and working Spot credentials/configuration in the installed
  driver workspace.

```bash
make deploy ROBOT=spot
```

Claim, release, sit, and stand are exposed as body commands. Automatic claim and
stand remain disabled; confirm the physical e-stop state before using them.
Complete the common [pre-flight checks](../operations/hardware-bringup.md).

Click-to-navigate goals preserve Spot's heading because a map click specifies a
point, not an orientation. Before each trajectory, the adapter applies the
configured `0.25 m/s` translation and `0.5 rad/s` rotation limits through the
driver's `/max_velocity` service. Change `trajectory.velocity_limit` in
`adapters/adapter_ros2/config/spot.yaml` to tune those limits; `duration_s` is
only the trajectory timeout.

```bash
ssh spot 'cd /home/indro/swarmdeck && docker compose --env-file .deploy/spot.env \
  -f deploy/compose/docker-compose.robot-spot.yml --profile "*" down'
```
