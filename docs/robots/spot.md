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
only the trajectory timeout. `control_mode: differential` makes the adapter
execute map goals as rotate, straight-drive, and optional final-rotate phases;
it never requests body-y translation. Keep the SDK's lateral (`linear_y`)
mobility limit slightly above zero (the profile uses `0.05 m/s`), because Spot's
gait and obstacle planner reports even straight trajectories as blocked with an
exactly zero lateral envelope. This allowance does not make the adapter issue
holonomic targets. A cancel sends zero velocity immediately and dispatches the
driver's SDK stop in the background, allowing teleoperation to take over
without waiting for the stop service response.

The driver, lidar, SLAM, and adapter containers use Fast DDS over host-loopback
UDP. Do not remove `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` from the Spot Compose
file: shared-memory port collisions can leave ROS endpoints discoverable while
blocking VectorNav IMU and static TF delivery. Camera/media traffic continues to
use the default shared-memory transport.

**Known limitation: `map -> body` (the adapter's `base_frame`, feeding both the
UI's live position and `pose7`'s keyframe upload) only updates at
`lio_sam_mapOptimization`'s keyframe rate, ~5 Hz**, because `body`'s parent
`lidar_link` is corrected at that rate, not the ~200 Hz IMU rate. LIO-SAM does
publish a faster branch — `odom_link -> imu_body` (`lio_sam.yaml`'s
`baselinkFrame`) via `lio_sam_imuPreintegration` — but it is not a drop-in
substitute: live measurement on 2026-08-31 showed its Z climbing continuously
(~0.03 m/s, unbounded) relative to the corrected `body` branch, while X/Y
tracked closely. Since `pose7` feeds the full 6DOF pose into the collaborative
pose-graph optimizer (see `slam/swarmdeck_slam/types.py`), routing it through
that branch would inject drifting Z into every keyframe upload. `base_frame`
stays on `body` until that Z drift is fixed at the source (LIO-SAM/IMU tuning)
or split from the fast branch's X/Y. Also note: `lio_sam_imuPreintegration`
does its own internal `imu_body -> lidar_link` TF lookup, so the static
`lidar_link -> imu_body` transform in `spot.launch.py` cannot be removed even
though a dynamic transform for the same child frame also exists — removing it
makes `lio_sam_imuPreintegration` fail with "extrapolation into the past"
errors.

The same ~5 Hz cap used to make differential-drive navigation look like it
never stopped: `_continue_diff_trajectory` re-checks remaining map-frame error
after every rotate/drive phase to decide whether another phase is needed, and
a stale (pre-phase) read made it issue one it didn't need. Fixed by
`trajectory.progress_frame: body_fast` — that re-check never touches Z (only
`dx`/`dy`/yaw), so it's safe to point at the fast branch even though
`base_frame` isn't. `trajectory.frame` (what's sent to Spot's driver, which
rejects any frame_id but `body`) is untouched; `body_fast`
(`spot.launch.py`'s `tf_imubody_bodyfast`) is only ever read internally by
`_goal_in_body`/`_progress_frame`, never sent to the driver.

```bash
ssh spot 'cd /home/indro/swarmdeck && docker compose --env-file .deploy/spot.env \
  -f deploy/compose/docker-compose.robot-spot.yml --profile "*" down'
```
