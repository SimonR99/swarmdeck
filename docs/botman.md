# Botman bring-up

Botman is an AgileX Bunker running ROS 2 Humble in Docker. SwarmDeck does not
modify its `/ssd/mist_ws` workspace: the workspace is mounted read-only and its
existing launch package is invoked as a dependency.

## What supplies SLAM

Botman uses **SuperOdometry** in mapping/SLAM mode (`localization_mode: false`),
fed by the Ouster lidar and IMU:

| Interface | Botman value | Source in `/ssd/mist_ws` |
|---|---|---|
| Full robot launch | `ros2 launch rover_launch bunker_gnm.launch.py` | `src/control/rover_launch/launch/bunker_gnm.launch.py` |
| SLAM package | `super_odometry` / `os1_128.launch.py` | `src/localization/SuperOdom/` |
| Lidar / IMU input | `/ouster/points`, `/ouster/imu` | `src/control/rover_launch/config/superodom/os1_128.yaml` |
| SLAM odometry | `/laser_odometry` | `LaserMapping/laserMapping.cpp`; also confirmed by `bag5_/metadata.yaml` |
| High-rate estimate | `/state_estimation` (declared, no live samples) | `ImuPreintegration/imuPreintegration.cpp` |
| Registered scan | `/registered_scan` in `map` | `LaserMapping/laserMapping.cpp` |
| SLAM TF | `map -> os_lidar` (intended, absent live) | `ImuPreintegration/imuPreintegration.cpp` |
| Base command | `/cmd_vel` | `bunker_base/include/bunker_base/bunker_messenger.hpp` |
| Camera | OAK-D Pro RGB, `/oak/rgb/image_raw/compressed` | SwarmDeck-owned `oak_camera` service using MIST's read-only DepthAI install |

There is no `OccupancyGrid`. The ROS 2 adapter therefore height-filters
`/registered_scan` and uploads XY returns for server-side raytracing, while a
separate voxel-reduced XYZ product feeds the 3D view. The live stack does not
publish its source-declared `map -> os_lidar` TF, so the adapter reads the same
map-frame sensor pose directly from `/laser_odometry`. The driver's independent
`odom -> base_link` tree remains deliberately unjoined.

Returns inside the configured 0.65 m footprint are discarded before upload so
the rearward lidar rays cannot paint the chassis into the persistent server
grid. If a grid was accumulated before that filter was active, select Botman in
the map's **Layers → Map data** menu and use **Reset Botman** while it is not
navigating. This resets only SwarmDeck's accumulator; SuperOdometry and session
recording continue uninterrupted.

Botman's Bunker chassis is 1023 x 778 mm, so the UI uses a conservative 0.65 m
circumscribed radius. See the [AgileX Bunker specifications](https://docs.trossenrobotics.com/agilex_bunker_docs/specifications.html).

## SwarmDeck-owned files

- `adapters/adapter_ros2/config/bunker.yaml`: topics, frames, rates and capabilities.
- `adapters/media/config/botman_oak_rgb.yaml`: 15 Hz, device-encoded MJPEG
  profile for the OAK-D Pro operator stream.
- `adapters/media/ros2_rtsp.py`: leaky-queue JPEG-to-H.264 publisher for the
  production RTSP/MediaMTX/WebRTC path.
- `adapters/perception/yoloe_server.py`: GPU YOLOE-26n inference sidecar shared
  with every other robot type.
- `docker/Dockerfile.duck-detector`: parameterized CPU/JetPack detector image.
- `docker-compose.robot-botman.yml`: calls the existing MIST launch package and
  starts separate SwarmDeck adapter, detector and OAK camera services. The MIST
  workspace remains read-only; host networking/IPC carries same-host ROS 2 samples.
- `docker/Dockerfile.robot-ros2`: generic Humble adapter runtime; contains no
  MIST packages or robot configuration.
- `swarmdeck_ros/src/swarmdeck_nav`: shared Nav2 launch machinery plus Botman's
  Humble parameters and the missing odometry-to-TF bridge.
- `docker/Dockerfile.robot-nav2`: isolated Humble Nav2 runtime. It installs no
  packages into the robot's MIST workspace.
- `study/hardware_botman.yaml`: single-robot backend/map session.

## Start

Start the backend on the operator machine:

```bash
cd server
.venv/bin/python -m swarmdeck_server --config ../study/hardware_botman.yaml
```

Then, from the SwarmDeck checkout on Botman:

```bash
BACKEND_HOST=192.168.1.223 \
  docker compose -f docker-compose.robot-botman.yml up -d --build
```

This starts only SwarmDeck-owned containers. The MIST workspace remains
read-only. The first detector start downloads about 253 MB of model/prompt assets into
the `duck_detector_models` Docker volume. It does place the Bunker driver into commanded mode, so first
bring-up must be supervised with the physical e-stop available even though no
motion command is sent automatically.

Stop the integration with:

```bash
docker compose -f docker-compose.robot-botman.yml down
```

Battery is intentionally not advertised: the Bunker driver publishes a custom
voltage field rather than `sensor_msgs/BatteryState.percentage`.

## Navigation: reuse the repository's Nav2 stack

The robot audit found two planners, but neither is a live pose-goal stack:

- DinoNav follows a pre-recorded image sequence toward a goal image index
  selected at launch. It cannot accept the GUI's runtime `(x, y)` goal.
- TARE Planner is built in `/ssd/mist_ws`, but it autonomously generates
  exploration waypoints. There is no matching local controller in the robot
  workspace and it does not expose `NavigateToPose`.

Botman therefore reuses `swarmdeck_nav`, the Nav2 stack already used by the
simulator. `botman.launch.py` applies a Humble-specific configuration rather
than duplicating the planner implementation:

| Nav2 input/output | Botman interface |
|---|---|
| pose and controller odometry | `/laser_odometry` (`map` -> `os_lidar`) |
| obstacle scan | `/ouster/scan`, 10 Hz, frame `os_lidar` |
| goal action | `/botman_0/navigate_to_pose` |
| isolated velocity output | `/botman_0/cmd_vel_nav` |
| real driver command | `/cmd_vel`, published only by the adapter |

SuperOdometry carries the correct pose in Odometry but does not broadcast its
declared `map -> os_lidar` transform. `odom_to_tf` republishes that exact pose
as a planar TF edge; it does not estimate or integrate another pose.

There is still no ROS `OccupancyGrid`. Nav2 therefore uses 8 m and 40 m rolling
costmaps populated from the live Ouster scan, with Botman's 0.65 m radius and
0.2 m/s / 0.2 rad/s limits. This provides obstacle-aware local and global path
planning inside the 40 m window, but it is not a persistent whole-building map:
a goal outside the rolling window will be rejected and obstacles outside live
lidar coverage are not retained. The footprint is conservatively centred on
`os_lidar`; the live TF tree contains no calibrated `os_lidar` -> `base_link`
edge, so that calibration remains a prerequisite for tightening the footprint.

Nav2 never publishes the real `/cmd_vel`. Its controller feeds the velocity
smoother, which publishes `/botman_0/cmd_vel_nav`; the adapter relays that topic
only while its action goal is active. Teleop explicitly cancels the action, and
cancel/e-stop immediately stop relaying autonomous velocity.

The `nav2` service is part of the normal Botman Compose graph and the adapter
waits for its lifecycle-managed action server to become healthy. Starting the
stack creates an active planner but sends no goal and no motion by itself.

### No-motion checks

After building, perform these before any goal:

```bash
docker compose -f docker-compose.robot-botman.yml ps
docker exec swarmdeck-botman-nav2 ros2 lifecycle get /botman_0/bt_navigator
docker exec swarmdeck-botman-nav2 ros2 action list | grep navigate_to_pose
docker exec swarmdeck-botman-stack ros2 topic info -v /cmd_vel
```

The navigator must be `active`; the action must be
`/botman_0/navigate_to_pose`; and `/cmd_vel` must have only the adapter as a
publisher. Also confirm `/botman_0/cmd_vel_nav` has Nav2 as publisher and the
adapter as subscriber.

### First live navigate_to test — human operator, not autonomous

1. Confirm nobody else is using Botman: `ssh botman 'who; docker ps'`.
2. Camera-check the space immediately beforehand; keep a hand near the
   physical e-stop for the whole test.
3. Run all no-motion checks above and inspect the Nav2 logs for costmap or TF
   errors before enabling the operator controls.
4. Confirm the adapter advertises `navigate`:
   `curl -s http://<backend-host>:8080/api/fleet | jq '.robots.botman_0.capabilities'`.
5. Send ONE small (0.5-1.5 m) goal computed from the robot's *live* pose, the
   same protocol used for tars's first test (see the ROS 1 adapter's git
   history) — small distance first, watch `/cmd_vel` and the reported pose,
   be ready to hit the physical e-stop.
6. Test a goal behind the current heading. Nav2 should rotate rather than show
   the old DinoNav `atan(dy/dx)` stall.
7. Test teleop preempting an active goal and then cancel/e-stop. Confirm the
   action becomes cancelled and `/cmd_vel` reaches zero each time.
8. When done, use `docker compose -f docker-compose.robot-botman.yml down` so
   the robot is left in the same state as before the test.

## Live verification (2026-08-06)

Botman was idle before launch. The SwarmDeck-owned Compose stack was then built
on the robot and verified with approximately 10 Hz `/ouster/points`, 100 Hz
`/ouster/imu`, and 10 Hz `/registered_scan` plus `/laser_odometry`. The backend
receives Botman's updating pose, generates its 2D local map, and includes
`botman_0` in the merged 3D cloud.

The upstream full launch starts `usb_cam` for `/dev/video0`, but Botman's camera
is a DepthAI device and intentionally exposes no V4L2 node. Its commented OAK
launch is replaced by SwarmDeck's independent `oak_camera` Compose service;
`/ssd/mist_ws` is still mounted read-only. The unrelated optional VectorNav
process may fail and is not an input to this configuration: SuperOdometry uses
the Ouster IMU and lidar.

The camera was re-checked live after an unplug/replug. USB enumerated an OAK-D
Pro (MXID `1844301041D4AB0F00`) at SuperSpeed. A temporary driver probe produced
RGB frames, and the final SwarmDeck profile produced JPEG
`sensor_msgs/CompressedImage` samples on `/oak/rgb/image_raw/compressed` at a
stable 15 Hz. The persistent `swarmdeck-botman-oak` container was then started
without restarting the robot stack or changing `/cmd_vel` ownership.

**Production video is live.** `swarmdeck-botman-media` subscribes to the same
JPEG topic, encodes constrained-baseline H.264 at 1280x720, 15 fps and 1.2 Mbps,
and pushes RTSP/TCP to MediaMTX path `botman_0`. An independent RTSP probe read
that exact profile. A Chrome WHEP probe then received HTTP 201, established its
peer connection, exposed a live video track and decoded frames. The adapter's
5 Hz `/api/camera/botman_0` JPEG upload remains active as automatic fallback.

**The Nav2 addition in this repository has not been deployed or motion-tested
on Botman.** Its inputs were verified read-only on the live graph and its local
configuration/tests pass. Building the new image, validating lifecycle/TF/
costmaps, and sending the first goal remain supervised hardware steps. No
`drive` or `navigate_to` command was sent during this work.
