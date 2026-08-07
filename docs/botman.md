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
| Camera | disabled: no `/dev/video0` | Live hardware check; old bags contain `/usb_cam/image_compressed` |

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
- `docker-compose.robot-botman.yml`: calls the existing MIST launch package and
  starts a separate SwarmDeck adapter. Both containers use host networking and
  host IPC because Fast DDS transports same-host samples through shared memory.
- `docker/Dockerfile.robot-ros2`: generic Humble adapter runtime; contains no
  MIST packages or robot configuration.
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
read-only. It does place the Bunker driver into commanded mode, so first
bring-up must be supervised with the physical e-stop available even though no
motion command is sent automatically.

Stop the integration with:

```bash
docker compose -f docker-compose.robot-botman.yml down
```

Navigation and battery are intentionally not advertised. DinoNav uses waypoint
topics rather than Nav2's `NavigateToPose`, and the Bunker driver publishes a
custom voltage field rather than `sensor_msgs/BatteryState.percentage`.

## Live verification (2026-08-06)

Botman was idle before launch. The SwarmDeck-owned Compose stack was then built
on the robot and verified with approximately 10 Hz `/ouster/points`, 100 Hz
`/ouster/imu`, and 10 Hz `/registered_scan` plus `/laser_odometry`. The backend
receives Botman's updating pose, generates its 2D local map, and includes
`botman_0` in the merged 3D cloud. The adapter advertises only `map` and `estop`.

The upstream full launch also attempts to start an absent USB camera and a
VectorNav process; those optional processes fail on this hardware. They are not
inputs to this configuration: SuperOdometry uses the Ouster IMU and lidar.
