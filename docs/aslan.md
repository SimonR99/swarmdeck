# Aslan bring-up

Aslan is an AgileX Bunker with a Jetson AGX Orin. Its ROS 2 Humble stack lives
in the `bunker:dev` image and its MIST workspace is
`/ssd/mist_ws_ros2`. SwarmDeck mounts that workspace read-only; packages that
were missing from the existing install are built into
`/ssd/swarmdeck/.aslan_ros`.

## Interfaces

| Purpose | Interface |
|---|---|
| lidar / IMU | `/ouster/points`, `/ouster/imu` |
| SLAM pose | `/laser_odometry` (`map` -> `os_lidar`) |
| registered 3D scan | `/registered_scan` |
| camera | `/oak/rgb/image_raw/compressed` (OAK-D, `oak_camera` service) |
| video | `swarmdeck-aslan-media` → RTSP `aslan_0` on MediaMTX |
| detector | YOLOE sidecar (`duck_detector`), same catalog as Botman |
| Nav2 action | `/aslan_0/navigate_to_pose` |
| isolated Nav2 velocity | `/aslan_0/cmd_vel_nav` |
| physical driver velocity | `/cmd_vel`, relayed only by the adapter |

The adapter height-filters `/registered_scan` for the server-side 2D map and
also uploads a reduced XYZ cloud for the 3D view. Nav2 uses live Ouster scans
in rolling costmaps. It does not publish directly to the physical driver.

## Build

From Aslan's SwarmDeck checkout:

```bash
cd /ssd/swarmdeck
./scripts/aslan-build-overlay
BACKEND_HOST=192.168.1.223 \
  docker compose -f docker-compose.robot-aslan.yml build nav2 media duck_detector
```

The overlay build was completed successfully on 2026-08-06: `bunker_base`,
`ouster_ros`, and the patched `super_odometry` package all resolve from
`/aslan_overlay` inside `bunker:dev`.

## No-motion software test

This validates process wiring without opening the CAN base or starting sensors:

```bash
BACKEND_HOST=192.168.1.223 \
ASLAN_START_BASE=false \
ASLAN_START_LIDAR=false \
ASLAN_START_SLAM=false \
  docker compose -f docker-compose.robot-aslan.yml up -d

docker compose -f docker-compose.robot-aslan.yml ps
docker exec swarmdeck-aslan-nav2 \
  ros2 lifecycle get /aslan_0/bt_navigator
docker exec swarmdeck-aslan-nav2 \
  ros2 action list | grep /aslan_0/navigate_to_pose
docker exec swarmdeck-aslan-stack ros2 topic info -v /cmd_vel
```

Stop the probe with:

```bash
docker compose -f docker-compose.robot-aslan.yml down
```

## Hardware prerequisites

The last live audit found `can0` and `can1` down, no expected `can2` device,
and no reachable Ouster at the configured `192.168.50.165` address. Aslan then
dropped off `192.168.1.139` while its two SwarmDeck images were being built.
Do not start the hardware stack until the robot is reachable and these devices
have been restored.

Once the lidar is reachable, test mapping first with
`ASLAN_START_BASE=false`: verify `/laser_odometry`, `/registered_scan`, the 2D
map, and the 3D cloud in the UI. Default `docker compose up -d` also starts
`oak_camera`, `swarmdeck-aslan-media` (H.264 to MediaMTX path `aslan_0`),
the YOLOE `duck_detector` sidecar, and Nav2 (`/aslan_0/navigate_to_pose`).
Camera needs `depthai_ros_driver` in `/ssd/mist_ws_ros2` and the OAK on USB;
mapping, teleop and the planner still come up if the camera or detector
container fails. Starting the base or sending the first goal requires a
person beside Aslan with the physical e-stop available. Before that goal,
confirm `/cmd_vel` has only the SwarmDeck adapter as publisher and use a
small goal derived from the live pose.

## UI

The persistent server settings include enabled robot `aslan_0` as ROS 2, and
the UI displays it as `aslan`. It will appear in the fleet and 3D map as soon
as `swarmdeck-aslan-adapter` connects to the backend; an offline configured
robot is intentionally not fabricated in `GET /api/fleet`.
