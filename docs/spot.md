# Spot bring-up

Spot is a Boston Dynamics quadruped with an InDro Orin payload (hostname
`orin`, user `indro`, `192.168.1.192` on mistmesh and `192.168.50.192` on
the payload ethernet). SwarmDeck talks to that payload's ROS 2 graph, not
to Spot's internal computer.

## Live audit (2026-08-12)

| Thing | Status |
|---|---|
| Payload SSH | `ssh spot` → `indro@orin.local` (`192.168.1.192`), JetPack R36.2 |
| Spot body | `192.168.50.3` pings from the Orin |
| Ouster | `192.168.50.165` pings; `driver_ouster.yaml` matches |
| VectorNav | `/dev/vectornav` → `ttyUSB0` |
| Native ROS | none — Humble lives in `spot:dev` |
| Workspace | `/home/indro/mist_ws_ros2` (no `/ssd`) |

`spot:dev`'s `/tmp/setup.sh` exports `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
SwarmDeck compose replaces that entrypoint and pins Fast-RTPS so the
adapter can see the same graph.

## What supplies SLAM

**LIO-SAM**, not SuperOdometry. SuperOdom is in the tree but
`COLCON_IGNORE`'d and is not in the install space. The launch that is
actually built is `rover_launch/launch/full_stack/spot_lio_sam.launch.py`.

| Interface | Spot value | Source |
|---|---|---|
| Lidar | `/ouster/points` from `192.168.50.165` | `config/drivers/driver_ouster.yaml` |
| IMU | `/vectornav/imu` | `config/drivers/driver_imu.yaml` |
| SLAM pose | `/lio_sam/mapping/odometry`; TF `map` → `odom_link` → `lidar_link` | LIO-SAM `mapOptimization` |
| Registered scan | `/lio_sam/mapping/cloud_registered` | LIO-SAM `mapOptimization` |
| Body command | `/cmd_vel` | `spot_driver` to `192.168.50.3` |

There is no `OccupancyGrid`. The adapter height-filters the registered
cloud for the server-side 2D map, same as Botman.

## What people actually launch for navigation

There is no Nav2 `NavigateToPose` on this payload. History, tmux, and the
full-stack launch show three stacks, none of which map onto SwarmDeck
`navigate_to`:

| Stack | Where | How it is used |
|---|---|---|
| **TARE planner** | `tare_planner`, `explore_spot.launch` | Included in `spot_lio_sam.launch.py` — the launch they actually run. Autonomous exploration, not click-to-pose. |
| **spot_high_level_controller** | `rover_launch/spot_high_level_controller.py` | Same full stack. Modes AUTONOMY_A / WAYPOINT_B / TELEOP_C. Calls `/power_on`, `/stand`, `/sit`, and relays TARE `/way_point` onto Spot's `Trajectory` action. |
| **DinoNav** | `/home/indro/dino/DinoNav`, image `dinonav_ros2:dev` | Visual topological nav via `tmux_vnm/spot_tmux_pipeline.sh`. Image-goal, not a map click. |
| **SafeGNM / visualnav** | `visualnav:dev` | `docker_setup.sh` option 2, "Visual_Navigation_spot". |
| **GraphNav** | `spot_msgs` services exist | Not seen launched in bash history. |

SwarmDeck click-to-pose uses the same `/trajectory` action as
`spot_high_level_controller`: the adapter transforms the map-frame goal
into `body` (the only frame Clearpath accepts) and sends
`spot_msgs/Trajectory`. Cancel also calls `/stop`, because that ROS 2
action server does not honour preempt. Point nav and the joystick need
`--profile driver`, a claimed lease, and a standing robot. TARE and
DinoNav stay unwired — they are exploration / image-goal, not a map click.

## SwarmDeck-owned files

- `adapters/adapter_ros2/config/spot.yaml`
- `adapters/adapter_ros2/launch/spot.launch.py` — lidar / LIO-SAM / driver split
- `docker-compose.robot-spot.yml` — lidar + slam + adapter by default;
  `--profile driver` starts `spot_driver`
- `study/hardware_spot.yaml`

## Start

Backend (already the usual operator session, or):

```bash
cd server
.venv/bin/python -m swarmdeck_server --config ../study/hardware_spot.yaml
```

On the payload, after a one-time `websockets` user install into
`/home/indro/swarmdeck/.spot_pip`:

```bash
BACKEND_HOST=192.168.1.223 \
  docker compose -f docker-compose.robot-spot.yml up -d
```

Verified 2026-08-12 against the live operator session: `spot_0` is online
with `map`, pose from `map` → `lidar_link`, and local-grid seq advancing.
LIO-SAM runs in `spot_lio_sam:dev` (Debian GTSAM); lidar and the adapter
use `spot:dev`. Mixing those for imuPreintegration dies on an undefined
`ISAM2::update` symbol.

`--profile driver` is no longer required: `spot_driver` starts with the
rest of the stack. Claim / Stand stay GUI actions (`auto_claim` /
`auto_stand` remain false). The launch overrides `start_estop` to true
so claim works without a tablet holding motor-power authority.

## Check, in this order

GUI appearance, pose not stuck at origin (`map` → `lidar_link`), map
patches from `/lio_sam/mapping/cloud_registered`, then — only with the
driver profile, a claimed lease, a standing robot, and a hand on the
e-stop — Point nav (`/trajectory`) and deadman on `cmd_vel`. Camera is
the payload Intel RealSense D435i color stream (`/d435/color/image_raw`),
not Spot's body fisheyes. Depth is on the same device (Z16 `/dev/video0`)
but `spot:dev` has no `realsense2_camera`, so the GUI shows color only.

The YOLOE sidecar (`duck_detector` in compose, JetPack 6) is the same
catalog as the rest of the fleet: rubber duck, wooden block, disc cone,
filament spool, pool noodle. Boxes stay in the image until D435 depth is
a ROS topic with TF to `map`.
