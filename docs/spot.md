# Spot bring-up

Spot is a Boston Dynamics quadruped with an InDro Orin payload. SwarmDeck
talks to the payload's ROS 2 graph, not to Spot's internal computer. The
payload was not on the lab LAN when this was written (2026-08-12); the
files here are wired from the MIST launch copies on Botman/Aslan and from
a live look at the robot's own Wi-Fi.

## What is on, and what is not

| Thing | Status on 2026-08-12 |
|---|---|
| Spot body | **on** — AP `spot-BD-03210008` at ~84% from the operator laptop |
| Spot SDK address | `192.168.50.3` on the payload ethernet (from `rover_launch/config/spot.yaml`) |
| Payload SSH | `ssh spot` → `indro@orin.local` — **does not resolve** on mistmesh |
| Payload on `192.168.1.0/24` | no extra NVIDIA MAC besides scout/botman/aslan |
| `192.168.50.0/24` from lab robots | unreachable (not routed) |

Previous editor sessions on this laptop opened `/home/indro/mist_ws_ros2`,
`/home/indro/dino/DinoNav` and `/home/indro/vision_astronaut` over that SSH
alias, so the payload is an Ubuntu user `indro` with the same MIST ROS 2
workspace layout as Aslan. It is just not on this SSID right now.

Until `orin.local` answers, do not start `spot_driver`. Claiming a lease
from the wrong machine, or with nobody at the e-stop, is how this robot
walks.

## What supplies SLAM

The ROS 2 stack on the payload is **SuperOdometry** in mapping mode
(`localization_mode: false`), same package as the Bunkers, plus Clearpath
`spot_driver` for the body:

| Interface | Spot value | Source |
|---|---|---|
| Full robot launch | `ros2 launch rover_launch spot.launch.py` | `mist_ws_ros2` / `mist_ws` `rover_launch` |
| SLAM package | `super_odometry` / `os1_128.launch.py` | `src/localization/SuperOdom/` |
| Lidar / IMU input | `/ouster/points`, `/ouster/imu` | `config/superodom/os1_128.yaml` |
| SLAM odometry | `/laser_odometry` (`map` → `os_lidar`) | SuperOdometry LaserMapping |
| Registered scan | `/registered_scan` in `map` | SuperOdometry LaserMapping |
| Body command | `/cmd_vel` | `spot_driver`, `cmd_duration` 0.25 s |
| Spot computer | `192.168.50.3` | `rover_launch/config/spot.yaml` |
| Camera (optional) | `usb_cam` `/usb_cam/image_raw` | `spot_gnm.launch.py` / `spot_vnm.launch.py` |

There is no `OccupancyGrid`. The ROS 2 adapter therefore height-filters
`/registered_scan` and uploads XY returns for server-side raytracing, as
on Botman.

An older ROS 1 stack still exists in Scout's `/ssd/mist_ws`: LVI-SAM
(`params_lidar_vectornav_spot.launch`), `gbplanner`, ROS 1 `spot_driver`,
and a RosBuzz adapter. The payload workspace that recent sessions actually
opened is ROS 2 Humble (`mist_ws_ros2`). Use that.

DinoNav (`/home/indro/dino/DinoNav`, image `dinonav_ros2:dev` on Botman) is
visual topological navigation toward a goal image. It is not a Nav2
`NavigateToPose` server. SwarmDeck does not advertise `navigate` on this
robot until that is mapped or Nav2 is brought up the way Botman was.

## SwarmDeck-owned files

- `adapters/adapter_ros2/config/spot.yaml`: topics, frames, rates.
- `adapters/adapter_ros2/launch/spot.launch.py`: lidar / SLAM / driver split
  so mapping can start without claiming a lease.
- `docker-compose.robot-spot.yml`: lidar + slam + adapter by default;
  `--profile driver` starts `spot_driver`.
- `study/hardware_spot.yaml`: single-robot backend/map session.

## First SSH

The payload needs a lab-facing address. Either join it to mistmesh so
`orin.local` resolves, or SSH through Spot's AP (`192.168.80.3` on
`spot-BD-03210008`) and from there to the Orin on `192.168.50.0/24`. Then:

1. Confirm hostname, `ip -4 addr`, Docker images, `ROS_DOMAIN_ID`.
2. Clone this repo if missing (`~/swarmdeck` or `/ssd/swarmdeck`).
3. Set `SPOT_IMAGE`, `SPOT_WORKSPACE`, `SPOT_ROS_DOMAIN_ID` from what is
   actually there — the compose defaults are guesses from Aslan's layout.
4. Bring up **lidar + slam + adapter only**. Check `/laser_odometry` and
   `/registered_scan` before touching the driver.

```bash
cd server
.venv/bin/python -m swarmdeck_server --config ../study/hardware_spot.yaml
```

On the payload, once the three compose variables are confirmed:

```bash
BACKEND_HOST=192.168.1.223 \
  docker compose -f docker-compose.robot-spot.yml up -d --build
```

`--profile driver` is a later step. It needs an operator at the e-stop.
`spot_driver` auto-claim / auto-stand are false in MIST's `spot.yaml`;
leave them that way.

## Check, in this order

Same table as `docs/hardware-bringup.md` step 1–3: GUI appearance, pose
not stuck at origin, map patches, then deadman on `cmd_vel` with a hand
on the e-stop. Camera and `navigate_to` stay off until those pass.
