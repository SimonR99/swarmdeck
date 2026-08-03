# SwarmDeck

Multi-robot supervision stack: a simulated fleet of lidar-equipped Duckiebot-style
robots, a merged 2D map, and a browser GUI from which one operator supervises all of
them. The Gazebo model follows the DB21 layered differential-drive layout and adds a
top-deck 2D lidar as a SwarmDeck mapping payload.

**The backend has no ROS dependency.** Robots connect through a version-agnostic
[adapter contract](adapters/protocol/README.md), so ROS 2 robots, ROS 1 robots, and
Gazebo can coexist in one fleet.

See [`docs/`](docs/) for [architecture](docs/architecture.md),
[requirements](docs/requirements.md), and the [roadmap](docs/roadmap.md).

## Quick start — Docker (recommended)

Full stack (Gazebo Harmonic + SLAM + Nav2 + `adapter_sim` + backend + UI):

```bash
make docker-up          # or: docker compose --profile gazebo up --build -d
```

Open <http://localhost:5173>. Robots appear after Gazebo/SLAM come up (~45 s).
API is on <http://localhost:8080>. Stop with `make docker-down`.

Synthetic fleet only (no Gazebo/ROS — useful for UI/backend work):

```bash
make docker-up-mock     # or: docker compose --profile mock up --build -d
```

```bash
make docker-logs        # follow logs
make docker-test        # backend pytest in the server image
```

The map supports pan/zoom, fleet and selection centring, click-to-select markers, a
metric grid, trails, labels, sensor/footprint overlays, map revision metadata, and live
registration diagnostics from `GET /api/map/status`.

The live map distinguishes the path already travelled (solid) from Nav2's current
predicted path (dashed). The settings button persists the unattended-warning delay,
expected fleet size, adapter identities/endpoints, and perception controls in
`sessions/settings.json`; the fleet size is consumed on the next Gazebo/adapter start.

The simulation adapter also exposes a 5 Hz JPEG camera preview when MediaMTX/WHEP is not
installed, so camera frames remain visible during development. A portable RGB-only
rubber-duck detector publishes normalized boxes over the same adapter contract and the
camera panel draws them without using Gazebo entity IDs.

## Quick start — local (no Docker)

Needs Node.js and Python 3.10+ with `venv` (`sudo apt install python3-venv` on Debian/Ubuntu):

```bash
make install          # ui deps + server venv
make server           # terminal 1 — backend on :8080
make mock N=4         # terminal 2 — 4 synthetic robots
make ui               # terminal 3 — GUI on :5173
```

Open <http://localhost:5173>.

**GUI only, nothing else running:** <http://localhost:5173/?mock=1&robots=4> — the
frontend falls back to a built-in simulator, so UI work needs no backend at all.

## Quick start — Gazebo

```bash
cd swarmdeck_ros && colcon build --symlink-install && source install/setup.bash
ros2 launch swarmdeck_bringup session.launch.py config:=study/4robot.yaml
# then, separately:
make server
python3 adapters/adapter_sim/adapter_sim.py   # count comes from persistent settings
make ui
```

The seeded 24 m indoor world contains five procedural yellow rubber ducks, tables,
chairs, paintings, and plants. All objects are ordinary SDF geometry with collisions
where appropriate, so lidar mapping and camera perception see the same environment.

## Layout

| Path | What |
|---|---|
| `adapters/protocol/` | **The contract of record.** Changing it is deliberate and versioned. |
| `adapters/adapter_*/` | One per robot type: `mock`, `sim`, `ros2`, `ros1`, `spot` |
| `swarmdeck_ros/src/` | Gazebo world, robot model, SLAM, Nav2, bringup |
| `server/` | FastAPI backend — fleet registry, map merge, events, recording |
| `ui/` | Svelte 5 + Tailwind frontend |
| `study/` | Session configs (`1robot.yaml`, `2robot.yaml`, `4robot.yaml`) |
| `sessions/` | Recorded output, one directory per session |
| `docker/` | Server, UI, and Gazebo/ROS images; compose file at repo root |

## Map merging

Each robot runs its own 2D SLAM, so every map is in that robot's own frame with the
origin wherever it started. `mapsvc` merges them:

- **`static`** — transforms come from configured start poses. Always available.
- **`auto`** — transforms estimated by grid registration (signed FFT cross-correlation over
  a coarse-to-fine yaw sweep, numpy only). Verified against Gazebo ground truth at **7.8 cm**
  with two robots at unknown relative poses; 52/52 correct at 0.1–3 cm on a synthetic
  all-headings sweep.

Four rejection tests guard the result — `score`, `ratio` (rival translation), `yaw_ratio`
(rival rotation), and `support` (shared known area) — so a repetitive building yields "not
confident" rather than a confident-but-wrong merge, and the merge keeps `static` transforms.
Registration needs the robots to have seen the same places; `GET /api/map/status` reports all
four metrics, and the map view explains which test refused.

This registration is automatic whenever `merge_mode: auto`, but it is **not** a shared
multi-robot pose graph. Each robot closes its own loops first; the map service then aligns
the corrected occupancy grids, so one robot's observations never correct another's drift.
[`docs/collaborative-slam.md`](docs/collaborative-slam.md) explains the limits of that and
the migration path to true collaborative SLAM.

### Per-robot SLAM backend

`slam_backend:=toolbox` (default) runs SLAM Toolbox on a single-ring lidar. For robots with
a 3D point cloud and a camera, `slam_backend:=rtabmap` runs RTAB-Map instead — ICP over the
full cloud plus appearance-based loop closure — publishing the same per-robot
`OccupancyGrid`, so nothing downstream changes. Set `fleet.lidar_rings` to an **odd** value
(the spawner refuses even counts, which leave no ring at zero elevation) to get a point
cloud. This path is implemented but not yet validated in Gazebo.

## Tests

```bash
make test                                # backend pytest + frontend typecheck
bash tests/integration/test_sim_headless.sh   # headless Gazebo, ~60 s
```

Manual stack for debugging:

```bash
bash tests/integration/run_stack.sh 2      # gazebo + 2 robots + SLAM, headless
python3 swarmdeck_ros/src/swarmdeck_sim/scenario/explore.py --robots 2 --seconds 240
bash tests/integration/stop_stack.sh
```

`explore.py` is reactive obstacle-avoiding wandering. Do **not** drive the robots
open-loop: a robot jammed against a wall spins its wheels, odometry integrates motion
that never happened, and SLAM (which uses odometry as its prior) produces a useless map.

## Adding a robot type

Write one adapter. Touch nothing else (NFR-9).

1. Connect to `ws://backend:8080/adapter`, send `hello` with your capabilities.
2. Publish `robot_state` at 5 Hz.
3. Accept `navigate_to` / `cancel_goal` / `stop`, mapping them to whatever your robot
   actually uses — Nav2, `move_base`, or a vendor SDK.
4. Optionally push occupancy grids and push camera to MediaMTX over RTSP.

`adapters/adapter_mock/mock_adapter.py` is a complete ~200-line reference with no ROS.

By default, goals and robot state use the robot's local navigation-map frame on the
adapter socket. The backend converts them to the merged-map frame seen by the GUI;
synthetic adapters may explicitly declare that they already use the merged frame.

## Prerequisites

Present on the dev machine: ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_*`,
`rosbag2_storage_mcap`, `cv_bridge`.

Still to install for the full simulation path:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan \
                 ros-jazzy-nav2-map-server
```

For `slam_backend:=rtabmap` (already in the Gazebo image):

```bash
sudo apt install ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-util
```

MediaMTX is still needed for video. `multirobot_map_merge` is deliberately *not* used: it is
a ROS node, and the backend is ROS-free by design — see `docs/architecture.md` §1.

**ROS 1 note.** Noetic is EOL and cannot install on Ubuntu 24.04, and
`ros-jazzy-ros1-bridge` does not exist. A ROS 1 robot runs `adapter_ros1` in its own
Noetic container, on the robot or a companion machine — which the adapter contract
requires anyway.
