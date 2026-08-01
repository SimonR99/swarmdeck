# SwarmDeck

Multi-robot supervision stack: a simulated fleet of lidar-equipped robots, a merged 2D
map, and a browser GUI from which one operator supervises all of them.

**The backend has no ROS dependency.** Robots connect through a version-agnostic
[adapter contract](adapters/protocol/README.md), so ROS 2 robots, ROS 1 robots, and
Gazebo can coexist in one fleet.

See [`docs/`](docs/) for [architecture](docs/architecture.md),
[requirements](docs/requirements.md), and the [roadmap](docs/roadmap.md).

## Quick start — no ROS, no Gazebo

The fastest way to see the whole GUI working:

```bash
make install          # ui deps + server venv
make server           # terminal 1 — backend on :8080
make mock N=4         # terminal 2 — 4 synthetic robots
make ui               # terminal 3 — GUI on :5173
```

Open <http://localhost:5173>. Four robots wander, build a shared map, raise alerts and
report detections. Click a robot card to select it, then tap the map to send a goal.

**GUI only, nothing else running:** <http://localhost:5173/?mock=1&robots=4> — the
frontend falls back to a built-in simulator, so UI work needs no backend at all.

## Quick start — Gazebo

```bash
cd swarmdeck_ros && colcon build --symlink-install && source install/setup.bash
ros2 launch swarmdeck_bringup session.launch.py config:=study/4robot.yaml
# then, separately:
make server
python3 adapters/adapter_sim/adapter_sim.py --robots 4
make ui
```

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

## Map merging

Each robot runs its own 2D SLAM, so every map is in that robot's own frame with the
origin wherever it started. `mapsvc` merges them:

- **`static`** — transforms come from configured start poses. Always available.
- **`auto`** — transforms estimated by grid registration (FFT cross-correlation over a
  yaw sweep, numpy only). Verified against Gazebo ground truth at **7.8 cm** with two
  robots at unknown relative poses.

A ratio test rejects ambiguous alignments, so a repetitive building yields "not
confident" rather than a confident-but-wrong merge; the merge then keeps `static`
transforms. Registration needs the robots to have seen the same places — check
`GET /api/map/status` for `score`, `ratio` and `overlap`.

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

## Prerequisites

Present on the dev machine: ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_*`,
`rosbag2_storage_mcap`, `cv_bridge`.

Still to install for the full simulation path:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan \
                 ros-jazzy-nav2-map-server
```

Source build: [`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)
(`multirobot_map_merge`) — not packaged for Jazzy. Plus MediaMTX for video.

**ROS 1 note.** Noetic is EOL and cannot install on Ubuntu 24.04, and
`ros-jazzy-ros1-bridge` does not exist. A ROS 1 robot runs `adapter_ros1` in its own
Noetic container, on the robot or a companion machine — which the adapter contract
requires anyway.
