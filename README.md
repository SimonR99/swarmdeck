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

## Tests

```bash
make test                                # backend pytest + frontend typecheck
bash tests/integration/test_sim_headless.sh   # headless Gazebo, ~60 s
```

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
