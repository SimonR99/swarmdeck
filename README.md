# SwarmDeck

Multi-robot supervision stack: a simulated fleet of lidar-equipped Duckiebot-style
robots, a merged 2D map, and a browser GUI from which one operator supervises all of
them. The Gazebo model follows the DB21 layered differential-drive layout and adds a
top-deck 2D lidar as a SwarmDeck mapping payload.

**The backend has no ROS dependency.** Robots connect through a version-agnostic
[adapter contract](adapters/protocol/README.md), so ROS 2 robots, ROS 1 robots, and
Gazebo can coexist in one fleet.

**For real robots**, `adapters/adapter_ros2/` (`rclpy`/Nav2) and `adapters/adapter_ros1/`
(`rospy`/`move_base`) are the hardware adapters — same protocol, same config schema, pick
whichever matches the robot's own ROS generation. [`docs/hardware-bringup.md`](docs/hardware-bringup.md)
is the ordered test plan. Neither has **been run against physical hardware** — the code
is unit-tested and the sim-only assumptions are now parameters, but every topic name,
QoS/latching choice and timeout is a hypothesis until a robot proves it.
[`docs/hardware-readiness.md`](docs/hardware-readiness.md) audits what is already correct
and what still is not. [`docs/fleet-status.md`](docs/fleet-status.md) tracks the actual
physical fleet — per-robot hardware, network and ROS state, and how far terrain prep has
gotten on each.

See [`docs/`](docs/) for [architecture](docs/architecture.md),
[requirements](docs/requirements.md), and the [roadmap](docs/roadmap.md).
Physical Bunker procedures are documented separately for
[Botman](docs/botman.md) and [Aslan](docs/aslan.md).

## Quick start — Docker (recommended)

Full stack (Gazebo Harmonic + SLAM + Nav2 + `adapter_sim` + backend + UI):

```bash
make docker-up-gpu      # NVIDIA GPU rendering — use this if you have one
make docker-up          # software rendering, portable
```

Open <http://localhost:5173>. Robots appear once Gazebo and their SLAM stacks are up
(~60 s; each robot's stack starts on a stagger, because four of them racing at once loses).
API is on <http://localhost:8080>. Stop with `make docker-down`.

**Use the GPU if you have one.** The lidar is a `gpu_lidar` sensor and raytracing it is
the simulation's dominant cost, which is what caps the sensor fidelity the fleet can
afford. On software rendering this stack ran at ~0.58x real time with a 360-sample lidar;
on an RTX 4070 it holds roughly the same real-time factor with **1800 samples**, so the GPU
buys five times the angular resolution rather than a higher frame rate. Steady state with
four robots, Nav2 and exploration all running measures 0.56-0.6; brief periods before the
full stack loads read close to 1.0, so measure late, not early.
`docker-compose.gpu.yml` is a separate overlay because a `devices: [driver: nvidia]`
reservation makes `docker compose up` fail outright on a machine without the NVIDIA
container runtime.

Synthetic fleet only (no Gazebo/ROS — useful for UI/backend work):

```bash
make docker-up-mock     # or: docker compose --profile mock up --build -d
```

```bash
make docker-logs        # follow logs
make docker-test        # backend pytest in the server image
```

### Resetting the simulation

**Reset sim** in the top bar returns the fleet to its start state: every robot back at its
spawn pose, every map discarded, goals cancelled, costmaps and odometry filters cleared.
It takes a couple of seconds and nothing restarts. The button asks for a second click,
because there is no undo.

It is gated on the `reset` adapter capability, which only `adapter_sim` advertises — so it
does not appear on a dashboard driving real robots, where "teleport to spawn and forget the
map" is not something that can happen. `curl -X POST localhost:8080/api/sim/reset` does the
same thing and returns which robots confirmed.

The backend clears its merged map only after the adapters report `reset_done`, never when
it sends the command: it holds each robot's last uploaded grid, and an upload already in
flight would otherwise restore the map a moment after it was cleared. If an adapter does not
confirm, the map is cleared anyway and that robot is named in a warning — its old map will
come back within a couple of seconds, and the reason needs to be visible.

Two things are worth expecting rather than debugging:

- **In `auto` merge mode the merged map does not come straight back, and may take a long
  time.** Registration has to re-earn its transforms against maps that are small again,
  and until it does it correctly reports `ambiguous occupancy match` and holds robots out.
  Measured after a real reset, with the fleet still exploring: over four minutes
  `global_members` oscillated between two robots and none, and never returned to the full
  fleet it had before the reset — the merged view flickers on and off for as long as that
  lasts. It is the four rejection tests doing their job on small maps, not a fault, but it
  does mean the merged map is not a good thing to watch right after a reset. Per-robot maps
  are correct immediately (`view_by_robot` in `GET /api/map/status`, which also names which
  test is refusing), and `static` mode has no such gap because its transforms are
  configured rather than estimated.
- **A reset does not restart the exploration bootstrap.** If `explore_seconds` has already
  elapsed the fleet comes back stationary and waits for goals.

Measured on the four-robot Gazebo stack: reset acknowledged by all four in 3.8 s, robots
returned from up to 13 m away to their spawn poses, 181,467 mapped cells cleared to 0, and
the simulation clock ran 87 s -> 99 s across the reset — forwards, which is the point of
resetting model poses rather than the world.

### Sharing the running sim over the web

```bash
make tunnel             # or: ./scripts/tunnel.sh --tool ngrok
```

One tunnel to port 5173 publishes the whole app — the `ui` container's nginx serves the
frontend and proxies `/api` and `/ws`, so the map, telemetry and controls all work through
it. Do not also tunnel 8080; it is the same backend without the frontend.

The script prefers ngrok when it is installed **and** has an authtoken (`ngrok config
add-authtoken <token>`), because ngrok v3 will not open a tunnel without one. Otherwise it
falls back to a cloudflared quick tunnel, which needs no account but mints a new hostname
every restart.

> **The URL has no authentication.** Anyone who has it can drive the robots, set goals and
> issue STOP ALL. That is acceptable for a simulator you are watching, and is not
> acceptable in front of a stack running `adapter_ros2` against real hardware — put an
> authenticating proxy there first.

The map supports pan/zoom, fleet and selection centring, click-to-select markers, a
metric grid, trails, labels, sensor/footprint overlays, map revision metadata, and live
registration diagnostics from `GET /api/map/status`.

The live map distinguishes the path already travelled (solid) from Nav2's current
predicted path (dashed). The settings button persists the unattended-warning delay,
expected fleet size, adapter identities/endpoints, and perception controls in
`sessions/settings.json`; the fleet size is consumed on the next Gazebo/adapter start.

The stack includes MediaMTX for low-latency WHEP/WebRTC video. Set
`MEDIAMTX_WEBRTC_HOSTS` to the operator host's LAN address when browsers run on other
machines; robots push H.264 to port 8554. The simulation and hardware adapters retain a
5 Hz JPEG fallback, so camera frames remain visible if the media service is unavailable.
A portable RGB-only YOLOE-26n detector publishes normalized boxes over the same adapter
contract and the camera panel draws them without using Gazebo entity IDs. Inference runs
in a local sidecar: the CPU image serves all simulated robots, while the ROS 1 and ROS 2
hardware Compose files select the matching JetPack GPU image.

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
| `adapters/adapter_*/` | `mock` (synthetic), `sim` (Gazebo), `ros2` (hardware, **untested on a robot**) |
| `swarmdeck_ros/src/` | Gazebo world, robot model, SLAM, Nav2, bringup |
| `server/` | FastAPI backend — fleet registry, map merge, events, recording |
| `ui/` | Svelte 5 + Tailwind frontend |
| `study/` | Session configs (`1robot.yaml`, `2robot.yaml`, `4robot.yaml`) |
| `sessions/` | Recorded output, one directory per session |
| `docker/` | Server, UI, and Gazebo/ROS images; compose file at repo root |
| `scripts/` | Operator helpers — `tunnel.sh` publishes the running stack on a public URL |

## Map merging

Each robot runs its own 2D SLAM, so every map is in that robot's own frame with the
origin wherever it started. `mapsvc` merges them:

- **`static`** — transforms come from configured start poses. A lone unconfigured
  reference may define identity; any additional unconfigured robot stays in its
  local view instead of being silently overlaid at identity.
- **`auto`** — transforms estimated by grid registration (signed FFT cross-correlation over
  a coarse-to-fine yaw sweep, numpy only). When both robots upload XYZ, independent
  height-band registration may propose a transform, but it is accepted only after
  the occupancy grids validate its wall overlap and shared known area. Verified
  against Gazebo ground truth at **7.8 cm** with two robots at unknown relative
  poses; 52/52 correct at 0.1–3 cm on a synthetic all-headings sweep.

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

### Odometry

Each robot fuses wheel odometry with its gyro through a `robot_localization` EKF, which owns
`odom -> base_link`. This matters more than it sounds: SLAM only searches a small window
around its motion prior, so a bad prior cannot be fixed by scan matching. Measured over 120 s
of driving, fusion cuts drift 3-45x — 1.87 m down to 0.04 m over 21 m travelled. Wheel-derived
heading is never fused (slip destroys it) and neither is the IMU's absolute orientation
(Gazebo's is perfect, real MEMS IMUs have none). `fuse_imu:=false` reverts to raw wheel
odometry if you want to see what that costs.

Gazebo publishes **all-zero covariance** on both odometry and IMU, which any estimator reads
as "infinitely precise". `covariance_relay.py` republishes them on `<ns>/odom_cov` and
`<ns>/imu_cov` carrying the noise `robot.sdf.jinja` actually injects.

The EKF deliberately does **not** consume those. Feeding it real covariance was measured to
make it ten times worse (0.46 m -> 4.80 m mean error over 300 s), because
`process_noise_covariance` was tuned for the zero-covariance regime where the filter tracks
measurements exactly. `fuse_covariance:=true` turns it on for anyone who wants to re-tune;
see `docs/KNOWN_ISSUES.md` #7. The relay is used on the 3D path, where RTAB-Map's
`icp_odometry` weighs the inertial prior by covariance and cannot work around a zero.

Exploration alternates wandering with two kinds of long-range leg. **Homing** returns a robot
to its own start pose, which makes a large *intra*-robot loop closure a property of the run.
**Rendezvous** sends a scheduled pair to the midpoint of their two start poses, which is the
only thing that makes *inter*-robot closures reliable — homing alone left encounters to
chance, and a measured four-robot run linked only 2 of 6 possible pairs, in two disjoint
clusters that could not be merged. Meeting points come from the configured start poses, the
same information `merge_mode: static` uses; ground truth stays out of the loop.

Robots must also be kept off walls: a jammed differential drive spins its wheels and the drive
plugin integrates motion that never happened, which no filter can undo. `explore_seconds:=N`
(or `EXPLORE_SECONDS` in Docker, default 600) drives the fleet reactively to bootstrap the
maps and then stops, handing control back to Nav2.

### The mapping lidar

`fleet.lidar` in the study config selects a sensor profile, so re-pointing the fleet at
different hardware is a one-line change rather than an SDF edit:

| Profile | Samples/rev | Rings | Range | Notes |
|---|---|---|---|---|
| `legacy_360` | 360 (1.003°) | 1 | 16 m | What the project shipped. Kept only as an A/B control. |
| `generic_2d` | 1800 (0.200°) | 1 | 30 m | **Default.** Same 2D pipeline, continuous walls. |
| `generic_32` | 1800 | 33 (±22.5°) | 30 m | Generic 32-beam 3D unit; needs `slam_backend:=rtabmap`. |
| `vlp16` | 1800 | 17 (±15°) | 100 m | |
| `os1_32` | 1024 | 33 (±22.5°) | 100 m | |

Any field can be overridden under the profile (`h_samples: 2048`), and the older
`fleet.lidar_rings` spelling still works.

Samples per revolution is the single biggest lever on map quality, and 360 was far too
few: adjacent rays land `range · 2π/samples` apart, so at 1.003° they are one 5 cm cell
apart at 2.9 m and three cells apart at 8.6 m, and distant walls come out as dotted fans
rather than lines. Real units are 0.1–0.4°. Rings must be 1 or **odd** — an even count
leaves no ring at zero elevation, and the spawner refuses it.

`study/baseline_legacy.yaml` reproduces the old sensor exactly, so "the map got better"
stays a measurement rather than an opinion:

```bash
SWARMDECK_CONFIG=/app/study/baseline_legacy.yaml make docker-up-gpu
```

### Per-robot SLAM backend

`slam_backend:=toolbox` (default) runs SLAM Toolbox on a planar scan.

`slam_backend:=rtabmap` is the 3D lidar + IMU path, and needs a multi-ring profile:

```bash
SLAM_BACKEND=rtabmap SWARMDECK_CONFIG=/app/study/4robot_3d.yaml make docker-up-gpu
```

Two nodes, and the split matters. `icp_odometry` owns `odom -> base_link`, registering each
cloud against a local map of recent ones — this **replaces** wheel odometry rather than
filtering it, because wheel odometry cannot observe slip at all and lidar odometry is simply
immune to it. `rtabmap` then owns `map_frame -> odom`: pose graph, loop closure, and the 2D
`OccupancyGrid` republished on `<ns>/map`. That last remap is what makes the swap cheap —
the adapter, the backend, Nav2's static layer and the GUI cannot tell which backend produced
the grid.

Verified in Gazebo over a four-robot run: 175k known cells across a 24.2 m extent with 93%
of occupied cells in continuous pieces, comparable to the 2D path, on an 82-node pose graph
per robot. De-skewing is off in simulation only, because Gazebo's cloud carries no per-point
timestamps; turn it on for real hardware, where every driver worth using stamps points.

### Odometry front ends

`icp_odometry` (RTAB-Map) is the shipped one. **DLIO** (vectr-ucla's Direct LiDAR-Inertial
Odometry) is wired up as an alternative for hardware evaluation:

```bash
SWARMDECK_CONFIG=/app/study/4robot_3d.yaml SLAM_BACKEND=rtabmap \
  docker compose -f docker-compose.yml -f docker-compose.gpu.yml \
                 -f docker-compose.dlio.yml --profile gazebo up --build -d
```

Measured in simulation it is **worse** — 0.262 m vs 0.137 m displacement error, 17.3 vs
9.7 deg heading — but that is a statement about Gazebo, not about DLIO: its central
mechanism is per-point de-skewing and Gazebo's cloud carries no per-point timestamps, so
what is being measured is DLIO without its main idea. It also publishes at ~100 Hz against
icp_odometry's ~2 Hz. See `docs/KNOWN_ISSUES.md` #6 before drawing conclusions.

### Swarm SLAM

The merged map is a **stitcher**: each robot keeps a private pose graph and `mapsvc` aligns
finished grids afterwards, so no robot's drift is ever corrected by another's observations.
`swarmdeck_cslam` wires MISTLab's [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) in to
change that, adding the missing capability — inter-robot loop closure and a joint GTSAM
optimisation:

```bash
make docker-up-cslam    # Gazebo + RTAB-Map + Swarm-SLAM, 3D lidar profile
```

cslam runs on Jazzy against the apt GTSAM 4.2 (no 4.1.1 pin needed) and **produces real,
geometrically verified inter-robot loop closures** — 30 verified out of 42 candidates in one
minute of a four-robot run, with the other 12 correctly rejected by TEASER++. The GUI's
Swarm SLAM panel shows it live and the map draws a dashed link between robots that have met.

`swarmdeck_cslam`'s `graph_reporter` summarises the joint graph onto `/swarmdeck/slam_graph`
as JSON, which `adapter_sim` forwards as the protocol's optional `slam_graph` message. That
indirection is deliberate: `cslam_common_interfaces` exists only in the cslam image, and a
subscriber cannot deserialise a type it does not have.

`merge_mode: cslam` exists to let the pose graph produce the merge transform instead of grid
correlation, and it is **not the default, because measured against ground truth it is much
worse** (11-16 m against 0.03-0.20 m). The grids come from RTAB-Map and the transforms from
cslam — two independent SLAM systems whose trajectories disagree by metres. See
`docs/KNOWN_ISSUES.md` #5 for the measurement and what unifying them would take.

Membership requires more than a loop closure. A fleet that has split into groups which never
met has *two* common frames, and overlaying them would place robots confidently in the wrong
building — so only robots sharing the majority frame are merged, and the rest are visibly
absent.

Getting there meant five silent traps — namespace convention, a shared IPC namespace, the
right front-end executable, TEASER++'s Python bindings, and a fleet that keeps moving. All
are written up in `docs/KNOWN_ISSUES.md`.

Everything downstream of it is built and tested. `map.merge_mode: cslam` makes membership of
the merged map depend on a robot actually having closed a loop with the fleet, and demotes
the grid registration to an *independent cross-check* — reported, never applied, using
evidence the loop closures did not use. Adapters report their own view of the graph with the
optional protocol-2 `slam_graph` message, and the GUI's Swarm SLAM panel shows keyframes,
the inter-robot closure matrix, and that check. `adapter_mock` emits a synthetic graph, so
the whole path runs with no ROS at all:

```bash
make docker-up-mock
```

### 3D view

The layers popover has a **3D cloud** toggle. It overlays the 2D map rather than replacing
it — 2D stays the working surface where goals are set, and 3D is for inspecting what the
robots have actually built. Drag to orbit, scroll to zoom; points are coloured by
contributing robot.

Adapters push a voxel-downsampled cloud to `POST /api/adapter/cloud` (zlib int16 xyz at
1 cm, ≤ 0.25 Hz) and the backend serves the merged result from `GET /api/map/cloud`, using
the same membership rule as the 2D merge: an unregistered robot contributes nothing, because
drawing its cloud in the shared frame would render a guess as a measurement.

**By default the cloud is flat.** RTAB-Map's `cloud_map` is a ground projection unless its
occupancy is kept in 3D, so every z is exactly 0 and the view draws a plane. `grid_3d:=true`
gives it real structure, and costs half the simulation speed — measured over matched 330 s
four-robot runs:

| `grid_3d` | cloud_map | real-time factor | known cells |
|---|---|---|---|
| `false` (default) | flat, z = 0.00 | 0.54 | 176k |
| `true` | z −1.68 … 1.80 | 0.25 | 121k |

The 2D map is byte-identical either way; this only decides what the optional 3D view has to
show. Off by default because the 3D view is optional and the map is not.

Only a 3D SLAM backend produces a cloud — `adapter_sim` forwards RTAB-Map's `cloud_map`, so
a SLAM Toolbox fleet correctly shows an empty 3D view rather than a misleading one.
`adapter_mock` emits a synthetic cloud so the view works with no ROS.

It is drawn in raw WebGL2, deliberately: a points-only viewer with an orbit camera is a
couple of hundred lines, and this frontend keeps exactly two dependencies. Revisit that if it
ever needs meshes, lighting or picking.

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

`explore.py` is reactive obstacle-avoiding wandering, steering off both the mapping lidar and
the bumper-height proximity scan (the mapping lidar at 0.402 m looks straight over another
robot's body, so on that scan alone the fleet is invisible to itself). It alternates wandering
with a leg back to the start pose every `--loop-period` seconds (default 90), because a pose
graph is only corrected where it closes a loop and pure random wandering closes loops by luck.
That is also what gives several robots the overlapping coverage map registration needs. Do
**not** drive the
robots open-loop: a robot jammed against a wall spins its wheels, odometry integrates motion
that never happened, and SLAM — which uses odometry as its prior — produces a useless map.
The Docker stack runs this automatically via `EXPLORE_SECONDS`.

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

Odometry fusion, and `slam_backend:=rtabmap` (both already in the Gazebo image):

```bash
sudo apt install ros-jazzy-robot-localization \
                 ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-util
```

MediaMTX is included in the Docker stack for video. `multirobot_map_merge` is deliberately *not* used: it is
a ROS node, and the backend is ROS-free by design — see `docs/architecture.md` §1.

**ROS 1 note.** Noetic is EOL and cannot install on Ubuntu 24.04, and
`ros-jazzy-ros1-bridge` does not exist. A ROS 1 robot runs `adapter_ros1` in its own
Noetic container, on the robot or a companion machine — which the adapter contract
requires anyway.
