# Simulation

in a fork that adds a Filament-based photorealism medium and a Jolt physics
engine. The simulator publishes sensor data and continuous odometry; peer
Swarm-SLAM and native MOLA provide the single mapping pipeline.

For the current timestamp contract, adapter/estimator optimizations, reproducible
benchmarks, and tuning tradeoffs, see [Simulation performance](../operations/simulation-performance.md).

## Why


**Camera images were not photographs.** The detection pipeline (prompted YOLOE,
depth projection, operator review) is a headline feature, and the classes in
`adapters/perception/catalog.py` were calibrated on real photographs. Flat-shaded
SDF primitives are not the input that catalog describes.

**Odometry drift was a model, not a measurement.** The `drift` mode is useful
for fast control-path smoke runs. Sensor-backed runs use Fast-LIVO2, while
peer Swarm-SLAM remains the only corrected-pose authority.

**Lidar fidelity was CPU-bound.** Without an NVIDIA runtime, `gpu_lidar`
raytraced on the CPU at roughly 0.58x real time, which is why the mapping lidar
sat at 360 samples per revolution and distant walls came out dotted. The
photorealism medium renders through Vulkan and reaches 0.93x real time on the
*software* rasterizer with four robots, a 17-ring lidar and 320x240 RGB-D
cameras.

## Topology

```
┌─ argos container (no ROS) ──────────────────┐
│ argos3 + fork plugins + libswarmdeck_argos  │
│   jolt physics · photorealism (Vulkan)      │
│   photorealistic_lidar · _camera · imu      │
│   <odometry implementation="external">      │
│                                             │
│   <loop_functions swarmdeck_bridge> ────────┼──► /run/swarmdeck/argos.sock
│   <media external_estimator id="uf"> ───────┼──► /run/swarmdeck/uf.sock
└─────────────────────────────────────────────┘
        │                              │
        ▼                              ▼
┌─ sim container (Jazzy) ─────────────────────┐
│ swarmdeck_argos_bridge                       │
│  /clock /ns/scan/points /ns/odom /ns/imu     │
│  /ns/camera/* /ns/tf                         │
│ adapter_sim ────────────────► server :8080  │
└─────────────────────────────────────────────┘
        │
        ▼
┌─ peer + MOLA + MGG services ────────────────┐
│ peer Swarm-SLAM → MOLA products → MGG       │
│ FollowPath controller and local costmap      │
└─────────────────────────────────────────────┘
```

Three containers around one volume. The split is not incidental: Fast-LIVO2
runs in its own container on ROS 2 and the rest of the stack is Jazzy, and
they never have to interoperate over DDS, because the estimator's answer returns through
ARGoS as an ordinary `CCI_OdometrySensor` reading rather than over DDS. The
estimator's own ROS traffic stays inside its container. Fast-LIVO2 builds from source,
enabling native deployment across both ARM64 (NVIDIA Jetson / Apple Silicon / ARM servers)
and AMD64 architectures.

ARGoS dials both sockets; the other two bind them. That decides startup order,
and `argos-entrypoint.sh` waits for the generated experiment rather than racing.

It waits for a *fresh* one specifically. The runtime directory is a named
volume that outlives the containers, so the previous run's `session.argos` is
already on disk at startup and mere existence proves nothing: ARGoS therefore
refuses any experiment older than its own container start, and the generator
renames its output into place so that mtime always belongs to a complete file.
Restarting the `argos` service alone against an already-running `sim` is the
one legitimate case this rejects; set `ARGOS_ACCEPT_STALE=true` for it.

## What generates what

| File | Written by | Read by |
|---|---|---|
| `<runtime>/indoor.gltf` | `scenario/make_argos_world.py` | photorealism `<prop>` (drawn) |
| `<runtime>/indoor_collision.gltf` | same run of the same script | Jolt `<mesh>` (collided with) |
| `<runtime>/session.argos` | `scenario/make_argos_session.py` | `argos3`, and `argos_mounts.py` for the estimator's extrinsics |
| `<runtime>/argos.sock` | the ROS bridge (bind) | the loop function (dial) |
| `<runtime>/uf.sock` | Fast-LIVO2 (bind) | the `external_estimator` medium (dial) |
| `argos/assets/props/*.glb` | `make_argos_world.py --props`, committed | both, per detection target |

`<runtime>` is `/run/swarmdeck` under Compose. Keep it short: a Unix socket path
over 107 bytes fails to bind, and the error names neither the socket nor the
limit.

## Single sources of truth

Two tables decide almost everything, and both are read by several subsystems
that would otherwise drift apart in silence.

**`scenario/spawn_fleet.py`** holds `RobotSpec` and `LidarSpec`. The same rows
become Nav2's footprint and inflation radius, SLAM's `base_link -> lidar` static
transform, the adapter's `hello`, and the sensor mounts in the generated
experiment. A lidar mounted 0.25 m below where SLAM believes it is tilts and
offsets every scan, and nothing raises.

`RobotSpec.track_gauge` is the sharpest edge here: the ARGoS controller turns a
commanded `(v, omega)` into wheel speeds with it and the Jolt model turns them
back with its own `TRACK_GAUGE` constant, so a disagreement scales every turn
rate for the whole run. `swarmdeck_sim/test/test_make_argos_session.py` asserts
they match.

**`scenario/generate_world.py`** holds the floor plan, the furniture and the
seeded target placement, and both world backends build from it. A detection
scored against one backend's world is otherwise being compared with a run of a
different building.

## Environments and Scenarios

SwarmDeck supports three simulation environments. The two prebuilt ones are
rows of one table, `scenario/worlds.py` (assets, transform, lamps, exposure,
deployment, targets); `make_argos_session.py` renders whichever row the
config's `world:` names, and `session.launch.py` and the visual test ask the
same table whether a world is prebuilt or procedural.

1. **Procedural Indoor World (`world: procedural`, default)**:
   - Configured in `configs/4robot.yaml`, `configs/3robot.yaml`, etc.
   - Procedurally generates a seeded multi-room indoor facility (`indoor.gltf`) with walls, doorways, rooms, and furniture.
   - Uses a 30 × 30 × 6 m arena, overcast daylight, and point lights.

2. **Amazon Lumberyard Bistro (`world: bistro`)**:
   - Configured in `configs/4robot_bistro.yaml`, `configs/3robot_bistro.yaml`, `configs/bistro.yaml`.
   - Uses the realistic Parisian street scene (`bistro_exterior.glb`) from `argos3-examples/experiments/bistro_exploration`.
   - The glTF geometry is loaded directly into Jolt physics as a
     `<mesh id="world_mesh">` entity, at exactly the position, orientation and
     scale of the photorealism `<prop>`, with zero proxy collision boxes.
   - Physics and rendering read two different FILES, though, and the difference
     is one mesh: the drawn `indoor.gltf` has a floor slab, the collided
     `indoor_collision.gltf` does not. The `<jolt>` engine already supplies the
     ground as a `<floor height="0">` plane, and the slab's top face is also at
     z=0, so cooking it into the collision mesh rests every robot on two
     coincident surfaces. The degenerate contacts cost 60-100% of the commanded
     turn rate (position-dependent, all three platforms) while leaving
     translation almost untouched: a robot that drives but will not turn, with
     nothing logged anywhere. The renderer still needs a floor to photograph,
     hence the split rather than deleting the slab outright.
   - Uses a 200 × 210 × 70 m arena, PBR night lighting with 28 street lamps, San Giuseppe IBL environment map, and EV 5.6 camera exposure (`aperture="2" shutter_speed="0.02" sensitivity="400"`).
   - Start poses distributed along the 141 m closed road circuit.

```bash
# Launch Bistro in ARGoS:
make up-argos-bistro          # software rendering
make up-argos-bistro-gpu      # NVIDIA GPU
make up-argos-bistro-dri      # Intel/AMD DRI

# Capture Bistro visual test dashboard:
make visual-test-bistro       # -> /tmp/swarmdeck_visual/fleet_visual_dashboard.png
```

3. **DARPA SubT Finals Prize Round World 01 (`world: subt_finals`)**:
   - Configured in `configs/4robot_subt_finals.yaml`.
   - The Finals staging hangar and the 450 m tunnel circuit behind it, from the
     argos3 fork's `photorealism/environments/finals_prize_round_world_01`
     are regenerated there, not vendored). Mounted into the `argos` and `sim`
     containers at `/app/argos3-environments/finals_prize_round_world_01`;
     `SWARMDECK_SUBT_FINALS_DIR` overrides the host path.
   - Visual (`finals_prize_round_world_01.glb`, 3.7 M triangles) and collision
     (`.collision.glb`, the tiles' own lower-poly colliders, 1.3 M) are
     different files at one transform, checked by the scenario tests.
   - Underground: no sky, no sun, 88 SDF lamps in the staging area and the lit
     tiles; the rest is dark, as in the competition (the robots there carried
     their own lights). Exposure `aperture="2" shutter_speed="0.04"
     sensitivity="400"`.
   - The fleet deploys in the staging hangar (floor z = -0.01, walls at
     y = ±5.5, back wall x = -20, all measured through the collision mesh with
     Jolt ray casts), two columns at x = -14.5 / -16.5, y = ±1, facing +x:
     the tunnel leaves through a 3.3 m gate at (-10.5, 0). robot_0 is at the
     head of the group. Detection targets stand along the walls of the first
     tunnel tile (0.7 m off each wall) and in the hangar's back corners;
     `MeshSurface(below=1.0)` keeps them on the floor rather than the 3.5 m
     ceiling.

```bash
make up-argos-subt            # software rendering
make up-argos-subt-gpu        # NVIDIA GPU
make up-argos-subt-dri        # Intel/AMD DRI
make visual-test-subt         # -> /tmp/swarmdeck_visual/fleet_visual_dashboard.png
```

## Rates

Set in `make_argos_session.py`, and not free choices.

| Setting | Value | Why |
|---|---|---|
| `ticks_per_second` | 100 | Ultra-Fusion's `make_profile.py` derives IMU noise densities that scale by `sqrt(rate)`; a 10x rate error is a 3.16x noise error it cannot see. |
| lidar | 10 Hz | What a VLP-16 spins at. `framerate_divider="10"`. |
| camera | 5 Hz | Matches the adapter's JPEG preview budget. Resolution, not rate, is the main render-cost dial. |
| bridge exchange | 10 Hz | What ROS sees, and the `/clock` rate. |
| lidar rings | 17 over ±15° | VLP-16 geometry plus a ring at elevation 0. The estimator needs the vertical structure; SLAM Toolbox needs the horizontal ring, because `cloud_to_scan` in `slice` mode selects exactly that and an even ring count leaves nothing at elevation 0. |

The simulation is paced to real time by the loop function, because the operator
UI, the WebRTC video and teleoperation all run on wall-clock time while
everything inside ROS runs on the bridged `/clock`. Headless ARGoS has no pacing
of its own. The pacing only ever sleeps: when the estimator holds the lockstep
exchange up the simulation simply runs slow, rather than skipping the exchange
that carries the sensor data.

## Robots

| Config profile | ARGoS entity | Chassis | Lidar height |
|---|---|---|---|
| `bunker` | `bunker` | 1.023 × 0.778 m, 170 kg | 0.720 m |
| `scout_mini` | `scout_mini` | 0.612 × 0.580 m, 25 kg | 0.4525 m |
| `spot` | `spot` | 1.100 × 0.500 m, 32.7 kg | 0.970 m |

All three are plugins in the ARGoS fork, beside its existing `bunker-mini`
(which is a *different, smaller* machine and not what SwarmDeck's hardware fleet
runs). Spot is a rigid body driven by differential steering: the gait is not
simulated, and its collision shape is the standing envelope so that nothing
drives through the space its legs occupy. The legs appear in the glTF visual,
which is what the cameras and the photorealistic lidar actually raytrace.

**Every entity type needs a `<type>.visual.xml` descriptor**, matched by exact
name, or `pr_scene_sync.cpp` skips it. A skipped robot is invisible in RGB,
depth and segmentation *and* to every other robot's lidar, and the only sign is
one `[INFO] Photorealism: no visual model for entity type` line.

## Odometry

`<odometry implementation="external" medium="uf"/>`. The pose is whatever
Ultra-Fusion estimated from the simulated IMU, 17-ring lidar and wheel encoders,
so it drifts the way a real front-end drifts.

Three consequences worth stating plainly:

- **No EKF.** `robot_localization` is not launched on this path. The pose
  arrives already fused and the bridge is the only publisher of
  `odom -> base_link`; a filter on top would add latency and double-count the
- **`alignment="none"`.** Each robot's estimate starts at its own origin, as a
  real robot's does. `alignment="ground_truth"` would put the whole fleet in a
  shared frame, which is exactly what `swarmdeck-slam` exists to recover.
- **A reset cannot re-zero it.** Ultra-Fusion runs outside ARGoS and has no
  reset input. Its frame carries on and the teleport appears as a
  discontinuity; the SLAM reset in the next step starts a fresh map anchored at
  whatever `odom` then reads, so the offset lands in `map -> odom` rather than
  accumulating. Faking a re-zero would put a step in `odom` that the estimator
  does not know about, and SLAM would integrate it as motion.

The extrinsics Ultra-Fusion corrects for are read out of the generated
experiment by `deploy/docker/ultrafusion/argos_mounts.py`, per robot. The
upstream tooling takes one fleet-wide `LIDAR_IN_BODY`, which is correct for a
homogeneous fleet and a fixed, invisible bias on three quarters of this one.

## The bumper scan

`<ns>/proximity_scan`, which `nav2_params.yaml` documents as the only source
that sees a rubber duck (0.33 m) or a neighbour's chassis (0.28 m). An ARGoS
robot has one lidar, so the bumper scan is a second `pointcloud_to_laserscan`
instance in `flatten` mode over the same cloud, range-limited. The costmap
parameters are unchanged and still name both sources.

## Developing against it

`make up-argos-dev` is the configuration to work against. Measured on a 20-core
laptop with the Intel iGPU:

| Configuration | Real-time factor |
|---|---|
| 4 robots, Ultra-Fusion, unbounded detector | 0.023x |
| 4 robots, Ultra-Fusion, detector capped | 0.089x |
| **3 robots, drift odometry** | **0.694x** |

Two independent savings. Almost all of the cost is per robot (a SLAM Toolbox
instance, a Nav2 stack, an odometry estimator and a set of rendered sensors
each), and `odometry:=drift` takes Ultra-Fusion out of the lockstep exchange
entirely, so the `ultrafusion` service is not started at all.

`configs/3robot.yaml` keeps one of every platform rather than dropping a robot
from `4robot.yaml`, which would drop the Spot and leave two Bunkers and a Scout
Mini. The platforms differ in footprint and in mapping-lidar height, and a
fleet of similar robots hides every problem that causes.

**What drift odometry costs.** It perturbs ground-truth motion with Gaussian
noise. It cannot slip a wheel against an obstacle, lose a scan to geometric
degeneracy, or fail to converge, and those are the failures `swarmdeck-slam`
exists to survive. A map made this way says the pipeline runs; it does not say
the pipeline works. Use `make up-argos` before believing anything about mapping
quality.

Ground truth is on `/<ns>/ground_truth` in both modes, so the two can be scored
against the same reference.

## Exploration and operator goals

`EXPLORE_SECONDS` (600 by default) drives the fleet reactively to bootstrap the
maps, because Nav2 goals issued against an empty map jam robots into walls.
That bootstrap publishes `cmd_vel` directly, so it and Nav2 address the same
topic.

An operator goal wins: `explore.py` watches each robot's `navigate_to_pose`
status and goes silent on that robot's `cmd_vel` for as long as a goal is
accepted or executing. Without that, the robot receives both streams
interleaved and follows neither, which presents as a robot ignoring its planned
path rather than as a conflict.

`--seconds` is wall-clock rather than simulation time, deliberately, because it
bounds how long an operator waits. The consequence under this backend is worth
knowing: at a real-time factor of 0.09 the same ten minutes buys about a
when the bootstrap ends. `EXPLORE_SECONDS=0` skips it entirely.

## Host requirements

One, and it is easy to miss because being short of it fails silently:

```bash
sudo sysctl -w net.core.rmem_max=8388608     # 7 MB DDS buffers, plus headroom
```

`net.core.rmem_max` is a global kernel setting rather than a per-namespace one,
so a container inherits the host's value and cannot raise it. Docker will not
even create a container that asks to. Below roughly 7 MB, Fast DDS quietly
hands out smaller buffers, most lidar revolutions never reassemble, and
Ultra-Fusion looks like it diverged. `run_uf.sh` reads the value at startup and
says so either way.

## Verification

```bash
# Sensor frames, end to end, with no estimator and no containers.
make visual-test          # -> /tmp/swarmdeck_visual/fleet_visual_dashboard.png

# The generated experiment against the shared spec tables.
python3 -m pytest swarmdeck_ros/src/swarmdeck_sim/test -q

# Every LaunchDescription, both backends.
make docker-test-launch

# The whole stack.
make up-argos             # or up-argos-dri / up-argos-gpu
docker logs swarmdeck-fast_livo2-1 2>&1 | grep arrivals
```

`make visual-test` is the one that catches the silent failures. A robot with no
glTF descriptor is invisible to its neighbours; a lidar mounted below the deck
reports a ring of obstacles at its own radius and the robot sits still having
"found nowhere to go"; a camera whose frames never arrive publishes black. None
of those raise, none appear in a log, and all three are obvious in a picture.

## Building the fork

The `argos` image builds it, from a pinned commit
(`ARGOS_REF` in `Dockerfile.argos` and `docker-compose.yml`, currently
`83ca4602`: contact-driven ground-robot traction, robot-mounted headlights, the
viewer flashlight and a 64 MiB Filament handle arena). Bump it deliberately and together with whatever in
`swarmdeck_sim` needs the newer simulator: a floating ref would mean the
simulator changes under the stack with nothing in this repository recording
that it did.

For host development (`make sim`, `make visual-test` against a source tree):

```bash
# Filament SDK, once
curl -LO https://github.com/google/filament/releases/download/v1.72.1/filament-v1.72.1-linux.tgz
mkdir -p ~/filament-sdk && tar xzf filament-v1.72.1-linux.tgz -C ~/filament-sdk

git clone https://github.com/beltrame/argos3 ~/Projects/argos3
cmake -S ~/Projects/argos3/src -B ~/Projects/argos3/build \
      -DCMAKE_BUILD_TYPE=Release -DARGOS_BUILD_JOLT=ON \
      -DFILAMENT_DIR=~/filament-sdk/filament
cmake --build ~/Projects/argos3/build -j && sudo cmake --install ~/Projects/argos3/build

# SwarmDeck's controller and bridge loop function
cmake -S argos -B argos/build -DCMAKE_BUILD_TYPE=Release && cmake --build argos/build -j
```

`-DARGOS_BUILD_JOLT=ON` is not optional: the generated experiment asks for a
`<jolt>` engine and a `<mesh>` collision entity, and both live in that plugin.

