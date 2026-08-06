# Known Issues

Live list. Update as things are resolved.

## Open

### 1. Production video pipeline unbuilt

`swarmdeck_media` is an empty package and MediaMTX is not installed, so the target WHEP
path and its <300 ms latency criterion remain Phase 4 work. Simulation cameras now use a
verified 5 Hz JPEG preview fallback through the adapter and ROS-free backend.

### 2. Registration needs shared coverage

`auto` mode aligns two maps to a few centimetres, but only when the robots have actually
seen the same places. With largely disjoint coverage the problem is ill-posed. The
dashboard then shows each selected robot's local map; it does not overlay grids using
configured spawn priors. This is inherent to map registration, not a bug, but it means
exploration has to be arranged so robots overlap.

Coverage is now measured rather than assumed: `support` is the fraction of the smaller
map's known area shared with the reference, and a match below 0.35 is refused however well
it scores. Near-disjoint coverage of a building with strong repeated structure can still
produce a wrong answer — `support` and `yaw_ratio` cut this sharply but cannot make an
ill-posed problem well-posed. Treat `auto` mode without configured start poses as
best-effort; with priors, a wrong match fails closed to the prior.

A low `support` is usually a statement about exploration or map quality rather than about
registration. On a 4-robot run with the odometry and startup problems below fixed, roughly
five minutes of exploration reaches support 0.87-0.98 and all three non-reference robots
register.

### 3. Duck detector is a portable classical baseline, not a trained neural model

The shipped detector already produces live RGB bounding boxes in Gazebo and accepts
the same BGR frames from a physical camera, but it currently uses colour/shape evidence.
A publicly available fine-tuned YOLO duck model was not embedded because its own model
card warns that the training data has unresolved licensing. The detector API is kept
model-shaped (`detect_bgr` in, normalized boxes out), so a licensed ONNX model can
replace the baseline without changing ROS, the adapter protocol, backend, or UI. A
production model still needs licensed real-camera training/validation data covering the
actual lighting and duck variants.

### 4. The merged map is a stitcher, not multi-robot SLAM

**This describes the default stack.** With `make docker-up-cslam` the fleet runs Swarm-SLAM
and genuinely does close inter-robot loops (see Resolved, below); without it the following
holds.

Every robot runs a private pose graph and the backend aligns the finished grids afterwards,
so no robot's drift is ever corrected by another robot's observations. There is no
inter-robot loop closure, no shared graph, and no transitive registration — a robot that
overlaps robot 2 but not the reference can never join the global map. Nav2 also still plans
on each robot's own map rather than the merged one (FR-M8). This is a deliberate
consequence of the ROS-free backend, not an oversight, but it caps what the fleet can do.
docs/collaborative-slam.md sets out what would change and a staged path.

### 5. cslam's own pose estimates are wrong; the SwarmDeck plumbing is not the problem

**This was measured directly, outside SwarmDeck.** `cslamtruth.py` compares
`/rN/cslam/current_pose_estimate` against Gazebo ground truth, composing with the anchor
robot's configured start pose (cslam anchors its common frame at the lowest-id robot's
first keyframe — Swarm-SLAM paper, anchor selection). With all four robots merged into one
frame and **6 of 6 pairs linked by verified closures**:

```text
robot    cslam frame  cslam pose + anchor      ground truth             error
robot_0  robot0_map   ( -17.59,  -7.48, -31.5)  (   3.00,   1.25,  95.8)   22.37 m  127.3 deg
robot_1  robot0_map   ( -16.77,   8.86,-135.3)  (   0.49,  -7.92,  20.3)   24.07 m  155.6 deg
robot_2  robot0_map   ( -14.15,   4.54, -14.9)  (  -2.68,  -3.75,-155.0)   14.15 m  140.1 deg
robot_3  robot0_map   ( -20.73,   9.00, 135.3)  (  -4.35,  -0.04,-148.1)   18.72 m   76.5 deg
```

Every robot correctly reports `robot0_map`, so **the frames merged**; the poses inside that
frame are 14-24 m and 76-155 deg wrong. Before merging, each robot in its own frame was
0.84-5.58 m out. **Merging made it worse**, which is the signature of false inter-robot
loop closures rather than of odometry drift — a wrong closure welds two graphs at the wrong
place and the optimiser spreads that error across both trajectories.

This settles a question three rounds of integration work could not: the SwarmDeck side is
correct. `cslam_grid.py` builds the map from cslam's own keyframes at cslam's own poses,
the adapter forwards cslam's common-frame pose verbatim, and the backend anchors it to the
configured start pose. All of that is right, and the map is still wrong, because the input
is wrong.

**Root cause of the stall that hid this for so long.** Swarm-SLAM elects the lowest-id
robot as optimizer (`is_optimizer()` compares ids only), while `connected_robots_` is
populated *exclusively* from verified inter-robot closures. A robot that has met nobody is
therefore unconnected and still wins the election, and the connected cluster defers to it:

* robot_0 wedged against geometry and stopped moving
* it stopped producing keyframes (frozen at 107 while others reached 300+)
* with no keyframes it got no inter-robot closures, so it was unconnected
* it still held the optimizer election, so robots 1-3 — which had a fully connected
  triangle of verified closures — published **empty** optimisation results and kept
  reporting poses in their own frames
* nothing logged an error anywhere

Two mitigations now ship. `explore.py` detects a wedged robot by comparing commanded motion
against actual odometry displacement (0.04 m in 8 s while being driven) and backs it out;
`graph_reporter.py` warns when the elected optimizer has no closures. With both, the fleet
reached **6 of 6 linked pairs and all four robots merged** for the first time.

**CONFIRMED: false loop closures are the cause, and verification strictness fixes it.**
Three configurations measured against ground truth on matched runs:

| `registration_min_inliers` / `similarity_threshold` | pairs linked | merged-robot error |
|---|---|---|
| 100 / 0.90 (upstream defaults) | 6 of 6 | 14.15 - 24.07 m |
| 175 / 0.92 | 5 of 6 | 0.50 - 40.71 m |
| **250 / 0.95 (now shipped)** | 1 of 6 | **0.73 - 3.07 m** |

Accuracy is **monotonically inverse** to how many closures are accepted — there is no happy
middle, and 175/0.92 still admitted one that put a robot 40 m out. Upstream's defaults merge
everything and get everything wrong. We ship the strict end: fewer robots merge, and the
ones that do are right, which is the same trade grid registration already makes by refusing
rather than guessing.

That does mean the collaborative map is currently *partial*. It is still worth having: a
correct two-robot merge beats a confidently wrong four-robot one, and the GUI shows exactly
who is in and who is not.

**Where to go next**, in order of expected value:

1. **Better place recognition** is the real fix, because it attacks the cause rather than
   filtering the symptom. ScanContext is rotation-tolerant but weak to lateral shift and to
   repeated structure. Scan Context++ targets exactly that; learned descriptors are the
   other direction. Our own height-band `register_3d` is the same idea — vertical structure
   disambiguates what a plan view cannot — and could feed cslam rather than only the merge.
2. **The back end runs GTSAM's GNC robust optimizer with DEFAULT parameters** and exposes no
   tuning at all. GNC tolerates outliers up to a breakdown point, past which the solution is
   garbage rather than degraded — consistent with what we measured. Exposing `GncParams`
   would be a small, high-value upstream patch.
3. **Odometry drift is a separate, smaller problem.** With strict verification one unmerged
   robot still sat 19.5 m out in its OWN frame with only 0.8 deg of heading error — pure
   translation drift, not a merging fault. That is where a better odometry front end (DLIO
   with de-skewing on real hardware) would help.

### (superseded) cslam cannot drive the merge while the grids come from RTAB-Map

`merge_mode: cslam` is implemented, unit-tested, and wired end to end — and it produces a
**much worse map than grid registration**. Measured against Gazebo ground truth on a live
four-robot run:

| | merged-pose error vs ground truth |
|---|---|
| `merge_mode: auto` (grid registration) | 0.03-0.20 m |
| `merge_mode: cslam` (pose graph) | 11-16 m |

The cause is architectural. The occupancy grids come from **RTAB-Map**; the transforms come
from **cslam**. Those are two independent SLAM systems with separately optimised
trajectories, and they disagree by metres:

```text
robot_0  cslam[robot0_map]=(12.67, 0.00)  rtabmap_map=(10.69, 7.86)  gap= 8.10 m
robot_1  cslam[robot0_map]=( 8.83, 8.31)  rtabmap_map=(-1.04,-8.47)  gap=19.47 m
robot_3  cslam[robot0_map]=( 2.68,-3.06)  rtabmap_map=(-3.32, 0.09)  gap= 6.79 m
robot_2  cslam[robot2_map]=( 1.08,-2.13)  rtabmap_map=( 1.08,-2.15)  gap= 0.02 m
```

robot_2 is the control: cslam had not merged it, so its cslam frame was still its own — and
there the two systems agree to 2 cm. That is what shows the problem is the frame
relationship, not the transform arithmetic (`T = pose_common o pose_own^-1`, which is
correct and tested).

Fixing it means unifying the two rather than adjusting a constant — either feed cslam's
optimised poses back into RTAB-Map as pose priors, or build the merged grid from cslam's own
keyframe clouds. Until then `make docker-up-cslam` uses `merge_mode: auto` and cslam earns
its keep as the loop-closure and monitoring layer; `study/4robot_cslam.yaml` keeps the
experimental path with its measurements recorded.

### 6. Nothing detects a robot stuck with its wheels spinning

The EKF fixes the heading channel, and `explore.py` keeps robots off walls, but neither
closes the underlying hole: at a constant velocity there is no inertial signature that
distinguishes driving from being stuck with the wheels turning, so wheel velocity is still
taken at face value. A jam that `explore.py` fails to avoid — wedged on furniture, or two
robots interlocking — still injects translational error, and the only thing that can catch
it is SLAM noticing the scan did not move. Detecting slip explicitly (comparing commanded
and scan-derived motion, or fusing scan-matching velocity into the EKF) is unimplemented.

The same limit applies to a robot picked up and moved. Gazebo's DiffDrive odometry, like
wheel encoders on hardware, cannot observe being dragged sideways. Scan matching and loop
closure repair ordinary drift, but a large instantaneous displacement stays wrong until
lidar SLAM recognises the place again. A production robot that must recover immediately
from "kidnapping" needs lidar odometry, visual odometry, landmarks, or global
relocalization. Gazebo ground truth remains scoring-only and is never fed into navigation.

### 6. DLIO builds and runs, and is WORSE than icp_odometry in simulation

Direct LiDAR-Inertial Odometry (vectr-ucla) is wired up behind
`docker-compose.dlio.yml` and measured against Gazebo ground truth over a shared window,
scoring displacement and heading change (which needs no frame alignment to be fair):

| front end | mean displacement error | mean heading error |
|---|---|---|
| `icp_odometry` (RTAB-Map, shipped) | **0.137 m** | **9.7 deg** |
| DLIO | 0.262 m | 17.3 deg |

**Do not read this as "DLIO is worse".** It is a statement about the simulator. DLIO's
central mechanism is de-skewing each sweep against a continuous-time IMU model, and
Gazebo's cloud has NO per-point timestamps — fields are `x y z intensity ring`, verified
on a live topic — so `pointcloud/deskew` is off and the thing being evaluated is DLIO
minus its main idea. On hardware, where Ouster stamps `t`, Velodyne `time` and Livox
`offset_time`, the distortion it removes is real: a 10 Hz sweep taken while turning at
0.8 rad/s smears 4.6 deg, which at 10 m is 0.8 m of phantom structure.

Two things worth knowing if you pick this up:

* Upstream's default branch is **ROS 1 / catkin**. Only `feature/ros2` builds against
  ROS 2 — a community port, pinned by commit in `docker/Dockerfile.dlio`.
* DLIO publishes at ~100 Hz against icp_odometry's ~2 Hz (11,916 vs 269 messages in 30 s),
  so on a robot that needs a fast pose for control rather than for mapping, that alone may
  decide it.

To make the simulation comparison meaningful, Gazebo's lidar would have to emit per-point
timestamps — a change to the sensor model, not to DLIO.

**Sample size caveat:** these numbers come from ONE 90 s window with three robots that
moved far enough to score. The direction of the result is consistent with deskew being
disabled, but treat the exact figures as indicative, not as a benchmark. Re-run
`odomcompare.py` over several windows before quoting them anywhere.

## Resolved

### Swarm-SLAM: real inter-robot loop closures, and five silent traps

**RESOLVED.** `make docker-up-cslam` runs MISTLab's Swarm-SLAM against the Gazebo fleet and
produces real, geometrically verified inter-robot loop closures. Measured over one minute of
a four-robot run:

```text
loop closure msgs: 42   verified success: 30
  robots 0<->3: 12 verified closures
  robots 1<->2: 18 verified closures
```

12 of 42 candidates were rejected by TEASER++, which is the verification doing its job. The
GUI's Swarm SLAM panel shows this live (4/4 in frame, 36 closures, 40 keyframes) and the map
draws a dashed link between robots that have met.

cslam builds and runs on **ROS 2 Jazzy against the apt `ros-jazzy-gtsam` 4.2** — the 4.1.1
pin upstream's docs call for is not needed.

Five things had to be right, and every one of them fails *silently* — healthy nodes, no
error, no keyframe:

* **cslam's namespace must be `r<robot_id>`.** The C++ `pose_graph_manager` builds its
  inter-robot topics as `/r{id}/cslam/...` from `robot_id` alone while the Python front end
  inherits the launch namespace. Launching under `/robot_0` produces two complete parallel
  sets of topics that both look healthy and never meet.
* **The IPC namespace must be shared, not just the network.** Fast DDS discovers over UDP
  but carries data over shared memory for same-host peers, and each container has its own
  `/dev/shm`. Symptom: `ros2 topic list` shows every fleet topic, `ros2 node info` shows
  subscriptions correctly bound, and not one message arrives. Needs `ipc: shareable` on
  gazebo and `ipc: "service:gazebo"` on cslam.
* **The lidar front end is `lidar_handler_node.py`**, not the C++ `map_manager` (that is the
  stereo/RGB-D one). Both start cleanly; the wrong one never emits a lidar descriptor.
* **TEASER++'s Python bindings need `-DBUILD_PYTHON_BINDINGS=ON` and an explicit install.**
  `cmake --install` does not install them: the module lands in `build/python/` and the
  `__init__.py` that makes it importable is in the source tree. Both halves must be copied.
* **The fleet has to keep moving.** cslam creates a keyframe every
  `keyframe_generation_ratio_distance` metres, so a parked robot produces exactly one
  keyframe forever — nothing to describe, nothing to exchange, no loop to close. This was
  the last and most misleading failure: every earlier measurement here was taken after
  `EXPLORE_SECONDS` had elapsed and the fleet had stopped, which reads exactly like "cslam
  produces nothing". The compose overlay now raises `EXPLORE_SECONDS` for cslam runs, and
  the keyframe spacing is 0.25 m rather than 0.5 m because 12 cslam nodes alongside Gazebo
  drop the real-time factor far enough that robots cover ~1.4 cm per wall-clock second.

The lesson worth keeping: **a successful `docker build` says almost nothing here, and neither
does a healthy `ros2 node list`.** Launch the nodes, then check that data is actually moving
through them.

### The GUI drew every robot in the wrong place

The marker was composed from two different odometry chains. `map_frame -> odom` comes from
SLAM, `odom -> base_link` from the EKF — but `adapter_sim` read the *pose topic*
`<ns>/odom`, which carries the drive plugin's raw wheel integration regardless of who owns
the transform. SLAM computed its correction against the EKF's `base_link`, so composing it
with the wheel topic was wrong by exactly however far wheel odometry had diverged.

Measured live on a four-robot run, wheel-vs-EKF divergence against GUI error, per robot:

| | odom vs filtered | GUI error vs ground truth |
|---|---|---|
| robot_0 | 0.475 m | 0.47 m |
| robot_1 | 0.479 m | 0.46 m |
| robot_2 | 0.183 m | 0.16 m |
| robot_3 | 0.409 m | 0.37 m |

Same numbers, robot for robot. Taking both links from `<ns>/tf` instead brings it to
**0.01 m**. The wheel topic is kept as a fallback for a robot whose TF carries no
`odom -> base_link`, because reporting the map origin forever is worse than reporting a
drifting pose — but it logs a warning rather than doing it quietly.

Worth noting how bad this could get. Wheel odometry is precisely the channel that breaks
without bound when a differential drive jams and spins its wheels (8.8-30.5 m measured
below), which is why markers would occasionally appear tens of metres outside the building.

### Three silent failures found while bringing up the 3D SLAM path

Each produced a healthy-looking stack that mapped nothing, and each is worth recording
because the shape recurs.

**`launch_ros` retyped RTAB-Map's parameters.** Every `Foo/Bar` parameter RTAB-Map exposes
is a string it parses itself, but a `LaunchConfiguration` substituted into a parameter dict
gets a ROS type inferred from its text: `"true"` became a bool, `"30.0"` a double. Both
`icp_odometry` and `rtabmap` aborted with SIGABRT on startup —
`InvalidParameterTypeException`, a message that reads as though the value were wrong rather
than its type. Fixed by wrapping substituted values in `ParameterValue(..., value_type=str)`;
literal strings were never affected, which is why most of the file worked.

**`pointcloud_to_laserscan` listened on the global `/tf`.** In `slice` mode `target_frame`
is empty, so no transform is ever looked up and a missing `/tf` remap cannot be noticed. In
`flatten` mode it looks up `lidar -> base_link`, found nothing on the un-namespaced topic,
and dropped every cloud through a `tf2_ros::MessageFilter` that reports only "discarding
message because the queue is full" — never "no such transform". With no `<ns>/scan`,
`explore.py` skipped every robot, the fleet never moved, and RTAB-Map reported a one-node
map for the whole run.

**Incompatible QoS made the fleet sit still.** `explore.py` subscribed to `<ns>/scan` with
the default RELIABLE profile. On the 2D path the publisher is the ros_gz bridge (RELIABLE,
so it worked); on the 3D path it is `pointcloud_to_laserscan`, which publishes BEST_EFFORT —
incompatible, and the warning is logged on the *publisher's* side, so the subscriber sees
only silence. It now subscribes with sensor-data QoS, which is compatible with both, and
reports an explicit error if any robot has produced no scan after 15 s. A fleet that never
moves is the exact failure `explore.py` exists to prevent; it should not be able to fail
that way in silence.

### The mapping lidar was coarser than any real unit, and the GPU sat idle

360 samples per revolution is 1.003 deg per ray, so adjacent returns landed one 5 cm cell
apart at 2.9 m and three cells apart at 8.6 m: distant walls came out as dotted fans rather
than lines, which weakens scan matching and registration alike (both count occupied cells).
Real units are 0.1-0.4 deg.

This had gone unfixed because raising it costs raytracing and the container ran
`LIBGL_ALWAYS_SOFTWARE=1` at ~0.58x real time — with an idle discrete GPU on the host.
`docker-compose.gpu.yml` puts Gazebo's rendering on the GPU, which converts that budget into
resolution: four robots at **1800 samples/rev** hold roughly the same real-time factor the
360-sample software build did, so the GPU buys five times the angular resolution rather than
speed. Measure it late — steady state with Nav2 and exploration running reads 0.56-0.6, while
the first minute before the full stack loads reads close to 1.0 and flatters the result. The
sensor is now a profile in the fleet config (`fleet.lidar`), and `study/baseline_legacy.yaml`
reproduces the old one so the comparison stays a measurement.

Measured over identical 330 s four-robot runs, same code, same seed, both on the GPU, with
the only difference being the profile. "Fragments" are 8-connected components of occupied
cells: a dotted wall is many tiny islands, a continuous one is a few long chains.

| merged map | legacy_360 | generic_2d |
|---|---|---|
| fragments | 2246 | **384** |
| cells per fragment | 6.3 | **40.7** |
| speckle (components <= 2 cells) | 2076 | **284** |
| occupied cells in components >= 20 | 77.6% | **94.9%** |
| known cells | 188803 | 195443 |

| robot_0's own map | legacy_360 | generic_2d |
|---|---|---|
| known cells | 97382 | **185590** |
| fragments | 629 | **359** |
| cells per fragment | 7.8 | **24.2** |
| speckle | 520 | **230** |

Nearly doubling one robot's known area is the 30 m range plus loop-closing exploration, not
resolution. Registration improved with it: 4 of 4 robots in the global map and 3 of 3
non-reference robots matched, against 3 robots / 2 matched before.

Two supporting changes landed with it. `slam_toolbox.yaml`'s `minimum_travel_distance` and
`minimum_travel_heading` dropped 0.3 -> 0.15, which was only ever set that high because
scans were expensive on the CPU path. And `scenario/explore.py` now alternates wandering
with a leg back to the start pose, because a pose graph is corrected only where it closes a
loop and reactive wandering closes loops by luck — that also produces the mutual coverage
issue 2 needs.

### Real measurement covariance makes the EKF worse, and is off by default

`covariance_relay.py` restamps Gazebo's all-zero odom/IMU covariance with the sensor's
actual noise. Feeding that to `robot_localization` made the filter **ten times worse**.
Measured over 300 s, four robots, same seed, only the input topics differing:

| | wheel odom | EKF |
|---|---|---|
| real covariance (`fuse_covariance:=true`) | 1.55 m | 4.80 m (robot_3: 17.02 m) |
| zero covariance (default) | 1.47 m | **0.46 m** (worst 0.83 m) |

With an all-zero R the Kalman gain is 1 and the filter tracks each measurement exactly;
`process_noise_covariance` was tuned in that regime. Introduce a realistic R and that same
process noise lets prediction error accumulate between updates, and the estimate wanders —
past the wheel odometry it exists to improve, which is this package's oldest warning sign.
It is not a throughput problem: the relay was measured keeping up (IMU 144 Hz in,
150 Hz out).

So the EKF reads the raw topics and the relay does not run on the 2D path. Fixing it
properly means re-tuning `process_noise_covariance` against real covariance, which is a
measurement campaign rather than an edit; `fuse_covariance:=true` exists to run it. The
relay is still built and still used — RTAB-Map's `icp_odometry` consumes `imu_cov` on the
3D path — but whether it *helps* there has not been measured either.

### Simulated sensors published zero covariance

Gazebo's DiffDrive odometry and IMU both shipped all-zero covariance matrices, which every
estimator reads as "infinitely precise". The EKF worked around it by running on
`process_noise_covariance` alone; RTAB-Map's ICP odometry and a GTSAM back end cannot work
around it at all, since both weigh their inputs by covariance.

`swarmdeck_slam/nodes/covariance_relay.py` republishes both on `<ns>/odom_cov` and
`<ns>/imu_cov` carrying the noise `robot.sdf.jinja` actually injects, and `ekf.yaml` consumes
those names. It invents no information. Orientation covariance is set to `-1`, which is
REP-145 for "no orientation estimate" — the truth, since the fleet's IMU has no magnetometer
and Gazebo's perfect world quaternion would be laundered ground truth.

One trap worth recording, because the error message points at the wrong place: this
workspace builds `--symlink-install`, so `install(PROGRAMS)` does not set the executable bit
on a copy — the install tree symlinks the source and inherits *its* mode. A source file
committed 644 produces `executable 'covariance_relay.py' not found on the libexec directory`
while the symlink is present and correct in exactly that directory. Worse, the launch file
then continues without the relay, the EKF subscribes to topics nobody publishes, and the
whole robot maps nothing.

### Per-robot maps were garbage — five stacked causes, none of them registration

Symptom: individual maps were starbursts with free space bleeding through walls, grids
grew past the world extents (536x701 cells at 5 cm = 27x35 m inside a 24x24 m building),
and registration would lock on early in a run and then drop out as the maps deteriorated.
Three stacked causes, none of them registration's fault.

**1. Wheel odometry was fused with nothing.** The 200 Hz IMU was bridged with **zero
subscribers**, so wheel integration was the only motion prior, and SLAM Toolbox searches
only `correlation_search_space_dimension` around that prior — a prior far enough off cannot
be rescued by scan matching. The drive kinematics themselves are correct
(`wheel_separation: 0.38` matches links at y = +/-0.19, `wheel_radius: 0.11` matches the
collision geometry), so what corrupted odometry was genuine wheel slip: a jammed
differential drive keeps turning its wheels and the plugin integrates rotation that
produced no motion, exactly as encoders do on hardware.

*Measure this correctly.* Comparing `<ns>/odom` to `<ns>/ground_truth` pose-by-pose is
wrong, because each robot's odom frame is its spawn pose and `4robot.yaml` spawns two
robots at yaw pi — a raw comparison reports the frame offset as error. Compare each chain
to ground truth *relative to its own first sample* instead. Doing that during clean driving
over 120 s:

| | travelled | wheel odom | EKF | after SLAM |
|---|---|---|---|---|
| robot_1 | 21.37 m | 1.87 m / 4.1 deg | **0.04 m / 1.0 deg** | 0.39 m / 1.2 deg |
| robot_2 | 2.85 m | 0.27 m / 0.3 deg | **0.05 m / 1.7 deg** | 0.06 m / 3.0 deg |
| robot_3 | 12.35 m | 0.72 m / 5.2 deg | **0.21 m / 1.9 deg** | 0.02 m / 0.4 deg |

So wheel odometry is *adequate* when the robot is not jammed, and the EKF still improves it
3-45x. It is jamming that destroys it, which is why cause 2 below matters more than this
one. Fixed with a `robot_localization` EKF per robot fusing wheel *velocity* with gyro
*rate* and owning `odom -> base_link`; the drive plugin's TF bridge is dropped so the two
do not fight over the same transform.

Two deliberate non-choices there: wheel-derived heading is never fused (it is the channel
slip destroys), and the IMU's absolute orientation is never fused (Gazebo publishes a
perfect world quaternion, and a real MEMS IMU without a magnetometer has no absolute yaw,
so using it would launder ground truth). Gazebo's IMU is noiseless by default, so
`robot.sdf.jinja` now injects consumer-MEMS noise and bias; a test asserts it, because
without it the EKF's accuracy is meaningless.

**2. Nothing drove the fleet.** `explore.py` — the reactive avoidance driver written for
exactly this failure — was never invoked by `session.launch.py` or the Docker entrypoint.
Robots moved only on operator Nav2 goals, and Nav2 planning on a corrupt map drove them
into walls, which produced more slip, which corrupted the map further: 80+ `failed to
plan` warnings plus `Robot is out of bounds of the costmap`. Now wired as
`explore_seconds:=N` (`EXPLORE_SECONDS` in compose, default 600) which bootstraps the maps
and then stops, handing the fleet back to the operator. `explore.py` also steered only off
the mapping lidar at 0.402 m, which looks straight over another Duckiebot's body, so the
fleet was invisible to itself; it now also uses the bumper-height proximity scan.

**3. `slam_toolbox.yaml` was never loaded.** The file was keyed `slam_toolbox:`, which ROS
2 resolves to the absolute node path `/slam_toolbox`, while the nodes run namespaced as
`/robot_N/slam_toolbox`. No match, no warning, node healthy — every one of the 40-odd
tuned parameters silently fell back to slam_toolbox's own defaults. Confirmed live:
`minimum_time_interval` read 0.5 against 0.2 in the file, `minimum_travel_distance` 0.5
against 0.3, `map_update_interval` 10.0 against 1.0. It went unnoticed because many of the
file's values had been copied from the defaults, so spot-checking agreed. Both YAMLs in
`swarmdeck_slam` are now keyed `/**`. Check with
`ros2 param get /robot_0/slam_toolbox minimum_travel_distance` after any change.

**4. The EKF's own sensor frame did not exist in TF.** Introduced while fixing cause 1, and
worth recording because the failure is silent and the shape of it recurs. The IMU stamps its
messages `<ns>/base_link/imu`; only the lidar and proximity lidar had static transforms, so
`robot_localization` could not rotate the gyro into `base_link` and dropped **every** IMU
sample without logging anything. Because the gyro was configured as the filter's only yaw
source, the estimate then carried no heading information at all: 8-17 m and 50-175 deg of
drift in 90 s, roughly 30x *worse* than the unfused wheel odometry it replaced, and maps
ballooned to 87 x 49 m. Fixed by publishing the `<ns>/base_link/imu` frame in
`slam.launch.py` (identity — the sensor has no `<pose>`). If EKF output is ever worse than
its inputs, check first that every fused sensor's `frame_id` resolves in TF.

**5. Robots lost a startup race against each other.** With four robots, one stack would
come up unmapped: `lifecycle_manager_slam` timed out on `slam_toolbox/get_state`, logged
"Failed to bring up all requested nodes", and aborted. The `async_slam_toolbox_node` process
stayed alive but sat in `unconfigured` with no subscribers, so that robot silently produced
no map at all while everything else looked healthy — trap 1 in `slam.launch.py`'s docstring,
reached by a different route. Four lifecycle managers, four SLAM nodes and four Nav2 stacks
were all being launched in the same instant on a container already at ~0.58x real time.
Fixed by staggering each robot's bringup (`ROBOT_STAGGER` in `session.launch.py`), with the
adapter's start delay moved out past the last one.

Result after all five, verified end to end from a clean image build with 4 robots:

- Per-robot maps fit the building: 23.8-24.0 m extents against a 24 x 24 m world. Before:
  27 x 35 m, and up to 87 x 49 m once the broken EKF was in the loop.
- All four robots map; **all three non-reference robots register and are accepted**, with
  `dyaw` of 0.1 deg, -179.9 deg and 179.9 deg against true relative headings of 0, 180 and
  180 deg. Scores 0.557-0.744, support 0.873-0.978, `yaw_ratio` <= 0.21.
- Pose error over 100 s of driving: wheel 0.27-2.10 m, EKF 0.26-0.53 m, and after SLAM
  **0.03-0.18 m with heading within 2.2 deg**.

Note the ordering of blame. Registration was never the problem here — it was correctly
refusing maps that had genuinely deteriorated (support 0.010-0.048 at the worst). The
symptom "it registers, then deregisters as noise accumulates" is what a working rejection
test looks like when its inputs are degrading.

`ros2 node list` and `ros2 topic info` are unreliable in this container under load and
reported nodes and publishers as absent while they were demonstrably working. Prefer the
backend API (`/api/map/local/<id>/info`, `/api/map/status`) or the costmap's own
`StaticLayer: Resizing costmap` log lines as evidence that a robot is mapping.

Diagnostics that came back clean, worth recording so they are not re-investigated: the
scan itself (360 finite ranges, no infs or NaNs, none pinned at `range_max`), scan cadence
(exactly 0.100 s of sim time per message, zero dropped messages, no TF extrapolation
warnings), and the drive kinematics above.

Still open from this investigation: Gazebo publishes **all-zero covariance** on both odom
and IMU, so `robot_localization` is working from `process_noise_covariance` alone and has no
per-message uncertainty to weigh; and the mapping lidar's 360 samples per revolution
(1.003 deg) put rays 3 cells apart at 8.6 m, which is why distant walls render dotted. See
open issues 7 and 8.

### Registration silently reported confident 90-degree errors

The correlation peak in yaw is under a degree wide (a yaw error at radius `r` displaces a
point by `r·dyaw`), and the coarse sweep sampled every 4 deg, so it could miss the true peak
entirely and lock onto the building shell's own 90 deg symmetry. On this project's test
floor plan, **13 of 52 headings falling between coarse samples produced a confident answer
exactly 90 deg wrong**, with up to 1.8 m of translation error. The ratio test could not see
it, because it only compares rival *translations* within the winning yaw.

It hid because the six parametrised test yaws (0, 0, 35, −60, 120, 175) are all on or within
1 deg of the 4 deg grid, and because `4robot.yaml` starts robots at 0, 0, π, π — every
relative heading in the demo is 0 or 180 deg, exactly on the grid.

Fixed by making every search stage sample finely enough to find its own peak: the coarse
stage correlates against a *dilated* reference so the peak is wide enough to hit on a 4 deg
grid, distinct rotation hypotheses are then ranked against each other at a resolution that
can separate them (`yaw_ratio`), free space votes against matches that drive walls through
known-empty rooms, and the peak is interpolated to sub-cell precision. Now 52/52 correct on
the same sweep, 0.1–3 cm and under 0.1 deg, and a genuinely symmetric building is refused
instead of guessed. Regression tests cover off-grid yaws and symmetric plans.

### Map ingest blocked the server event loop

`POST /api/adapter/map` awaited nothing and ran ~190 ms of registration numpy inline, so
with three non-reference robots at 0.5 Hz roughly a quarter of the event loop was consumed
in 190 ms blocks and every WebSocket stuttered, telemetry included. Ingest now runs through
`asyncio.to_thread` under a lock, and an already-registered robot refines in a narrow yaw
window instead of re-sweeping all 360 deg on every upload.

### Nav2 costmaps could not see other robots

Two issues were stacked. Relative `scan` names resolved below the costmap nodes as
`/robot_N/local_costmap/scan`, where no sensor published, and the high mapping lidar
passed above the short Duckiebot collision body. Costmap sources now use absolute
per-robot topics, and a separate bumper-height forward proximity scan marks robots and
low obstacles without contaminating SLAM. In a controlled test, a robot 0.60 m ahead
added 13 lethal cells plus an inflated safety region to the live local costmap.

### Frontend rendering and map reload visually confirmed

The light, compact interface is verified at 1440×900 in headless Chrome against the
live backend. A one-robot exploration grew the map to 58,522 known cells, and a fresh
browser session restored and rendered that full map. The reconnect path now fetches
`GET /api/map` before resuming incremental patches.

### Nav2/diagnostic_updater ABI mismatch

`nav2_lifecycle_manager` 1.3.12 expected a constructor absent from the installed
`ros-jazzy-diagnostic-updater` 4.2.6. Upgrading diagnostic_updater to 4.2.7 restored
both SLAM and Nav2 lifecycle managers.

### Simulation clock and namespaced SLAM topics were not wired

The one-command launch had no Gazebo `/clock` bridge, and SLAM remappings expanded to
global `/scan` plus a duplicated `/robot_0/robot_0/map`. Bringup now creates one shared
clock bridge, while SLAM scan, map, metadata, and TF topics stay inside each robot's
namespace. Verified with a live map and a successful one-metre Nav2 goal.

### Nav2 was configured but not wired

Phase 3 now launches one lifecycle-managed Nav2 stack per robot, and `adapter_sim` uses
the `NavigateToPose` action with result and cancellation tracking. The backend converts
poses and goals between each robot's SLAM frame and the shared merged-map frame.
One-robot end-to-end navigation is verified; multi-robot reliability still needs a soak
run.

### `spawn_fleet.py` emitted invalid non-zero-yaw quaternions

It used `{z: yaw/2, w: 1.0}` even though the 4-robot config starts two robots at π yaw.
It now emits the normalized `{z: sin(yaw/2), w: cos(yaw/2)}` quaternion.

### The merged map was garbage — four stacked causes

Worth recording in full, because each masked the next.

**1. Multi-ring lidar cannot feed 2D SLAM through a height band.** The lidar had 8
vertical rings over ±0.26 rad. `pointcloud_to_laserscan` filters by height *in the lidar
frame*, so each ring truncates at a different range: a ±0.15 m band caps every ring below
4 m of the sensor's 16 m, producing a hatched wedge. There is no good band — the cutoff
is inherently range-dependent. Fixed by making the lidar **single-ring**, which also lets
Gazebo publish a usable `LaserScan` directly and removes `pointcloud_to_laserscan` from
the pipeline entirely.

*Amended after later audit.* The diagnosis above is right, including that no band and no
choice of filter frame can fix it — the truncation is geometric. But the root cause is
narrower than "multi-ring", and going single-ring gave up the 3D sensor to avoid it.
Gazebo spreads N vertical samples evenly across `[min_angle, max_angle]` **inclusive**, so
an **even** N leaves no ring at elevation 0 and every ring is tilted. A ring at elevation
`e` survives a band of half-height `h` only to `h/sin(e)`; with 8 rings over ±0.26 rad the
innermost sits at 0.0371 rad, giving `0.15/sin(0.0371) = 4.04 m` — exactly the 4 m that was
measured. An **odd** ring count puts one ring at exactly 0 elevation, which passes any band
at full range, and the tilted rings then add real 3D structure. `fleet.lidar_rings` now
selects the ring count, `spawn_fleet.py` refuses an even value, and a tight band selects
the horizontal ring (a wide one would only admit floor returns near the robot). See
docs/collaborative-slam.md §2.4.

**2. The world was not enclosed.** 40 × 40 m with a dozen thin partitions meant that from
most poses nothing was within sensor range, so SLAM carved a giant free-space starburst
with no structure to scan-match against. Fixed by a compact 24 × 24 m building with rooms
6 m across.

**3. Open-loop driving jammed robots into walls.** A stuck differential drive keeps
spinning its wheels, so the DiffDrive plugin integrates motion that never happened — we
measured odometry claiming (+5.1, −19.3) while the robot was actually at (−8.0, −2.1).
SLAM uses odometry as its motion prior, so this alone guarantees a broken map. Fixed by
`scenario/explore.py`, a reactive obstacle-avoiding wanderer.

**4. A symmetric floor plan makes registration ambiguous.** With uniformly spaced
identical rooms, shifting by one room width scores nearly as well as the truth, and the
merge locked onto a 5.9 m error. Fixed by irregular room widths plus distinct interior
features, and by adding a **ratio test** so near-ties are rejected rather than trusted.
(That ratio test turned out to cover only rival translations at one heading, which is how
the 90 deg failures above survived it. Rotation hypotheses are now ratio-tested too.)

After all four: ground truth scores 0.509 against 0.11–0.16 for neighbouring offsets, and
registration recovers it to **7.8 cm** with exact yaw.

### `slam_toolbox` silently does nothing

Three traps, all silent — no error, no log line, node appears healthy:

1. It is a **lifecycle node**. On Jazzy it sits in `unconfigured` with zero subscribers,
   logging nothing. `use_lifecycle_manager: false` did *not* make it self-transition.
   Fixed with `nav2_lifecycle_manager` + `autostart`.
2. The **`scan_topic` parameter has no effect** — the topic must be remapped. With the
   parameter alone, subscription count stays 0.
3. **Sim-time mismatch.** Without `/clock` bridged and `use_sim_time:=true` everywhere,
   TF lookups never resolve and SLAM stalls.

### Corner-based registration (added, then removed)

Shi-Tomasi corner extraction was added to beat corridor aliasing. Once the odometry bug
was fixed it bought nothing — plain occupancy matched to 7.8 cm, corners to 7.0 cm — and
its threshold was mis-calibrated (a straight wall scored as 300 corners). Removed rather
than shipped as unproven complexity.

### PNG/patch Y-orientation

`as_png` rendered grid row 0 at the image top while the frontend's `worldToGrid` flips Y.
`as_png` now applies `np.flipud`. **`take_patch` does not flip** — it ships raw grid rows,
which is correct because the frontend blits patches into a canvas it also indexes in grid
order. Worth re-checking the first time patches and the base map are viewed together.

## Process hygiene

`pkill -f <pattern>` matches the agent's own shell command line and kills the calling
shell (observed as exit code 144). Use `pkill -x <exe>`, or collect PIDs with
`ps aux | grep '[x]yz'` and kill by PID. Orphaned `gz sim` processes hold DDS ports and
silently poison the next run — `tests/integration/stop_stack.sh` does this correctly.
