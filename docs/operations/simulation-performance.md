# Simulation timing and performance

Use `make up-sim RENDER=software ODOMETRY=drift` for a portable development
fleet. Use `ODOMETRY=fast_livo2` to exercise sensor-driven odometry. Synthetic
drift avoids the external estimator's CPU cost, but does not test estimator
convergence or geometric degeneracy. See [simulation architecture](../architecture/simulation.md)
for rendering backends, scenarios, calibration, and launch configuration.

## Where the work runs

| Stage | Responsibility | Main cost |
|---|---|---|
| ARGoS | Physics, camera/depth and LiDAR rendering | Scene complexity, sensor resolution, render rate, robot count |
| ARGoS ROS bridge | Decode packets; publish truth, odometry/TF, RGB-D, cloud, planar and proximity scans | Socket bytes, point projections, ROS serialization |
| Fast-LIVO2 link and estimators | Separate sensor socket; publish estimator inputs; return estimated poses | Per-robot estimation; optional image processing |
| `adapter_sim` | Capture-time pose lookup, keyframe gating, RGB projection, detections, cloud/map uploads | Point processing, detection inference, image encoding, network |
| Collaborative SLAM/server/UI | Register keyframes, optimize maps, serve and display geometry | Registration and browser rendering; separate from simulation real-time factor |

Ground truth is published separately for evaluation. It is not substituted for
an unconverged estimator. Odometry profile selection and TF ownership are defined
in `adapters/protocol/swarmdeck_protocol/odometry.py`; do not enable another
publisher for the same `odom → base_link` transform.

## Timestamp contract

The ARGoS observation protocol carries an exchange tick and individual odometry,
LiDAR, and camera ticks. The C++ loop function forwards those fields from the
sensor readings. The ROS bridge uses:

| Output | Timestamp |
|---|---|
| `/clock`, ground truth, IMU | Exchange tick (the IMU packet has no separate timestamp) |
| Odometry and its TF | Estimator tick; an unchanged estimate is not republished |
| Point cloud, planar scan, proximity scan | Same LiDAR capture tick, including the projection into `base_link` |
| RGB image, depth image, camera info | Same camera capture tick |

Ticks are converted with integer arithmetic. RGB-D and odometry frames with
future or non-increasing ticks are withheld. Zero is a valid initial tick.
LiDAR retains its existing compatibility fallback: if the capture tick is in the
future or more than one simulation second old, use the exchange tick and warn
once. Investigate that warning; this fallback cannot recover the true pose of
an incorrectly timestamped scan.

Duplicate camera and LiDAR packets are fully read from the socket before being
skipped, so subsequent robots remain aligned. Empty LiDAR packets produce
all-infinite planar/proximity scans without disconnecting the bridge. A bridge
reconnection or clock rewind clears its sensor deduplication state. This is not
a reset of the estimator or SLAM graph: start a fresh stack/session for a fresh
trajectory. The legacy Gazebo reset service is not a complete ARGoS reset path.

Capture-time TF checks and the keyframe yaw-rate gate remain necessary even with
correct stamps: a delayed transform or a rolling scan during a fast turn can
still be unsuitable for reconstruction. See [keyframe timing](keyframe-yaw.md).

The separate **AEBR external-estimator input protocol** has only the exchange
tick, not individual camera/LiDAR capture ticks. The Python Fast-LIVO2 link
cannot recover timing that this protocol does not transmit. Correcting that
contract requires coordinated changes to the ARGoS fork and estimator bridge;
observation-bridge timestamp fixes alone do not establish estimator accuracy.
A Fast-LIVO2 `Path` is a pose fallback and cannot erase measured twist from an
odometry message at the same timestamp.

## Tuning

| Setting | Effect and tradeoff |
|---|---|
| `RENDER=software`, `dri`, or `gpu` | Software Vulkan is portable but uses CPU; choose a supported hardware renderer to free CPU for estimation. |
| `ODOMETRY=drift` / `fast_livo2` | Development speed versus exercising the real estimator. |
| Scenario robot count and sensor configuration | Fewer robots, lower resolution, or less frequent rendering reduce work at the source. Keep estimator sampling and calibration consistent. |
| `SWARMDECK_ESTIMATOR_CHANNELS` | Compose defaults to `imu,lidar,wheels`. Camera rendering for the dashboard does not require forwarding images to the estimator. |
| `IMG_EN` | Compose defaults to `0`. Visual estimation also requires camera data forwarded through the estimator channel configuration. |
| `ADAPTER_DELAY` | Defaults to 60 seconds for startup ordering; lowering it is not a throughput optimization. |
| `SWARMDECK_DETECTION_PERIOD_S` | Adapter detection interval defaults to 1 second; increasing it reduces inference load but delays detections. |
| `SWARMDECK_KEYFRAME_VOXEL_M` | Defaults to 0.05 m. Larger voxels reduce keyframe size at the cost of detail. |
| `SWARMDECK_KEYFRAME_MAX_YAW_RATE_DEG` | Defaults to 8 degrees/s. An integrity gate, not a performance setting to disable to increase map updates. |

Environment settings must reach the process/container that reads them. Inspect
the resolved Compose configuration when adding overrides. The adapter's display
cloud uses a separate 0.10 m voxel grid; changing keyframe resolution does not
change that display transport. A two-second minimum keyframe interval and
bounded upload queue already limit downstream work.

Fast-LIVO2 raw image channel conversion and PNG encoding now run only when the
corresponding publisher has subscribers. Standard PNG bytes preserve RGB;
the raw estimator image remains BGR. A new subscriber receives the next frame.
Camera calibration continues to publish independently.

## Reproduce CPU measurements

From the repository root, with development dependencies installed:

```sh
server/.venv/bin/python scripts/benchmark-sim.py
server/.venv/bin/python -m pytest -q adapters/test swarmdeck_ros/src/swarmdeck_sim/test swarmdeck_ros/src/swarmdeck_bringup/test
```

A local run on September 7, 2026 measured the following medians over 30 warmed
iterations. Inputs are deterministic synthetic points and a noisy RGB image;
these are CPU microbenchmarks, not guarantees for a particular GPU or scenario.

| Operation | Median |
|---|---:|
| 32,000 voxel keys, former structured `np.unique(axis=0)` | 16.40 ms |
| Same keys, shared packed-integer deduplication | 3.92 ms |
| 32,000 hits, planar and proximity projections together | 0.18 ms |
| 320×240 noisy RGB, PNG encoding | 4.27 ms |

The adapter display-cloud upload now uses the packed helper already used by
keyframe downsampling. It preserves first-point selection and wire order, with
the existing overflow fallback. PNG cost is eliminated when no compressed-image
subscriber exists. Existing vectorized scan projections are retained.

For end-to-end evaluation, record the scenario, robot count, rendering backend,
odometry profile, sensor rates, and container revisions. Compare simulation-time
advance against wall time, CPU/GPU utilization, capture-to-TF age, keyframe
acceptance, and trajectory/map error against ground truth. Check turns as well
as straight motion. Historical real-time factors in the architecture document
are specific to their measured configurations.

Packet tests cover delayed/duplicate frames, empty scans, reconnects, clock
rewinds, RGB channel order, and odometry fallback precedence. They do not replace
a live ROS/ARGoS trajectory comparison. The Fast-LIVO2 link's single-callback
non-lockstep polling and lockstep timeout behavior also warrant profiling under
large fleets before changing executor scheduling.

## Bistro deployment and collision geometry

The `bistro`, `4robot_bistro`, and `3robot_bistro` configurations now deploy the
fleet together on the street near `(-13, 5)`: a two-metre grid with every robot
facing south. The four-robot formation spans 2 × 2 m between robot centres.
The 0.15 m initial anchor height lets the bodies settle onto the uneven street.
Custom start poses may specify `z`; indoor defaults retain 0.02 m clearance.

The scene uses a visual roll of +90° to convert glTF Y-up to ARGoS Z-up.
**Jolt's mesh loader also performs this conversion by default.** Previously,
copying the visual roll to the collision mesh applied the rotation twice,
putting collision triangles in a different plane from the rendered surfaces.
All generated world and target meshes now explicitly set `y_up="false"` and
keep the same position, orientation, and scale as their visual prop. Do not
remove that attribute merely because the XML transforms already match.

Bistro now uses its triangle mesh as the ground. It no longer has a second,
invisible infinite plane at z=0. The indoor world keeps that plane because its
collision asset intentionally excludes its floor slab. In a local query of the
actual Bistro GLB, 1.3 × 1.3 m patches around the new spawn points had no obstacle
triangle bounds in the 0.15–1.3 m height band. Nine ground probes per footprint
ranged from −0.048 to +0.117 m after the scene's −0.3 m translation, which is why
fixed z=0 spawning and a second flat floor were inappropriate.

Rebuild the ARGoS and simulation images when activating these changes, using
your chosen rendering/odometry settings. Start a fresh mapping session: an old
map recorded against the incorrectly rotated geometry is not a validation of
the corrected scene. XML transform, spawn-spacing, and asset tests run in the
simulation test suite. An isolated local ARGoS/Jolt run also completed 60 exchanges (six simulation
seconds) while commanding 0.30 m/s and 0.15 rad/s. All four robots produced RGB,
depth, and about 30,600 LiDAR hits; final ground-anchor heights were 0.01–0.02 m
after driving approximately 1.7 m. All three custom visuals loaded successfully.
This short spawn-area check does not cover every curb or the full street circuit.
It used local ARGoS build plugins and software Vulkan, not rebuilt deployment
containers; its 95.4-second wall time includes startup/rendering overhead.

To reproduce with locally built ARGoS plugins:

```sh
server/.venv/bin/python tests/integration/run_visual_test.py \
  --config configs/4robot_bistro.yaml --ticks 60 \
  --outdir /tmp/swarmdeck-bistro-check
```

The [robot visual assets](../../argos/assets/robots/README.md) describe the new
Bunker, Scout Mini, and Spot meshes and how to regenerate them.

## Missing local maps and sparse loop closures

In a live Bistro run, Spot had valid odometry and scans but no local map or
keyframes. `/robot_3/slam_toolbox` was **inactive**, despite successful
configuration: its configure service response timed out and the lifecycle
manager never proceeded to activation. The process being alive did not imply
that it was mapping. Check its state inside the sim container:

```sh
docker exec swarmdeck-sim-1 bash -c \
  'source /opt/ros/jazzy/setup.bash; ros2 lifecycle get /robot_3/slam_toolbox'
```

If it is configured/inactive and should be mapping, activate it with
`ros2 lifecycle set /robot_3/slam_toolbox activate` in the same environment.
This preserves the running simulation. Once the map appears, check the Nav2
lifecycle states too: a missing map can leave its planner and subsequent nodes
inactive. Activate only nodes confirmed inactive; do not reset working nodes
or issue motion commands as part of this diagnosis. Startup staggering and
longer client timeouts reduce contention but do not recover every lost service
response automatically.

The same investigation found the nearest-return scan-novelty signature was
mostly measuring the street: 60/60 sectors on robots 0 and 2, 56/60 on robot 1,
and 57/60 on Spot. Ground intersections form a near ring that changes little
with translation, hiding changing walls behind it. Profiles with a physical
height band now compute novelty using that band above the ground. Full 3D
geometry, including the ground, is still uploaded; timestamp, yaw-rate, and
registration verification gates remain in effect.

Use `/api/slam/backend` to distinguish a stopped ingest pipeline from sparse
accepted matches. `queued=0` with increasing keyframes means the worker is
keeping up. `accepted_closures` includes same-robot closures;
`inter_robot_closures` counts connections between robots. The diagnosed run had
one same-robot closure and zero inter-robot closures, rather than a disabled
loop-closure worker. More useful captures improve matching opportunities;
they do not guarantee geometrically valid loop closures.

The novelty fix requires a new adapter process. The current simulation
entrypoint exits if its adapter child exits, so plan a coordinated simulation
restart instead of killing that child in an active run.

## Detection targets on the Bistro road

Target placement now samples the Bistro GLB's pavement triangles over each
rotated model footprint at 5 cm spacing. It subtracts the model's actual bottom
height and leaves 5 mm clearance above the highest sampled surface. This replaces
the old fixed z=0 placement: the brick road under the first duck is about 7 cm
above zero. Physics and rendering receive the same computed position. Missing
pavement raises a generation error instead of silently burying a collider.

ARGoS composes Euler rotations as `Rx * Ry * Rz`. For a Y-up glTF model that
needs a +90° X rotation, world yaw therefore goes in the **second** XML angle:
`orientation="0,yaw_degrees,90"`. Putting it in the first angle tipped the props
sideways. This correction applies to indoor targets as well as Bistro.

Blocks, spools, disc cones, and foam noodles are now nonblocking perception
targets: they remain visible to RGB-D and LiDAR but have no static collision
mesh. These 6–9 cm objects otherwise acted as infinite-mass barriers below the
navigation proximity scan's 15 cm cutoff. The large ducks remain collidable.
This does not simulate pushing, rolling, or deforming the small objects.
