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

## Bistro terrain and deployment

Bistro configurations deploy the fleet on a two-metre grid near `(-13, 5)`,
facing south. The 0.15 m initial anchor height lets bodies settle onto uneven
pavement. Custom start poses can specify `z`; indoor defaults use 0.02 m.

Keep these geometry contracts when changing scenario generation:

- World and target collision meshes use `y_up="false"` and the same transform
  as their visual props. The visual +90° roll already converts glTF Y-up into
  ARGoS Z-up; enabling Jolt's automatic conversion applies it twice.
- Bistro uses its mesh as ground. An additional infinite plane would create
  invisible surfaces. Indoor scenarios need that plane because their collision
  asset excludes the floor slab.
- Target placement samples pavement across the rotated footprint at 5 cm
  spacing, compensates for the model's bottom, and adds 5 mm clearance. Missing
  pavement is a generation error.
- ARGoS composes Euler rotations as `Rx * Ry * Rz`. For Y-up target models,
  use `orientation="0,yaw_degrees,90"` to apply world yaw without tipping them.
- Blocks, spools, disc cones, and foam noodles are nonblocking perception
  targets. RGB-D and LiDAR still see them; large ducks remain collidable.

The simulation step limit is 15 cm for Bunker/Scout and 30 cm for Spot, shared
by MGG and the ARGoS contact helper. These are simulation settings, not hardware
ratings. Traversal requires full-body clearance and static-support checks.
Navigation may still avoid low obstacles seen by its conservative proximity scan. Internal-edge correction prevents false Jolt
contacts at road/manhole seams. See the [physics patch guide](../../deploy/patches/argos/README.md)
for implementation and native regression commands.

Rebuild the ARGoS and simulation images after geometry changes and start a fresh
mapping session. XML transforms, spawn spacing, and assets have regression tests
in the simulation suite. To check rendering with locally built ARGoS plugins:

```sh
server/.venv/bin/python tests/integration/run_visual_test.py \
  --config configs/4robot_bistro.yaml --ticks 60 \
  --outdir /tmp/swarmdeck-bistro-check
```

This short spawn-area check does not validate every curb or the full street.
See [robot visual assets](../../argos/assets/robots/README.md) for regeneration.

## Missing local maps

A running SLAM Toolbox process may still be inactive after a lifecycle service
timeout. Check its state inside the simulation container:

```sh
docker exec swarmdeck-sim-1 bash -c \
```

If configured but inactive, activate it with
Check Nav2 lifecycle states too: a missing map can leave downstream nodes
inactive. Activate only nodes confirmed inactive. Startup staggering and longer
client timeouts reduce contention but cannot recover every lost response.

An active mapper with a **0×0 map** may have initialized its laser model from an
unrendered ARGoS scan (`MaxRange=0`). The bridge drains these packets and withholds
scans until the range is finite and above the minimum. Empty scans with valid
metadata still publish. For an already affected empty mapper, deactivate,
clean up, configure, and activate that node after valid scans arrive. Cleanup
discards its map; do not use this recovery on a working trajectory.

## Keyframe density and loop closures

Capture periods apply in both sensor time and monotonic wall time before cloud
processing. This bounds observation density in slow simulation and processing
cost during replay. Profiles with a physical height band compute scan novelty
above the ground, keeping street returns from hiding changes in nearby walls.
The novelty threshold is 0.25 m; full 3D geometry still uploads. Timestamp,
yaw-rate, and registration gates remain independent checks.


| Signal | Interpretation |
|---|---|
| `queued=0` with increasing keyframes | The ingest worker is keeping up. |
| `accepted_closures` | Accepted scan-pair constraints, including same-robot pairs; not independent revisits or surviving graph factors. |
| `inter_robot_closures` | Connections between robots. Proximity alone does not guarantee valid correspondences. |
| SLAM `/status` → `verification.intra` / `verification.inter` | Cumulative geometric verification outcomes, including convergence, inliers, error, translation, yaw, and degeneracy. |

Inter-robot recall remains a limitation. Same-robot scans can fill the descriptor
shortlist, but reserving peer slots previously caused a false merge in the
disjoint-building regression. Validate recall improvements against false merges
before relaxing thresholds or changing candidate selection.

Adapter changes require a process restart. The simulation entrypoint exits when
its adapter child exits, so use a coordinated simulation restart instead of
killing that child during an active session.
