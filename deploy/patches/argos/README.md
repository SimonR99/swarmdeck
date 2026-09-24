# Jolt terrain contact and step traversal

Native contact physics belongs in the sibling `../argos3` fork.
`Dockerfile.argos` applies only the bounded-step integration in `apply_steps.py`
to that native implementation. A missing insertion anchor fails the build
instead of silently omitting traversal after an upstream update. Reapplying the
step patch is idempotent.

The contact traction lives in fork commit `45aabc8a`; the image pin
(`ARGOS_REF`, currently `83ca4602`) must include it, because the step patch
anchors on its `SetDriveVelocity` call and fails the build without it.

`SwarmDeckStep` runs only for a commanded translation. A cheap forward shape
cast normally returns immediately. On contact, full-body casts check current
support, overhead clearance, a short raised advance, and static landing support.
The helper changes position only after every check passes. Dynamic bodies are
never treated as stairs. Jolt handles gravity and descent normally.

Robot bodies also enable Jolt's `mEnhancedInternalEdgeRemoval`. The Bistro
manhole reproduction found a robot stuck on an internal triangle edge despite
clear forward shape casts. This option rejects ghost edge contacts while
retaining the terrain faces and actual obstacles. The extra contact processing
is enabled per robot, rather than across every body in the scene.

Roll and pitch remain contact-driven rather than being locked upright.
`CJoltGroundRobotModel` sets moving-contact surface velocities, and Jolt supplies
traction bounded by friction and support load. It does not overwrite chassis
linear/angular velocity, so gravity, terrain attitude, and airborne motion
remain physical. The same mechanism supports differential steering.

Ground robots inherit the engine's configured `default_friction`. Drive targets
are updated in `UpdateFromEntityStatus`; the contact solver applies them at each
physics substep. The bounded step probe runs once per control tick.
With friction 0.6, all four native chassis crossed the actual SubT metal-platform
approach without tipping. A commanded Bunker descent reached Z = -3.916 m while
following the approximately 16-degree ramp. These isolated physics runs do not
qualify MGG navigation or mapping through a live descent.

Run both regression suites against a local ARGoS build:

```bash
server/.venv/bin/python deploy/patches/argos/run_tests.py ../argos3/build \
  --bistro ../argos3-examples/experiments/bistro_exploration/assets/bistro_exterior.glb
```

The runner uses the Jolt build's ABI flags. It extracts nearby road/manhole
triangles into temporary storage, reproduces the legacy stall, verifies traversal
with corrected contacts, and verifies that an added wall still stops the robot.
No Bistro geometry is copied into the repository.

Omit `--bistro` to run just the step suite (exact and near-limit steps, taller
obstacles, overhead clearance, dynamic bodies, and absent support). The runner
requires a C++ compiler; the optional mesh fixture also requires NumPy.

The native ARGoS checkout also contains real Bunker, Scout Mini and Spot
attitude regressions on a mesh incline, including differential yaw while
following the surface normal:

```bash
cmake --build ../argos3/build --target mesh_jolt_assets mesh_jolt_controller mesh_jolt_loop_functions
ctest --test-dir ../argos3/build -R '^jolt_mesh_' --output-on-failure
```

## Maintenance

`apply_steps.py` owns the per-platform limits (Bunker/Scout: 15 cm; Spot: 30 cm)
and idempotent source edits. Reapplying it updates an existing generated call
to the current limit, and rejects an unrecognized call instead of leaving a
stale setting.
`swarmdeck_step.h` owns the clearance algorithm. The build fails if upstream
insertion anchors change, including under Python's optimized mode. When updating
the ARGoS pin, review the generated model sources and rerun both suites. Match
the Jolt library's build flags: profiling and SIMD definitions affect its ABI.

## Opt-in parked lidar scheduling

`apply_parked_lidar.py` adds `parked_framerate_divider` and `parked_after_ticks`
to the lidar sensor and camera pool without changing RGB/depth cameras or
normal lidar defaults. `swarmdeck_lidar_schedule.h` chooses the original
schedule until the physical anchor has been exactly still for the configured
grace period; any translation or rotation restores it immediately. Simulation
clock reset restarts the grace period. The four lidar faces remain synchronized.
The patch also adds the helper to the installed public-header list. Unknown
upstream insertion anchors fail before any source files are written; applying
the patch twice is idempotent.

`Dockerfile.argos` applies it after the step patch. Offline regressions compile
and execute the standalone scheduling helper and test patch application against
the pinned ARGoS sources:

```bash
server/.venv/bin/python -m pytest -q \
  swarmdeck_ros/src/swarmdeck_sim/test/test_argos_parked_lidar.py \
  swarmdeck_ros/src/swarmdeck_sim/test/test_make_argos_session.py \
  swarmdeck_ros/src/swarmdeck_sim/test/test_simspeed_benchmark.py
```

Configuration, fidelity trade-offs, the **tuf-only** unpaced benchmark, and
remaining runtime qualification are in `configs/README-simspeed.md`.
