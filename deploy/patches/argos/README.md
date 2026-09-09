# Jolt step traversal

`Dockerfile.argos` applies `apply_steps.py` to the pinned ARGoS checkout and
rebuilds the four upright robot models. Upstream source outside the Docker build
is never edited. A missing insertion anchor fails the build instead of silently
omitting the helper after an upstream update.

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

## Maintenance

`apply_steps.py` owns the per-platform limits and idempotent source edits;
`swarmdeck_step.h` owns the clearance algorithm. The build fails if upstream
insertion anchors change, including under Python's optimized mode. When updating
the ARGoS pin, review the generated model sources and rerun both suites. Match
the Jolt library's build flags: profiling and SIMD definitions affect its ABI.
