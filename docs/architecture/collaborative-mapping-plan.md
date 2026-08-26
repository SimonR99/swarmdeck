# Collaborative mapping rebuild — implementation plan

Status as of 2026-08-24. Supersedes the map-registration approach described in
[collaborative-slam.md](collaborative-slam.md), which stays as the record of why
that approach was abandoned.

## The decision

**Stop merging maps. Merge trajectories.**

An occupancy grid is not data — it is a *rendering* of (trajectory x sensor
returns). Rasterizing a scan discards the pose information; grid registration is
an attempt to recover it afterwards by correlating pixels. That is a harder,
lossier problem than the one we already had the answer to, and it is why `auto`
mode is fragile on symmetric layouts, needs known-free cells as a tiebreaker, and
refuses near-disjoint coverage.

The `cslam` result is the same defect in sharper form: RTAB-Map produced the
grids, Swarm-SLAM produced the trajectory, and they disagreed by 11-16 m. Two
estimators, two answers, no reconciliation.

In the new design there is **no registration step anywhere**. Poses are optimized
in one joint graph; occupancy is rendered from the result; consistency is
guaranteed by construction.

## Architecture

```
per robot                              server
---------                              ------
Ouster + internal IMU
      |
unified LIO front-end   --keyframes-->  pose-graph back-end
      |                 --descriptors-> - intra-robot closures
odom->base_link                         - inter-robot closures (verified)
(continuous; the controller uses this)  - PCM + GNC outlier rejection
      ^                                 - joint optimization
world->map  <--transform only---------        |
                                        render occupancy from optimized
local costmap <- LIVE SENSORS ONLY      keyframe poses -> UI / allocation
```

Four properties this buys, none of which the old design had:

1. **One front-end everywhere.** Heterogeneous SLAM (LVI-SAM / SuperOdometry x2 /
   LIO-SAM) is the root cause of unmergeable maps: you cannot align two estimates
   that were never estimating the same thing.
2. **Robots ship keyframes and descriptors, not grids.** Kilobytes, not megabytes.
3. **The global map is rendered, never stitched.** Disagreement is impossible.
4. **What goes back down is a transform, not a map.** A handful of floats.

### What is allowed to use a foreign map

| Layer | Foreign map? | Rationale |
|---|---|---|
| Local costmap | **Never** | Collision authority. Transform error here is a crash. |
| Global planner | **Yes**, gated | Transform error here is a bad route: drive, discover, replan. |

Gates on the global planner path: verified transform only (same component);
foreign `occupied` trusted, foreign `free` treated as a penalized hypothesis;
cells age-weighted. Robots in different components are **never** placed in a
common frame — an unmerged map is a correct statement of ignorance, an overlaid
one is a confident lie the operator cannot see through.

## Phases

### Phase 0 — Environment and contract — DONE

New `slam/` distribution (`swarmdeck-slam`), pinned to **Python 3.12**.

> **The pin is load-bearing.** gtsam 4.2.2 declares `numpy<2`, and the constraint
> is real rather than stale metadata: under numpy 2.x it imports successfully and
> then **segfaults** on the first array-marshalling call (verified 2026-08-24).
> The server stays on 3.13/numpy 2.x; the two never share a process. This is one
> of the reasons the optimizer is a separate service rather than a module inside
> the FastAPI app.

Stack: `gtsam` 4.2.2 (LM + GNC), `small_gicp` 1.0.1 (GICP; far lighter than
Open3D and the only one of the two with cp313 wheels should the pin ever lift),
`scipy`, `numpy` 1.26.

Contract files, written first so parallel work could not diverge:

- `adapters/protocol/swarmdeck_protocol/keyframe.py` — the wire format, one
  source of truth shared by adapters, server, and back-end. Encode and decode
  live together so they cannot drift.
- `slam/swarmdeck_slam/types.py` — domain types, SE(3) helpers, the frame
  convention, and the `PoseGraphOptimizer` Protocol that keeps gtsam (the one
  Python-3.12-pinned component) behind a swappable interface.
- `slam/tests/synthetic.py` — a deterministic synthetic fleet with exact ground
  truth, drifting odometry, and occlusion. Shared by every test in the package.

**Frame convention:** every transform is named `T_a_b` and maps points in frame
`b` into frame `a`. Information matrices are 6x6 in gtsam Pose3 tangent order,
**rotation first**. Direction and ordering errors here fail silently — the graph
still optimizes, it just converges to a mirrored or mis-weighted map — so both
conventions are stated in every module and guarded by tests that fail on inversion.

### Phase 1 — SLAM core — COMPLETE (95 tests, one tracked open issue)

| Module | Responsibility |
|---|---|
| `graph.py` | gtsam pose graph; GNC on loop closures only; PCM pre-filter on inter-robot edges; union-find components; per-component gauge anchor |
| `descriptors.py` | Scan Context descriptor, rotation-invariant ring key, KD-tree candidate index returning a yaw prior |
| `verify.py` | GICP geometric verification; hard rejection gates; information matrix from the Hessian |
| `render.py` | Occupancy rendering from optimized poses; per-keyframe ground-relative height bands; vectorized free-space raytracing; per-component grids |
| `evaluation.py` | ATE / RPE, inter-robot transform error, component correctness, ablation tabulation |

Structural rules enforced across all five: reuse the shared SE(3) helpers rather
than reimplementing them; no compatibility shims; every module carries a test
that fails if a transform direction is inverted; every scaling claim is a
measured number in the suite rather than an assumption.

`tests/test_integration.py` runs the whole pipeline, because every module's own
suite passed while a seam between two of them was still unverified. That is the
normal outcome of parallel development rather than carelessness: each side tests
against its own self-consistent reading of a shared convention, and both readings
can be self-consistent while disagreeing with each other.

**Verified end to end.** Robots sharing a building merge into one component and
render one grid; robots in two different buildings are never merged, with zero
cross-building closures surviving geometric verification, so PCM never has to
fire as the second line of defense. True correspondences are recovered with a
median error of 1.3 cm.

**One open issue, tracked as a strict xfail rather than tuned away.** Optimizing
with real closures currently raises translation ATE by ~1.2-1.4x instead of
lowering it, consistently across injected drift from 0.012 to 0.15 m/m. Bisecting
the pipeline places the cause precisely:

| Input | Result |
|---|---|
| Exact ground-truth edges into `graph.py` | ATE 0.0 -- the solver is correct |
| Real closure transforms vs ground truth | 1.3 cm median -- the geometry is correct |
| Ground-truth transforms + `verify.py` information | still degrades (1.20x) |
| Real transforms + isotropic information | **improves (0.93x)** |

So the fault is in the information matrices, not the transforms or the solver:
GICP's Hessian arrives with roughly a 30:1 rotation-to-translation ratio, which
over-constrains orientation relative to position. Overall magnitude is not the
lever -- sweeping `info_scale` from 1 to 100 never recovers an improvement, and
at 100 every closure is rejected outright.

Correct weighting depends on a real sensor noise model, which is exactly what
Phase 6 produces. Calibrating it against a synthetic fixture would fit the
fixture's noise rather than the Ouster's, so it is deliberately left open with a
divergence guard (`< 3x`) beside it: a calibration error of centimetres and a
broken solver must not be able to look alike in the suite.

### Phase 2 — Ingestion path — DONE (tested offline; hardware tomorrow)

- Adapter-side keyframe production: voxel downsample, quantize, **bounded async
  upload queue that drops rather than blocks**. ROS 1 and ROS 2 adapters both
  stream `POST /api/adapter/keyframe`. Descriptor computation lives in the
  back-end (the tested copy); adapters send the cloud and pose.
- Server ingress: `POST /api/adapter/keyframe`, validating identity (query
  `robot_id` must match the blob) and forwarding the opaque body. The server
  stays a dumb pipe and ROS-free. `POST /api/slam/update` adopts the optimized
  `T_world_map` and membership.
- The SLAM back-end is its own process (`python -m swarmdeck_slam`, Docker
  service `slam`). Production loop closures use isotropic information — the
  Hessian weighting is the tracked ATE xfail; isotropic is what actually
  improves it until real Ouster bags exist to calibrate against.

`configs/hardware_fleet.yaml` is `merge_mode: graph`. Local maps keep flowing
with no registration. The merged view stays empty until two robots close a
verified loop, then occupancy is rendered from the joint trajectory.

Offline tests: synthetic two-robot fleet through the wire format merges into
one component and one grid; disjoint buildings never merge; identity mismatch
is 400; a down SLAM process does not 500 the adapter.

### Phase 3 — Robot-side front-end unification

Every robot has an Ouster with a time-synced internal IMU, which is the lever
that makes one front-end possible across the fleet. Target FAST-LIO2 or DLIO.

Ignore this repo's own DLIO benchmark: simulated clouds lack per-point
timestamps, disabling the de-skewing that is DLIO's central advantage. Real
Ouster clouds have them.

Gravity alignment from the IMU should also remove the 2.52 deg floor tilt
calibrated in `adapters/adapter_ros1/config/scout_mini.yaml` — that tilt is a
front-end defect, not a sloped room.

**tars stays on ROS 1.** Run the front-end in its own container reading the
Ouster directly and leave the Noetic base driver alone. Mapping middleware and
control middleware do not need to match; pretending they do is what forced the
`ros1_bridge` problem.

### Phase 4 — Navigation

**Merged occupancy goes to Nav2's global planner; local costmaps stay live.**
The pose-graph process renders one grid per multi-robot component. The server
warps that grid into each robot's own map frame (`GET /api/map/nav/<id>`).
Adapters publish it as a latched OccupancyGrid on `/<ns>/global_map` (sim) or
`/global_map` (hardware). Nav2's `static_layer` reads that topic. The local
costmap still has no static layer — collision authority is live lidar/bumper
only. Operator `navigate_to` goals are converted with `T_world_map` into the
robot's frame, same as before.

Sim is wired like hardware: `adapter_sim` streams keyframes from `/scan` (or
`/scan/points` when the 3D cloud is live), `4robot.yaml` / `2robot.yaml` are
`merge_mode: graph`, and `make up-sim` starts the slam process.

### Phase 5 — Legacy removal

Only once Phase 1-4 is validated end to end. Removes `registration.py`, the
`auto`/`cslam` merge modes, and the grid-registration path through `service.py`.

Grid registration is **retained as an independent diagnostic**: when the pose
graph and a blind grid correlation disagree, that is a warning light worth having.

### Phase 6 — Hardware validation

- Multi-robot MCAP recording, time-aligned across robots. **Pull this forward if
  anything slips** — without bags, every back-end iteration costs a full fleet
  bring-up; with them, iteration is seconds and deterministic.
- Ground truth: surveyed control points at minimum, total station or mocap if
  available.
- Then: ATE/RPE per robot, inter-robot transform error against surveyed points,
  map-consistency proxies.

## Research framing

The architecture is built to support the ablation ladder directly:

| Run | Configuration |
|---|---|
| A | Independent maps, no sharing (baseline) |
| B | Independent maps + shared frontier/goal allocation |
| C | B + collaborative pose-graph correction fed back into each robot's own SLAM |
| D | C + merged grid into global planning, additive-only, gated |

Metrics: coverage time, redundant-area fraction, total path length, inter-robot
transform error vs surveyed landmarks, replan count, interventions.

`evaluation.py` treats labelled side-by-side runs as a first-class case, so the
ladder is a harness feature rather than a manual spreadsheet.

## Testing strategy

Pose-graph code is unusually easy to test wrongly. A hand-built two-pose fixture
passes against implementations that are mirrored, transposed, or
rotation-first-when-they-should-be-translation-first, because with one edge there
is nothing to be inconsistent with.

Every test therefore runs against `synthetic.py`: a closed loop, exact ground
truth, injected random-walk drift (not white noise — white noise cannot
distinguish an optimizer that closes loops from one that merely smooths), and
approximated occlusion (without it every robot sees through every wall and loop
closure looks far more reliable than it is on hardware).

Required in every module: a test that fails on inverted transform direction, and
a measured scaling bound.
