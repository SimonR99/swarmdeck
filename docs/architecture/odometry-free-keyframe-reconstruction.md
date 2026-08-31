# Odometry-free reconstruction of saved keyframes

## Purpose

This path reconstructs a map when the pose stored with each keyframe is
unusable. It is designed for the real hardware failure modes seen in the field:

- wheel/SLAM odometry drifts or jumps;
- SLAM is stopped and restarted, so two consecutive sequence numbers can live
  in unrelated map frames;
- indoor corridors admit geometrically excellent registrations at both the
  true heading and a heading rotated by 180 degrees;
- repeated walls can produce several plausible translations.

The implementation is in `slam/swarmdeck_slam/odom_free.py` and
`slam/swarmdeck_slam/reconstruction.py`. The offline entry point is
`slam/tools/reconstruct_odom_free.py`.

## Inputs that are and are not trusted

The reconstruction uses:

- each point cloud in its local keyframe/base frame;
- robot identity;
- sequence number, only to propose temporal neighbours;
- timestamp, only to reject motion faster than the physical robot and to split
  long capture gaps;
- optionally, `t_odom_base` as a **weak mode vote** after registration.

Pair registration still never reads `t_odom_base`. There is no pose parameter
in that API, so recorded odometry cannot become an ICP seed or a graph
factor. After GICP has returned several geometrically valid modes, Viterbi
may prefer the mode whose hop agrees with a kinematically plausible odom
step. A 20 m jump, a yaw spike, or a missing pose is ignored -- the chain
falls back to geometry and the zero-yaw prior. Occupancy is rasterized at
the reconstructed poses, not at `T_world_map @ t_odom_base`.


Sequence adjacency is not treated as proof that two keyframes are connected.
A long timestamp gap or unsupported registration creates a new fragment. The
global matcher may reconnect it later, but only from independent geometric
evidence.

## Algorithm

### 1. Prepare structural clouds

For the confirmed Bunker capture, points are restricted to approximately
0.15--1.80 m above the floor (`z=-0.37..1.28` in the old Ouster/base frame),
0.8--18 m in radial range, then voxelized at 0.20 m. This removes the vehicle,
most floor returns, distant clutter, and redundant samples while retaining
walls, doorways, shelves, and other stable structure.

Current packets carry `ground_z`, `min_height`, and `max_height`; those
producer-measured limits override the legacy fallback independently for each
keyframe. This matters for mixed platforms whose lidars sit at different
heights. Legacy Gazebo captures use an explicit `--min-z 0.08 --max-z 2.20`.

Each prepared cloud stores two coarse representations:

- a 20 x 60 Scan Context descriptor for candidate yaw/place retrieval;
- a 0.25 m bird's-eye binary occupancy image for translation correlation.

### 2. Keep several pairwise transformations

For target cloud A and source cloud B, registration returns several
`T_A_B` hypotheses:

1. Score every Scan Context sector shift and retain distinct yaw modes. Do not
   discard the runner-up; on the real Aslan scans it is frequently the correct
   alternative to a 180-degree corridor match.
2. Rotate B by each yaw and cross-correlate its bird's-eye occupancy with A by
   FFT. Retain several separated translation peaks. This supplies translation
   seeds beyond GICP's roughly one-metre convergence basin.
3. Refine every yaw/translation seed with GICP.
4. Re-score independently with symmetric nearest-neighbour overlap and RMSE.
5. Reject weak modes and deduplicate modes that converged to the same SE(3)
   transform.

GICP residual and inlier count validate local geometric fit; they do not prove
that the mode is globally correct. A symmetric hallway can give a near-perfect
residual to two incompatible headings.

### 3. Build local fragments with discrete mode selection

Adjacent keyframes are eligible only when sequence is consecutive and the
capture gap is at most 60 seconds. Candidate transforms must fit deliberately
generous Bunker kinematic bounds. These constraints use elapsed time and robot
physics, not a reported displacement.

The 60-second value is measured, not arbitrary. Gazebo suppresses keyframes
during turns and produced valid 10--42 second gaps in `3d-run-01`, with less
than one metre of true displacement. The former 10-second cutoff split that
run into 20 fragments and enabled a globally consistent 180-degree corridor
mistake. Sixty seconds leaves those scans in the temporal Viterbi chain; a
longer outage, sequence reset, or failed geometric registration still splits.

Within each eligible run, Viterbi dynamic programming selects a sequence of
registration modes. Its cost combines:

- pair-registration quality;
- linear velocity and yaw magnitude;
- changes in linear velocity and yaw rate;
- agreement with independently registered skip-one pairs, forming
  three-keyframe cycles;
- when present, agreement with a kinematically plausible odometry hop
  (ignored if the hop could not be robot motion).

An isolated 180-degree flip therefore costs much more than a sustained physical
turn. If no physically plausible pair mode survives, the run is split rather
than bridged with a weak factor.

### 4. Reconnect fragments by consensus

Candidate fragment pairs come from Scan Context nearest neighbours plus dense
keyframe windows around known temporal breaks. Every matched keyframe pair votes
for an implied transform between its two fragment frames.

A fragment merge is accepted only when one SE(3) cluster has:

- at least three distinct keyframe-pair votes;
- at least two distinct keyframes on each side;
- at least 0.60 m of spatial span on each side;
- a clear margin over any competing transform cluster.

The first three requirements prevent one lucky scan from being counted several
times. The last explicitly rejects the 180-degree/repeated-corridor ambiguity.
When two same-robot fragments lie on opposite sides of one sequence boundary,
the winning window consensus must also agree with the best direct registration
of the exact last-to-first scan pair. This prevents several correlated window
matches from outvoting the actual transition into a repeated corridor copy.
Accepted proposal factors are optimized with a robust Huber pose graph. Each
disconnected graph component receives a separate origin and is rendered as a
separate map; the software never invents a transform merely to show one merged
image.

Cross-robot connections have one additional gate. Several adjacent scans at a
single intersection are correlated observations, even if each pair registers
well. A fleet-frame merge needs compatible encounters separated by at least
3 m on both robots. Those encounters and the paths between them form a
cross-robot cycle. They may be two fragment connections, or spatially separated
scan votes inside one connection when both robots remained continuous. This is
what allows `3d-run-02`, where each robot is exactly one fragment, to merge
without weakening the single-intersection rejection on hardware.

### 5. Close loops and optimize every keyframe

Within each long fragment, Scan Context also proposes non-local pairs separated
by at least 20 sequence numbers. Direct GICP measurements become loop factors
only when they agree with the geometry-only temporal path within 0.75 m and
15 degrees. This loose path gate lets a revisit remove accumulated scan drift
without letting a visually identical parallel corridor teleport the graph.

The final GTSAM graph contains every selected temporal registration, every
verified fragment/cross-robot proposal, and every accepted intra-fragment loop.
Huber factors suppress residual outliers. Optimization is per keyframe, so loop
error is distributed along the trajectory rather than moving a whole fragment
as one rigid block. Registrations from one rendezvous remain correlated even
when many scan pairs support them, so their aggregate information is normalized
by cluster size; otherwise a 24-pair encounter can deform a more accurate local
scan chain simply by being sampled densely.

## Four-robot coordinated results (2026-08-30)

`coordinated-gt-run-10` is a clean four-robot Gazebo capture produced by the
joint frontier planner. Ground truth was recorded in a separate evaluation
process and was never subscribed to by the planner or reconstruction service.
The final cold replay contains 288 calibrated keyframes:

- all 288 keyframes and all four robots are in one verified component;
- 7 fragment links survive the full gates (6 inter-robot, 1 same-robot
  continuation), plus 48 intra-fragment loop closures;
- joint translation ATE is **0.0250 m RMSE**, 0.0190 m median, and 0.0778 m
  maximum; joint rotation ATE is **0.632 degrees RMSE**;
- the absolute pose set using only surveyed starts to fix gauge is **0.0267 m
  RMSE**, with 0.0731 m maximum translation error;
- the largest disagreement between independently aligned robot frames is
  0.0488 m and 0.086 degrees;
- the surveyed-world wall-surface map score at 0.10 m tolerance is **F1 0.933,
  precision 0.990, recall 0.882**, with 0.112 m symmetric p95. Filled-cell IoU
  is reported separately
  because lidar observes wall faces while the SDF contains filled collision
  boxes.

Removing every recorded `t_odom_base` value produces identical accepted and
rejected constraints, fragment and optimized poses, components, render
metadata, and SHA-256 hashes for all five PNGs. Thus odometry contributes no
decision or pose to this capture. An exact incremental arrival replay merges
robot 3 at keyframe 100 and remains one component through keyframe 288; retained
pair identities are re-registered and re-gated on every solve, not retained as
unconditional factors.

The capture also exposed three aliasing edge cases now covered by regressions:

- fixed descriptor budgets are sampled across both fragment trajectories so a
  repeated doorway cannot evict a spatially distributed rendezvous;
- only structurally valid clusters compete, and an exact adjacent-boundary
  registration may select a lower-ranked but independently supported mode;
- a coarse-start-corroborated primary connection outranks a contradictory
  high-score alias to a later fragment. Coarse starts still never seed ICP or
  enter the pose graph as factors.

## Gazebo ground-truth results (2026-08-26)

`3d-run-01` contains 239 keyframes from two robots and surveyed Gazebo poses.
The reconstruction uses no saved pose or odometry:

- 235 keyframes form the verified two-robot component; four weak tail scans
  remain a separate reviewable component;
- 3 fragment links, including 2 independently verified cross-robot links;
- 24 intra-fragment loop closures;
- joint translation ATE: **0.036 m RMSE**, 0.019 m median, 0.218 m maximum;
- joint rotation ATE: **0.489 degrees RMSE**;
- independently fitted robot frames disagree by 0.063 m and 0.298 degrees.

`3d-run-02` has 282 keyframes and no saved truth file. Both robots remain one
fragment. Twenty-four spatially distributed scan votes verify their single
fragment-pair connection, 51 intra-fragment loops are accepted, and all 282
keyframes form one visually coherent component. This is a structural check,
not a metric accuracy claim, because the capture has no ground truth.

A fresh run, `gazebo-odomfree-20260826`, was then generated after the initial
algorithm and thresholds were chosen. It contains 221 calibrated keyframes and
9,982 synchronized truth poses from a ten-minute exploration:

- 215 keyframes (97.3%) form the verified two-robot component;
- one 24-vote cross-robot connection and 25 intra-fragment loops;
- joint translation ATE: **0.052 m RMSE**, 0.028 m median, 0.376 m maximum;
- joint rotation ATE: **0.513 degrees RMSE**;
- independently fitted robot frames disagree by 0.076 m and 0.672 degrees;
- six startup frames are quarantined in two small components because the only
  adjacent scan mode has score 0.387 and a 180-degree ambiguity.

The startup quarantine is intentional: before the weak-boundary gate, forcing
those six scans into the graph increased their component error to 0.70 m and
103 degrees while contributing almost no map coverage. Because that failure
was used to add the weak-boundary gate, the final number is a development-run
result, not an untouched test-set claim; another fresh seeded run is still the
right release gate.

### Severe odometry-fault replay (2026-08-27)

The same fresh Gazebo keyframes were copied into a deterministic fault-injection
dataset. The compressed point-cloud and descriptor body of every packet is
byte-identical to the clean capture; only `t_odom_base` differs. Each robot has
an independent fault history containing sample jitter (0.12 m / 4 degrees),
accumulating translation/yaw bias, a persistent wheel-slip jump, an encoder
spike, a complete odometry reset, and a second reset representing SLAM
reconnection. The largest single reported steps are:

- robot 0: 23.44 m and 163.83 degrees;
- robot 1: 19.20 m and 139.71 degrees.

This is intentionally more severe than realistic wheel slip. Hops that fail
the kinematic gate (the 20 m jumps and 160-degree spikes) are ignored, so
they cannot flip a corridor alias. Small jitter inside the gate may still
break a remaining mode tie; that is the intended "minor help". The
reconstruction is therefore invariant to catastrophic odometry, not to
every perturbation of `t_odom_base`.

The 2026-08-27 fault-injection replay (before the weak mode vote) was
byte-identical to the clean run:

- all accepted/rejected constraints, fragment poses, optimized keyframe poses,
  components, and render metadata match the clean run exactly;
- all three rendered PNG files have identical SHA-256 hashes;
- the verified 215-keyframe two-robot component remains at 0.0518 m translation
  ATE and 0.513 degrees rotation ATE;
- robot-frame disagreement remains 0.0757 m / 0.672 degrees;
- cold reconstruction took 84.4 seconds for 515 required pair registrations.

As a control, the current odometry-based production Hessian backend was replayed
at its real optimization cadence on the same corrupted packets. It still put
both robots in one graph, but produced 5.046 m translation ATE, 61.20 degrees
rotation ATE, 19.85 m maximum error, and 1.921 m / 7.694 degrees of relative
robot-frame error. Its merged map contains many rotated copies of the building.

The reproducible inputs and outputs are:

- `sessions/analysis/datasets/gazebo-odom-jitter-20260827/`;
- `sessions/analysis/gazebo-odom-jitter-20260827/`;
- `sessions/analysis/gazebo-odom-jitter-production-hessian-20260827/`.

## Hardware result (2026-08-26)

The calibration and evaluation dataset is only
`sessions/captures/hw-run-01`, the confirmed real Botman/Aslan Bunker capture:

- 553 keyframes total: 377 Botman and 176 Aslan;
- 37 conservative temporal fragments with the measured 60-second and weak-edge
  policies;
- 1,950 cached geometry-only pair registrations;
- 21 accepted and 59 rejected fragment-pair candidates;
- 19 final independent components;
- largest Botman component: 342 keyframes;
- largest Aslan component: 54 keyframes;
- zero accepted Botman-to-Aslan connections.

The Botman component reconstructs the long rectangular floor loop. An earlier
version incorrectly overlaid an Aslan intersection segment: it counted several
adjacent scans at one repetitive intersection as independent cross-robot
evidence. The two cross-robot connections were graph bridges and did not form a
cycle. Both are now explicitly rejected as
`inter-robot merge has no independent cycle`.

An additional false Botman branch was traced to fragment `botman_0:017`
(sequences 203--233). Its only connection to the main floor was a four-vote
window consensus across the 233->234 boundary. That consensus implied 3.79 m
and -177 degrees for the direct transition, while the best adjacent-cloud mode
was near +2 degrees. The fragment-frame solutions differed by 4.64 m and 179.75
degrees. The boundary-consistency gate now rejects that link and leaves the
31-keyframe corridor as a separately reviewable component instead of drawing it
through the main Botman map.

A held-out check registered every third keyframe directly, so those pair factors
were not used by the temporal selector. Against the nearest surviving geometric
mode over 81 checks inside Botman's best component:

| prediction | translation median | translation p90 | rotation median | rotation p90 |
|---|---:|---:|---:|---:|
| geometry-only | 0.010 m | 0.124 m | 0.20 deg | 0.64 deg |
| recorded hardware pose (diagnostic only) | 0.016 m | 0.046 m | 0.27 deg | 0.66 deg |

The nine available checks inside Aslan's best component are similarly close:
2.5 cm translation median and 0.39 degree rotation median for geometry-only,
versus 2.2 cm and 0.31 degree for recorded poses. These measurements are not
proof of metric ground-truth accuracy; the capture has no survey truth. More
importantly, the system does not force Aslan into Botman's frame when the saved
geometry cannot support a cross-robot cycle.

Generated evidence is under
`sessions/analysis/hw-run-01-odom-free-gap60/` (the earlier 10-second control is
retained in `hw-run-01-odom-free/`):

- `manifest.json`: every included keyframe, fragment pose, boundary, component,
  and accepted/rejected fragment connection;
- `best-botman_0.png`: largest verified Botman component;
- `best-aslan_0.png`: largest verified Aslan component, in its own frame;
- `comparison-botman_0.png` and `comparison-aslan_0.png`: post-hoc comparison
  with recorded poses; recorded poses are not reconstruction inputs;
- `heldout-consistency.json`: independent gap-three consistency statistics;
- `registrations.pickle`: resumable geometry-only pair cache.

## Run and review

From `slam/`:

```bash
.venv/bin/python tools/reconstruct_odom_free.py \
  ../sessions/captures/hw-run-01 \
  --output ../sessions/analysis/hw-run-01-odom-free
```

Create a repeatable odometry-jitter/slip/reset replay from any saved capture:

```bash
.venv/bin/python tools/inject_odometry_faults.py \
  ../sessions/captures/gazebo-odomfree-20260826 \
  --output ../sessions/analysis/datasets/my-odometry-fault-test \
  --seed 20260827
```

The generated `odometry_faults.json` records every injected event and severity,
source/output hashes, and confirms that all compressed scan bodies are
byte-identical.

Select one robot or a sequence interval:

```bash
.venv/bin/python tools/reconstruct_odom_free.py \
  ../sessions/captures/hw-run-01 \
  --output /tmp/botman-review \
  --robot botman_0 --start-seq 200 --end-seq 300
```

Select exact sessions from an append-only capture without mixing replays or
sequence resets:

```bash
.venv/bin/python tools/reconstruct_odom_free.py \
  ../sessions/captures/hw-run-02 \
  --output /tmp/selected-hardware-sessions \
  --trajectory botman_0@1787767117-442cb204 \
  --trajectory aslan_0@1787767119-838ee945
```

Exclude one or more suspicious ranges without modifying or deleting the source
capture:

```bash
.venv/bin/python tools/reconstruct_odom_free.py \
  ../sessions/captures/hw-run-01 \
  --output /tmp/hardware-with-exclusions \
  --exclude botman_0:180-202 \
  --exclude aslan_0:93-100
```

The manifest is the review surface for programmatic tooling. A UI can expose
the same robot/sequence selection and exclusion fields, then render each
component independently. Deleting a robot from a merged map should mean
excluding its keyframes and rebuilding the graph; it must not erase the saved
capture or subtract pixels from an already rasterized merged image.

## Known limits and next data to collect

- Exact symmetry is unobservable from geometry alone. If all supporting scans
  see the same symmetric corridor and no cycle reaches distinctive structure,
  the correct result is multiple components/hypotheses, not a guessed heading.
- Timestamp-based kinematics assumes the sensor timestamps are monotonic. Bad
  timestamps produce safe fragmentation rather than a wrong transform.
- The optimizer still performs a batch solve, but the production service is
  online: it retains a geometry-only pair cache and the identities of verified
  support pairs as new keyframes arrive. A cold production rebuild of the 288
  keyframe four-robot capture took 83.5 seconds; the cached standalone rebuild
  took 4.4 seconds. Incremental factorization remains future work, and labelled
  hardware captures must validate the current thresholds before deployment.
- Camera images would make this much closer to photogrammetry: visual features
  and learned/global image descriptors can break lidar's 180-degree symmetry.
  They are not present in this saved keyframe format, so the current method is
  lidar-geometric rather than visual photogrammetry.
- Stronger global recovery would come from deliberately capturing overlapping
  revisits around every restart and at corridor junctions. Three or more views
  spanning a few metres are far more valuable than many stationary scans.
