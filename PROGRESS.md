# SwarmDeck mapping — progress and handoff

Last updated 2026-08-24, evening. Written so another agent (or the same
human, a day later) can pick this up without re-deriving anything.

## The goal, stated precisely

Three things, in priority order:

1. **A good local map per robot, visible in the SwarmDeck UI.** Each robot's own
   map, its own frame, no merging involved. This is the immediate goal.
2. **A good single-robot map with loop closure.** A local map that stays
   self-consistent as the robot re-visits places, instead of smearing walls as
   odometry drifts.
3. **A good merged multi-robot map**, via pose-graph optimization across robots.

The important thing to understand -- and the reason the work so far looks
research-heavy for a goal that sounds simple -- is that **(2) and (3) are the
same machinery**. A pose graph that closes loops within one robot's trajectory
is the identical mechanism that closes loops between two robots' trajectories;
the only difference is whether the two keyframes in a constraint carry the same
robot id. So the collaborative SLAM back-end is not a separate research project
sitting beside "make the local map good" -- it *is* how the local map gets good.

Goal (1) is different in kind: it needs no optimization at all, only working
sensor plumbing. That is why it is first, and why it is nearly done.

## The governing architectural decision

**Stop merging maps. Merge trajectories.**

An occupancy grid is not data -- it is a *rendering* of (trajectory x sensor
returns). Rasterizing a scan throws away the pose information, and grid
registration is an attempt to recover it afterwards by correlating pixels. That
is a harder, lossier version of a problem we already had the answer to. It is
why the old `auto` mode is fragile on symmetric layouts, needs known-free cells
as a tiebreaker, and refuses near-disjoint coverage.

The previous `cslam` attempt failed the same way in sharper form: RTAB-Map
produced the grids, Swarm-SLAM produced the trajectory, and they disagreed by
11-16 m (recorded in `docs/architecture/collaborative-slam.md`).

In the new design there is **no registration step anywhere**. Poses are optimized
in one joint graph; occupancy is rendered from the optimized poses; the map
cannot disagree with the trajectory that produced it.

Full design: `docs/architecture/collaborative-mapping-plan.md`.

---

## Where things actually stand

### Built and tested: the SLAM back-end (`slam/`)

A new distribution, `swarmdeck-slam`. **99 tests pass, 1 tracked xfail.**
Run with `make test-slam`; bootstrap with `make install-slam`.

| Module | Does | State |
|---|---|---|
| `types.py` | Domain types, SE(3) helpers, frame convention, solver Protocol | contract; do not edit casually |
| `descriptors.py` | Scan Context + ring key + KD-tree index, returns a yaw prior | 14 tests |
| `verify.py` | GICP geometric verification, rejection gates, information matrix | 10 tests |
| `graph.py` | gtsam pose graph, GNC, PCM, union-find components | 14 tests |
| `render.py` | Occupancy rendered from optimized poses, per component | 10 tests |
| `evaluation.py` | ATE / RPE / inter-robot transform error / component scoring | 40 tests |
| `backend.py` | Online ingest of wire packets → optimize → render | 5 tests |
| `service.py` | HTTP process: `POST /keyframe`, publishes maps back | own process |
| `tests/test_integration.py` | The whole pipeline end to end | 6 pass, 1 xfail |

Measured, not assumed:

- True loop closures recovered at **1.3 cm median** error vs ground truth.
- Robots sharing a building **merge**; robots in two different buildings are
  **never merged**, with *zero* cross-building closures surviving geometric
  verification (so PCM never even has to fire as the second line of defense).
- 800 keyframes optimize in ~0.13 s.
- Descriptor query ~1.1 ms, flat from 200 to 5000 keyframes.
- `graph.py` fed exact ground-truth edges returns exact ground truth (ATE 0.0).
- The *online* path (encode → decode → `CollaborativeBackend`) recovers the
  same merge/no-merge behaviour on the synthetic two-robot and disjoint fleets.

### Bridged: research stack ↔ live stack (Phase B, tested offline)

This was the critical gap. It is now wired, and it is tested without a robot:

1. **Adapter keyframe producer** (`adapters/keyframe_producer.py`). Voxel
   downsample, transform the registered cloud into the base frame, encode,
   bounded drop-oldest queue. ROS 1 and ROS 2 adapters both call it from
   `_on_map_cloud` and drain it from `tx_maps` without blocking telemetry.
   Default cap: 0.5 m or 15 deg, min 2 s apart (~0.5 Hz).
2. **Server ingress** `POST /api/adapter/keyframe?robot_id=`. Peeks the JSON
   header only (no zip-bomb on the event loop), rejects identity mismatch, and
   forwards the opaque body to the SLAM process through a drop-queue. A down
   SLAM process returns 200, not 500, and never stalls the adapter.
3. **SLAM process** `python -m swarmdeck_slam` (Docker service `slam`, port
   8090). Python 3.12 / numpy 1.26, never inside the server. Production loop
   closures use **isotropic** information (the Hessian path is the tracked
   ATE xfail; isotropic is the weighting that actually improves ATE). Pushes
   `POST /api/slam/update` + `POST /api/adapter/global_map` for the majority
   multi-robot component only. Singletons stay off the merged map on purpose.

`configs/hardware_fleet.yaml` is `merge_mode: graph`. No grid registration, so
ingest cannot stall. Local maps keep flowing. The merged view is empty until
two robots close **two** corroborating inter-robot loops (PCM min clique size
is 2). `make up-deploy` now starts this config plus the slam service.

Offline tests: 422 server+adapter tests pass; 99 slam tests pass, 1 xfail.

### Live fleet, as of the previous session (not re-checked; no robot tonight)

All four robots are registered with the backend and connected.

| Robot | Local map at `/api/map/local/<id>` | Notes |
|---|---|---|
| `botman_0` | **200** -- 600x600, seq 757 | working |
| `tars_0` | **200** -- 2485x1593, seq 604 | working, but see below |
| `aslan_0` | 404 | SuperOdometry produces nothing |
| `spot_0` | 404 | containers started under a broken clock |

So **half the immediate goal is already met** -- botman and tars should be
visible in the UI's per-robot map view right now, with no code change. Confirm
that in the browser before touching anything; it sets the baseline.

---

## Open problems, with diagnosis

### 1. The server is running a SIMULATION config against real hardware

It was started with `/app/configs/4robot.yaml`, which declares `robot_0..robot_3`
and elects `robot_0` as the merge reference. **No such robot ever registers**, so
every real robot is permanently a non-reference robot -- and in `merge_mode:
auto` that re-runs grid registration on *every* map/cloud ingest, measured at
**2.44 s per call** (`adapters/adapter_ros2/config/bunker.yaml:107`). That is the
throughput trap that previously stalled `_ingest_lock`, blew adapter HTTP
timeouts, and flapped robots offline.

**Prepared and now the deploy default:** `configs/hardware_fleet.yaml` --
`robot_count: 0`, `merge_mode: graph`, no start poses. `make up-deploy` uses
this config and starts the `slam` service. Graph mode performs no registration,
so ingest never blocks and every robot's own map keeps flowing.

```bash
make up-deploy
# equivalent:
SWARMDECK_CONFIG=/app/configs/hardware_fleet.yaml \
  docker compose -f deploy/compose/docker-compose.yml \
                 -f deploy/compose/docker-compose.zenoh.yml \
  up -d --force-recreate server ui slam zenoh-router mediamtx
```

Do **not** switch it to `auto` to get merging "for free". That is the exact
regression this whole rebuild exists to undo. Merging now comes from the
pose-graph process, not from grid correlation.

### 2. `aslan_0` -- SuperOdometry advertises but publishes nothing

`/registered_scan` and `/laser_odometry` appear in `ros2 topic list`, but
`ros2 topic hz /registered_scan` returns nothing in 10 s. **No data at all.**

The adapter log is full of `"map" passed to lookupTransform argument
target_frame does not exist`. That is a **symptom, not the cause** -- the `map`
frame is missing because SuperOdometry is not running its mapping output, not
because of a frame-name misconfiguration. Aslan's config is byte-identical to
botman's in every frame and topic field (`map_frame: map`, `base_frame:
os_lidar`, `map_cloud: /registered_scan`), and botman works. So this is robot
state, not configuration.

Start here: restart `swarmdeck-aslan-slam`, then re-check `ros2 topic hz
/registered_scan`. If still silent, verify the Ouster on VLAN 2 is actually
feeding it -- no lidar in, no registered scan out. Note `swarmdeck-aslan-nav2`
also reports `unhealthy`, which may or may not be related.

### 3. `spot_0` -- containers started under a broken system clock

`docker ps` reports every SwarmDeck container as **"Up 56 years"**. The clock is
correct *now* (`timedatectl` says synchronized, and it matches the operator host
exactly), which means those containers **started while the clock was wrong and
then lived through a ~56-year time jump**. ROS nodes do not survive that in any
meaningful sense: message timestamps and TF lookups are timestamp-based, and a
TF buffer populated with far-future stamps will never satisfy a lookup at the
current time.

Restart spot's whole SwarmDeck stack, then re-check the local map. This one is
probably a quick win, and it was the least-investigated robot.

### 4. `tars_0`'s map is suspiciously large

2485x1593 cells at 0.05 m = **124 m x 80 m**, for robots the operator says are
all in the same room. Either LVI-SAM's accumulated map is retaining a much older
session, or there is drift/noise inflating the bounds. Worth a look, but it is
producing a map, so it is lower priority than the two 404s.

Related and already documented: `adapters/adapter_ros1/config/scout_mini.yaml`
records a floor plane pitched **2.52 degrees** from a lidar/IMU extrinsic error,
which `docs/robots/fleet.md` flags as the reason not to trust tars's displayed
pose. A unified gravity-aligned front-end (Phase 3) should remove that.

### 5. Tracked xfail: optimization currently RAISES ATE

`slam/tests/test_integration.py::test_optimized_poses_beat_raw_odometry` is a
**strict xfail**, deliberately not tuned away. Optimizing with real closures
raises translation ATE by ~1.2-1.4x instead of lowering it, consistently across
injected drift from 0.012 to 0.15 m/m.

The pipeline was bisected and the cause is isolated:

| Input | Result |
|---|---|
| Exact ground-truth edges into `graph.py` | ATE **0.0** -- solver is correct |
| Real closure transforms vs ground truth | **1.3 cm** median -- geometry is correct |
| Ground-truth transforms + `verify.py` information | still degrades (1.20x) |
| Real transforms + **isotropic** information | **improves (0.93x)** |

So the fault is in the **information matrices**, not the transforms and not the
solver. GICP's Hessian arrives with roughly a **30:1 rotation-to-translation
ratio**, over-constraining orientation relative to position. Overall magnitude is
not the lever: sweeping `verify.py`'s `info_scale` from 1 to 100 never recovers
an improvement, and at 100 every closure is rejected outright.

Correct weighting depends on a real sensor noise model. Calibrating it against
the synthetic fixture would fit the fixture's noise rather than the Ouster's,
which is why it is left open. A `< 3x` divergence guard sits beside it so that a
calibration error of centimetres and a genuinely broken solver cannot look alike
in the suite. **This blocks goals (2) and (3), and it unblocks on real recorded
data -- see Phase 6.**

---

## The road, in order

### Phase A -- finish the local-map goal (tomorrow morning, on hardware)

1. `make up-deploy` so the server is on `hardware_fleet.yaml` with the slam
   process. Confirm `curl -fsS http://localhost:8090/health`.
2. Confirm botman and tars local maps render in the UI. Baseline first.
3. Restart spot's stack -- suspected clock-jump casualty, likely quick.
4. Restart `swarmdeck-aslan-slam`; if still silent, chase the Ouster feed.

Exit: four robots, four local maps, visible in the UI. Merged map may still be
empty -- that is correct until two robots overlap enough to close two loops.

### Phase B -- bridge the research stack to the live stack -- DONE offline

Wired and tested without a robot. Tomorrow's hardware check:

1. Drive botman ~5 m, confirm `curl http://localhost:8090/status` shows
   growing `keyframes` for `botman_0`. Adapter log should not mention
   stalled uploads.
2. Drive tars through the same space. After two corroborating inter-robot
   closures, `/api/map/status` `global_members` should list both and the
   merged map should show one grid, not two overlaid at the origin.
3. If they never overlap, the merged map stays empty. That is a correct
   statement of ignorance, not a failure.

PCM will not merge on a single inter-robot closure. Drive a short loop that
revisits overlap, not a one-shot drive-by.

### Phase C -- record real data, then fix the ATE issue

**Multi-robot MCAP recording, time-aligned across robots.** Pull this forward if
anything slips. Without bags, every back-end iteration costs a full fleet
bring-up; with them, iteration is seconds and deterministic. Then surveyed
control points for ground truth (total station or mocap if available).

With real bags, the information-matrix issue (#5) becomes calibratable, which is
what unblocks single-robot loop closure and multi-robot merging.

### Phase D -- unify the front-end

Every robot has an Ouster with a time-synced internal IMU. That is the lever that
makes one front-end possible across a fleet currently running four different SLAM
systems (LVI-SAM / SuperOdometry x2 / LIO-SAM) -- and heterogeneous SLAM is the
root cause of unmergeable maps, because you cannot align two estimates that were
never estimating the same thing. Target FAST-LIO2 or DLIO.

Ignore this repo's own DLIO benchmark: simulated clouds lack per-point
timestamps, disabling the de-skewing that is DLIO's whole advantage. Real Ouster
clouds have them.

**tars stays on ROS 1.** Run the front-end in its own container reading the
Ouster directly; leave the Noetic base driver alone. Mapping middleware and
control middleware do not have to match, and pretending they do is what forced
the `ros1_bridge` problem.

### Phase E -- navigation, then legacy removal

Server plans globally, robot executes locally: hand each robot a waypoint
sequence of ordinary `navigate_to` goals. Needs no new protocol direction, works
identically on Nav2 and on tars's reactive `local_planner` (which has no global
planner at all -- `adjacentRange` is 5 m), and structurally preserves the safety
property that the server can only ever suggest *where*, never command *how*.

Only then remove `registration.py` and the `auto`/`cslam` merge modes. Keep grid
registration as an **independent diagnostic**: when the pose graph and a blind
grid correlation disagree, that is a warning light worth having.

---

## Things that will bite you

- **`slam/` is pinned to Python 3.12 and the pin is load-bearing.** gtsam 4.2.2
  declares `numpy<2`, and it is real, not stale metadata: under numpy 2.x it
  imports fine and then **segfaults** on the first array-marshalling call, with a
  bare SIGSEGV and no Python traceback. The server venv is 3.13/numpy 2.x. Never
  put gtsam in it. This is a second, independent reason the back-end is its own
  process.
- **Frame convention, enforced package-wide:** every transform is named `T_a_b`
  and maps points in frame `b` into frame `a`. Information matrices are 6x6 in
  gtsam Pose3 tangent order, **rotation first**. Direction and ordering errors
  fail *silently* -- the graph still optimizes, it just converges to a mirrored
  or mis-weighted map. Every module carries a test that fails on inversion; keep
  that up.
- **Parallel module development hides integration bugs.** All five modules passed
  their own suites while the assembled system had a real defect (#5). Each side
  tests against its own self-consistent reading of a shared convention, and both
  readings can be self-consistent while disagreeing. `test_integration.py` exists
  for this reason.
- **Integration tests that pool populations produce false alarms.** An early
  version of the pipeline check reported a catastrophic frame-convention error
  that did not exist, because it averaged true correspondences together with
  known false positives. Stratify by ground-truth separation. A sloppy
  integration test is worse than none.
- **PCM will not merge on one inter-robot closure.** `min_pcm_clique_size` is 2:
  a single loop between two robots is exactly the case PCM exists to reject. A
  hardware test that drive-bys once and expects a merged map will wait forever.
  Overlap twice.
- **Production loop closures use isotropic information.** The Hessian path is
  still the default in `VerifyConfig` (and in the xfail). The live back-end
  opts into isotropic because that is the weighting that improves ATE. Do not
  "fix" the xfail by pointing the tests at isotropic -- that would hide the
  calibration problem rather than solving it.
- **Adapters must be redeployed**, not only the server. Keyframe production
  lives on the robot. A server running `merge_mode: graph` against adapters
  that have not been updated will still show local maps and will never merge.
