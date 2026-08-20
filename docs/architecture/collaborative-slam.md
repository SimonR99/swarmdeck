# Where the mapping stack stands, and what swarm SLAM would change

Written after an audit of the per-robot SLAM and the multi-robot merge. It records
what the system actually does today, three defects found and fixed, and a staged
migration for robots whose real sensors are **odometry + a 3D point cloud + an RGB
camera** (and, on most platforms, an IMU).

## 1. What exists today

```
wheel odom ─┐
            ├─► SLAM Toolbox (2D)  ─► <ns>/map  ─┬─► Nav2 static layer
planar scan ┘   scan match                       │
                loop closure                     └─► adapter ─HTTP─► backend
                Ceres pose graph                                       mapsvc
                                                                         │
                                              per-robot grids ──► FFT registration
                                                                         │
                                                                   merged grid ─► GUI
```

Each robot corrects its own odometry drift, and does it properly: SLAM Toolbox
scan-matches, closes loops, and optimises a pose graph with Ceres, publishing a
corrected `map_frame → odom`. Within its own frame, each robot's map is sound.

Three things are worth being precise about, because they are easy to overstate.

**The merge is a stitcher, not SLAM.** `mapsvc/registration.py` aligns *finished
occupancy grids* pairwise against one reference robot and never sends anything
back. No robot's drift is corrected by another robot's observations; there is no
shared pose graph, no inter-robot loop closure, and no transitive registration, so
a robot overlapping robot 2 but not the reference can never join the global map.

**Only one sensor localises.** The `PointCloud2` topic and the IMU are bridged to
ROS and, until this change, had no subscriber. The camera drives a preview stream
and a duck detector. Nothing visual or inertial contributes to the pose.

**It is 2D throughout.** SE(2) transforms, occupancy grids, a planar scan. There
is no representation for z, roll, or pitch drift, so nothing can correct them.

## 2. Defects found and fixed

### 2.1 The registration search silently reported 90°-wrong transforms

The correlation score as a function of yaw is very sharp — a yaw error at radius
`r` displaces a point by `r·dyaw`, so the peak is roughly `cell/r` radians wide,
well under a degree on a 15 m map. The coarse sweep sampled yaw every 4°, so it
could miss the true peak entirely and settle on whatever rotational symmetry the
building had. Measured on the project's own test floor plan, **13 of 52 headings
that fall between coarse samples produced a confident answer that was exactly 90°
wrong**, with translation errors up to 1.8 m.

Two things hid it. The six parametrised test yaws are 0, 0, 35, −60, 120, 175 — of
those, 0, −60 and 120 sit exactly on the 4° grid and the other two are 1° off,
inside the peak. And `configs/4robot.yaml` starts robots at yaw 0, 0, π, π, so every
*relative* heading in the demo is 0 or 180°, also exactly on the grid.

The ratio test could not catch it either: it compares rival *translations* within
the winning yaw's correlation surface, and never compares across rotations.

Fixed by a coarse-to-fine search whose every stage is sampled finely enough to
find its own peak — the coarse stage correlates against a *dilated* reference, so
the peak it is looking for is wide enough to be found on a 4° grid — plus a
`yaw_ratio` that ranks distinct rotation hypotheses against each other at a
resolution that can tell them apart.

### 2.2 Matching only occupied cells cannot resolve rotational symmetry

A floor plan and the same plan rotated onto its own symmetry both put walls on
walls. What separates them is *observed free space*: a wrong rotation drives walls
through rooms another robot has already seen to be empty. The reference is now
rasterised signed — occupied positive, known-free negative, unknown zero — with
free-space contradiction weighted equal to wall agreement, and a one-cell guard
band around walls so a correct match is not punished for landing beside a wall
rather than on it.

A `support` gate was also added: the fraction of the smaller map's known area that
is actually shared. A strong score over a sliver of overlap is not evidence, and
this is the ill-posed case the docs already warned about.

Result on the project's own plan, over 52 off-grid headings:

| | confident & correct | confidently wrong | refused |
|---|---|---|---|
| before | 39 | **13** | 0 |
| after | 52 | **0** | 0 |

Accuracy went from ~7.8 cm (the old search's quantisation floor) to **0.1–3 cm and
under 0.1° yaw**, from sub-cell interpolation of both the translation peak and the
yaw curve. On a deliberately 4-fold-symmetric building the new code refuses to
answer, where the old code confidently picked whichever alias it landed on.

### 2.3 Registration blocked the server's event loop

`POST /api/adapter/map` is `async def` and called `map_service.ingest()` inline —
about 190 ms of numpy on a realistic grid, so with three non-reference robots
uploading at 0.5 Hz roughly a quarter of the event loop was consumed in ~190 ms
blocks, stalling every WebSocket including the 5 Hz telemetry broadcast. Ingest is
now offloaded with `asyncio.to_thread` under a lock, and a robot that is already
registered refines its transform in a narrow yaw window instead of re-running a
full 360° search on every upload.

### 2.4 The multi-ring lidar was removed for the wrong reason

`KNOWN_ISSUES.md` recorded that a multi-ring lidar could not feed 2D SLAM through
a height band, and resolved it by making the lidar single-ring. The diagnosis was
right and the resolution was wrong.

Gazebo spreads N vertical samples evenly across `[min_angle, max_angle]`
*inclusive*, so an **even** N puts no ring at elevation 0 — every ring is tilted.
A ring at elevation `e` leaves a band of half-height `h` at range `h/sin(e)` and is
invisible past it. With 8 rings over ±0.26 rad the innermost sits at 0.0371 rad,
and a 0.15 m band truncates it at `0.15/sin(0.0371) = 4.04 m` — precisely the "caps
every ring below 4 m of the sensor's 16 m" that was measured. No band fixes that,
and **no choice of filter frame fixes it either**: the truncation is geometric, so
a gravity-aligned `target_frame` does not help.

The fix is an **odd** ring count, which puts one ring at exactly 0 elevation. That
ring passes any band at full range, and the tilted rings then contribute genuine
3D structure. `fleet.lidar_rings` now selects this and `spawn_fleet.py` refuses an
even value rather than emitting a model that comes up healthy and maps badly.

## 3. Per-robot SLAM for odometry + cloud + camera

One constraint decides most of this: **FAST-LIO2, Point-LIO and LIO-SAM all
require a well time-synchronised IMU**, roughly 100–400 Hz, because they
preintegrate it between scans to de-skew and to seed registration. Below about
100 Hz the preintegration error over one 10 Hz scan swamps the benefit. The
simulated IMU was 50 Hz and is now 200 Hz for this reason. Confirm the rate and
the axis convention on the real hardware before assuming any LIO option is open.

| Option | IMU | What it adds over SLAM Toolbox | Cost |
|---|---|---|---|
| **RTAB-Map** | no | Uses the cloud (ICP) *and* the camera (appearance-based loop closure), pose graph, still emits a 2D `OccupancyGrid` | Lowest — drops in behind the same topic |
| KISS-ICP | no | Strong lidar odometry to replace drifting wheel odom; tuning-free | Low, but odometry only — no loop closure |
| FAST-LIO2 / Point-LIO | **yes** | Best 3D odometry accuracy | Needs a loop-closure layer added |
| GLIM | **yes** | 3D SLAM with global optimisation built in | Higher, GPU-oriented |
| LIO-SAM | **yes** (9-axis) | Integrated loop closure | Highest; most sensitive to IMU quality |

**RTAB-Map is the closest fit** and is now wired up as `slam_backend:=rtabmap`. It
consumes external odometry as its motion guess, refines with ICP against the
cloud, optionally closes loops from the RGB image, optimises with g2o, and
publishes `grid_map` — remapped to `<ns>/map`, so the adapter, backend, Nav2 static
layer and GUI are untouched. It also derives the 2D grid by segmenting ground from
obstacles rather than slicing a height band, which is the part a band can never do
properly.

It does **not** make this a swarm SLAM system. Each robot still keeps a private
pose graph and its own map frame.

## 4. What swarm SLAM would actually change

The missing capability is an **inter-robot loop closure**: robot A recognises a
place robot B has been, that becomes a constraint between the two pose graphs, and
optimising the joint graph corrects *both* robots' drift while simultaneously
yielding the relative transform. Grid registration only ever recovers the
transform, and only after each robot has already finished being wrong.

Relevant prior art:

| System | Approach | Fit here |
|---|---|---|
| **Swarm-SLAM** (Sherbrooke) | Decentralised, sparse inter-robot loop closure detection, GTSAM back end, lidar / stereo / RGB-D, ROS 2 | Closest match — built for swarms |
| DiSCo-SLAM | Distributed lidar, Scan Context descriptors, two-stage optimisation | Good if staying lidar-only |
| Kimera-Multi | Distributed visual-inertial, metric-semantic | Needs good VIO and an IMU |
| COVINS-G | Centralised, many-agent visual-inertial | Centralised suits this backend |
| `multirobot_map_merge` (m-explore-ros2) | Grid merging via OpenCV feature matching | The maintained equivalent of what is hand-rolled here |

On `multirobot_map_merge` specifically: writing the registration by hand was a
defensible call, because the backend is required to import no ROS (stated as
acceptance criterion 12 in `api/app.py`) and that package is a ROS 2 node. What
should not have happened silently is that `docs/architecture/roadmap.md` named it as the plan
and something else shipped.

### A staged path

1. **Per-robot 3D SLAM.** `lidar_rings: 9`, `slam_backend:=rtabmap`, camera loop
   closure once the Gazebo `camera_info` topic is confirmed. Nothing downstream
   changes. Verify against `<ns>/ground_truth`, which is already published and
   currently only used for scoring.
2. **Keep grid registration as the bootstrap.** It is now reliable enough to
   supply an initial relative transform, which is exactly what a collaborative
   back end needs as a prior for inter-robot place recognition.
3. **Add inter-robot loop closure.** Descriptors per keyframe (Scan Context for
   lidar, bag-of-words for vision), exchanged between robots or through the
   backend, verified geometrically before becoming a constraint.
4. **Joint optimisation.** A GTSAM back end over both graphs, with robust kernels,
   emitting corrected `map → odom` per robot *and* the inter-robot transform. At
   this point the merged grid becomes a rendering of the optimised graph rather
   than a stitch of independent maps, and step 2's registration becomes a prior
   instead of the answer.
5. **Then, and only then, coordinated exploration.** Frontier assignment across a
   fleet is only meaningful once the robots agree on one frame. Goal assignment is
   operator-driven today, which is a reasonable place to stop until step 4 lands.

The adapter seam survives all of this: robots exchange descriptors and constraints
in their own environment, and the backend keeps receiving grids and poses. What
changes is that the poses arriving at the backend are already jointly consistent,
so `mapsvc` transforms become bookkeeping rather than estimation.

## 5. Known remaining limits

- Registration is still pairwise against one reference, with no loop-consistency
  check across transforms and no reference re-election if that robot goes quiet.
- Near-disjoint coverage can still produce a wrong answer on a building with
  strong repeated structure. `support` and `yaw_ratio` cut this sharply but the
  problem is genuinely ill-posed; a configured start pose remains the reliable
  guard, and `auto` mode without priors should be treated as best-effort.
- Nav2 still plans on each robot's private map (FR-M8 unimplemented).
- `slam_backend:=rtabmap` and `lidar_rings > 1` are wired and syntax-checked but
  have **not** been run against Gazebo; they need `make docker-up`. The RGB loop
  closure path additionally needs the sim's `camera_info` topic name confirmed
  with `gz topic -l`.
