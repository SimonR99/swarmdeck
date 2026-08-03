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

Every robot runs a private pose graph and the backend aligns the finished grids afterwards,
so no robot's drift is ever corrected by another robot's observations. There is no
inter-robot loop closure, no shared graph, and no transitive registration — a robot that
overlaps robot 2 but not the reference can never join the global map. Nav2 also still plans
on each robot's own map rather than the merged one (FR-M8). This is a deliberate
consequence of the ROS-free backend, not an oversight, but it caps what the fleet can do.
docs/collaborative-slam.md sets out what would change and a staged path.

### 5. 3D SLAM path is wired but unproven

`fleet.lidar_rings > 1` and `slam_backend:=rtabmap` are implemented and syntax-checked but
have never been run against Gazebo; they need `make docker-up`. RTAB-Map's RGB loop closure
(`use_camera:=true`) additionally needs the simulated `camera_info` topic name confirmed
with `gz topic -l`, which is why the lidar-only path is the default.

### 6. Sudden lateral displacement needs exteroceptive odometry

Gazebo's DiffDrive odometry, like wheel encoders on hardware, cannot directly observe a
robot being dragged sideways. The adapter already displays SLAM Toolbox's corrected
`map -> odom -> base_link` pose, so ordinary scan matching and loop closure repair drift,
but a large instantaneous displacement can remain wrong until lidar SLAM finds the place
again. Wheel + IMU EKF fusion improves heading and short-term motion but cannot provide
an absolute translation constraint; a production robot that must recover immediately
from "kidnapping" needs lidar odometry, visual odometry, landmarks, or global
relocalization. Gazebo ground truth remains scoring-only and is not fed into navigation.

## Resolved

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
