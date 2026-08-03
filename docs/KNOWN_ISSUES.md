# Known Issues

Live list. Update as things are resolved.

## Open

### 1. Production video pipeline unbuilt

`swarmdeck_media` is an empty package and MediaMTX is not installed, so the target WHEP
path and its <300 ms latency criterion remain Phase 4 work. Simulation cameras now use a
verified 5 Hz JPEG preview fallback through the adapter and ROS-free backend.

### 2. Registration needs shared coverage

`auto` mode aligns two maps to ~8 cm, but only when the robots have actually seen the
same places. With largely disjoint coverage the problem is ill-posed and the ratio test
correctly refuses to answer. The dashboard then shows each selected robot's local map;
it does not overlay grids using configured spawn priors. This is inherent to map
registration, not a bug, but it means exploration has to be arranged so robots overlap.
Measured: ~2150 overlapping occupied cells gave score 0.54, ratio 0.58.

### 3. Duck detector is a portable classical baseline, not a trained neural model

The shipped detector already produces live RGB bounding boxes in Gazebo and accepts
the same BGR frames from a physical camera, but it currently uses colour/shape evidence.
A publicly available fine-tuned YOLO duck model was not embedded because its own model
card warns that the training data has unresolved licensing. The detector API is kept
model-shaped (`detect_bgr` in, normalized boxes out), so a licensed ONNX model can
replace the baseline without changing ROS, the adapter protocol, backend, or UI. A
production model still needs licensed real-camera training/validation data covering the
actual lighting and duck variants.

### 4. Sudden lateral displacement needs exteroceptive odometry

Gazebo's DiffDrive odometry, like wheel encoders on hardware, cannot directly observe a
robot being dragged sideways. The adapter already displays SLAM Toolbox's corrected
`map -> odom -> base_link` pose, so ordinary scan matching and loop closure repair drift,
but a large instantaneous displacement can remain wrong until lidar SLAM finds the place
again. Wheel + IMU EKF fusion improves heading and short-term motion but cannot provide
an absolute translation constraint; a production robot that must recover immediately
from "kidnapping" needs lidar odometry, visual odometry, landmarks, or global
relocalization. Gazebo ground truth remains scoring-only and is not fed into navigation.

## Resolved

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
