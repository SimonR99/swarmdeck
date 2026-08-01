# Known Issues

Live list. Update as things are resolved.

## Open

### 1. Frontend never visually confirmed

Builds clean and `svelte-check` reports 0 errors, but Chrome in this environment cannot
reach the dev server (`localhost`, `127.0.0.1`, and the LAN IP all fail), so the layout
has never been seen rendered. Open `http://localhost:5173/?mock=1&robots=4` and check
before trusting it.

### 2. Video pipeline unbuilt

`swarmdeck_media` is an empty package and MediaMTX is not installed. `CameraPanel`
degrades to a "no signal" placeholder, which *is* verified. Phase 4.

### 3. Registration needs shared coverage

`auto` mode aligns two maps to ~8 cm, but only when the robots have actually seen the
same places. With largely disjoint coverage the problem is ill-posed and the ratio test
correctly refuses to answer — the merge then falls back to `static` transforms. This is
inherent to map registration, not a bug, but it means exploration has to be arranged so
robots overlap. Measured: ~2150 overlapping occupied cells gave score 0.54, ratio 0.58.

### 4. Nav2 not yet wired

`nav2_params.yaml` exists and Nav2 is installed, but nothing launches it and
`adapter_sim.navigate_to` publishes `goal_pose` with no planner listening. Phase 3.
`scenario/explore.py` provides reactive wandering in the meantime.

### 5. `spawn_fleet.py` quaternion is wrong for non-zero yaw

It emits `orientation: {z: yaw/2, w: 1.0}`, which is only correct at yaw = 0. Should be
`z = sin(yaw/2), w = cos(yaw/2)`. Harmless today because all start poses use yaw 0, but
it will silently mis-place robots the moment one doesn't.

## Resolved

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
