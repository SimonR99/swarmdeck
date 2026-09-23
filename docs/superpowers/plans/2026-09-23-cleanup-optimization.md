# SwarmDeck clean-up, reorganization and optimization plan

Measured on 2026-09-23 with the SubT Finals scenario, four robots and drift
odometry. The first survey used the operator workstation (20 cores, Intel Iris
Xe used by ARGoS, NVIDIA driver not loaded, DRI rendering, fleet mostly idle).
A second pass ran on tuf (RTX 4070), idle and exploring. Profiles are py-spy
samples, plus `perf` for MGG, taken from a throwaway sidecar container
(`--pid=container:...`), so nothing in the stack was modified. Numbers are
snapshots, not averages over missions. All reports are in
`docs/operations/performance.md`.

This supersedes the structural-only plan of 2026-09-22 in the same folder.
Its diagnosis still stands, but several details are out of date (see Phase 4).

## Hosts and access

- **tuf**: the measurement and GPU host, used for every before/after number
  unless an item is specific to the workstation.
  - Access: `ssh tuf` (`mistlab@minecraft.mistlab.ca`, port 10022, key auth).
  - Hardware: 20 cores, RTX 4070.
  - Checkout: `~/swarmdeck-ws/swarmdeck`, with the SubT assets in
    `~/swarmdeck-ws/argos3` and logs in `~/swarmdeck-ws/logs/`.
  - The `perf` image is built from `~/swarmdeck-ws/tools/perf`.
  - The stack runs there as Compose project `swarmdeck`.
  - Its checkout follows `planning-refactor`. It accepts pushes to its checked-out branch
    (`receive.denyCurrentBranch=updateInstead`), and the workstation has a
    `tuf` remote. To sync it, run `git push tuf planning-refactor`. Nothing goes through
    `origin`, and git refuses the push if tuf has local edits to tracked
    files. Push before measuring a change, and put the commit in the report
    label. The running containers bind-mount `adapters/`, `autonomy/`,
    `configs/`, `deploy/{autonomy,mgg}/`, `scripts/`, `sessions/` and
    `swarmdeck_ros/src/`. A push changes what they run the next time they
    restart.
  - `perf_event_paranoid` is 4. `profile_stack.py --perf-mgg` works around
    this with a sidecar that has `SYS_ADMIN`, so no sudo is needed.
- **workstation**: 20 cores, iGPU, shared with the operator's desktop. Its
  load varies, so use it only for dashboard and browser measurements and for
  quick checks.
- **botman**: the reference onboard computer, used to judge onboard targets.
  - Access: `ssh botman@192.168.1.49` from the lab Wi-Fi. It has no
    `~/.ssh/config` alias and is not reachable from tuf.
  - Hardware: Jetson AGX Orin (12 Cortex-A78AE cores at 2.2 GHz, 61 GiB RAM,
    16-SM Ampere GPU), `MODE_50W`, Jetson Linux R36.4.4.
  - `gcc` 11.4, CUDA 12.6 and `tegrastats` are installed. `botman` is in the
    `sudo` and `docker` groups, and `perf_event_paranoid` is 2.
  - It runs the `main` deployment (`/ssd/swarmdeck`, with uncommitted
    experiment edits), and experiments run on it. Never change its code,
    images or configuration. There is no MOLA there.
  - For benchmarks, the stack may be paused with `docker stop` on the 14
    `swarmdeck-botman-*` containers. It is then resumed with `docker start` on
    the same containers: no `compose up`, so nothing is recreated. Never move
    the robot.
- **benchbot** (`ssh benchbot`, user `sroy`): not used by this plan. Its
  running SwarmDeck stack belongs to someone else.

---

## 1. What is slow, measured

| Symptom | Measurement | Where the time goes |
|---|---|---|
| **The simulation runs at ~0.17-0.62x real time on the workstation** (1.00 on tuf with an RTX 4070, idle and exploring) | `/clock` advanced 5.5 s in 33 s of wall time; 300 s of sim time in ~1000 s since start | ARGoS is *not* CPU-bound (busiest threads are Filament render threads at ~25 %) and does not wait for the bridge (streaming mode, `realtime` defaults to true). It is GPU-bound: per sim second it renders 4 lidars x 4 faces of 512x301 at 10 Hz plus 4 cameras at 5 Hz, each against the 3.7 M-triangle SubT visual mesh. On the iGPU that is too slow. On tuf's RTX 4070 it runs at real time with 56-72 % CPU for `argos3`. Everything downstream (Nav2, SLAM, exploration pace, teleop feel) runs at this rate. |
| **Exploration pauses 3-11 s between moves** | 23 MGG plan cycles: 3.2-10.9 s wall each | 70-80 % in the local lattice build ("grid" 2.2-8.2 s for ~1,400 vertices / 25k edges), 10-20 % in gain. **Update (tuf, same graph size): median 0.69 s, max 1.04 s**, so most of it was workstation contention. `perf` on tuf: 62-65 % of MGG's time is `NativeMolaGrid::status`, which answers every voxel query with two binary searches over sorted cell vectors; a hashed or blocked index is the first MGG fix. |
| **The dashboard costs ~1.7 cores in the browser** | Firefox 109 % + content process 60 % at idle | 3D view renders continuously at 30-60 fps whether or not anything changed; every 200 ms `map3dLayers.update` JSON-stringifies robots, paths and review entities to detect changes and re-uploads the network texture; the 2D canvas redraws at display rate; map status/catalogue are polled every 2-3 s in overlapping loops. WebSocket traffic is *not* the cause: 66 KB/s at idle (3.4 KB `robot_state` at 5 Hz/robot). |
| **Peers burn ~1.6 cores while parked** | per peer: `cslam_bridge.py` ~20 %, `lidar_handler` ~9-11 %, loop closure 5-8 % (100 % on a peer that explored a lot), `pose_graph_manager` ~3 % | Bridge: 34 % of samples rebuild and JSON-encode the full mapping snapshot (`snapshot` -> `snapshot_dict` -> `envelope`); `status.json` rewritten every second; 20 Hz / 10 Hz polling timers. Lidar handler: 30 % downsampling point clouds in Python. Loop closure: 75 % in a linear ScanContext search over every descriptor (grows with the map). Pose graph: ~100 of 104 optimizations re-solve an unchanged graph. |
| **Server ~30 % idle** | py-spy | ~30 % in the deployment raster refresh (per-point Python loop, JSON+sha256 of every submap each 3 s tick), ~18 % JSON encode/decode; `state_loop` rebuilds and broadcasts every robot at 5 Hz whether or not it changed; a synchronous SQLite query per inbound `robot_state`. |
| **ARGoS<->ROS bridge ~40 % of a core** | py-spy | 21 % `publish`, ~20 % the planar nav-scan projections, ~20 % point-cloud packing + SHA-256; the "parked robot" cache never hits (keyed on the odometry tick). Not the RTF limiter (ARGoS does not wait for it). |
| **MGG map briefly "unavailable"** | seen live; now retried by the adapter | Heartbeat to MGG is gated on sensor freshness, TF lookups and product reads in a busy single-threaded executor; a gap of ~3 s expires the snapshot. |

A functional defect found on the way (not a performance item, but it makes
the fleet explore redundantly):

- **The planners never share roadmaps.** `/robot_N/mgg/neighbour_graph_in`
  has 0 publishers and `neighbour_graph_out` 0 subscribers
  (`deploy/mgg/robot.launch.py` remaps only TF and odometry); the MGG log has
  no "roadmap update from robot" line. The inter-robot offsets are all zero
  (`bistro.yaml`), which would misplace the other robots' vertices by their
  spawn offsets even if connected, and each planner's map holds only its own
  robot's scans, so a goal in a tunnel another robot mapped is refused with
  "no mapped ground under the goal". This is why R0 explored toward a goal in
  a tunnel R1 had already explored.

---

## 2. Principles

- **Measure, change, re-measure.** Every item below names the metric it must
  move; a change that does not move it is reverted.
- **Behaviour-preserving unless a decision says otherwise.** Structural moves
  keep endpoints, topics, file layouts and message contracts.
- **Planner changes are discussed first and stay within the MGG concept**
  (upstream mechanisms, no tiers or special cases); MGG changes go to
  MGGPlanner `ros2` and are pinned by `MGG_REV`; C-SLAM changes stay as
  `deploy/patches/cslam-*.patch`.
- **Green at every commit**: `pytest -q`, `autonomy/tests`, `tests/deployment`,
  UI `check` + `test:map3d`, MGG `colcon test`, mapping image build (ctest +
  smokes).

---

## Phase 0 - Measurement harness (first, small)

A repeatable `scripts/profile-stack` so every later phase reports numbers:

1. **Sim real-time factor:** `/clock` against wall time over 30 s.
2. **Per-process CPU:** `top` sample grouped by container.
3. **py-spy profiles:** for the bridge, `adapter_sim`, server, one peer's bridge, lidar handler and loop closure, from the sidecar container used for this plan.
4. **MGG plan-cycle statistics:** parsed from its own timing log lines.
5. **GUI WebSocket bytes and messages per second.**
6. **Browser main-thread load:** CPU of the dashboard tab at idle and while navigating.

Output is a dated table appended to `docs/operations/performance.md`. MGG C++
is profiled with `perf` from a sidecar container (`--perf-mgg`, see Hosts and
access).

**Status:** `scripts/profile_stack.py` exists (committed in `88f3de9`,
`--perf-mgg` in `0bb7f5a`) and covers items 1-5 plus `--start-explore`. Still
missing:

- item 6, browser load;
- the plan-to-motion latency trace from Sequencing step 2.

Two baselines are recorded, both on tuf: idle, and exploring.

## Phase 1 - Low-risk wins (each independently revertible)

**UI** (largest single consumer):
- Render 3D on demand: a dirty flag set by input, data and store changes; animate only while something moves or follow mode is on (`Map3D.svelte:720-785`).
- Replace the `JSON.stringify` change signatures in `map3dLayers.update` with revision counters; move goal and loop markers in place; flag the network texture only when a patch arrives.
- Give the 2D canvas the same dirty-flag redraw. Record trails in the store, not in the 2D draw loop; that also fixes stale trails in 3D.
- Merge the 2 s and 3 s map polls into one scheduler, skipped while the tab is hidden or the view doesn't need it. Only replace arrays when their content changed.
- Memoise `fleet.robots` and the enabled list. Stop the replica catalogue effect from re-running on every `robot_state`.
- Cap `session.detections`. Load the mock simulator only with `?mock`.
- *Target: dashboard tab under 25 % CPU at idle.*

**Server:**
- `state_loop`: broadcast a robot only when its state changed, plus a 1 Hz keep-alive, and skip entirely with no GUI connected.
- `network_loop`: skip when the grid revision hasn't moved.
- Cache `map_epoch` in memory instead of querying SQLite per message.
- Drop the duplicated paths in `robot_state`: they're sent top-level and again inside `live_mapping`.
- Deployment raster: compute submap keys once per catalogue snapshot, vectorise `_contribution`, and don't rebuild on a pose-only change unless it moved beyond a tolerance.
- Cache optimized-map PNGs by (scope, seq), with an ETag.
- *Target: server under 10 % CPU at idle.*

**Sim container:**
- `adapter_sim`: don't subscribe to camera, depth or camera info when there's no detector.
- One subscription per process for the fleet-wide coordination topics and `map_authority`, instead of four.
- Bridge: key the scan cache on the raw scan only, recomputing the nav projections only when the pose changes, and skip building messages nobody subscribes to.
- Media: push raw RGB into the pipeline, removing the JPEG encode → decode round trip, and gate fps before encoding.
- *Target: sim container under 60 % at idle.*

**Peers:**
- Write `status.json` only when it changes.
- Mapping worker: `stat` before hashing `snapshot.json`; trust the native runtime's output hashes instead of re-hashing.
- Raise `product_authority.MAX_PRODUCT_BYTES` from 4 MiB to the 64 MiB the worker and MGG allow. This is a latent failure at about 2,400 keyframes.
- Keep re-sending the last good map authority to MGG instead of going silent, and log every gap with its reason. That removes the "map unavailable" cause at the source.
- Garbage-collect old missions under `/maps` (about 500 MB across 8) and checkpoint the SQLite WAL.

## Phase 2 - The three big costs (need an experiment or a decision)

1. **Simulation real-time factor (Q1).** A discrete GPU fixes it: tuf runs at 1.00. What is left is making the simulation usable on weaker GPUs, such as the workstation's iGPU. Measure on the workstation, and use tuf to check that nothing regresses. The first step is to find which option buys the most, with two short runs: lidar at 5 Hz, and lidar rendering against the collision mesh. The options:
   - render lidar faces from the low-poly collision mesh (1.35 M vs 3.66 M triangles) or with distance culling;
   - reduce face resolution or scan rate for robots that aren't moving;
   - reuse depth faces between robots' scans.

   *Target: real-time factor of at least 0.8 with 4 robots on the workstation's iGPU, and still 1.00 on tuf.*
2. **MGG lattice build.** The 3-11 s cycles were mostly host contention: on tuf the same graph size (about 1,500 vertices and 25.7k edges) takes a median of 0.69 s (max 1.04 s), of which the lattice is 492 ms and the gain 142 ms. The planning CPU still matters on the robots' slower onboard CPUs. `perf` on tuf, while exploring, found:
   - 62-65 % of the time in `NativeMolaGrid::status`, which answers every voxel query with two `std::binary_search` calls over the sorted occupied and free cell vectors (`native_mola_grid.cpp:77-83`). Replacing them with a hashed or blocked cell index is the first fix;
   - 7-9 % in `NativeMolaGrid::walk` (ray status);
   - 2-4 % in `std::map<Cell, double>` lookups (`surface_max_z_`), a second candidate for the same index.

   Optimise inside MGG without changing behaviour, then profile again. Pin with the existing MGG tests plus a lattice-equality test on a recorded map. On botman, the same voxel queries run 2.3-2.9x slower than on tuf with the robot idle, and 2.7-3.6x slower under the robot's own load (`scripts/bench/host_bench.cpp`, see `performance.md`). So tuf's 0.69 s cycle would be about 1.7-2.4 s onboard. On the Orin, a `std::unordered_set` is *no* faster than the binary search below about 1 M cells. On x86 it is 2-3x faster. So the index has to be cache-friendly, such as open addressing or dense blocks. Chosen by its botman numbers, not tuf's. *Target: median cycle under 1 s on botman under its deployment load, estimated from tuf times the measured botman/tuf factor until MGG with MOLA can run there. Also halve the median lattice time on tuf, from 492 ms.*
3. **C-SLAM idle and growth costs (upstream patches in `deploy/patches/cslam-*.patch`):**
   - skip pose-graph optimisation when no factor or keyframe changed;
   - index the ScanContext search (ring-key kd-tree, as upstream ScanContext does) instead of a linear scan;
   - vectorise downsampling in the lidar handler;
   - make the bridge event-driven (normalise in the cloud callback, flush after keyframes) and move the heartbeat to its own thread;
   - stop caching RGB-D outside keyframe windows.

   *Target: a parked peer under 10 %, and loop closure flat as the map grows.*
4. **DDS load:**
   - give sim, peers and MGG a shared-memory transport (they share a network namespace but force UDPv4 for about 20 MB/s of reliable clouds and images);
   - publish static mounts once on `tf_static` instead of 16 `static_transform_publisher` processes;
   - put each robot's controller and smoother in one component container (about 34 participants down to about 10).

## Phase 3 - Roadmap sharing between planners (Q2, functional)

Connect `neighbour_graph_in`/`out` on a shared topic. Give the merge a real
inter-robot transform from the selectable pose source. Resolve a goal in
another robot's map through the shared roadmap (both are under Decisions).
Pinned by an integration test with two planners and a recorded map.

## Phase 4 - Reorganization (behaviour-preserving)

Corrected version of the 2026-09-22 plan:

1. **Package boundary (Q3):** `autonomy/` stays ROS-free. The ROS peer runtime (`cslam_bridge.py`, `mola_worker.py`, `mola_process.py`, `peer.launch.py`) moves out of `deploy/autonomy/` into `swarmdeck_ros/src/swarmdeck_peer` (decided), not into `autonomy/`. `colorize_ros_rgbd` moves out of `adapters/`. `reconstruction.py` and `replica_maintenance.py` go to `tools/`.
2. **Server:** a `state.py` owns the globals (session, alerts, camera, detections, review), and routers import it instead of calling `_app()`. The two WebSocket dispatchers get their own modules. Registry → api imports are inverted. `app.py` ends as the factory plus registration. A route-inventory test is written first; it doesn't exist yet.
3. **Adapters:** a shared goal-ownership mixin in `runtime.py`. The sim and ros2 behaviours differ today, so characterisation tests come first. One image decoder. Remove the byte-identical `_network_quality` overrides. Split the bridge's 400-line `_read_robot` into a decoder, projections and publishers.
4. **UI:** shared frame, path and goal code for 2D and 3D with one membership rule. Split `Map3D` and `MapView` into controllers plus views, and make the replica catalogue one polled store.
5. **Name collisions:** rename `adapters/live_mapping.py` → `navigation_display.py` (including its test) and `adapters/reconstruction.py` → `colorize.py`, updating its three importers.

## Phase 5 - Dead code and hygiene

Candidates, each with grep evidence in the survey:

- **Server:**
  - the `POST /api/adapter/camera` JPEG path, with `_camera_frames`, the stream-loss alert and `GET /api/camera/{id}` (no adapter posts frames);
  - `camera_interest` (no adapter handles it);
  - `/api/agent/*` in the server (nginx and vite route it to Cortex);
  - the `Bus` class;
  - unused registry and raster helpers;
  - `POST /api/fleet/{id}/discard`;
  - the stale `slam_graph` UI handler;
  - `server/build/lib`.
- **Adapters and bridge:**
  - unused `adapter_sim` parameters, attributes and imports;
  - the bridge's `_iter_hits` and the unread wheel-encoder payload;
  - the broken `scripts/benchmark-sim.py` and `check_map_color.py`;
  - orphaned `__pycache__` directories.
- **Peers:**
  - the `SWARMDECK_SLAM_BACKEND` switch (its only value is `cslam`);
  - the worker's `oneshot` and `--all-missions` modes;
  - the empty `swarmdeck_cslam/launch/`;
  - duplicate `status.json` fields;
  - stale references to `autonomy/mola_mapping.py`.
- **Legacy paths:** the legacy adapter reset handshake and the no-mission branches. The default compose file never exercises them. They are removed (Q4, decided below).

---

## Decisions (2026-09-23)

- **Priority:** exploration pace first, because that is what matters on the real robots. Simulation speed second, dashboard third (it is responsive enough today).
- **Measurement and GPU host (Q1):** tuf (RTX 4070, `ssh tuf`, see Hosts and access). The workstation's NVIDIA GPU is in an external Thunderbolt box and not attached. Benchbot is not used.
- **Inter-robot transforms for roadmap sharing:** selectable, like odometry. `cslam` (C-SLAM's inter-robot estimates, the default and the only choice on hardware) or `ground_truth` (simulation). A `--robot-poses cslam|ground_truth` option in `sim-up` / `make up-sim`, carried to MGG's neighbour-graph merge.
- **Package boundary (Q3):** the ROS peer runtime moves into a ROS package, `swarmdeck_ros/src/swarmdeck_peer`. `autonomy/` stays ROS-free.
- **Legacy paths go (Q4):** the legacy adapter reset handshake, the no-mission server branches, and the server-side camera JPEG path are removed. This branch is moving on.
- **Goal in another robot's map (Q2):** route it through the shared roadmap. Terrain another robot drove over is known traversable, so the planner may plan over the other robot's vertices and edges even where its own map has no geometry. The local controller follows that global path. It stops if something is impassable, and the robot builds its own map on the way. Consequences for Phase 3:
  - Where the goal is on or next to the other robot's roadmap, MGG attaches it to that roadmap, using the other robot's traversal as evidence instead of its own map's ground check.
  - The exactness rule still holds: the route ends on the requested goal.
  - Any check that re-validates the whole route against the robot's own map (for example the explicit-objective refinement) must accept segments that follow the other robot's edges.
  - A blocked segment ends the navigation with the controller's reason, like any other obstruction.

## Sequencing (by priority)

1. **Phase 0, the harness.** Add the MGG cycle statistics and a plan-cycle wall-time trace first, because exploration pace is the headline metric.
2. **Exploration pace**, on the code that also runs on the robots:
   - MGG lattice build and gain (Phase 2 item 2), profiled with `perf`;
   - the adapter's plan-to-motion latency: how long from the path being published to the robot moving, and the replan cadence;
   - the peer map heartbeat and authority fixes (Phase 1, peers);
   - C-SLAM's onboard costs (Phase 2 item 3), which compete with MGG for the robot's CPU.

   *Target: the Phase 2 item 2 target, and no "map unavailable" gaps.*
3. **Roadmap sharing (Phase 3)** with the selectable pose source. It avoids re-exploring what another robot already covered, which is exploration pace at fleet level.
4. **Simulation speed (Phase 2 item 1)** for weaker GPUs, measured on the workstation and checked on tuf. Then the sim-container and DDS items (Phase 1 sim, Phase 2 item 4), measured on tuf.
5. **Server and dashboard (Phase 1 UI and server).**
6. **Reorganization (Phase 4)** interleaved where it touches the same files, then dead code (Phase 5, legacy included).

Each item is one or a few commits and reports before/after numbers from Phase 0.

## Open questions

- **Measuring MGG itself on botman.** botman runs `main` without MOLA, so the onboard estimate comes from a proxy benchmark. A real measurement needs this branch's MGG image built for arm64, run next to the paused deployment against a recorded map. Is that acceptable on botman, or should it wait for another Orin?
- **botman idle load.** With no plan cycles, MGG already uses 32-63 % of a core on botman. `vectornav` uses 41 %. The whole deployment keeps about 6 of 12 cores busy. This is worth a look under Phase 1 (peers), since it competes with planning.
- **botman base driver.** `swarmdeck-botman-stack` is in a restart loop. `bunker_base_node` aborts on `can2` with `Resource deadlock avoided`. It was like this before the benchmark, and I left it alone.
