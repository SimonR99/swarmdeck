# SwarmDeck — Roadmap

Ordered so each phase produces something runnable, and the adapter contract — the thing
that makes a mixed ROS 1 / ROS 2 fleet possible — is fixed early, while it is still
cheap to change.

Every phase has a **demo** (what you can show) and an **exit criterion** (what must be
true to continue). Do not advance past a failing exit criterion.

---

## Phase 0 — Foundations
**Goal:** a repo that builds, and a simulation that starts.

- Repo skeleton per `architecture.md` §9; `Makefile`; CI.
- Install (verified in apt): `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`,
  `ros-jazzy-slam-toolbox`, `ros-jazzy-pointcloud-to-laserscan`,
  `ros-jazzy-nav2-map-server`. Plus MediaMTX, GStreamer, ONNX Runtime.
- ~~Source build: `m-explore-ros2` (`multirobot_map_merge`).~~ Not used — see Phase 5.
  `ros-jazzy-rtabmap-slam` + `ros-jazzy-rtabmap-util` for the optional 3D SLAM backend.
- `swarmdeck_description`: one robot with lidar, IMU, odom, camera.
- `swarmdeck_sim`: indoor world, seeded spawner.
- Headless sim smoke test, `timeout`-wrapped with PID reaping.

**Demo:** `gz sim -s -r` runs headless in CI, publishing lidar and camera topics.
**Exit:** one robot drives via `/cmd_vel`; CI green; headless test reliably terminates.

> Orphaned `gz sim` processes hold DDS ports and silently poison the next run. Get
> process cleanup right here, not later.

---

## Phase 1 — Adapter contract and backend spine ⚠️ decides fleet heterogeneity
**Goal:** the seam that lets any robot type join, proven without ROS.

- `adapters/protocol/`: JSON schemas, `hello`/capability handshake, reference client.
- `adapter_mock`: synthetic robot, no ROS.
- Backend `api/` + `fleet/`: adapter registry, capability negotiation, liveness,
  command routing. Internal bus.
- `record/`: MCAP writer, session directory, `manifest.json`.
- `events/`: operator action log with three timestamps.
- UI skeleton: Svelte + Vite, WebSocket client, robot cards driven by capabilities.
- `validate-session` tool.

**Demo:** two mock robots appear in the GUI, accept goals, and record a valid session —
**with no ROS installed**.
**Exit:** acceptance criterion 12 (backend runs ROS-free); invalid payloads rejected
loudly; adding a mock robot type requires no backend change.

> Everything downstream depends on this contract. Changing it after Phase 4 means
> reworking every adapter. Spend the time here.

---

## Phase 2 — Simulation adapter and 2D map
**Goal:** a real robot in the loop, and a live map.

- `swarmdeck_slam`: `pointcloud_to_laserscan` (height band) + SLAM Toolbox per namespace.
- `adapter_sim`: rclpy, bridges the Gazebo fleet to the adapter protocol.
- `mapsvc/`: per-robot grid store, merged grid, patch generation, PNG endpoint.
- UI `map2d`: offscreen canvas, patch blitting, robot/trail/goal overlays.
- Reconnect: fetch full grid once, resume patches.

**Demo:** drive one simulated robot, watch the map build; reload the browser and the map
returns without re-running the simulation.
**Exit:** map survives reload (FR-M7); updates arrive < 1 s (NFR-6); no full-grid
re-fetch on incremental change.

---

## Phase 3 — Navigation and control
**Goal:** the operator can command a robot.

- `swarmdeck_nav`: Nav2 per namespace, consuming the merged grid.
- Adapter `navigate_to` / `cancel_goal` / `stop` mapped to Nav2 actions.
- UI: click-to-navigate, goal markers, cancel, stop-all.
- `sendAction()` chokepoint — every action stamped and logged in one place.

**Demo:** click the map, robot plans and navigates, action lands in `events.jsonl`.
**Exit:** goals succeed reliably; stop-all halts immediately; every UI action logged.

> Keep planner specifics inside the adapter. The GUI must never send Nav2-shaped
> commands, or a ROS 1 robot will not be able to accept them (FR-N8).

---

## Phase 4 — Video
**Goal:** camera viewing at target latency.

- `swarmdeck_media`: image topic → GStreamer → x264 → RTSP → MediaMTX.
- UI video panel: WHEP client, `<video>` + overlay `<canvas>`.
- Latency harness: timestamp burned into the frame, read back from screen capture.

**Demo:** live camera in the GUI with a measured latency figure.
**Exit:** < 300 ms sustained (NFR-1); camera switching logged as an action.

---

## Phase 5 — Multi-robot ⚠️ highest risk
**Goal:** 4 robots, one merged map.

- Namespaced stacks `/robot_0` … `/robot_3`; bringup parameterized by robot count.
- **`static` merge mode first** — known start poses, fixed transforms.
- Ground-truth accuracy scoring node (translation/rotation error per robot).
- Then `auto` mode for unknown relative starts (FR-S4). **Shipped differently from the plan:**
  this line named `multirobot_map_merge`, but that is a ROS 2 node and the backend must import
  no ROS (acceptance criterion 12), so registration is implemented in `mapsvc` with numpy —
  see `docs/architecture.md` §5.2 and `docs/collaborative-slam.md`.
- UI: multi-robot selection, duplicate-goal rejection, per-robot trails.

**Demo:** four robots with unknown starts converge to one aligned map after observing
shared areas.
**Exit:** acceptance criteria 1–4; merge error reported numerically.

> `static` mode is a permanent fallback. If `auto` merging is unreliable, you lose the
> unknown-start condition, not the project.

---

## Phase 6 — Perception
**Goal:** detections as persistent map entities.

- `swarmdeck_perception`: ONNX detector per robot camera.
- Projection of detections into map coordinates.
- `detect/`: cross-robot dedup, map-entity lifecycle.
- UI: map markers, video bbox overlay, "report target" action.

**Demo:** two robots see the same object; one marker appears, with both observations.
**Exit:** acceptance criterion 5; dedup verified with a scripted two-robot encounter.

---

## Phase 7 — Session tooling and alerts
**Goal:** a non-ROS user can run the whole thing.

- Alerts: inactivity threshold, nav failure, robot fault, adapter disconnect, stream loss.
- Session control UI: start/stop, config selection, live fleet and stream health.
- Session replay: `record/` drives the bus with no adapters connected.
- `analysis/`: session loaders, per-robot interaction and neglect timelines.
- README runbook.

**Demo:** someone with no ROS knowledge runs a full session, then replays it.
**Exit:** all 13 acceptance criteria pass.

---

## Phase 8 — Real robot adapters *(conditional — schedule when hardware is confirmed)*
**Goal:** a physical robot joins the fleet alongside simulated ones.

- **Determine Spot's actual ROS version first.** This decides the whole phase:
  - ROS 2 driver → extend `adapter_ros2`, smallest path.
  - ROS 1 driver → `adapter_ros1` in a Noetic container **on the robot or a companion
    machine**, never on the 24.04 workstation.
  - Neither / uncertain → `adapter_spot` against the Boston Dynamics Python SDK,
    bypassing ROS entirely. Often the most robust option.
- Whichever path: camera goes to MediaMTX over RTSP, unchanged.
- Mixed-fleet test: one real robot + simulated robots in one session.

**Demo:** a physical robot and simulated robots on the same map, in one GUI.
**Exit:** NFR-9 — the real robot required one new adapter and no other module change.

> Nothing before this phase depends on knowing Spot's ROS version. That uncertainty is
> deliberately isolated behind the adapter contract.

---

## Phase 9 — Hardening
**Goal:** trustworthy runs.

- 15-minute 4-robot soak: no frame drops, no logging gaps (NFR-2).
- Failure drills: kill an adapter mid-run, drop a camera, fill the disk, reload the
  browser mid-navigation.
- Determinism check: same seed twice, compare start poses and target placement.
- Tag a release and freeze configs before data collection.

**Exit:** two consecutive clean soak runs; determinism verified.

---

## Risk register

| Risk | Impact | Mitigation |
|---|---|---|
| Adapter contract changes late | Every adapter reworked | Fixed and CI-validated in Phase 1, before any adapter exists |
| Spot's ROS version unknown | Could block real-robot integration | Isolated behind the adapter seam; Phase 8 has three documented paths including a ROS-free one |
| ROS 1 cannot run on the 24.04 host | No native ROS 1 path at all | Adapters never share a ROS graph; `adapter_ros1` ships as a Noetic container |
| Grid registration unreliable on sparse maps (originally written against `multirobot_map_merge`) | Blocks unknown-start condition | `static` mode ships first and remains permanent; `support`/`ratio`/`yaw_ratio` make a weak match refuse rather than guess |
| 4 robots + 4 cameras too heavy for one workstation | Frame drops, unusable GUI | Measure in Phase 5; reduce camera resolution and lidar rate before robot count |
| Orphaned `gz sim` processes hold DDS ports | Silent cross-test contamination | Solved in Phase 0, enforced in CI |
| GUI leaks planner-specific commands | ROS 1 robots cannot accept goals | Phase 3 note; contract review in CI |
| Scope drift toward a production product | Indefinite delay | Non-goals in `architecture.md` §10 are binding |

## Parallelization

**Phases 0 and 1 block everything.** After them:

- Phases 2→3 are a chain (map, then control).
- Phase 4 (video) is independent — a second person can take it straight from Phase 1.
- Phase 6 (perception) needs only Phase 2 and a camera; it can run alongside Phase 5.
- Phase 8 (real robots) needs only Phase 1's contract, plus hardware access. It can
  start any time after Phase 3 and does not block Phases 5–7.
