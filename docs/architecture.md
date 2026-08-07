# SwarmDeck — Architecture

## 1. Principles

1. **The backend knows nothing about ROS.** Robots connect through an adapter contract.
   This is what makes a mixed ROS 1 / ROS 2 / simulated fleet possible.
2. **Four data classes, four transports.** Commands, telemetry, map, and video have
   opposite requirements. One channel cannot serve them. See §3.
3. **The backend owns state; the browser is a view.** Reload must never lose the map.
4. **Adopt, don't author.** SLAM Toolbox, RTAB-Map, Nav2, MediaMTX, MCAP. Custom code only
   for adapters, map service (including registration), and GUI. Registration is the one
   place this principle is knowingly bent: `multirobot_map_merge` is the obvious package to
   adopt, but it is ROS-node-shaped and would put ROS inside the backend, breaking principle
   1 and the mixed ROS 1 / ROS 2 fleet it exists for. The cost of that decision is that the
   algorithm — grid-to-grid correlation with rejection tests — had to be written and
   calibrated here instead of inherited. §5.2 documents it as a first-class component
   accordingly.
5. **Config is data.** Study geometry stays in versioned YAML; operator preferences
   are validated and atomically persisted as JSON.
6. **Every module testable alone**, with the rest mocked.

## 2. System diagram

```
┌────────────────────── Operator Station (browser) ─────────────────────┐
│  2D map (Canvas2D) · camera panel (WebRTC) · robot cards · alerts     │
└────────┬──────────────────┬────────────────────┬──────────────────────┘
         │ WebSocket        │ HTTP              │ WebRTC/WHEP
         │ state + cmd      │ map               │ video
┌────────▼──────────────────▼───────────────────┐  ┌───────────────────┐
│  Backend  (Python · FastAPI · NO ROS)         │  │  MediaMTX  (SFU)  │
│   ├─ api/      WS hub, REST, schema validation│  │ RTSP in→WebRTC out│
│   ├─ fleet/    adapter registry + sessions    │  └─────────▲─────────┘
│   ├─ mapsvc/   grid merge, patches, transforms│            │ RTSP
│   ├─ detect/   detection fusion + dedup       │            │
│   ├─ events/   operator action log            │            │
│   └─ record/   MCAP + JSONL session writer    │            │
└────────▲──────────────────────────────────────┘            │
         │ Adapter Protocol  (WebSocket + HTTP)              │
         │                                                   │
┌────────┴───────────┬──────────────────┬───────────────────┴─────────┐
│ adapter_sim        │ adapter_ros2     │ adapter_ros1 / adapter_spot │
│ (Gazebo fleet)     │ (rclpy, Jazzy)   │ (rospy or vendor SDK)       │
├────────────────────┼──────────────────┼─────────────────────────────┤
│ Gazebo Harmonic    │ real ROS 2 robot │ ROS 1 robot, own container  │
│ 4× robot, sensors  │                  │ (Noetic) or Spot SDK        │
│ SLAM Toolbox, Nav2 │                  │ move_base / vendor autonomy │
└────────────────────┴──────────────────┴─────────────────────────────┘
```

## 3. Network architecture

### 3.1 Transport per data class

| Class | Rate / size | Latency | Loss policy | Transport |
|---|---|---|---|---|
| **Command** | tiny, bursty | critical | must not lose | WebSocket, acked, seq numbered |
| **Telemetry** | ~1 KB @ 5–10 Hz | soft | drop freely | WebSocket, best effort |
| **Map** | 20–200 KB full, ~2 KB patches | tolerant | must arrive | HTTP full + WS patches |
| **Video** | 1–2 Mbps/camera | critical | drop frames, never retransmit | WebRTC via SFU |

A 2D grid is small enough that the 3D tile machinery is unnecessary: full grid over
HTTP on connect (browser-cacheable, survives reload), incremental patches over WS.

### 3.2 Adapter protocol

The **only** interface between robots and the backend. Deliberately boring so it can be
implemented in any language, on any OS, against any ROS version.

| Direction | Channel | Content |
|---|---|---|
| adapter → backend | WebSocket `/adapter` | `hello`, `robot_state`, `nav_status`, `detections`, `map_update` |
| backend → adapter | same socket | `navigate_to`, `cancel_goal`, `stop`, `set_mode` |
| adapter → backend | HTTP POST `/api/adapter/map` | full `OccupancyGrid`, throttled |
| adapter → MediaMTX | RTSP push `:8554` | H.264 camera stream |

Connection handshake declares identity and capabilities:

```jsonc
{ "type": "hello", "robot_id": "spot_0", "robot_type": "spot",
  "adapter": "adapter_spot/0.3.1", "ros": "noetic",
  "capabilities": ["navigate", "camera", "map", "battery", "estop"],
  "map_frame": "map", "footprint_radius": 0.55 }
```

The GUI renders from declared capabilities — a robot without `map` contributes no grid,
one without `navigate` shows no goal controls. No backend change to add a robot type.

### 3.3 Deployment topologies

**A — Single workstation (default).** Everything local. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
fixed `ROS_DOMAIN_ID`.

**B — Sim host + operator host.** One LAN switch.

**C — Mixed real fleet.** Each robot runs its adapter locally and connects outward to
the backend over TCP. **Robots do not share a ROS graph** — no cross-machine DDS, no
`ros1_bridge`, no discovery tuning. Each robot's ROS domain stays entirely private.

```
robot (ROS 1 / Noetic ctr) ─┐
robot (ROS 2 / Jazzy)      ─┼── WS + HTTP :8080 ──→ backend ──→ browser
sim-host (Gazebo ×4)       ─┘   RTSP :8554     ──→ MediaMTX ──┘
```

Topology C is the reason the adapter seam exists. It is also the only one that works
here: Noetic is EOL, cannot install on Ubuntu 24.04, and `ros-jazzy-ros1-bridge` does
not exist.

### 3.4 Ports

| Port | Service | Protocol |
|---|---|---|
| 8080 | Backend HTTP + WebSocket (GUI and adapters) | TCP |
| 8554 | MediaMTX RTSP ingest | TCP |
| 8889 | MediaMTX WebRTC / WHEP | TCP + UDP ephemeral |
| 7400+ | DDS, **within a single host only** | UDP |

### 3.5 Bandwidth budget (4 robots)

| Stream | Estimate |
|---|---|
| Video, 4× 640×480 @15 fps H.264 | ~6 Mbps |
| Telemetry, 4× 10 Hz | < 0.2 Mbps |
| Map patches + periodic full grids | < 1 Mbps |
| **Total** | **< 10 Mbps** — trivial on LAN; the budget exists to catch regressions |

## 4. Module architecture

### 4.1 Adapters

Each runs in the robot's own environment. Shared contract, independent implementations.

| Adapter | Runs on | Talks to |
|---|---|---|
| `adapter_sim` | Sim host | Gazebo fleet via rclpy; one process for all simulated robots |
| `adapter_ros2` | ROS 2 robot | rclpy — SLAM Toolbox, Nav2, camera |
| `adapter_ros1` | ROS 1 robot, Noetic container | rospy — `gmapping`/`slam_toolbox` ROS 1, `move_base` |
| `adapter_spot` | Spot or a companion machine | ROS 1 driver **or** the Boston Dynamics Python SDK directly |
| `adapter_mock` | Anywhere | Nothing — synthetic robot for testing the backend without ROS |

**Adapter responsibilities:** publish `robot_state` @ 5 Hz, push its occupancy grid,
publish detections, accept `navigate_to`/`cancel_goal`/`stop`, report nav status,
stream camera to RTSP.

Because Spot's ROS version is uncertain, `adapter_spot` may bypass ROS entirely and use
the vendor SDK. The contract does not care — that is the point of the seam.

### 4.2 ROS 2 packages (simulation and ROS 2 robots)

| Package | Responsibility |
|---|---|
| `swarmdeck_description` | URDF/xacro robot model, sensor config |
| `swarmdeck_sim` | Gazebo worlds, model spawning, seeded scenario generator, reactive explorer |
| `swarmdeck_slam` | Per-namespace localisation and mapping: wheel/gyro EKF, sensor static TFs, SLAM Toolbox or RTAB-Map, plus cloud→scan slicing when the lidar has multiple rings |
| `swarmdeck_nav` | Nav2 params and per-namespace bringup |
| `adapters/perception` | Portable RGB perception with no ROS/Gazebo dependency |
| `swarmdeck_media` | Camera → GStreamer → RTSP encoder node |
| `swarmdeck_bringup` | Top-level launch, config resolution, one-command start |

No custom ROS messages. Everything crossing the seam uses the adapter protocol, so
message definitions never need duplicating across ROS versions.

### 4.3 Backend modules

One process, strictly separated modules, no cross-imports — all communication through
the internal bus.

| Module | Responsibility |
|---|---|
| `api/` | FastAPI routes, WebSocket hubs (GUI + adapter), schema validation |
| `fleet/` | Adapter registry, capability negotiation, liveness, command routing |
| `mapsvc/` | Per-robot grids, transform estimation, merged grid, patch generation |
| `detect/` | Detection aggregation, dedup, map-entity lifecycle |
| `events/` | Operator action log, JSONL writer |
| `record/` | MCAP recording, session directory, replay source |
| `config/` | YAML load, schema check, snapshot into session |

**Internal bus:** in-process async pub/sub (`asyncio` queues keyed by topic). Enables
replay — `record/` drives the bus with no adapters connected, so the whole GUI is
testable with no simulation and no ROS.

## 5. Map merging

### 5.1 Pipeline

```
wheel odom ──┐
             ├─► EKF (robot_localization) ──► odom → base_link
gyro 200 Hz ─┘                                     │
                                                   ↓
lidar_rings=1:  Gazebo LaserScan ──────────────┐   (default)
lidar_rings=N:  PointCloud2 → cloud_to_scan ───┤   (odd N; ring at elevation 0)
                                               ↓
                        slam_backend:=toolbox → SLAM Toolbox   (per robot, 2D)
                        slam_backend:=rtabmap → RTAB-Map       (per robot, 3D → 2D grid)
                                               ↓
                                     OccupancyGrid per robot
                                               ↓
                          Map Service: transform estimation + grid merge
                                               ↓
                            merged OccupancyGrid → GUI + Nav2 costmaps
```

The EKF exists because SLAM only searches a small window around its motion prior, so the
prior has to be roughly right before scan matching can do anything. It fuses wheel
*velocity* with gyro *rate* — never wheel-derived heading, which is what slip destroys, and
never the IMU's absolute orientation, which no magnetometer-free MEMS IMU actually has.
Measured over 120 s of driving it cuts odometry drift 3-45x (21 m travelled: 1.87 m → 0.04 m).
`fuse_imu:=false` reverts to raw wheel odometry, and the drive plugin's TF bridge is dropped
whenever the EKF is on so only one node publishes `odom → base_link`.

Either backend maintains its own pose graph and republishes a corrected grid after loop
closure, so the map service consumes already-corrected grids and needs no keyframe
machinery of its own. That is also the boundary of the design: correction is *per robot*.
The map service aligns finished grids and never feeds one robot's observations back into
another's graph, so inter-robot drift is stitched over rather than corrected. See
`docs/collaborative-slam.md` for what changes if that is required.

SLAM Toolbox is the default: single-ring lidar, 2D scan matching, Ceres pose graph, and
the lightest dependency set. RTAB-Map is wired as an alternative (`slam_backend:=rtabmap`)
for the case the user's real robots present — point cloud plus camera plus uncorrected
odometry — where ICP over the full cloud and RGB appearance-based loop closure are
stronger than 2D scan matching. It is implemented but not yet validated against Gazebo.

### 5.2 Merge modes

| Mode | How | When |
|---|---|---|
| `static` | Start poses from config → fixed `T_world_robot` | Operator explicitly trusts surveyed starts. |
| `auto` | Grid registration estimates `T_world_robot` | Unknown starts (FR-S4) |

In `auto`, configured starts are safety/search priors only. They are not registration
evidence and are never used to overlay unregistered grids. Until a match is accepted,
selecting a robot displays its native local SLAM grid. Once matches are accepted, the
reference and its accepted peers form the global-map membership; robots outside that
connected set continue to show local maps. `static` remains available when surveyed
transforms are an explicit deployment guarantee.

**Registration algorithm** (`mapsvc/registration.py`, numpy only). Three stages, each
sampling yaw finely enough to resolve its own correlation peak:

```
coarse   4.0 deg over the search window, reference dilated so the peak is 4 deg wide
medium   0.5 deg around the coarse winner, undilated — ranks rival rotations (yaw_ratio)
fine     0.1 deg around the medium winner, then parabolic interpolation to sub-cell
```

Within each yaw candidate, FFT cross-correlation evaluates *all* translations in one
O(n log n) pass, so cost is one transform per yaw candidate rather than a full 3D sweep.
The correlation is **signed**: occupied-on-occupied scores +1, occupied-on-free −1, with a
one-cell neutral guard band around walls so that a near-miss is not punished as hard as
driving a wall through a known-empty room. Free space is what disambiguates buildings whose
walls alone are ambiguous. Needs no inter-robot communication, which matters for a mixed
ROS 1 / ROS 2 fleet that cannot share a SLAM system, and no OpenCV/PCL.

The dilated coarse stage is the load-bearing detail. A yaw error of `dyaw` displaces a point
at radius `r` by `r·dyaw`, so at 20 m the undilated peak is well under a degree wide and a
4 deg sweep steps straight over it — which is exactly how the first implementation produced
confident 90 deg errors (see `docs/KNOWN_ISSUES.md`).

Four **rejection tests** guard the result, all necessary:

| Metric | Rejects | Threshold |
|---|---|---|
| `score` | Weak agreement — wrong or too little to see | `>= 0.20` |
| `ratio` | Rival *translation* at the winning yaw scores nearly as well | `<= 0.80` |
| `yaw_ratio` | Rival *rotation* scores nearly as well (symmetric building) | `<= 0.80` |
| `support` | Fraction of the smaller map's known area shared with the reference | `>= 0.35` |

Plus `overlap >= 80` occupied cells in common. If a configured start pose is available, the
candidate must also remain within 2 m and 30 degrees of that safety prior; otherwise it is
rejected and remains local.

Measured against Gazebo ground truth with two robots at unknown relative poses:
**7.8 cm translation error, exact yaw**. On a 52-heading synthetic sweep after the rewrite:
52/52 correct at 0.1–3 cm and under 0.1 deg, and a rotationally symmetric building is
refused rather than guessed.

Once a robot's transform is accepted, its yaw is cached and subsequent uploads refine
within a ±8 deg window instead of re-sweeping 360 deg. Ingest runs on a worker thread
(`asyncio.to_thread` under a lock) so a ~190 ms registration never blocks the event loop
and the WebSocket telemetry stream.

Registration requires the robots to have **seen the same places**. With disjoint coverage
the problem is ill-posed and `support` refuses — see `docs/KNOWN_ISSUES.md`.

### 5.3 Map Service state

```python
grids:      robot_id → {stamp, origin, resolution, w, h, data}   # latest per robot
transforms: robot_id → T_world_robot                             # 2D: x, y, theta
merged:     {origin, resolution, w, h, data}                     # derived
members:    set[robot_id]                                        # accepted global component
```

Merged grid is regenerated when any robot's grid or transform changes, then diffed
against the previous version to emit a patch.

### 5.4 Map transport

- **Full grid**: `GET /api/map` → PNG (grey = unknown, black = occupied, white = free)
  plus a JSON sidecar for origin and resolution. ETag-cached, fetched on connect and
  reload.
- **Local grid**: `GET /api/map/local/{robot_id}` plus `/info` → that robot's raw SLAM
  map and native frame, used whenever it is outside the accepted global component.
- **Patches**: changed bounding box only, over WebSocket, throttled to 2 Hz.

```jsonc
{ "type": "map_patch", "seq": 218, "resolution": 0.05,
  "origin": {"x": -25.0, "y": -25.0},
  "x0": 412, "y0": 388, "w": 64, "h": 48,
  "data": "<base64 zlib int8[]>" }
```

Occupancy grids are mostly unknown space and compress heavily — a 50 m × 50 m map at
5 cm is ~1 MB raw and tens of KB compressed.

### 5.5 Accuracy scoring

Gazebo publishes ground-truth poses. A test node compares estimated `T_world_robot`
against truth and reports translation and rotation error per robot. CI asserts on the
number rather than on a screenshot.

## 6. Video pipeline

```
camera → /robot_N/camera/image_raw
   → swarmdeck_media node (cv_bridge → GStreamer appsrc)
   → x264enc  tune=zerolatency speed-preset=ultrafast key-int-max=30
   → rtspclientsink → MediaMTX :8554
   → MediaMTX WHEP :8889 → browser RTCPeerConnection → <video>
```

**Why an SFU:** each robot encodes once regardless of viewer count. It is also
ROS-agnostic — a ROS 1 robot or a vendor SDK pushes RTSP the same way, so video needs no
adapter-specific work.

**Browser side:** WHEP (HTTP POST of an SDP offer) — no signalling server to write.
Video renders into `<video>`; detections draw on an overlaid `<canvas>` sized to the
video element. **No iframes.**

Latency budget (target < 300 ms):

| Stage | Budget |
|---|---|
| Capture + transport | 40 ms |
| Encode (ultrafast, zerolatency) | 30 ms |
| RTSP → MediaMTX → WebRTC | 50 ms |
| Network (LAN) | 10 ms |
| Jitter buffer + decode + paint | 120 ms |
| **Total** | **~250 ms** |

Measured with a timestamp burned into the frame and read back from a screen capture.

### 6.1 Portable rubber-duck perception

`adapters/perception/duck_detector.py` accepts an ordinary OpenCV BGR frame and returns
normalized adapter-protocol boxes. It imports neither ROS nor Gazebo and never reads
simulation entity names, positions, segmentation buffers, or ground truth. The current
real-time baseline combines yellow-body and adjacent orange-beak evidence; the
`detect_bgr` boundary is the replacement point for a trained ONNX implementation after
licensed, representative real-camera data is available. This keeps transport, UI, and
real-robot integration unchanged when the model is upgraded.

Hardware adapters join each RGB box to either an aligned depth image plus CameraInfo or
an organised RGB-aligned PointCloud2. They select a coherent foreground depth band,
deproject it to camera XYZ, and use tf2 to express the result in the robot's map frame.
The protocol, backend, and 2D map already treat that optional `map_position` as a
persistent marker. Invalid or stale depth is bbox-only; it is never replaced by a guessed
range.

## 7. Frontend

**Stack:** TypeScript, Svelte, Canvas2D, Vite. No three.js — the map is 2D.

**Layout:** map centre, camera panel right, robot cards left, alerts top. Responsive,
touch-first, single-screen.

| Store | Contents | Source |
|---|---|---|
| `fleet` | per-robot pose, battery, mode, nav state, capabilities, attention timer | WS |
| `map` | `ImageData` grid + origin/resolution, patch-applied | HTTP + WS patches |
| `detections` | map entities with position and provenance | WS |
| `alerts` | active alerts, ack state | WS |
| `settings` | thresholds, fleet inventory, perception controls | HTTP + WS |
| `ui` | selection, active camera, view transform | local |

**Rules**

- WebSocket only, never long-polling.
- Map renders to an offscreen canvas; patches blit into it. Never re-fetch the whole
  grid for an incremental change.
- Robots, actual trails, dashed planner paths, goals, and detections draw as overlay
  layers above the map canvas.
- On reconnect, fetch the full grid once, then resume patches.
- Every operator action goes through one `sendAction()` chokepoint that stamps and logs
  it — so the event log cannot drift from what the UI actually did.

**Navigation sensing:** the top 360-degree lidar feeds SLAM and both costmaps for walls
and room geometry. A bumper-height forward proximity scan feeds only Nav2's obstacle
layers so another low robot cannot pass below the mapping beam. Sensor topics are fully
qualified because costmap plugins run in nested `local_costmap` / `global_costmap`
namespaces.
- Render only from declared capabilities; never assume a robot can navigate or map.

## 8. Backend API

```
GET  /api/config                 resolved config + versions
GET  /api/settings               persisted operator settings
PUT  /api/settings               validate, atomically save, and broadcast settings
GET  /api/fleet                  connected robots and capabilities
GET  /api/session                current session state
POST /api/session/start|stop
GET  /api/map                    full merged grid, PNG + JSON sidecar, ETag
POST /api/map/reset/{robot_id}   clear one stationary robot's accumulated map products
POST /api/map/reset              clear all accumulated maps while the fleet is stationary
POST /api/adapter/map            adapter pushes its grid
WS   /ws                         GUI ↔ backend
WS   /adapter                    adapter ↔ backend
```

Server → GUI: `robot_state`, `map_patch`, `detection`, `alert`, `fleet_change`, `session_state`
GUI → server: `set_goal`, `cancel_goal`, `select_robots`, `switch_camera`,
`acknowledge_alert`, `report_target`, `stop_all`

```jsonc
// robot_state, 5 Hz per robot
{ "type": "robot_state", "robot_id": "robot_0",
  "t_mono": 18234.55, "t_wall": 1754038800.1, "t_sess": 142.5,
  "pose": {"x":1.2,"y":-3.4,"yaw":0.78},
  "battery": 0.82, "mode": "nav", "nav_status": "active",
  "goal": {"x":5.0,"y":2.0},
  "planned_path": [{"x":1.2,"y":-3.4},{"x":2.1,"y":-2.7}],
  "unattended_s": 12.4 }

// operator action, GUI → server, mirrored into events.jsonl
{ "type": "set_goal", "seq": 41, "robot_id": "robot_1",
  "t_mono": 18240.11, "payload": {"x":4.0,"y":1.5} }
```

Three timestamps on every record: `t_mono` (drift-free intervals), `t_wall`
(cross-host correlation), `t_sess` (seconds since session start).

All messages are JSON, schema-validated from `server/schemas/`. Invalid payloads are
rejected loudly, never silently coerced.

## 9. Repository structure

```
swarmdeck/
├── docs/                          architecture.md · requirements.md · roadmap.md
├── adapters/
│   ├── protocol/                  SHARED CONTRACT — schemas + reference client
│   ├── perception/                portable RGB detectors
│   ├── adapter_sim/
│   ├── adapter_ros2/
│   ├── adapter_ros1/              Noetic container, Dockerfile included
│   ├── adapter_spot/              ROS 1 driver or BD SDK
│   └── adapter_mock/
├── swarmdeck_ros/src/
│   ├── swarmdeck_description/     urdf/ meshes/ config/
│   ├── swarmdeck_sim/             worlds/ launch/ scenario/
│   ├── swarmdeck_slam/            slam_toolbox / rtabmap launch + cloud_to_scan
│   ├── swarmdeck_nav/             config/nav2_params.yaml launch/
│   ├── swarmdeck_media/
│   └── swarmdeck_bringup/         launch/session.launch.py
├── server/
│   ├── swarmdeck_server/
│   │   ├── api/  fleet/  mapsvc/  detect/  events/  record/  config/
│   │   └── bus.py
│   ├── schemas/                   *.schema.json
│   └── tests/
├── ui/
│   ├── src/  lib/{map2d,video,cards,alerts}/  stores/  api/
│   └── vite.config.ts
├── study/                         1robot.yaml · 2robot.yaml · 4robot.yaml
├── analysis/                      session loaders, metric scripts
├── sessions/                      recorded output (gitignored)
├── tests/integration/             headless sim end-to-end
├── docker/                        backend, ui, adapter_ros1 (Noetic)
└── Makefile                       make sim · make server · make ui · make test
```

`adapters/protocol/` is the contract of record. Changing it is a versioned, deliberate
act; every adapter validates against it in CI.

**Session directory:**

```
sessions/S07_4robot_20260801T143000/
├── manifest.json         config, seed, git SHA, adapter versions
├── fleet.mcap            all robot state and map updates
├── events.jsonl          operator actions and system events
├── map_final.png         final merged map + sidecar
├── merge_accuracy.csv    per-robot error vs ground truth
└── notes.md
```

## 10. Deliberate non-goals

- **No 3D map in the GUI.** The operator view is a 2D occupancy grid: no three.js, no tiles,
  no voxels. RTAB-Map may consume the full point cloud internally and project a 2D grid out,
  but the merged map and everything the operator sees stay 2D.
- **No shared pose graph across robots.** Each robot's SLAM is private and the backend merges
  finished grids. This follows from the ROS-free backend rather than from the mapping being
  easy; `docs/collaborative-slam.md` states what it costs and what lifting it would take.
- **No shared ROS graph across machines.** Adapters connect over TCP; each robot's ROS
  domain stays private. No `ros1_bridge`, which does not exist for Jazzy anyway.
- **No custom ROS messages.** The adapter protocol is the interface.
- **No Zenoh.** LAN only; the adapter protocol already decouples ROS versions.
- **No microservices.** One backend process, modular inside.
- **No database.** A session is a directory.
- **No auth.** Single operator, lab network.
