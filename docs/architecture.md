# SwarmDeck — Architecture

## 1. Principles

1. **The backend knows nothing about ROS.** Robots connect through an adapter contract.
   This is what makes a mixed ROS 1 / ROS 2 / simulated fleet possible.
2. **Four data classes, four transports.** Commands, telemetry, map, and video have
   opposite requirements. One channel cannot serve them. See §3.
3. **The backend owns state; the browser is a view.** Reload must never lose the map.
4. **Adopt, don't author.** SLAM Toolbox, Nav2, `multirobot_map_merge`, MediaMTX, MCAP.
   Custom code only for adapters, map service, and GUI.
5. **Config is data.** Robot count, thresholds, seeds, merge mode in versioned YAML.
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
| `swarmdeck_sim` | Gazebo worlds, model spawning, seeded scenario generator |
| `swarmdeck_slam` | `pointcloud_to_laserscan` + SLAM Toolbox bringup per namespace |
| `swarmdeck_nav` | Nav2 params and per-namespace bringup |
| `swarmdeck_perception` | ONNX detector node, projection of detections to map frame |
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
3D lidar → pointcloud_to_laserscan → 2D LaserScan
              (height band 0.1–1.8 m)
                        ↓
                  SLAM Toolbox  (per robot, 2D)
                        ↓
              OccupancyGrid per robot
                        ↓
   Map Service: transform estimation + grid merge
                        ↓
        merged OccupancyGrid → GUI + Nav2 costmaps
```

SLAM Toolbox is the right choice now that the map is 2D — it maintains its own pose
graph and republishes a corrected grid after loop closure, so the map service consumes
already-corrected grids and needs no keyframe machinery of its own.

### 5.2 Merge modes

| Mode | How | When |
|---|---|---|
| `static` | Start poses from config → fixed `T_world_robot` | Default. Always available. Build first. |
| `auto` | `multirobot_map_merge` estimates transforms by feature-matching the grids | Unknown starts (FR-S4) |

`multirobot_map_merge` (from `m-explore-ros2`, source build) is purpose-built for
merging N occupancy grids with unknown relative poses. It is far lighter than a
pose-graph SLAM backend and needs no inter-robot communication — which matters for a
mixed fleet, since a ROS 1 robot cannot join a ROS 2 distributed SLAM system.

`static` mode is a permanent fallback, not a stepping stone: if transform estimation
fails or drifts, the system still runs with known start poses.

### 5.3 Map Service state

```python
grids:      robot_id → {stamp, origin, resolution, w, h, data}   # latest per robot
transforms: robot_id → T_world_robot                             # 2D: x, y, theta
merged:     {origin, resolution, w, h, data}                     # derived
```

Merged grid is regenerated when any robot's grid or transform changes, then diffed
against the previous version to emit a patch.

### 5.4 Map transport

- **Full grid**: `GET /api/map` → PNG (grey = unknown, black = occupied, white = free)
  plus a JSON sidecar for origin and resolution. ETag-cached, fetched on connect and
  reload.
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
| `ui` | selection, active camera, view transform | local |

**Rules**

- WebSocket only, never long-polling.
- Map renders to an offscreen canvas; patches blit into it. Never re-fetch the whole
  grid for an incremental change.
- Robots, trails, goals, and detections draw as overlay layers above the map canvas.
- On reconnect, fetch the full grid once, then resume patches.
- Every operator action goes through one `sendAction()` chokepoint that stamps and logs
  it — so the event log cannot drift from what the UI actually did.
- Render only from declared capabilities; never assume a robot can navigate or map.

## 8. Backend API

```
GET  /api/config                 resolved config + versions
GET  /api/fleet                  connected robots and capabilities
GET  /api/session                current session state
POST /api/session/start|stop
GET  /api/map                    full merged grid, PNG + JSON sidecar, ETag
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
  "goal": {"x":5.0,"y":2.0}, "unattended_s": 12.4 }

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
│   ├── adapter_sim/
│   ├── adapter_ros2/
│   ├── adapter_ros1/              Noetic container, Dockerfile included
│   ├── adapter_spot/              ROS 1 driver or BD SDK
│   └── adapter_mock/
├── swarmdeck_ros/src/
│   ├── swarmdeck_description/     urdf/ meshes/ config/
│   ├── swarmdeck_sim/             worlds/ launch/ scenario/
│   ├── swarmdeck_slam/            pointcloud_to_laserscan + slam_toolbox config
│   ├── swarmdeck_nav/             config/nav2_params.yaml launch/
│   ├── swarmdeck_perception/      models/ nodes/
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

- **No 3D map.** Lidar pointclouds reduce to 2D scans. No three.js, no tiles, no voxels.
- **No shared ROS graph across machines.** Adapters connect over TCP; each robot's ROS
  domain stays private. No `ros1_bridge`, which does not exist for Jazzy anyway.
- **No custom ROS messages.** The adapter protocol is the interface.
- **No Zenoh.** LAN only; the adapter protocol already decouples ROS versions.
- **No microservices.** One backend process, modular inside.
- **No database.** A session is a directory.
- **No auth.** Single operator, lab network.
