# SwarmDeck Adapter Protocol v1

The **only** interface between robots and the backend. The backend has no ROS
dependency; any robot that speaks this protocol joins the fleet.

Implement it in any language, on any OS, against any ROS version (or none).

## Channels

| Direction | Channel | Purpose |
|---|---|---|
| adapter → backend | `WS /adapter` | `hello`, `robot_state`, `nav_status`, `detections`, `map_meta` |
| backend → adapter | same socket | `navigate_to`, `cancel_goal`, `drive`, `stop`, `set_mode` |
| adapter → backend | `POST /api/adapter/map` | occupancy grid, throttled ≤ 1 Hz |
| adapter → backend | `POST /api/adapter/camera` | optional JPEG preview fallback, ≤ 5 Hz |
| adapter → MediaMTX | `RTSP push :8554/<robot_id>` | H.264 camera |

## Handshake

First message on connect. The backend replies `hello_ack` or closes with a reason.

```jsonc
{ "type": "hello", "protocol": 1,
  "robot_id": "spot_0", "robot_type": "spot",
  "adapter": "adapter_spot/0.3.1", "ros": "noetic",
  "coordinate_frame": "local",
  "capabilities": ["navigate", "camera", "map", "battery", "estop"],
  "footprint_radius": 0.55 }
```

`coordinate_frame` defaults to `local`: poses, goals, and occupancy grids are in that
robot's own navigation-map frame. An adapter whose data already uses the backend's
merged frame (for example the synthetic mock adapter) declares `merged` instead.

`capabilities` drives the GUI. A robot without `navigate` shows no goal controls; one
without `map` contributes no grid. **Never assume a capability.**

| Capability | Meaning |
|---|---|
| `navigate` | accepts `navigate_to` / `cancel_goal` |
| `map` | pushes occupancy grids |
| `camera` | streams to MediaMTX under its `robot_id` |
| `battery` | reports `battery` in `robot_state` |
| `estop` | accepts `stop` |

## Adapter → backend

```jsonc
// robot_state — 5 Hz, required
{ "type": "robot_state", "robot_id": "robot_0", "t_mono": 18234.55,
  "pose": {"x": 1.2, "y": -3.4, "yaw": 0.78},
  "battery": 0.82, "mode": "nav", "nav_status": "active",
  "goal": {"x": 5.0, "y": 2.0},
  "planned_path": [{"x": 1.2, "y": -3.4}, {"x": 2.1, "y": -2.7}] }

// detections — on detection
{ "type": "detections", "robot_id": "robot_2", "t_mono": 18251.7,
  "camera": "front",
  "items": [{"id": "duck_0", "class": "rubber_duck", "score": 0.91,
             "bbox": [0.41, 0.33, 0.12, 0.18],
             "map_position": {"x": 7.1, "y": -2.2}}] }

// map_meta — after POST /api/adapter/map
{ "type": "map_meta", "robot_id": "robot_0", "t_mono": 18260.0,
  "resolution": 0.05, "width": 1000, "height": 1000,
  "origin": {"x": -25.0, "y": -25.0} }
```

`nav_status` ∈ `idle` | `active` | `succeeded` | `failed` | `cancelled`
`mode` ∈ `idle` | `nav` | `teleop` | `estop`

`planned_path` is the adapter's current planner output, downsampled to a bounded
polyline. The GUI renders it dashed beside the solid path actually travelled.
Detection boxes use normalized `[x, y, width, height]` image coordinates; stable item
IDs let the backend update one observation instead of stacking duplicate boxes.

## Backend → adapter

```jsonc
{ "type": "navigate_to", "seq": 41, "goal": {"x": 4.0, "y": 1.5, "yaw": 0.0} }
{ "type": "cancel_goal", "seq": 42 }
{ "type": "drive",       "seq": 43, "linear": 0.28, "angular": 0.0 }
{ "type": "stop",        "seq": 44 }
{ "type": "set_mode",    "seq": 45, "mode": "teleop" }
```

**Commands are planner-agnostic.** The adapter maps `navigate_to` to Nav2, `move_base`,
or a vendor SDK. The GUI must never send planner-specific fields.

For the default `local` frame, `navigate_to.goal` is expressed in that robot's local
navigation map. The backend converts between it and the shared merged-map frame, using
the same transform as map merging. Adapter `robot_state.pose` and `robot_state.goal` use
the declared frame too; GUI clients always receive shared-frame coordinates.

## Map upload

```
POST /api/adapter/map?robot_id=robot_0
Content-Type: application/octet-stream
Body: zlib-compressed int8[] row-major, -1 unknown, 0 free, 100 occupied
```

Send `map_meta` on the WebSocket immediately after, so the backend can interpret it.

## Map-from-scan upload

For a robot whose SLAM stack registers a point cloud but never projects one to a 2D
`OccupancyGrid` (a LOAM-family pipeline like LVI-SAM is the common case) — send scans
instead of a finished grid, and the backend raytraces them into one itself
(`mapsvc/scan_grid.py`), then feeds it through the exact same merge/registration path a
native grid uses:

```
POST /api/adapter/scan?robot_id=robot_0&origin_x=1.2&origin_y=-3.4
Content-Type: application/octet-stream
Body: zlib-compressed int16[] xy pairs, 1 cm units (points * 100, rounded)
```

`origin_x`/`origin_y` are the sensor's position, in the same frame as the points, at
capture time — used to raytrace free space from the sensor back to each return. Points
should already be deduplicated onto (roughly) the grid's resolution before upload; a raw
registered scan is far denser than a raytraced grid needs. Mutually exclusive in
practice with `POST /api/adapter/map` per robot, not mutually exclusive in the protocol —
an adapter advertising `map` because it configures this path rather than a native grid
topic is exactly protocol rule 4 working as intended.

## Camera preview fallback

MediaMTX/WHEP remains the production low-latency path. During development, or when that
pipeline is unavailable, an adapter may send its newest browser-ready JPEG to:

```text
POST /api/adapter/camera?robot_id=robot_0
Content-Type: image/jpeg
```

Throttle previews to 5 Hz. The GUI first attempts WHEP, then polls
`GET /api/camera/<robot_id>` without caching. This fallback keeps the backend ROS-free
and makes camera wiring testable; it is not the Phase 4 latency solution.

## Rules

1. `t_mono` is the adapter's monotonic clock in seconds. The backend adds wall and
   session time on receipt.
2. Reconnect with backoff. Re-send `hello` every time.
3. Unknown message types are ignored, not fatal — forward compatibility.
4. Never send a capability you cannot honour.
5. `robot_id` must be stable across reconnects; it is the identity key everywhere.
