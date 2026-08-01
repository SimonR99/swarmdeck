# SwarmDeck Adapter Protocol v1

The **only** interface between robots and the backend. The backend has no ROS
dependency; any robot that speaks this protocol joins the fleet.

Implement it in any language, on any OS, against any ROS version (or none).

## Channels

| Direction | Channel | Purpose |
|---|---|---|
| adapter → backend | `WS /adapter` | `hello`, `robot_state`, `nav_status`, `detections`, `map_meta` |
| backend → adapter | same socket | `navigate_to`, `cancel_goal`, `stop`, `set_mode` |
| adapter → backend | `POST /api/adapter/map` | occupancy grid, throttled ≤ 1 Hz |
| adapter → MediaMTX | `RTSP push :8554/<robot_id>` | H.264 camera |

## Handshake

First message on connect. The backend replies `hello_ack` or closes with a reason.

```jsonc
{ "type": "hello", "protocol": 1,
  "robot_id": "spot_0", "robot_type": "spot",
  "adapter": "adapter_spot/0.3.1", "ros": "noetic",
  "capabilities": ["navigate", "camera", "map", "battery", "estop"],
  "footprint_radius": 0.55 }
```

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
  "goal": {"x": 5.0, "y": 2.0} }

// detections — on detection
{ "type": "detections", "robot_id": "robot_2", "t_mono": 18251.7,
  "camera": "front",
  "items": [{"class": "duck", "score": 0.91,
             "bbox": [0.41, 0.33, 0.12, 0.18],
             "map_position": {"x": 7.1, "y": -2.2}}] }

// map_meta — after POST /api/adapter/map
{ "type": "map_meta", "robot_id": "robot_0", "t_mono": 18260.0,
  "resolution": 0.05, "width": 1000, "height": 1000,
  "origin": {"x": -25.0, "y": -25.0} }
```

`nav_status` ∈ `idle` | `active` | `succeeded` | `failed` | `cancelled`
`mode` ∈ `idle` | `nav` | `teleop` | `estop`

## Backend → adapter

```jsonc
{ "type": "navigate_to", "seq": 41, "goal": {"x": 4.0, "y": 1.5, "yaw": 0.0} }
{ "type": "cancel_goal", "seq": 42 }
{ "type": "stop",        "seq": 43 }
{ "type": "set_mode",    "seq": 44, "mode": "teleop" }
```

**Commands are planner-agnostic.** The adapter maps `navigate_to` to Nav2, `move_base`,
or a vendor SDK. The GUI must never send planner-specific fields.

## Map upload

```
POST /api/adapter/map?robot_id=robot_0
Content-Type: application/octet-stream
Body: zlib-compressed int8[] row-major, -1 unknown, 0 free, 100 occupied
```

Send `map_meta` on the WebSocket immediately after, so the backend can interpret it.

## Rules

1. `t_mono` is the adapter's monotonic clock in seconds. The backend adds wall and
   session time on receipt.
2. Reconnect with backoff. Re-send `hello` every time.
3. Unknown message types are ignored, not fatal — forward compatibility.
4. Never send a capability you cannot honour.
5. `robot_id` must be stable across reconnects; it is the identity key everywhere.
