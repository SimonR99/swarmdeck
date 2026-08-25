# SwarmDeck adapter protocol

This is the only robot/backend interface. The backend supports protocol versions
1 and 2; version 2 adds the optional `slam_graph` message. Adapters may use any
language, OS, middleware, or planner.

## Transport

| Direction | Endpoint | Data |
|---|---|---|
| bidirectional | `WS /adapter` | Registration, telemetry, detections, commands. |
| adapter → backend | `POST /api/adapter/map` | One robot's occupancy grid. |
| adapter → backend | `POST /api/adapter/scan` | Registered XY scan when no grid is available. |
| adapter → backend | `POST /api/adapter/cloud` | Optional registered XYZ cloud. |
| adapter → backend | `POST /api/adapter/keyframe` | Pose-graph keyframe (cloud + odom pose + optional descriptor). |
| collaborative backend → backend | `POST /api/adapter/global_map` | Already-merged common-frame grid. |
| pose-graph back-end → backend | `POST /api/slam/update` | Optimized `T_world_map`, membership, common poses. |
| adapter → MediaMTX | `RTSP :8554/<robot_id>` | Production H.264 video. |

Binary uploads are zlib-compressed and capped by the server. Retry rejected
uploads only after correcting the payload.

## Registration

The first WebSocket message must be `hello`; reconnects send it again. The
backend replies with `hello_ack` or closes the socket.

```jsonc
{
  "type": "hello",
  "protocol": 2,
  "robot_id": "spot_0",
  "robot_type": "spot",
  "adapter": "adapter_ros2/0.3.1",
  "ros": "humble",
  "coordinate_frame": "local",
  "capabilities": ["navigate", "camera", "map", "battery", "network", "estop", "body"],
  "footprint_radius": 0.55,
  "footprint": [[0.50, 0.30], [0.50, -0.30], [-0.50, -0.30], [-0.50, 0.30]]
}
```

`robot_id` is the stable identity key. `coordinate_frame` defaults to `local`,
meaning pose, goal, map, scan, and cloud data use that robot's navigation-map
frame. Use `merged` only when data already uses the backend's shared frame.
`footprint` is optional and is a polygon in the robot's reported `base_frame`,
with x forward and y left. The dashboard draws it when present and falls back
to `footprint_radius`; navigation stacks should use the same polygon for their
collision footprint.

An adapter may also expose an accumulated 3D SLAM cloud as a global map source.
That source is projected into an occupied-only 2D grid in the adapter's map
frame; cells not hit by the cloud remain unknown because a historical cloud
does not contain the individual sensor origins needed to prove free space.

Capabilities drive the UI and command routing:

| Capability | Contract |
|---|---|
| `navigate` | Accept `navigate_to` and `cancel_goal`. |
| `map` | Upload grids or registered scans. |
| `camera` | Publish video under `robot_id`. |
| `battery` | Include `battery` in `robot_state`. |
| `network` | Include synchronized Wi-Fi quality in `robot_state`. |
| `estop` | Accept `stop`. |
| `body` | Accept `body_command`. |
| `reset` | Accept full simulation reset; never advertise on hardware. |

Never advertise a capability the adapter cannot currently honor.

## Adapter messages

`robot_state` is required at 5 Hz:

```jsonc
{
  "type": "robot_state",
  "robot_id": "robot_0",
  "t_mono": 18234.55,
  "pose": {"x": 1.2, "y": -3.4, "yaw": 0.78},
  "battery": 0.82,
  "mode": "nav",
  "nav_status": "active",
  "network": {"interface": "wlan0", "quality_pct": 71.4, "rssi_dbm": -58.0},
  "goal": {"x": 5.0, "y": 2.0},
  "planned_path": [{"x": 1.2, "y": -3.4}, {"x": 2.1, "y": -2.7}],
  "global_planned_path": [{"x": 1.2, "y": -3.4}, {"x": 5.0, "y": 2.0}],
  "local_planned_path": [{"x": 1.2, "y": -3.4}, {"x": 1.7, "y": -3.1}]
}
```

- `mode`: `idle`, `nav`, `teleop`, or `estop`.
- `nav_status`: `idle`, `active`, `succeeded`, `failed`, or `cancelled`.
- `planned_path` is the backward-compatible effective route (local when
  available, otherwise global). `global_planned_path` and
  `local_planned_path` are optional bounded, downsampled polylines that let the
  UI show the full planner route and the currently selected controller route
  separately.
- `network` is optional and must be sampled with the accompanying pose. Use
  `network_iface: auto` or an explicit Linux wireless interface in the supplied
  hardware adapters; an empty value disables it.

Send a complete `detections` batch for the current camera frame:

```jsonc
{
  "type": "detections",
  "robot_id": "robot_2",
  "t_mono": 18251.7,
  "camera": "front",
  "items": [{
    "id": "disc_cone_0",
    "class": "disc_cone",
    "score": 0.91,
    "bbox": [0.41, 0.33, 0.12, 0.18],
    "polygon": [[0.47, 0.33], [0.53, 0.44], [0.41, 0.51]],
    "map_position": {"x": 7.1, "y": -2.2}
  }]
}
```

Boxes and polygon points are normalized image coordinates. IDs are stable
within a class. An item absent from the next batch is no longer visible.
`map_position` is optional and must be derived from valid, fresh depth and TF;
never guess a location.

Adapters capture detections at `detection_capture_floors` from
`GET /api/settings`, not at the operator display floors. The backend keeps the
lower-level evidence so threshold changes remain reversible.

After completing a simulation reset, drop cached maps/clouds and send:

```jsonc
{
  "type": "reset_done",
  "robot_id": "robot_0",
  "t_mono": 18299.1,
  "ok": true,
  "steps": {"world": true, "pose": true, "odometry": true, "slam": true, "costmaps": true}
}
```

The backend waits for acknowledgements before clearing its cached map. Send
`ok: false` with per-step results after partial failure. Hardware adapters must
not advertise or implement reset.

Protocol 2 adapters may report collaborative graph state:

```jsonc
{
  "type": "slam_graph",
  "robot_id": "robot_0",
  "t_mono": 18300.0,
  "keyframes": 82,
  "in_common_frame": true,
  "residual": 0.04,
  "inter_robot": ["robot_1"],
  "common_pose": {"x": 1.0, "y": 2.0, "yaw": 0.1},
  "origin": {"x": 0.9, "y": 2.1, "yaw": 0.1, "frame": "swarm_map"}
}
```

Adapters without a collaborative backend omit this message.

`map_meta` remains accepted after a map upload for compatibility, but grid
metadata is carried by the HTTP request.

## Backend commands

```jsonc
{ "type": "navigate_to", "seq": 41, "goal": {"x": 4.0, "y": 1.5, "yaw": 0.0} }
{ "type": "cancel_goal", "seq": 42 }
{ "type": "drive", "seq": 43, "linear": 0.28, "angular": 0.0 }
{ "type": "stop", "seq": 44 }
{ "type": "set_mode", "seq": 45, "mode": "teleop" }
{ "type": "reset", "seq": 46 }
{ "type": "body_command", "seq": 47, "action": "stand" }
{ "type": "camera_interest", "watched": false }
```

Commands are planner-agnostic. The adapter translates `navigate_to` to Nav2,
`move_base`, or a vendor API. Local-frame goals are converted by the backend
before transmission.

`body_command.action` is `claim`, `release`, `sit`, or `stand`. Ignore it without
the `body` capability.

`camera_interest` is retained as a compatibility command and has no effect on
the H.264 stream or detection. Never gate detection on camera interest.

## Binary payloads

### Occupancy grid

```text
POST /api/adapter/map?robot_id=<id>&resolution=<m>&width=<n>&height=<n>&origin_x=<m>&origin_y=<m>
Content-Type: application/octet-stream
Body: zlib(int8 row-major cells), values -1 unknown, 0 free, 100 occupied
```

### Registered scan

```text
POST /api/adapter/scan?robot_id=<id>&origin_x=<m>&origin_y=<m>&scale=0.01&retain_free_space=0|1
Body: zlib(int16 XY pairs), coordinates = metres / scale
```

Points and sensor origin use the same local map frame. Deduplicate points near
the target grid resolution before upload. Normally a robot sends either grids or
scans, not both. With `retain_free_space=1`, the backend keeps cells crossed by
previous lidar rays known-free (white) instead of aging them back to unknown;
never-observed cells remain unknown.

### Point cloud

```text
POST /api/adapter/cloud?robot_id=<id>&scale=0.01
Body: zlib(int16 XYZ triples), coordinates = metres / scale
```

Send registered, voxel-downsampled points slowly (about 0.25–0.5 Hz). Only
robots accepted into the shared frame contribute to the merged 3D view.

### Collaborative global grid

```text
POST /api/adapter/global_map?resolution=<m>&width=<n>&height=<n>&origin_x=<m>&origin_y=<m>
Body: zlib(int8 row-major cells)
```

Use only for a grid already optimized in the collaborative common frame.

### Pose-graph keyframe

```text
POST /api/adapter/keyframe?robot_id=<id>
Content-Type: application/octet-stream
Body: SDKF blob (see swarmdeck_protocol.keyframe)
```

One voxel-downsampled cloud in the **base frame at capture**, plus `T_odom_base`.
The query-string `robot_id` must match the blob. The server is a dumb pipe: it
checks identity and forwards the opaque body to the SLAM process. Adapters
upload through a bounded queue that **drops** rather than blocking telemetry.

### Pose-graph snapshot

```text
POST /api/slam/update
Content-Type: application/json
```

Published by the SLAM process, not by robots. Carries per-robot `T_world_map`
(`origins`), `in_common_frame` membership, and optimized `common_poses`. The
rendered occupancy still uses `POST /api/adapter/global_map`.

### Nav2 occupancy downlink

```text
GET /api/map/nav/<robot_id>
If-None-Match: <seq>
```

The common-frame grid warped into that robot's own map frame. 404 until the
robot is in a multi-robot component; 304 if `If-None-Match` matches the current
seq. Body is zlib(int8 row-major cells); metadata is in `X-Map-*` headers.
Adapters publish this as a latched OccupancyGrid on `/global_map` (hardware)
or `/<ns>/global_map` (sim). **Local costmaps must not subscribe.**

### Camera video

```text
RTSP :8554/<robot_id>  (H.264, baseline, 640x480, low-latency TCP)
```

The browser consumes this stream through MediaMTX WHEP/WebRTC. There is no JPEG
network fallback; if WHEP is unavailable, the UI reports `NO SIGNAL` and retries.
Robot-side ROS `CompressedImage` data may still be JPEG internally because the
media publisher decodes it before encoding H.264.

## Rules

1. `t_mono` is the adapter's monotonic clock in seconds; the backend adds wall
   and session timestamps.
2. Reconnect with backoff and send `hello` after every connection.
3. Ignore unknown messages for forward compatibility.
4. Advertise only capabilities that are currently usable.
5. Keep `robot_id` stable across reconnects.
