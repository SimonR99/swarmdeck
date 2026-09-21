# SwarmDeck adapter protocol

This is the robot/backend interface. Protocol version 2 adds the optional
`slam_graph` message.

## Architecture

Adapters report continuous odometry-frame pose and operational telemetry over
`WS /adapter`. Swarm-SLAM owns collaborative corrected poses and replica
geometry; MOLA products provide occupancy to MGG. MGG routes are delivered to
the controller as `FollowPath` trajectories. Adapters do not upload grids,
scans, clouds, or keyframes.

The 2D dashboard map is rasterized from replica chunks by the server. The 3D
dashboard consumes the same replica products. Video uses RTSP.

## Transport

| Direction | Endpoint | Data |
|---|---|---|
| bidirectional | `WS /adapter` | Registration, telemetry, detections, commands |
| adapter -> backend | `POST /api/autonomy/chunks/<sha256>` | Replica geometry chunk |
| adapter -> backend | `POST /api/autonomy/replicas` | Replica manifest |
| adapter -> MediaMTX | `RTSP :8554/<robot_id>` | Video |


## Registration

The first WebSocket message is `hello`; reconnects send it again. The backend
replies with `hello_ack` or closes the socket.

```jsonc
{
  "type": "hello",
  "protocol": 2,
  "robot_id": "robot_0",
  "robot_type": "diffdrive",
  "adapter": "adapter_ros2/0.1.0",
  "ros": "jazzy",
  "coordinate_frame": "local",
  "capabilities": ["plan_objective", "camera", "battery", "network", "estop"],
  "footprint_radius": 0.35,
  "footprint": [[0.5, 0.3], [0.5, -0.3], [-0.5, -0.3], [-0.5, 0.3]]
}
```

`coordinate_frame` is `local`: pose and goals use the robot navigation frame,
which is its continuous odometry frame. `footprint` is optional and uses x
forward, y left.

## Robot state

Adapters send a complete `robot_state` at 5 Hz:

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
  "planned_path": [{"x": 1.2, "y": -3.4}, {"x": 2.1, "y": -2.7}]
}
```

`planned_path` is the effective bounded route. `global_planned_path` and
`local_planned_path` may be included when available. `nav_status` is one of
`idle`, `active`, `succeeded`, `failed`, or `cancelled`.

## Commands

```jsonc
{ "type": "plan_objective", "objective": "navigate", "goal": {"x": 4.0, "y": 1.5, "yaw": 0.0} }
{ "type": "plan_objective", "objective": "return_home" }
{ "type": "cancel_goal" }
{ "type": "drive", "linear": 0.28, "angular": 0.0 }
{ "type": "stop" }
{ "type": "set_mode", "mode": "teleop" }
{ "type": "body_command", "action": "stand", "height": 0.0 }
{ "type": "explore", "enabled": true }
{ "type": "reset" }
```

MGG goals are navigation-frame goals. Objective planning resolves and validates
mapping authority, then sends a trajectory to `FollowPath`; vendor adapters
may pass a navigation-frame goal to their native controller. `stop`, manual
drive, reset, and a replacement objective cancel active route execution.

## Collaborative graph

Protocol 2 adapters may report graph health:

```jsonc
{
  "type": "slam_graph",
  "robot_id": "robot_0",
  "keyframes": 82,
  "in_common_frame": true,
  "residual": 0.04,
  "inter_robot": ["robot_1"]
}
```

## Detections and reset

A `detections` message contains normalized image boxes and optional positions
only when fresh depth and TF make the position valid. After simulation reset,
adapters send `reset_done` with per-step results. Hardware adapters do not
advertise reset.

## Capabilities

| Capability | Contract |
|---|---|
| `plan_objective` | Accept MGG objective commands |
| `camera` | Publish video under `robot_id` |
| `battery` | Include `battery` in state |
| `network` | Include synchronized link quality |
| `estop` | Accept `stop` |
| `body` | Accept `body_command` |
| `reset` | Accept simulation reset |
| `explore` | Start/stop configured exploration |

Never advertise a capability the adapter cannot honour.
