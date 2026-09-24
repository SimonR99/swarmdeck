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
`idle`, `active`, `succeeded`, `failed`, or `cancelled`. In both simulation
and ROS 2 hardware, `active` includes a pending replacement route: it indicates
an owned navigation objective, not necessarily controller acceptance or motion.

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
```

MGG goals are navigation-frame goals. Objective planning resolves and validates
mapping authority, then sends a trajectory to `FollowPath`; vendor adapters
may pass a navigation-frame goal to their native controller. `stop`, manual
drive, and a replacement objective cancel active route execution.

### Simulation velocity ownership

The simulation session remaps Nav2's smoothed output to
`/<robot_id>/cmd_vel_adapter`. The simulation adapter uses the same velocity
gate as ROS 2 hardware and is the sole publisher to `/<robot_id>/cmd_vel`:
only an accepted, current route with an active status and fresh operator link
can relay Nav2 output. Both adapters run the shared 20 Hz ROS watchdog separately
from websocket telemetry: a stale link cancels active navigation, publishes
a stop and reports `cancelled` rather than merely holding the velocity relay
closed. The same watchdog
consumes pending manual-drive commands and enforces their deadman timeout.
Pending routes, cancellation, stop, and manual drive
close the relay; manual drive and recovery still publish directly through the
adapter. On sim and hardware, a terminal result that closes an open relay also
publishes one zero velocity, so the driver cannot retain the last moving
command while the smoother's delayed stop is blocked. An already closed relay
does not publish another zero. This adds one DDS hop after the smoother, not
another smoothing stage.
Bring up the updated session launch and adapter together; a legacy Nav2 launch
publishing directly to `cmd_vel` bypasses this gate. The generic Nav2 launch
and hardware launch defaults are unchanged.

As on hardware, a current objective may update its navigation status and plan a
replacement immediately after cancellation; it does not wait for the cancelled
action's terminal result. Generation checks still reject superseded commands.
Simulation retains its separate conservative recovery rule: loss of action
monitoring suppresses automatic reverse escape, but does not prevent a new
owned route from being planned.

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
only when fresh depth and TF make the position valid. Simulation reset is not
an adapter command: the host reset supervisor restarts the simulation in a
fresh mission (docs/operations/simulation-reset.md). Hardware adapters do not
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
| `reset` | Restartable by the simulation reset supervisor |
| `explore` | Start/stop configured exploration |

Never advertise a capability the adapter cannot honour.
