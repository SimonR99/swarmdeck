# Server boundaries

`swarmdeck_server.api.app:create_app` wires the shared registry and registers
HTTP, GUI WebSocket and adapter WebSocket routers. `api.state` owns the shared
session, alert, detection and review state and its operations. The factory does
not create independent fleet instances: this remains one fleet per process.

## Removed legacy routes

- `/api/agent/*` belongs to Cortex, not this server. The production nginx and
  development Vite proxies route those requests to the agent service on port
  8085. Direct requests to the fleet server now return 404.
- `POST /api/adapter/camera` and `GET /api/camera/{robot_id}` are removed:
  adapters use the media pipeline, not server-side JPEG uploads. Camera stream
  loss is no longer inferred from that unused cache. `/api/robot/{id}/vision`
  retains `camera_streaming: false`, `frame_age_ms: null`, and `frame_seq: null`
  for CLI consumers; detections and robot metadata are unchanged. The robot
  tool's `snap` and `snapshot` commands print media-pipeline guidance and exit 1
  without a traceback. Use `doctor` or the dashboard for RTSP/WHEP camera access.
- Adapter `camera_interest` messages are no longer emitted. GUI
  `switch_camera` messages remain accepted and logged for existing dashboards,
  but camera selection has no server-to-adapter side effect.
- `POST /api/fleet/{robot_id}/discard` is removed. Use the existing
  `DELETE /api/fleet/{robot_id}` route (as the dashboard already does).

## Mission boundary

The server still starts without `SWARMDECK_MISSION_ID` for `make server`,
`make mock`, and `make demo`. Missionless mock fleets retain telemetry, drive,
body commands, goals, exploration, and the legacy adapter reset handshake.
Mock adapters' per-robot replica session IDs do not configure a server mission.

Live mapping authority (`live_mapping` and `peer_slam`) is admitted only with
an active server mission. Unfenced `home_pose` telemetry no longer establishes
mapping home; epoch-qualified live mapping supplies it. Without a mission,
`GET/POST /api/map/reset/{robot_id}`, `POST /api/map/reset`, and live-component
GET/goal operations return HTTP 409, `no active mission`. All-map reset remains
unsupported for a live peer mission; use the full mission reset instead.
Historical replica browsing and safe raster retirement when a mission ends
remain available. Replica publication and generic mock controls are unchanged.
