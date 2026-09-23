# Server boundaries

`swarmdeck_server.api.app:create_app` wires the shared registry and registers
HTTP, GUI WebSocket and adapter WebSocket routers. `api.state` owns the shared
session, alert, detection and review state and its operations. The factory does
not create independent fleet instances: this remains one fleet per process.

## Removed legacy routes

- `/api/agent/*` belongs to Cortex, not this server. The production nginx and
  development Vite proxies route those requests to the agent service on port
  8085. Direct requests to the fleet server now return 404.
