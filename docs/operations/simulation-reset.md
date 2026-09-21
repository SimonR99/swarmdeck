# Epoch-safe simulation reset

There are two reset boundaries: a robot map epoch and the whole simulated
mission. A frontend restart no longer reuses old keyframe identities.

## Reset one robot's map

The dashboard's **Reset map** action calls
`POST /api/map/reset/{robot}?request_id=<canonical UUID>`. The native
four-robot simulation supervisor stops the target's active motion, restarts
only its peer frontend/bridge/MOLA source and MGG state under a fresh durable
`robot_map_epoch`, clears its costmaps, and waits for fresh mapping authority
and navigation readiness. The physical simulation and fleet mission continue.
Unrelated peers retain their own runs and captured geometry; every reference
to the target's retired run is removed, including inter-robot closures,
descriptors, graph anchors and pair-cap state.

POST returns 202 while pending and 200 for a completed replay of the same UUID.
GET on the same URL reports progress and terminal failure. A failed or
unavailable POST returns 503. Completion has a 60 s deadline; do not infer
success merely from a new epoch or an online container. Benchbot manual trials
completed in 20.4–24.1 s, including retirement from a merged four-peer graph
and reset during active Explore. The latter returned the target ready/idle
with at most 1.65 cm independent XY displacement from quiescence through ten
seconds after completion.
The UI keeps the action disabled until the request is terminal.

The run UUID is derived from mission, robot and epoch. It occupies
`KeyframeId.session_id`; the outer replica `session_id` remains the fleet
mission. Old replica uploads return 409 before requesting chunks, and moving
commands and indexed queries are fenced against retired epochs. A direct
frontend restart also claims a fresh epoch and reconciles its local planner;
it does not require a fleet reset. Home after either operation means the new
run's initial keyframe where the robot was stopped, unless a surveyed home
was configured.

The supervisor owns persistent DDS reset clients and records source ACKs.
Cancellation and costmap clearing are bounded idempotent operations;
destructive source Reset is never blindly retried after an ambiguous reply.
A durable completed ACK can be reused after restart, but an abandoned
`starting` or `failed` source operation requires a fresh request UUID and epoch.
Mixed host/container UIDs use only the narrowly shared epoch/reset directories
with non-sticky mode 0777 and 0644 lock/state files; parent directories are not
recursively chmodded. Hardware without a qualified robot-local supervisor is
explicitly unavailable rather than falling back to a rendered-map clear.

## Reset the entire simulation

The simulation reset uses a host-side supervisor when
`SWARMDECK_SIM_RESET_DIR` is configured on the server.

The server writes a bounded request to the shared reset directory and returns
immediately. The supervisor stops the configured simulation services, writes a
fresh mission UUID and advances the dedicated peer DDS domain in a generated
environment file, then recreates the services and waits for their Compose health
checks. The dashboard supplies a UUID idempotency key; if stopping the server
drops the POST response, it retries that key and receives the original active or
terminal result without starting another reset. An idle supervisor refreshes a
lease file, so a server with a reset directory but no running supervisor rejects
the button immediately instead of showing progress until a timeout. It never
deletes frontend lifetime markers or mounts the Docker socket inside the server.
`GET /api/sim/reset` reports `idle`, `accepted`, `stopping`,
`starting`, `done`, or `failed` with the request ID.
Long Compose operations refresh the status timestamp every two seconds. The API
reports an active status without a heartbeat for fifteen seconds as an explicit
supervisor-unavailable failure. A stop failure preserves the old environment.
After writing a new epoch, a failed start never rolls back to the old mission,
because some participants may already have observed the new identity.

Run the supervisor from the same composition root and with the exact Compose
files used to start the stack. Start it before Compose: it creates the shared
directory and initial environment file with host ownership, then advertises the
lease that makes the API button available. For example:

```bash
COMPOSE_PROFILES=argos python3 deploy/simulation_reset.py \
  --root sessions/simulation-reset \
  --env-file sessions/simulation-reset/deployment.env \
  --project planning-next \
  --server-url http://127.0.0.1:18080 --expected-robots 4 \
  --compose-file deploy/compose/docker-compose.yml \
  --compose-file deploy/compose/docker-compose.gpu.yml \
  --compose-file deploy/compose/docker-compose.planning-test.yml \
  --service mgg --service peer0 --service peer1 --service peer2 --service peer3 \
  --service mapping
```

Mount `sessions/simulation-reset` into the server and set
`SWARMDECK_SIM_RESET_DIR` to that container path. Keep `ui` out of the service
list so the browser remains loaded while the backend and robot graph restart.
The host supervisor must inherit `COMPOSE_PROFILES=argos` (set it with
`Environment=COMPOSE_PROFILES=argos` in a systemd unit). Otherwise Compose can
reject `stop` because active peers depend on the profile-gated `sim` service.
The isolated `planning-next` dashboard remains at **http://127.0.0.1:15173**;
do not start a second UI or Compose project for the reset.
With `--expected-robots 4`, completion waits until four reset-capable simulation adapters are online and
report that both navigation action servers and the configured MGG objective
service are ready after the server restart; a two-minute readiness timeout fails
the reset. The dashboard follows the same request across backend reconnects for
up to ten minutes. Each status request is capped at five seconds, so an
unresponsive replacement server cannot consume that whole monitoring window.
Include every process with mission-local in-memory state (`server`, `sim`,
peers, MGG and mapping services) and every service sharing `sim`'s network
namespace. Leaving a peer, MGG, or mapping container attached to the stopped
namespace does not make a clean epoch.

Connected hardware adapters cannot satisfy that count. Old mission data remains
in the map volumes for inspection; reset does not delete it. Provision storage
for retained missions or archive them separately.

The environment file is supervisor-owned for the two epoch keys, but other
deployment settings already in it are preserved. Pass that same file to the
original Compose startup, so the first reset advances from the active domain.
The supervisor also overrides inherited shell values for those two keys when it
recreates the stack; otherwise Compose gives the old exported values precedence
over `--env-file` and silently starts the old mission again.

The planning overlay deliberately uses drift odometry and moves Fast-LIVO2 to a
separate inactive profile, so it is absent from this service list. A non-test
stack that uses Fast-LIVO2 must include that service and its exact GPU or DRI
overlay in both the initial Compose invocation and the supervisor command.

Without `SWARMDECK_SIM_RESET_DIR`, the endpoint retains the legacy in-process
adapter reset for deployments that do not run onboard peer mapping.
