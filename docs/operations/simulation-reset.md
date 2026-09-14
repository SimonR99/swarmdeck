# Epoch-safe simulation reset

Onboard Swarm-SLAM frontends cannot restart inside the same mission. Their
upstream keyframe identifiers contain a robot index and sequence number but no
process epoch, so restarting at sequence zero would collide with the old graph.
The dashboard reset therefore uses a host-side supervisor when
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
  --compose-file deploy/compose/docker-compose.mgg.yml \
  --compose-file deploy/compose/docker-compose.planning-test.yml \
  --compose-file deploy/compose/docker-compose.peers.yml \
  --compose-file deploy/compose/docker-compose.mapping.yml \
  --compose-file deploy/compose/docker-compose.onboard-planning.yml \
  --service server --service slam --service sim --service argos \
  --service mgg --service peer0 --service peer1 --service peer2 --service peer3 \
  --service mapping --service mapping-query
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
Include every process with mission-local in-memory state (`slam` as well as the
server) and every service sharing `sim`'s network namespace. Leaving a peer,
MGG, or mapping-query container attached to the stopped namespace does not make
a clean epoch.

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
