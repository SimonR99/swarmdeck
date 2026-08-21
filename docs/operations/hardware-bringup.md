# Hardware bring-up

## Deploy from the operator workstation

Set `BACKEND_HOST` in `deploy/fleet.env` to the workstation running SwarmDeck.
Robot profiles in `deploy/robots/` hold SSH, workspace, sensor, and calibration
values. Deployment performs SSH preflight, sync, configuration, build, and
Compose reset/start. Readiness verification waits for required containers to
run (and report healthy when they define healthchecks), confirms the source
mount, and queries the operator backend until the profile's adapter is
registered and reporting live state. Scout also checks live ROS data first. The
manual checklist below is still required.

```bash
make deploy ROBOT=botman
make deploy ROBOT=all
make deploy ROBOT=botman DEPLOY_ARGS=--dry-run
```

Supply per-run calibration through the environment:

```bash
BOTMAN_OAK_X=0.42 BOTMAN_OAK_Y=0.00 BOTMAN_OAK_Z=0.80 \
BOTMAN_OAK_ROLL=0 BOTMAN_OAK_PITCH=0 BOTMAN_OAK_YAW=0 \
make deploy ROBOT=botman
```

Use `./scripts/deploy botman` directly when needed. `--no-build` reuses images;
`--no-reset` avoids `compose down`; `--no-native-reset` preserves Scout's native
ROS launchers; `--no-up` stops after preparation. Deployment reset affects
containers and the known Scout launchers only; it never moves the robot or clears
robot-side SLAM state.

## Start operator services

```bash
make up-server       # server + UI
# or, when a Zenoh router is required:
make up-deploy       # server + UI + router
```

Open <http://localhost:5173>. `make tunnel` is simulator-only unless an
authenticating proxy protects hardware controls.

## Pre-flight checklist

1. Confirm the physical and software e-stop path before enabling motion.
2. Verify SSH/ping, `BACKEND_HOST`, and the robot's documented `ROS_DOMAIN_ID`.
3. With `network_iface: auto` or an explicit Wi-Fi interface, confirm samples
   under **Local map → Layers → Network heatmap**. Wired robots may have none.
4. Confirm current telemetry and a live WHEP camera stream.
5. Drive about one metre manually; verify pose, local map, and free-space rays.
6. If navigation is advertised, issue a short clear-space goal and cancel it.
7. Confirm stop-all halts motion, then inspect container/ROS logs for restarts or
   missing sensor data.
