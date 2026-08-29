# Robot deployment profiles

`scripts/deploy` loads shared values from `deploy/fleet.env`, then one profile
from this directory. Profiles contain SSH, checkout, Compose, workspace, sensor,
and calibration settings. They are trusted Bash fragments; operator environment
values take precedence.

Keep secrets and improved per-run calibration in the operator environment, not
tracked profiles. Generated overrides are written remotely under `.deploy/`
with mode 0600.

From the operator workstation:

```bash
make deploy ROBOT=botman
make deploy ROBOT=all

BACKEND_HOST=192.168.1.10 \
BOTMAN_OAK_X=0.42 BOTMAN_OAK_Y=0.00 BOTMAN_OAK_Z=0.80 \
BOTMAN_OAK_ROLL=0 BOTMAN_OAK_PITCH=0 BOTMAN_OAK_YAW=0 \
make deploy ROBOT=botman
```

The pipeline is SSH preflight, source sync, remote override, optional preparation,
build, Compose reset/start, and bounded readiness verification. Required
containers must be running (and healthy when they define a healthcheck), keep
the expected source mount, and have the profile's `DEPLOY_ROBOT_ID` registered
and reporting live state at `BACKEND_HOST:BACKEND_PORT/api/fleet`. Scout's native
helper additionally verifies its ROS data before checking backend liveness.
`--dry-run` previews without writes; `--no-build`, `--no-reset`, `--no-up`, and
`--no-verify` skip stages.

Aslan's base driver is safety-gated behind a Compose profile:
`DEPLOY_COMPOSE_PROFILES=base make deploy ROBOT=aslan`.

Adding a Compose robot requires `deploy/robots/<name>.env`, a robot Compose file,
and adapter/sensor configuration. `ROBOT=all` discovers the profile automatically.
Deployment reset means `docker compose down --remove-orphans`; it does not move
hardware or clear robot-side SLAM.

## Asimov (Unitree G1)

Asimov's camera, media relay, and SwarmDeck adapter run in ROS 2 Humble
containers. Locomotion, odometry, joint state, and TF use the host-installed
Foxy `g1_ros2_bridge` on Unitree's `eth0` interface. The normal deployment
starts both layers and verifies the G1 service endpoints, `/cmd_vel` subscriber,
odometry, joint state, RTSP stream, and backend registration:

```bash
make deploy ROBOT=asimov
```

The profile intentionally advertises teleoperation only. Asimov has no verified
Nav2 action server or battery adapter yet, so those capabilities stay disabled
until their native producers are installed and tested.
