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

The Bunker base driver starts up by default alongside sensing and navigation.
Every ROS 2 robot may enable the `peer_mapping` Compose profile. It is the
documented map source: the peer Swarm-SLAM service publishes corrected poses,
the local MOLA worker builds occupancy products, and the indexed query serves
MGG. Navigation uses the robot's continuous odometry frame; corrections remain
map-authority data rather than TF edges.

## Asimov (Unitree G1)

Asimov's camera, media relay, and SwarmDeck adapter run in ROS 2 Humble
containers. Locomotion, odometry, joint state, and TF use the host-installed
Foxy `g1_ros2_bridge` on Unitree's `eth0` interface. The normal deployment
starts both layers and verifies the G1 service endpoints, `/cmd_vel` subscriber,
odometry, joint state, RTSP stream, and backend registration:

```bash
make deploy ROBOT=asimov
```

The profile brings up the Nav2 trajectory controller and local obstacle costmap
against the onboard Unitree/Livox localization's live continuous frame. MGG is
the sole planner and sends `FollowPath` trajectories; Nav2 does not run a
planner or behavior tree.
One detail remains unverified on real hardware: `asimov.launch.py`'s
`_SENSOR_YAW_IN_BASE` assumes the Mid-360's `livox_frame` already agrees with
`base_link`'s forward axis (0 rad), unlike the Bunkers' confirmed pi-yaw
mount offset. Confirm this on first bring-up by checking whether the
projected obstacle scan lines up with the visible world while walking
forward, and correct it if not before trusting autonomous navigation near
obstacles.
