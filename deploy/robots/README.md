# Robot deployment profiles

`scripts/deploy` loads one profile from this directory. A profile contains the
SSH target, the checkout path on the robot, the Compose file, defaults for that
robot, its robot-specific workspace/calibration values, and the environment
keys that should be written as its deployment override. Adding a Compose-based
robot normally means adding one `.env` profile and one
`docker-compose.robot-*.yml`; the deployment script discovers the new profile
automatically for `ROBOT=all`.

Profiles are trusted Bash fragments (the `:=` defaults preserve operator
overrides); they are not the generated Compose override files. Keep secrets and
per-run calibration overrides in the operator environment, not in the profile.

From the operator workstation:

```bash
make deploy ROBOT=botman
make deploy ROBOT=all

# Override profile defaults without editing the profile:
BACKEND_HOST=192.168.1.10 \
BOTMAN_OAK_X=0.42 BOTMAN_OAK_Y=0.00 BOTMAN_OAK_Z=0.80 \
BOTMAN_OAK_ROLL=0 BOTMAN_OAK_PITCH=0 BOTMAN_OAK_YAW=0 \
make deploy ROBOT=botman
```

The default pipeline is: SSH preflight, full source sync, remote override
generation, optional robot-specific preparation, image build, Compose stack
reset, stack recreation, and a container check. `--dry-run` performs no writes;
`--no-reset` keeps the existing stack running while it is recreated.

Profiles may expose optional Compose profiles without adding deployment logic:
for example, Aslan's physical base driver stays opt-in until the e-stop is
ready: `DEPLOY_COMPOSE_PROFILES=base make deploy ROBOT=aslan`.

“Reset” here means `docker compose down --remove-orphans`; it does not move a
physical robot or erase its SLAM state. The real hardware adapters intentionally
do not advertise the simulator-only reset capability.
