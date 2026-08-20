# Hardware Bring-up & Fleet Deployment

This guide outlines the standard procedure for deploying SwarmDeck adapters and starting real-world robot sessions.

---

## 1. Fleet Deployment from the Operator Console

The operator workstation owns the full robot lifecycle. Profiles in
`deploy/robots/` contain the SSH target, remote checkout path, Compose file,
defaults, and required calibration overrides. The standard command performs
source sync, override generation, image build, a safe Compose stack reset,
recreation, and a source-mount/container check:

```bash
# Deploy one robot
make deploy ROBOT=botman

# Deploy every profile, continuing past an unreachable robot
make deploy ROBOT=all

# Preview transfer and overrides without changing a robot
make deploy ROBOT=botman DEPLOY_ARGS=--dry-run
```

Operator overrides stay out of the Compose files:

```bash
BACKEND_HOST=192.168.1.10 \
BOTMAN_OAK_X=0.42 BOTMAN_OAK_Y=0.00 BOTMAN_OAK_Z=0.80 \
BOTMAN_OAK_ROLL=0 BOTMAN_OAK_PITCH=0 BOTMAN_OAK_YAW=0 \
make deploy ROBOT=botman
```

The lower-level equivalent is `./scripts/deploy botman`. `--no-build` is the
fast path when only mounted source changed; `--no-reset` keeps the current
Compose stack running while it is recreated. “Reset” means stopping/removing
Compose containers only. It does not teleport a physical robot or reset SLAM;
that capability is simulator-only.

---

## 2. Operator Station Startup

On the operator host computer:

```bash
# 1. Start Server + UI
make up-server
# Or manually:
# cd server && .venv/bin/python -m swarmdeck_server --config ../configs/hardware_botman.yaml
# cd ui && npm run dev

# 2. (Optional) Publish dashboard via secure tunnel
./scripts/tunnel.sh
```

---

## 3. Pre-Flight Checklist

Before starting autonomous or teleoperated runs:
1. **Physical E-Stop**: Confirm wireless e-stop is armed and functioning.
2. **Network Connectivity**: Verify ping to robot IP and check `ROS_DOMAIN_ID` separation.
3. **Camera Stream**: Confirm WHEP WebRTC stream shows live FPS and latency HUD on the dashboard.
4. **Map Accumulation**: Drive the robot 1 meter and confirm raytraced free space appears on the 2D grid.
