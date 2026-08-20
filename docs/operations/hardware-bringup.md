# Hardware Bring-up & Fleet Deployment

This guide outlines the standard procedure for deploying SwarmDeck adapters and starting real-world robot sessions.

---

## 1. Fleet Deployment via `scripts/deploy`

All robots bind-mount the checkout read-only on their onboard storage. Code updates are deployed directly via `scripts/deploy`:

```bash
# Deploy to a specific robot (rsync source + restart containers)
./scripts/deploy aslan
./scripts/deploy botman
./scripts/deploy spot

# Deploy to all reachable robots
./scripts/deploy all

# Dry-run deployment preview
./scripts/deploy botman --dry-run
```

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
