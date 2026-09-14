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

Override the coarse profile calibration with measured values when available:

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
make up-server       # server + UI + pose-graph SLAM back-end
# Physical fleet (uses configs/hardware_fleet.yaml, merge_mode: graph):
make up-deploy       # server + UI + SLAM + Zenoh router
```

The pose-graph process is required for merged maps. Per-robot local maps still
upload if it is down; they just will not merge. Confirm it with
`curl -fsS http://localhost:8090/health`.

Open <http://localhost:5173>. `make tunnel` is simulator-only unless an
authenticating proxy protects hardware controls.

## Pre-flight checklist

1. Confirm the physical and software e-stop path before enabling motion.
2. Verify SSH/ping, `BACKEND_HOST`, and the robot's documented `ROS_DOMAIN_ID`.
3. With `network_iface: auto` or an explicit Wi-Fi interface, confirm samples
   under **Local map → Layers → Network heatmap**. Wired robots may have none.
4. Confirm current telemetry and a live WHEP camera stream.
5. Drive about one metre manually; verify pose, local map, and free-space rays.
6. Confirm `curl -fsS http://localhost:8090/status` shows a growing keyframe
   count for that robot. Redeploy the adapter if it stays at zero -- keyframe
   production lives on the robot, not only on the server.
7. If navigation is advertised, issue a short clear-space goal and cancel it.
8. Confirm stop-all halts motion, then inspect container/ROS logs for restarts or
   missing sensor data.

## Adapter running but robot offline

Check `docker logs --tail 100 swarmdeck-botman-adapter` and backend reachability
from the robot. A running container alone does not mean ROS initialization or
adapter registration completed.

On 2026-09-08, Botman's adapter and two Nav2 processes hung opening
`/dev/shm/fastrtps_port11707`: an abandoned **zero-byte** shared-memory object.
Fast DDS 2.6.12's Boost 1.74 code waits indefinitely for such a file to acquire
its initial size, before Fast DDS can run its port health check. A native stack
showed `SharedMemTransport::CreateInputChannelResource`; the blocked process
held that empty file open. Shared memory worked in another ROS domain, and
quarantining that specific empty object restored node creation in domain 17.
A subsequent 10-second read-only SHM probe received 100 Ouster clouds and
89 odometry/registered-scan messages, with clouds up to 3.1 MB.
This was not a mismatch with MGG's UDP transport. The original creator's
interruption was not captured, so the event that abandoned the file is unknown.

Nav2's `timeout 5 ros2 lifecycle get ...` probes amplified the fault: ROS signal
handlers could not finish shutdown inside the blocked native call, and timeout
had no forced-kill deadline. We found and terminated 252 leaked read-only probes.
Their DDS participants also contributed to several GiB of SHM allocations.

ROS CLI health checks now request graceful SIGINT shutdown after 4 seconds and
force termination one second later. On the Bunkers, these disposable probes use
`deploy/dds/fastdds_udp_only.xml` so a killed probe cannot abandon a host SHM
port. The long-lived Botman adapter, lidar, SLAM, and Nav2 nodes use
`fastdds_large_data.xml` for efficient local cloud transport. MGG remains on UDP
in its private IPC namespace.

On 2026-09-09, Aslan had the same initialization hang on
`/dev/shm/fastrtps_port19669`, but the file was **52,416 bytes with an all-zero
initialization header**, not zero-length. Its adapter and eight Nav2 processes
held this file open, and the adapter's native stack was sleeping under
`SharedMemTransport::CreateInputChannelResource`. The base driver, camera
launcher, odometry TF, and six SLAM processes also held the same file. Other
ports started with `02 00 00 00`; this port's first 64 bytes were zero.
Quarantining only this diagnosed port and restarting those six affected
containers restored adapter registration and activated Nav2. SLAM's
process-presence health check had reported healthy during the hang, so verify
actual telemetry as well. Zero-byte `_el` lock files are normal and are not,
by themselves, evidence of this fault. The event that left the port
uninitialized was not captured.

Do not blanket-delete `/dev/shm/fastrtps_*` while ROS is running. Diagnose the
specific object and its users first; preserve a verified old, empty port outside
the DDS naming scheme when recovering it. Processes already waiting on its old
file descriptor need restarting. Changing ROS domain is a diagnostic only;
robot services must remain on their configured domain to communicate.

Aslan now has a once-per-boot cleanup guard installed for its next boot. The
[systemd installer](../../deploy/robots/systemd/README.md#fast-dds-shared-memory-cleanup-at-boot)
orders cleanup before Docker/containerd and skips the installation boot. It
refuses to remove files while container runtimes or DDS users are active. This
prevents stale files present at startup from blocking DDS; it does not clean
files left by a crash after ROS has started.

### LiDAR return range

Botman, Aslan, and TARS are configured for a **30 m maximum return range**.
Botman and Aslan set `max_range` in their repo-owned Ouster driver YAML files;
TARS overrides `/os_cloud_node/os_cloud_node/max_range` after its vendor Ouster
launch include. Its installed Ouster 0.5.2 predates this parameter, so
`scripts/scout-build-lidar` backports filtering into the cloud nodelet before
normal Scout deployment. It preserves complete point records and publishes a
dense cloud: LVI-SAM rejects the NaN-filled organized clouds used by newer
drivers. The SDK's per-point range is in millimetres; the ROS setting is metres.
The original source and library are backed up beside the patched files.
This is distance from the sensor at capture time, not distance from the map
origin. Navigation can retain its shorter obstacle/raytrace horizon.

Apply these startup parameters by restarting the corresponding LiDAR driver.
On TARS, restart its coupled sensor/SLAM launch after building the backport.
Check the published cloud's range afterward. Existing spurious
returns already accumulated in a SLAM/server map are not removed retroactively.
