# DDS transport profiles

The ARGoS Compose stack uses `fastdds_large_data.xml` for **sim, peer0–3 and MGG**.
It explicitly enables 16 MiB shared-memory segments and UDPv4, without adding
Fast DDS's smaller built-in SHM transport. UDP remains available for discovery
and participants on other hosts. The planning-test overlay inherits this profile.
`fast_livo2` stays in its separate domain with `/tools/fastdds_large.xml`;
ARGoS itself does not use ROS/DDS. Hardware Compose profiles are unchanged.

Sim owns a private, shareable 2 GiB `/dev/shm`; peers and MGG join it with
`ipc: service:sim`, just as they join its network namespace. They depend on sim
being started. Profile XML is bind-mounted read-only, so image rebuilds are not
needed to change transport settings. All participating containers must have
compatible permissions on the shared files (the shipped images run as root).

Fleet reset stops and force-recreates the complete fleet through Compose;
Compose dependencies start sim before its consumers. **Do not recreate sim
alone:** existing peers/MGG would retain the old network/IPC namespaces.
Recreate its namespace consumers together with sim. An ordinary process restart
within a container does not replace its namespaces. Avoid manually deleting
SHM files while any participant is running.

Keep `fastdds_udp_only.xml` for short-lived health probes and deployments that
cannot share `/dev/shm`. Override `FASTRTPS_DEFAULT_PROFILES_FILE` with its mounted
path for such probes; `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` alone does not override
an XML profile with `useBuiltinTransports=false`.

## Per-robot reset and forced planner stops

Unlike a fleet reset, a per-robot reset keeps sim's shared `/dev/shm` alive.
The peer stop has a five-second grace period, and MGG's planner supervisor can
SIGKILL its process group after ten seconds. Either forced stop can leave
Fast DDS segments/ports behind. The short-lived `robot_reset.py` quiesce and
readiness probes therefore explicitly use the UDP-only XML.

The host reset supervisor runs `fastdds shm clean` in sim after each acknowledged
MGG stop (the acknowledgement does not say whether SIGKILL was needed) and after
each peer stop. Each attempt is bounded to three seconds, within the reset's
remaining deadline, including an in-container timeout. Failures are ignored;
results are logged at most once per 30 seconds per outcome. Fast DDS 2.14.6's
cleaner checks file locks (`flock`), retaining live owners even in another PID
namespace; it does not guess ownership from process IDs or delete all SHM files.

MGG's supervisor allows ten seconds for ROS launch to stop its children,
including launch's SIGTERM escalation after five seconds. A normal stop is
unaffected: the wait returns as soon as launch exits. Only a planner that ignores
SIGTERM now waits up to ten seconds before SIGKILL; that delay adds to a
per-robot reset. The host's bounded cleanup above covers forced kills on the
reset path. Stops outside that path gain the longer grace period but do not run
SHM cleanup, so an unresponsive planner can still leave stale objects there.
Do not replace lock-aware cleanup with `rm /dev/shm/fastrtps*`.

Live leak check: with the same fleet running, record `du -sh /dev/shm` and
`find /dev/shm -maxdepth 1 -name 'fastrtps_*' | wc -l` inside sim. Run **N=10**
per-robot resets serially (wait for readiness after each, without recreating
sim), then repeat both measurements. Check after each reset too: usage/file
counts should return to a bounded steady level, not grow with reset count.
Repeat while exercising a forced planner stop, and inspect cleanup warnings.
Check cloud/image delivery and other robots throughout, to detect accidental
removal of a live participant's resources.

## Simulation static mounts

`session.launch.py` starts one `sensor_mounts` process for the whole fleet,
replacing four `static_transform_publisher` processes per robot. It publishes
one reliable, transient-local TFMessage per `/<robot>/tf_static`, containing
all four existing mount edges (lidar, IMU, proximity lidar, camera), then stays
alive to serve late subscribers. Mount geometry and frame IDs are unchanged;
Nav2 and peer consumers keep their namespaced TF subscriptions. The old
`/<robot>/{lidar,imu,proximity_lidar,camera}_tf` diagnostic node names disappear.

## Nav2 composition

Simulation passes `use_composition:=true` to `nav.launch.py`: each robot gets
one `nav_container` running the controller and velocity smoother, with isolated
executors. The complete rewritten YAML is also passed to the container so the
controller's internally constructed local costmap inherits its parameters.
Lifecycle ownership, per-robot node names, action endpoints, TF remaps, and the
`cmd_vel_nav` → smoother → `cmd_vel` chain remain unchanged. The bounded
simulation startup process still exits after activation. Hardware callers keep
`use_composition:=false` by default and retain their separate processes.

With four robots, static batching removes 15 processes/participants and Nav2
composition removes another four processes/participants. Actual discovery
counts include adapters, media, launch loaders and transient diagnostics; count
DDS GUID prefixes, not ROS node names, when comparing the fleet.

## Isolated transport regression

Run `python -m pytest -m docker -s tests/deployment/test_dds_transport_docker.py`
with the prebuilt `swarmdeck-sim:local` image (or set `SWARMDECK_DDS_TEST_IMAGE`).
The test compares reliable 1 MiB image delivery over UDP-only and SHM+UDP in
throwaway containers with a private network/IPC namespace, checking actual
loopback bytes and segment sizes. It skips if Docker/the image is unavailable
and is outside the default pytest testpaths. It never joins the running fleet.

## Validate on a running fleet

Compare an identical domain, fleet size and scenario before/after. Inspect
`docker compose config` for the effective IPC and XML settings. In sim, peers
and MGG, check `ls -l /dev/shm/fastrtps*`: large segments (at least 16 MiB plus
metadata), not only 549408-byte default segments, must be visible from all
containers. Size/visibility alone is not proof of traffic: confirm reduced UDP
loopback bytes and unchanged point-cloud/image delivery under the same load.
Measure CPU with `docker stats` and graph integrity with `ros2 node list`,
`ros2 topic info -v`, `ros2 topic hz`, and late-joining `/tf_static` subscribers.
Use the UDP-only XML for diagnostic ROS CLI processes to avoid leaked SHM
objects if a probe has to be killed. A daemon caches its environment/domain;
use `--no-daemon` where supported, or restart the diagnostic daemon explicitly.
