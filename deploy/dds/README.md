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
