# DDS transport profiles

The ARGoS Compose stack uses `fastdds_large_data.xml` for **sim and peer0–3**.
It explicitly enables 16 MiB shared-memory segments and UDPv4, without adding
Fast DDS's smaller built-in SHM transport. UDP remains available for discovery
and participants on other hosts. The planning-test overlay inherits this profile.
`fast_livo2` stays in its separate domain with `/tools/fastdds_large.xml`;
ARGoS itself does not use ROS/DDS. Hardware Compose profiles are unchanged.

Sim owns a private, shareable 2 GiB `/dev/shm`; peers join it with
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
