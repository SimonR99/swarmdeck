# Performance measurements

Dated reports from `scripts/profile_stack.py`, newest last. Each clean-up or
optimization change quotes the report before and after it (see
docs/superpowers/plans/2026-09-23-cleanup-optimization.md). Measure on the
same host, scenario and fleet state to compare.

Hosts:

- **workstation**: 20 cores, Intel Iris Xe for ARGoS (the NVIDIA GPU is in an
  external Thunderbolt box, usually detached), also runs the operator's
  desktop, so its load varies.
- **tuf** (`ssh tuf`): 20 cores, RTX 4070, workspace `~/swarmdeck-ws`
  (`swarmdeck/` plus the SubT assets under `argos3/`), logs in
  `~/swarmdeck-ws/logs/`. Used for GPU and repeatable measurements.

MGG's C++ is profiled with `perf`, which needs the performance-monitoring
capability (`perf_event_paranoid` is 4 on both hosts): run it from a sidecar
container given `--cap-add PERFMON` in the MGG container's PID namespace.

## 2026-09-23 - workstation, SubT, 4 robots, drift odometry, DRI (iGPU), mostly idle

Baseline for the plan, from manual runs before the harness existed and the
harness's first run.

- Real-time factor: 0.17 (under host load average 12-27 with the dashboard
  open), 0.62 later (lighter load). It varies with host load.
- MGG plan cycles (23, during an explore-to-goal test): 3.2-10.9 s wall;
  lattice build 2.2-8.2 s, gain 0.7-2.0 s; ~1,400 vertices, ~25k edges.
- Browser tab (Firefox, 3D view): ~170 % CPU at idle.

| Container | CPU % (idle) |
|---|---:|
| argos | 124 |
| sim | 90 |
| peer0..3 | 28-30 each |
| mgg | 12 |
| server | 6-34 |

| Process | CPU % |
|---|---:|
| argos3 | 124 |
| swarmdeck_argos_bridge.py | 40 |
| cslam_bridge.py (each peer) | 18-20 |
| adapter_sim.py | 15-21 |
| lidar_handler_node.py (each) | 5-11 |
| loop_closure_detection_node.py (each) | 4-8; 66-100 on a peer after a long drive |

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 18.9 | 64.8 |
| others | 1.4 | 1.6 |

py-spy (share of active samples):

- bridge: 21 % rclpy publish, ~20 % nav-scan projections, ~20 % cloud packing + SHA-256.
- cslam_bridge: 34 % rebuilding and JSON-encoding the mapping snapshot.
- lidar_handler: 30 % point-cloud downsampling.
- loop_closure (peer after a long drive): 75 % linear ScanContext search.
- server: ~30 % deployment raster refresh, ~18 % JSON encode/decode.
