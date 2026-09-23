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

## 2026-09-23T12:04 - tuf, SubT, 4 robots, drift, RTX 4070, idle

- **Real-time factor 1.00** (29.8 s simulated in 29.8 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| argos | 56 |
| sim | 32 |
| peer3 | 11 |
| peer2 | 10 |
| peer0 | 10 |
| peer1 | 10 |
| mgg | 5 |
| server | 1 |
| mediamtx | 0 |
| mapping | 0 |
| ui | 0 |
| **total** | **136** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 56 |
| swarmdeck_argos_bridge.py | sim | 15 |
| python3 /r3 | peer3 | 7 |
| python3 /r2 | peer2 | 7 |
| python3 /r1 | peer1 | 6 |
| python3 /r0 | peer0 | 6 |
| adapter_sim.py | sim | 4 |
| lidar_handler_node.py /r3 | peer3 | 2 |
| lidar_handler_node.py /r0 | peer0 | 2 |
| lidar_handler_node.py /r2 | peer2 | 2 |
| lidar_handler_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r3 | peer3 | 2 |

- MGG plan cycles: none in the window (fleet not exploring?)

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 20.0 | 49.3 |
| fleet_change | 0.1 | 1.0 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

## 2026-09-23T12:05 - tuf, SubT, exploring (4 robots), RTX 4070

- Explore started by the harness
- **Real-time factor 1.00** (119.8 s simulated in 119.8 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| mapping | 195 |
| peer0 | 112 |
| mgg | 94 |
| peer2 | 88 |
| peer1 | 84 |
| server | 79 |
| sim | 79 |
| argos | 72 |
| peer3 | 65 |
| mediamtx | 0 |
| ui | 0 |
| **total** | **869** |

| Process | Container | CPU % |
|---|---|---:|
| python | server | 79 |
| argos3 | argos | 72 |
| swarmdeck-mola-import | mapping | 64 |
| loop_closure_detection_node.py /r0 | peer0 | 61 |
| loop_closure_detection_node.py /r2 | peer2 | 60 |
| loop_closure_detection_node.py /r1 | peer1 | 58 |
| swarmdeck-mola-import | mapping | 54 |
| swarmdeck-mola-import | mapping | 52 |
| loop_closure_detection_node.py /r3 | peer3 | 44 |
| swarmdeck_argos_bridge.py | sim | 31 |
| lidar_handler_node.py /r0 | peer0 | 30 |
| mggplanner_node /robot_1/mgg | mgg | 28 |

- **MGG plan cycles: 32**, wall ms median 692 (max 1040); lattice median 492, gain median 142; median 1499 vertices, 25739 edges

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 18.1 | 185.1 |
| fleet_change | 0.1 | 5.0 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

### perf: MGG planner while exploring (tuf, 40 s, robot with the busiest planner)

Self time (the release build has no frame pointers, so no callers):

| Share | Symbol |
|---:|---|
| 62-65 % | `std::binary_search` over `NativeMolaGrid::Cell` (`NativeMolaGrid::status`, `native_mola_grid.cpp:77-83`: every voxel query binary-searches the sorted occupied and free cell vectors) |
| 7-9 % | `NativeMolaGrid::walk` (ray status) |
| 2-4 % | `std::map<Cell, double>` lookups (`surface_max_z_`) |
| ~5 % | Fast DDS UDP listen / executor wait |

### Reading

- On tuf the simulation runs at real time (1.00, paced; it could go faster) and the idle stack uses 136 % CPU against 351 % on the workstation.
- MGG plan cycles on tuf: median 0.69 s, max 1.04 s, for the same graph size that took 3.2-10.9 s on the workstation. Most of the workstation's slowness was host contention (desktop, browser, iGPU simulation), not the planner.
- Within MGG, about two thirds of the planning CPU is voxel lookup by binary search. A hashed or blocked cell index would cut the lattice build several-fold without changing behaviour; that matters most on the robots' onboard CPUs.
