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
  `~/swarmdeck-ws/logs/`. Used for GPU and repeatable measurements. Synced
  from the workstation with `git push tuf planning-refactor`.
- **botman** (`ssh -J benchbot botman@192.168.1.49`, only through benchbot): Jetson AGX Orin, 12
  Cortex-A78AE cores at 2.2 GHz, `MODE_50W`. It runs the `main` deployment and
  its experiments, so never change it. For benchmarks, pause it with
  `docker stop` and resume with `docker start` on the same containers.

MGG's C++ is profiled with `perf` (`scripts/profile_stack.py --perf-mgg`)
from a sidecar container in the MGG container's PID namespace. Because
`perf_event_paranoid` is 4 on both hosts, Ubuntu refuses `PERFMON` alone, so
the sidecar gets `SYS_ADMIN` and `SYS_PTRACE` with no seccomp filter. No sudo
is needed.

Each report header now includes the short git commit (`--commit SHA` to
override; default is auto-detected from the script's own repo). Use this to
correlate each dated entry with the code it measured.

`--browser [--browser-url URL] [--browser-idle-s N]` measures the launched
Chromium process tree's CPU from Linux `/proc` (100 % = one core), including
GPU/compositor descendants, and WebGL frames per second. Opens the dashboard
at URL (default `http://localhost:5173`) for an idle window and synthetic
mouse-drag panning. A WebGL frame is an animation-frame interval containing
clear/draw work; Canvas2D work is not counted as WebGL. The browser report
records `nproc` alongside the one-core CPU units. Endpoint process sampling
can miss short-lived processes. Linux only; requires `node` ≥ 18,
Playwright and its Chromium binary. The probe forces SwiftShader software
rendering, which inflates CPU per frame and cannot establish a real-GPU idle
CPU target. Prints an unavailable result if the optional probe fails.

`--latency` measures **plan-log-to-displacement (proxy)** and replan cadence.
It collects `robot_state` WebSocket events and resets from the server container
and MGG `docker logs --timestamps`, then reports median/p90/max time from a
plan log to navigation-frame displacement >= 0.10 m. Each plan's window ends
at the next plan for that robot or a reset; registration changes beyond 1 mm
or 1 mrad also cut it off. Reports include the count of plans cut off without
a displacement sample. This is not dispatch latency: reservations can delay
or reject a logged plan. Clock: host UTC wall clock; typical error < 1 ms.
Telemetry cadence and the displacement threshold also affect this proxy.

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

## 2026-09-23T12:40 - host comparison: botman (Jetson AGX Orin) vs tuf vs workstation

`scripts/bench/host_bench.cpp` and `scripts/bench/gpu_bench.cu`, built on each
host with `-O2` (g++ 11.4 on botman, 9.5 on tuf, 15.2 on the workstation;
CUDA 12.6 and 12.4). `voxel_status` reproduces `NativeMolaGrid::status`: two
`std::binary_search` over sorted 24-byte cells, with ray-like coherent queries.
It also times a `std::unordered_set` as a reference for the planned index.

- **botman loaded:** the `main` deployment running (about 50 % of each of the
  12 cores, see below).
- **botman idle:** all 14 `swarmdeck-botman-*` containers stopped with
  `docker stop`, then restarted with `docker start`; nothing was recreated or
  moved.
- **tuf:** its stack was running idle; tuf is also a shared machine.
- **workstation:** load average 22, so its numbers are erratic.

| Single thread, ns per voxel query | botman loaded | botman idle | tuf | workstation |
|---|---:|---:|---:|---:|
| binary search, 250k cells | 330 | 278 | 115 | 1351 |
| binary search, 1M cells | 702 | 572 | 197 | 1613 |
| binary search, 4M cells | 1090 | 923 | 405 | 621 |
| `unordered_set`, 250k cells | 477 | 402 | 52 | 632 |
| `unordered_set`, 1M cells | 533 | 492 | 98 | 168 |
| `unordered_set`, 4M cells | 573 | 515 | 129 | 158 |

| Other | botman loaded | botman idle | tuf | workstation |
|---|---:|---:|---:|---:|
| binary search, 4M cells, all threads (ns per query) | 185 | 130 | 64 | 279 |
| sort 10M doubles (ms) | 1016 | 1013 | 603 | 1289 |
| float matmul 768, one thread (GFLOP/s) | 3.2 | 3.2 | 8.0 | 4.3 |
| memcpy, one thread / all threads (GB/s) | 18.8 / 60.8 | 19.7 / 74.7 | 26.4 / 40.2 | 32.4 / 66.8 |

| GPU | botman (Orin, 16 SMs) | tuf (RTX 4070) |
|---|---:|---:|
| device copy (GB/s) | 114 | 427 |
| cuBLAS SGEMM 4096 (TFLOP/s) | 3.4 | 20.7 |
| cuBLAS HGEMM 4096 (TFLOP/s) | 36.7 | 92.8 |

botman's deployment load at idle (`docker stats`, % of one core):

| Container | CPU % |
|---|---:|
| slam | 182 |
| nav2 | 77 |
| mgg | 63 (no plan cycles running) |
| adapter | 47 |
| duck-detector | 47 |
| vectornav | 41 |
| lidar | 41 |
| oak | 30 |
| media | 20 |
| TF publishers (4) | 7-9 each |

### Reading

- **MGG's voxel lookups are 2.3-2.9x slower on botman than on tuf** with the
  robot idle, and 2.7-3.6x slower under its own deployment load. Scaling tuf's
  plan cycles (median 0.69 s, lattice 0.49 s) gives about 1.7-2.4 s onboard.
  This is an estimate: botman runs `main`, with no MOLA grid, so MGG itself
  could not be measured there.
- **On the Orin, `std::unordered_set` barely beats the binary search.** It is
  slower at 250k cells and only 1.8x faster at 4M. On tuf it is 2-3x faster
  everywhere. The Orin's memory latency punishes node-based hashing, so the
  replacement index must be cache-friendly and chosen by its botman numbers.
- **The Orin's GPU is about 6x below the RTX 4070 in FP32 and 2.5x in FP16.**
  That rules it out for running the simulation, not for onboard inference.

## 2026-09-23T15:40 - tuf, SubT, exploring (4 robots): MGG A/B, flat cell index

Same stack (`0bb7f5a`); only the MGG image changes: `46e3e8e` (binary-search
cell lookup) against `bb45403` (flat open-addressed cell index). Each run
follows a simulation reset and explores for 180 s; runs alternate
baseline, index, baseline, index. The real-time factor was 1.00 in every run,
and the graph sizes match (about 1,500 vertices, 26-27k edges).

| MGG | Cycles | Wall median (max) ms | Lattice median ms | Gain median ms | mgg CPU % |
|---|---:|---:|---:|---:|---:|
| `46e3e8e` run 1 | 48 | 726 (1075) | 534 | 143 | 118 |
| `bb45403` run 1 | 52 | 200 (305) | 100 | 56 | 101 |
| `46e3e8e` run 2 | 60 | 674 (1050) | 466 | 134 | 114 |
| `bb45403` run 2 | 63 | 224 (366) | 115 | 64 | 87 |

- Plan cycles are 3.2x faster, the lattice build 4.6x (the plan's target was
  2x), the gain 2.3x. MGG uses less CPU while running more cycles.

### botman, `cell_index_bench` (MGG `ros2/src/mgg_map_octomap/bench`), deployment running

Jetson AGX Orin, `-O3`, g++ 11.4, the `main` deployment running (load
average ~4.8, two minutes after boot). Two runs, ns per query:

| Cells | Binary search | Flat index | Speed-up |
|---|---:|---:|---:|
| 250k | 350-363 | 147-152 | 2.3-2.5x |
| 1M | 706-712 | 161 | 4.4x |
| 4M | 1122-1126 | 170 | 6.6x |

- The flat index is 40.8 bytes per input cell against 24 for the sorted
  vectors.
- Scaling tuf's new plan cycle (~0.21 s) by the botman/tuf CPU factor
  measured earlier (2.3-3.6x) puts the onboard cycle at about 0.5-0.8 s,
  inside the 1 s target. This is an estimate until MGG runs on botman.

## 2026-09-23T17:21 - tuf, SubT, 4 robots, drift, RTX 4070, idle, after clean-up wave 1 (commit 5a40e7b)

- **Real-time factor 1.00** (29.6 s simulated in 29.6 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| argos | 56 |
| sim | 29 |
| peer2 | 11 |
| peer3 | 10 |
| peer0 | 10 |
| peer1 | 10 |
| mgg | 6 |
| server | 1 |
| mediamtx | 0 |
| mapping | 0 |
| ui | 0 |
| **total** | **133** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 56 |
| swarmdeck_argos_bridge.py | sim | 13 |
| python3 /r2 | peer2 | 7 |
| python3 /r3 | peer3 | 7 |
| python3 /r0 | peer0 | 7 |
| python3 /r1 | peer1 | 6 |
| adapter_sim.py | sim | 3 |
| lidar_handler_node.py /r0 | peer0 | 2 |
| lidar_handler_node.py /r3 | peer3 | 2 |
| lidar_handler_node.py /r2 | peer2 | 2 |
| lidar_handler_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r0 | peer0 | 2 |
| loop_closure_detection_node.py /r2 | peer2 | 2 |

- MGG plan cycles: none in the window (fleet not exploring?)

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 19.3 | 46.4 |
| fleet_change | 0.1 | 1.0 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

## 2026-09-23T17:23 - tuf, SubT, exploring (4 robots), RTX 4070, after clean-up wave 1 (commit 5a40e7b)

- Explore started by the harness
- **Real-time factor 1.00** (118.3 s simulated in 118.3 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| mapping | 175 |
| peer0 | 78 |
| mgg | 76 |
| server | 74 |
| peer2 | 73 |
| argos | 70 |
| peer1 | 67 |
| sim | 60 |
| peer3 | 12 |
| mediamtx | 0 |
| ui | 0 |
| **total** | **686** |

| Process | Container | CPU % |
|---|---|---:|
| swarmdeck-mola-import | mapping | 82 |
| python | server | 74 |
| argos3 | argos | 70 |
| loop_closure_detection_node.py /r0 | peer0 | 50 |
| loop_closure_detection_node.py /r2 | peer2 | 50 |
| swarmdeck-mola-import | mapping | 46 |
| loop_closure_detection_node.py /r1 | peer1 | 44 |
| swarmdeck-mola-import | mapping | 44 |
| mggplanner_node /robot_1/mgg | mgg | 26 |
| swarmdeck_argos_bridge.py | sim | 25 |
| mggplanner_node /robot_0/mgg | mgg | 24 |
| mggplanner_node /robot_2/mgg | mgg | 23 |
| python3 /r0 | peer0 | 21 |
| python3 /r1 | peer1 | 17 |
| python3 /r2 | peer2 | 16 |

- **MGG plan cycles: 37**, wall ms median 188 (max 283); lattice median 96, gain median 56; median 1496 vertices, 26173 edges

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 18.9 | 106.1 |
| fleet_change | 0.1 | 2.4 |
| session_state | 1.1 | 0.1 |
| alert | 0.5 | 0.1 |
| settings_state | 0.1 | 0.1 |
| detection_review | 0.1 | 0.0 |

### Plan-log-to-displacement (proxy; MGG plan → pose displacement > 0.10 m)

These historical proxy samples predate registration/reset rejection and
next-plan window cut-offs. World-pose corrections and later plans may have
been counted as motion for earlier plans; these are not dispatch latency
measurements and need remeasurement. CPU and planner-cycle measurements are
unaffected.

- MGG plan cycles in window: 35
  - Clock: host UTC wall clock (docker log timestamps vs container time.time()); typical error < 1 ms
  - **robot_0**; replan cadence: median 7.81 s, p90 13.07 s, max 31.28 s (11 intervals); latency: median 1.33 s, p90 3.55 s, max 6.64 s (11 samples)
  - **robot_1**; replan cadence: median 8.90 s, p90 12.24 s, max 12.99 s (11 intervals); latency: median 1.32 s, p90 1.53 s, max 2.17 s (12 samples)
  - **robot_2**; replan cadence: median 11.39 s, p90 15.80 s, max 15.80 s (10 intervals); latency: median 1.34 s, p90 3.02 s, max 3.14 s (11 samples)

## 2026-09-23T17:31 - tuf, SubT, 4 robots, drift, RTX 4070, idle (after exploring), robot_state signature fix (commit 8f00773)

- **Real-time factor 1.00** (28.5 s simulated in 28.5 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| argos | 56 |
| peer0 | 51 |
| sim | 29 |
| peer3 | 11 |
| peer2 | 10 |
| peer1 | 10 |
| mgg | 6 |
| server | 2 |
| mediamtx | 0 |
| mapping | 0 |
| ui | 0 |
| **total** | **174** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 56 |
| pose_graph_manager /r0 | peer0 | 41 |
| swarmdeck_argos_bridge.py | sim | 14 |
| python3 /r3 | peer3 | 7 |
| python3 /r0 | peer0 | 7 |
| python3 /r2 | peer2 | 6 |
| python3 /r1 | peer1 | 6 |
| adapter_sim.py | sim | 3 |
| lidar_handler_node.py /r3 | peer3 | 2 |
| lidar_handler_node.py /r0 | peer0 | 2 |
| lidar_handler_node.py /r2 | peer2 | 2 |
| lidar_handler_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r0 | peer0 | 2 |
| loop_closure_detection_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r2 | peer2 | 2 |

- MGG plan cycles: none in the window (fleet not exploring?)

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 4.1 | 10.6 |
| fleet_change | 0.1 | 1.0 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

### Reading: clean-up wave 1 (Phase 0, Phase 1, MGG index) on tuf

Against the 12:04/12:05 baselines (`0bb7f5a`), same launch (SubT, 4 robots,
drift, RTX 4070), real-time factor 1.00 throughout:

- **Exploring, 120 s:** stack CPU 869 % -> 686 % (-21 %). sim 79 -> 60, mgg
  94 -> 76, server 79 -> 74, peers 65-112 -> 12-78, mapping 195 -> 175.
- **MGG:** plan-cycle median 692 -> 188 ms, lattice 492 -> 96 ms, gain
  142 -> 56 ms, same graph size.
- **Dashboard traffic:** `robot_state` 185 -> 106 KB/s while exploring (the
  duplicate paths are gone). At idle, 19.3 -> 4.1 messages/s and 46 -> 11 KB/s
  after `8f00773`, whose signature ignores the authority age and float noise; the
  change-only broadcast did not fire before it.
- **Idle stack:** 136 % -> 133 %; tuf's idle load was already small. The
  17:31 idle sample was taken while the fleet was still settling after the
  exploration run, so its total (174 %) is not comparable.
- **Historical plan-log-to-displacement proxy (before the fixes above):**
  median ~1.3 s from an MGG plan to 0.1 m of reported pose displacement
  (p90 1.5-3.6 s, max 6.6 s), and a replan every 8-11 s.
  Remeasure before comparing this proxy to planning time (0.19 s) or treating
  it as an exploration-pace target. `robot_3` (Spot) produced no latency samples: its log
  shows lost peer reservations and "controller patience exceeded".
- **Not measured yet:** dashboard browser CPU (`--browser`), sim-container idle
  target (29 % against < 60 %: met), server idle target (1-2 %: met).

## 2026-09-23T18:30 - dashboard, old (`9a647b3`) against new (`daff383`, merged as `263dd92`), fleet parked

Production builds of both versions, served with `vite preview` on the
workstation against tuf's live backend (tunnelled), measured in headless
Chromium with SwiftShader, which makes each frame far more expensive than a
GPU would. CPU is the launched browser's process tree from `/proc`
(`profile_stack.py --browser`); frames are WebGL frames drawn per second.
Two alternating runs each, 15 s idle, then three mouse-drag pans.

| View | Build | Idle frames/s | Idle CPU % | Pan frames/s | Pan CPU % |
|---|---|---:|---:|---:|---:|
| 3D | old | 24.1 / 24.0 (30 with the fleet moving) | 1130 / 1129 | 25.9 / 26.8 | 1212 / 1230 |
| 3D | new | 0 / 0 | 35 / 35 | 9.5 / 9.6 | 251 / 254 |
| 2D | old | - | 1458 / 1478 | - | - |
| 2D | new | - | 81 / 81 (56 in a later run) | - | - |

- A parked fleet without animated decoration no longer redraws the maps.
  Keep-alives that change only clocks and float noise are ignored, and the
  replica poll redraws only on visible change. The 3D map renders on demand,
  while movement or panning uses the quality-tier cap. A selected robot's
  decoration still draws at 12 frames per second; a hidden tab draws nothing.
- The last ~400 % was the browser re-blurring the panels over the map every
  display frame (`backdrop-blur`); the panels are now opaque, without blur.
- The plan's target, the dashboard tab under 25 % CPU at idle, was set for
  Firefox with a GPU. It needs a check in a real browser, which SwiftShader
  cannot stand in for.

## 2026-09-23T19:17 - tuf, SubT, 4 robots, drift, RTX 4070, idle, final (commit 6cc9f15)

- **Real-time factor 1.00** (29.9 s simulated in 29.9 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| argos | 58 |
| sim | 30 |
| peer3 | 11 |
| peer1 | 11 |
| peer2 | 11 |
| peer0 | 11 |
| mgg | 6 |
| server | 0 |
| mediamtx | 0 |
| mapping | 0 |
| ui | 0 |
| **total** | **138** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 58 |
| swarmdeck_argos_bridge.py | sim | 14 |
| python3 /r3 | peer3 | 7 |
| python3 /r2 | peer2 | 7 |
| python3 /r1 | peer1 | 7 |
| python3 /r0 | peer0 | 7 |
| adapter_sim.py | sim | 3 |
| lidar_handler_node.py /r1 | peer1 | 2 |
| lidar_handler_node.py /r0 | peer0 | 2 |
| lidar_handler_node.py /r2 | peer2 | 2 |
| lidar_handler_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r2 | peer2 | 2 |
| loop_closure_detection_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r0 | peer0 | 2 |

- MGG plan cycles: none in the window (fleet not exploring?)

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 4.1 | 9.9 |
| fleet_change | 0.1 | 1.0 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

## 2026-09-23T19:18 - tuf, SubT, exploring (4 robots), RTX 4070, final (commit 6cc9f15)

- Explore started by the harness
- **Real-time factor 1.00** (119.5 s simulated in 119.5 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| mapping | 123 |
| mgg | 71 |
| peer2 | 69 |
| sim | 69 |
| argos | 66 |
| peer1 | 62 |
| server | 33 |
| peer0 | 30 |
| peer3 | 12 |
| mediamtx | 0 |
| ui | 0 |
| **total** | **535** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 66 |
| swarmdeck-mola-import | mapping | 55 |
| loop_closure_detection_node.py /r2 | peer2 | 46 |
| swarmdeck-mola-import | mapping | 41 |
| loop_closure_detection_node.py /r1 | peer1 | 41 |
| python | server | 33 |
| swarmdeck_argos_bridge.py | sim | 27 |
| swarmdeck-mola-import | mapping | 25 |
| mggplanner_node /robot_1/mgg | mgg | 23 |
| mggplanner_node /robot_2/mgg | mgg | 22 |
| mggplanner_node /robot_0/mgg | mgg | 18 |
| python3 /r2 | peer2 | 16 |
| python3 /r1 | peer1 | 15 |
| loop_closure_detection_node.py /r0 | peer0 | 14 |
| python3 /r0 | peer0 | 11 |

- **MGG plan cycles: 41**, wall ms median 192 (max 314); lattice median 105, gain median 50; median 1499 vertices, 26388 edges

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 18.9 | 125.2 |
| fleet_change | 0.1 | 2.5 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

### Plan-log-to-displacement (proxy) (MGG plan → navigation-frame displacement >= 0.10 m)
- MGG plan cycles in window: 32
- Cut-off plans without a displacement sample: 1
  - Clock: host UTC wall clock (docker log timestamps vs container time.time()); typical error < 1 ms
  - **robot_0**; replan cadence: median 12.04 s, p90 16.57 s, max 16.57 s (9 intervals); latency: median 1.39 s, p90 1.80 s, max 1.80 s (10 samples)
  - **robot_1**; replan cadence: median 12.14 s, p90 16.71 s, max 16.71 s (8 intervals); latency: median 1.38 s, p90 2.04 s, max 2.04 s (9 samples)
  - **robot_2**; replan cadence: median 11.01 s, p90 13.37 s, max 14.38 s (11 intervals); latency: median 1.44 s, p90 2.17 s, max 2.52 s (11 samples)

### Reading: clean-up wave 1, final (`6cc9f15`)

Same launch as the 12:04/12:05 baselines (`0bb7f5a`). The simulation ran at real time throughout.

| | Baseline `0bb7f5a` | Final `6cc9f15` |
|---|---:|---:|
| Stack CPU, idle | 136 % | 138 % |
| Stack CPU, exploring 120 s | 869 % | 535 % (-38 %) |
| server, exploring | 79 % | 33 % |
| sim, exploring | 79 % | 69 % |
| mapping, exploring | 195 % | 123 % |
| mgg, exploring | 94 % | 71 % |
| MGG plan cycle, median (max) | 692 (1040) ms | 192 (314) ms |
| MGG lattice build, median | 492 ms | 105 ms |
| `robot_state` at idle | 20 msg/s, 49 KB/s | 4.1 msg/s, 9.9 KB/s |

- Plan-log-to-displacement proxy, with the corrected trace (registration
  changes rejected, each plan cut off at the next): median 1.38-1.44 s,
  p90 1.8-2.2 s per robot; replans every 11-12 s. The time from a plan to
  the robot moving is now about seven times the planning time, which makes it
  the next exploration-pace target. The earlier ~1.3 s figure was measured
  with the uncorrected trace.
- The stack CPU while exploring varies with what the fleet is doing: peer3,
  the Spot, stays near idle in both runs.

## 2026-09-24T09:20 - tuf, SubT, 4 robots, drift, RTX 4070, idle, all lanes + nav-startup (commit 992272b)

- **Real-time factor 1.00** (27.9 s simulated in 27.9 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| argos | 57 |
| sim | 26 |
| peer3 | 11 |
| peer2 | 11 |
| peer0 | 11 |
| peer1 | 11 |
| mgg | 8 |
| server | 1 |
| mediamtx | 0 |
| mapping | 0 |
| ui | 0 |
| **total** | **134** |

| Process | Container | CPU % |
|---|---|---:|
| argos3 | argos | 57 |
| swarmdeck_argos_bridge.py | sim | 13 |
| cslam_bridge.py /r3 | peer3 | 7 |
| cslam_bridge.py /r2 | peer2 | 7 |
| cslam_bridge.py /r0 | peer0 | 7 |
| cslam_bridge.py /r1 | peer1 | 7 |
| adapter_sim.py | sim | 3 |
| lidar_handler_node.py /r1 | peer1 | 2 |
| lidar_handler_node.py /r0 | peer0 | 2 |
| lidar_handler_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r3 | peer3 | 2 |
| loop_closure_detection_node.py /r2 | peer2 | 2 |
| lidar_handler_node.py /r2 | peer2 | 2 |
| loop_closure_detection_node.py /r1 | peer1 | 2 |
| loop_closure_detection_node.py /r0 | peer0 | 2 |

- MGG plan cycles: none in the window (fleet not exploring?)

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 4.0 | 8.5 |
| fleet_change | 0.1 | 0.9 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

## 2026-09-24T09:21 - tuf, SubT, exploring (4 robots), RTX 4070, all lanes + nav-startup (commit 992272b)

- Explore started by the harness
- **Real-time factor 1.00** (117.6 s simulated in 117.6 s; /clock 10.0 Hz)

| Container | CPU % |
|---|---:|
| mapping | 139 |
| mgg | 98 |
| peer0 | 90 |
| argos | 69 |
| sim | 61 |
| peer3 | 47 |
| peer2 | 37 |
| server | 34 |
| peer1 | 12 |
| mediamtx | 0 |
| ui | 0 |
| **total** | **588** |

| Process | Container | CPU % |
|---|---|---:|
| swarmdeck-mola-import | mapping | 75 |
| argos3 | argos | 69 |
| lidar_handler_node.py /r0 | peer0 | 43 |
| swarmdeck-mola-import | mapping | 34 |
| python | server | 34 |
| mggplanner_node /robot_0/mgg | mgg | 27 |
| swarmdeck-mola-import | mapping | 27 |
| mggplanner_node /robot_3/mgg | mgg | 25 |
| mggplanner_node /robot_2/mgg | mgg | 24 |
| cslam_bridge.py /r0 | peer0 | 23 |
| swarmdeck_argos_bridge.py | sim | 22 |
| mggplanner_node /robot_1/mgg | mgg | 18 |
| cslam_bridge.py /r2 | peer2 | 18 |
| cslam_bridge.py /r3 | peer3 | 16 |
| loop_closure_detection_node.py /r3 | peer3 | 16 |

- **MGG plan cycles: 49**, wall ms median 178 (max 307); lattice median 102, gain median 48; median 1499 vertices, 26023 edges

| GUI message | per s | KB/s |
|---|---:|---:|
| robot_state | 16.8 | 113.9 |
| fleet_change | 0.1 | 2.7 |
| session_state | 1.1 | 0.1 |
| settings_state | 0.1 | 0.1 |
| alert | 0.4 | 0.1 |
| detection_review | 0.1 | 0.0 |

### Plan-log-to-displacement (proxy) (MGG plan → navigation-frame displacement >= 0.10 m)
- MGG plan cycles in window: 38
- Cut-off plans without a displacement sample: 2
  - Clock: host UTC wall clock (docker log timestamps vs container time.time()); typical error < 1 ms
  - **robot_0**; replan cadence: median 7.75 s, p90 12.39 s, max 13.48 s (12 intervals); latency: median 0.80 s, p90 1.65 s, max 2.35 s (13 samples)
  - **robot_1**; replan cadence: median 8.10 s, p90 14.74 s, max 16.74 s (12 intervals); latency: median 1.47 s, p90 2.07 s, max 3.29 s (12 samples)
  - **robot_2**; replan cadence: median 11.25 s, p90 30.62 s, max 30.62 s (8 intervals); latency: median 0.82 s, p90 2.99 s, max 2.99 s (9 samples)
  - **robot_3**; replan cadence: median 12.80 s, p90 20.67 s, max 20.67 s (2 intervals)

### Reading: clean-up waves 2 and 3, final (`992272b`)

Same launch as every tuf report above (SubT, 4 robots, drift, RTX 4070),
real-time factor 1.00 throughout.

| | Before the clean-up (`0bb7f5a`) | Wave 1 (`6cc9f15`) | Final (`992272b`) |
|---|---:|---:|---:|
| Stack CPU, idle | 136 % | 138 % | 134 % |
| Stack CPU, exploring 120 s | 869 % | 535 % | 509-588 % (varies with how many robots are actually driving) |
| MGG plan cycle, median | 692 ms | 192 ms | 168-187 ms |
| Plan log to 0.1 m of motion, median (proxy) | - | 1.38-1.44 s | 0.80-1.47 s (0.90-1.00 s typical) |
| Nav2 startup, robots active per launch | - | - | 4/4 in 9 of 9 launches (base: 19/20 robots) |
| `robot_state` at idle | 20 msg/s, 49 KB/s | 20 msg/s | 4.0 msg/s, 8.5 KB/s |

- The adapter's own per-path timing shows the reservation granted 0.15 s after
  the path, Nav2 accepting 0.15 s after it, and the next plan requested within
  0.002 s of the controller's result.
- Stop check: after `stop_explore`, every driving robot stopped within a second
  and did not move again.
- Shared memory: stable over three per-robot map resets (360 Fast DDS files,
  759 MB of the sim's 2 GB `/dev/shm`).
- Simulation speed (RTX 4070, unpaced, parked fleet): baseline 2.08x real time,
  all lidars at 5 Hz 2.26x (+9 %), parked lidars at 2 Hz 2.39x (+15 %); the GPU
  stays under 30 % busy, so on this host lidar rendering is not the bottleneck.
  Plan item 2.1 (weak GPUs) stays open.
