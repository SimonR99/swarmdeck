# SwarmDeck — Architecture, Operational Reference & State

---

## 1. Architectural Principles

### "Merge Trajectories, Not Grids"
- **Raw Occupancy Grids Are Projections, Not Source Truth**: Rasterizing range returns discards sensor origins and ray histories. Correlating grids after the fact is lossy and fragile.
- **Collaborative Pose-Graph Optimization (`swarmdeck-slam`)**:
  - Robots continuously stream voxel-downsampled 3D keyframe clouds in their `base_link` frame along with odometry poses (`POST /api/adapter/keyframe`).
  - The SLAM backend uses **Scan Context** descriptors with a KD-tree index for loop closure candidate generation.
  - Candidates undergo **Generalized ICP (GICP)** geometric verification with stringent fitness/overlap gates and information matrix estimation.
  - **Pairwise Consistency Maximization (PCM)** and **Graduated Non-Convexity (GNC)** reject perceptual aliasing and false positive inter-robot loop closures.
  - Consistent 2D occupancy grids and 3D voxel centroids are rendered directly from optimized trajectory poses.

---

## 2. System Components & Ports

| Component | Port / Path | Purpose & Responsibilities |
|---|---|---|
| **Frontend UI** | `:5173` / `ui/` | SvelteKit dashboard, WebGL2 3D viewer, Canvas 2D map viewer, teleoperation controls, detection reviewer. Defaults to `mapSource: "optimized"`. |
| **Server Backend** | `:8080` / `server/` | FastAPI orchestration, robot registry, telemetry routing, map patch broadcaster, navigation map downlink, object review persistence. |
| **Collaborative SLAM** | `:8090` / `slam/` | Python 3.12 / GTSAM pose graph optimizer, conditioned-Hessian loop closure weighting, probabilistic ray clearing (`render.py`), and scoped grid publisher. |
| **MediaMTX** | `:8554` (RTSP), `:8889` (WHEP) | Low-latency H.264 video streaming from onboard robot cameras. |
| **Zenoh Router** | `:7447` | Zero-overhead, multi-robot DDS bridging. |

---

## 3. Fleet Configurations & Hardware Profiles

| Robot ID | Platform | Mapping Stack | Map / Cloud Topics | Video Topic |
|---|---|---|---|---|
| `tars_0` | Scout Mini (Jetson Noetic) | LVI-SAM + EKF | `/lvi_sam/lidar/mapping/cloud_registered` ($Z \in [0.1, 4.5]\text{ m}$) | `/camera/color/image_raw/compressed` |
| `botman_0` | Bunker (Jetson Humble) | SuperOdometry | `/registered_scan`, `/laser_odometry` | `/oak/rgb/image_raw/compressed` |
| `aslan_0` | Bunker (Jetson Humble) | SuperOdometry | `/registered_scan`, `/laser_odometry` | `/oak/rgb/image_raw/compressed` |
| `spot_0` | Boston Dynamics Spot | Spot SDK / Clearpath | `/spot/odometry`, `/spot/lidar/points` | `/spot/camera/frontright/image/compressed` |

---

## 4. Key Data Ingestion & Rendering Pipelines

1. **Keyframe Production & Streaming**:
   - `KeyframeUploader` (`adapters/keyframe_producer.py`): Captures voxelized point clouds (5 cm rendering downsample) when the robot moves >= 0.5 m or turns >= 15 deg.
   - Encodes opaque wire packets with JSON metadata and zlib-compressed int16 coordinate arrays (`swarmdeck_protocol`).
2. **Real-Time SLAM Optimization**:
   - The worker drains everything queued, then optimizes once (`OPTIMIZE_EVERY_N = 1`), so a keyframe triggers an optimization while the service keeps up and a backlog costs one solve rather than one per blob.
   - `T_world_map` per robot is least-squares fitted over that robot's whole trajectory, not read off its newest keyframe — the source frame moves whenever a robot's own SLAM re-optimizes.
   - Occupancy grid rasterization uses log-odds ray clearing (+3 hits, -1 misses) to remove transient dynamic clutter, walked in length-sorted batches so peak memory does not track cloud density.
   - Output scopes (`component:<id>`, `robot:<id>`) are published to the server and pushed live via WebSocket. Each update names its live scopes; the server drops the rest, since component ids are positional and retire on merge.
3. **3D Point Cloud Denoising**:
   - `server/swarmdeck_server/mapsvc/output.py` runs 3D voxel-centroid consolidation (4 cm lattice) over multi-robot returns for clean, noise-free 3D visualization.
4. **2D Viewport Stability**:
   - Viewport translations dynamically compensate for world origin shifts when map bounding boxes expand, ensuring walls and robot markers remain stable on screen.
   - Canvas coordinate row-order conversions ensure incremental patches align seamlessly with full snapshots.

---

## 5. Verification Commands

```bash
# Run server and adapter test suite (441 tests)
./server/.venv/bin/pytest server/tests adapters/test

# Run SLAM test suite (111 tests)
./slam/.venv/bin/pytest slam/tests

# Deploy software updates to real robot hardware
make deploy ROBOT=scout
make deploy ROBOT=botman
make deploy ROBOT=aslan
make deploy ROBOT=spot

# Check live SLAM backend status
curl -s http://localhost:8090/status | jq .

# Check active optimized map scopes
curl -s http://localhost:8080/api/map/optimized | jq .
```
