# Tactical 3D map

The dashboard opens the tactical 3D map without a URL parameter. Deselect
**Layers → 3D cloud** to return to 2D, and select it to reopen the tactical map.
The optional `?view=2d` URL starts directly in 2D. The map uses registered
clouds selected by the map source. **Optimized** uses the accumulated, solved
keyframe cloud; **Robot SLAM** uses the latest robot uploads. Optimized falls
back to those uploads while reconstruction is unavailable or empty. X/Y are world
metres and Z is up. A local robot cloud is transformed into world coordinates
for agreement with robot overlays and navigation goals.

- **Voxels:** instanced occupied cells, automatically coarsened to fit the budget.
- **Mesh:** triangle boundary of those occupied cells; internal faces are removed.
  This is a block surface mesh, not a watertight TSDF or a photogrammetric mesh.
- **Points:** the measured cloud, with elevation, team, or calibrated camera color.
- **Gaussians:** a published reconstruction, rendered with projected anisotropic
  covariance and alpha blending. Missing reconstructions show an explicit status.
  The initial integration supports world-aligned global reconstructions only.

Left drag orbits, right/middle drag pans, scroll zooms, and left click selects a
robot. Shift-click adds to selection. Navigation mode sends ground-plane X/Y
coordinates through the existing navigation action. The height slider clips the
map to inspect interiors. It does not change robot navigation maps.

## Rendering budgets

| Profile | Points | Occupied cells | Gaussians | Max device pixel ratio | FPS cap |
| --- | ---: | ---: | ---: | ---: | ---: |
| Low power (default) | 60,000 | 8,000 | 30,000 | 1 | 30 |
| Balanced | 160,000 | 20,000 | 80,000 | 1.5 | 45 |
| High detail | 300,000 | 40,000 | 150,000 | 2 | 60 |

These are workload limits, not measured FPS guarantees. Shadows and multisample
antialiasing are disabled. Geometry preparation and Gaussian sorting use workers.
Only selected terrain representations allocate GPU geometry; switching modes
caches their buffers until the next cloud. Overlays reuse unchanged geometry,
and mode switches reuse the current color palette without re-uploading unchanged
color buffers. Changing color mode or loading a new cloud invalidates that palette;
only one palette is retained, keeping memory bounded. Terrain preparation avoids
allocating a typed-array view for every ground sample and mesh vertex.

Overlay textures are disposed/reused, and rendering pauses in hidden tabs. The 2D canvas
pauses behind the 3D view. Three.js is loaded when 3D is first opened. Switching
to 2D retains the bounded 3D scene, camera, and display settings while stopping
its animation loop and map requests. Returning to 3D immediately resumes the
loaded tactical map, including when the server returns 304 or is unavailable.

The viewer polls clouds every two seconds with conditional ETags and cancels
requests on scope changes, hiding, and unmount. Gaussian models poll every five seconds while
selected. Server fusion/compression runs off the async event loop. Gaussian
sorting runs at most 10 Hz and only when camera orientation changes. DC color
replaces higher spherical-harmonic bands; a 384-pixel maximum ellipse radius
bounds close-up overdraw. This reduces fidelity relative to UMAMI's CUDA viewer.

## Colorize LiDAR with camera images

In the ROS 2 hardware adapter configuration, opt into calibrated color projection:

```yaml
map_color:
  enabled: true
  max_age_s: 0.05
```

Configure the existing `topics.camera` or `topics.camera_compressed` and
`topics.camera_color_info` (or `topics.camera_info`) to a **rectified** image and
its matching CameraInfo. The image and cloud timestamps must be within the
configured tolerance. Capture-time TF must resolve camera-optical <- map.
Nonzero distortion coefficients, mismatched sizes/frames, missing calibration,
or stale images suppress color rather than projecting incorrectly. Only the
nearest cloud surface per pixel is colored; unseen points are neutral gray.
This is per-upload coloring, not persistent texture fusion, and sparse LiDAR
cannot guarantee occlusion rejection between its returns. The shared projection
helper also accepts aligned metric depth for stronger visibility checking.

Legacy XYZ uploads remain accepted. `POST /api/adapter/cloud?robot_id=...&format=xyzrgb32`
accepts zlib-compressed planar data: N little-endian float32 XYZ triples followed
by N uint8 RGB triples. No scale is applied to float32 positions. Colors use the
same voxel membership as positions during fusion.

`GET /api/map/cloud` describes its layout with `X-Cloud-Format` (`xyz32` or the
SLAM fallback's `xyz16`), `X-Cloud-Points`, `X-Cloud-Scale`, `X-Cloud-Robots`, and
`X-Cloud-RGB`. Layout: XYZ, then one uint8 robot index per point, then optional
RGB triples. Consumers must honor the format header. Float32 output avoids
wrapping world coordinates beyond the legacy int16 range.

## Capture RGB-D and reconstruct with UMAMI-SLAM

The integration was checked against private UMAMI-SLAM commit
`b1251d435b09f4298a414dbc1151c9bae42c3c37`, specifically
`examples/train_colmap.cpp` and `src/gaussian_model.cpp`, rather than its copied
README. UMAMI is an external installation; its private code is not vendored.
The reconstruction runner is separate from the server and UI.

1. In the robot's ROS 2 environment (with NumPy and `message_filters`), record
   synchronized rectified RGB and RGB-aligned depth. Depth must be `16UC1`
   millimetres or `32FC1` metres. Use the actual topic names:

   ```bash
   python3 scripts/reconstruction/capture_rgbd.py \
     --rgb /camera/color/image_rect \
     --depth /camera/aligned_depth_to_color/image_raw \
     --camera-info /camera/color/camera_info \
     --world-frame world --output /data/run-rgbd
   ```

   Capture is bounded to 2,000 frames, one per second by default, with one writer
   and no unbounded queue. Stop with Ctrl-C. RGB/depth optical frames and pixel
   grids must match. TF is sampled at the image timestamp, with no latest-pose
   fallback. `world` must be the **same metric frame as SwarmDeck's displayed
   cloud**. A robot-local map frame is not automatically the fleet world frame.
   If necessary, compose the fleet transform into each recorded pose before export.

2. Export on the reconstruction machine (NumPy and Pillow):

   ```bash
   python3 scripts/reconstruction/umami.py export /data/run-rgbd /data/run-colmap
   ```

   Captures are NPZ files containing `rgb` (uint8 HxWx3), `depth_m` (HxW), `K`
   (3x3), `T_world_camera` (4x4, optical axes right/down/forward), and `stamp`.
   The exporter writes COLMAP binary cameras/images/points and RGB PNGs. Metric
   depth supplies bounded seed geometry; the camera poses are fixed during
   UMAMI's photometric training. Export a consistent pose snapshot: re-export
   and retrain after loop-closure corrections. Do not mix disconnected robots.

3. Build UMAMI in its supported CUDA environment, then run its actual headless
   COLMAP trainer and publish the trained result:

   ```bash
   git clone git@github.com:lemonci/UMAMI-SLAM.git /opt/UMAMI-SLAM
   # Build bin/train_colmap using UMAMI's environment/build instructions.
   python3 scripts/reconstruction/umami.py train \
     --umami /opt/UMAMI-SLAM \
     --config /opt/UMAMI-SLAM/cfg/colmap/gaussian_splatting.yaml \
     --dataset /data/run-colmap --output /data/run-trained \
     --publish /data/swarmdeck-reconstructions/global.swgs
   ```

   Training errors propagate and leave the previous published model intact.
   The converter reads UMAMI's binary little-endian Gaussian PLY: log scales,
   WXYZ quaternions, opacity logits, and DC spherical harmonics. It filters
   invalid/invisible entries and limits the published model to 150,000 splats.
   An existing **world-aligned** result can be published separately:

   ```bash
   python3 scripts/reconstruction/umami.py convert \
     /data/result/point_cloud.ply /data/swarmdeck-reconstructions/global.swgs
   ```

4. Set `SWARMDECK_RECONSTRUCTION_DIR=/data/swarmdeck-reconstructions` in the
   **server process**. For Docker, bind-mount that directory and set the environment
   variable in the server container. Select **Gaussians** in the global 3D view.
   Publishing atomically replaces the file and the viewer loads the new version.
   Remove `global.swgs` when starting a different mapping session: it is a saved
   reconstruction and does not automatically follow live map resets.

The compact SWGS v1 format is a 16-byte little-endian header (`SWGS`, version 1,
uint32 count, reserved uint32), then 14 float32 values per Gaussian: XYZ, positive
XYZ scales, XYZW quaternion, linear RGB, opacity. Models are in world metres with Z up.
The read-only endpoint accepts no user-supplied filesystem path.

## Validation

```bash
npm run check --prefix ui
npm run build --prefix ui
npm run test:map3d --prefix ui
python3 -m pytest server/tests/test_reconstruction.py \
  server/tests/test_map_concurrency.py server/tests/test_stack.py \
  adapters/test/test_adapter_ros2.py adapters/test/test_runtime.py
```

Geometry/transport/export regressions do not need CUDA. Real camera/TF capture,
UMAMI training quality, and sustained low-end GPU FPS require hardware runs;
synthetic browser rendering checks cannot establish those results.

## Overlay frames and camera color

The tactical view uses world coordinates in both global and single-robot views.
The graph backend accepts a position-fitted map frame only when both trajectories
have at least 5 cm RMS spread in a second direction and that spread is at least
1% of their dominant spread. Stationary and nearly straight paths cannot
determine a full 3D rotation from positions alone, even with low fitting error;
they retain the frame derived from keyframe poses. This prevents unconstrained
pitch/roll from tilting a robot's entire cloud. The check runs once per trajectory
fit, not per point or rendered frame.

Cloud responses declare `X-Cloud-Frame`: direct robot maps are `local`, while the
collaborative SLAM cloud is already `world`. Only local XYZ receives the robot's
map-to-world transform. Robot markers and paths arrive in world coordinates;
costmaps and network grids receive their source robot's translation and yaw.

Planned routes use screen-space triangle lines with dark outlines, and the
selected robot has an outlined team ring and white brackets. Robot symbols have
a minimum display size as the camera zooms out. These overlays render above the
terrain to remain legible; sensor footprints retain their physical scale.

Simulator keyframes can now carry measured camera colors. The producer projects
only accepted, voxel-reduced scans using the RGB camera's intrinsics, known
optical mount, capture-time pose, and aligned depth for occlusion rejection.
RGB/depth stamps must agree within 50 ms, and the image must be within 250 ms of
the scan. Missing calibration, historical poses, or valid depth leaves the
keyframe uncolored. Color does not change geometry or optimizer constraints.

The SLAM cloud prefers a colored observation when several returns share a
voxel. The Camera control enables when the map supplies RGB; unobserved areas
remain neutral gray. Existing XYZ-only keyframes cannot gain camera colors
retroactively. Simulator and SLAM processes must load this version and collect
new keyframes before Camera becomes available on such a run.

### Stable raster updates

Map PNG responses carry their resolution, dimensions, and origin in `X-Map-*`
headers from the same snapshot as the pixels. The UI uses those headers instead
of an earlier metadata poll, preventing stretching or displacement while the map
expands. Raster updates preserve world positions on screen, including when the
view is rotated or the grid resolution changes.
