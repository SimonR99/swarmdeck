# Reconstruction jobs

Gaussian reconstruction is an optional, asynchronous map product. Capture and
the geometric mapper remain usable when the worker, CUDA runtime, or private
UMAMI installation is unavailable. The worker consumes a fixed pose snapshot;
it does not publish TF, update Swarm-SLAM, or change navigation frames.

The durable runner is `autonomy.reconstruction`. It stores one JSON journal per
job, a content hash for every captured NPZ frame, the capture metadata, the pose
revision, and a backend capability declaration. A job whose input or pose
revision changes before publication becomes `stale`; its artifact cannot replace
the newer target. The runner uses a per-job lock for workers in separate CLI
processes. Cancellation writes a marker that a running process polls while the
trainer is active.

## Capture and metadata

Run capture in the ROS 2 environment with NumPy and `message_filters`:

```bash
python3 scripts/reconstruction/capture_rgbd.py \
  --rgb /camera/color/image_rect \
  --depth /camera/aligned_depth_to_color/image_raw \
  --camera-info /camera/color/camera_info \
  --world-frame odom \
  --output /data/run-rgbd \
  --robot-id botman_0 \
  --session-id 550e8400-e29b-41d4-a716-446655440000 \
  --submap-id submap-0 \
  --calibration-version oak-cal-4 \
  --capture-id run-20260909-120000 \
  --keyframe-metadata-topic /botman_0/keyframes
```

The old command line remains valid. The capture writes `capture_manifest.json`
and keeps the legacy `rgb`, `depth_m`, `K`, `T_world_camera`, and `stamp` NPZ
fields. New frames also carry IDs, calibration and rectification metadata,
depth validity, and depth units. When a stable SLAM keyframe TF frame is
available, add `--keyframe-frame <frame>` to retain `T_keyframe_camera`; this
relation is what allows a later pose solution to produce a new training pose
snapshot. With `--keyframe-metadata-topic`, each RGB-D frame is associated by
timestamp with onboard JSON metadata containing `keyframe_id` in
`robot/session/seq` form, `stamp_ns`, `odom_frame`, and
`T_odom_keyframe`; the capture looks up exact-time `T_odom_camera` and records
`inverse(T_odom_keyframe) @ T_odom_camera` as dynamic `T_keyframe_camera` for
corrected-pose export. It only associates a preceding keyframe within the
configured maximum age. The legacy static
`--keyframe-id`/`--keyframe-frame` options remain available for old captures,
but they cannot provide dynamic keyframe association. `--metadata-json` can
carry measured camera-body or RGB-depth extrinsics and exposure metadata that
are available in the deployment.

Supplied capture, robot, submap, keyframe, and calibration identifiers must be
safe nonempty identifiers; a supplied session ID must be a canonical lowercase
UUID. Empty values remain accepted for older deployments, and omitted capture
IDs are generated automatically.

The capture requires synchronized, registered RGB and depth with matching
optical frames and pixel dimensions. `16UC1` depth is converted from
millimetres; `32FC1` depth is treated as metres. A captured `world` pose must
be the same metric frame used by the map product. A robot-local frame is not
silently promoted to a fleet frame.

## Export and direct manual training

Export a bounded COLMAP dataset without the job runner when inspecting a
dataset manually:

```bash
python3 scripts/reconstruction/umami.py export \
  /data/run-rgbd /data/run-colmap \
  --stride 8 --max-points 150000
```

The exporter records capture IDs, frame IDs, calibration metadata, and whether
each frame retained a keyframe-to-camera transform in `swarmdeck.json`. Depth
seeds metric points; it does not prove that UMAMI applies a depth loss.

An existing world-aligned PLY can be compacted independently:

```bash
python3 scripts/reconstruction/umami.py convert \
  /data/result/point_cloud.ply \
  /data/swarmdeck-reconstructions/global.swgs \
  --budget 150000
```

The converter filters invalid and invisible entries and writes SWGS atomically.
The SWGS artifact is an inspection product. Its opacity is not occupancy
probability and it is not a navigation map.

## Durable batch job runner

The CLI requires the public SwarmDeck repository, a capture directory, a UMAMI
installation supplied by the operator, and a UMAMI config. It does not clone a
private repository or read SSH credentials.

Submit a fixed-pose batch job:

```bash
python3 -m autonomy.reconstruction submit \
  --store /data/reconstruction-journal \
  /data/run-rgbd \
  --umami /opt/UMAMI-SLAM \
  --config /opt/UMAMI-SLAM/cfg/colmap/gaussian_splatting.yaml \
  --artifact /data/swarmdeck-reconstructions/global.swgs \
  --pose-snapshot /maps/550e8400-e29b-41d4-a716-446655440000/botman_0/graph_solution.json \
  --max-frames 2000
```

Use the same mission UUID and robot ID as the onboard peer when capturing;
its map directory publishes `graph_solution.json` automatically. Mount that
directory read-only into the reconstruction worker, or synchronize it while
preserving atomic file replacement.

The pose snapshot is an immutable JSON wrapper around one validated
`GraphSolution` (`schema: swarmdeck.pose-snapshot.v1`). At submission the
runner copies and digest-checks it under the job journal; the backend reads
that copy while the original source path remains recorded for stale-result
detection. Its
`T_component_keyframe` values are composed with each dynamically captured
`T_keyframe_camera` as `T_component_camera`; missing, mismatched, retracted,
or changed keyframes fail the job rather than silently using the old capture
pose. A job fingerprints the snapshot bytes at submission and marks itself
stale if those bytes change before publication.

This first batch runner conservatively treats every source snapshot change,
including newly added keyframes, as stale. Run training after capture and graph
optimization have settled. Incremental training and invalidation limited to
affected submaps remain later milestones.

The command prints a `job_id`. Query it while a worker is running:

```bash
python3 -m autonomy.reconstruction status \
  --store /data/reconstruction-journal <job_id>
```

Start the worker from a machine with the native UMAMI executable and its CUDA
environment:

```bash
python3 -m autonomy.reconstruction run \
  --store /data/reconstruction-journal <job_id> \
  --umami /opt/UMAMI-SLAM \
  --config /opt/UMAMI-SLAM/cfg/colmap/gaussian_splatting.yaml
```

The runner invokes the existing `umami.py export` and `umami.py train` commands.
Training first writes a job-local staged SWGS artifact. Publication moves it
to an immutable, job-suffixed artifact and atomically replaces the adjacent
`global.swgs.manifest.json` pointer containing the backend, input fingerprint,
pose revision, and authoritative artifact path. The requested
`global.swgs` path is retained as a compatibility copy; readers that need a
coherent artifact/metadata pair must follow the manifest's `artifact` path.
The server reads that same pointer when `SWARMDECK_RECONSTRUCTION_DIR` points
at the publication directory. `GET /api/map/gaussians` serves the immutable
artifact bytes with frame, session, component, version, and integrity headers. Use
`GET /api/map/gaussians?format=status` for bounded source metadata (including capture, calibration, frame inventory,
and pose revisions) without downloading the artifact. A stale, canceled, failed, malformed, or checksum-
mismatched pointer returns `409`; the previous ready pointer remains the
authoritative served result when a newer job is rejected before publication.
When the pointer declares `source.frame: component`, both `session_id` and
`component_id` query parameters are required and must match the pose snapshot's
component authority; an unscoped global request is rejected. Clients may add
`pose_revision` and `input_fingerprint` query parameters as expected-revision
guards. This endpoint serves one operator-published model at a time; it does
not replicate or transform Gaussian artifacts between disconnected components.
Raw captures in `odom`, `map`, or another robot-local frame are labeled `local`
and cannot be served here until corrected component poses or a verified world
transform are supplied. `frame_id` identifies the output frame and
`capture_frame` preserves the original camera-pose frame. Explicit `world` and
legacy captures without frame metadata retain world compatibility. Older schema-2
pointers lacking source metadata must be republished; their output frame cannot
be inferred safely. Validation uses one disk worker with eight waiting slots and
a bounded stat-keyed checksum cache, so repeated status polls do not rehash the
model or block the server event loop.
The validated private integration target is UMAMI-SLAM commit
`b1251d435b09f4298a414dbc1151c9bae42c3c37`; source inspection confirmed its
native `bin/train_colmap` invocation is
`bin/train_colmap <config> <dataset> <output> no_viewer`.

The journal and capture directory are local trusted inputs. Keep the journal
directory writable only by the worker account, and review a job manifest before
running a journal received from another machine. The runner rejects frame names
that escape the capture directory, staged artifacts that escape the job work
directory, and publication targets inside the capture or journal trees. The
publication target itself is an operator-supplied path and should be a dedicated
reconstruction directory.

Cancel from another terminal or process:

```bash
python3 -m autonomy.reconstruction cancel \
  --store /data/reconstruction-journal <job_id>
```

The process that owns the trainer polls the cancellation marker and terminates
the trainer process group. A canceled or stale artifact is never published.
If the worker process disappears, a new runner checks the per-job lock before
recovering a journal entry. A live `training` entry is left alone; an abandoned
entry is returned to `queued` and retried from a clean preparation directory
because this backend does not declare checkpoint or resume support.

## Budgets and backend limits

The UMAMI adapter declares fixed camera poses and batch operation. It declares
no checkpoint resume, incremental updates, internal pose refinement, external
pose-constraint API, or depth-loss capability. Requests for undeclared
capabilities fail validation rather than silently changing the pose authority.

LiDAR odometry supplies continuous motion estimates, and Swarm-SLAM supplies
corrected keyframe poses. Calibrated camera extrinsics place the RGB views in
that same frame. UMAMI receives these poses through `train_colmap`; this path
does not start ORB-SLAM3 tracking or add a second visual pose optimizer. The
pinned Gaussian mapper and viewer libraries still link to ORB-SLAM3, so the
current image builds it as a dependency. Extracting a standalone reconstruction
library can remove that build/runtime dependency without changing pose ownership.

The runner enforces frame count, readable input resolution, input byte limit,
training subprocess wall time, job work-directory disk usage, and compact
Gaussian count. `max_upload_bytes` is currently a validation limit on the
selected input because this slice does not implement an upload transport.
`max_gpu_memory_mb` is enforced only when the runner is constructed with a GPU
memory monitor; the CLI does not claim GPU memory enforcement without one.
Training wall time applies to each bounded trainer subprocess. Dataset export
has no native UMAMI progress API, so an operator should run export with a
bounded capture and disk budget. The work-directory disk cap is checked while
commands run; the final artifact destination is expected to be a separately
managed filesystem budget.

## Validation boundary

CPU-only tests use inert fake `bin/train_colmap` executables and verify journal,
publication, cancellation, and stale-result behavior. They do not validate
CUDA kernels, UMAMI photometric quality, checkpoint behavior, incremental
training, native pose constraints, depth losses, metric alignment on hardware,
or viewer performance. The private UMAMI commit recorded by the architecture
plan is an integration target, not a substitute for validating the exact built
executable and configuration in its authorized CUDA environment.

Native UMAMI/GPU validation was not run for this repository slice. A bounded
RTX 3080 validation is feasible in an isolated CUDA image, using the pinned
private checkout only as a build input. The reproducible build recipe is
[`scripts/reconstruction/Dockerfile.build`](../../scripts/reconstruction/Dockerfile.build):
it uses BuildKit named contexts for the private source and machine-provided
dependencies, so no private source is copied from the public repository. The
inspected UMAMI CMake project requires CUDA and LibTorch, OpenCV with CUDA and
opencv_contrib, Eigen, Boost, jsoncpp, OpenGL/GLFW, and GLM. Its tested
dependency versions are CUDA 11.8, cuDNN 8.7/8.9, OpenCV 4.7/4.8, and
LibTorch 2.0.1 cu118 with the cxx11 ABI. The recipe selects
`GIOJO_CUDA_ARCHS=86` for the RTX 3080.

Build and run the pinned native trainer in the isolated image:

```bash
UMAMI_SRC=/tmp/swarmdeck-umami-inspect
UMAMI_REF=$(git -C "$UMAMI_SRC" rev-parse HEAD)
DOCKER_BUILDKIT=1 docker build \
  -f scripts/reconstruction/Dockerfile.build \
  --build-context umami="$UMAMI_SRC" \
  --build-arg UMAMI_REF="$UMAMI_REF" \
  --build-arg OPENCV_VERSION=4.8.0 \
  --build-arg LIBTORCH_URL=https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip \
  --build-arg BOOST_VERSION=1.80.0 \
  --build-arg JSONCPP_VERSION=1.9.5 \
  --build-arg GIOJO_CUDA_ARCHS=86 \
  -t "swarmdeck-umami:${UMAMI_REF}" .

docker run --rm --gpus all \
  -v /data/umami-config:/inputs/config:ro \
  -v /data/run-colmap:/inputs/dataset:ro \
  -v /data/umami-output:/outputs \
  "swarmdeck-umami:${UMAMI_REF}" \
  /inputs/config/gaussian_splatting.yaml /inputs/dataset /outputs no_viewer
```

The dependency stage is the expensive part: the LibTorch archive is about
2--3 GB compressed, and the CUDA OpenCV build, compiler temporaries, CUDA base
layers, and Docker cache can consume substantially more than the downloaded
archives. Allow at least 30 GB of free disk for a clean build and its cache.
The validation workstation has 61 GiB of system RAM and a 10 GiB RTX 3080;
lower-memory build configurations have not been validated. The
DBoW2, g2o, and ORB-SLAM3 libraries are built explicitly because this pinned
checkout has no prebuilt `.so` files. ORB links g2o by filename, so selecting
the ORB target alone does not build its g2o prerequisite.
`BUILD_JOBS` controls dependency builds; `NATIVE_BUILD_JOBS` independently limits
UMAMI/ORB compilation and defaults to two jobs. This lets native concurrency
change without rebuilding CUDA/OpenCV dependencies. ORB's upstream configuration
forces Debug while adding `-O3`; the image disables its debug symbols with `-g0`.
The three-view fixture bounds the input size; native training time and GPU
memory use still need to be measured.

The source manifest for this validation target is UMAMI-SLAM
`b1251d435b09f4298a414dbc1151c9bae42c3c37`; the native command above is the
actual `train_colmap` interface found in `examples/train_colmap.cpp`. No
native config has completed validation yet; the CPU tests use an inert trainer and a
minimal `fixed_pose: true` test config. The repository includes a deterministic
three-view, 64x48 calibrated loader fixture for the next authorized GPU run:

```bash
python3 scripts/reconstruction/umami_native_fixture.py /data/umami-native-smoke
docker run --rm --gpus all \
  -v /data/umami-native-smoke/dataset:/inputs/dataset:ro \
  -v /data/umami-native-smoke/umami_smoke.yaml:/inputs/config/umami_smoke.yaml:ro \
  -v /data/umami-native-smoke/output:/outputs \
  "swarmdeck-umami:${UMAMI_REF}" \
  /inputs/config/umami_smoke.yaml /inputs/dataset /outputs no_viewer
```

The fixture checks binary COLMAP export and native loading/training only; it is
not a quality or convergence benchmark. Run the image only on the authorized
GPU workstation, mounting capture/config/output data as shown. This validates
the native executable in its isolated environment; it does not grant the
trainer pose refinement, checkpoint, incremental, or depth-loss capabilities.

After a real job, inspect the generated manifest and compare the Gaussian model
against an independent geometric reference. A successful SWGS conversion only
proves that the PLY was structurally readable and bounded; it does not establish
map accuracy or justify replacing a geometric map.
