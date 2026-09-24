# C-SLAM performance patches

`upstream.repos` pins the sources. Apply the patches in
`deploy/docker/Dockerfile.cslam` order; do not apply them alphabetically.

- `cslam-unchanged-graph.patch` still collects peer graphs, but skips the solver
  when the collected measurement factors, initial poses, map epochs and origin
  exactly match a successful solve. Feedback-derived anchor/attitude priors are
  added only after comparing this immutable key. A remote-only change
  invalidates the cache. Failed solves are retried. Unchanged rounds publish the
  cached successful result with a new solution clock, preserving deferred adoption
  and restarted-consumer recovery. The bridge authority heartbeat is unchanged.
- `cslam-scancontext-index.patch` retains a ring-key KD-tree and searches up to
  63 newly appended keys directly. Every 64 additions rebuild the index. Ties
  fall back to the full tree to preserve upstream candidate ordering. Thresholds
  and the single-best-match API are unchanged.
- `cslam-scancontext-distance.patch` normalizes descriptor columns once and uses
  one matrix product for every sector pairing. Cyclic diagonals produce all yaw
  shifts, retaining zero-column exclusion and the original score definition.
- `cslam-downsample-buffer.patch` converts ROS float32 points to contiguous
  float64 arrays in NumPy before Open3D downsampling. Voxel centroids and
  nonfinite-point rejection are unchanged.

The bridge normalizes directly in the cloud callback and flushes captures on
keyframe and raw-provenance events. Each DDS-delivered cloud is now considered:
the previous 50 ms latest-cloud coalescing is gone. At the configured 10 Hz lidar
rate this does not increase normal work, but under executor overload up to the
sensor QoS queue depth (5) of queued clouds can be considered. A newer cloud still
replaces an older cloud waiting for capture-time TF. There are no idle 20 Hz normalization or
10 Hz capture-join polls. Missing capture-time TF arms an on-demand steady-clock
retry, backing off from 50 ms to 1 s. A pending cloud is discarded after the
existing 3 s sensor-freshness limit, with a rate-limited warning; a newer cloud
replaces it. A raw/provenance join retains its existing 0.5 s grace period with
an on-demand deadline timer. Both steady-clock retry timers are allocated once,
then rearmed with `reset()` and canceled when no work remains. Only the snapshot
timer remains active at idle. `status.json.capture_tf_lookup_misses` counts failed
capture-time TF lookup attempts (including retries), so the controller can measure
the miss rate. The 1 Hz snapshot cadence, write-on-change status behavior,
authority thread and epoch fence are unchanged.

RGB-D caches remain bounded (8 records, 64 MiB per stream, 16 MiB per message)
and decode-free. Frames up to 250 ms before a scan can be valid color sources;
opening a cache only after forwarding that scan would lose those observations.

## Focused verification

Build from the repository root:

```sh
docker build -f deploy/docker/Dockerfile.cslam -t swarmdeck-cslam:lane .
```

Run native tests and Python tests against the actual patched upstream modules:

```sh
docker run --rm -v "$PWD/deploy/cslam/tests:/tests:ro" \
  -e PYTHONPATH=/cslam_ws/src/cslam -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  swarmdeck-cslam:lane bash -lc '
    . /opt/ros/jazzy/setup.sh
    . /cslam_ws/install/setup.sh
    bash /tests/run_pose_graph_test.sh
    cmake -S /tests/native -B /tmp/native-tests
    cmake --build /tmp/native-tests -j2
    /tmp/native-tests/pose_graph_rounds
    /tmp/native-tests/pose_graph_rounds residual
    python3 -m pytest -q -p no:cacheprovider /tests/test_*.py
    python3 /tests/benchmark_scancontext.py
    python3 /tests/benchmark_downsample.py
    python3 /tests/benchmark_scancontext_distance.py
  '
```

The Python repository suite does not collect these upstream-dependent tests;
run them in the image. Bridge tests remain ROS-free under
`autonomy/tests/test_cslam_bridge_solutions.py`.
