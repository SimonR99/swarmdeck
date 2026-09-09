# Ultra-Fusion native ARM investigation

Status: **partial numerical reconstruction; no runnable Ultra-Fusion backend added**.
The [native C++ numerical library](../../slam/ultra_fusion_native/README.md) now
implements selected paper equations and a plane residual/Jacobian traced through
the binary. This does not establish full estimator equivalence, ARM compatibility,
or real-time performance. The inspection below establishes the remaining binary scope.

This is a bounded investigation, not the default odometry path or a committed
rewrite objective. Current simulation uses Fast-LIVO2; see the
[development objectives](roadmap.md) before extending this work.

## Release examined

- Public repository revision: `9235a9601c532fc570f9ee7337a03d23611e148c`.
- [ROS 2 v0.2.2 release](https://github.com/sjtuyinjie/Ultra-Fusion/releases/tag/v0.2.2).
- Asset: `ultrafusion-ros2_0.2.2_amd64.deb`.
- Verified SHA256: `243f88fa5e3d87fcd96a2b02c8561fca4d5e56419ab72ec0a3a731b0ea34cccc`.
- Inspection used archive extraction, ELF headers, dynamic dependencies, exported
  symbols, and embedded strings. No release binary or installation script was run.

The [upstream README at the inspected revision](https://github.com/sjtuyinjie/Ultra-Fusion/blob/9235a9601c532fc570f9ee7337a03d23611e148c/README.md)
says that full source will be released after paper acceptance. Public Python files
are tooling, not the estimator implementation. The
[packaging script](https://github.com/sjtuyinjie/Ultra-Fusion/blob/9235a9601c532fc570f9ee7337a03d23611e148c/scripts/package_ros2_deb_from_build.sh)
expects a separate source/build tree and explicitly strips the compiled artifacts.
Running that script does not reconstruct the missing sources.

## Measured binary scope

| Artifact under `/opt/ultrafusion` | Bytes | Exported function symbols (`T`/`W`) |
| --- | ---: | ---: |
| `bin/uf_node` | 998,376 | 285 |
| `bin/uf_ros2_adapter` | 838,000 | 25 |
| `lib/libultra_lib.so` | 9,383,048 | 6,385 |

All three are x86-64 ELF files without `.debug_info` or `.symtab`. Dynamic C++
symbols survive; the counts include template instantiations and aliases, so they
are not counts of unique original source functions. The package also bundles
x86-64 Ceres 2.1.0, which would need a native dependency build or package.

`uf_node` dynamically depends on `libultra_lib.so`. Consequently, rewriting only
the executable or ROS adapter cannot produce native ARM odometry.

Surviving library symbols identify these implementation areas:

- `UltraFusion::Estimator::UFEstimate`, state conversion, local-map search,
  and GNSS factor construction.
- `UltraFusion::IMUIntegrator` preintegration and gyro integration.
- `UltraFusion::GfStandaloneVio` initialization, optimization, and sliding window.
- `UltraFusion::MAP_MANAGER` map insertion and coordinate transformations.
- GNSS alignment, visual feature tracking, and numerous Ceres factors.

Embedded assertion paths also identify missing translation units including
`src/ultra_model/lio/Estimator.cpp`, `IMUIntegrator.cpp`,
`lidar_undistortion.cpp`, `src/model/factor/integration_base.cpp`,
`src/model/initial/initial_alignment.cpp`, and `src/sub/ros2/ros_subscriber.cpp`.
These are filename strings, not embedded source files.

Exported names aid reverse engineering but do not recover class member layouts,
function bodies, all types, or build configuration. A decompiler can assist a
manual reconstruction; its pseudocode is not automatically portable C++.
The plane-factor evaluator has since been manually traced in disassembly and
reconstructed in C++; full estimator decompilation and equivalence remain incomplete.

## Reproduce the inspection

On a Linux host with Python 3, `dpkg-deb`, and GNU binutils, download the asset
above and run from the swarmdeck repository root:

```bash
python3 slam/tools/inspect_ultrafusion.py \
  /path/to/ultrafusion-ros2_0.2.2_amd64.deb \
  /tmp/ultrafusion-evidence
```

The output directory must not already exist. The tool verifies the pinned archive
hash before extraction and records per-artifact hashes, machine types, dependencies,
exported symbols, section headers, and surviving source paths. It neither installs
nor executes the inspected binaries. The compact results from this investigation
are in [ultra-fusion-binary-manifest.json](ultra-fusion-binary-manifest.json).

## Work needed for a native backend

The missing prerequisite is a buildable native estimator. Obtaining upstream C++
is the most direct route. Binary reconstruction remains a separate substantial
engineering task: recover the data layouts and control flow, identify reusable
dependency implementations, reconstruct the numerical kernels and orchestration,
then compare behavior with the reference release. Related Ground-Fusion code is a
potential reference, not evidence of Ultra-Fusion equivalence.

Before registering `ultra_fusion` in
`adapters/protocol/swarmdeck_protocol/odometry.py`, implementation needs to cover:

1. A reproducible ARM64 Release build of the estimator and its native dependencies.
2. A ROS launch/configuration layer for sensor topics, calibration, timestamps,
   namespaces, and the chosen fusion modes. Required sensors depend on that mode.
3. Verified odometry frame semantics and one owner for `odom -> base_link`;
   hardware compose files currently configure backend selection and TF sidecars.
4. If simulation support is included, a sensor/pose bridge for the ARGoS Unix
   socket transport used by the existing Fast-LIVO2 integration. The registry's
   `medium="uf"` is a transport name, not an existing Ultra-Fusion backend.
5. Recorded-data comparisons of initialization, trajectory error, resets,
   sensor dropouts, and calibration behavior against the reference binary.
6. Latency, CPU, and memory measurements on the actual ARM robot at its sensor
   rates. Native compilation alone does not establish adequate performance.

The registry is unchanged because there is no executable ARM implementation to
launch. No ARM build, SLAM replay, or runtime performance test has been performed.
