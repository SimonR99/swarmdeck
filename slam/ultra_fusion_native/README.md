# Ultra-Fusion numerical reconstruction

This is a **partial C++ reconstruction**, not a runnable odometry node. It builds
without ROS, Ceres, the upstream binaries, or binary translation. Eigen is the only
dependency. No claim of full Ultra-Fusion equivalence or ARM performance is made.

## Evidence and implemented scope

Paper: [Ultra-Fusion, arXiv:2606.21223v1, Section III](https://github.com/sjtuyinjie/Ultra-Fusion/blob/9235a9601c532fc570f9ee7337a03d23611e148c/paper/Ultra-Fusion.pdf).
The PDF examined has SHA256
`fc0001cc3a94a7460569f84f104d273f34065dc43a0de3518ce61936ea5b9931`.
The inspected ROS 2 v0.2.2 library has SHA256
`c6b311b3ca7c976f3c3639da11d9a444f4b18ce9309f0dede5895698549a940e`.

| Component | Evidence | What was reconstructed |
| --- | --- | --- |
| `planeFactor` | Paper Eqs. 6–7 and disassembly below | Signed plane residual, multiplicative weighting, translation Jacobian, right-local rotation Jacobian |
| `interpolate`, `continuousPlaneResidual` | Paper Eqs. 3, 6–7 | SLERP rotation, linear translation, LiDAR-to-IMU extrinsic composition at point time |
| `plus` | Paper Eq. 4 | Right SO(3) rotation update and additive world translation |
| `visualResidual` | Paper Eq. 10 | Inverse-depth normalized reprojection with camera extrinsics; observations must already be time compensated |
| `marginalize` | Paper Eq. 13 and accompanying Schur-complement description | Gaussian prior construction with pseudoinverse elimination and eigenvalue truncation |
| `admitFactor` | Paper Eq. 15 | Score/support admission rule, including equality boundaries |
| `preserveLidarPose` | Paper Eq. 32 | World-LiDAR-pose continuity when committing new extrinsics |

Except for the plane-factor inspection, these are implementations of the published
mathematics, not recovered upstream function bodies. APIs, input validation,
relative eigentruncation threshold, and test harness are new implementation choices.
The prior is linearized at a caller-owned anchor; reconstructing its state packing
and manifold displacement remains work for the window implementation.

## Binary reconstruction of the plane factor

The constructor at virtual address `0x551d80` stores its first vector at object
offset `0x28`, second vector at `0x40`, and scalar arguments at `0x58` and `0x60`.
`UF::CT_ICP::LidarPlaneNormFactor::Evaluate`, at `0x553f70`, reveals:

- Parameter block 0 is translation; block 1 is an Eigen quaternion in `x,y,z,w`
  storage order. The constructor configures Ceres block sizes 3 and 4.
- The point at `0x28` is quaternion-rotated and translated, dotted with the vector
  at `0x40`, then added to scalar `0x58`.
- The residual is multiplied by scalar `0x60` and the static `sqrt_info` value.
- The translation derivative is the normal multiplied by both weights.
- The rotation derivative computes `-sqrt_info * normal.transpose() * R * skew(point)`
  and multiplies by scalar `0x60`. The fourth derivative slot is zero.

Thus the recovered expression is
`r = sqrt_info * factor_scale * (normal.dot(R * point + t) + offset)`.
`planeFactor` returns the six **local tangent** derivatives explicitly. The binary's
four-slot rotation output is not a general ambient quaternion derivative and must
not be attached to a standard Ceres quaternion manifold without reconciling its
parameterization. This library deliberately does not expose an upstream Ceres ABI.
Neither the global runtime `sqrt_info` setting nor its initialization is guessed.

To inspect the same code in an extracted release:

```bash
objdump -d -C --start-address=0x553f70 --stop-address=0x554240 \
  /path/to/opt/ultrafusion/lib/libultra_lib.so
objdump -d -C --start-address=0x551d80 --stop-address=0x551e70 \
  /path/to/opt/ultrafusion/lib/libultra_lib.so
```

## Build and verification

```bash
cmake -S slam/ultra_fusion_native -B /tmp/uf-native -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uf-native -j2
ctest --test-dir /tmp/uf-native --output-on-failure
```

Tests check 200 randomized plane Jacobians by central differences on the pose
manifold, scan endpoint and midpoint geometry, quaternion sign ambiguity, extrinsic
commit invariance, nonzero camera extrinsics, invalid input rejection, marginal
cost differences against independent QR elimination, and rank-deficient priors.
Checks remain active in Release builds. This validates selected mathematics;
it is not a differential test against the running upstream estimator.

Validation performed: GNU C++ 15.2.0 Release build on x86-64 succeeded; CTest
passed the numerical test executable, including all checks described above.

The build has no x86-specific ISA options. ARM compilation and measurements on
robot hardware remain unverified.

## Remaining estimator work

No ROS node, sensor queues, point-cloud processing, feature tracker, initialization,
IMU/wheel preintegration, nonlinear sliding-window optimizer, map correspondence
search, raw GNSS factors, or calibration worker has been reconstructed here. The
paper references other work for several of these components and leaves some
scheduler and admission details to implementation. Binary investigation is still
needed for those details and for differences between the paper and v0.2.2.

The scalar admission rule is not the complete reliability scheduler: modality
scores, hysteresis, and covariance inflation remain absent. The visual factor does
not estimate camera time offset. The calibration function applies an accepted
extrinsic update; it does not estimate that update or decide when to accept it.

Swarmdeck backend registration remains pending an executable and validated native
estimator. See [the integration investigation](../../docs/architecture/ultra-fusion-native-arm.md).
