#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// Partial reconstruction. See README.md for equation and binary provenance.
// This library is not a complete estimator or an upstream ABI replacement.
namespace swarmdeck::ultra_fusion_native {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Row6 = Eigen::Matrix<double, 1, 6>;

// Maps coordinates from the child frame into the parent frame.
struct Pose {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Vec3 translation = Vec3::Zero();
  Vec3 apply(const Vec3& point) const;
  Pose inverse() const;
  Pose operator*(const Pose& other) const;
};

Mat3 skew(const Vec3& value);
Eigen::Quaterniond expSO3(const Vec3& tangent);
// Tangent ordering: world translation, then body-frame rotation (paper Eq. 4).
Pose plus(const Pose& pose, const Vec6& delta);
Pose interpolate(const Pose& begin, const Pose& end, double alpha);

struct PlaneEvaluation {
  double residual;
  Row6 jacobian;
};
// Signed plane equation: normal.dot(world_point) + offset = 0.
// Binary weighting is sqrt_information * factor_scale, not sqrt(factor_scale).
// normal is used as supplied, matching the binary. Normally it is unit length.
PlaneEvaluation planeFactor(const Pose& world_from_body, const Vec3& body_point,
                            const Vec3& normal, double offset,
                            double sqrt_information, double factor_scale);
// Paper Eqs. 3, 6, 7. sqrt_weight = sqrt(omega_i); robust loss is applied later.
double continuousPlaneResidual(const Pose& begin, const Pose& end,
                               const Pose& imu_from_lidar, const Vec3& lidar_point,
                               double alpha, const Vec3& normal,
                               const Vec3& plane_point, double sqrt_weight);
// Paper Eq. 10. Inputs are already time-compensated normalized image points.
// Rejects invalid inverse depth or a reconstructed point behind either camera.
Eigen::Vector2d visualResidual(const Pose& world_from_previous,
                              const Pose& world_from_current,
                              const Pose& imu_from_camera,
                              const Eigen::Vector2d& previous_observation,
                              const Eigen::Vector2d& current_observation,
                              double inverse_depth);
// Paper Eq. 32: preserve the current world LiDAR pose at calibration commit.
Pose preserveLidarPose(const Pose& world_from_imu, const Pose& old_imu_from_lidar,
                       const Pose& new_imu_from_lidar);

// A linearized prior r(delta) = J * delta + residual at a fixed anchor.
// Caller owns the anchor and manifold difference; this is NOT relinearization.
struct LinearPrior {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
};
// Eliminate the first drop columns of a linear least-squares system using a
// Schur complement and PSD eigentruncation (paper Eq. 13). Keeps gauge nullspaces.
LinearPrior marginalize(const Eigen::MatrixXd& jacobian,
                        const Eigen::VectorXd& residual, Eigen::Index drop,
                        double relative_eigenvalue_threshold = 1e-10);
// Paper Eq. 15 only; hysteresis and modality score construction are not included.
bool admitFactor(double degeneracy, double threshold, int count, int minimum);
}  // namespace swarmdeck::ultra_fusion_native
