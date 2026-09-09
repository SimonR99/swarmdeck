#include "ultra_fusion_native/numerics.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>

namespace swarmdeck::ultra_fusion_native {
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
void validPose(const Pose& pose) {
  require(pose.rotation.coeffs().allFinite() && pose.translation.allFinite() &&
              std::abs(pose.rotation.squaredNorm() - 1.0) < 1e-8,
          "pose requires finite translation and a unit quaternion");
}
void validWeight(double value) {
  require(std::isfinite(value) && value >= 0.0, "weight must be finite and nonnegative");
}
Eigen::MatrixXd symmetric(const Eigen::MatrixXd& matrix) {
  return (matrix + matrix.transpose()) * 0.5;
}
}  // namespace

Vec3 Pose::apply(const Vec3& point) const {
  validPose(*this);
  require(point.allFinite(), "point must be finite");
  return rotation * point + translation;
}
Pose Pose::inverse() const {
  validPose(*this);
  const Eigen::Quaterniond inverse_rotation = rotation.conjugate();
  return {inverse_rotation, -(inverse_rotation * translation)};
}
Pose Pose::operator*(const Pose& other) const {
  validPose(*this);
  validPose(other);
  return {(rotation * other.rotation).normalized(), apply(other.translation)};
}
Mat3 skew(const Vec3& value) {
  Mat3 result;
  result << 0, -value.z(), value.y(), value.z(), 0, -value.x(),
      -value.y(), value.x(), 0;
  return result;
}
Eigen::Quaterniond expSO3(const Vec3& tangent) {
  require(tangent.allFinite(), "rotation tangent must be finite");
  const double theta = tangent.norm();
  require(std::isfinite(theta), "rotation tangent norm overflow");
  const double theta2 = theta * theta;
  const double scale = theta < 1e-7 ? 0.5 - theta2 / 48.0
                                  : std::sin(theta * 0.5) / theta;
  return Eigen::Quaterniond(std::cos(theta * 0.5), scale * tangent.x(),
                            scale * tangent.y(), scale * tangent.z()).normalized();
}
Pose plus(const Pose& pose, const Vec6& delta) {
  validPose(pose);
  require(delta.allFinite(), "pose increment must be finite");
  return {(pose.rotation * expSO3(delta.tail<3>())).normalized(),
          pose.translation + delta.head<3>()};
}
Pose interpolate(const Pose& begin, const Pose& end, double alpha) {
  validPose(begin);
  validPose(end);
  require(std::isfinite(alpha) && alpha >= 0 && alpha <= 1,
          "point time must be in the scan interval");
  return {begin.rotation.slerp(alpha, end.rotation).normalized(),
          (1.0 - alpha) * begin.translation + alpha * end.translation};
}
PlaneEvaluation planeFactor(const Pose& pose, const Vec3& point,
                            const Vec3& normal, double offset,
                            double sqrt_information, double factor_scale) {
  validPose(pose);
  require(point.allFinite() && normal.allFinite() && std::isfinite(offset),
          "plane data must be finite");
  validWeight(sqrt_information);
  validWeight(factor_scale);
  const double scale = sqrt_information * factor_scale;
  validWeight(scale);
  PlaneEvaluation result;
  result.residual = scale * (normal.dot(pose.apply(point)) + offset);
  result.jacobian.head<3>() = scale * normal.transpose();
  result.jacobian.tail<3>() =
      -scale * normal.transpose() * pose.rotation.toRotationMatrix() * skew(point);
  return result;
}
double continuousPlaneResidual(const Pose& begin, const Pose& end,
                               const Pose& imu_from_lidar, const Vec3& lidar_point,
                               double alpha, const Vec3& normal,
                               const Vec3& plane_point, double sqrt_weight) {
  require(plane_point.allFinite(), "plane point must be finite");
  return planeFactor(interpolate(begin, end, alpha), imu_from_lidar.apply(lidar_point),
                     normal, -normal.dot(plane_point), sqrt_weight, 1.0).residual;
}
Eigen::Vector2d visualResidual(const Pose& previous, const Pose& current,
                              const Pose& imu_from_camera,
                              const Eigen::Vector2d& previous_observation,
                              const Eigen::Vector2d& current_observation,
                              double inverse_depth) {
  require(std::isfinite(inverse_depth) && inverse_depth > 0.0,
          "inverse depth must be positive and finite");
  require(previous_observation.allFinite() && current_observation.allFinite(),
          "image observations must be finite");
  const Vec3 ray(previous_observation.x(), previous_observation.y(), 1.0);
  const Pose current_from_previous =
      imu_from_camera.inverse() * current.inverse() * previous * imu_from_camera;
  const Vec3 point = current_from_previous.apply(ray / inverse_depth);
  require(point.allFinite() && point.z() > 1e-12, "point is behind the camera or at infinity");
  return point.head<2>() / point.z() - current_observation;
}
Pose preserveLidarPose(const Pose& world_from_imu, const Pose& old_imu_from_lidar,
                       const Pose& new_imu_from_lidar) {
  return world_from_imu * old_imu_from_lidar * new_imu_from_lidar.inverse();
}
LinearPrior marginalize(const Eigen::MatrixXd& jacobian,
                        const Eigen::VectorXd& residual, Eigen::Index drop,
                        double tolerance) {
  require(jacobian.rows() == residual.size() && jacobian.cols() > 0 &&
              drop >= 0 && drop < jacobian.cols(), "invalid marginalization dimensions");
  require(jacobian.allFinite() && residual.allFinite(), "linearization must be finite");
  require(std::isfinite(tolerance) && tolerance > 0 && tolerance < 1,
          "eigenvalue threshold must be between zero and one");
  const Eigen::MatrixXd hessian = jacobian.transpose() * jacobian;
  const Eigen::VectorXd gradient = jacobian.transpose() * residual;
  const Eigen::Index keep = jacobian.cols() - drop;
  Eigen::MatrixXd reduced = hessian.bottomRightCorner(keep, keep);
  Eigen::VectorXd reduced_gradient = gradient.tail(keep);
  if (drop > 0) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(hessian.topLeftCorner(drop, drop));
    require(solver.info() == Eigen::Success, "elimination eigensolve failed");
    const double cutoff = tolerance * solver.eigenvalues().cwiseAbs().maxCoeff();
    const Eigen::VectorXd inverse = solver.eigenvalues().unaryExpr(
        [cutoff](double value) { return value > cutoff ? 1.0 / value : 0.0; });
    const Eigen::MatrixXd pseudo_inverse =
        solver.eigenvectors() * inverse.asDiagonal() * solver.eigenvectors().transpose();
    const Eigen::MatrixXd cross = hessian.topRightCorner(drop, keep);
    reduced -= cross.transpose() * pseudo_inverse * cross;
    reduced_gradient -= cross.transpose() * pseudo_inverse * gradient.head(drop);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric(reduced));
  require(solver.info() == Eigen::Success, "prior eigensolve failed");
  // Refer to the original Hessian scale: cancellation during elimination must
  // not turn numerical residue in a gauge nullspace into artificial information.
  const double cutoff = tolerance * hessian.cwiseAbs().maxCoeff();
  Eigen::VectorXd root = solver.eigenvalues().unaryExpr(
      [cutoff](double value) { return value > cutoff ? std::sqrt(value) : 0.0; });
  Eigen::VectorXd inverse_root = root.unaryExpr(
      [](double value) { return value > 0 ? 1.0 / value : 0.0; });
  return {root.asDiagonal() * solver.eigenvectors().transpose(),
          inverse_root.asDiagonal() * solver.eigenvectors().transpose() * reduced_gradient};
}
bool admitFactor(double degeneracy, double threshold, int count, int minimum) {
  require(std::isfinite(threshold) && threshold >= 0 && threshold <= 1 &&
              minimum >= 0, "invalid factor admission settings");
  // Invalid evidence never admits a measurement.
  return std::isfinite(degeneracy) && degeneracy >= 0 && degeneracy <= 1 &&
         degeneracy <= threshold && count >= minimum;
}
}  // namespace swarmdeck::ultra_fusion_native
