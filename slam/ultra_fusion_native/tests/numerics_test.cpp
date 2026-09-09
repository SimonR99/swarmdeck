#include "ultra_fusion_native/numerics.hpp"
#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

using namespace swarmdeck::ultra_fusion_native;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <class Function> void rejects(Function action) {
  bool rejected = false;
  try { action(); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "invalid input accepted");
}
void geometry() {
  std::mt19937 generator(793);
  std::normal_distribution<double> distribution;
  auto vector = [&]() { return Vec3(distribution(generator), distribution(generator),
                                   distribution(generator)); };
  for (int trial = 0; trial < 200; ++trial) {
    const Pose pose{expSO3(vector()), vector()};
    const Vec3 point = vector(), normal = vector().normalized();
    const auto factor = planeFactor(pose, point, normal, 0.7, 2.3, 0.4);
    for (int axis = 0; axis < 6; ++axis) {
      Vec6 delta = Vec6::Zero();
      delta(axis) = 1e-6;
      const double derivative =
          (planeFactor(plus(pose, delta), point, normal, 0.7, 2.3, 0.4).residual -
           planeFactor(plus(pose, -delta), point, normal, 0.7, 2.3, 0.4).residual) / 2e-6;
      check(std::abs(derivative - factor.jacobian(axis)) < 1e-8,
            "plane Jacobian disagrees with manifold finite difference");
    }
    const Pose old_extrinsic{expSO3(vector()), vector()};
    const Pose new_extrinsic{expSO3(vector()), vector()};
    const Pose updated = preserveLidarPose(pose, old_extrinsic, new_extrinsic);
    check(((updated * new_extrinsic).apply(point) -
           (pose * old_extrinsic).apply(point)).norm() < 1e-12,
          "extrinsic update moved a world LiDAR point");
    check((pose.inverse().apply(pose.apply(point)) - point).norm() < 1e-12,
          "pose inverse is inconsistent");
  }
  const Pose begin;
  Pose end;
  end.translation.x() = 2;
  end.rotation = expSO3(Vec3(0, 0, std::acos(-1.0) / 2));
  check((interpolate(begin, end, 0).apply(Vec3::UnitX()) - Vec3::UnitX()).norm() < 1e-12,
        "scan begin incorrect");
  check((interpolate(begin, end, 1).apply(Vec3::UnitX()) - end.apply(Vec3::UnitX())).norm() < 1e-12,
        "scan end incorrect");
  const Vec3 halfway(1 + std::sqrt(0.5), std::sqrt(0.5), 0);
  check((interpolate(begin, end, 0.5).apply(Vec3::UnitX()) - halfway).norm() < 1e-12,
        "scan rotation must interpolate on SO(3)");
  Pose negative = end;
  negative.rotation.coeffs() *= -1;
  check((interpolate(end, negative, 0.5).rotation.toRotationMatrix() -
         end.rotation.toRotationMatrix()).norm() < 1e-12, "quaternion sign changed the path");
  Pose extrinsic;
  extrinsic.translation = Vec3(0, 0, 1);
  check(std::abs(continuousPlaneResidual(begin, end, extrinsic, Vec3::UnitX(),
      0.5, Vec3::UnitZ(), Vec3(0, 0, 1), 2)) < 1e-12, "deskew/extrinsic composition incorrect");
  check(planeFactor(begin, Vec3::Ones(), Vec3::UnitZ(), 0, 0, 1).jacobian.isZero(),
        "zero weight must remove information");
  rejects([&]() { interpolate(begin, end, 1.1); });
  rejects([&]() { planeFactor(begin, Vec3::Zero(), Vec3::UnitZ(), 0, -1, 1); });
  Pose invalid;
  invalid.rotation.coeffs().setZero();
  rejects([&]() { plus(invalid, Vec6::Zero()); });
}
void vision() {
  Pose previous, current, extrinsic;
  current.translation = Vec3(1, 0, 0);
  check(visualResidual(previous, current, extrinsic, Eigen::Vector2d::Zero(),
                       Eigen::Vector2d(-0.5, 0), 0.5).norm() < 1e-12,
        "visual reprojection has wrong frame direction or depth");
  extrinsic.rotation = expSO3(Vec3(0.1, 0.3, -0.2));
  extrinsic.translation = Vec3(0.3, -0.1, 0.2);
  const Vec3 landmark(1, 2, 8);
  const Vec3 a = (previous * extrinsic).inverse().apply(landmark);
  const Vec3 b = (current * extrinsic).inverse().apply(landmark);
  check(visualResidual(previous, current, extrinsic, a.head<2>() / a.z(),
                       b.head<2>() / b.z(), 1.0 / a.z()).norm() < 1e-12,
        "visual camera lever arm incorrect");
  rejects([&]() { visualResidual(previous, current, extrinsic,
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), 0); });
  current.translation.z() = 10;
  rejects([&]() { visualResidual(previous, current, Pose{},
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), 1); });
}
void priors() {
  Eigen::MatrixXd j(5, 3);
  j << 1, 2, 0, 0, 1, 3, 2, -1, 1, 1, 0, 2, 0, 2, -1;
  Eigen::VectorXd r(5);
  r << 0.2, -1, 2, 0.4, 0.3;
  const auto prior = marginalize(j, r, 1);
  auto minimized_cost = [&](const Eigen::Vector2d& retained) {
    const Eigen::VectorXd rhs = r + j.rightCols(2) * retained;
    const Eigen::VectorXd eliminated = j.leftCols(1).colPivHouseholderQr().solve(-rhs);
    return (j.leftCols(1) * eliminated + rhs).squaredNorm();
  };
  const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  for (const Eigen::Vector2d x : {Eigen::Vector2d(1, 2), Eigen::Vector2d(-3, 0.5)}) {
    const double actual = (prior.jacobian * x + prior.residual).squaredNorm() -
                          prior.residual.squaredNorm();
    check(std::abs(actual - (minimized_cost(x) - minimized_cost(zero))) < 1e-10,
          "marginal prior does not preserve least-squares cost differences");
  }
  Eigen::MatrixXd gauge(1, 2);
  gauge << -1, 1;
  const auto null_prior = marginalize(gauge, Eigen::VectorXd::Ones(1), 1);
  check(null_prior.jacobian.isZero(1e-12) && null_prior.residual.isZero(1e-12),
        "eliminating an unanchored relative constraint must not create a prior");
  Eigen::MatrixXd singular(2, 3);
  singular << 1, 0, 1, 0, 0, 2;
  const auto singular_prior = marginalize(singular, Eigen::VectorXd::Zero(2), 2);
  check(std::abs(singular_prior.jacobian.squaredNorm() - 4) < 1e-12,
        "singular elimination lost an independent retained constraint");
  rejects([&]() { marginalize(j, r, 3); });
  check(admitFactor(0.5, 0.5, 10, 10), "admission boundary must be inclusive");
  check(!admitFactor(0.6, 0.5, 10, 10), "degraded factor admitted");
  check(!admitFactor(0.1, 0.5, 9, 10), "insufficient support admitted");
  check(!admitFactor(std::numeric_limits<double>::quiet_NaN(), 0.5, 10, 10),
        "NaN evidence admitted");
}
}  // namespace
int main() {
  try {
    geometry(); vision(); priors();
    std::cout << "Geometry, derivatives, reprojection, calibration continuity, priors, and admission passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
