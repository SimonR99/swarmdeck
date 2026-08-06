#!/usr/bin/env python3
"""Republish Gazebo's odometry and IMU with covariance that means something.

Gazebo's DiffDrive plugin and IMU sensor both ship **all-zero covariance
matrices**, and every estimator downstream reads that the same way: as a
measurement of infinite precision. `robot_localization` sidesteps it today by
running on `process_noise_covariance` alone, which works but leaves the filter
unable to down-weight a momentarily bad sample. Anything that weighs its inputs
properly — RTAB-Map's ICP odometry, and a GTSAM back end most of all — cannot
sidestep it: a zero covariance is either rejected or believed absolutely, and
neither produces a sane estimate. This is docs/KNOWN_ISSUES.md #7.

So this node stamps the noise that `robot.sdf.jinja` actually injects, and
nothing more. It invents no information: the numbers here are the simulated
sensor's own noise model, and on hardware they are replaced by whatever the
driver reports. It publishes to `<ns>/odom_cov` and `<ns>/imu_cov` rather than
overwriting the bridged topics, so the raw Gazebo output stays available for
comparison and no topic ever has two publishers.

Two deliberate choices:

* **Orientation covariance is -1.** REP-145 reserves a leading -1 to mean "this
  message carries no orientation estimate", which is the truth: the fleet's IMU
  has no magnetometer, so it has no absolute yaw. Gazebo would happily hand over
  a perfect world-referenced quaternion, and fusing it would be laundering
  ground truth through a filter — the same trap ekf.yaml already documents.
* **`vy` is tight, `vx` is loose.** A differential drive cannot move sideways,
  so zero lateral velocity is a kinematic constraint rather than a measurement
  and deserves to be trusted. Forward velocity comes from wheel encoders, whose
  dominant error is slip, so it does not.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Defaults mirror the noise in swarmdeck_description/urdf/robot.sdf.jinja.
# Gyro: stddev 9.0e-4 rad/s, so variance 8.1e-7; rounded up to leave room for
# the bias random walk the sensor also carries.
GYRO_VAR = 1.0e-6
# Accelerometer: stddev 1.7e-2 m/s^2, so variance 2.89e-4.
ACCEL_VAR = 3.0e-4
# Wheel-derived forward velocity. sigma 0.05 m/s is not the encoder's precision,
# it is an allowance for slip, which is the error that actually matters.
VX_VAR = 2.5e-3
# Lateral velocity: a constraint, not a measurement. sigma 0.01 m/s.
VY_VAR = 1.0e-4
# Integrated pose. Nothing fuses this today (ekf.yaml takes velocity only), but
# lidar-inertial odometry and a pose-graph back end both read it.
POSE_XY_VAR = 0.05
POSE_YAW_VAR = 0.02
# Large but finite, so a 2D estimator is not tempted to believe z/roll/pitch.
UNOBSERVED_VAR = 1.0e6


def _diagonal(*values: float) -> list[float]:
    """A row-major 6x6 covariance with the given diagonal."""
    matrix = [0.0] * 36
    for i, value in enumerate(values):
        matrix[i * 7] = value
    return matrix


class CovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("covariance_relay")
        self.declare_parameter("gyro_variance", GYRO_VAR)
        self.declare_parameter("accel_variance", ACCEL_VAR)
        self.declare_parameter("vx_variance", VX_VAR)
        self.declare_parameter("vy_variance", VY_VAR)
        self.declare_parameter("pose_xy_variance", POSE_XY_VAR)
        self.declare_parameter("pose_yaw_variance", POSE_YAW_VAR)

        def value(name: str) -> float:
            return float(self.get_parameter(name).value)

        big = UNOBSERVED_VAR
        self._pose_cov = _diagonal(
            value("pose_xy_variance"), value("pose_xy_variance"), big,
            big, big, value("pose_yaw_variance"),
        )
        self._twist_cov = _diagonal(
            value("vx_variance"), value("vy_variance"), big,
            big, big, value("gyro_variance"),
        )
        gyro = value("gyro_variance")
        accel = value("accel_variance")
        self._gyro_cov = [gyro, 0.0, 0.0, 0.0, gyro, 0.0, 0.0, 0.0, gyro]
        self._accel_cov = [accel, 0.0, 0.0, 0.0, accel, 0.0, 0.0, 0.0, accel]

        self._odom_pub = self.create_publisher(Odometry, "odom_cov", 10)
        self._imu_pub = self.create_publisher(Imu, "imu_cov", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(Imu, "imu", self._on_imu, 50)

    def _on_odom(self, msg: Odometry) -> None:
        msg.pose.covariance = self._pose_cov
        msg.twist.covariance = self._twist_cov
        self._odom_pub.publish(msg)

    def _on_imu(self, msg: Imu) -> None:
        # -1 in the leading element is REP-145 for "no orientation estimate".
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        msg.angular_velocity_covariance = self._gyro_cov
        msg.linear_acceleration_covariance = self._accel_cov
        self._imu_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
