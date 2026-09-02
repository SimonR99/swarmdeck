#!/usr/bin/env python3
"""Re-express FAST-LIVO2's odometry in the chassis frame, for nav2.

WHY THIS IS NEEDED

  FAST-LIVO2 publishes the pose of the IMU it fuses, not the robot. Its
  odometry message carries T_world_imu with frame_id "camera_init" and
  child_frame_id "aft_mapped", taken straight from the filter state.

  On Botman the IMU is a VN-100 mounted about 87.5 degrees off the chassis
  (measured by scripts/calibration/calibrate_imu_to_imu.py, composed with the
  Ouster's factory 180 degree os_lidar mounting). Feeding that to odom_to_tf
  would publish map -> botman_base_link with the robot's heading wrong by that
  much, so nav2's footprint and every costmap would be rotated.

  This node applies the fixed rotation and republishes:

      T_world_base = T_world_imu . T_imu_base

  q_out = q_in (x) q_imu_base,  p_out = p_in + R_in . t_imu_base

CAVEAT ON TRANSLATION
  imu_base_translation defaults to zero because it has never been measured:
  comparing two gyros gives rotation only, since angular velocity is identical
  everywhere on a rigid body. While that is zero, a pure spin in place shows
  spurious translation equal to the true lever arm. Measure the VN-100's
  position relative to base_link with a ruler and set it.

Usage:
    ros2 run ... fastlivo_odom_to_base.py --ros-args \
        -p in_topic:=/fastlivo/aft_mapped_to_init -p out_topic:=/laser_odometry
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FastLivoOdomToBase(Node):
    def __init__(self) -> None:
        super().__init__("fastlivo_odom_to_base")
        self.declare_parameter("in_topic", "/fastlivo/aft_mapped_to_init")
        self.declare_parameter("out_topic", "/laser_odometry")
        self.declare_parameter("child_frame", "botman_base_link")
        # R(base_link -> vectornav) as (x, y, z, w). Default measured on Botman
        # 2026-08-28: RPY 0.88, 0.45, 87.48 deg.
        self.declare_parameter("imu_base_quat", [0.00284045, 0.00820173, 0.69135670, 0.72246147])
        # Vectornav origin in base_link. UNMEASURED, see the caveat above.
        self.declare_parameter("imu_base_translation", [0.0, 0.0, 0.0])
        # Re-origin so the first pose is identity, which makes the output
        # directly comparable with another odometry source starting at rest.
        self.declare_parameter("zero_at_start", True)
        # Divergence gate. FAST-LIVO2 fails by losing scan-match lock, after
        # which the pose runs away on IMU integration alone: measured on Botman
        # going 9,696 -> 17,201 m over 18 s while still publishing at 15 Hz and
        # reporting finite numbers. A NaN check does not catch that, so gate on
        # implied speed and stop forwarding once it is clearly gone.
        self.declare_parameter("max_speed_mps", 5.0)
        self.declare_parameter("max_bad_before_mute", 5)

        self.q_ib = np.array(self.get_parameter("imu_base_quat").value, dtype=float)
        self.q_ib /= np.linalg.norm(self.q_ib)
        self.t_ib = np.array(self.get_parameter("imu_base_translation").value, dtype=float)
        self.child = self.get_parameter("child_frame").value
        self.zero_at_start = self.get_parameter("zero_at_start").value
        self.origin: tuple[np.ndarray, np.ndarray] | None = None
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.max_bad = int(self.get_parameter("max_bad_before_mute").value)
        self.bad = 0
        self.muted = False
        self.last: tuple[float, np.ndarray] | None = None

        rpy = self._rpy(quat_to_R(self.q_ib))
        self.get_logger().info(
            "base_link -> imu rotation RPY %.2f %.2f %.2f deg, lever arm %s%s"
            % (*[math.degrees(v) for v in rpy], self.t_ib.tolist(),
               "  (lever arm is zero: spins will show spurious translation)"
               if not self.t_ib.any() else "")
        )

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            Odometry, self.get_parameter("out_topic").value, qos)
        self.create_subscription(
            Odometry, self.get_parameter("in_topic").value, self.on_odom,
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                       reliability=ReliabilityPolicy.BEST_EFFORT))

    @staticmethod
    def _rpy(R: np.ndarray) -> tuple[float, float, float]:
        sy = math.hypot(R[0, 0], R[1, 0])
        return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        p_wi = np.array([p.x, p.y, p.z])
        q_wi = np.array([o.x, o.y, o.z, o.w])
        n = np.linalg.norm(q_wi)
        if not np.isfinite(p_wi).all() or not np.isfinite(n) or n < 1e-9:
            # The estimator emits NaN once scan matching collapses. Do not
            # forward that to nav2: a diverged pose is worse than none.
            self.get_logger().warn("dropping a non-finite pose from the estimator",
                                   throttle_duration_sec=5.0)
            return
        q_wi = q_wi / n

        q_wb = quat_mul(q_wi, self.q_ib)
        p_wb = p_wi + quat_to_R(q_wi) @ self.t_ib

        if self.zero_at_start:
            if self.origin is None:
                self.origin = (p_wb.copy(), q_wb.copy())
            p0, q0 = self.origin
            q0_inv = np.array([-q0[0], -q0[1], -q0[2], q0[3]])
            R0_inv = quat_to_R(q0_inv)
            p_wb = R0_inv @ (p_wb - p0)
            q_wb = quat_mul(q0_inv, q_wb)

        # Reject physically impossible motion rather than hand it to nav2.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None:
            dt = stamp - self.last[0]
            if dt > 1e-4:
                speed = float(np.linalg.norm(p_wb - self.last[1])) / dt
                if speed > self.max_speed:
                    self.bad += 1
                    if self.bad >= self.max_bad and not self.muted:
                        self.muted = True
                        self.get_logger().error(
                            "estimator diverged (%.0f m/s implied over %d samples); "
                            "muting output. Restart the estimator to clear." %
                            (speed, self.bad))
                    return
                self.bad = 0
        if self.muted:
            return
        self.last = (stamp, p_wb.copy())

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = self.child
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p_wb
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q_wb
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = FastLivoOdomToBase()
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
