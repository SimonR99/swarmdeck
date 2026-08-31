#!/usr/bin/env python3
"""Calibrate IMU extrinsics relative to the robot base and LiDAR via passive motion listening.

SAFETY GUARANTEE: This script is strictly PASSIVE. It NEVER commands or publishes velocity to the robot.
All movements are performed manually by the operator using their remote control / joystick.

Procedure:
1. Static phase: records stationary IMU data to estimate the gravity vector, static tilt and gyro bias.
2. Motion phase: the operator rotates the robot in place. The script records angular velocities and
   estimates the direction of the rotation axis in the IMU frame.
3. Lever arm: solves a = ([alpha]x + [omega]x^2) r for the IMU offset from the rotation axis.

OBSERVABILITY (read this before trusting any number below):
  A single in-place yaw spin excites exactly one rotation axis. That determines the DIRECTION of
  that axis in each sensor frame (2 DoF: roll and pitch) and nothing else. The rotation ABOUT the
  spin axis (yaw) is mathematically unobservable from such a run, so this script refuses to report
  it. To pin yaw down you need a second, non-parallel rotation (drive one side over a threshold or
  a ramp so the robot pitches or rolls) or an independent yaw reference.

  Likewise, r_z is unobservable from a yaw-only spin: for omega = [0, 0, w] both the centripetal and
  the tangential term are independent of r_z. Only (r_x, r_y) are reported.

Usage:
    python3 calibrate_imu_motion.py --duration 25
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# A spin whose second singular value is below this fraction of the first is a
# single-axis rotation, so everything but the axis direction is unobservable.
SINGLE_AXIS_RATIO = 0.05


@dataclass
class IMUSample:
    stamp: float
    gyro: np.ndarray  # [wx, wy, wz] rad/s
    accel: np.ndarray  # [ax, ay, az] m/s^2


@dataclass
class OdomSample:
    stamp: float
    angular_z: float  # wz rad/s
    linear_x: float  # vx m/s


@dataclass
class LidarOdomSample:
    stamp: float
    gyro: np.ndarray  # [wx, wy, wz] rad/s
    linear_vel: np.ndarray  # [vx, vy, vz] m/s


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    K = skew(axis)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    m = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    trace = np.trace(m)
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def euler_from_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def align_vectors(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Minimal rotation taking unit vector `src` onto unit vector `dst`.

    This is the ONLY rotation a single-axis spin supports. It carries no
    information about rotation around the common axis, and by construction
    introduces none.
    """
    src = src / np.linalg.norm(src)
    dst = dst / np.linalg.norm(dst)
    axis = np.cross(src, dst)
    s = np.linalg.norm(axis)
    c = float(np.dot(src, dst))
    if s < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3) + 2 * np.outer(src, src)
    return rodrigues(axis / s, math.atan2(s, c))


def dominant_axis(omega: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """Principal rotation axis of a set of angular velocity samples.

    Returns (unit axis, singular value ratio s2/s1, peak |omega|). The ratio is
    the observability figure: near 0 means a pure single-axis spin.
    """
    U, S, Vt = np.linalg.svd(omega, full_matrices=False)
    axis = Vt[0, :]
    # The SVD sign is arbitrary and a CW-then-CCW spin carries no preferred
    # direction, so the sign is resolved against a reference signal instead
    # (see resolve_sign). Getting this wrong flips the alignment by 180 deg.
    ratio = float(S[1] / S[0]) if S[0] > 0 else 0.0
    return axis, ratio, float(np.max(np.linalg.norm(omega, axis=1)))


def resolve_sign(t_ref: np.ndarray, proj_ref: np.ndarray,
                 t_other: np.ndarray, proj_other: np.ndarray) -> float:
    """+1 or -1: does `proj_other` turn the same way as `proj_ref`?"""
    lo, hi = max(t_ref[0], t_other[0]), min(t_ref[-1], t_other[-1])
    if hi <= lo:
        return 1.0
    grid = np.linspace(lo, hi, 500)
    a = np.interp(grid, t_ref, proj_ref)
    b = np.interp(grid, t_other, proj_other)
    return 1.0 if float(np.dot(a, b)) >= 0 else -1.0


def resample_uniform(t: np.ndarray, v: np.ndarray, rate: float) -> Tuple[np.ndarray, np.ndarray]:
    """Put an irregularly stamped signal on a uniform grid.

    /vectornav/imu on Botman arrives at 35-50 Hz with gaps up to 0.28 s, so
    neither integration nor differentiation may assume a fixed dt.
    """
    n = max(int((t[-1] - t[0]) * rate), 2)
    tu = np.linspace(t[0], t[-1], n)
    vu = np.column_stack([np.interp(tu, t, v[:, i]) for i in range(v.shape[1])])
    return tu, vu


def smooth(v: np.ndarray, window: int) -> np.ndarray:
    if window < 3:
        return v
    w = np.hanning(window)
    w /= w.sum()
    pad = window // 2
    out = np.empty_like(v)
    for i in range(v.shape[1]):
        padded = np.pad(v[:, i], pad, mode="edge")
        out[:, i] = np.convolve(padded, w, mode="same")[pad:pad + len(v)]
    return out


def resample_signals(
    t_ref: np.ndarray,
    v_ref: np.ndarray,
    t_target: np.ndarray,
    v_target: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_start = max(t_ref[0], t_target[0])
    t_end = min(t_ref[-1], t_target[-1])
    if t_end <= t_start:
        return np.array([]), np.array([]), np.array([])
    t_common = np.linspace(t_start, t_end, int((t_end - t_start) * 100))
    v_ref_interp = np.array([np.interp(t_common, t_ref, v_ref[:, i]) for i in range(v_ref.shape[1])]).T
    v_tgt_interp = np.array([np.interp(t_common, t_target, v_target[:, i]) for i in range(v_target.shape[1])]).T
    return t_common, v_ref_interp, v_tgt_interp


if HAVE_ROS2:
    class PassiveIMUMotionRecorder(Node):
        def __init__(
            self,
            imu_topic: str,
            odom_topic: str,
            lidar_odom_topic: str,
        ) -> None:
            super().__init__("passive_imu_recorder")
            self.imu_samples: List[IMUSample] = []
            self.odom_samples: List[OdomSample] = []
            self.lidar_samples: List[LidarOdomSample] = []

            # BEST_EFFORT on every subscription: it is compatible with both
            # RELIABLE and BEST_EFFORT publishers, whereas a RELIABLE
            # subscription silently receives nothing from a BEST_EFFORT one.
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=200,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )

            self.sub_imu = self.create_subscription(Imu, imu_topic, self._on_imu, qos)
            self.sub_odom = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
            self.sub_lidar = self.create_subscription(Odometry, lidar_odom_topic, self._on_lidar_odom, qos)

        def _on_imu(self, msg: Imu) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.imu_samples.append(IMUSample(stamp=t, gyro=gyro, accel=accel))

        def _on_odom(self, msg: Odometry) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.odom_samples.append(
                OdomSample(
                    stamp=t,
                    angular_z=msg.twist.twist.angular.z,
                    linear_x=msg.twist.twist.linear.x,
                )
            )

        def _on_lidar_odom(self, msg: Odometry) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gyro = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
            vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            self.lidar_samples.append(LidarOdomSample(stamp=t, gyro=gyro, linear_vel=vel))


def analyze_static_phase(imu_samples: List[IMUSample]) -> Tuple[float, float, np.ndarray, np.ndarray]:
    if len(imu_samples) < 20:
        raise ValueError(f"Insufficient static IMU samples ({len(imu_samples)} samples). Ensure /vectornav/imu is publishing.")

    acc_arr = np.array([s.accel for s in imu_samples])
    gyro_arr = np.array([s.gyro for s in imu_samples])

    g_vec = np.mean(acc_arr, axis=0)
    g_norm = np.linalg.norm(g_vec)
    gyro_bias = np.mean(gyro_arr, axis=0)
    acc_noise = np.std(acc_arr, axis=0)

    up_imu = g_vec / g_norm
    pitch_static = math.atan2(-up_imu[0], math.sqrt(up_imu[1]**2 + up_imu[2]**2))
    roll_static = math.atan2(up_imu[1], up_imu[2])

    print("  Gravity Vector (stationary): [%.4f, %.4f, %.4f] m/s^2 (|g| = %.3f m/s^2)" % (g_vec[0], g_vec[1], g_vec[2], g_norm))
    print("  Accelerometer noise (1 sd):  [%.4f, %.4f, %.4f] m/s^2" % tuple(acc_noise))
    print("  Gyroscope Static Bias:       [%.6f, %.6f, %.6f] rad/s" % (gyro_bias[0], gyro_bias[1], gyro_bias[2]))
    print("  Static Tilt relative to g:   Roll = %.2f deg (%.4f rad), Pitch = %.2f deg (%.4f rad)" % (
        math.degrees(roll_static), roll_static, math.degrees(pitch_static), pitch_static
    ))
    tilt = math.degrees(math.acos(min(1.0, abs(up_imu[2]))))
    print("  Total tilt from vertical:    %.2f deg  ->  gravity sweeps +/-%.3f m/s^2 through the" % (
        tilt, g_norm * math.sin(math.radians(tilt))))
    print("                               IMU x/y axes during an in-place spin. This is removed")
    print("                               by de-rotating gravity, not by subtracting a constant.")
    return roll_static, pitch_static, g_vec, gyro_bias


def estimate_lever_arm(
    stamps: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    g_static: np.ndarray,
) -> Optional[dict]:
    """Solve accel - g(t) = ([alpha]x + [omega]x^2) r + b for the lever arm r.

    Two things make this different from regressing a_x on w_z^2:

    1. Gravity is de-rotated. The chassis spins about ITS OWN z axis, which is
       tilted about 1.2 deg from vertical on Botman, so gravity sweeps a cone
       through the IMU x/y axes with an amplitude near 0.2 m/s^2. The true
       centripetal signal at 0.4 rad/s and r = 0.2 m is only 0.03 m/s^2, so the
       artefact is roughly seven times the signal.
    2. A bias intercept b is fitted. Without it, any DC residual c is divided by
       mean(w^2) and reported as a lever arm: at 0.4 rad/s the gain is
       1 / 0.16 = 6.25 m per m/s^2, which is how a few centimetres of true
       offset came out as 1.125 m.
    """
    if len(stamps) < 50:
        return None

    dt_med = float(np.median(np.diff(stamps)))
    if dt_med <= 0:
        return None
    rate = 1.0 / dt_med
    t, gyro_u = resample_uniform(stamps, gyro, rate)
    _, accel_u = resample_uniform(stamps, accel, rate)
    n = len(t)

    win = max(3, int(round(0.3 * rate)) | 1)
    gyro_s = smooth(gyro_u, win)
    alpha = np.gradient(gyro_s, t, axis=0)

    # Integrate the gyro to track where gravity has rotated to in the IMU frame.
    g_seq = np.empty((n, 3))
    R = np.eye(3)
    dt = np.gradient(t)
    for i in range(n):
        theta = gyro_u[i] * dt[i]
        ang = float(np.linalg.norm(theta))
        if ang > 1e-12:
            R = R @ rodrigues(theta / ang, ang)
        g_seq[i] = R.T @ g_static

    A = np.zeros((3 * n, 6))
    b = np.zeros(3 * n)
    for i in range(n):
        A[3 * i:3 * i + 3, :3] = skew(alpha[i]) + skew(gyro_s[i]) @ skew(gyro_s[i])
        A[3 * i:3 * i + 3, 3:] = np.eye(3)
        b[3 * i:3 * i + 3] = accel_u[i] - g_seq[i]

    sol, _, _, sv = np.linalg.lstsq(A, b, rcond=None)
    r_fit = sol[:3]
    bias_fit = sol[3:]

    residual = b - A @ sol
    sigma2 = float(residual @ residual) / max(1, 3 * n - 6)
    cov = sigma2 * np.linalg.pinv(A.T @ A)
    stderr = np.sqrt(np.clip(np.diag(cov), 0.0, None))

    # Per-component observability: the column norm of A tells us how much
    # excitation each element of r actually received.
    col_energy = np.linalg.norm(A[:, :3], axis=0)
    observable = col_energy > 0.02 * max(col_energy.max(), 1e-12)

    return {
        "r": r_fit,
        "stderr": stderr[:3],
        "accel_bias": bias_fit,
        "observable": observable,
        "singular_values": sv,
        "residual_rms": math.sqrt(sigma2),
        "peak_omega": float(np.max(np.linalg.norm(gyro_u, axis=1))),
    }


def report_axis_alignment(
    label: str,
    axis_src: np.ndarray,
    axis_dst: np.ndarray,
    ratio: float,
    results: dict,
    key: str,
) -> None:
    R = align_vectors(axis_src, axis_dst)
    roll, pitch, yaw = euler_from_matrix(R)
    results[f"R_{key}"] = R
    results[f"q_{key}"] = quaternion_from_matrix(R)
    results[f"rpy_{key}"] = (roll, pitch, yaw)
    results[f"axis_{key}"] = axis_src
    results[f"axis_ratio_{key}"] = ratio

    print(f"\n  [Solved] {label}")
    print("    Spin axis in source frame: [%.5f, %.5f, %.5f]" % tuple(axis_src))
    print("    Spin axis in target frame: [%.5f, %.5f, %.5f]" % tuple(axis_dst))
    tilt = math.degrees(math.acos(float(np.clip(np.dot(
        axis_src / np.linalg.norm(axis_src), axis_dst / np.linalg.norm(axis_dst)), -1, 1))))
    print("    Angle between the axes:    %.2f deg  (decomposition-free; trust this one)" % tilt)
    print("    Minimal alignment RPY:     Roll=%.2f deg, Pitch=%.2f deg, Yaw=%.2f deg" % (
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
    if tilt > 150.0:
        print("    NOTE: the axes are nearly opposed, so the ZYX readout above splits one")
        print("          flip across roll and yaw. The matrix is right; read the angle instead.")
    print("    Quaternion [x, y, z, w]:   [%.6f, %.6f, %.6f, %.6f]" % tuple(results[f"q_{key}"]))
    if ratio < SINGLE_AXIS_RATIO:
        print("    NOTE: single-axis motion (s2/s1 = %.4f). Roll and pitch above are measured;" % ratio)
        print("          yaw is 0 by construction and is NOT observable from this run. Add a")
        print("          non-parallel rotation (pitch or roll the robot) to determine it.")
    else:
        print("    Motion excited a second axis (s2/s1 = %.3f), so yaw is partially constrained." % ratio)


def analyze_motion_phase(
    imu_samples: List[IMUSample],
    odom_samples: List[OdomSample],
    lidar_samples: List[LidarOdomSample],
    g_vec: np.ndarray,
    gyro_bias: np.ndarray,
) -> dict:
    if len(imu_samples) < 50:
        raise ValueError("Insufficient motion IMU samples.")

    t0 = imu_samples[0].stamp
    t_imu = np.array([s.stamp - t0 for s in imu_samples])
    gyro_motion = np.array([s.gyro for s in imu_samples]) - gyro_bias
    acc_motion = np.array([s.accel for s in imu_samples])

    results: dict = {}

    axis_imu, ratio_imu, peak = dominant_axis(gyro_motion)
    duration = t_imu[-1] - t_imu[0]
    print("\n  Motion summary: %d IMU samples over %.1f s (%.1f Hz), peak |omega| = %.3f rad/s" % (
        len(imu_samples), duration, len(imu_samples) / max(duration, 1e-6), peak))
    print("  Rotation axis in IMU frame: [%.5f, %.5f, %.5f]  (s2/s1 = %.4f)" % (*axis_imu, ratio_imu))
    if peak < 0.15:
        print("  [WARN] Peak rotation rate is low. Spin at 0.3-0.5 rad/s for a usable lever arm.")

    # IMU <-> Base. /odom carries only angular.z, so the base-frame axis is
    # known a priori to be +z; there is nothing to fit with an SVD, and running
    # Kabsch on collinear targets returns an arbitrary yaw from its null space.
    proj_imu = gyro_motion @ axis_imu
    if len(odom_samples) > 20:
        t_odom = np.array([s.stamp - t0 for s in odom_samples])
        wz = np.array([s.angular_z for s in odom_samples])
        if np.max(np.abs(wz)) > 0.05:
            # Point the IMU axis the same way the chassis actually turned, so
            # +axis_imu corresponds to +z of base_link rather than -z.
            flip = resolve_sign(t_imu, proj_imu, t_odom, wz)
            axis_imu = flip * axis_imu
            proj_imu = flip * proj_imu
            report_axis_alignment(
                "base_link <- vectornav (from wheel odometry)",
                axis_imu, np.array([0.0, 0.0, 1.0]), ratio_imu, results, "base_imu")
        else:
            print("\n  [SKIP] Wheel odometry reported no rotation; cannot align to base_link.")
    else:
        print("\n  [SKIP] Too few wheel odometry samples (%d); IMU axis sign is unresolved,"
              % len(odom_samples))
        print("         so the alignment below may be flipped by 180 deg.")

    # IMU <-> LiDAR.
    if len(lidar_samples) > 20:
        t_lidar = np.array([s.stamp - t0 for s in lidar_samples])
        w_lidar = np.array([s.gyro for s in lidar_samples])
        lidar_rate = len(lidar_samples) / max(t_lidar[-1] - t_lidar[0], 1e-6)
        axis_lidar, ratio_lidar, peak_l = dominant_axis(w_lidar)
        print("\n  LiDAR odometry: %d samples over %.1f s (%.2f Hz), peak |omega| = %.3f rad/s" % (
            len(lidar_samples), t_lidar[-1] - t_lidar[0], lidar_rate, peak_l))
        if lidar_rate < 5.0:
            print("  [WARN] /laser_odometry is only %.2f Hz. A 0.4 rad/s spin is badly undersampled at" % lidar_rate)
            print("         this rate, and the interpolation to 100 Hz below invents the samples in")
            print("         between. Treat the IMU-to-LiDAR result as indicative only.")
        results["lidar_odom_rate"] = lidar_rate
        # Both axes come from independent SVDs whose signs are arbitrary; without
        # this the two can disagree and produce a spurious 180 deg alignment.
        axis_lidar = resolve_sign(t_imu, proj_imu, t_lidar, w_lidar @ axis_lidar) * axis_lidar
        report_axis_alignment(
            "vectornav <- os_lidar", axis_lidar, axis_imu, min(ratio_imu, ratio_lidar), results, "imu_laser")
    else:
        print("\n  [SKIP] Too few LiDAR odometry samples (%d)." % len(lidar_samples))

    # Lever arm.
    lever = estimate_lever_arm(t_imu, gyro_motion, acc_motion, g_vec)
    if lever is None:
        print("\n  [SKIP] Not enough motion samples for a lever arm fit.")
        return results

    results["lever_arm_fit"] = lever
    r = lever["r"]
    se = lever["stderr"]
    obs = lever["observable"]
    results["lever_arm"] = (float(r[0]), float(r[1]))

    print("\n  [Solved] IMU offset from the rotation axis (lever arm):")
    for i, name in enumerate("xyz"):
        if obs[i]:
            print("    r_%s = %+.3f m  +/- %.3f m (1 sigma)" % (name, r[i], se[i]))
        else:
            print("    r_%s : UNOBSERVABLE from a yaw-only spin (no excitation in this direction)" % name)
    print("    Fitted accelerometer bias: [%+.4f, %+.4f, %+.4f] m/s^2" % tuple(lever["accel_bias"]))
    print("    Residual RMS: %.4f m/s^2" % lever["residual_rms"])
    reach = float(np.linalg.norm(r[:2]))
    if reach > 0.8:
        print("    [WARN] %.2f m exceeds the Bunker footprint. Suspect a tilted spin (gravity leak)," % reach)
        print("           too slow a spin, or the robot translating instead of turning in place.")
    print("    Sanity: at peak |omega| = %.2f rad/s this offset produces only %.3f m/s^2 of" % (
        lever["peak_omega"], lever["peak_omega"] ** 2 * reach))
    print("            centripetal acceleration. Compare it against the accelerometer noise above.")

    return results


def run_interactive_imu_calibration(
    imu_topic: str = "/vectornav/imu",
    odom_topic: str = "/odom",
    lidar_odom_topic: str = "/laser_odometry",
    motion_duration: float = 25.0,
) -> Optional[dict]:
    if not HAVE_ROS2:
        print("ERROR: ROS 2 (rclpy, sensor_msgs, nav_msgs) is required.", file=sys.stderr)
        return None

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = PassiveIMUMotionRecorder(
        imu_topic=imu_topic,
        odom_topic=odom_topic,
        lidar_odom_topic=lidar_odom_topic,
    )

    try:
        print("\n" + "=" * 60)
        print(" PHASE 1: STATIC GRAVITY & BIAS CAPTURE")
        print("=" * 60)
        print("--> Keep the robot COMPLETELY STATIONARY on level ground.")
        print(f"--> Listening to IMU topic: {imu_topic}")
        input("--> Press [ENTER] when the robot is stationary to record static baseline (5s)... ")

        node.imu_samples.clear()
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            rem = 5.0 - (time.time() - start_t)
            sys.stdout.write(f"\rRecording static IMU data: {rem:.1f}s remaining | IMU samples: {len(node.imu_samples)}... ")
            sys.stdout.flush()
        print()

        if len(node.imu_samples) == 0:
            print(f"\n[ERROR] 0 samples received on '{imu_topic}'.", file=sys.stderr)
            print("Possible causes:", file=sys.stderr)
            print("  1. The VectorNav driver is not running. (Ensure vectornav node is started)", file=sys.stderr)
            print("  2. ROS_DOMAIN_ID mismatch (Botman uses ROS_DOMAIN_ID=17).", file=sys.stderr)
            return None

        roll_s, pitch_s, g_vec, gyro_bias = analyze_static_phase(node.imu_samples)

        print("\n" + "=" * 60)
        print(" PHASE 2: MANUAL IN-PLACE ROTATION")
        print("=" * 60)
        print("--> Get ready with your remote control / joystick.")
        print(f"--> When you press [ENTER], rotate the robot IN PLACE for ~{int(motion_duration)}s.")
        print("    (Tip: Rotate CW for ~10s, pause briefly, then CCW for ~10s at moderate speed ~0.3-0.5 rad/s)")
        print("    Turn in place: any translation adds acceleration the lever arm fit cannot separate.")
        input("--> Press [ENTER] to start recording motion... ")

        node.imu_samples.clear()
        node.odom_samples.clear()
        node.lidar_samples.clear()
        start_t = time.time()

        while rclpy.ok() and (time.time() - start_t) < motion_duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            rem = motion_duration - (time.time() - start_t)
            sys.stdout.write(
                f"\rRecording spin: {rem:4.1f}s left | IMU: {len(node.imu_samples):4d} | Encoders: {len(node.odom_samples):3d} | LiDAR: {len(node.lidar_samples):3d} "
            )
            sys.stdout.flush()
        print("\nMotion recording complete. Stop rotating the robot.")

        results = analyze_motion_phase(node.imu_samples, node.odom_samples, node.lidar_samples, g_vec, gyro_bias)
        results["static_tilt"] = (roll_s, pitch_s)
        results["gravity_vec"] = g_vec
        results["gyro_bias"] = gyro_bias
        return results
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--imu-topic", default="/vectornav/imu", help="IMU topic name")
    parser.add_argument("--odom-topic", default="/odom", help="Wheel odometry topic")
    parser.add_argument("--lidar-odom-topic", default="/laser_odometry", help="LiDAR odometry topic")
    parser.add_argument("--duration", type=float, default=25.0, help="Rotation recording duration (seconds)")
    args = parser.parse_args()

    run_interactive_imu_calibration(
        imu_topic=args.imu_topic,
        odom_topic=args.odom_topic,
        lidar_odom_topic=args.lidar_odom_topic,
        motion_duration=args.duration,
    )


if __name__ == "__main__":
    main()
