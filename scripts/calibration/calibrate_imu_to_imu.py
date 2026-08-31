#!/usr/bin/env python3
"""Measure the rotation between two IMUs, so SuperOdometry can use the VectorNav.

WHY THIS EXISTS
  SuperOdometry needs imu^R_laser: the rotation from os_lidar into whichever IMU
  it fuses. For the Ouster's own IMU that is exact and free, because the sensor
  publishes it (see ouster_imu_to_lidar below). For the VN-100 it is not known
  by anyone: there is no base_link -> vectornav TF anywhere in this repo, and an
  in-place yaw spin cannot recover it. A spin about the vertical excites ONE
  axis, which fixes the axis direction (roll and pitch) and leaves rotation
  about that axis (yaw) completely free.

  So this script asks for motion about a DIFFERENT axis and compares the two
  gyros directly. Two non-parallel axes are enough; three are not needed:

      omega_vectornav(t) = R * omega_ouster(t)

  Both sensors are rigid on the same chassis, so R is the mounting rotation.
  Both publish at high rate (200 Hz and ~100 Hz), which is why this works in a
  few seconds where LiDAR odometry at 3 Hz would not.

  R is then composed with the Ouster's factory os_lidar -> os_imu transform to
  give the os_lidar -> vectornav rotation SuperOdometry actually wants.

SAFETY: PASSIVE. It never commands the robot. The motion is you rocking the
chassis by hand: press down on a front corner and release, a few times, then a
side corner. No driving is required and none is asked for.

Usage:
    python3 calibrate_imu_to_imu.py --duration 30
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# Ouster factory extrinsics, read from this sensor's own metadata API on
# 2026-08-27 (OS-0-64-U13, serial 122325000476):
#   imu_to_sensor_transform   R = I,           t = [ 6.253, -11.775,  7.645] mm
#   lidar_to_sensor_transform R = 180 deg yaw, t = [ 0,      0,      36.18 ] mm
# Composing them, os_lidar -> os_imu is a 180 degree yaw. Note that this is NOT
# identity: os_lidar is rotated 180 degrees from os_sensor by Ouster's own
# convention, and os1_128_calibration.yaml currently ships identity, which
# negates the x and y of every IMU sample fed to the estimator.
R_LIDAR_TO_OUSTER_IMU = np.diag([-1.0, -1.0, 1.0])
T_LIDAR_TO_OUSTER_IMU = np.array([-0.006253, 0.011775, 0.028535])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def solve_rotation_svd(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Kabsch: returns R with dst ~= R @ src."""
    U, _, Vt = np.linalg.svd(src.T @ dst)
    d = np.linalg.det(Vt.T @ U.T)
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


def euler_from_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy >= 1e-6:
        return math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0


if HAVE_ROS2:
    class DualImuRecorder(Node):
        def __init__(self, topic_a: str, topic_b: str) -> None:
            super().__init__("dual_imu_recorder")
            self.a: List[Tuple[float, np.ndarray]] = []
            self.b: List[Tuple[float, np.ndarray]] = []
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=500,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Imu, topic_a, lambda m: self._on(m, self.a), qos)
            self.create_subscription(Imu, topic_b, lambda m: self._on(m, self.b), qos)

        @staticmethod
        def _on(msg: Imu, sink: List[Tuple[float, np.ndarray, np.ndarray]]) -> None:
            # Accelerometers are kept as well as gyros: the rotation comes from
            # the gyros, but only the accelerometers can see the translation.
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            sink.append((t, np.array([msg.angular_velocity.x,
                                      msg.angular_velocity.y,
                                      msg.angular_velocity.z]),
                         np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])))


def _resample(t_ref: np.ndarray, t_src: np.ndarray, v_src: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(t_ref, t_src, v_src[:, i]) for i in range(3)])


def estimate_time_offset(ta: np.ndarray, va: np.ndarray,
                         tb: np.ndarray, vb: np.ndarray,
                         max_offset: float = 0.25) -> float:
    """Align the two streams by cross-correlating |omega|.

    The two drivers stamp independently; tens of milliseconds of skew rotates
    the fit during fast motion, which is exactly the motion this needs.
    """
    lo, hi = max(ta[0], tb[0]) + max_offset, min(ta[-1], tb[-1]) - max_offset
    if hi <= lo:
        return 0.0
    grid = np.arange(lo, hi, 0.005)
    na = np.linalg.norm(_resample(grid, ta, va), axis=1)
    best, best_score = 0.0, -np.inf
    for off in np.arange(-max_offset, max_offset, 0.005):
        nb = np.linalg.norm(_resample(grid + off, tb, vb), axis=1)
        score = float(np.dot(na - na.mean(), nb - nb.mean()))
        if score > best_score:
            best_score, best = score, float(off)
    return best


def solve_imu_rotation(ta: np.ndarray, va: np.ndarray,
                       tb: np.ndarray, vb: np.ndarray,
                       min_rate: float = 0.10) -> Optional[dict]:
    offset = estimate_time_offset(ta, va, tb, vb)
    lo, hi = max(ta[0], tb[0] - offset) + 0.05, min(ta[-1], tb[-1] - offset) - 0.05
    if hi <= lo:
        return None
    grid = np.arange(lo, hi, 0.005)
    wa = _resample(grid, ta, va)
    wb = _resample(grid + offset, tb, vb)

    # Only use samples where something is actually rotating: at rest the two
    # gyros carry nothing but bias and noise, and including them drags the fit
    # toward whatever the bias difference happens to look like.
    moving = np.linalg.norm(wb, axis=1) > 0.05
    if np.sum(moving) < 100:
        return None
    wa, wb = wa[moving], wb[moving]

    # TWO non-parallel axes fully determine a 3-D rotation: Kabsch fixes the
    # plane they span, and orthogonality plus det=+1 fixes the third axis. So
    # the figure of merit is the SECOND singular value, not the third. Testing
    # the third rejects a perfectly good pitch-then-roll rock, which recovers
    # the rotation to 0.01 deg in simulation.
    sv = np.linalg.svd(wb, compute_uv=False)
    ratio = float(sv[1] / sv[0]) if sv[0] > 0 else 0.0

    R = solve_rotation_svd(wb, wa)
    resid = wa - (R @ wb.T).T
    rms = float(np.sqrt((resid ** 2).sum(axis=1).mean()))
    return {
        "R": R, "time_offset": offset, "singular_values": sv, "axis_ratio": ratio,
        "residual_rms": rms, "n_samples": int(len(wa)),
        "observable": ratio >= min_rate,
    }


def solve_translation(ta: np.ndarray, wa: np.ndarray, aa: np.ndarray,
                      tb: np.ndarray, wb: np.ndarray, ab: np.ndarray,
                      R_ba: np.ndarray) -> Optional[dict]:
    """Solve the offset between two IMUs from their accelerometers.

    Gyros cannot see it, but accelerometers can: two points on a rigid body
    differ in specific force by the rigid-body terms, so in B's frame

        f_B - R_BA f_A = ([alpha_B]x + [omega_B]x^2) r + b

    with r the position of B relative to A expressed in B. This is linear in r.

    The bias column b is what makes it usable. Any error in R_BA leaks gravity
    into the left-hand side, and at 1 deg that leak is 0.17 m/s^2 against a
    signal of order omega^2 * r, which for 0.5 rad/s and 0.1 m is 0.025 m/s^2 --
    seven times smaller. But while the spin axis is vertical the leak is
    CONSTANT in the body frame, whereas the rigid-body terms vary with omega^2
    and alpha, so fitting b separates them. Measured on Botman, perturbing R_BA
    by 1 deg moves r by 0.8-1.5 mm.

    Spin fast (0.3-0.5 rad/s or better) and reverse direction: the signal grows
    with omega^2, and reversing flips the sign of alpha but not of omega^2,
    which is what separates the two terms.
    """
    if len(ta) < 200 or len(tb) < 200:
        return None
    lo, hi = max(ta[0], tb[0]), min(ta[-1], tb[-1])
    sel = (tb >= lo) & (tb <= hi)
    t = tb[sel]
    if len(t) < 200:
        return None
    w = wb[sel]
    f_b = ab[sel]
    f_a = _resample(t, ta, aa)

    alpha = np.stack([np.gradient(w[:, k], t) for k in range(3)], axis=1)
    ker = np.ones(9) / 9.0
    alpha = np.stack([np.convolve(alpha[:, c], ker, mode="same") for c in range(3)], axis=1)

    y = f_b - (R_ba @ f_a.T).T
    rows = np.empty((len(t) * 3, 6))
    for i in range(len(t)):
        M = _skew(alpha[i]) + _skew(w[i]) @ _skew(w[i])
        rows[3 * i:3 * i + 3, :3] = M
        rows[3 * i:3 * i + 3, 3:] = np.eye(3)
    sol, _, rank, _ = np.linalg.lstsq(rows, y.reshape(-1), rcond=None)
    resid = rows @ sol - y.reshape(-1)

    # Observability, and why the covariance cannot supply it. The bias columns
    # are a constant per axis, so fitting b is exactly removing the mean of the
    # rigid-body block; whatever variation is left is all that identifies r.
    # Centre the block and take its singular values. For a spin about a nearly
    # vertical axis the r_z direction barely varies, so its singular value
    # collapses and that component is not measured however many samples are
    # averaged. np.linalg.pinv hides this: it reports a ~0 standard error for a
    # direction it has silently dropped, which is worse than no number at all.
    block = rows[:, :3].reshape(-1, 3, 3)
    centred = (block - block.mean(axis=0)).reshape(-1, 3)
    sv = np.linalg.svd(centred, compute_uv=False)
    observability = sv / sv[0] if sv[0] > 0 else np.zeros(3)
    return {
        "r": sol[:3],
        "bias": sol[3:],
        "rms": float(np.sqrt((resid ** 2).mean())),
        "rank": int(rank),
        "singular_values": sv,
        "observability": observability,
        "peak_omega": float(np.linalg.norm(w, axis=1).max()),
    }


def emit_calibration_yaml(R_lidar_imu: np.ndarray, t_lidar_imu: np.ndarray) -> str:
    rows = ",\n        ".join(
        ", ".join(f"{v: .8f}" for v in row) for row in R_lidar_imu)
    return f"""%YAML:1.0

# Rotation from laser frame to imu frame, imu^R_laser
extrinsicRotation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [{rows}]

# Translation from laser frame to imu frame, imu^T_laser
extrinsicTranslation_imu_laser: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [{t_lidar_imu[0]: .8f}, {t_lidar_imu[1]: .8f}, {t_lidar_imu[2]: .8f}]

imu_laser_rotation_offset: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [0.0, 0.0, 0.0]

yaw_ratio: 0.0
"""


def run(target_topic: str, reference_topic: str, duration: float) -> Optional[dict]:
    if not HAVE_ROS2:
        print("ERROR: ROS 2 (rclpy, sensor_msgs) is required.", file=sys.stderr)
        return None

    owns = not rclpy.ok()
    if owns:
        rclpy.init()
    node = DualImuRecorder(target_topic, reference_topic)
    try:
        print("=" * 66)
        print(" IMU-TO-IMU ROTATION  (%s  <-  %s)" % (target_topic, reference_topic))
        print("=" * 66)
        print("--> This needs rotation about a NON-VERTICAL axis. A yaw spin will not do:")
        print("    it leaves the rotation about the spin axis unobservable, which is the")
        print("    whole quantity we are missing.")
        print("--> With the robot stationary and powered, ROCK THE CHASSIS BY HAND:")
        print("      1. press down on a front corner and release, ~5 times")
        print("      2. press down on a side corner and release, ~5 times")
        print("      3. if you can, lift one side a few cm and set it down")
        print("    Firm and brisk beats gentle; the fit wants real angular rate.")
        input(f"--> Press [ENTER] to record for {int(duration)}s... ")

        node.a.clear()
        node.b.clear()
        start = time.time()
        while rclpy.ok() and time.time() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            rem = duration - (time.time() - start)
            sys.stdout.write(f"\rRecording: {rem:4.1f}s left | {target_topic}: {len(node.a):5d}"
                             f" | {reference_topic}: {len(node.b):5d} ")
            sys.stdout.flush()
        print()

        if len(node.a) < 100 or len(node.b) < 100:
            print(f"[ERROR] too few samples ({len(node.a)} / {len(node.b)}). Are both topics live?",
                  file=sys.stderr)
            return None

        ta = np.array([s[0] for s in node.a]); va = np.array([s[1] for s in node.a])
        tb = np.array([s[0] for s in node.b]); vb = np.array([s[1] for s in node.b])
        aa = np.array([s[2] for s in node.a]); ab = np.array([s[2] for s in node.b])
        res = solve_imu_rotation(ta, va, tb, vb)
        if res is None:
            print("[ERROR] not enough overlapping motion to solve.", file=sys.stderr)
            return None

        R = res["R"]
        r, p, y = [math.degrees(v) for v in euler_from_matrix(R)]
        print("\n  Samples used      : %d" % res["n_samples"])
        print("  Stamp offset      : %+.3f s (estimated by cross-correlation)" % res["time_offset"])
        print("  Excitation        : singular values %s" % np.round(res["singular_values"], 3))
        print("  Second-axis ratio : %.3f  %s" % (
            res["axis_ratio"],
            "OK, a second axis was excited" if res["observable"]
            else "TOO LOW: this is still essentially single-axis motion"))
        print("  Residual RMS      : %.4f rad/s" % res["residual_rms"])
        print("\n  R (%s <- %s):" % (target_topic, reference_topic))
        for row in R:
            print("    [%9.5f, %9.5f, %9.5f]" % tuple(row))
        print("  RPY: roll=%.2f deg  pitch=%.2f deg  yaw=%.2f deg" % (r, p, y))

        if not res["observable"]:
            print("\n[REFUSING] Second-axis excitation %.3f is below 0.10, so the motion was"
                  % res["axis_ratio"], file=sys.stderr)
            print("           effectively about one axis and the yaw this script exists to", file=sys.stderr)
            print("           measure is not determined by it. Rock about a second axis.", file=sys.stderr)
            return res

        R_lidar_target = R @ R_LIDAR_TO_OUSTER_IMU
        # Comparing two GYROS gives the rotation and nothing else: angular
        # velocity is identical everywhere on a rigid body, so it says nothing
        # about where on that body either sensor sits. The ACCELEROMETERS do
        # carry it, which is what solve_translation() above exploits, so the
        # offset is measured rather than left at zero or guessed with a ruler.
        #
        # What is never emitted is R @ T_LIDAR_TO_OUSTER_IMU: that would hand
        # back the Ouster's own 3 cm internal lever arm dressed up in the target
        # frame, the mistake aslan_superodom_calibration.yaml warns against
        # ("that lever arm is the Ouster's internal IMU ... does not apply to a
        # separately mounted VN-100").
        # Gyros cannot see the offset, but the accelerometer pair can. Solve it
        # rather than defaulting to zero, then compose with the Ouster factory
        # os_lidar -> os_imu to express it from the lidar.
        trans = solve_translation(tb, vb, ab, ta, va, aa, R)
        if target_topic == reference_topic:
            t_lidar_target = T_LIDAR_TO_OUSTER_IMU
        elif trans is not None:
            res["translation_fit"] = trans
            r_target_ref = trans["r"]
            # os_lidar relative to the target IMU, in the target frame.
            t_lidar_target = R_lidar_target @ (-T_LIDAR_TO_OUSTER_IMU) - r_target_ref
        else:
            t_lidar_target = np.zeros(3)
        rr, pp, yy = [math.degrees(v) for v in euler_from_matrix(R_lidar_target)]
        print("\n  Composed with the Ouster's factory os_lidar -> os_imu (180 deg yaw):")
        print("  R (os_lidar -> %s): RPY roll=%.2f  pitch=%.2f  yaw=%.2f deg" % (
            target_topic, rr, pp, yy))
        print("\n" + "=" * 66)
        print(" SuperOdometry calibration file for imu_topic: %s" % target_topic)
        print("=" * 66)
        print(emit_calibration_yaml(R_lidar_target, t_lidar_target))
        if not np.any(t_lidar_target):
            tf = res.get("translation_fit")
            if tf is None:
                print("  NOTE: the translation is zero: the accelerometer solve did not converge.")
                print("        Spin the robot in place at 0.3-0.5 rad/s, reversing a few times.")
            else:
                print("\n  [Solved] translation from the accelerometer pair:")
                print("    target relative to reference: [%+.4f, %+.4f, %+.4f] m" % tuple(tf["r"]))
                obs = tf["observability"]
                print("    observability (1.0 = fully determined by this motion):")
                print("                                  [ %.3f,  %.3f,  %.3f]" % tuple(obs))
                if obs[-1] < 0.1:
                    print("    [WARN] the weakest direction is not measured by this motion.")
                    print("           A spin about a vertical axis cannot see the vertical")
                    print("           offset: both rigid-body terms vanish along the axis.")
                    print("           Rotate about a HORIZONTAL axis (pitch or roll the")
                    print("           sensor head) for that component, or take it from CAD.")
                print("    fitted bias (absorbs any gravity leak): [%+.3f, %+.3f, %+.3f] m/s^2"
                      % tuple(tf["bias"]))
                print("    peak |omega| %.3f rad/s, rms residual %.4f m/s^2, rank %d"
                      % (tf["peak_omega"], tf["rms"], tf["rank"]))
                if tf["peak_omega"] < 0.3:
                    print("    [WARN] peak rotation below 0.3 rad/s: the signal scales with")
                    print("           omega^2, so spin faster and re-run to tighten this.")
            print("        Measure os_lidar -> %s with a ruler and fill it in if the two" % target_topic)
            print("        are more than ~10 cm apart; the rotation above is the load-bearing part.")
        res["R_lidar_target"] = R_lidar_target
        res["t_lidar_target"] = t_lidar_target
        return res
    finally:
        node.destroy_node()
        if owns and rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="/vectornav/imu",
                    help="IMU whose mounting you want (default: /vectornav/imu)")
    ap.add_argument("--reference", default="/ouster/imu",
                    help="IMU whose lidar extrinsic is already known (default: /ouster/imu)")
    ap.add_argument("--duration", type=float, default=30.0)
    a = ap.parse_args()
    run(a.target, a.reference, a.duration)


if __name__ == "__main__":
    main()
