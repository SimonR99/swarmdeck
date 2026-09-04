#!/usr/bin/env python3
"""Measure an IMU's static bias and noise densities for a SuperOdometry config.

WHY THIS EXISTS
  Three numbers in every *_superodom.yaml have to come from the specific device
  the robot carries, and until now they were produced by hand each time:

    g_norm  what THIS accelerometer reads when static. NOT true local gravity.
            GTSAM builds its gravity vector from it
            (PreintegrationParams::MakeSharedU) and separately estimates
            accelerometer bias; a standing offset between the two has to be
            carried by the bias state, failureDetection() trips at
            ba.norm() > 2.0, and with acc_w at 6.4e-05 the bias cannot track it
            before the solve runs away. Established by experiment on aslan
            2026-09-03: the physically correct local value (9.8064, Somigliana
            on WGS84 for Polytechnique Montreal) made imu_preintegration reset
            1.67 times a second, with yaw drifting +7.43 deg/min and position
            0.135 m/min while the robot sat still. The device's own reading
            gave 0 resets, +0.015 deg/min and 0.0007 m/min.

            The offset is device-specific, so it can never be borrowed from
            another unit: the two OS0-64s in this fleet disagree by 1.75 %.

    acc_n   accelerometer and gyro noise densities, read as the overlapping
    gyr_n   Allan deviation at tau = 1 s, RMS across the three axes. The values
            these replace were an upstream sample config shared byte-identically
            across every robot and both sensor models, and overstated gyro noise
            by 4x on the Ouster and 24x on the VN-100. Overstating gyro noise
            makes the estimator discount a gyro that is better than advertised,
            which shows up as attitude wander.

  acc_w and gyr_w (the bias random walks) are NOT estimated here: they need the
  Allan deviation's rising right-hand slope, which a ten-minute log does not
  reach. The existing values are kept.

SAFETY: PASSIVE. It never commands the robot. It only subscribes.

HOW TO RUN
  The robot must be powered, level and COMPLETELY still for the whole window:
  no one leaning on the chassis, no fans or motors starting, no driving. Ten
  minutes is the length the fleet's other measurements used.

      python3 scripts/calibration/measure_imu_static.py \
          --topic /vectornav/imu --duration 600

  Paste the reported block into the robot's SuperOdometry config, for example
  adapters/adapter_ros2/config/aslan_superodom.yaml.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False


if HAVE_ROS2:
    class StaticImuRecorder(Node):
        def __init__(self, topic: str) -> None:
            super().__init__("static_imu_recorder")
            self.samples: List[Tuple[float, np.ndarray, np.ndarray]] = []
            # BEST_EFFORT with a deep queue: IMU drivers publish best-effort,
            # and a 600 s log at 100 Hz is 60k samples that must not be dropped
            # by a shallow queue while the callback runs.
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2000,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Imu, topic, self._on, qos)

        def _on(self, msg: Imu) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append((
                t,
                np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z]),
                np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z]),
            ))


def allan_deviation_at(data: np.ndarray, rate: float, tau: float = 1.0) -> np.ndarray:
    """Overlapping Allan deviation at one averaging time, per axis.

    data is (N, 3) of raw samples. The overlapping estimator is used rather than
    the non-overlapping one because it uses every available cluster pair and so
    has far lower variance at a given tau for the same log length.
    """
    n = data.shape[0]
    m = int(round(tau * rate))
    if m < 1 or n < 2 * m + 1:
        raise ValueError(
            f"log too short for tau={tau}s: need > {2 * m + 1} samples at "
            f"{rate:.1f} Hz, have {n}"
        )
    # theta is the integrated angle/velocity, so a cluster average is a simple
    # difference of two of its entries.
    theta = np.cumsum(data, axis=0) / rate
    theta = np.vstack([np.zeros((1, data.shape[1])), theta])
    k = np.arange(0, theta.shape[0] - 2 * m)
    diff = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k]
    return np.sqrt(np.sum(diff ** 2, axis=0) / (2.0 * (tau ** 2) * len(k)))


def analyze(samples: List[Tuple[float, np.ndarray, np.ndarray]], tau: float) -> dict:
    stamps = np.array([s[0] for s in samples])
    accel = np.array([s[1] for s in samples])
    gyro = np.array([s[2] for s in samples])

    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span if span > 0 else float("nan")

    mean_accel = accel.mean(axis=0)
    # The mean of the norms, not the norm of the mean: it is the magnitude each
    # individual sample reports that GTSAM has to reconcile.
    g_norm = float(np.linalg.norm(accel, axis=1).mean())
    gyro_bias = gyro.mean(axis=0)

    acc_dev = allan_deviation_at(accel, rate, tau)
    gyr_dev = allan_deviation_at(gyro, rate, tau)
    # RMS across axes, matching how the fleet's existing figures were reduced.
    acc_n = float(np.sqrt(np.mean(acc_dev ** 2)))
    gyr_n = float(np.sqrt(np.mean(gyr_dev ** 2)))

    return {
        "count": len(stamps), "span": span, "rate": rate,
        "mean_accel": mean_accel, "g_norm": g_norm, "gyro_bias": gyro_bias,
        "acc_dev": acc_dev, "gyr_dev": gyr_dev, "acc_n": acc_n, "gyr_n": gyr_n,
        "accel_sd": accel.std(axis=0), "gyro_sd": gyro.std(axis=0),
    }


def report(topic: str, res: dict, tau: float) -> None:
    print()
    print("=" * 70)
    print(" STATIC IMU MEASUREMENT: %s" % topic)
    print("=" * 70)
    print("  samples          : %d over %.1f s (%.2f Hz)"
          % (res["count"], res["span"], res["rate"]))
    print("  mean accel       : [%.4f, %.4f, %.4f] m/s^2"
          % tuple(res["mean_accel"]))
    print("  accel 1 sd       : [%.4f, %.4f, %.4f] m/s^2" % tuple(res["accel_sd"]))
    print("  gyro bias        : [%.2e, %.2e, %.2e] rad/s" % tuple(res["gyro_bias"]))
    print("  gyro bias        : %.4f deg/s"
          % float(np.degrees(np.linalg.norm(res["gyro_bias"]))))
    print("  Allan dev @%gs   : accel [%.3e, %.3e, %.3e]"
          % (tau, *res["acc_dev"]))
    print("                     gyro  [%.3e, %.3e, %.3e]" % tuple(res["gyr_dev"]))
    print()

    # Sanity: a static log whose acceleration wanders is not static.
    worst_sd = float(np.max(res["accel_sd"]))
    if worst_sd > 0.05:
        print("  [WARN] accel 1 sd is %.4f m/s^2, which is high for a static log."
              % worst_sd)
        print("         Something moved: someone leaning on the chassis, a fan or")
        print("         motor starting, or the robot not settled. Re-record.")
    tilt = float(np.degrees(np.arctan2(
        np.linalg.norm(res["mean_accel"][:2]), abs(res["mean_accel"][2]))))
    if tilt > 5.0:
        print("  [WARN] the sensor is %.1f deg off level. That does not affect" % tilt)
        print("         g_norm, which is a magnitude, but check it is expected.")
    print()
    print("  Paste into the robot's SuperOdometry config:")
    print()
    print("    imu_preintegration_node:")
    print("        acc_n: %.6e" % res["acc_n"])
    print("        gyr_n: %.6e" % res["gyr_n"])
    print("        # acc_w / gyr_w are not measured here; keep the existing values.")
    print("        g_norm: %.4f" % res["g_norm"])
    print()
    print("  g_norm is this device's own static reading and is NOT transferable")
    print("  to another unit, even the same model. See the note in the config.")
    print("=" * 70)


def run(topic: str, duration: float, tau: float, no_prompt: bool) -> dict:
    if not HAVE_ROS2:
        print("ERROR: rclpy is not importable. Run this inside a sourced ROS 2 "
              "environment on the robot.", file=sys.stderr)
        raise SystemExit(2)

    if not no_prompt:
        print("The robot must be powered, level and COMPLETELY still for the next")
        print("%.0f s: no leaning on the chassis, no driving, no motors starting."
              % duration)
        input("--> Press [ENTER] to start recording... ")

    owns = not rclpy.ok()
    if owns:
        rclpy.init()
    node = StaticImuRecorder(topic)
    try:
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            remaining = deadline - time.time()
            sys.stdout.write("\rRecording %s: %6.1f s left | %d samples... "
                             % (topic, max(remaining, 0.0), len(node.samples)))
            sys.stdout.flush()
        print()

        if len(node.samples) < 100:
            print("ERROR: only %d samples on %s. Is the driver running and "
                  "publishing?" % (len(node.samples), topic), file=sys.stderr)
            raise SystemExit(1)

        res = analyze(node.samples, tau)
        report(topic, res, tau)
        return res
    finally:
        node.destroy_node()
        if owns and rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/vectornav/imu",
                    help="IMU topic to record (default: /vectornav/imu)")
    ap.add_argument("--duration", type=float, default=600.0,
                    help="Recording length in seconds (default: 600, as used "
                         "for the fleet's existing figures)")
    ap.add_argument("--tau", type=float, default=1.0,
                    help="Allan deviation averaging time in seconds (default: 1)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="Start recording immediately without waiting for enter")
    a = ap.parse_args()
    run(a.topic, a.duration, a.tau, a.no_prompt)


if __name__ == "__main__":
    main()
