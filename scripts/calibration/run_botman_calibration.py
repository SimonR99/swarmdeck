#!/usr/bin/env python3
"""Interactive end-to-end calibration orchestrator for Botman.

Step-by-step interactive workflow:
  Step 1: Check sensor connectivity (VectorNav IMU, Ouster LiDAR, OAK-D Pro camera, CAN bus).
  Step 2: Static gravity & bias capture (prompts operator to keep robot still).
  Step 3: Manual in-place rotation (prompts operator to spin robot via remote control).
  Step 4: Camera-to-LiDAR calibration with the Spot ChArUco panel (several board poses).
  Step 5: Review results & optionally export to deploy/robots/botman.env.

SAFETY:
  This tool is completely PASSIVE. It NEVER commands velocity (/cmd_vel) to the robot.

Usage (inside container or host):
  python3 scripts/calibration/run_botman_calibration.py
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

OAK_ENV_KEYS = ("BOTMAN_OAK_X", "BOTMAN_OAK_Y", "BOTMAN_OAK_Z",
                "BOTMAN_OAK_ROLL", "BOTMAN_OAK_PITCH", "BOTMAN_OAK_YAW")


def check_sensor_health() -> None:
    print("=" * 65)
    print(" STEP 1: SENSOR & NETWORK CONNECTIVITY CHECK")
    print("=" * 65)

    vn_dev = next((d for d in ("/dev/vectornav", "/dev/ttyUSB0") if os.path.exists(d)), None)
    if vn_dev:
        print(f"  [OK] VectorNav serial device is present locally ({vn_dev})")
    else:
        print("  [INFO] VectorNav serial not directly in container /dev "
              "(subscribing via ROS 2 topic /vectornav/imu)")

    ouster_ip = "192.168.2.199"
    try:
        with socket.create_connection((ouster_ip, 80), timeout=1.0):
            print(f"  [OK] Ouster LiDAR is reachable at {ouster_ip}:80 (HTTP API)")
    except (socket.timeout, ConnectionRefusedError, OSError):
        print(f"  [WARN] Ouster LiDAR not reachable at {ouster_ip}:80")

    if os.path.exists("/sys/class/net/can2"):
        print("  [OK] Bunker CAN interface can2 is present locally")
    else:
        print("  [INFO] can2 not in container sysfs (subscribing via ROS 2 topic /odom)")

    # The camera stage needs OpenCV's aruco module; report it here rather than
    # letting Phase 3 fail 20 minutes into the session.
    try:
        import cv2
        import cv2.aruco  # noqa: F401
        print(f"  [OK] OpenCV {cv2.__version__} with aruco module")
    except ImportError as exc:
        print(f"  [FAIL] OpenCV aruco unavailable ({exc}). Phase 3 cannot run; "
              "install opencv-contrib-python.")
    print()


def update_env_file(env_file: Path, values: dict) -> bool:
    """Rewrite only the value inside `: "${KEY:=VALUE}"`, leaving the shell
    quoting intact.

    The previous version substituted `KEY:=.*` with a replacement ending in `"}`,
    which turned `: "${BOTMAN_OAK_X:=0.03}"` into `: "${BOTMAN_OAK_X:=0.030"}`:
    the brace and quote swapped, leaving an unbalanced quote that breaks the
    file for every later reader.
    """
    if not env_file.exists():
        print(f"  [ERROR] {env_file} does not exist", file=sys.stderr)
        return False

    text = env_file.read_text()
    original = text
    for key, value in values.items():
        pattern = re.compile(r"(\$\{" + re.escape(key) + r":=)[^}]*(\})")
        text, n = pattern.subn(lambda m: f"{m.group(1)}{value}{m.group(2)}", text)
        if n == 0:
            print(f"  [WARN] {key} not found in {env_file.name}; left unchanged")
        elif n > 1:
            print(f"  [WARN] {key} appears {n} times; all updated")

    if text == original:
        print("  [INFO] nothing changed")
        return False

    try:
        env_file.write_text(text)
    except OSError as exc:
        # The repo is bind-mounted read-only into the robot containers
        # (/ssd/swarmdeck -> /app/swarmdeck), so a run from inside one cannot
        # write here. Do not lose the result: put it somewhere writable and
        # print it, rather than dying with a traceback after a 20 minute session.
        print(f"  [WARN] cannot write {env_file}: {exc}", file=sys.stderr)
        fallback = Path("/tmp") / f"botman.env.calibrated.{os.getpid()}"
        try:
            fallback.write_text(text)
            print(f"  [OK] wrote the updated file to {fallback} instead")
        except OSError:
            fallback = None
        print("\n  Apply it from a host shell with:\n")
        for key, value in values.items():
            print(f"      {key}={value}")
        print("\n  or, on the machine that holds the checkout:\n")
        assign = " ".join(f"{k}={v}" for k, v in values.items())
        print(f"      python3 scripts/calibration/run_botman_calibration.py --write-env {assign}\n")
        return False
    print(f"  [OK] Updated {env_file}")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=25.0,
                        help="Manual rotation duration in seconds (default: 25)")
    parser.add_argument("--square-size", type=float, default=0.1125,
                        help="Spot panel chequer square side in metres (default: 0.1125, derived "
                             "from the panel documented 118.5 x 50 cm outer size)")
    parser.add_argument("--marker-size", type=float, default=None,
                        help="Marker side in metres (default: 0.75 * square size)")
    parser.add_argument("--dictionary", default="DICT_4X4_50",
                        help="ArUco dictionary of the panel (default: DICT_4X4_50, the Spot panel)")
    parser.add_argument("--captures", type=int, default=5,
                        help="Number of board poses to capture (minimum 3, default: 5)")
    parser.add_argument("--write-env", nargs="+", metavar="KEY=VALUE", default=None,
                        help="Apply KEY=VALUE pairs to deploy/robots/botman.env and exit. Use this "
                             "from a host shell when the calibration ran inside a container whose "
                             "repo mount is read-only.")
    parser.add_argument("--skip-imu", action="store_true", help="Skip phases 1 and 2")
    parser.add_argument("--skip-camera", action="store_true", help="Skip phase 3")
    args = parser.parse_args()

    if args.write_env:
        pairs = {}
        for item in args.write_env:
            if "=" not in item:
                parser.error(f"--write-env expects KEY=VALUE, got {item!r}")
            k, v = item.split("=", 1)
            pairs[k] = v
        update_env_file(REPO_ROOT / "deploy/robots/botman.env", pairs)
        return

    print("\n" + "=" * 65)
    print(" BOTMAN MULTI-SENSOR TF & EXTRINSIC CALIBRATION")
    print("=" * 65)
    print("This interactive tool will guide you step-by-step to calibrate:")
    print("  1. VectorNav IMU <-> Robot Base <-> Ouster LiDAR (via in-place rotation)")
    print("  2. OAK-D Pro Camera <-> Ouster LiDAR (via the Spot ChArUco panel)")
    print("=" * 65 + "\n")

    check_sensor_health()

    sys.path.insert(0, str(REPO_ROOT / "scripts/calibration"))
    from calibrate_imu_motion import run_interactive_imu_calibration
    from calibrate_camera_lidar_panel import run_interactive_camera_lidar_calibration

    imu_results = None
    if not args.skip_imu:
        input("--> Press [ENTER] to begin Phase 1 (Static Gravity & Tilt Estimation)... ")
        imu_results = run_interactive_imu_calibration(
            imu_topic="/vectornav/imu",
            odom_topic="/odom",
            lidar_odom_topic="/laser_odometry",
            motion_duration=args.duration,
        )

    cam_results = None
    if not args.skip_camera:
        cam_results = run_interactive_camera_lidar_calibration(
            image_topic="/oak/rgb/image_raw/compressed",
            camera_info_topic="/oak/rgb/camera_info",
            lidar_topic="/ouster/points",
            square_size=args.square_size,
            marker_size=args.marker_size,
            dictionary=args.dictionary,
            captures=args.captures,
        )

    print("\n" + "=" * 65)
    print(" FINAL CALIBRATION SUMMARY")
    print("=" * 65)

    if cam_results is not None:
        t = cam_results["translation"]
        rpy = cam_results["rpy_rad"]
        print("\n[Camera Extrinsics] os_lidar -> oak-d-base-frame:")
        print(f"  Translation (x, y, z): [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
        print(f"  Euler RPY (rad):       [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")
    else:
        print("\n[Camera Extrinsics] not determined.")

    if imu_results is not None:
        if "rpy_imu_laser" in imu_results:
            rpy_l = imu_results["rpy_imu_laser"]
            print("\n[IMU-to-LiDAR Extrinsics] vectornav <- os_lidar:")
            print(f"  Euler RPY (rad):       [{rpy_l[0]:.4f}, {rpy_l[1]:.4f}, {rpy_l[2]:.4f}]")
            print("  Yaw is 0 by construction: an in-place spin cannot observe it.")
        lever = imu_results.get("lever_arm_fit")
        if lever is not None:
            r, se = lever["r"], lever["stderr"]
            print("\n[IMU lever arm] offset from the rotation axis:")
            print(f"  r_x = {r[0]:+.3f} +/- {se[0]:.3f} m, r_y = {r[1]:+.3f} +/- {se[1]:.3f} m")
        print("\n  Note: botman.env carries no IMU keys, so these are reported only.")

    if cam_results is not None:
        print("\n" + "=" * 65)
        ans = input("--> Update deploy/robots/botman.env with the measured camera values? [y/N]: ").strip().lower()
        if ans == "y":
            t = cam_results["translation"]
            rpy = cam_results["rpy_rad"]
            update_env_file(REPO_ROOT / "deploy/robots/botman.env", {
                "BOTMAN_OAK_X": f"{t[0]:.3f}",
                "BOTMAN_OAK_Y": f"{t[1]:.3f}",
                "BOTMAN_OAK_Z": f"{t[2]:.3f}",
                "BOTMAN_OAK_ROLL": f"{rpy[0]:.4f}",
                "BOTMAN_OAK_PITCH": f"{rpy[1]:.4f}",
                "BOTMAN_OAK_YAW": f"{rpy[2]:.4f}",
            })

    print("\nCalibration session finished.")


if __name__ == "__main__":
    main()
