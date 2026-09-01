#!/usr/bin/env python3
"""Interactive end-to-end calibration orchestrator for Aslan.

Step-by-step interactive workflow:
  Step 1: Check sensor connectivity (VectorNav IMU, Ouster LiDAR, CAN bus).
  Step 2: Dual IMU calibration (VectorNav <-> Ouster LiDAR via passive chassis rocking).
  Step 3: Review results & export to deploy/robots/aslan.env and aslan_superodom_calibration.yaml.

SAFETY:
  This tool is completely PASSIVE. It NEVER commands velocity (/cmd_vel) to the robot.

Usage:
  python3 scripts/calibration/run_aslan_calibration.py [options]
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def check_sensor_health() -> None:
    print("=" * 65)
    print(" STEP 1: SENSOR & NETWORK CONNECTIVITY CHECK (ASLAN)")
    print("=" * 65)

    domain_id = os.environ.get("ROS_DOMAIN_ID", "")
    if domain_id == "49":
        print("  [OK] ROS_DOMAIN_ID is set to 49")
    else:
        print(f"  [WARN] ROS_DOMAIN_ID is '{domain_id}' (setting to 49 for Aslan)")
        os.environ["ROS_DOMAIN_ID"] = "49"

    vn_dev = next((d for d in ("/dev/vectornav", "/dev/ttyUSB0") if os.path.exists(d)), None)
    if vn_dev:
        print(f"  [OK] VectorNav serial device is present locally ({vn_dev})")
    else:
        print("  [INFO] VectorNav serial not in local /dev (subscribing via ROS 2 topic /vectornav/imu)")

    ouster_ip = "192.168.2.118"
    try:
        with socket.create_connection((ouster_ip, 80), timeout=1.0):
            print(f"  [OK] Ouster LiDAR is reachable at {ouster_ip}:80 (HTTP API)")
    except (socket.timeout, ConnectionRefusedError, OSError):
        print(f"  [INFO] Ouster LiDAR IP not directly routable from this host (checking via ROS 2 topics)")

    if os.path.exists("/sys/class/net/can2"):
        print("  [OK] Bunker CAN interface can2 is present locally")
    else:
        print("  [INFO] can2 not in local sysfs (subscribing via ROS 2 topic /bunker_status)")
    print()


def update_env_file(env_file: Path, values: dict) -> bool:
    """Rewrite only the value inside `: "${KEY:=VALUE}"`, leaving the shell quoting intact."""
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
        print("  [INFO] nothing changed in env file")
        return False

    try:
        env_file.write_text(text)
    except OSError as exc:
        print(f"  [WARN] cannot write {env_file}: {exc}", file=sys.stderr)
        return False
    print(f"  [OK] Updated {env_file}")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Calibration duration in seconds (default: 30)",
    )
    parser.add_argument(
        "--target-topic",
        default="/vectornav/imu",
        help="Target IMU topic (default: /vectornav/imu)",
    )
    parser.add_argument(
        "--reference-topic",
        default="/ouster/imu",
        help="Reference IMU topic (default: /ouster/imu)",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Start recording immediately without waiting for enter",
    )
    args = parser.parse_args()

    # Ensure ROS_DOMAIN_ID=49 for Aslan
    os.environ["ROS_DOMAIN_ID"] = "49"

    print("\n" + "=" * 65)
    print(" ASLAN MULTI-SENSOR TF & EXTRINSIC CALIBRATION")
    print("=" * 65)

    check_sensor_health()

    sys.path.insert(0, str(REPO_ROOT / "scripts/calibration"))
    from calibrate_imu_to_imu import run as run_dual_imu_calibration, emit_calibration_yaml

    print("--> PHASE 1: Dual-IMU Gyro Calibration (VectorNav <-> Ouster)")
    imu_to_imu_res = run_dual_imu_calibration(
        target_topic=args.target_topic,
        reference_topic=args.reference_topic,
        duration=args.duration,
        no_prompt=args.no_prompt,
    )

    print("\n" + "=" * 65)
    print(" CALIBRATION SUMMARY & EXPORT (ASLAN)")
    print("=" * 65)

    if imu_to_imu_res and "R_lidar_target" in imu_to_imu_res:
        R_lidar_vn = imu_to_imu_res["R_lidar_target"]
        t_lidar_vn = imu_to_imu_res["t_lidar_target"]
        calib_yaml = emit_calibration_yaml(R_lidar_vn, t_lidar_vn)
        calib_file = REPO_ROOT / "adapters/adapter_ros2/config/aslan_superodom_calibration.yaml"
        print(f"\n[IMU Calibration] Writing calibration to {calib_file.name}...")
        calib_file.write_text(calib_yaml)
        print(f"  [OK] Saved {calib_file}")

        # Update aslan.env to enable VectorNav
        env_file = REPO_ROOT / "deploy/robots/aslan.env"
        update_env_file(
            env_file,
            {
                "ASLAN_START_VECTORNAV": "true",
                "ASLAN_IMU_TOPIC": "/vectornav/imu",
                "ASLAN_SUPERODOM_CONFIG": "aslan_vectornav.yaml",
                "ASLAN_SUPERODOM_CALIB": "aslan_superodom_calibration.yaml",
            },
        )
        print(f"  [OK] Updated {env_file.name} with VectorNav parameters.")

    print("\nCalibration complete.")


if __name__ == "__main__":
    main()
