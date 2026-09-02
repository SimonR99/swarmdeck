#!/usr/bin/env python3
"""Builds a Fast-LIVO2 configuration profile tailored to an ARGoS robot.

Generates a complete Fast-LIVO2 configuration YAML by configuring:
  1. Sensor extrinsics (LiDAR-to-IMU and Camera-to-IMU) from the ARGoS session.
  2. Camera intrinsics (PINHOLE model fx, fy, cx, cy) from vertical FOV and resolution.
  3. Continuous-time IMU noise densities derived from ARGoS simulation parameters.
  4. LiDAR parameters (17-ring / Livox / VLP-16, blind distance, voxel grid sizes).

Usage:
  make_profile.py --out /work/profiles/robot_0 \\
      --lidar-in-body 0.0 0.0 0.720 \\
      --camera-in-body 0.15 0.0 0.620 \\
      --camera-resolution 320,240 --camera-fov 60.0 \\
      --lidar-elev -15 15 --argos-imu-noise 0.002 0.02 0.0002 0.002
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import yaml


def compute_camera_intrinsics(resolution_str: str, fov_deg: float) -> dict:
    """Computes pinhole camera intrinsics from vertical FOV and resolution.

    ARGoS renders using vertical field of view (filament::Camera::Fov::VERTICAL).
    Therefore:
      fy = height / (2.0 * tan(fov / 2.0))
      fx = fy  (square pixels)
      cx = width / 2.0
      cy = height / 2.0
    """
    w, h = (int(v) for v in resolution_str.split(","))
    fy = h / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    fx = fy
    cx = w / 2.0
    cy = h / 2.0
    return {
        "width": w,
        "height": h,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
    }


def compute_imu_noise(argos_noise: list[float] | None, imu_hz: float, scale: float = 3.0) -> dict:
    """Converts ARGoS discrete per-sample IMU noise to continuous-time densities and covariances.

    ARGoS imu_default_sensor draws:
      reading += Gaussian(gyro_noise_std_dev)
      bias    += Gaussian(gyro_bias_walk_std_dev)

    Continuous-time spectral densities (per sqrt(Hz)):
      density_white = sigma_sample / sqrt(rate)
      density_walk  = sigma_step * sqrt(rate)
    """
    if argos_noise is not None and len(argos_noise) == 4:
        g_n, a_n, g_w, a_w = argos_noise
        root_rate = math.sqrt(imu_hz)
        gyr_n = (g_n / root_rate) * scale
        acc_n = (a_n / root_rate) * scale
        gyr_w = (g_w * root_rate) * scale
        acc_w = (a_w * root_rate) * scale
    else:
        # Default datasheet baseline
        gyr_n = 0.008
        acc_n = 0.040
        gyr_w = 2.0e-5
        acc_w = 2.0e-4

    return {
        "gyr_n": gyr_n,
        "acc_n": acc_n,
        "gyr_w": gyr_w,
        "acc_w": acc_w,
        "cov_gyr": gyr_n ** 2,
        "cov_acc": acc_n ** 2,
        "cov_bias_gyr": gyr_w ** 2,
        "cov_bias_acc": acc_w ** 2,
    }


def build_fast_livo_config(
    lidar_in_body: list[float],
    camera_in_body: list[float],
    cam_intrinsics: dict,
    imu_noise: dict,
    lidar_elev: list[float],
    voxel_size: float = 0.5,
    blind_dist: float = 0.2,
    point_filter_num: int = 1,
    max_iteration: int = 4,
    gravity: float = 9.81,
    lidar_type: int = 7,  # 7 = Livox layout as PointCloud2, which ARGoS publishes
    scan_line: int = 17,
    img_en: int = 0,
) -> dict:
    """Constructs the Fast-LIVO2 parameter dictionary."""

    # ARGoS body frame:  +x forward, +y left,  +z up
    # ARGoS lidar frame: +x forward, +y left,  +z up (identity rotation)
    # Camera optical:    +x right,   +y down,  +z forward
    #
    # Camera to Body rotation matrix (R_I_C):
    #   col 0 (cam x, right)   = body -y  -> [ 0, -1,  0]
    #   col 1 (cam y, down)    = body -z  -> [ 0,  0, -1]
    #   col 2 (cam z, forward) = body +x  -> [ 1,  0,  0]
    # Row-major representation:
    #   [ 0.0,  0.0, 1.0,
    #    -1.0,  0.0, 0.0,
    #     0.0, -1.0, 0.0]

    cfg = {
        "common": {
            # Bound the input queues. Upstream leaves them unbounded and
            # sync_packages() drains them only when a full package forms, so a
            # sensor stall or clock desync grows memory at the sensor rate.
            "max_img_buffer": 30,
            "max_lidar_buffer": 20,
            "max_imu_buffer": 4000,
            "lid_topic": "points",
            "imu_topic": "imu",
            "img_topic": "color/image_raw/compressed",
            # Neither of these was emitted before, so both took the code's
            # defaults. img_en defaults on, and FAST-LIVO2's sync_packages will
            # not emit a pose until it has an image to go with the scan, so a
            # sim that is not rendering leaves the estimator silent with no
            # error. Default to LiDAR-inertial here and turn the visual half on
            # deliberately once imagery is confirmed flowing.
            "img_en": int(img_en),
            "lidar_en": 1,
            "con_frame": False,
            "time_sync_en": False,
            "time_offset_lidar_to_imu": 0.0,
            "time_offset_time_camera_to_imu": 0.0,
        },
        # Sensor extrinsics. The stub this harness was written against ignored
        # these entirely; the real FAST-LIVO2 indexes them as [0..8] with no
        # bounds check, so omitting them used to be a segfault (it now raises).
        #
        # ARGoS mounts every sensor axis-aligned with the body, and the IMU sits
        # at the body origin, so the LiDAR-to-IMU rotation is identity and the
        # translation is just the LiDAR's mount height.
        "extrin_calib": {
            "extrinsic_T": [float(v) for v in lidar_in_body],
            "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            # LiDAR (x fwd, y left, z up) -> camera optical (x right, y down,
            # z fwd) is the fixed axis permutation; the offset is the LiDAR
            # origin expressed in the camera frame.
            "Rcl": [0.0, -1.0, 0.0,
                    0.0, 0.0, -1.0,
                    1.0, 0.0, 0.0],
            "Pcl": [
                -(float(lidar_in_body[1]) - float(camera_in_body[1])),
                -(float(lidar_in_body[2]) - float(camera_in_body[2])),
                 (float(lidar_in_body[0]) - float(camera_in_body[0])),
            ],
        },
        "preprocess": {
            "lidar_type": lidar_type,
            "scan_line": scan_line,
            "blind": blind_dist,
            "point_filter_num": point_filter_num,
            "time_scale": 1e-3,
            "lidar_elev_min": float(lidar_elev[0]),
            "lidar_elev_max": float(lidar_elev[1]),
        },
        "mapping": {
            "extrinsic_T": [float(v) for v in lidar_in_body],
            "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "camera_ext_t": [float(v) for v in camera_in_body],
            "camera_ext_R": [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
            "fov_degree": 60.0,
            "satu_acc": 30.0,
            "satu_gyro": 35.0,
            "acc_norm": float(gravity),
            "gyr_cov": float(imu_noise["cov_gyr"]),
            "acc_cov": float(imu_noise["cov_acc"]),
            "b_gyr_cov": float(imu_noise["cov_bias_gyr"]),
            "b_acc_cov": float(imu_noise["cov_bias_acc"]),
            "max_iteration": max_iteration,
            "voxel_size": float(voxel_size),
        },
        # Sections the real FAST-LIVO2 reads. The generator was written against
        # a stub that used FAST-LIO's names ("mapping", "odometry"), which the
        # real code never looks at, so IMU fusion silently stayed off and every
        # estimator logged "IMU Disabled !!!!!". The "mapping" block is kept
        # above for continuity; these are the ones that take effect.
        # IMU covariances use FAST-LIVO2's OWN convention, not the VINS-style
        # per-sqrt(Hz) densities computed above.
        #
        # This is the difference that mattered. compute_imu_noise() derives
        # densities for Ultra-Fusion, which inherits VINS's convention
        # (acc_n ~ 0.04, gyr_n ~ 0.008) and works correctly in this simulator.
        # FAST-LIVO2's acc_cov/gyr_cov are a different quantity: upstream ships
        # 0.5 and 0.3 for a real Livox rig. Feeding the densities in gave
        # acc_cov 3.6e-05 and gyr_cov 3.6e-07, i.e. 1e4 and 1e6 times too small,
        # which tells the filter the IMU is essentially perfect. The estimate
        # then rides IMU integration and runs away at a steady ~2 m/s: measured
        # -150 m to -167 m over 8 s against a ground truth of about 2 m.
        #
        # The densities remain available in imu_noise if someone wants to derive
        # these properly; upstream's proven values are the honest default until
        # then.
        "imu": {
            "imu_en": True,
            "imu_int_frame": 30,
            "acc_cov": 0.5,
            "gyr_cov": 0.3,
            "b_acc_cov": 0.0001,
            "b_gyr_cov": 0.0001,
        },
        "time_offset": {
            "imu_time_offset": 0.0,
            "img_time_offset": 0.0,
            "exposure_time_init": 0.0,
        },
        "vio": {
            "max_iterations": 5,
            "outlier_threshold": 1000.0,
            "img_point_cov": 100.0,
            "patch_size": 8,
            "patch_pyrimid_level": 4,
            "normal_en": True,
            "raycast_en": False,
            "inverse_composition_en": False,
            "exposure_estimate_en": True,
            "inv_expo_cov": 0.1,
            # Bound the visual sparse map. Upstream never evicts from feat_map,
            # so any grid cell that fails to match appends a new VisualPoint
            # every frame and each one pins a whole decoded frame via
            # Feature::img_. Zero restores the upstream unbounded behaviour.
            "max_pts_per_voxel": 20,
            "max_warp_cache": 2000,
            # Bounds the span of frames the map pins through Feature::img_,
            # which is what actually bounds resident memory.
            "max_point_age": 300,
            "map_sliding_en": True,
            "sliding_thresh": 8.0,
            "half_map_size": 50.0,
            "map_report_every": 25,
        },
        "lio": {
            "max_iterations": 5,
            "dept_err": 0.02,
            "beam_err": 0.05,
            "min_eigen_value": 0.0025,
            "voxel_size": float(voxel_size),
            "max_layer": 2,
            "max_points_num": 50,
            "layer_init_num": [5, 5, 5, 5, 5],
        },
        "local_map": {
            # Enabled: the LiDAR voxel map is otherwise unbounded too.
            "map_sliding_en": True,
            "half_map_size": 100,
            "sliding_thresh": 8.0,
        },
        "uav": {"imu_rate_odom": False, "gravity_align_en": False},
        "publish": {
            "dense_map_en": True,
            "pub_effect_point_en": False,
            "pub_plane_en": False,
            "pub_scan_num": 1,
            "blind_rgb_points": 0.0,
            # Upstream republishes the whole path each frame; unbounded.
            "path_max_poses": 2000,
        },
        "evo": {"seq_name": "argos", "pose_output_en": False},
        "pcd_save": {
            "pcd_save_en": False,
            "colmap_output_en": False,
            "filter_size_pcd": 0.15,
            "interval": -1,
        },
        "camera": {
            "camera_name": "camera",
            "image_width": int(cam_intrinsics["width"]),
            "image_height": int(cam_intrinsics["height"]),
            "distortion_model": "plumb_bob",
            "distortion_parameters": [0.0, 0.0, 0.0, 0.0, 0.0],
            "projection_parameters": [
                float(cam_intrinsics["fx"]),
                float(cam_intrinsics["fy"]),
                float(cam_intrinsics["cx"]),
                float(cam_intrinsics["cy"]),
            ],
        },
        "odometry": {
            "publish_odometry_without_downsample": True,
            "dense_map_en": False,
            "voxel_size": float(voxel_size),
            "patch_size": 8,
            "min_dist": max(6, int(round(30 * cam_intrinsics["width"] / 640.0))),
        },
    }
    return cfg


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--lidar-in-body", nargs=3, type=float, default=[0.0, 0.0, 0.720],
                    metavar=("X", "Y", "Z"), help="LiDAR position relative to body/IMU")
    ap.add_argument("--camera-in-body", nargs=3, type=float, default=[0.15, 0.0, 0.620],
                    metavar=("X", "Y", "Z"), help="Camera position relative to body/IMU")
    ap.add_argument("--camera-resolution", default="320,240",
                    help="Camera resolution as width,height")
    ap.add_argument("--camera-fov", type=float, default=60.0,
                    help="Camera vertical field of view in degrees")
    ap.add_argument("--lidar-elev", nargs=2, type=float, default=[-15.0, 15.0],
                    metavar=("MIN", "MAX"), help="LiDAR vertical elevation range")
    ap.add_argument("--imu-hz", type=float, default=100.0, help="IMU publish frequency")
    ap.add_argument("--gravity", type=float, default=9.81, help="Gravity constant")
    ap.add_argument("--argos-imu-noise", nargs=4, type=float, default=None,
                    metavar=("GYRO", "ACCEL", "GYRO_WALK", "ACCEL_WALK"),
                    help="ARGoS IMU noise parameters")
    ap.add_argument("--imu-noise-scale", type=float, default=3.0,
                    help="Safety margin multiplier for IMU noise")
    ap.add_argument("--voxel", type=float, default=0.5, help="Voxel size in meters")
    ap.add_argument("--scan-line", type=int, default=17, help="Number of LiDAR rings")
    ap.add_argument("--img-en", type=int, default=0,
                    help="1 to fuse camera (LIVO); 0 for LiDAR-inertial only")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    intrinsics = compute_camera_intrinsics(args.camera_resolution, args.camera_fov)
    imu_noise = compute_imu_noise(args.argos_imu_noise, args.imu_hz, args.imu_noise_scale)

    config = build_fast_livo_config(
        lidar_in_body=args.lidar_in_body,
        camera_in_body=args.camera_in_body,
        cam_intrinsics=intrinsics,
        imu_noise=imu_noise,
        lidar_elev=args.lidar_elev,
        voxel_size=args.voxel,
        gravity=args.gravity,
        scan_line=args.scan_line,
        img_en=args.img_en,
    )

    out_path = os.path.join(args.out, "fast_livo_argos.yaml")
    # ROS 2 parameter files must nest everything under a node pattern and
    # ros__parameters. Dumping the upstream ROS 1 layout straight out is what
    # aborted every previous run here with
    #   "Cannot have a value before ros__parameters at line 2"
    # and, because the link process reported "Initialized ... for robot_N"
    # against nodes that were already dead, it looked like it had worked.
    with open(out_path, "w") as f:
        yaml.safe_dump({"/**": {"ros__parameters": config}}, f, sort_keys=False)

    print(f"Generated Fast-LIVO2 config: {out_path}", file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()
