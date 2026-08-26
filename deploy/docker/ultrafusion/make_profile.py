#!/usr/bin/env python3
"""
Builds an ARGoS Ultra-Fusion profile by patching a released one.

Ultra-Fusion's own guidance is emphatic: copy the closest released
profile whole and edit it, because "the runtime expects the full field
set at startup". So this does not generate a YAML; it copies
/opt/ultrafusion/config/m3dgr/uf_m3dgr_ros2_<mode>.yaml and rewrites
only the keys that describe OUR sensors.

The files are `%YAML:1.0` (OpenCV flavoured) and a real YAML round trip
would both choke on that directive and destroy the ordering and
comments, so the edits are line-oriented and surgical.

Topic names are deliberately NOT patched. Ultra-Fusion declares its
topics with absolute names, so run_uf.sh remaps every one of them on the
uf_node command line instead ("-r /a:=/b" is verified to work on the
0.2.2 binary, including for the published /odom_lidar). Leaving the
topic block untouched keeps this profile as close to the released one as
possible.

Usage:
  make_profile.py --mode lwio --out /work/profiles/r0 \\
      [--lidar-elev -15 15] [--imu-noise ...] [--map-pcd]
"""
import argparse
import math
import os
import re
import shutil
import sys

RELEASED = "/opt/ultrafusion/config/m3dgr"

# Fusion mode -> the released profile to start from. These are the
# switch combinations documented in README section 3.1.
# Fusion mode -> (released profile to start from, switch overrides).
#
# The .deb ships no lvio profile, so it is lvwio with the wheel turned
# off, exactly as the README's mode table defines it (use_lidar 1,
# use_image 1, wheel 0). The overrides are applied after loading, so a
# mode is never silently the wrong combination just because no released
# file happened to match it.
MODES = {
    "lio":   ("uf_m3dgr_ros2_lio.yaml",   {}),                 # lidar + imu
    "lwio":  ("uf_m3dgr_ros2_lwio.yaml",  {}),                 # + wheel
    "lvwio": ("uf_m3dgr_ros2_lvwio.yaml", {}),                 # + visual (all)
    "lvio":  ("uf_m3dgr_ros2_lvwio.yaml", {"wheel": 0}),       # all but wheel
    "vio":   ("uf_m3dgr_ros2_vio.yaml",   {}),
    "viwo":  ("uf_m3dgr_ros2_viwo.yaml",  {}),
}


def set_scalar(text, key, value, section=None, add_if_missing=False):
    """Replaces `key: <anything>` with `key: value`.

    When `section` is given, only the block introduced by that key is
    searched, so `enable:` under map_pcd cannot be confused with another
    `enable:` elsewhere.
    """
    body = text
    start, end = 0, len(text)
    if section is not None:
        m = re.search(r"^%s:\s*$" % re.escape(section), text, re.M)
        if not m:
            raise KeyError("section %r not found" % section)
        start = m.end()
        # The block runs until the next line that starts in column 0
        nxt = re.search(r"^\S", text[start:], re.M)
        end = start + (nxt.start() if nxt else len(text) - start)
        body = text[start:end]

    # A section-less lookup must anchor at column 0. Several top-level
    # switches share a name with a key nested in a block -- "wheel" is
    # both the top-level fusion switch and an entry under sensor_freq --
    # and matching the first occurrence anywhere silently edits the
    # wrong one.
    indent_pat = r"^(\s*)" if section is not None else r"^()"
    pattern = re.compile(indent_pat + r"%s:[ \t]*.*$" % re.escape(key), re.M)
    if not pattern.search(body):
        if not add_if_missing:
            raise KeyError("key %r not found%s"
                           % (key, " in section %r" % section if section else ""))
        if section is None:
            raise ValueError("add_if_missing needs a section")
        # Indent like the block's existing entries
        indent = re.search(r"^(\s+)\S", body, re.M)
        body = body.rstrip("\n") + "\n%s%s: %s\n" % (
            indent.group(1) if indent else "  ", key, value)
        return text[:start] + body + text[end:]
    patched = pattern.sub(lambda m: "%s%s: %s" % (m.group(1), key, value),
                          body, count=1)
    return text[:start] + patched + text[end:]


def write_camera_yaml(args):
    """Emits the camodocal PINHOLE calibration for the ARGoS camera.

    ARGoS renders with filament::Camera::Fov::VERTICAL, so the field of
    view relates to the HEIGHT:  fy = h / (2 tan(fov/2)), and fx = fy
    because the projection uses square pixels. Dividing the width by it
    instead inflates both focal lengths by the aspect ratio.
    """
    w, h = (int(v) for v in args.camera_resolution.split(","))
    fy = h / (2.0 * math.tan(math.radians(args.camera_fov) / 2.0))
    fx = fy
    text = """%%YAML:1.0
---
model_type: PINHOLE
camera_name: camera
image_width: %d
image_height: %d
distortion_parameters:
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
projection_parameters:
  fx: %.8f
  fy: %.8f
  cx: %.8f
  cy: %.8f
""" % (w, h, fx, fy, w / 2.0, h / 2.0)
    for name in ("color.yaml", "cam0_pinhole.yaml"):
        path = os.path.join(args.out, name)
        if name == "color.yaml" or os.path.exists(path):
            open(path, "w").write(text)
    print("camera %dx%d fov %g deg -> fx=fy=%.3f cx=%.1f cy=%.1f"
          % (w, h, args.camera_fov, fx, w / 2.0, h / 2.0), file=sys.stderr)
    return w, h


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=sorted(MODES), default="lwio")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--released", default=RELEASED)
    # --- what ARGoS's sensors actually are -------------------------
    ap.add_argument("--lidar-elev", nargs=2, type=float, default=[-15.0, 15.0],
                    metavar=("MIN", "MAX"),
                    help="lidar vertical FOV in degrees; the ARGoS "
                         "photorealistic_lidar defaults model a VLP-16 at "
                         "-15..+15, while the released profile is a Livox "
                         "Mid-360 at -7..+52")
    ap.add_argument("--camera-resolution", default="320,240",
                    help="the <photorealistic_camera> resolution attribute")
    ap.add_argument("--camera-fov", type=float, default=60.0,
                    help="the camera's VERTICAL field of view in degrees, "
                         "from the fov attribute")
    ap.add_argument("--depth-threshold", type=int, default=10,
                    help="furthest depth accepted for a visual feature, "
                         "metres. The released profiles use 3, tuned for an "
                         "indoor RealSense; a street scene starves at that.")
    ap.add_argument("--img0-type", type=int, default=None,
                    help="0 = raw sensor_msgs/Image, 1 = CompressedImage")
    ap.add_argument("--img1-type", type=int, default=None,
                    help="same, for the depth image. uf_link publishes depth "
                         "as a raw Image, so this must be 0; the released "
                         "profiles ship 1 and the mismatch is silent -- "
                         "uf_node just never receives depth, and an RGB-D "
                         "tracker with no depth reports zero features.")
    ap.add_argument("--imu-hz", type=float, default=100.0)
    ap.add_argument("--wheel-hz", type=float, default=100.0)
    ap.add_argument("--image-hz", type=float, default=10.0)
    ap.add_argument("--gravity", type=float, default=9.81,
                    help="must match <gravity g=...> in the .argos file")
    # Defaults are the released profile's own IMU noise model, in the
    # VINS convention Ultra-Fusion inherits: continuous-time densities,
    # m/s^2/sqrt(Hz) and rad/s/sqrt(Hz) for the white noise, m/s^3/sqrt(Hz)
    # and rad/s^2/sqrt(Hz) for the bias random walks.
    ap.add_argument("--imu-acc-n", type=float, default=0.04)
    ap.add_argument("--imu-gyr-n", type=float, default=0.008)
    ap.add_argument("--imu-acc-w", type=float, default=2.0e-4)
    ap.add_argument("--imu-gyr-w", type=float, default=2.0e-5)
    ap.add_argument("--argos-imu-noise", nargs=4, type=float, default=None,
                    metavar=("GYRO", "ACCEL", "GYRO_WALK", "ACCEL_WALK"),
                    help="the four <imu> attributes from the .argos file "
                         "(gyro_noise_std_dev, accel_noise_std_dev, "
                         "gyro_bias_walk_std_dev, accel_bias_walk_std_dev), "
                         "converted to Ultra-Fusion's units. Prefer this over "
                         "the --imu-*-n/w flags: the two conventions differ by "
                         "a factor of sqrt(rate), which is 10x at 100 Hz and "
                         "silently mis-weights every IMU factor.")
    ap.add_argument("--imu-noise-scale", type=float, default=3.0,
                    help="factor applied to the converted densities before "
                         "they reach the profile. An estimator should assume "
                         "noise at or above the device's true level, never "
                         "below it, so matching the simulator exactly makes "
                         "it over-trust the IMU. The released profiles are "
                         "themselves inflated well above any datasheet.")
    ap.add_argument("--planar-wheel", action="store_true",
                    help="use the planar wheel factor: correct for a "
                         "differential drive on flat ground, which is what "
                         "the foot-bot is")
    ap.add_argument("--map-pcd", action="store_true",
                    help="enable the optional map-PCD export (the mapper)")
    ap.add_argument("--map-dir", default="/results/map")
    ap.add_argument("--map-service", default="/ultrafusion/generate_map_pcd",
                    help="namespace this per robot, or several uf_node "
                         "instances fight over one service name")
    ap.add_argument("--voxel", type=float, default=None,
                    help="override odometry.size_voxel_map and surf_res; the "
                         "released 0.5 m is tuned for vehicle-scale outdoor "
                         "runs and is coarse for indoor/street scenes")
    # --- extrinsics ------------------------------------------------
    # ARGoS mounts the IMU at the body origin, and gives the lidar and
    # camera a "position" relative to that same anchor, so these are
    # read straight off the .argos file.
    ap.add_argument("--lidar-in-body", nargs=3, type=float,
                    default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"),
                    help="lidar <position> in the .argos file: T_I_L")
    ap.add_argument("--camera-in-body", nargs=3, type=float,
                    default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"),
                    help="camera <position> in the .argos file: T_I_C")
    args = ap.parse_args()

    base, overrides = MODES[args.mode]
    src = os.path.join(args.released, base)
    if not os.path.exists(src):
        sys.exit("released profile not found: %s" % src)
    os.makedirs(args.out, exist_ok=True)
    # Camera intrinsics live in a file next to the main YAML, resolved
    # relative to it, so the whole directory has to travel together
    for name in os.listdir(args.released):
        if name.endswith(".yaml") and not name.startswith("uf_"):
            shutil.copy2(os.path.join(args.released, name), args.out)
    # ...but the released color.yaml describes an M3DGR RealSense
    # (640x480, fx 607.8). Ours is whatever the <photorealistic_camera>
    # is, so overwrite it. Left alone, the visual path runs with a focal
    # length ~3x too long and simply fails to converge, which looks like
    # a tuning problem rather than a calibration one.
    write_camera_yaml(args)

    text = open(src).read()

    for key, value in sorted(overrides.items()):
        text = set_scalar(text, key, value)
    if overrides:
        print("mode %s = %s with %s" % (args.mode, base, overrides),
              file=sys.stderr)

    text = set_scalar(text, "imu", int(args.imu_hz), section="sensor_freq")
    text = set_scalar(text, "wheel", int(args.wheel_hz), section="sensor_freq")
    text = set_scalar(text, "image", int(args.image_hz), section="sensor_freq")

    text = set_scalar(text, "lidar_elev_min", args.lidar_elev[0])
    text = set_scalar(text, "lidar_elev_max", args.lidar_elev[1])
    text = set_scalar(text, "imu_res", args.gravity)

    acc_n, gyr_n = args.imu_acc_n, args.imu_gyr_n
    acc_w, gyr_w = args.imu_acc_w, args.imu_gyr_w
    if args.argos_imu_noise is not None:
        #
        # ARGoS and Ultra-Fusion state IMU noise in different units, and
        # nothing warns you about it.
        #
        # imu_default_sensor.cpp draws ONE Gaussian per tick:
        #   reading += Gaussian(gyro_noise_std_dev)
        #   bias    += Gaussian(gyro_bias_walk_std_dev)
        # so those attributes are per-SAMPLE standard deviations at the
        # simulation rate. Ultra-Fusion inherits VINS's convention, where
        # the same quantities are continuous-time DENSITIES, per sqrt(Hz).
        #
        # For white noise a per-sample sigma is the density times
        # sqrt(rate); for a random walk the per-step sigma is the density
        # times sqrt(dt). Inverting both:
        #
        #   density_white = sigma_sample / sqrt(rate)
        #   density_walk  = sigma_step   * sqrt(rate)
        #
        # At 100 Hz that is a factor of 10 in opposite directions, i.e.
        # 100x between them. Getting it wrong does not fail loudly: the
        # estimator just weights its IMU factors wrongly and drifts.
        g_n, a_n, g_w, a_w = args.argos_imu_noise
        root_rate = math.sqrt(args.imu_hz)
        k = args.imu_noise_scale
        gyr_n, acc_n = g_n / root_rate * k, a_n / root_rate * k
        gyr_w, acc_w = g_w * root_rate * k, a_w * root_rate * k
        print("IMU noise from ARGoS at %g Hz (x%g margin): gyr_n=%.6g "
              "acc_n=%.6g gyr_w=%.6g acc_w=%.6g"
              % (args.imu_hz, k, gyr_n, acc_n, gyr_w, acc_w), file=sys.stderr)

    text = set_scalar(text, "imu_acc_n", acc_n, section="noise")
    text = set_scalar(text, "imu_gyr_n", gyr_n, section="noise")
    text = set_scalar(text, "imu_acc_w", acc_w, section="noise")
    text = set_scalar(text, "imu_gyr_w", gyr_w, section="noise")

    text = set_scalar(text, "use_planar_wheel_factor",
                      "true" if args.planar_wheel else "false")

    # --- visual front end ------------------------------------------
    #
    # The released feature parameters assume the 640x480 RealSense the
    # profiles were tuned on. min_dist is a MINIMUM SEPARATION IN
    # PIXELS, so carrying 30 over to a 320x240 image doubles the
    # effective spacing and roughly quarters the feature count the
    # tracker can hold -- on a low-parallax planar platform that is the
    # difference between tracking and not. Scale it with the width.
    cam_w, _cam_h = (int(v) for v in args.camera_resolution.split(","))
    ref_w = 640.0
    min_dist = max(6, int(round(30 * cam_w / ref_w)))
    text = set_scalar(text, "min_dist", min_dist)
    # depth_threshold is the furthest depth accepted for a feature. The
    # released 3 m suits an indoor RealSense; in a street it throws away
    # every facade and keeps only the road right in front of the robot.
    # It must be written as an INTEGER: yaml-cpp reads it into an int and
    # "10.0" fails the whole config load with a bare
    # "bad conversion" naming only the line number.
    text = set_scalar(text, "depth_threshold", int(args.depth_threshold))
    if args.img0_type is not None:
        text = set_scalar(text, "img0_type", args.img0_type, section="common")
    if args.img1_type is not None:
        text = set_scalar(text, "img1_type", args.img1_type, section="common")
    print("visual: min_dist %d px (scaled from 30 at %dpx wide), "
          "depth_threshold %g m" % (min_dist, cam_w, args.depth_threshold),
          file=sys.stderr)

    text = set_scalar(text, "enable", "true" if args.map_pcd else "false",
                      section="map_pcd")
    text = set_scalar(text, "output_directory", '"%s"' % args.map_dir,
                      section="map_pcd")
    text = set_scalar(text, "service_name", '"%s"' % args.map_service,
                      section="map_pcd")

    if args.voxel is not None:
        text = set_scalar(text, "size_voxel_map", args.voxel, section="odometry")
        text = set_scalar(text, "surf_res", args.voxel, section="odometry")

    # --- extrinsics -------------------------------------------------
    #
    # Every one of these MUST be replaced: the released values describe
    # an M3DGR vehicle with a Livox Mid-360, and leaving them in place
    # would have the estimator correct for mounts that do not exist.
    #
    # ARGoS body frame:   +x forward, +y left,  +z up
    # ARGoS lidar frame:  the same (see ci_photorealistic_lidar_sensor.h)
    # Camera optical:     +x right,   +y down,  +z forward
    #
    # T_I_L is therefore a pure translation, and R_I_C is the fixed
    # body-to-optical rotation, whose columns are the camera axes
    # expressed in the body frame:
    #     cam x (right)   = body -y
    #     cam y (down)    = body -z
    #     cam z (forward) = body +x
    # (the same matrix Ground-Fusion's own Isaac Sim config uses).
    text = set_scalar(text, "extrinsic_T",
                      "[ %g, %g, %g]" % tuple(args.lidar_in_body),
                      section="mapping")
    text = set_scalar(text, "extrinsic_R",
                      "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
                      section="mapping")
    #
    # The released profile carries extrinsic_TCL/RCL and TOL/ROL, from
    # which Ultra-Fusion derives T_I_C and T_I_O. The explicit TIC/RIC
    # and TIO/RIO keys take priority when present (README 3.4), so they
    # are added here rather than trying to express our mounts through
    # the lidar-relative fallbacks.
    text = set_scalar(text, "extrinsic_TIC",
                      "[ %g, %g, %g]" % tuple(args.camera_in_body),
                      section="mapping", add_if_missing=True)
    text = set_scalar(text, "extrinsic_RIC",
                      "[0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]",
                      section="mapping", add_if_missing=True)
    # T_I_O: the frame the medium dead-reckons the wheels in IS the body
    # frame, so this is the identity.
    text = set_scalar(text, "extrinsic_TIO", "[0.0, 0.0, 0.0]",
                      section="mapping", add_if_missing=True)
    text = set_scalar(text, "extrinsic_RIO",
                      "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
                      section="mapping", add_if_missing=True)

    dst = os.path.join(args.out, "uf_argos.yaml")
    open(dst, "w").write(text)
    print(dst)


if __name__ == "__main__":
    main()
