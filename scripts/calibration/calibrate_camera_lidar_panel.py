#!/usr/bin/env python3
"""Calibrate the OAK-D to Ouster extrinsic using the Spot robot calibration target.

THE TARGET IS A ChArUco BOARD, NOT A BARE APRILTAG.
  The physical panel is the Boston Dynamics "SPOT ROBOT CALIBRATION TARGET
  D5-00103A-001-A01": a 9 x 4 chequerboard whose light squares carry ArUco
  markers from DICT_4X4_50 with ids 0..17, giving 8 x 3 interior corners.
  Verified against a live /oak/rgb/image_raw/compressed frame on Botman:
  DICT_4X4_50 finds 17 of 18 markers and 23 of 24 ChArUco corners at a 0.47 px
  reprojection RMS, while DICT_APRILTAG_36h11 finds nothing at all.

  On OpenCV >= 4.6 the board must be built with setLegacyPattern(True). Without
  it the markers are still found but zero ChArUco corners are interpolated,
  because the marker-to-square mapping changed in 4.6.

MEASURE THE SQUARE SIZE. --square-size scales every translation below linearly.
  Put a ruler across the chequer squares and pass the real number.

Procedure:
1. Capture the board from several DIFFERENT orientations (>= 3, ideally 5-6).
2. For each capture, solve the board pose from all ChArUco corners (camera) and
   fit the board plane in the point cloud (LiDAR).
3. Solve the extrinsic from the plane correspondences:
       n_lidar = R_lc n_cam                        (rotation, needs >= 2 non-parallel normals)
       n_lidar . t_lc = d_lidar - d_cam            (translation, needs >= 3 spanning normals)

  A SINGLE capture cannot determine this transform. One plane fixes 3 of the 6
  DoF: rotation about the board normal and translation within the board plane
  are both free. Tilt and move the board between captures.

Usage:
    python3 calibrate_camera_lidar_panel.py --square-size 0.060 --captures 5
    python3 calibrate_camera_lidar_panel.py --identify-target    # sweep dictionaries and report
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
    import cv2.aruco as aruco
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# cv_bridge is only needed for uncompressed Image topics. Botman publishes only
# /oak/rgb/image_raw/compressed, which is decoded with cv2.imdecode, so a
# missing cv_bridge must not disable the whole module.
try:
    from cv_bridge import CvBridge
    HAVE_CV_BRIDGE = True
except ImportError:
    HAVE_CV_BRIDGE = False

# sensor_msgs_py is likewise optional: unpack_pointcloud2_xyz falls back to
# reading the raw buffer, which is what the Ouster layout allows anyway.
try:
    import sensor_msgs_py.point_cloud2 as pc2
    HAVE_PC2 = True
except ImportError:
    HAVE_PC2 = False

# Boston Dynamics Spot calibration target D5-00103A-001-A01.
SPOT_BOARD_SQUARES_X = 9
SPOT_BOARD_SQUARES_Y = 4
SPOT_BOARD_DICT = "DICT_4X4_50"
SPOT_BOARD_MARKER_RATIO = 0.75  # marker side / square side, measured 0.771 in-frame

# Nominal os_lidar -> oak-d-base-frame. Used only to decide WHICH plane in the
# cloud is the board, never as part of the answer.
NOMINAL_CAM_IN_LIDAR = np.array([0.03, 0.00, -0.22])

# Camera optical (+X right, +Y down, +Z forward) expressed in os_lidar
# (+X forward, +Y left, +Z up), before the mounting yaw below.
R_OPTICAL_TO_LIDAR = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)

# MEASURED ON BOTMAN: the OAK-D and the Ouster are mounted 180 degrees apart in
# yaw. The camera looks along os_lidar -X, not +X. Sweeping yaw in 15 degree
# steps against a live cloud, 180 deg put the panel plane 1.5 deg from the
# normal the camera predicts with 2771 inliers; every other hypothesis was 6.5
# deg or worse, and straight ahead (+X) held only empty floor.
# deploy/robots/botman.env still carries BOTMAN_OAK_YAW=0, which is wrong.
NOMINAL_YAW_DEG = 180.0


def yaw_matrix(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def nominal_lidar_cam(yaw_deg: float = NOMINAL_YAW_DEG) -> np.ndarray:
    return yaw_matrix(yaw_deg) @ R_OPTICAL_TO_LIDAR


# Backwards-compatible alias for the nominal orientation actually in use.
R_NOMINAL_LIDAR_CAM = nominal_lidar_cam()


def search_mounting_yaw(cloud: np.ndarray, t_cam_centre: np.ndarray, n_cam: np.ndarray,
                        ground_z: float) -> List[Tuple[float, int, float]]:
    """Which mounting yaw puts the panel where the camera says it is?

    Run when the board cannot be found at the assumed yaw: a 180 degree mount
    error looks exactly like 'the panel is not in the cloud'.
    """
    out = []
    for yaw in range(0, 360, 15):
        R = nominal_lidar_cam(float(yaw))
        centre = R @ t_cam_centre + NOMINAL_CAM_IN_LIDAR
        near = cloud[(np.linalg.norm(cloud - centre, axis=1) < 0.5) & (cloud[:, 2] > ground_z + 0.06)]
        if len(near) < 60:
            continue
        normal, inliers, _ = ransac_plane_fit(near, 800, 0.025)
        if abs(normal[2]) > 0.5:
            continue
        err = math.degrees(math.acos(min(1.0, abs(float(np.dot(normal, R @ n_cam))))))
        out.append((float(yaw), int(np.sum(inliers)), err))
    return sorted(out, key=lambda r: r[2])


def dictionary_by_name(name: str):
    attr = getattr(aruco, name, None)
    if attr is None:
        raise ValueError(f"OpenCV {cv2.__version__} has no aruco dictionary '{name}'")
    return aruco.getPredefinedDictionary(attr)


def make_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()  # OpenCV < 4.7 (4.5.4 on Humble)
    return aruco.DetectorParameters()


def detect_markers(gray_img: np.ndarray, aruco_dict, params):
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(aruco_dict, params).detectMarkers(gray_img)
    return aruco.detectMarkers(gray_img, aruco_dict, parameters=params)


def make_charuco_board(squares_x: int, squares_y: int, square_size: float,
                       marker_size: float, aruco_dict):
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(squares_x, squares_y, square_size, marker_size, aruco_dict)
    board = aruco.CharucoBoard((squares_x, squares_y), square_size, marker_size, aruco_dict)
    if hasattr(board, "setLegacyPattern"):
        # Required for boards produced before OpenCV 4.6, which includes the
        # Boston Dynamics panel. Verified: without it, 0 corners are found.
        board.setLegacyPattern(True)
    return board


def board_object_points(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float64)
    return np.asarray(board.chessboardCorners, dtype=np.float64)


def detect_charuco(gray: np.ndarray, board, aruco_dict, params):
    """Return (charuco_corners, charuco_ids, n_markers) across OpenCV versions."""
    if hasattr(aruco, "CharucoDetector"):
        ch_c, ch_i, m_c, m_i = aruco.CharucoDetector(board).detectBoard(gray)
        return ch_c, ch_i, 0 if m_i is None else len(m_i)
    m_c, m_i, _ = detect_markers(gray, aruco_dict, params)
    if m_i is None or len(m_i) == 0:
        return None, None, 0
    _, ch_c, ch_i = aruco.interpolateCornersCharuco(m_c, m_i, gray, board)
    return ch_c, ch_i, len(m_i)


def unpack_pointcloud2_xyz(msg) -> np.ndarray:
    """Extract xyz as float32. The raw-buffer path is the fast one and is valid
    for the Ouster layout (x, y, z at byte offsets 0, 4, 8)."""
    point_step = msg.point_step
    fields = {f.name: f for f in msg.fields}
    if all(k in fields for k in ("x", "y", "z")) and (
        fields["x"].offset, fields["y"].offset, fields["z"].offset
    ) == (0, 4, 8) and fields["x"].datatype == 7:
        n_points = len(msg.data) // point_step
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)
        xyz = np.frombuffer(raw[:, :12].tobytes(), dtype=np.float32).reshape(n_points, 3)
        return xyz[np.isfinite(xyz).all(axis=1)]

    if HAVE_PC2:
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
        if pts.ndim == 2 and pts.shape[1] >= 3:
            return pts[:, :3]
    return np.empty((0, 3), dtype=np.float32)


def ransac_plane_fit(pts: np.ndarray, max_iterations: int = 600,
                     distance_threshold: float = 0.02) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit the dominant plane. The returned normal points back toward the origin
    (n . centroid < 0), matching the camera normal convention used below."""
    if len(pts) < 10:
        return np.zeros(3), np.zeros(len(pts), dtype=bool), np.zeros(3)

    best_inliers = np.zeros(len(pts), dtype=bool)
    n_pts = len(pts)

    for _ in range(max_iterations):
        p1, p2, p3 = pts[np.random.choice(n_pts, 3, replace=False)]
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        distances = np.abs(pts @ normal - np.dot(normal, p1))
        inliers = distances < distance_threshold
        if np.sum(inliers) > np.sum(best_inliers):
            best_inliers = inliers

    if np.sum(best_inliers) < 10:
        return np.zeros(3), best_inliers, np.zeros(3)

    inlier_pts = pts[best_inliers]
    centroid = np.mean(inlier_pts, axis=0)
    _, _, Vt = np.linalg.svd(inlier_pts - centroid)
    refined_normal = Vt[2, :]
    if np.dot(refined_normal, centroid) > 0:
        refined_normal = -refined_normal
    return refined_normal, best_inliers, centroid


def euler_from_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy >= 1e-6:
        return math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0


def solve_rotation_svd(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Kabsch: returns R with dst ~= R @ src."""
    H = src.T @ dst
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


if HAVE_ROS2:
    class CameraLidarCalibratorNode(Node):
        def __init__(self, image_topic: str, camera_info_topic: str, lidar_topic: str) -> None:
            super().__init__("camera_lidar_calibrator")
            self.bridge = CvBridge() if HAVE_CV_BRIDGE else None
            self.camera_matrix: Optional[np.ndarray] = None
            self.dist_coeffs: Optional[np.ndarray] = None
            self.latest_image: Optional[np.ndarray] = None
            self.latest_cloud: Optional[np.ndarray] = None
            self.img_count = 0
            self.cloud_count = 0
            self.decode_failures = 0

            # BEST_EFFORT everywhere: compatible with RELIABLE and BEST_EFFORT
            # publishers alike, whereas a RELIABLE subscription gets nothing
            # from a BEST_EFFORT publisher.
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT)

            self.create_subscription(CameraInfo, camera_info_topic, self._on_info, qos)
            if image_topic.endswith("compressed"):
                self.create_subscription(CompressedImage, image_topic, self._on_compressed_image, qos)
            else:
                if not HAVE_CV_BRIDGE:
                    raise RuntimeError(
                        f"'{image_topic}' is a raw Image topic and cv_bridge is not installed. "
                        "Use the /compressed topic instead, or install ros-humble-cv-bridge.")
                self.create_subscription(Image, image_topic, self._on_raw_image, qos)
            self.create_subscription(PointCloud2, lidar_topic, self._on_lidar, qos)

        def _on_info(self, msg: CameraInfo) -> None:
            if self.camera_matrix is None:
                self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                self.dist_coeffs = (np.array(msg.d, dtype=np.float64) if len(msg.d) > 0
                                    else np.zeros(5, dtype=np.float64))

        def _on_raw_image(self, msg: Image) -> None:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.img_count += 1

        def _on_compressed_image(self, msg: CompressedImage) -> None:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.decode_failures += 1
                return
            self.latest_image = img
            self.img_count += 1

        def _on_lidar(self, msg: PointCloud2) -> None:
            self.latest_cloud = unpack_pointcloud2_xyz(msg)
            self.cloud_count += 1


def identify_target(gray: np.ndarray) -> None:
    """Report which marker dictionaries match what the camera is actually seeing."""
    print("\n  Sweeping every predefined dictionary against the current frame:")
    hits = []
    for name in sorted(n for n in dir(aruco) if n.startswith("DICT_")):
        try:
            d = aruco.getPredefinedDictionary(getattr(aruco, name))
            _, ids, _ = detect_markers(gray, d, make_detector_params())
        except Exception:
            continue
        if ids is not None and len(ids) > 0:
            hits.append((len(ids), name, sorted(ids.flatten().tolist())))
    if not hits:
        print("    No markers found in ANY dictionary. Check exposure, focus and framing.")
        return
    for n, name, ids in sorted(hits, reverse=True):
        print(f"    {name:<26} {n:3d} markers  ids={ids[:20]}")
    for size in ((8, 3), (9, 4), (6, 4)):
        ok, _ = cv2.findChessboardCorners(
            gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        print(f"    chequerboard {size}: {'found' if ok else 'not found'}")


def camera_board_plane(gray: np.ndarray, board, aruco_dict, params, K, dist,
                       min_corners: int) -> Tuple[Optional[dict], str]:
    ch_c, ch_i, n_markers = detect_charuco(gray, board, aruco_dict, params)
    if n_markers == 0:
        return None, "no ArUco markers in frame (wrong dictionary? board out of view?)"
    if ch_i is None or len(ch_i) < min_corners:
        got = 0 if ch_i is None else len(ch_i)
        return None, f"only {got} ChArUco corners from {n_markers} markers (need {min_corners})"

    all_corners = board_object_points(board)
    obj = all_corners[ch_i.flatten()].reshape(-1, 3)
    img = ch_c.reshape(-1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, "solvePnP failed on the ChArUco corners"

    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(((proj.reshape(-1, 2) - img) ** 2).sum(axis=1).mean()))
    if rms > 3.0:
        return None, f"reprojection RMS {rms:.2f} px is too high to trust"

    R_cb, _ = cv2.Rodrigues(rvec)
    t_cb = tvec.flatten()
    n_c = R_cb[:, 2].copy()
    d_c = float(np.dot(n_c, t_cb))
    if d_c > 0:  # orient the normal back toward the camera
        n_c, d_c = -n_c, -d_c

    # solvePnP returns the board ORIGIN, which is a corner of the panel. The
    # centre is what the LiDAR search must be aimed at: on this board the two
    # are half a metre apart, which is enough to point the search at the floor.
    # The interior-corner bounding box is centred on the board centre.
    centre_board = (all_corners.min(axis=0) + all_corners.max(axis=0)) / 2.0
    t_centre = R_cb @ centre_board + t_cb

    return {
        "n_cam": n_c, "d_cam": d_c, "t_cam_board": t_cb, "R_cam_board": R_cb,
        "t_cam_centre": t_centre,
        "n_corners": len(ch_i), "n_markers": n_markers, "reproj_rms": rms,
    }, ""


def lidar_board_plane(cloud: np.ndarray, expected_centre: np.ndarray,
                      expected_normal: np.ndarray, search_radius: float,
                      plane_band: float, max_tilt_cos: float,
                      ground_reject: float = 0.08) -> Tuple[Optional[dict], str]:
    """Fit the board plane, isolating it from the floor.

    Selection is done against the plane the camera predicts, not by height. The
    panel is used standing on the floor (the Spot manual allows its bottom edge
    within 5 cm of it), so a height-based ground cut either keeps the floor or
    throws the board away with it. The floor is close to perpendicular to the
    board, so a band about the predicted board plane removes nearly all of it
    while keeping the whole panel.

    Why this matters: inside a plain forward ROI the floor sits 0.616 m below
    os_lidar and supplies 65 percent of the points, and RANSAC returned normal
    [0.026, 0.002, 1.000] with 1990 inliers, i.e. the floor rather than the panel.
    """
    if len(cloud) == 0:
        return None, "empty point cloud"

    # Drop the floor first. Measured on Botman: of 1212 points inside a 0.9 m
    # sphere on the panel, 835 were floor, and a slab about the (near vertical)
    # board plane still left 218 floor points out of 357. Neither gate alone is
    # enough, because a vertical slab cuts a strip of floor rather than missing it.
    hist, edges = np.histogram(cloud[:, 2], bins=120)
    ground_z = float(edges[int(np.argmax(hist))])
    cloud = cloud[cloud[:, 2] > ground_z + ground_reject]

    near = cloud[np.linalg.norm(cloud - expected_centre, axis=1) < search_radius]
    if len(near) < 40:
        return None, (f"only {len(near)} non-floor points within {search_radius:.2f} m of the "
                      f"expected board centre {np.round(expected_centre, 2)} "
                      f"(floor at z = {ground_z:.2f} m)")

    n_exp = expected_normal / np.linalg.norm(expected_normal)
    d_exp = float(np.dot(n_exp, expected_centre))
    band = near[np.abs(near @ n_exp - d_exp) < plane_band]
    if len(band) < 30:
        return None, (f"only {len(band)} of {len(near)} nearby points lie within {plane_band:.2f} m "
                      "of the predicted board plane (is the nominal extrinsic or square size far off?)")

    normal, inliers, centroid = ransac_plane_fit(band, max_iterations=1200, distance_threshold=0.02)
    n_in = int(np.sum(inliers))
    if n_in < 30:
        return None, f"plane fit found only {n_in} inliers among {len(band)} candidate points"
    if abs(normal[2]) > max_tilt_cos:
        return None, (f"selected plane is horizontal (|n_z| = {abs(normal[2]):.2f}); "
                      "this is the floor, not the board. Stand the board more upright.")
    off = math.degrees(math.acos(float(np.clip(abs(np.dot(normal, n_exp)), -1, 1))))
    if off > 25.0:
        return None, (f"fitted plane is {off:.0f} deg from the one the camera predicts; "
                      "this is a wall or a desk, not the panel")

    return {
        "n_lidar": normal, "d_lidar": float(np.dot(normal, centroid)),
        "centroid": centroid, "n_inliers": n_in, "n_candidates": len(band),
    }, ""


def solve_extrinsic(captures: List[dict]) -> Optional[dict]:
    """Plane-correspondence solve. Needs >= 3 captures with spanning normals."""
    n_c = np.array([c["n_cam"] for c in captures])
    n_l = np.array([c["n_lidar"] for c in captures])
    d_c = np.array([c["d_cam"] for c in captures])
    d_l = np.array([c["d_lidar"] for c in captures])

    s_rot = np.linalg.svd(n_c, compute_uv=False)
    R_lc = solve_rotation_svd(n_c, n_l)

    # n_lidar . t = d_lidar - d_cam
    sol, _, rank, sv = np.linalg.lstsq(n_l, d_l - d_c, rcond=None)
    residual = n_l @ sol - (d_l - d_c)

    angle_err = [math.degrees(math.acos(np.clip(np.dot(R_lc @ a, b), -1, 1)))
                 for a, b in zip(n_c, n_l)]
    return {
        "R_lidar_cam": R_lc,
        "translation": sol,
        "rpy_rad": euler_from_matrix(R_lc),
        "normal_singular_values": s_rot,
        "translation_rank": int(rank),
        "translation_singular_values": sv,
        "plane_residuals_m": residual,
        "normal_angle_errors_deg": np.array(angle_err),
    }


def run_interactive_camera_lidar_calibration(
    image_topic: str = "/oak/rgb/image_raw/compressed",
    camera_info_topic: str = "/oak/rgb/camera_info",
    lidar_topic: str = "/ouster/points",
    squares_x: int = SPOT_BOARD_SQUARES_X,
    squares_y: int = SPOT_BOARD_SQUARES_Y,
    square_size: float = 0.1125,
    marker_size: Optional[float] = None,
    dictionary: str = SPOT_BOARD_DICT,
    captures: int = 5,
    min_corners: int = 8,
    search_radius: float = 0.7,
    plane_band: float = 0.20,
    nominal_yaw: float = NOMINAL_YAW_DEG,
    identify_only: bool = False,
) -> Optional[dict]:
    if not HAVE_CV2:
        print("ERROR: OpenCV with the aruco module is required "
              "(pip install opencv-contrib-python).", file=sys.stderr)
        return None
    if not HAVE_ROS2:
        print("ERROR: ROS 2 (rclpy, sensor_msgs) is required. Source the ROS setup first.",
              file=sys.stderr)
        return None

    if marker_size is None:
        marker_size = square_size * SPOT_BOARD_MARKER_RATIO

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = CameraLidarCalibratorNode(image_topic, camera_info_topic, lidar_topic)
    aruco_dict = dictionary_by_name(dictionary)
    params = make_detector_params()
    board = make_charuco_board(squares_x, squares_y, square_size, marker_size, aruco_dict)

    print("\n" + "=" * 65)
    print(" PHASE 3: CAMERA-TO-LIDAR CALIBRATION (SPOT ChArUco PANEL)")
    print("=" * 65)
    print(f"  Board: {squares_x} x {squares_y} squares, {dictionary}, "
          f"square {square_size * 1000:.1f} mm, marker {marker_size * 1000:.1f} mm")
    print(f"  OpenCV {cv2.__version__}")
    print("  Square size scales every translation linearly. Measure it with a ruler.")

    collected: List[dict] = []
    try:
        def wait_for_data(timeout: float = 20.0) -> bool:
            start = time.time()
            while rclpy.ok() and time.time() - start < timeout:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.camera_matrix is not None and node.latest_image is not None \
                        and node.latest_cloud is not None:
                    return True
                missing = []
                if node.camera_matrix is None:
                    missing.append(f"camera_info ({camera_info_topic})")
                if node.latest_image is None:
                    missing.append(f"image ({node.img_count} rx, {node.decode_failures} decode fails)")
                if node.latest_cloud is None:
                    missing.append(f"cloud ({node.cloud_count} rx)")
                sys.stdout.write(f"\r  waiting on: {', '.join(missing)}   ")
                sys.stdout.flush()
            print()
            return False

        if not wait_for_data():
            print("[ERROR] Never received all three of camera_info, image and point cloud.",
                  file=sys.stderr)
            print("        Check ROS_DOMAIN_ID (Botman uses 17) and that the topics above exist.",
                  file=sys.stderr)
            return None

        if identify_only:
            identify_target(cv2.cvtColor(node.latest_image, cv2.COLOR_BGR2GRAY))
            return None

        target = captures
        while len(collected) < target:
            i = len(collected) + 1
            print("\n" + "-" * 65)
            print(f"  CAPTURE {i} of {target}")
            if i == 1:
                print("  Place the board 1.5 - 2.5 m in front, upright, facing the robot.")
            else:
                print("  Now MOVE AND TILT the board to a clearly different orientation.")
                print("  Repeating the same pose adds no information: the solve needs")
                print("  non-parallel board normals.")
            ans = input(f"--> [ENTER] to capture, 's' to skip, 'q' to solve with {len(collected)}: ").strip().lower()
            if ans == "q":
                break
            if ans == "s":
                continue

            for _ in range(40):  # refresh to the current frame
                rclpy.spin_once(node, timeout_sec=0.05)

            gray = cv2.cvtColor(node.latest_image, cv2.COLOR_BGR2GRAY)
            cam, why = camera_board_plane(gray, board, aruco_dict, params,
                                          node.camera_matrix, node.dist_coeffs, min_corners)
            if cam is None:
                print(f"  [CAMERA FAILED] {why}")
                identify_target(gray)
                continue
            print(f"  [camera] {cam['n_markers']} markers, {cam['n_corners']} ChArUco corners, "
                  f"reproj RMS {cam['reproj_rms']:.2f} px")
            print(f"           board centre {np.round(cam['t_cam_centre'], 3)} m, "
                  f"range {np.linalg.norm(cam['t_cam_centre']):.3f} m")

            R_nom = nominal_lidar_cam(nominal_yaw)
            expected = R_nom @ cam["t_cam_centre"] + NOMINAL_CAM_IN_LIDAR
            expected_n = R_nom @ cam["n_cam"]
            lid, why = lidar_board_plane(node.latest_cloud, expected, expected_n,
                                         search_radius, plane_band, max_tilt_cos=0.5)
            if lid is None:
                print(f"  [LIDAR FAILED]  {why}")
                hist, edges = np.histogram(node.latest_cloud[:, 2], bins=120)
                gz = float(edges[int(np.argmax(hist))])
                cand = search_mounting_yaw(node.latest_cloud, cam["t_cam_centre"], cam["n_cam"], gz)
                if cand:
                    print("  Mounting-yaw search (which yaw puts the panel where the camera sees it):")
                    for yaw, n_in, err in cand[:3]:
                        flag = "  <-- assumed" if abs(yaw - nominal_yaw) < 1e-6 else ""
                        print(f"    yaw {yaw:5.1f} deg: {n_in:5d} inliers, normal off by {err:5.1f} deg{flag}")
                    if cand[0][2] < 5.0 and abs(cand[0][0] - nominal_yaw) > 1e-6:
                        print(f"    Re-run with --nominal-yaw {cand[0][0]:.0f}")
                continue
            print(f"  [lidar]  {lid['n_inliers']} inliers, centroid {np.round(lid['centroid'], 3)} m, "
                  f"normal {np.round(lid['n_lidar'], 3)}")

            cam.update(lid)
            collected.append(cam)
            if len(collected) >= 2:
                angles = [math.degrees(math.acos(np.clip(abs(np.dot(c["n_cam"], collected[0]["n_cam"])), -1, 1)))
                          for c in collected[1:]]
                print(f"  orientation spread vs capture 1: "
                      f"{', '.join('%.0f deg' % a for a in angles)}")

        if len(collected) < 3:
            print(f"\n[ERROR] Only {len(collected)} usable captures. The plane-correspondence solve "
                  "needs at least 3 with different board orientations.", file=sys.stderr)
            return None

        results = solve_extrinsic(collected)
        t = results["translation"]
        rpy = results["rpy_rad"]
        sv_n = results["normal_singular_values"]

        print("\n" + "=" * 65)
        print(f" SOLVED EXTRINSIC from {len(collected)} captures (os_lidar -> oak-d-base-frame)")
        print("=" * 65)
        print(f"  Translation (x, y, z): [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
        print(f"  Euler RPY (rad):       [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")
        print(f"  Euler RPY (deg):       [{math.degrees(rpy[0]):.2f}, "
              f"{math.degrees(rpy[1]):.2f}, {math.degrees(rpy[2]):.2f}]")
        print(f"  Board normal spread (singular values): {np.round(sv_n, 4)}")
        if sv_n[1] < 0.15 * sv_n[0]:
            print("  [WARN] The board normals are nearly parallel. Rotation about the board")
            print("         normal is poorly constrained. Re-run with the board tilted more.")
        if results["translation_rank"] < 3:
            print(f"  [WARN] Translation rank {results['translation_rank']}/3: the component")
            print("         perpendicular to the span of the board normals is NOT determined.")
        print(f"  Per-plane residual: {np.round(results['plane_residuals_m'], 4)} m")
        print(f"  Normal misalignment: {np.round(results['normal_angle_errors_deg'], 2)} deg")
        results["captures"] = collected
        return results
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--image-topic", default="/oak/rgb/image_raw/compressed")
    p.add_argument("--camera-info-topic", default="/oak/rgb/camera_info")
    p.add_argument("--lidar-topic", default="/ouster/points")
    p.add_argument("--squares-x", type=int, default=SPOT_BOARD_SQUARES_X)
    p.add_argument("--squares-y", type=int, default=SPOT_BOARD_SQUARES_Y)
    p.add_argument("--square-size", type=float, default=0.1125,
                   help="Chequer square side in metres (default 0.1125, derived from the panel's "
                        "documented 118.5 x 50 cm outer size; confirm with a tape across the 9 "
                        "squares, which should read about 101.5 cm)")
    p.add_argument("--marker-size", type=float, default=None,
                   help=f"Marker side in metres (default: square-size * {SPOT_BOARD_MARKER_RATIO})")
    p.add_argument("--dictionary", default=SPOT_BOARD_DICT)
    p.add_argument("--captures", type=int, default=5)
    p.add_argument("--min-corners", type=int, default=8)
    p.add_argument("--search-radius", type=float, default=0.7,
                   help="Radius around the nominal board position used to isolate it in the cloud")
    p.add_argument("--nominal-yaw", type=float, default=NOMINAL_YAW_DEG,
                   help="Assumed camera-to-lidar mounting yaw in degrees, used only to find the "
                        "panel in the cloud (default 180, measured on Botman)")
    p.add_argument("--plane-band", type=float, default=0.20,
                   help="Half-thickness of the slab about the predicted board plane used to "
                        "separate the panel from the floor")
    p.add_argument("--identify-target", action="store_true",
                   help="Report which dictionaries match the current frame, then exit")
    a = p.parse_args()

    run_interactive_camera_lidar_calibration(
        image_topic=a.image_topic, camera_info_topic=a.camera_info_topic, lidar_topic=a.lidar_topic,
        squares_x=a.squares_x, squares_y=a.squares_y, square_size=a.square_size,
        marker_size=a.marker_size, dictionary=a.dictionary, captures=a.captures,
        min_corners=a.min_corners, search_radius=a.search_radius,
        plane_band=a.plane_band, nominal_yaw=a.nominal_yaw, identify_only=a.identify_target,
    )


if __name__ == "__main__":
    main()
