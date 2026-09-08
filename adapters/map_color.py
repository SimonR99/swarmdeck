"""Bounded, capture-time camera/LiDAR color joins shared by ROS 1 and ROS 2.

Keep compressed images locally: registered scans commonly arrive half a second
late. Decode only for a display sample or an accepted keyframe, never every scan.
No images are sent to the server. Unknown colors retain an explicit alpha mask.
"""

from __future__ import annotations

from collections import deque
import threading

import numpy as np

from adapters.keyframe_producer import se3_from_quat_xyz
from adapters.perception.depth_projection import transform_points
from adapters.reconstruction import colorize_points
from adapters.runtime import stamp_seconds


class CameraColorizer:
    def __init__(self, config):
        self.enabled = bool(config.get("enabled", False))
        self.max_age_s = float(config.get("max_age_s", 0.05))
        self.history_s = float(config.get("history_s", 2.0))
        self.camera_frame = str(config.get("camera_frame", "")).lstrip("/")
        # Both byte and frame limits matter for arbitrary camera resolutions.
        self._images = deque(maxlen=60)
        self._bytes = 0
        self._lock = threading.Lock()
        self._cached = None
        self.last_status = "waiting for image" if self.enabled else "disabled"

    def push(self, jpeg, header):
        if not self.enabled:
            return
        stamp = stamp_seconds(header)
        if stamp is None or len(jpeg) > 16 * 1024 * 1024:
            return
        with self._lock:
            # A ROS clock reset starts a new capture epoch.
            if self._images and stamp < self._images[-1][0] - self.history_s:
                self._images.clear()
                self._bytes = 0
            while self._images and (
                len(self._images) == self._images.maxlen
                or self._bytes + len(jpeg) > 16 * 1024 * 1024
                or stamp - self._images[0][0] > self.history_s
            ):
                self._bytes -= len(self._images.popleft()[1])
            self._images.append((stamp, jpeg, header))
            self._bytes += len(jpeg)

    def colorize(self, points, cloud_header, info, lookup):
        """Return RGBA or None; lookup(image_header) returns camera <- map TF.

        CameraInfo must describe the RGB topic's actual pixels, including lens
        distortion. An explicit camera_frame permits an empty CameraInfo frame
        (OAK driver bug), but never overrides a conflicting nonempty frame.
        """
        if not self.enabled:
            return None
        stamp = stamp_seconds(cloud_header)
        with self._lock:
            image = (
                min(self._images, key=lambda item: abs(item[0] - stamp))
                if self._images and stamp is not None
                else None
            )
        if image is None or info is None:
            self.last_status = "missing image or RGB calibration"
            return None
        if abs(image[0] - stamp) > self.max_age_s:
            self.last_status = "no image near scan timestamp"
            return None
        _, jpeg, header = image
        frame = str(header.frame_id).lstrip("/")
        info_frame = str(info.header.frame_id).lstrip("/")
        if (
            not frame
            or (self.camera_frame and frame != self.camera_frame)
            or (info_frame or self.camera_frame) != frame
        ):
            self.last_status = "RGB calibration frame mismatch"
            return None
        try:
            import cv2

            # ROS 1 subscribers may run concurrently; a cache entry is immutable.
            cached = self._cached
            if cached is None or cached[0] is not jpeg:
                bgr = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if bgr is None:
                    self.last_status = "invalid JPEG"
                    return None
                cached = (jpeg, bgr[:, :, ::-1])
                self._cached = cached
            rgb = cached[1]
            if rgb.shape[:2] != (info.height, info.width):
                self.last_status = "RGB calibration dimensions mismatch"
                return None
            tf = lookup(header)
            basis = transform_points(
                np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                ),
                tf.transform,
            )
            transform = np.eye(4)
            transform[:3, 3] = basis[0]
            transform[:3, :3] = (basis[1:] - basis[0]).T
            k = info.k if hasattr(info, "k") else info.K
            d = info.d if hasattr(info, "d") else info.D
            colors, visible = colorize_points(
                points,
                rgb,
                np.asarray(k).reshape(3, 3),
                transform,
                distortion=d,
                distortion_model=info.distortion_model,
            )
            self.last_status = "colored" if visible.any() else "no visible points"
            return (
                np.column_stack((colors, visible.astype(np.uint8) * 255))
                if visible.any()
                else None
            )
        except Exception as exc:
            # Best effort: missing historical TF or malformed calibration cannot
            # interrupt map geometry/telemetry. Inspect last_status for diagnosis.
            self.last_status = f"projection unavailable: {exc}"
            return None


class MapColorMixin:
    """Small adapter seam: subclasses provide a capture-time ROS TF lookup."""

    def _remember_mapping_image(self, jpeg, header):
        colorizer = getattr(self, "_map_color", None)
        if colorizer is not None:
            colorizer.push(jpeg, header)

    def _colorize_map_rgba(self, points, header):
        colorizer = getattr(self, "_map_color", None)
        if colorizer is None:
            return None
        # Never substitute depth CameraInfo for RGB calibration.
        return colorizer.colorize(
            points, header, self._camera_color_info, self._lookup_color_transform
        )

    def _colorize_map(self, points, header):
        rgba = self._colorize_map_rgba(points, header)
        return rgba[:, :3].copy() if rgba is not None else None

    def _keyframe_colorizer(self, pose, header):
        if not getattr(self, "_map_color", None) or not self._map_color.enabled:
            return None

        # KeyframeUploader invokes this only AFTER novelty/motion gating and
        # downsampling. Convert its base points back to the registered map frame.
        def project(base_points):
            transform = se3_from_quat_xyz(pose)
            return self._colorize_map_rgba(
                base_points @ transform[:3, :3].T + transform[:3, 3], header
            )

        return project
