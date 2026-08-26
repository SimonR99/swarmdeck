#!/usr/bin/env python3
"""Build one occupancy grid from Swarm-SLAM's own keyframes and optimised poses.

    ros2 run swarmdeck_cslam cslam_grid.py --ros-args -p robots:=4

Why this exists — the whole point of the node.

The merge used to take occupancy grids from RTAB-Map and transforms from cslam.
Those are two independent SLAM systems with separately optimised trajectories,
and they disagree: measured on a live four-robot run, cslam and RTAB-Map placed
the *same robot* 8.1 m apart in what is nominally that robot's own map frame,
and 6.8-19.5 m apart for robots in the shared cluster frame. Merging across that
gap put every robot 11-16 m from ground truth while plain grid registration
managed 0.03-0.20 m (docs/operations/known-issues.md).

The fix is not a better transform. It is to stop mixing the two systems: take
BOTH the geometry and the poses from cslam. Each keyframe cloud is rendered at
the pose the joint optimiser assigned to that exact keyframe, so the grid and
the transform are the same estimate by construction and cannot drift apart.
When the back end re-optimises, every keyframe moves with it and the map is
rebuilt — which is what "one robot's observations correct another's" actually
looks like on a map.

Published as a `nav_msgs/OccupancyGrid` on `/cslam/map`, the same type and
convention `<ns>/map` already uses, so the adapter uploads it through the
existing path and nothing downstream learns a new format.

Cost: the grid is rebuilt from all keyframes whenever the optimiser reports, at
`rebuild_period_s` at most. That is O(points) per rebuild and deliberately not
incremental — an optimisation can move every past keyframe, so an incremental
grid would bake in poses that are no longer current, which is the bug this node
exists to remove.
"""

from __future__ import annotations

import math
import struct

import numpy as np
import rclpy
from cslam_common_interfaces.msg import PoseGraph, VizPointCloud
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def cloud_xyz(msg) -> np.ndarray:
    """xyz from a PointCloud2, reading field offsets rather than assuming them."""
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if len(offsets) != 3 or not msg.point_step:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = len(raw) // msg.point_step
    if not count:
        return np.zeros((0, 3), dtype=np.float32)
    rows = raw[: count * msg.point_step].reshape(count, msg.point_step)
    cols = [
        rows[:, offsets[a] : offsets[a] + 4].copy().view(np.float32).ravel()
        for a in ("x", "y", "z")
    ]
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


class CslamGrid(Node):
    def __init__(self) -> None:
        super().__init__("cslam_grid")
        self.declare_parameter("robots", 4)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("size_m", 30.0)
        self.declare_parameter("rebuild_period_s", 3.0)
        # Only points in this height band become obstacles. Below is floor,
        # above is ceiling; both would fill the map with false walls.
        self.declare_parameter("min_height", 0.12)
        self.declare_parameter("max_height", 1.60)

        count = int(self.get_parameter("robots").value)
        self.res = float(self.get_parameter("resolution").value)
        size = float(self.get_parameter("size_m").value)
        self.n = int(size / self.res)
        self.origin = (-size / 2.0, -size / 2.0)
        self.min_h = float(self.get_parameter("min_height").value)
        self.max_h = float(self.get_parameter("max_height").value)

        # (robot_id, keyframe_id) -> cloud in that keyframe's own sensor frame.
        self.clouds: dict[tuple[int, int], np.ndarray] = {}
        # (robot_id, keyframe_id) -> optimised pose in the common frame.
        self.poses: dict[tuple[int, int], tuple[float, float, float]] = {}
        self.dirty = False

        for i in range(count):
            self.create_subscription(
                VizPointCloud,
                f"/r{i}/cslam/viz/keyframe_pointcloud",
                self._on_cloud,
                100,
            )
        self.create_subscription(
            VizPointCloud, "/cslam/viz/keyframe_pointcloud", self._on_cloud, 100
        )
        self.create_subscription(PoseGraph, "/cslam/viz/pose_graph", self._on_graph, 10)
        self.create_subscription(PoseGraph, "/cslam/pose_graph", self._on_graph, 10)

        # Latched, like every other map topic here: a late subscriber gets the
        # current map instead of waiting for the next rebuild.
        self.pub = self.create_publisher(
            OccupancyGrid,
            "/cslam/map",
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_timer(
            float(self.get_parameter("rebuild_period_s").value), self._rebuild
        )
        self.get_logger().info(
            f"building a cslam-native grid for {count} robots "
            f"({self.n}x{self.n} @ {self.res} m)"
        )

    def _on_cloud(self, msg: VizPointCloud) -> None:
        pts = cloud_xyz(msg.pointcloud)
        if len(pts):
            self.clouds[(int(msg.robot_id), int(msg.keyframe_id))] = pts
            self.dirty = True

    def _on_graph(self, msg: PoseGraph) -> None:
        for value in msg.values:
            self.poses[(int(value.key.robot_id), int(value.key.keyframe_id))] = (
                value.pose.position.x,
                value.pose.position.y,
                yaw_of(value.pose.orientation),
            )
        self.dirty = True

    def _rebuild(self) -> None:
        """Render every keyframe at its CURRENT optimised pose."""
        if not self.dirty:
            return
        self.dirty = False
        usable = [k for k in self.clouds if k in self.poses]
        if not usable:
            return

        grid = np.full((self.n, self.n), UNKNOWN, dtype=np.int8)
        for key in usable:
            pts = self.clouds[key]
            band = pts[(pts[:, 2] >= self.min_h) & (pts[:, 2] <= self.max_h)]
            if not len(band):
                continue
            tx, ty, yaw = self.poses[key]
            c, s = math.cos(yaw), math.sin(yaw)
            wx = tx + band[:, 0] * c - band[:, 1] * s
            wy = ty + band[:, 0] * s + band[:, 1] * c

            gx = ((wx - self.origin[0]) / self.res).astype(np.int32)
            gy = ((wy - self.origin[1]) / self.res).astype(np.int32)
            ok = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
            if not ok.any():
                continue

            # Free space along each ray, then the endpoint as occupied. Without
            # the free pass the map is a point cloud rendered as walls with no
            # traversable space, which Nav2 cannot plan through.
            sx = int((tx - self.origin[0]) / self.res)
            sy = int((ty - self.origin[1]) / self.res)
            if 0 <= sx < self.n and 0 <= sy < self.n:
                self._mark_free(grid, sx, sy, gx[ok], gy[ok])
            grid[gy[ok], gx[ok]] = OCCUPIED

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cslam_map"
        msg.info.resolution = self.res
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        self.pub.publish(msg)

    def _mark_free(self, grid, sx: int, sy: int, gx, gy) -> None:
        """Mark cells between the sensor and each endpoint as free.

        Samples along each ray rather than walking Bresenham per cell: with tens
        of thousands of points per keyframe the exact traversal is far too slow
        to run inside a rebuild, and a sampled ray leaves at most single-cell
        gaps which the occupied pass writes over anyway.
        """
        steps = 48
        t = np.linspace(0.0, 1.0, steps, endpoint=False)[1:]
        fx = (sx + np.outer(gx - sx, t)).astype(np.int32).ravel()
        fy = (sy + np.outer(gy - sy, t)).astype(np.int32).ravel()
        ok = (fx >= 0) & (fx < self.n) & (fy >= 0) & (fy < self.n)
        flat = fy[ok].astype(np.int64) * self.n + fx[ok].astype(np.int64)
        cells = grid.reshape(-1)
        # Only unknown becomes free; never overwrite an obstacle another
        # keyframe has already seen.
        unknown = cells[flat] == UNKNOWN
        cells[flat[unknown]] = FREE


def main() -> None:
    rclpy.init()
    node = CslamGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
