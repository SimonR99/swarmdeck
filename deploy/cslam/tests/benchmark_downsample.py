"""Compare float32 ROS cloud conversion before and after the buffer fix."""

from time import perf_counter

import numpy as np
import open3d

from cslam.lidar_pr.icp_utils import downsample


def before(points, voxel_size):
    points = points[np.isfinite(points).all(axis=1)]
    cloud = open3d.geometry.PointCloud()
    cloud.points = open3d.utility.Vector3dVector(points)
    return cloud.voxel_down_sample(voxel_size=voxel_size)


def median_ms(function, points):
    samples = []
    for _ in range(10):
        start = perf_counter()
        function(points, 0.25)
        samples.append((perf_counter() - start) * 1000)
    return round(float(np.median(samples)), 3)


rng = np.random.default_rng(321)
for count in (10000, 50000, 100000):
    points = rng.uniform(-40, 40, size=(count, 3)).astype(np.float32)
    print(
        f"points={count}: old={median_ms(before, points)}ms "
        f"vectorized={median_ms(downsample, points)}ms",
        flush=True,
    )
