"""Exercise real Open3D voxel centroids and the native input buffer contract."""

import numpy as np
import open3d
import pytest

from cslam.lidar_pr import icp_utils


def sorted_points(cloud):
    points = np.asarray(cloud.points)
    return points[np.lexsort(points.T)]


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_downsample_preserves_voxel_centroids_and_filters_nonfinite(dtype):
    rng = np.random.default_rng(193)
    points = rng.normal(size=(1000, 3)).astype(dtype)
    points[0, 0] = np.nan
    points[1, 2] = np.inf
    expected = open3d.geometry.PointCloud()
    expected.points = open3d.utility.Vector3dVector(
        points[np.isfinite(points).all(axis=1)].astype(np.float64)
    )
    expected = expected.voxel_down_sample(voxel_size=0.5)
    actual = icp_utils.downsample(points, 0.5)
    np.testing.assert_array_equal(sorted_points(actual), sorted_points(expected))


def test_downsample_passes_native_contiguous_double_buffer(monkeypatch):
    original = open3d.utility.Vector3dVector
    buffers = []

    def capture(points):
        buffers.append(points)
        return original(points)

    monkeypatch.setattr(open3d.utility, "Vector3dVector", capture)
    points = np.arange(300, dtype=np.float32).reshape(-1, 3)[::2]
    result = icp_utils.downsample(points, 0.1)
    assert len(result.points) == 50
    assert buffers[0].dtype == np.float64
    assert buffers[0].flags.c_contiguous
