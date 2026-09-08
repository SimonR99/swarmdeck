"""Run in the MGG image: validate depth axes, padding, range and frame retention."""
import importlib.util
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

spec = importlib.util.spec_from_file_location('inputs', '/app/deploy/mgg/inputs.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
node = module.Inputs.__new__(module.Inputs)
node.latest = None
node.cloud = Mock()
info = CameraInfo(width=8, height=8)
info.k = [4., 0., 4., 0., 4., 4., 0., 0., 1.]
node.intrinsics = info
for endian in (False, True):
    depth = Image(width=8, height=8, step=40, encoding='32FC1', is_bigendian=endian)
    depth.header.frame_id = 'robot_0/base_link/camera'
    depth.header.stamp.sec = 42
    data = np.full((8, 10), 2., dtype='>f4' if endian else '<f4')
    data[0, 0] = np.nan
    data[4, 4] = 30.
    depth.data = data.tobytes()
    node.depth = depth
    node.publish_cloud()
    cloud = node.cloud.publish.call_args.args[0]
    points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3)
    np.testing.assert_allclose(points, [[2., 0., 2.], [2., 2., 0.]])
    assert cloud.header == depth.header and cloud.width == 2
    count = node.cloud.publish.call_count
    node.publish_cloud()
    assert node.cloud.publish.call_count == count  # no stale depth replay
print('MGG depth projection smoke passed')
