"""Run in the MGG image: validate depth axes, padding, range and frame retention."""

import importlib.util
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

spec = importlib.util.spec_from_file_location("inputs", "/app/deploy/mgg/inputs.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
node = module.Inputs.__new__(module.Inputs)
node.latest = None
node.sim_depth = True
node.base_frame = ""
node.cloud = Mock()
info = CameraInfo(width=8, height=8)
info.header.frame_id = "robot_0/base_link/camera"
info.k = [4.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 0.0, 1.0]
node.intrinsics = info
for endian in (False, True):
    depth = Image(width=8, height=8, step=40, encoding="32FC1", is_bigendian=endian)
    depth.header.frame_id = "robot_0/base_link/camera"
    depth.header.stamp.sec = 42
    data = np.full((8, 10), 2.0, dtype=">f4" if endian else "<f4")
    data[0, 0] = np.nan
    data[4, 4] = 30.0
    depth.data = data.tobytes()
    node.depth = depth
    node.publish_cloud()
    cloud = node.cloud.publish.call_args.args[0]
    points = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
    np.testing.assert_allclose(points, [[2.0, 0.0, 2.0], [2.0, 2.0, 0.0]])
    assert cloud.header == depth.header and cloud.width == 2
    count = node.cloud.publish.call_count
    node.publish_cloud()
    assert node.cloud.publish.call_count == count  # no stale depth replay
print("MGG depth projection smoke passed")

# Matching TF may arrive after odometry. Retry the original timestamp, and
# expire unavailable history without using a newer transform.
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

node.pending_odom = deque(maxlen=10)
node.frame = "robot_0/map_frame"
node.odom = Mock()
node.buffer = Mock()
node.get_clock = Mock()
node.get_clock.return_value.now.return_value.nanoseconds = 42_500_000_000
odom = Odometry()
odom.header.stamp.sec = 42
odom.child_frame_id = "robot_0/base_link"
node.on_odom(odom)
node.buffer.lookup_transform.side_effect = TransformException("TF not here yet")
node.publish_odom()
assert len(node.pending_odom) == 1
node.odom.publish.assert_not_called()
tf = TransformStamped()
tf.transform.translation.x = 3.0
node.buffer.lookup_transform.side_effect = None
node.buffer.lookup_transform.return_value = tf
node.publish_odom()
assert not node.pending_odom
output = node.odom.publish.call_args.args[0]
assert output.header.stamp == odom.header.stamp
assert output.header.frame_id == node.frame and output.pose.pose.position.x == 3.0
assert node.buffer.lookup_transform.call_args.args[2].nanoseconds == 42_000_000_000
node.on_odom(odom)
node.get_clock.return_value.now.return_value.nanoseconds = 44_000_000_000
node.buffer.lookup_transform.side_effect = TransformException("history expired")
node.publish_odom()
assert not node.pending_odom and node.odom.publish.call_count == 1
print("MGG delayed TF smoke passed")

# Hardware uses millimetre depth and optical axes, with no invented camera TF.
node.sim_depth = False
info.header.frame_id = "camera_optical_frame"
depth = Image(width=8, height=8, step=16, encoding="16UC1")
depth.header.frame_id = info.header.frame_id
depth.data = np.full((8, 8), 2000, dtype="<u2").tobytes()
node.depth = depth
node.publish_cloud()
cloud = node.cloud.publish.call_args.args[0]
points = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
np.testing.assert_allclose(
    points, [[-2.0, -2.0, 2.0], [0.0, -2.0, 2.0], [-2.0, 0.0, 2.0], [0.0, 0.0, 2.0]]
)
assert cloud.header.frame_id == "camera_optical_frame"
node.base_frame = "chassis"
node.buffer.lookup_transform.side_effect = None
node.buffer.lookup_transform.return_value = tf
node.on_odom(odom)
node.publish_odom()
assert node.buffer.lookup_transform.call_args.args[1] == "chassis"
assert node.odom.publish.call_args.args[0].child_frame_id == "chassis"
print("MGG hardware depth and chassis-frame smoke passed")
