"""Run in the MGG image: validate depth axes, padding, range and frame retention."""

import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

spec = importlib.util.spec_from_file_location("inputs", "/app/deploy/mgg/inputs.py")
sys.path.insert(0, str(Path(spec.origin).parent))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
node = module.Inputs.__new__(module.Inputs)
node.latest = None
node.sim_depth = True
node.mapping_max_range = 20.0
node.surface_budget_warning = False
node.base_frame = ""
node.cloud = Mock()
node.depth = None
node.intrinsics = None

# ROS zero time asks TF2 for the latest transform and must never enter the
# mapping buffers. A later invalid frame also cannot overwrite pending valid
# capture-time data.
zero_cloud = PointCloud2()
node.on_cloud(zero_cloud)
assert node.latest is None
valid_cloud = PointCloud2()
valid_cloud.header.stamp.nanosec = 1
node.on_cloud(valid_cloud)
node.on_cloud(zero_cloud)
assert node.latest is valid_cloud
node.publish_cloud()
assert node.cloud.publish.call_args.args[0].header == valid_cloud.header
assert node.latest is None

zero_depth = Image()
node.on_depth(zero_depth)
assert node.depth is None
valid_depth = Image()
valid_depth.header.stamp.sec = 1
node.on_depth(valid_depth)
node.on_depth(zero_depth)
assert node.depth is valid_depth
node.depth = None
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
    assert cloud.header == depth.header and cloud.width == 15
    np.testing.assert_allclose(points[0], [2.0, 1.0, 2.0])
    np.testing.assert_allclose(points[-1], [2.0, -1.0, -1.0])
    assert any(np.allclose(point, [30.0, 0.0, 0.0]) for point in points)
    count = node.cloud.publish.call_count
    node.publish_cloud()
    assert node.cloud.publish.call_count == count  # no stale depth replay
print("MGG depth projection smoke passed")

# Realistic organized road: keep clear-to-range rays and fill sparse ground
# support between original pixels without changing the camera stamp/frame.
info = CameraInfo(width=320, height=240)
info.header.frame_id = "robot_0/base_link/camera"
focal = 240 / (2 * np.tan(np.pi / 6))
info.k = [focal, 0.0, 160.0, 0.0, focal, 120.0, 0.0, 0.0, 1.0]
rows = np.arange(240)[:, None]
road = (
    np.broadcast_to(
        np.where(rows > 120, 0.3 * focal / np.maximum(rows - 120, 1), 40.0),
        (240, 320),
    )
    .astype("<f4")
    .copy()
)
road = np.minimum(road, 40.0)
depth = Image(width=320, height=240, step=1280, encoding="32FC1")
depth.header.frame_id = info.header.frame_id
depth.header.stamp.sec = 43
depth.data = road.tobytes()
node.intrinsics, node.depth = info, depth
node.publish_cloud()
cloud = node.cloud.publish.call_args.args[0]
points = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
assert cloud.width > 19200
assert np.count_nonzero(points[:, 0] == 40.0) > 0
# The pixel grid lacks this exact forward floor interval; surface samples fill it.
assert np.any(
    (np.abs(points[:, 0] - 12.0) < 0.1)
    & (np.abs(points[:, 1]) < 0.1)
    & (np.abs(points[:, 2] + 0.3) < 0.001)
)
assert cloud.header == depth.header
from unittest.mock import patch

node.depth = depth
node.get_logger = Mock()
with patch.object(
    module, "rasterize_flat_depth_quads", side_effect=ValueError("budget")
):
    node.publish_cloud()
fallback = node.cloud.publish.call_args.args[0]
assert fallback.width == 19200
assert np.any(np.frombuffer(fallback.data, dtype="<f4").reshape(-1, 3)[:, 0] == 40.0)
print("MGG organized surface integration and raw-ray fallback smoke passed")
# Restore the small fixture for the hardware projection checks below.
info = CameraInfo(width=8, height=8)
info.header.frame_id = "robot_0/base_link/camera"
info.k = [4.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 0.0, 1.0]
node.intrinsics = info

# Matching TF may arrive after odometry. Retry the original timestamp, and
# expire unavailable history without using a newer transform.
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

node.pending_odom = deque(maxlen=10)
node.frame = "robot_0/navigation_frame"
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

# MOLA still needs odometry/TF, but must not receive or convert duplicate clouds.
module.rclpy.init(
    args=["--ros-args", "-p", "cloud_enabled:=false", "-p", "depth_enabled:=true"]
)
relay = module.Inputs()
try:
    subscriptions = {subscription.topic_name for subscription in relay.subscriptions}
    assert "/input_odometry" in subscriptions
    assert not subscriptions.intersection({"/input_cloud", "/depth", "/camera_info"})
    assert sum(timer.timer_period_ns == 100_000_000 for timer in relay.timers) == 1
    assert all(timer.timer_period_ns != 500_000_000 for timer in relay.timers)
finally:
    relay.destroy_node()
    module.rclpy.shutdown()
print("MGG MOLA odometry-only relay smoke passed")
