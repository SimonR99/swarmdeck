"""Re-export of adapters.colorize for deploy/autonomy/cslam_bridge.py.

Delete once cslam_bridge.py imports adapters.colorize; the peer-runtime move
does that after the C-SLAM lane merges.
"""

from adapters.colorize import colorize_points, colorize_ros_rgbd  # noqa: F401
