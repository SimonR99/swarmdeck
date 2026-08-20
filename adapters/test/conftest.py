"""Import adapter_sim without a sourced ROS workspace.

adapter_sim imports the whole ROS stack at module scope, but a good deal of what
it does is pure logic — SE(2) composition, reset ordering, service discovery —
and that deserves to run in `make test` rather than only inside the Gazebo image.

The stub list lives here rather than in one test file because it is a shared
hazard: every ROS import added to adapter_sim breaks every test module that
stubs it, and finding out twice is once too often.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

REPO = Path(__file__).resolve().parents[2]

import numpy as np

class _Cv2Stub:
    IMWRITE_JPEG_QUALITY = 1
    COLOR_RGB2BGR = 1
    COLOR_RGBA2BGR = 2
    COLOR_BGRA2BGR = 3

    @staticmethod
    def cvtColor(frame, code):
        return frame

    @staticmethod
    def imencode(ext, frame, params=None):
        return True, np.array([0xFF, 0xD8, 0xFF, 0xD9], dtype=np.uint8)


if "cv2" not in sys.modules and importlib.util.find_spec("cv2") is None:
    sys.modules["cv2"] = _Cv2Stub()

# Submodules must be listed individually: `from nav2_msgs.srv import X` imports
# `nav2_msgs.srv`, and a MagicMock parent does not make its children importable.
STUBBED_ROS_MODULES = [
    "websockets", "rclpy", "rclpy.action", "rclpy.node", "rclpy.parameter",
    "rclpy.qos",
    "action_msgs", "action_msgs.msg", "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg", "nav2_msgs", "nav2_msgs.action", "nav2_msgs.srv",
    "robot_localization", "robot_localization.srv",
    "sensor_msgs", "sensor_msgs.msg", "tf2_msgs", "tf2_msgs.msg",
    "std_msgs", "std_msgs.msg",
    # Not imported at module scope — the SLAM reset resolves whichever back end
    # is running through importlib. Stubbed anyway, because without them that
    # lookup silently finds nothing and the reset "fails" for the wrong reason.
    "slam_toolbox", "slam_toolbox.srv", "std_srvs", "std_srvs.srv",
]


@pytest.fixture(scope="module")
def sim_module():
    """The imported adapter_sim module, with its ROS imports stubbed."""
    saved = {name: sys.modules.get(name) for name in STUBBED_ROS_MODULES}
    for name in STUBBED_ROS_MODULES:
        sys.modules[name] = MagicMock()
    sys.path.insert(0, str(REPO / "adapters" / "adapter_sim"))
    try:
        yield importlib.import_module("adapter_sim")
    finally:
        sys.path.remove(str(REPO / "adapters" / "adapter_sim"))
        sys.modules.pop("adapter_sim", None)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value
