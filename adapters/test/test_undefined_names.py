"""No adapter module may reference a name it never defines or imports.

The hardware adapters' unit tests build bridges with ``__new__`` and stub ROS,
so a lost ``from nav_msgs.msg import Odometry`` passed every test while
``HardwareBridge.__init__`` raised NameError on the robot. Pyflakes finds that
class of error statically, without importing ROS.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
CHECKED = [
    *sorted((REPO / "adapters").rglob("*.py")),
    REPO / "swarmdeck_ros/src/swarmdeck_sim/nodes/swarmdeck_argos_bridge.py",
]


def test_adapter_modules_have_no_undefined_names():
    api = pytest.importorskip("pyflakes.api")
    messages = pytest.importorskip("pyflakes.messages")

    class Collect:
        def __init__(self):
            self.found = []

        def flake(self, message):
            if isinstance(message, messages.UndefinedName):
                self.found.append(str(message))

        def unexpectedError(self, filename, message):
            self.found.append(f"{filename}: {message}")

        def syntaxError(self, filename, message, *_):
            self.found.append(f"{filename}: {message}")

    reporter = Collect()
    for path in CHECKED:
        if "__pycache__" in path.parts:
            continue
        api.check(path.read_text(), str(path.relative_to(REPO)), reporter)
    assert reporter.found == []
