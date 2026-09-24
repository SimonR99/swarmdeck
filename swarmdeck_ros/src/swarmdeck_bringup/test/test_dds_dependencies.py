"""Direct runtime dependencies of composed Nav2 and the static mount writer."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

SRC = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "package,required",
    [
        ("swarmdeck_bringup", {"rclpy", "tf2_msgs", "geometry_msgs"}),
        ("swarmdeck_nav", {"rclcpp_components"}),
    ],
)
def test_dds_runtime_dependencies_are_declared(package, required):
    manifest = ET.parse(SRC / package / "package.xml")
    assert required <= {element.text for element in manifest.findall("exec_depend")}
