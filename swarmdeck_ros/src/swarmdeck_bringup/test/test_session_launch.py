"""Static contracts for the simulator's generated navigation includes."""

import ast
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_bringup/launch/session.launch.py"


def test_session_launches_one_static_mount_writer_for_the_fleet():
    source = LAUNCH.read_text()
    assert "static_transform_publisher" not in source
    assert source.count('with_name("static_mounts.py")') == 1
    assert "json.dumps(mounts)" in source
    assert 'f"{prefix}{i}": sensor_mount_transforms' in source


def test_static_mount_helper_resolves_through_an_existing_install_symlink(tmp_path):
    installed_launch = (
        tmp_path / "install/share/swarmdeck_bringup/launch/session.launch.py"
    )
    installed_launch.parent.mkdir(parents=True)
    installed_launch.symlink_to(LAUNCH)
    # An image built before this helper was added has only the old launch link.
    assert not installed_launch.with_name("static_mounts.py").exists()
    helper_path = next(
        node
        for node in ast.walk(ast.parse(LAUNCH.read_text()))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "with_name"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "static_mounts.py"
    )
    resolved = eval(
        compile(ast.Expression(helper_path), str(LAUNCH), "eval"),
        {"Path": Path, "__file__": str(installed_launch)},
    )
    assert resolved == LAUNCH.with_name("static_mounts.py")
    assert resolved.is_file()


def test_session_passes_each_simulated_robot_geometry_to_nav2():
    source = LAUNCH.read_text()

    assert '"robot_base_frame": f"{ns}/base_link"' in source
    assert '"robot_radius": f"{robot.footprint_radius:.3f}"' in source
    assert '"footprint": robot.footprint' in source
    assert (
        '"inflation_radius": f"{robot.footprint_radius + INFLATION_MARGIN:.3f}"'
        in source
    )
