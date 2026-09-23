"""Static contracts for the simulator's generated navigation includes."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_bringup/launch/session.launch.py"


def test_session_launches_one_static_mount_writer_for_the_fleet():
    source = LAUNCH.read_text()
    assert "static_transform_publisher" not in source
    assert source.count('with_name("static_mounts.py")') == 1
    assert "json.dumps(mounts)" in source
    assert 'f"{prefix}{i}": sensor_mount_transforms' in source


def test_session_passes_each_simulated_robot_geometry_to_nav2():
    source = LAUNCH.read_text()

    assert '"robot_base_frame": f"{ns}/base_link"' in source
    assert '"robot_radius": f"{robot.footprint_radius:.3f}"' in source
    assert '"footprint": robot.footprint' in source
    assert (
        '"inflation_radius": f"{robot.footprint_radius + INFLATION_MARGIN:.3f}"'
        in source
    )
