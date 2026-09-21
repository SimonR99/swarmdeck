"""Static contracts for the live, rolling local costmaps."""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
CONFIG_DIR = REPO / "swarmdeck_ros/src/swarmdeck_nav/config"

COSTMAPS = (
    pytest.param(
        "simulation",
        CONFIG_DIR / "nav2_params.yaml",
        "robot_0/odom",
        ("scan", "proximity_scan"),
        id="simulation",
    ),
    pytest.param(
        "botman",
        CONFIG_DIR / "botman_nav2_params.yaml",
        "map",
        ("scan",),
        id="botman",
    ),
    pytest.param(
        "aslan",
        CONFIG_DIR / "botman_nav2_params.yaml",
        "map",
        ("scan",),
        id="aslan",
    ),
    pytest.param(
        "asimov",
        CONFIG_DIR / "asimov_nav2_params.yaml",
        "world",
        ("scan",),
        id="asimov",
    ),
)


def _local_params(path: Path) -> dict:
    raw = path.read_text().replace("<robot_namespace>", "robot_0")
    document = yaml.safe_load(raw) or {}
    return document["local_costmap"]["local_costmap"]["ros__parameters"]


@pytest.mark.parametrize("robot, path, frame, sources", COSTMAPS)
def test_local_costmap_uses_continuous_frame_and_live_obstacles(
    robot, path, frame, sources
):
    params = _local_params(path)
    assert params["global_frame"] == frame, robot
    assert params["rolling_window"] is True
    assert tuple(params["obstacle_layer"]["observation_sources"].split()) == sources

    for source in sources:
        config = params["obstacle_layer"][source]
        assert config["marking"] is True
        assert config["clearing"] is True
        # Nav2's default is zero; keeping this explicit on hardware prevents a
        # Bunker return at the lidar origin from being treated as out of range.
        assert config.get("obstacle_min_range", 0.0) == 0.0
        assert config["obstacle_max_range"] > config.get("obstacle_min_range", 0.0)


def test_every_local_marking_source_has_a_nonempty_height_window():
    for path in sorted(CONFIG_DIR.glob("*nav2_params.yaml")):
        params = _local_params(path)
        layer = params["obstacle_layer"]
        for source in layer["observation_sources"].split():
            config = layer[source]
            if not config.get("marking", True):
                continue
            low = config.get("min_obstacle_height", 0.0)
            high = config.get("max_obstacle_height")
            assert high is not None, f"{path.name}:{source} has no height ceiling"
            assert high > low, f"{path.name}:{source} has an empty height window"
