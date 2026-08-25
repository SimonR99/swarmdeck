"""Every costmap observation source must declare the heights it accepts.

This exists because of a failure that looked like nothing at all. nav2 defaults
a source's `max_obstacle_height` to 0.0, and `ObservationBuffer` keeps a
hitpoint only while `min < z < max`. Omit the value and the layer still reports
the source as enabled and marking, the topic still has a subscriber, the sensor
still publishes — and every return is discarded.

That is what stopped the simulated fleet avoiding rubber ducks and each other.
The bumper scan at 0.15 m is the only sensor that sees either (ducks top out at
0.33 m; the 2D mapping planes sit at 0.45 / 0.72 / 0.97 m, above almost every
chassis in the fleet), and its returns were being thrown away.
"""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
CONFIG_DIR = REPO / "swarmdeck_ros/src/swarmdeck_nav/config"


def _costmap_sources(path: Path):
    """(costmap, source, config) for every observation source in a params file."""
    raw = path.read_text().replace("<robot_namespace>", "ns")
    doc = yaml.safe_load(raw) or {}
    for costmap in ("local_costmap", "global_costmap"):
        node = (doc.get(costmap) or {}).get(costmap, {}).get("ros__parameters", {})
        layer = node.get("obstacle_layer") or {}
        for source in (layer.get("observation_sources") or "").split():
            yield costmap, source, (layer.get(source) or {})


def test_every_marking_source_declares_a_usable_height_window():
    files = sorted(CONFIG_DIR.glob("*nav2_params.yaml"))
    assert files, "no nav2 params files found"

    checked = 0
    for path in files:
        for costmap, source, cfg in _costmap_sources(path):
            if not cfg.get("marking", True):
                continue
            checked += 1
            where = f"{path.name}:{costmap}.{source}"
            top = cfg.get("max_obstacle_height")
            assert top is not None, (
                f"{where} marks obstacles but does not declare "
                "max_obstacle_height; nav2 defaults it to 0.0 and silently "
                "discards every return from this sensor"
            )
            bottom = cfg.get("min_obstacle_height", 0.0)
            assert top > bottom, f"{where} has an empty height window {bottom}..{top}"
    assert checked, "no marking sources were checked"


def test_the_bumper_accepts_returns_from_its_own_mounting_height():
    """A window that excludes the sensor's own plane accepts nothing.

    The proximity scan is horizontal at PROXIMITY_SCAN_HEIGHT (0.15 m), so
    every hitpoint it produces lands at roughly that height. The window has to
    contain it.
    """
    scan_height = 0.15
    for path in sorted(CONFIG_DIR.glob("*nav2_params.yaml")):
        for costmap, source, cfg in _costmap_sources(path):
            if source != "proximity_scan":
                continue
            low = cfg.get("min_obstacle_height", 0.0)
            high = cfg.get("max_obstacle_height", 0.0)
            assert low <= scan_height <= high, (
                f"{path.name}:{costmap}.proximity_scan accepts {low}..{high} m, "
                f"which excludes its own {scan_height} m scan plane"
            )


def test_global_static_layer_reads_the_collaborative_map():
    """Nav2's global planner must not subscribe to onboard SLAM's /map.

    That topic is still the per-robot local map the operator looks at. The
    merged product is published separately so the two cannot fight.
    """
    sim = yaml.safe_load(
        (CONFIG_DIR / "nav2_params.yaml").read_text().replace("<robot_namespace>", "ns")
    )
    static = (
        sim["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"]
    )
    assert static["map_topic"] == "/ns/global_map"
    sim_global = sim["global_costmap"]["global_costmap"]["ros__parameters"]
    assert sim_global["rolling_window"] is True
    assert sim_global["width"] >= 40
    assert sim_global["height"] >= 40
    assert "static_layer" not in sim["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]

    hardware = yaml.safe_load((CONFIG_DIR / "botman_nav2_params.yaml").read_text())
    hw_static = (
        hardware["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"]
    )
    assert hw_static["map_topic"] == "/global_map"
    local_plugins = hardware["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    assert "static_layer" not in local_plugins
