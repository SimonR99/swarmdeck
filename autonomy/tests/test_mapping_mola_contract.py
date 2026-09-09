from pathlib import Path

PACKAGE = Path(__file__).parents[2] / "swarmdeck_ros/src/swarmdeck_mapping"


def test_native_bridge_uses_correction_capable_mola_map_api() -> None:
    source = (PACKAGE / "src/mola_submap_bridge.cpp").read_text()
    header = (PACKAGE / "include/swarmdeck_mapping/mola_submap_bridge.hpp").read_text()
    assert "public mola::MapSourceBase" in header
    assert "mola::KeyframePointCloudMap" in header
    assert "setKeyframePose" in source
    assert "insertObservation" in source
    assert "advertiseUpdatedMap" in source


def test_native_bridge_declares_real_mola_link_dependencies() -> None:
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    assert "find_package(mola_kernel REQUIRED)" in cmake
    assert "find_package(mola_metric_maps REQUIRED)" in cmake
    assert "mola::mola_metric_maps" in cmake
    assert "mrpt::maps" in cmake
    assert "swarmdeck-mola-import" in cmake
