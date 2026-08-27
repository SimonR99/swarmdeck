"""What the generated ARGoS experiment must say, and why.

None of this needs ARGoS. What it checks is the seam that ARGoS cannot check
for us: that the XML handed to the simulator carries the same chassis and
sensor geometry that Nav2's costmaps, SLAM's static transforms and the
adapter's `hello` are configured from. Every mismatch here is silent at
runtime. A lidar mounted 0.25 m below where SLAM believes it is tilts and
offsets every scan; a track gauge that disagrees with the entity plugin scales
every commanded turn rate; a collision mesh transformed differently from the
visual prop makes robots hit a building that is not where it is drawn.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from xml.etree import ElementTree

import pytest

SCENARIO = Path(__file__).resolve().parents[1] / "scenario"
sys.path.insert(0, str(SCENARIO))

yaml = pytest.importorskip("yaml")

import make_argos_session as mas  # noqa: E402
import spawn_fleet  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / "configs" / "4robot.yaml"


@pytest.fixture(scope="module")
def tree():
    return ElementTree.fromstring(mas.generate_argos_xml(CONFIG))


@pytest.fixture(scope="module")
def cfg():
    return yaml.safe_load(CONFIG.read_text())


def controllers(tree):
    return {c.get("id"): c for c in tree.find("controllers")}


# ------------------------------------------------------------------ geometry


def test_every_sensor_mount_comes_from_the_shared_robot_profiles(tree, cfg):
    """RobotSpec is the fleet's single source of truth for mount geometry.

    It is read here, by session.launch.py for SLAM's static transforms, and by
    adapter_sim for what it reports to the backend. A second table would drift
    without anything failing.
    """
    fleet = cfg["fleet"]
    prefix = fleet.get("robot_prefix", "robot_")
    count = int(fleet["robot_count"])
    types = spawn_fleet.robot_types(fleet, count, prefix)
    blocks = controllers(tree)

    for i, platform in enumerate(types):
        spec = spawn_fleet.robot_spec(platform)
        block = blocks[f"{prefix}{i}_ctrl"]
        lidar = block.find("./sensors/photorealistic_lidar")
        camera = block.find("./sensors/photorealistic_camera")

        # ARGoS mounts on the origin anchor, which sits ON THE FLOOR;
        # RobotSpec's offsets are relative to base_link, which floats
        # base_height above it. The sum is the whole conversion, and getting it
        # wrong puts every scan a chassis height away from where SLAM expects.
        lx, _ly, lz = (float(v) for v in lidar.get("position").split(","))
        assert lx == pytest.approx(spec.lidar_x, abs=1e-4)
        assert lz == pytest.approx(spec.base_height + spec.lidar_z, abs=1e-4)

        cx, _cy, cz = (float(v) for v in camera.get("position").split(","))
        assert cx == pytest.approx(spec.camera_x, abs=1e-4)
        assert cz == pytest.approx(spec.base_height + spec.camera_z, abs=1e-4)


def test_the_track_gauge_matches_the_entity_plugin(tree, cfg):
    """The controller turns (v, omega) into wheel speeds with this number and
    the Jolt model turns them back with its own TRACK_GAUGE constant. If the
    two disagree every commanded turn rate is scaled wrong, and nothing says
    so: the robot simply under- or over-steers for the whole run."""
    fleet = cfg["fleet"]
    prefix = fleet.get("robot_prefix", "robot_")
    types = spawn_fleet.robot_types(fleet, int(fleet["robot_count"]), prefix)
    blocks = controllers(tree)
    for i, platform in enumerate(types):
        params = blocks[f"{prefix}{i}_ctrl"].find("params")
        assert float(params.get("track_gauge")) == pytest.approx(
            spawn_fleet.robot_spec(platform).track_gauge, abs=1e-4)


def test_the_lidar_matches_the_configured_profile(tree, cfg):
    spec = spawn_fleet.lidar_spec(cfg["fleet"])
    lidar = list(controllers(tree).values())[0].find(
        "./sensors/photorealistic_lidar")
    assert int(lidar.get("rings")) == spec.rings
    # ARGoS takes an azimuth STEP in degrees where LidarSpec counts samples per
    # revolution. 1800 samples is 0.2 deg, which is what a real unit does.
    assert float(lidar.get("horizontal_resolution")) == pytest.approx(
        360.0 / spec.h_samples, abs=1e-4)
    assert float(lidar.get("max_range")) == pytest.approx(spec.range_max)
    lo, hi = (float(v) for v in lidar.get("vertical_fov").split(","))
    assert hi == pytest.approx(math.degrees(spec.vfov), abs=1e-3)
    assert lo == pytest.approx(-hi, abs=1e-3)


def test_each_robot_is_the_entity_its_profile_names(tree, cfg):
    fleet = cfg["fleet"]
    prefix = fleet.get("robot_prefix", "robot_")
    types = spawn_fleet.robot_types(fleet, int(fleet["robot_count"]), prefix)
    arena = tree.find("arena")
    for i, platform in enumerate(types):
        found = arena.findall(f"./{platform}[@id='{prefix}{i}']")
        assert found, f"{prefix}{i} should be a <{platform}> entity"


def test_robots_spawn_on_the_floor_at_their_configured_poses(tree, cfg):
    """The origin anchor is on the floor, so z is ~0 plus settling clearance.

    An earlier version added base_height here as well, which floats every robot
    a chassis height in the air and drops it on the first physics step.
    """
    starts = cfg["map"]["start_poses"]
    arena = tree.find("arena")
    for rid, pose in starts.items():
        body = next(b for e in arena for b in e.findall("body")
                    if e.get("id") == rid)
        x, y, z = (float(v) for v in body.get("position").split(","))
        assert x == pytest.approx(pose["x"], abs=1e-3)
        assert y == pytest.approx(pose["y"], abs=1e-3)
        assert 0.0 <= z <= 0.05
        yaw, _p, _r = (float(v) for v in body.get("orientation").split(","))
        assert yaw == pytest.approx(math.degrees(pose["yaw"]), abs=1e-2)


# --------------------------------------------------------------------- world


def test_the_collision_mesh_and_the_visual_prop_agree(tree):
    """Physics and rendering must place the building identically.

    They are the same file loaded twice, by two subsystems that share nothing.
    When they disagree the robots collide with a building that is not where it
    is drawn, the lidar reports free space through a wall, and nothing raises.
    """
    mesh = tree.find("./arena/mesh[@id='world_mesh']")
    prop = tree.find("./media/photorealism/scenery/prop")
    assert mesh.get("file") == prop.get("model")
    for attr in ("position", "orientation", "scale"):
        assert mesh.get(attr) == prop.get(attr), attr


def test_every_detection_target_is_both_collidable_and_visible(tree):
    """A prop with no mesh is driven through; a mesh with no prop is invisible
    to the cameras and to the photorealistic lidar, which raytrace the render
    scene rather than the collision geometry."""
    meshes = {m.get("file"): m for m in tree.findall("./arena/mesh")
              if m.get("id") != "world_mesh"}
    props = {p.get("model"): p for p in
             tree.findall("./media/photorealism/scenery/prop")}
    world = tree.find("./arena/mesh[@id='world_mesh']").get("file")
    assert meshes, "no detection targets were placed"
    assert set(meshes) == set(props) - {world}
    for model, mesh in meshes.items():
        for attr in ("position", "orientation", "scale"):
            assert mesh.get(attr) == props[model].get(attr), (model, attr)


def test_target_classes_come_from_the_detector_catalog():
    """The classes placed in the world and the classes the detector is prompted
    with are one list. A target nothing is looking for is scenery."""
    from adapters.perception.catalog import CLASS_NAMES  # noqa: E402

    import make_argos_world as maw  # noqa: E402
    assert tuple(maw.PROPS) == CLASS_NAMES
    for name in maw.PROPS:
        assert (REPO / "argos" / "assets" / "props" / f"{name}.glb").exists()


# ---------------------------------------------------------------- estimator


def test_the_odometry_is_the_estimator_and_not_ground_truth(tree):
    """The whole point of the ARGoS backend. `positioning` stays declared, for
    the bridge's `/ns/ground_truth` topic, but nothing navigates on it."""
    for block in controllers(tree).values():
        odometry = block.find("./sensors/odometry")
        assert odometry.get("implementation") == "external"
        assert odometry.get("medium") == "uf"


def test_the_estimator_reads_only_the_channels_it_fuses(tree):
    """An unread VLP-16 revolution is ~630 KB per robot per 100 ms, and Fast
    DDS drops the channels that matter to make room for it."""
    estimator = tree.find("./media/external_estimator")
    assert set(estimator.get("channels").split(",")) == {"imu", "lidar", "wheels"}


def test_every_robot_is_registered_with_the_estimator(tree, cfg):
    """Ultra-Fusion keys its estimates by the ids in this attribute. A robot
    missing from it never gets a pose, its odometry stays invalid, and it
    stands still for the whole run."""
    fleet = cfg["fleet"]
    prefix = fleet.get("robot_prefix", "robot_")
    expected = [f"{prefix}{i}" for i in range(int(fleet["robot_count"]))]
    estimator = tree.find("./media/external_estimator")
    assert estimator.get("robots").split(",") == expected
    assert tree.find("loop_functions").get("robots").split(",") == expected


def test_the_estimator_is_not_handed_the_answer(tree):
    """alignment="ground_truth" would put every robot in a shared frame, which
    is precisely what swarmdeck-slam exists to recover."""
    assert tree.find("./media/external_estimator").get("alignment") == "none"


def test_the_development_config_keeps_one_of_every_platform():
    """Dropping a robot for speed must not drop a platform.

    The three differ in footprint and in mapping-lidar height, so the merged
    map is built from robots seeing the building from different heights. Simply
    running 4robot.yaml with three robots would drop robot_3, the Spot, and
    leave two Bunkers and a Scout Mini.
    """
    dev = REPO / "configs" / "3robot.yaml"
    cfg = yaml.safe_load(dev.read_text())
    fleet = cfg["fleet"]
    types = spawn_fleet.robot_types(
        fleet, int(fleet["robot_count"]), fleet.get("robot_prefix", "robot_")
    )
    assert sorted(types) == ["bunker", "scout_mini", "spot"]

    arena = ElementTree.fromstring(mas.generate_argos_xml(dev)).find("arena")
    for platform in types:
        assert arena.findall(f"./{platform}"), platform


def test_the_development_config_starts_every_robot_it_declares():
    """A start pose missing from the config puts that robot at a fallback
    position nothing else in the stack knows about."""
    cfg = yaml.safe_load((REPO / "configs" / "3robot.yaml").read_text())
    fleet = cfg["fleet"]
    prefix = fleet.get("robot_prefix", "robot_")
    expected = {f"{prefix}{i}" for i in range(int(fleet["robot_count"]))}
    assert set(cfg["map"]["start_poses"]) == expected


def test_drift_odometry_drops_the_estimator_entirely(tree):
    """`--odometry drift` must not leave a dangling medium="uf" reference: the
    sensor would fail to resolve its medium and ARGoS would refuse to start."""
    diag = ElementTree.fromstring(
        mas.generate_argos_xml(CONFIG, estimator=False)
    )
    assert diag.find("./media/external_estimator") is None
    for block in controllers(diag).values():
        odometry = block.find("./sensors/odometry")
        assert odometry.get("implementation") == "drift"
        assert odometry.get("medium") is None


def test_diagnostics_mode_drops_the_estimator_entirely(tree):
    """`--no-estimator` is for frame capture and CI, and must not leave a
    dangling `medium="uf"` reference behind."""
    xml = mas.generate_argos_xml(CONFIG, estimator=False)
    diag = ElementTree.fromstring(xml)
    assert diag.find("./media/external_estimator") is None
    for block in controllers(diag).values():
        assert block.find("./sensors/odometry").get("implementation") == "drift"


# ------------------------------------------------------------------- refusals


def test_a_planar_lidar_is_refused_rather_than_silently_useless(tmp_path):
    """Ultra-Fusion's lidar-inertial modes register on vertical structure.

    Given a single ring it never converges, every robot's odometry stays
    invalid, and the fleet stands still. That presents as a bridge fault, so
    the generator refuses and names the fix instead.
    """
    cfg = yaml.safe_load(CONFIG.read_text())
    cfg["fleet"]["lidar"] = {"profile": "generic_2d"}
    path = tmp_path / "planar.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="vlp16|generic_32"):
        mas.generate_argos_xml(path)


# --------------------------------------------------------------------- rates


def test_the_sensor_rates_divide_the_tick_rate(tree):
    """100 ticks/s is what Ultra-Fusion's noise densities were derived for; the
    sensors divide it back down to their own real rates, so the tick rate does
    not multiply the rendering cost."""
    assert int(tree.find("./framework/experiment").get("ticks_per_second")) == 100
    block = list(controllers(tree).values())[0]
    lidar = block.find("./sensors/photorealistic_lidar")
    camera = block.find("./sensors/photorealistic_camera")
    assert 100 / int(lidar.get("framerate_divider")) == pytest.approx(mas.LIDAR_HZ)
    assert 100 / int(camera.get("framerate_divider")) == pytest.approx(mas.CAMERA_HZ)
    assert 100 / int(tree.find("loop_functions").get("exchange_period")) == (
        pytest.approx(mas.EXCHANGE_HZ))


def test_the_same_config_renders_the_same_bytes():
    """The world is regenerated on every launch; a run that cannot be repeated
    is not a baseline (NFR-5)."""
    assert mas.generate_argos_xml(CONFIG) == mas.generate_argos_xml(CONFIG)


# -------------------------------------------------------------------- bistro


BISTRO_CONFIG = REPO / "configs" / "4robot_bistro.yaml"


@pytest.fixture(scope="module")
def bistro_tree():
    return ElementTree.fromstring(mas.generate_argos_xml(BISTRO_CONFIG))


@pytest.fixture(scope="module")
def bistro_cfg():
    return yaml.safe_load(BISTRO_CONFIG.read_text())


def test_bistro_scenario_generates_valid_xml(bistro_tree):
    """The bistro scenario config emits a well-formed ARGoS experiment."""
    assert bistro_tree.tag == "argos-configuration"
    assert bistro_tree.find("arena") is not None
    assert bistro_tree.find("physics_engines/jolt") is not None
    assert bistro_tree.find("media/photorealism") is not None


def test_bistro_arena_and_collision_mesh(bistro_tree):
    """Bistro uses the large 200x210 arena and loads the glTF mesh into Jolt physics."""
    arena = bistro_tree.find("arena")
    assert arena.get("size") == "200,210,70"
    assert arena.get("center") == "24,-4,25"

    mesh = arena.find("./mesh[@id='world_mesh']")
    assert mesh is not None, "Bistro must declare world_mesh for Jolt physics"
    assert "bistro_exterior.glb" in mesh.get("file")
    assert mesh.get("position") == "0,0,-0.3"
    assert mesh.get("orientation") == "0,0,90"


def test_bistro_scenery_and_lighting(bistro_tree):
    """Scenery prop matches the physics mesh at z=-0.3 with night lighting."""
    pr = bistro_tree.find("media/photorealism")
    assert pr.get("draw_floor") == "false"

    scenery = pr.findall("scenery/prop")
    world_prop = next(p for p in scenery if "bistro_exterior.glb" in p.get("model", ""))
    assert world_prop.get("position") == "0,0,-0.3"
    assert world_prop.get("orientation") == "0,0,90"

    # Physics mesh and visual prop agree bitwise
    arena = bistro_tree.find("arena")
    mesh = arena.find("./mesh[@id='world_mesh']")
    assert mesh.get("file") == world_prop.get("model")
    assert mesh.get("position") == world_prop.get("position")
    assert mesh.get("orientation") == world_prop.get("orientation")

    lights = pr.findall("lights/point")
    assert len(lights) == 28, "Bistro has 28 street lamps"

    exposure = pr.find("exposure")
    assert float(exposure.get("aperture")) == pytest.approx(2.0)
    assert float(exposure.get("shutter_speed")) == pytest.approx(0.02)
    assert float(exposure.get("sensitivity")) == pytest.approx(400.0)


def test_bistro_robot_spawn_poses(bistro_tree, bistro_cfg):
    """Robots spawn at their designated positions along the 141 m Bistro street tour."""
    starts = bistro_cfg["map"]["start_poses"]
    arena = bistro_tree.find("arena")
    for rid, pose in starts.items():
        body = next(b for e in arena for b in e.findall("body") if e.get("id") == rid)
        x, y, z = (float(v) for v in body.get("position").split(","))
        assert x == pytest.approx(pose["x"], abs=1e-3)
        assert y == pytest.approx(pose["y"], abs=1e-3)
        assert 0.0 <= z <= 0.05
        yaw, _p, _r = (float(v) for v in body.get("orientation").split(","))
        assert yaw == pytest.approx(math.degrees(pose["yaw"]), abs=1e-2)


def test_bistro_3robot_dev_config():
    """3-robot bistro config starts one of each platform."""
    dev = REPO / "configs" / "3robot_bistro.yaml"
    cfg = yaml.safe_load(dev.read_text())
    fleet = cfg["fleet"]
    types = spawn_fleet.robot_types(
        fleet, int(fleet["robot_count"]), fleet.get("robot_prefix", "robot_")
    )
    assert sorted(types) == ["bunker", "scout_mini", "spot"]
    tree = ElementTree.fromstring(mas.generate_argos_xml(dev))
    arena = tree.find("arena")
    for platform in types:
        assert arena.findall(f"./{platform}"), platform

