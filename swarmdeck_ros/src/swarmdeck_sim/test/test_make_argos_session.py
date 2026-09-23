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
            spawn_fleet.robot_spec(platform).track_gauge, abs=1e-4
        )


def test_the_lidar_matches_the_configured_profile(tree, cfg):
    spec = spawn_fleet.lidar_spec(cfg["fleet"])
    lidar = list(controllers(tree).values())[0].find("./sensors/photorealistic_lidar")
    assert int(lidar.get("rings")) == spec.rings
    # ARGoS takes an azimuth STEP in degrees where LidarSpec counts samples per
    # revolution. 1800 samples is 0.2 deg, which is what a real unit does.
    assert float(lidar.get("horizontal_resolution")) == pytest.approx(
        360.0 / spec.h_samples, abs=1e-4
    )
    assert float(lidar.get("max_range")) == pytest.approx(spec.range_max)
    lo, hi = (float(v) for v in lidar.get("vertical_fov").split(","))
    assert hi == pytest.approx(math.degrees(spec.vfov), abs=1e-3)
    assert lo == pytest.approx(-hi, abs=1e-3)


@pytest.mark.parametrize("profile", ["vlp16", "os1_32"])
def test_lidar_extreme_rays_fit_rendered_face_corners(tmp_path, cfg, profile):
    config = yaml.safe_load(yaml.safe_dump(cfg))
    config["fleet"]["lidar"] = {"profile": profile}
    path = tmp_path / "lidar.yaml"
    path.write_text(yaml.safe_dump(config))
    root = ElementTree.fromstring(mas.generate_argos_xml(path))
    lidar = root.find("./controllers/*/sensors/photorealistic_lidar")
    width, height = map(int, lidar.get("face_resolution", "512,192").split(","))
    half_span = math.pi / int(lidar.get("faces", "4"))
    elevation = max(abs(float(v)) for v in lidar.get("vertical_fov").split(","))
    # At a face boundary, the forward projection is smaller than at its
    # centre. A centre-only FOV check still clips the upper/lower corner rays.
    ray_up_over_forward = math.tan(math.radians(elevation)) / math.cos(half_span)
    face_up_over_forward = math.tan(half_span) * height / width
    assert ray_up_over_forward <= face_up_over_forward


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
        body = next(b for e in arena for b in e.findall("body") if e.get("id") == rid)
        x, y, z = (float(v) for v in body.get("position").split(","))
        assert x == pytest.approx(pose["x"], abs=1e-3)
        assert y == pytest.approx(pose["y"], abs=1e-3)
        assert z == pytest.approx(pose.get("z", 0.02))
        yaw, _p, _r = (float(v) for v in body.get("orientation").split(","))
        assert yaw == pytest.approx(math.degrees(pose["yaw"]), abs=1e-2)


# --------------------------------------------------------------------- world


def test_the_collision_mesh_and_the_visual_prop_agree(tree):
    """Physics and rendering must place the building identically.

    Two subsystems that share nothing load the building separately. When they
    disagree the robots collide with a building that is not where it is drawn,
    the lidar reports free space through a wall, and nothing raises.

    The transform therefore has to match exactly. The FILE deliberately does
    not: see the companion test below.
    """
    mesh = tree.find("./arena/mesh[@id='world_mesh']")
    prop = tree.find("./media/photorealism/scenery/prop")
    for attr in ("position", "orientation", "scale"):
        assert mesh.get(attr) == prop.get(attr), attr


def test_physics_collides_with_the_floorless_copy_of_the_world(tree):
    """The Jolt mesh must be the collision variant, not the drawn one.

    `<physics_engines>` provides the ground as a `<floor height="0">` plane.
    `indoor.gltf` also carries a floor slab whose top face is at z=0, for the
    renderer, which needs something to photograph. Cook that slab into the
    collision mesh as well and every robot rests on two coincident surfaces;
    the degenerate contacts cost 60-100% of the commanded turn rate, varying
    with position, while translation is almost unaffected. The symptom is a
    robot that drives but will not turn, and nothing anywhere reports it.

    So the two files must differ, and the collision one must be the floorless
    variant that `make_argos_world.collision_path` names.
    """
    mesh = tree.find("./arena/mesh[@id='world_mesh']")
    prop = tree.find("./media/photorealism/scenery/prop")

    assert (
        tree.find("./physics_engines/jolt/floor") is not None
    ), "the reasoning below assumes the engine supplies the ground plane"
    assert mesh.get("file") != prop.get("model")

    import make_argos_world as maw  # noqa: E402

    assert mesh.get("file") == str(maw.collision_path(Path(prop.get("model"))))


def test_only_large_detection_targets_are_collidable(tree):
    """A prop with no mesh is driven through; a mesh with no prop is invisible
    to the cameras and to the photorealistic lidar, which raytrace the render
    scene rather than the collision geometry."""
    meshes = {
        m.get("file"): m
        for m in tree.findall("./arena/mesh")
        if m.get("id") != "world_mesh"
    }
    props = {
        p.get("model"): p for p in tree.findall("./media/photorealism/scenery/prop")
    }
    world = tree.find("./arena/mesh[@id='world_mesh']").get("file")
    assert meshes, "no detection targets were placed"

    # The building is in both sets under DIFFERENT names: the prop draws
    # indoor.gltf, physics collides with indoor_collision.gltf. Drop it from
    # the prop side by the same mapping the generator used, so this stays a
    # statement about detection targets and nothing else.
    import make_argos_world as maw  # noqa: E402

    world_props = {m for m in props if str(maw.collision_path(Path(m))) == world}
    assert len(world_props) == 1, world_props
    small = {m for m in props if Path(m).stem in mas.NONBLOCKING_TARGET_CLASSES}
    assert small
    assert set(meshes) == set(props) - world_props - small
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
    diag = ElementTree.fromstring(mas.generate_argos_xml(CONFIG, estimator=False))
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


def test_heterogeneous_odometry_per_robot(tmp_path):
    """A fleet with mixed odometry must configure each controller independently
    and register only Fast-LIVO2 robots on the external estimator medium."""
    cfg = yaml.safe_load(CONFIG.read_text())
    cfg["fleet"]["odometry"] = "fast_livo2"
    cfg["fleet"]["odometry_types"] = {
        "robot_1": "drift",
        "robot_3": "drift",
    }
    path = tmp_path / "mixed_odom.yaml"
    path.write_text(yaml.safe_dump(cfg))

    xml = mas.generate_argos_xml(path)
    tree = ElementTree.fromstring(xml)

    ctrls = controllers(tree)
    # robot_0 and robot_2 have Fast-LIVO2 (external, uf)
    assert (
        ctrls["robot_0_ctrl"].find("./sensors/odometry").get("implementation")
        == "external"
    )
    assert ctrls["robot_0_ctrl"].find("./sensors/odometry").get("medium") == "uf"
    assert (
        ctrls["robot_2_ctrl"].find("./sensors/odometry").get("implementation")
        == "external"
    )
    assert ctrls["robot_2_ctrl"].find("./sensors/odometry").get("medium") == "uf"

    # robot_1 and robot_3 have synthetic drift
    assert (
        ctrls["robot_1_ctrl"].find("./sensors/odometry").get("implementation")
        == "drift"
    )
    assert ctrls["robot_1_ctrl"].find("./sensors/odometry").get("medium") is None
    assert (
        ctrls["robot_3_ctrl"].find("./sensors/odometry").get("implementation")
        == "drift"
    )
    assert ctrls["robot_3_ctrl"].find("./sensors/odometry").get("medium") is None

    # external_estimator must list ONLY robot_0 and robot_2
    ext = tree.find("./media/external_estimator")
    assert ext is not None
    assert set(ext.get("robots").split(",")) == {"robot_0", "robot_2"}


def test_custom_prefix_is_registered_with_external_estimator(tmp_path):
    """Fast-LIVO2 discovers IDs from this generated experiment at cold start."""
    cfg = yaml.safe_load(CONFIG.read_text())
    fleet = cfg["fleet"]
    fleet["robot_count"] = 2
    fleet["robot_prefix"] = "rover_"
    fleet["robot_types"] = {
        key.replace("robot_", "rover_"): value
        for key, value in (fleet.get("robot_types") or {}).items()
        if key in {"robot_0", "robot_1"}
    }
    cfg["map"]["start_poses"] = {
        key.replace("robot_", "rover_"): value
        for key, value in cfg["map"]["start_poses"].items()
        if key in {"robot_0", "robot_1"}
    }
    path = tmp_path / "custom_prefix_fast_livo.yaml"
    path.write_text(yaml.safe_dump(cfg))

    tree = ElementTree.fromstring(mas.generate_argos_xml(path))

    assert set(controllers(tree)) == {"rover_0_ctrl", "rover_1_ctrl"}
    assert tree.find("./media/external_estimator").get("robots") == "rover_0,rover_1"


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
        pytest.approx(mas.EXCHANGE_HZ)
    )


def test_the_same_config_renders_the_same_bytes():
    """The world is regenerated on every launch; a run that cannot be repeated
    is not a baseline (NFR-5)."""
    assert mas.generate_argos_xml(CONFIG) == mas.generate_argos_xml(CONFIG)


# -------------------------------------------------------------------- bistro


BISTRO_CONFIG = REPO / "configs" / "4robot_bistro.yaml"


def test_sim_progress_checker_has_a_bounded_translation_watchdog():
    nav_params = yaml.safe_load(
        (REPO / "swarmdeck_ros/src/swarmdeck_nav/config/nav2_params.yaml").read_text()
    )
    controller = nav_params["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    goal = controller["goal_checker"]

    assert progress["plugin"] == "nav2_controller::SimpleProgressChecker"
    assert 0.0 < progress["required_movement_radius"] < goal["xy_goal_tolerance"]
    assert "required_movement_angle" not in progress
    assert progress["movement_time_allowance"] == 20.0


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
    """Robots deploy together with clearance above the uneven Bistro street."""
    starts = bistro_cfg["map"]["start_poses"]
    arena = bistro_tree.find("arena")
    for rid, pose in starts.items():
        body = next(b for e in arena for b in e.findall("body") if e.get("id") == rid)
        x, y, z = (float(v) for v in body.get("position").split(","))
        assert x == pytest.approx(pose["x"], abs=1e-3)
        assert y == pytest.approx(pose["y"], abs=1e-3)
        assert z == pytest.approx(pose.get("z", 0.02))
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


def test_system_threads_matches_default_or_override():
    # default is 0 for Vulkan thread safety
    xml = mas.generate_argos_xml(CONFIG)
    tree = ElementTree.fromstring(xml)
    system = tree.find("./framework/system")
    assert system is not None
    assert int(system.get("threads")) == 0

    # explicit override
    xml_override = mas.generate_argos_xml(CONFIG, threads=2)
    tree_override = ElementTree.fromstring(xml_override)
    system_override = tree_override.find("./framework/system")
    assert int(system_override.get("threads")) == 2


def test_collision_applies_axis_conversion_exactly_once(tree, bistro_tree):
    """Jolt defaults y_up=true; copying the prop roll then rotates twice."""
    import numpy as np

    for scene in (tree, bistro_tree):
        props = scene.findall("./media/photorealism/scenery/prop")
        for mesh in scene.findall("./arena/mesh"):
            assert mesh.get("y_up") == "false"
            candidates = [
                p
                for p in props
                if p.get("position") == mesh.get("position")
                and p.get("orientation") == mesh.get("orientation")
            ]
            assert candidates
            # An asymmetric Y-up point must land in the same Z-up location.
            point = np.array([2.0, 3.0, 5.0])
            roll = math.radians(float(mesh.get("orientation").split(",")[2]))
            rotation = np.array(
                [
                    [1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)],
                ]
            )
            np.testing.assert_allclose(rotation @ point, [2.0, -5.0, 3.0], atol=1e-6)
    assert bistro_tree.find("./physics_engines/jolt/floor") is None
    assert tree.find("./physics_engines/jolt/floor") is not None


def test_bistro_deployment_is_compact_and_separated(bistro_cfg):
    from itertools import combinations

    poses = list(bistro_cfg["map"]["start_poses"].values())
    for a, b in combinations(poses, 2):
        distance = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
        assert 2 <= distance < 3
        assert a["yaw"] == b["yaw"]


def test_robot_visuals_are_packaged_and_selected(tree, bistro_tree, tmp_path):
    import json
    import struct
    import make_robot_visuals as visuals

    for scene in (tree, bistro_tree):
        assets = Path(scene.find("./media/photorealism").get("asset_path"))
        for name, build in [
            ("bunker", visuals.bunker),
            ("scout_mini", visuals.scout_mini),
            ("spot", visuals.spot),
        ]:
            blob = (assets / (name + ".glb")).read_bytes()
            magic, version, size, json_length = struct.unpack_from("<4I", blob)
            assert magic == 0x46546C67 and version == 2 and size == len(blob)
            doc = json.loads(blob[20 : 20 + json_length])
            assert len(doc["materials"]) <= 6
            assert (
                sum(
                    doc["accessors"][p["indices"]]["count"] // 3
                    for p in doc["meshes"][0]["primitives"]
                )
                < 3000
            )
            model = build()
            regenerated = tmp_path / (name + ".glb")
            model.export_glb(regenerated)
            assert regenerated.read_bytes() == blob
            # Wound surface normals must agree with triangle orientation.
            import numpy as np

            for positions, normals, indices in model._groups.values():
                points = np.array(positions).reshape(-1, 3)
                triangles = points[np.array(indices).reshape(-1, 3)]
                cross = np.cross(
                    triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
                )
                assert np.all(
                    np.sum(cross * np.array(normals).reshape(-1, 3)[::3], axis=1) > 0
                )


def test_targets_stay_upright_when_rotated(tree, bistro_tree):
    import numpy as np

    for scene in (tree, bistro_tree):
        for mesh in scene.findall("./arena/mesh"):
            if not mesh.get("id").startswith("target_"):
                continue
            z, y, x = map(math.radians, map(float, mesh.get("orientation").split(",")))
            rx = np.array(
                [
                    [1, 0, 0],
                    [0, math.cos(x), -math.sin(x)],
                    [0, math.sin(x), math.cos(x)],
                ]
            )
            ry = np.array(
                [
                    [math.cos(y), 0, math.sin(y)],
                    [0, 1, 0],
                    [-math.sin(y), 0, math.cos(y)],
                ]
            )
            rz = np.array(
                [
                    [math.cos(z), -math.sin(z), 0],
                    [math.sin(z), math.cos(z), 0],
                    [0, 0, 1],
                ]
            )
            # glTF up must stay world up under ARGoS's actual Rx*Ry*Rz order.
            np.testing.assert_allclose(rx @ ry @ rz @ [0, 1, 0], [0, 0, 1], atol=1e-6)


def test_bistro_duck_is_raised_above_brick_road(bistro_tree):
    mesh = bistro_tree.find('./arena/mesh[@id="target_0_rubber_duck"]')
    z = float(mesh.get("position").split(",")[2])
    assert 0.075 < z < 0.09
    prop = next(
        p
        for p in bistro_tree.findall("./media/photorealism/scenery/prop")
        if p.get("position") == mesh.get("position")
    )
    assert prop.get("orientation") == mesh.get("orientation")


def test_bistro_small_props_remain_visible_without_static_colliders(bistro_tree):
    meshes = bistro_tree.findall("./arena/mesh")
    props = bistro_tree.findall("./media/photorealism/scenery/prop")
    for name in mas.NONBLOCKING_TARGET_CLASSES:
        assert any(Path(p.get("model")).stem == name for p in props)
        assert not any(Path(m.get("file")).stem == name for m in meshes)
    assert sum("rubber_duck" in m.get("file") for m in meshes) == 2


# --------------------------------------------------------------- subt finals


SUBT_CONFIG = REPO / "configs" / "4robot_subt_finals.yaml"


@pytest.fixture(scope="module")
def subt_tree():
    return ElementTree.fromstring(mas.generate_argos_xml(SUBT_CONFIG))


def test_subt_world_selected_by_config_or_by_name(subt_tree):
    """`world: subt_finals` in the config, or --world subt_finals from
    session.launch.py, both load the Finals mesh; neither adds the flat
    floor plane, whose z=0 would cut through the hangar floor at -0.01."""
    by_name = ElementTree.fromstring(
        mas.generate_argos_xml(CONFIG, world_gltf="subt_finals")
    )
    for tree in (subt_tree, by_name):
        mesh = tree.find("./arena/mesh[@id='world_mesh']")
        assert "finals_prize_round_world_01.collision.glb" in mesh.get("file")
        assert tree.find("./physics_engines/jolt/floor") is None


def test_subt_collision_and_visual_share_one_transform(subt_tree):
    """The collision file is not the visual file (the tiles ship lower-poly
    colliders), so agreement has to be on the transform alone."""
    mesh = subt_tree.find("./arena/mesh[@id='world_mesh']")
    prop = next(
        p
        for p in subt_tree.findall("./media/photorealism/scenery/prop")
        if "finals_prize_round_world_01.glb" in p.get("model", "")
    )
    assert Path(mesh.get("file")).parent == Path(prop.get("model")).parent
    assert mesh.get("position") == prop.get("position") == "0,0,0"
    assert mesh.get("orientation") == prop.get("orientation") == "0,0,90"


def test_subt_fleet_deploys_in_the_hangar_facing_the_tunnel(subt_tree):
    """The staging hangar spans x in [-20, -11] and y within +-5.5 (measured
    through the collision mesh); the tunnel leaves it through a 3.3 m gate at
    (-10.5, 0) along +x. Every robot must stand inside, point at the gate,
    and clear the floor at z = -0.01."""
    arena = subt_tree.find("arena")
    bodies = {
        e.get("id"): e.find("body")
        for e in arena
        if e.tag in ("bunker", "scout_mini", "spot")
    }
    assert len(bodies) == 4
    for body in bodies.values():
        x, y, z = (float(v) for v in body.get("position").split(","))
        assert -19.5 < x < -11.5 and abs(y) < 4.5
        assert 0.1 <= z <= 0.3
        yaw, _p, _r = (float(v) for v in body.get("orientation").split(","))
        assert yaw == pytest.approx(0.0, abs=1e-6)
    # The head of the group, released first by fleet-wide Explore, is the
    # robot nearest the gate.
    front = max(
        bodies, key=lambda rid: float(bodies[rid].get("position").split(",")[0])
    )
    assert front == "robot_0"


def test_subt_is_lit_by_its_own_lamps_only(subt_tree):
    pr = subt_tree.find("media/photorealism")
    assert pr.get("draw_floor") == "false"
    assert float(pr.find("sun").get("intensity")) == 0.0
    assert pr.find("environment") is None
    lamps = pr.findall("lights/spot") + pr.findall("lights/point")
    world_lamps = [l for l in lamps if l.get("entity") is None]
    assert len(world_lamps) == 88, "the importer found 88 SDF lights in this world"
    assert all(float(l.get("intensity")) > 0 for l in lamps)


def test_subt_robots_carry_forward_floodlights(subt_tree):
    """Each robot in the dark SubT tunnels carries a forward-facing floodlight."""
    pr = subt_tree.find("media/photorealism")
    robot_lamps = [l for l in pr.findall("lights/spot") if l.get("entity") is not None]
    assert len(robot_lamps) == 4
    assert {l.get("entity") for l in robot_lamps} == {
        "robot_0",
        "robot_1",
        "robot_2",
        "robot_3",
    }
    for lamp in robot_lamps:
        assert lamp.get("direction") == "1,0,-0.05"
        assert float(lamp.get("intensity")) >= 3000.0
        assert float(lamp.get("falloff")) >= 20.0
        assert float(lamp.get("inner_angle")) >= 30.0
        assert float(lamp.get("outer_angle")) >= 45.0
        # Origin anchor: position x is positive (at front of chassis) and z is positive (off the floor)
        x, y, z = (float(v) for v in lamp.get("position").split(","))
        assert x > 0.3
        assert y == 0.0
        assert z > 0.2


def test_subt_targets_are_scattered_through_the_tunnels(subt_tree):
    """Targets spread over the whole reachable network, seeded by the
    scenario: the first (a duck) at the far end, none by the entrance, none
    near another, each on its own level's floor rather than a ceiling."""
    props = subt_tree.findall("./media/photorealism/scenery/prop")
    targets = [p for p in props if "finals_prize_round_world_01" not in p.get("model")]
    assert len(targets) == 10
    positions = [tuple(map(float, p.get("position").split(","))) for p in targets]
    start = (-14.5, 1.0)
    assert Path(targets[0].get("model")).stem == "rubber_duck"
    # The network reaches x = 411 m; its far end is hundreds of metres away.
    assert math.dist(positions[0][:2], start) > 300.0
    for i, (x, y, z) in enumerate(positions):
        assert math.dist((x, y), start) > mas.SCATTER_MIN_DISTANCE_M / 2
        for other in positions[i + 1 :]:
            assert math.dist((x, y, z), other) >= mas.SCATTER_SPACING_M
    # Several levels: the tunnels descend from the hangar floor at z = 0.
    assert len({round(z) for _, _, z in positions}) > 1
    assert all(-16.0 < z < 1.5 for _, _, z in positions)
