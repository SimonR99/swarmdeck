"""Fleet SDF rendering. Pure Python — no ROS, no Gazebo, so `make test` runs it."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))

from spawn_fleet import (  # noqa: E402
    CHASSIS_SECTIONS,
    IDENTITY_COLORS,
    LIDAR_PROFILES,
    PROXIMITY_SCAN_HEIGHT,
    ROBOT_PROFILES,
    LidarSpec,
    chassis_sections,
    color_for,
    lidar_spec,
    render,
    robot_spec,
    robot_types,
)


def test_single_ring_is_horizontal():
    fields = LidarSpec(rings=1).fields()
    assert fields["LIDAR_RINGS"] == "1"
    assert float(fields["LIDAR_VMIN"]) == 0.0
    assert float(fields["LIDAR_VMAX"]) == 0.0


def test_odd_ring_count_spans_the_vertical_fov():
    fields = LidarSpec(rings=9, vfov=0.26).fields()
    assert fields["LIDAR_RINGS"] == "9"
    assert float(fields["LIDAR_VMIN"]) == pytest.approx(-0.26)
    assert float(fields["LIDAR_VMAX"]) == pytest.approx(0.26)


@pytest.mark.parametrize("rings", [2, 4, 8, 16])
def test_even_ring_counts_are_refused(rings):
    """An even count leaves no ring at elevation 0, so every ring is tilted and
    the sliced 2D scan truncates at short range. Fail loudly, not silently."""
    with pytest.raises(ValueError, match="even"):
        LidarSpec(rings=rings, vfov=0.26)


def test_single_ring_with_a_vertical_fov_is_refused():
    """Self-contradictory: one sample spread over a non-zero span is still one
    horizontal ring, so the config would not mean what it says."""
    with pytest.raises(ValueError, match="horizontal"):
        LidarSpec(rings=1, vfov=0.26)


@pytest.mark.parametrize("rings", [1, 9, 17])
def test_render_substitutes_every_placeholder(rings):
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(rings=rings, vfov=0.26 * (rings > 1)))
    assert "{{" not in sdf, "unsubstituted template placeholder left in the SDF"
    assert "robot_0/scan" in sdf
    assert f"<samples>{rings}</samples>" in sdf


def test_render_keeps_the_proximity_lidar_planar():
    """Only the mapping lidar gains rings; the bumper scan stays 2D."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(rings=9, vfov=0.26))
    proximity = sdf.split('<sensor name="proximity_lidar"')[1]
    assert "<samples>1</samples>" in proximity.split("</sensor>")[0]


def test_horizontal_resolution_reaches_the_far_wall():
    """The default profile must put adjacent rays inside one 5 cm grid cell
    across the 24 m building, or distant walls come out dotted — the defect this
    whole profile mechanism exists to fix (docs/KNOWN_ISSUES.md #8)."""
    spec = lidar_spec({})
    spacing_at_12m = 12.0 * math.radians(spec.h_step_deg)
    assert spacing_at_12m < 0.05, (
        f"{spec.h_samples} samples/rev = {spec.h_step_deg:.3f} deg leaves "
        f"{spacing_at_12m * 100:.1f} cm between rays at 12 m"
    )


def test_legacy_profile_reproduces_the_shipped_sensor():
    """Kept as the A/B control for measuring the change, so it must not drift."""
    spec = LIDAR_PROFILES["legacy_360"]
    assert (spec.h_samples, spec.rings, spec.range_max) == (360, 1, 16.0)
    assert spec.h_step_deg == pytest.approx(1.003, abs=1e-3)


def test_lidar_rings_is_still_honoured_as_a_config_alias():
    """Pre-profile study configs must keep working."""
    assert lidar_spec({"lidar_rings": 9}).rings == 9
    assert lidar_spec({"lidar": {"profile": "generic_32"}, "lidar_rings": 1}).rings == 1


def test_profile_fields_can_be_overridden_individually():
    spec = lidar_spec({"lidar": {"profile": "generic_32", "h_samples": 2048}})
    assert spec.h_samples == 2048
    assert spec.rings == LIDAR_PROFILES["generic_32"].rings


def test_dropping_a_3d_profile_to_one_ring_makes_it_horizontal():
    """Otherwise the carried-over vfov would make the spec self-contradictory."""
    spec = lidar_spec({"lidar": {"profile": "generic_32", "rings": 1}})
    assert (spec.rings, spec.vfov) == (1, 0.0)


def test_unknown_profile_and_keys_are_refused():
    with pytest.raises(ValueError, match="unknown lidar profile"):
        lidar_spec({"lidar": {"profile": "nope"}})
    with pytest.raises(ValueError, match="unknown fleet.lidar keys"):
        lidar_spec({"lidar": {"hsamples": 1800}})


@pytest.mark.parametrize("name", sorted(LIDAR_PROFILES))
def test_every_profile_renders(name):
    sdf = render("robot_0", "0.2 0.7 0.9", LIDAR_PROFILES[name])
    assert "{{" not in sdf, f"{name}: unsubstituted placeholder"


def test_imu_is_fast_enough_for_inertial_odometry():
    """LIO packages need >= ~100 Hz; below that they refuse or drift."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec())
    imu = sdf.split('<sensor name="imu"')[1].split("</sensor>")[0]
    rate = int(imu.split("<update_rate>")[1].split("</update_rate>")[0])
    assert rate >= 100


def test_imu_is_noisy():
    """A noiseless Gazebo IMU reports the simulator's exact angular rate, so an
    EKF fused with it would be laundering ground truth and its accuracy would not
    transfer to hardware. Both channels must carry noise and a bias."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec())
    imu = sdf.split('<sensor name="imu"')[1].split("</sensor>")[0]
    for channel in ("angular_velocity", "linear_acceleration"):
        block = imu.split(f"<{channel}>")[1].split(f"</{channel}>")[0]
        assert block.count('<noise type="gaussian">') == 3, f"{channel}: needs x/y/z noise"
        assert "<bias_mean>" in block, f"{channel}: needs a bias, not just white noise"
        stddevs = [
            float(part.split("</stddev>")[0])
            for part in block.split("<stddev>")[1:]
        ]
        assert all(s > 0.0 for s in stddevs), f"{channel}: zero stddev is no noise"


# ------------------------------------------------------------- mixed platforms


def _model(sdf: str):
    import xml.etree.ElementTree as ET

    return ET.fromstring(sdf).find("model")


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_every_platform_renders_valid_sdf_with_the_full_sensor_suite(platform):
    """The shell owns the sensors, so no platform may be missing one."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(), robot_spec(platform))
    model = _model(sdf)
    base = model.find("link")
    assert base.get("name") == "base_link", "the adapter's TF chain keys off this name"
    sensors = {s.get("name") for s in base.findall("sensor")}
    assert sensors == {"lidar", "proximity_lidar", "imu", "camera"}
    # Both plugins: without DiffDrive nothing moves; without OdometryPublisher
    # there is no ground truth to score map merging against.
    assert len(model.findall("plugin")) == 2


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_no_placeholder_survives_rendering(platform):
    """An unreplaced {{LIDAR_Z}} parses as 0 and mounts the lidar in the chassis."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(), robot_spec(platform))
    assert "{{" not in sdf and "}}" not in sdf


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_drive_joints_exist_for_the_joints_the_plugin_names(platform):
    """A DiffDrive naming a joint that does not exist fails silently: the model
    spawns, publishes odometry of zeros, and simply never moves."""
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(), robot_spec(platform))
    model = _model(sdf)
    joints = {j.get("name") for j in model.findall("joint")}
    plugin = next(
        p for p in model.findall("plugin") if p.get("name").endswith("DiffDrive")
    )
    named = {e.text for e in plugin if e.tag in ("left_joint", "right_joint")}
    assert named, "DiffDrive names no joints"
    assert named <= joints, f"DiffDrive names joints that do not exist: {named - joints}"


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_every_robot_scans_for_neighbours_at_the_same_absolute_height(platform):
    """The one thing that lets a tall robot and a short one see each other.

    A Scout Mini is 0.245 m tall overall; a Spot's body floats at 0.40-0.60 m.
    Each platform's bumper scan is therefore offset from ITS base_link so that
    all of them end up at the same height above the floor.
    """
    spec = robot_spec(platform)
    assert spec.base_height + spec.prox_z == pytest.approx(PROXIMITY_SCAN_HEIGHT)


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_the_bumper_scan_starts_outside_the_chassis(platform):
    """Inside it, every scan returns the robot's own body at zero range."""
    spec = robot_spec(platform)
    assert spec.prox_x >= spec.length / 2.0


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_footprint_circumscribes_rather_than_inscribes(platform):
    """Nav2 plans with a circle; an inscribed radius lets a corner clip a wall."""
    spec = robot_spec(platform)
    assert spec.footprint_radius >= max(spec.length, spec.width) / 2.0


def test_platforms_are_actually_different_sizes():
    """Guards against every entry silently collapsing onto one default."""
    radii = {name: robot_spec(name).footprint_radius for name in ROBOT_PROFILES}
    assert len(set(round(r, 3) for r in radii.values())) == len(radii), radii


def test_mixed_fleet_resolves_per_robot_overrides():
    cfg = {"robot_type": "bunker", "robot_types": {"robot_2": "scout_mini",
                                                   "robot_3": "spot"}}
    assert robot_types(cfg, 4, "robot_") == ["bunker", "bunker", "scout_mini", "spot"]


def test_named_robots_and_spot_chassis_keep_identity_colours():
    assert color_for("spot_0", 3) == IDENTITY_COLORS["spot"]
    assert color_for("botman_0", 5) == IDENTITY_COLORS["botman"]
    assert color_for("aslan_0", 6) == IDENTITY_COLORS["aslan"]
    assert color_for("robot_3", 3, "spot") == IDENTITY_COLORS["spot"]
    assert color_for("robot_0", 0, "bunker") != IDENTITY_COLORS["spot"]


def test_a_typo_in_robot_types_is_refused_rather_than_ignored():
    """Silently ignoring `robot_9` would spawn a fleet the operator did not ask
    for, and nothing downstream would say so."""
    with pytest.raises(ValueError, match="not in this fleet"):
        robot_types({"robot_types": {"robot_9": "spot"}}, 4, "robot_")


def test_an_unknown_platform_is_refused():
    with pytest.raises(ValueError, match="unknown robot profile"):
        robot_spec("wall_e")


def test_spawn_height_clears_the_floor_for_every_platform():
    """EntityFactory's pose REPLACES the model's own, so this is the height the
    robot is actually created at. Below base_height it spawns inside the floor."""
    for name in ROBOT_PROFILES:
        spec = robot_spec(name)
        assert spec.spawn_z > spec.base_height


def test_chassis_fragments_declare_every_required_section():
    for name in ROBOT_PROFILES:
        sections = chassis_sections(robot_spec(name).chassis)
        assert set(sections) == set(CHASSIS_SECTIONS)
        assert all(body.strip() for body in sections.values()), name


# ------------------------------------------------------- footprint vs a circle


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_footprint_polygon_is_the_chassis_rectangle(platform):
    spec = robot_spec(platform)
    pts = json.loads(spec.footprint)
    assert len(pts) == 4
    xs = sorted({round(abs(x), 3) for x, _ in pts})
    ys = sorted({round(abs(y), 3) for _, y in pts})
    assert xs == [round(spec.length / 2, 3)]
    assert ys == [round(spec.width / 2, 3)]


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_the_polygon_inscribes_tighter_than_the_circle(platform):
    """The whole reason for passing a footprint to Nav2.

    Cells within the INSCRIBED radius of an obstacle are lethal. A circle has to
    circumscribe, so it inscribes at the circumscribed radius too — a 0.778 m
    wide Bunker becomes lethal within 0.643 m of a wall instead of 0.389 m, and
    the planner refuses gaps the robot fits through.
    """
    spec = robot_spec(platform)
    inscribed = min(spec.length, spec.width) / 2.0
    assert inscribed < spec.footprint_radius
    # And the rectangle must still cover the chassis, not undercut it.
    assert inscribed == pytest.approx(min(spec.length, spec.width) / 2.0)


def test_every_platform_fits_a_door_once_modelled_as_a_rectangle():
    """DOOR is a HALF-width in generate_world.py, so the opening is 2.2 m."""
    opening = 2 * 1.1
    for name in ROBOT_PROFILES:
        spec = robot_spec(name)
        assert spec.width < opening, f"{name} is wider than a door"


# ------------------------------------------------- the lidar must not see itself


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_the_mapping_lidar_clears_the_robots_own_deck(platform):
    """A multi-ring lidar sweeps DOWN as well as out.

    Gazebo's gpu_lidar raytraces the render scene, so it hits visuals — decks,
    masts, and on Spot the legs, none of which are collision geometry. Mounted
    too low, the steepest downward ring lands on the robot's own back and the
    robot reports a ring of obstacles at its own radius: every heading blocked,
    nowhere to go, no error anywhere. Observed on Spot, whose mount was 0.171 m
    below what the 33-ring profile needs.
    """
    spec = robot_spec(platform)
    widest = max(p.vfov for p in LIDAR_PROFILES.values())
    assert spec.lidar_z > spec.min_lidar_z(widest), (
        f"{platform}: lidar at {spec.lidar_z:.3f} m is below the "
        f"{spec.min_lidar_z(widest):.3f} m needed to clear its own deck"
    )


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_deck_geometry_is_declared_so_the_clearance_can_be_checked(platform):
    """min_lidar_z returns 0 for an undeclared deck, which would pass the test
    above vacuously."""
    spec = robot_spec(platform)
    assert spec.deck_top > 0.0 and spec.deck_half_length > 0.0


# --------------------------------------------------- the camera must see depth


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_the_camera_is_an_rgbd_sensor_on_the_topic_the_bridge_expects(platform):
    """Colour and depth must come from one sensor, on Gazebo's own topic names.

    `rgbd_camera` renders both from a single pose and set of intrinsics, which
    is what lets the adapter read a range straight out of a detection box. Two
    separate sensors would need calibrating to agree, and would agree worst at
    the edges of frame, where a detection usually is.

    The base topic is load-bearing: Gazebo appends `/image`, `/depth_image`,
    `/camera_info` and `/points` to it, and those are the names
    session.launch.py bridges and adapter_sim subscribes to. A bridge for a
    topic Gazebo does not publish is silent — it presents as a camera that
    never appears, not as an error.
    """
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(), robot_spec(platform))
    camera = _model(sdf).find("link").find("sensor[@name='camera']")
    assert camera.get("type") == "rgbd_camera"
    assert camera.find("topic").text == "robot_0/camera"


@pytest.mark.parametrize("platform", sorted(ROBOT_PROFILES))
def test_the_camera_sits_where_the_adapter_believes_it_does(platform):
    """adapter_sim composes optical -> base_link with this mount, from the same
    profile table. The SDF and RobotBridge.camera_x/z must stay one number."""
    spec = robot_spec(platform)
    sdf = render("robot_0", "0.2 0.7 0.9", LidarSpec(), robot_spec(platform))
    camera = _model(sdf).find("link").find("sensor[@name='camera']")
    x, y, z, roll, pitch, yaw = (float(v) for v in camera.find("pose").text.split())
    assert (x, z) == (pytest.approx(spec.camera_x), pytest.approx(spec.camera_z))
    # Straight ahead and level: the adapter's optical -> base_link step is a
    # fixed axis relabel with no rotation of its own to apply.
    assert (y, roll, pitch, yaw) == (0.0, 0.0, 0.0, 0.0)
