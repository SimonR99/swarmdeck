"""Run one onboard MGG instance using the robot adapter's configuration."""

import importlib.util
import json
import os
from pathlib import Path

import yaml
from launch import LaunchDescription


def hardware_settings(config):
    exploration = config["exploration"]
    planner = exploration["planner"]
    footprint = config["footprint"]
    xs, ys = zip(*footprint)
    height = float(planner["body_height_m"])
    return {
        "RobotParams.size": [max(xs) - min(xs), max(ys) - min(ys), height],
        "RobotParams.center_offset": [
            (max(xs) + min(xs)) / 2,
            (max(ys) + min(ys)) / 2,
            0.0,
        ],
        "PlanningParams.robot_height": height,
        "PlanningParams.max_ground_height": height / 2 + 0.175,
        "BoundedSpaceParams.Global.min_val": planner["bounds_min"],
        "BoundedSpaceParams.Global.max_val": planner["bounds_max"],
    }


def hardware_fleet(robot):
    """(MGG robot id, other robots) from SWARMDECK_PEER_NAMES (a JSON list).

    The merge keys roadmaps by the 1-based robot id, which is the robot's
    position in that list, and drops one carrying its own id. The others'
    roadmaps are placed with C-SLAM's estimates: a robot has no ground truth,
    so `cslam` is the only source here."""
    if (os.environ.get("SWARMDECK_ROBOT_POSES") or "cslam") != "cslam":
        raise ValueError("hardware roadmap sharing supports only cslam robot poses")
    names = json.loads(os.environ.get("SWARMDECK_PEER_NAMES") or "[]")
    if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
        raise ValueError("SWARMDECK_PEER_NAMES must be a JSON list of robot names")
    index = names.index(robot) + 1 if robot in names else 1
    return index, [name for name in names if name != robot]


def generate_launch_description():
    here = Path(__file__).parent
    spec = importlib.util.spec_from_file_location("mgg_robot", here / "robot.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with open(os.environ["SWARMDECK_ROBOT_CONFIG"]) as stream:
        config = yaml.safe_load(stream)
    planner = config["exploration"]["planner"]
    topics = config["topics"]
    robot = os.environ["SWARMDECK_ROBOT_ID"]
    robot_index, peers = hardware_fleet(robot)
    return LaunchDescription(
        module.robot_nodes(
            robot,
            config["navigation_frame"],
            topics["odom"],
            "/tf",
            "/tf_static",
            os.environ.get("MGG_PARAMS_FILE", str(here / "config/hardware.yaml")),
            False,
            robot_index=robot_index,
            planner_overrides=hardware_settings(config),
            peers=peers,
            robot_poses="cslam",
        )
    )
