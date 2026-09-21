"""Run one onboard MGG instance using the robot adapter's configuration."""

import importlib.util
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


def generate_launch_description():
    here = Path(__file__).parent
    spec = importlib.util.spec_from_file_location("mgg_robot", here / "robot.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with open(os.environ["SWARMDECK_ROBOT_CONFIG"]) as stream:
        config = yaml.safe_load(stream)
    planner = config["exploration"]["planner"]
    topics = config["topics"]
    return LaunchDescription(
        module.robot_nodes(
            os.environ["SWARMDECK_ROBOT_ID"],
            config["navigation_frame"],
            topics["odom"],
            "/tf",
            "/tf_static",
            os.environ.get("MGG_PARAMS_FILE", str(here / "config/hardware.yaml")),
            False,
            planner_overrides=hardware_settings(config),
        )
    )
