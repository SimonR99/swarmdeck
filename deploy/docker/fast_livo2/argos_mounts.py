#!/usr/bin/env python3
"""Read each robot's sensor mounts out of the generated ARGoS experiment.

Fast-LIVO2 corrects for where the LiDAR and camera are mounted relative to the
IMU/body, so an incorrect extrinsic is a systematic bias that the estimator cannot
recover from. While single-robot setups can use fixed environment variables,
SwarmDeck's fleet is heterogeneous:
  - Scout Mini: LiDAR at 0.4525 m, camera at 0.3525 m
  - Bunker:     LiDAR at 0.7200 m, camera at 0.6200 m
  - Spot:       LiDAR at 0.9700 m, camera at 0.8700 m

Reading from the experiment file guarantees exact consistency with what ARGoS runs.

Usage:
    argos_mounts.py /run/swarmdeck/session.argos --robots
    argos_mounts.py /run/swarmdeck/session.argos --lidar robot_0
    argos_mounts.py /run/swarmdeck/session.argos --camera robot_0
"""

from __future__ import annotations

import argparse
import sys
from xml.etree import ElementTree


def controllers(root):
    """Map robot id -> its controller element, via the arena entities."""
    blocks = {c.get("id"): c for c in root.find("controllers")}
    out = {}
    for entity in root.find("arena"):
        controller = entity.find("controller")
        if controller is None:
            continue
        block = blocks.get(controller.get("config"))
        if block is not None:
            out[entity.get("id")] = block
    return out


def mount(block, tag: str) -> str:
    node = block.find(f"./sensors/{tag}")
    if node is None:
        raise SystemExit(f"no <{tag}> in {block.get('id')}")
    return node.get("position", "0,0,0")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("experiment")
    ap.add_argument("--robots", action="store_true",
                    help="Print the robot ids, in order, space separated")
    ap.add_argument("--lidar", metavar="ROBOT",
                    help="Print that robot's lidar mount as 'x y z'")
    ap.add_argument("--camera", metavar="ROBOT",
                    help="Print that robot's camera mount as 'x y z'")
    ap.add_argument("--lidar-elev", metavar="ROBOT",
                    help="Print that robot's lidar vertical FOV as 'min max'")
    ap.add_argument("--camera-resolution", metavar="ROBOT")
    ap.add_argument("--camera-fov", metavar="ROBOT")
    args = ap.parse_args()

    root = ElementTree.parse(args.experiment).getroot()
    blocks = controllers(root)

    if args.robots:
        print(" ".join(blocks))
        return 0
    for flag, tag, attr in (
        (args.lidar, "photorealistic_lidar", "position"),
        (args.camera, "photorealistic_camera", "position"),
        (args.lidar_elev, "photorealistic_lidar", "vertical_fov"),
        (args.camera_resolution, "photorealistic_camera", "resolution"),
        (args.camera_fov, "photorealistic_camera", "fov"),
    ):
        if not flag:
            continue
        block = blocks.get(flag)
        if block is None:
            raise SystemExit(f"no robot {flag!r} in {args.experiment}")
        node = block.find(f"./sensors/{tag}")
        if node is None:
            raise SystemExit(f"no <{tag}> for {flag}")
        # run_fast_livo.sh word-splits these, so commas become spaces.
        print(node.get(attr, "").replace(",", " "))
        return 0
    ap.error("nothing to print")
    return 2


if __name__ == "__main__":
    sys.exit(main())
