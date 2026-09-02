#!/usr/bin/env python3
"""Read each robot's sensor mounts out of the generated ARGoS experiment.

Ultra-Fusion corrects for where the lidar is bolted, so a wrong extrinsic is a
bias the estimator cannot see and cannot recover from. The reference tooling
takes it from a `LIDAR_IN_BODY` environment variable, which is fine for a fleet
of identical robots and wrong for SwarmDeck's: a Scout Mini carries its lidar at
0.4525 m, a Bunker at 0.720 m and a Spot at 0.970 m, all in the same run.

So it is read from the experiment file instead, which is the same file ARGoS is
running. The two cannot disagree.

    argos_mounts.py /run/swarmdeck/session.argos --robots
    argos_mounts.py /run/swarmdeck/session.argos --lidar robot_0
"""

from __future__ import annotations

import argparse
import sys
from xml.etree import ElementTree


def controllers(root):
    """Map robot id -> its controller element, via the arena entities.

    The entity names the controller config, and the controller carries the
    sensors; neither alone says which robot a mount belongs to.
    """
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
        # run_uf.sh word-splits these, so commas become spaces.
        print(node.get(attr, "").replace(",", " "))
        return 0
    ap.error("nothing to print")
    return 2


if __name__ == "__main__":
    sys.exit(main())
