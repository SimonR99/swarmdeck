#!/usr/bin/env python3
"""Reactive wander-and-explore, so robots build usable maps without Nav2.

    python3 explore.py --robots 2 --seconds 180

Why this exists: open-loop driving jams robots into walls. A stuck differential
drive keeps spinning its wheels, so the DiffDrive plugin integrates motion that
never happened — we measured odometry claiming (+5.1, -19.3) while the robot was
actually at (-8.0, -2.1). SLAM Toolbox uses odometry as its motion prior, so bad
odometry means a garbage map, and map registration cannot possibly work on top of
that.

Keeping robots off the walls is therefore a prerequisite for mapping, not a
convenience. This is deliberately reactive (no planner, no costmap) so it can run
before Nav2 is wired up.
"""

from __future__ import annotations

import argparse
import math
import random
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

STOP_DIST = 0.9       # metres: rotate in place below this ahead
CLEAR_DIST = 1.6      # metres: considered open
MAX_LIN = 0.45
MAX_ANG = 0.8


class Explorer(Node):
    def __init__(self, robot_ids: list[str], seed: int = 0) -> None:
        super().__init__("swarmdeck_explorer")
        self.rng = random.Random(seed)
        self.scan: dict[str, LaserScan] = {}
        self.pubs: dict[str, object] = {}
        self.turn_dir: dict[str, float] = {}
        self.stuck_since: dict[str, float] = {}

        for rid in robot_ids:
            self.create_subscription(
                LaserScan, f"/{rid}/scan",
                (lambda r: lambda m: self.scan.__setitem__(r, m))(rid), 10,
            )
            self.pubs[rid] = self.create_publisher(Twist, f"/{rid}/cmd_vel", 10)
            self.turn_dir[rid] = 1.0
            self.stuck_since[rid] = 0.0

        self.create_timer(0.1, self.step)

    @staticmethod
    def _sector(scan: LaserScan, centre_deg: float, half_deg: float) -> float:
        """Minimum finite range in an angular sector, metres (inf if empty)."""
        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        lo, hi = math.radians(centre_deg - half_deg), math.radians(centre_deg + half_deg)
        sel = (ang >= lo) & (ang <= hi) & np.isfinite(r) & (r > scan.range_min)
        return float(r[sel].min()) if sel.any() else float("inf")

    def step(self) -> None:
        for rid, pub in self.pubs.items():
            s = self.scan.get(rid)
            if s is None:
                continue

            front = self._sector(s, 0, 25)
            left = self._sector(s, 55, 30)
            right = self._sector(s, -55, 30)

            cmd = Twist()
            if front > CLEAR_DIST:
                # Open ahead: run, with a gentle bias away from the nearer side.
                cmd.linear.x = MAX_LIN
                bias = 0.0
                if min(left, right) < CLEAR_DIST:
                    bias = -0.5 if left < right else 0.5
                cmd.angular.z = bias + self.rng.uniform(-0.06, 0.06)
                self.stuck_since[rid] = 0.0
            elif front > STOP_DIST:
                cmd.linear.x = 0.18
                cmd.angular.z = MAX_ANG * (1.0 if left > right else -1.0)
            else:
                # Blocked: rotate in place toward the more open side. Commit to a
                # direction so we do not oscillate in a corner.
                now = time.monotonic()
                if self.stuck_since[rid] == 0.0:
                    self.stuck_since[rid] = now
                    self.turn_dir[rid] = 1.0 if left > right else -1.0
                cmd.linear.x = -0.12 if now - self.stuck_since[rid] > 4.0 else 0.0
                cmd.angular.z = MAX_ANG * self.turn_dir[rid]
            pub.publish(cmd)

    def halt(self) -> None:
        for pub in self.pubs.values():
            pub.publish(Twist())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=2)
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rclpy.init()
    ids = [f"{args.prefix}{i}" for i in range(args.robots)]
    node = Explorer(ids, seed=args.seed)
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    node.halt()
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
