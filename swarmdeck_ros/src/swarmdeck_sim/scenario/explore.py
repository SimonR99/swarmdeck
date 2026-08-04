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

Steering uses the mapping lidar *and* the bumper-height proximity scan. The mapping
lidar sits at 0.402 m and looks straight over another Duckiebot's body, so on the
mapping scan alone the fleet is invisible to itself and robots jam into each other
— which is the same wheel-slip failure as jamming into a wall.

Wandering alternates with **homing**, and that is not decoration. A pose graph is
only corrected where it closes a loop, and pure reactive wandering closes loops by
luck: it is a random walk, so it may or may not revisit anywhere before the run
ends. Driving back to the start every `--loop-period` seconds makes at least one
large loop closure per cycle a property of the run rather than an accident, which
is what pulls accumulated drift out of the whole graph — and it is also what gives
several robots overlapping coverage, without which map registration is ill-posed
(docs/KNOWN_ISSUES.md #2) and inter-robot loop closure has nothing to match.

Homing steers on `<ns>/odom`, which drifts. That is deliberate: it is the same
information the robot itself has, and homing only has to get near enough for SLAM
to recognise the place. Obstacle avoidance always outranks it, so a homing robot
still never drives into a wall.

`--seconds` is wall-clock, not simulation time, because it exists to bound how long
an operator waits. Under a real-time factor below 1 the fleet covers proportionally
less ground.
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

STOP_DIST = 0.9       # metres: rotate in place below this ahead
CLEAR_DIST = 1.6      # metres: considered open
MAX_LIN = 0.45
MAX_ANG = 0.8

# Close enough to the start pose to count as a revisit. Comfortably inside
# slam_toolbox.yaml's loop_search_maximum_distance (3.0 m), so arriving here puts
# the robot where the loop closer is already looking.
HOME_RADIUS = 1.5
# Give up on a homing leg after this long and go back to exploring, rather than
# grinding against whatever is in the way.
HOME_TIMEOUT = 75.0
# Proportional gain from bearing error to yaw rate while homing.
HOME_GAIN = 1.2


class Explorer(Node):
    def __init__(
        self, robot_ids: list[str], seed: int = 0, loop_period: float = 90.0
    ) -> None:
        super().__init__("swarmdeck_explorer")
        self.rng = random.Random(seed)
        self.scan: dict[str, LaserScan] = {}
        self.pubs: dict[str, object] = {}
        self.turn_dir: dict[str, float] = {}
        self.stuck_since: dict[str, float] = {}
        self.running = True
        self.loop_period = loop_period

        self.bumper: dict[str, LaserScan] = {}
        self.pose: dict[str, tuple[float, float, float]] = {}
        self.origin: dict[str, tuple[float, float]] = {}
        self.phase: dict[str, str] = {}
        self.phase_since: dict[str, float] = {}

        for rid in robot_ids:
            # Sensor-data QoS (BEST_EFFORT), not the default RELIABLE. Which node
            # publishes `<ns>/scan` depends on the SLAM backend: the ros_gz bridge
            # on the 2D path, `pointcloud_to_laserscan` on the 3D one — and the
            # latter publishes BEST_EFFORT, which a RELIABLE subscriber may not
            # receive at all. A BEST_EFFORT subscriber is compatible with both.
            #
            # This is not cosmetic. With a RELIABLE subscription on the 3D path
            # this node received no scans, so `step()` skipped every robot, no
            # cmd_vel was ever published, the fleet sat still for the whole run,
            # and RTAB-Map reported a one-node map. Nothing logged an error:
            # incompatible QoS is a warning on the PUBLISHER's side.
            self.create_subscription(
                LaserScan, f"/{rid}/scan",
                (lambda r: lambda m: self.scan.__setitem__(r, m))(rid),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                LaserScan, f"/{rid}/proximity_scan",
                (lambda r: lambda m: self.bumper.__setitem__(r, m))(rid),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry, f"/{rid}/odom",
                (lambda r: lambda m: self._on_odom(r, m))(rid), 10,
            )
            self.pubs[rid] = self.create_publisher(Twist, f"/{rid}/cmd_vel", 10)
            self.turn_dir[rid] = 1.0
            self.stuck_since[rid] = 0.0
            self.phase[rid] = "wander"
            self.phase_since[rid] = time.monotonic()

        self.create_timer(0.1, self.step)
        # A fleet that never moves is the failure this file exists to prevent,
        # and it is silent: `step()` simply skips a robot with no scan. Say so.
        self._started = time.monotonic()
        self._silence_reported = False
        self.create_timer(5.0, self._check_scans)

    def _check_scans(self) -> None:
        if self._silence_reported or time.monotonic() - self._started < 15.0:
            return
        missing = [rid for rid in self.pubs if self.scan.get(rid) is None]
        if missing:
            self._silence_reported = True
            self.get_logger().error(
                f"no scans from {missing} after 15 s: these robots will not move. "
                f"Check that <ns>/scan is published and that its QoS is "
                f"compatible (this node subscribes BEST_EFFORT)."
            )

    def _on_odom(self, rid: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose[rid] = (p.x, p.y, yaw)
        # The first sample is the start pose, which is what homing aims at. Odom
        # starts at the spawn pose, so this is (0, 0) in practice — taking it
        # from the data anyway keeps this correct if that ever stops being true.
        self.origin.setdefault(rid, (p.x, p.y))

    @staticmethod
    def _sector(scan: LaserScan, centre_deg: float, half_deg: float) -> float:
        """Minimum finite range in an angular sector, metres (inf if empty)."""
        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        lo, hi = math.radians(centre_deg - half_deg), math.radians(centre_deg + half_deg)
        sel = (ang >= lo) & (ang <= hi) & np.isfinite(r) & (r > scan.range_min)
        return float(r[sel].min()) if sel.any() else float("inf")

    def _nearest(self, rid: str, centre_deg: float, half_deg: float) -> float:
        """Closest obstacle in a sector across every scanner that can see it."""
        scans = [s for s in (self.scan.get(rid), self.bumper.get(rid)) if s is not None]
        return min(
            (self._sector(s, centre_deg, half_deg) for s in scans),
            default=float("inf"),
        )

    def _home_error(self, rid: str) -> tuple[float, float] | None:
        """Distance and bearing to the robot's start pose, in its own frame."""
        pose = self.pose.get(rid)
        origin = self.origin.get(rid)
        if pose is None or origin is None:
            return None
        dx, dy = origin[0] - pose[0], origin[1] - pose[1]
        bearing = math.atan2(dy, dx) - pose[2]
        return math.hypot(dx, dy), (bearing + math.pi) % (2 * math.pi) - math.pi

    def _advance_phase(self, rid: str, now: float) -> None:
        """Alternate exploring with a leg back to the start, so the pose graph
        gets a large loop closure per cycle instead of hoping for one."""
        if self.loop_period <= 0:
            return
        elapsed = now - self.phase_since[rid]
        if self.phase[rid] == "wander":
            if elapsed > self.loop_period and self._home_error(rid) is not None:
                self.phase[rid] = "home"
                self.phase_since[rid] = now
            return
        error = self._home_error(rid)
        if error is None or error[0] < HOME_RADIUS or elapsed > HOME_TIMEOUT:
            self.phase[rid] = "wander"
            self.phase_since[rid] = now

    def step(self) -> None:
        if not self.running:
            return
        now = time.monotonic()
        for rid, pub in self.pubs.items():
            if self.scan.get(rid) is None:
                continue
            self._advance_phase(rid, now)

            front = self._nearest(rid, 0, 25)
            left = self._nearest(rid, 55, 30)
            right = self._nearest(rid, -55, 30)

            cmd = Twist()
            if front > CLEAR_DIST:
                # Open ahead: run, with a gentle bias away from the nearer side.
                cmd.linear.x = MAX_LIN
                bias = 0.0
                if min(left, right) < CLEAR_DIST:
                    bias = -0.5 if left < right else 0.5
                cmd.angular.z = bias + self.rng.uniform(-0.06, 0.06)
                self.stuck_since[rid] = 0.0

                # Homing only ever steers where the reactive controller has
                # already decided it is safe to drive — the wall-avoidance bias
                # stays in the sum, so a robot heading home still gives way to
                # whatever is beside it.
                error = self._home_error(rid) if self.phase[rid] == "home" else None
                if error is not None:
                    _, bearing = error
                    cmd.angular.z = max(
                        -MAX_ANG, min(MAX_ANG, bias + HOME_GAIN * bearing)
                    )
                    if abs(bearing) > 0.8:
                        cmd.linear.x = 0.2
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
        # Disable the timer first. Otherwise it can overwrite this zero command
        # during the grace period before rclpy shuts down.
        self.running = False
        for pub in self.pubs.values():
            pub.publish(Twist())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=2)
    ap.add_argument("--prefix", default="robot_")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--loop-period", type=float, default=90.0,
                    help="seconds of exploring between legs back to the start "
                         "pose, which is what guarantees a loop closure per "
                         "cycle. 0 disables homing and wanders throughout.")
    args = ap.parse_args()

    rclpy.init()
    ids = [f"{args.prefix}{i}" for i in range(args.robots)]
    node = Explorer(ids, seed=args.seed, loop_period=args.loop_period)
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    node.halt()
    time.sleep(0.5)
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
