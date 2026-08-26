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

Wandering alternates with two kinds of long-range leg, and neither is
decoration.

**Homing** returns a robot to its own start pose, which makes a large
INTRA-robot loop closure a property of the run. **Muster** sends the whole fleet
to one shared point, which is the only thing that makes INTER-robot closures
reliable: they are a different problem, and homing does not solve it. Scheduling
pairs instead of the whole fleet was tried and is too slow — O(n^2) cycles of
several minutes each, so robots that spawned far apart never met.

On homing: A pose graph is
only corrected where it closes a loop, and pure reactive wandering closes loops by
luck: it is a random walk, so it may or may not revisit anywhere before the run
ends. Driving back to the start every `--loop-period` seconds makes at least one
large loop closure per cycle a property of the run rather than an accident, which
is what pulls accumulated drift out of the whole graph — and it is also what gives
several robots overlapping coverage, without which map registration is ill-posed
(docs/operations/known-issues.md, "Auto map registration") and inter-robot loop
closure has nothing to match.

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
import json
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

# Clearances are measured from the CHASSIS, not from base_link, because the
# fleet is no longer one size. These are the margins added to a robot's
# circumscribed radius; the absolute thresholds fall out per platform.
#
# Calibrated against the constants these replaced: the Duckiebot this file was
# tuned on had a circumscribed radius of 0.27 m, and 0.27 + 0.63 = 0.90 m and
# 0.27 + 1.33 = 1.60 m reproduce the old STOP_DIST and CLEAR_DIST exactly. So a
# fleet of Duckiebots behaves identically, and an AgileX Bunker (r = 0.64 m)
# now stops at 1.27 m instead of driving to 0.90 m and wedging its nose.
STOP_CLEARANCE = 0.63  # metres beyond the chassis: rotate in place below this
OPEN_CLEARANCE = 1.33  # metres beyond the chassis: considered open
# A robot rotating in place sweeps its circumscribed radius in every direction.
# Committing to a turn without this much room to the side is how two robots that
# have already stopped for each other grind together instead of backing off —
# measured with an AgileX Bunker and a Spot 1.44 m apart, combined radii 1.25 m.
TURN_CLEARANCE = 0.25
# Fallback radius for a robot whose platform was not supplied.
DEFAULT_RADIUS = 0.27
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

# A robot commanded to drive that covers less than this in WEDGE_WINDOW seconds
# is wedged, not slow. 0.12 m over 8 s is far below anything the reactive
# controller produces when it is actually moving, so this cannot fire on a robot
# that is merely turning in place.
WEDGE_DISTANCE = 0.12
WEDGE_WINDOW = 8.0
# How long to spend backing out once wedged. Long enough to clear whatever the
# robot climbed onto, short enough not to reverse across the building.
WEDGE_ESCAPE = 3.5

# Seconds a rendezvous leg may take before the pair gives up and wanders again.
#
# Generous on purpose, and the number is arithmetic rather than taste. With the
# cslam stack running, the simulation is slow enough that robots cover roughly
# 1.4 cm of ground per wall-clock second, so a 6 m leg — the distance between
# neighbouring spawn poses in this building — takes over 400 s. At the previous
# 110 s every pair timed out short of the meeting point and wandered off, which
# looks exactly like "rendezvous does not work".
RENDEZVOUS_TIMEOUT = 480.0
# Close enough to a meeting point to count as arrived. Larger than HOME_RADIUS
# because two robots converging on one spot must not try to occupy it.
RENDEZVOUS_RADIUS = 2.5


class Explorer(Node):
    def __init__(
        self,
        robot_ids: list[str],
        seed: int = 0,
        loop_period: float = 90.0,
        starts: dict[str, tuple[float, float, float]] | None = None,
        radii: dict[str, float] | None = None,
    ) -> None:
        super().__init__("swarmdeck_explorer")
        self.rng = random.Random(seed)
        # Circumscribed radius per robot. Everything this class treats as a
        # distance threshold is derived from it, so a mixed fleet stops at the
        # right range for each platform rather than at one Duckiebot's range.
        self.radius = {rid: (radii or {}).get(rid, DEFAULT_RADIUS) for rid in robot_ids}
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
        # World-frame spawn poses, from the study config. Needed because every
        # robot steers in its OWN odom frame, so a shared meeting point can only
        # be expressed per robot if we know where each one started.
        self.starts: dict[str, tuple[float, float, float]] = dict(starts or {})
        self.rendezvous_target: dict[str, tuple[float, float]] = {}
        self.muster: tuple[float, float] | None = None
        # Wedge detection. A differential drive jammed against geometry keeps
        # turning its wheels, so neither the scan nor the commanded velocity
        # reveals it — only the absence of actual displacement does. Left
        # undetected this is not a cosmetic problem: a wedged robot stops
        # producing keyframes, so it never closes an inter-robot loop, and in
        # Swarm-SLAM the LOWEST-ID robot is elected optimizer whether or not it
        # is connected to anyone. One wedged robot_0 therefore stalls the whole
        # fleet's collaborative optimisation, silently.
        self.wedge_until: dict[str, float] = {}
        self.wedge_ref: dict[str, tuple[float, float, float]] = {}
        self.wedge_reported: set[str] = set()
        self.cycle = 0
        self.cycle_since = time.monotonic()

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
                LaserScan,
                f"/{rid}/scan",
                (lambda r: lambda m: self.scan.__setitem__(r, m))(rid),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                LaserScan,
                f"/{rid}/proximity_scan",
                (lambda r: lambda m: self.bumper.__setitem__(r, m))(rid),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry,
                f"/{rid}/odom",
                (lambda r: lambda m: self._on_odom(r, m))(rid),
                10,
            )
            self.pubs[rid] = self.create_publisher(Twist, f"/{rid}/cmd_vel", 10)
            self.turn_dir[rid] = 1.0
            self.stuck_since[rid] = 0.0
            self.wedge_until[rid] = 0.0
            self.wedge_ref[rid] = None
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
        lo, hi = math.radians(centre_deg - half_deg), math.radians(
            centre_deg + half_deg
        )
        sel = (ang >= lo) & (ang <= hi) & np.isfinite(r) & (r > scan.range_min)
        return float(r[sel].min()) if sel.any() else float("inf")

    def _nearest(self, rid: str, centre_deg: float, half_deg: float) -> float:
        """Closest obstacle in a sector across every scanner that can see it."""
        scans = [s for s in (self.scan.get(rid), self.bumper.get(rid)) if s is not None]
        return min(
            (self._sector(s, centre_deg, half_deg) for s in scans),
            default=float("inf"),
        )

    def _error_to(
        self, rid: str, target: tuple[float, float]
    ) -> tuple[float, float] | None:
        """Distance and bearing from a robot to a target in its OWN odom frame."""
        pose = self.pose.get(rid)
        if pose is None:
            return None
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        bearing = math.atan2(dy, dx) - pose[2]
        return math.hypot(dx, dy), (bearing + math.pi) % (2 * math.pi) - math.pi

    def _home_error(self, rid: str) -> tuple[float, float] | None:
        """Distance and bearing to the robot's start pose, in its own frame."""
        origin = self.origin.get(rid)
        return None if origin is None else self._error_to(rid, origin)

    def _muster_point(self, cycle: int) -> tuple[float, float] | None:
        """Where the WHOLE fleet is asked to gather this cycle, in world coords.

        One shared point rather than a scheduled pair, and the reason is the
        arithmetic. Pairwise rotation needs O(n^2) cycles to give every pair a
        chance — 6 cycles for four robots — and each leg can take minutes, so a
        full rotation ran for about an hour and robots that started far apart
        never met at all. A single muster point exposes every pair
        simultaneously, so one cycle is worth the whole rotation.

        The point moves between cycles: the fleet centroid, then the four
        quadrant centres of the area the robots span. Gathering at the same spot
        forever would close the same loops repeatedly and stop the fleet
        exploring, which is the opposite of what this is for.
        """
        if not self.starts:
            return None
        xs = [p[0] for p in self.starts.values()]
        ys = [p[1] for p in self.starts.values()]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        # Inset from the extremes: the corners of the spawn box are usually
        # against a wall, and a muster point inside furniture is unreachable.
        rx = max(1.0, (max(xs) - min(xs)) * 0.25)
        ry = max(1.0, (max(ys) - min(ys)) * 0.25 or rx)
        offsets = [(0.0, 0.0), (-rx, -ry), (rx, -ry), (rx, ry), (-rx, ry)]
        dx, dy = offsets[cycle % len(offsets)]
        return (cx + dx, cy + dy)

    def _rendezvous_error(self, rid: str) -> tuple[float, float] | None:
        """Error to this cycle's meeting point, or None if not participating.

        The meeting point is the midpoint of the pair's two start poses, in
        world coordinates, converted into this robot's odom frame. Start poses
        come from the study config, not from sensing — the same information
        `merge_mode: static` already uses. Ground truth stays out of the loop.
        """
        target = self.rendezvous_target.get(rid)
        return None if target is None else self._error_to(rid, target)

    def _set_rendezvous(self, cycle: int) -> None:
        """Resolve this cycle's muster point into every robot's own frame."""
        self.rendezvous_target.clear()
        point = self._muster_point(cycle)
        if point is None:
            return
        self.muster = point
        for rid, (sx, sy, syaw) in self.starts.items():
            dx, dy = point[0] - sx, point[1] - sy
            c, s = math.cos(-syaw), math.sin(-syaw)
            # World -> that robot's odom frame, whose origin is its spawn pose.
            self.rendezvous_target[rid] = (dx * c - dy * s, dx * s + dy * c)

    def _advance_phase(self, rid: str, now: float) -> None:
        """Cycle exploring with two kinds of long-range leg.

        `home` returns a robot to its own start pose, which makes a large
        INTRA-robot loop closure a property of the run rather than an accident.
        `rendezvous` sends a scheduled pair to a shared meeting point, which is
        the only thing that makes INTER-robot closures reliable — the two are
        different problems and homing alone does not solve the second.
        """
        if self.loop_period <= 0:
            return
        elapsed = now - self.phase_since[rid]
        if self.phase[rid] == "wander":
            if elapsed <= self.loop_period:
                return
            # Rendezvous whenever this robot is scheduled, and home otherwise.
            # Alternating cycles halved the rate at which pairs could meet, and
            # intra-robot loop closure is already abundant (~100 keyframes per
            # robot per run) while INTER-robot encounters are the scarce thing.
            if self._rendezvous_error(rid) is not None:
                self.phase[rid] = "rendezvous"
                self.phase_since[rid] = now
            elif self._home_error(rid) is not None:
                self.phase[rid] = "home"
                self.phase_since[rid] = now
            return

        if self.phase[rid] == "rendezvous":
            error = self._rendezvous_error(rid)
            done = (
                error is None
                or error[0] < RENDEZVOUS_RADIUS
                or elapsed > RENDEZVOUS_TIMEOUT
            )
        else:
            error = self._home_error(rid)
            done = error is None or error[0] < HOME_RADIUS or elapsed > HOME_TIMEOUT

        if done:
            self.phase[rid] = "wander"
            self.phase_since[rid] = now

    def _advance_cycle(self) -> None:
        """Rotate the pair schedule, at most once per cycle period.

        The gate is its OWN timestamp, not the robots' phase timers. Keying it
        off `phase_since` instead let the cycle re-fire on every 10 Hz tick once
        the robots had been wandering long enough — 130 cycles in under a
        second, so no pair was ever scheduled long enough to actually travel
        anywhere.
        """
        if not self.starts:
            return
        now = time.monotonic()
        elapsed = now - self.cycle_since
        if elapsed < max(self.loop_period, 1.0):
            return
        # Normally wait for the fleet to finish its legs before re-scheduling,
        # but never wait forever: a pair grinding against furniture until its
        # timeout would otherwise freeze the whole rotation and starve every
        # other pair of the chance to meet.
        if elapsed < RENDEZVOUS_TIMEOUT + self.loop_period and not all(
            phase == "wander" for phase in self.phase.values()
        ):
            return
        self.cycle += 1
        self.cycle_since = now
        self._set_rendezvous(self.cycle)
        if self.muster is not None:
            self.get_logger().info(
                f"cycle {self.cycle}: fleet muster at "
                f"({self.muster[0]:.1f}, {self.muster[1]:.1f})"
            )

    def _wedged(self, rid: str, now: float) -> bool:
        """Has this robot stopped moving despite being driven?

        Compares commanded intent against actual odometry displacement over a
        window. Uses `<ns>/odom` deliberately: wheel odometry is the channel
        that CANNOT see slip, so if even it reports no displacement the robot is
        not merely slipping, it is pinned.
        """
        pose = self.pose.get(rid)
        if pose is None:
            return False
        ref = self.wedge_ref.get(rid)
        if ref is None or now - ref[2] > WEDGE_WINDOW:
            moved = (
                math.hypot(pose[0] - ref[0], pose[1] - ref[1])
                if ref is not None
                else float("inf")
            )
            self.wedge_ref[rid] = (pose[0], pose[1], now)
            if ref is not None and moved < WEDGE_DISTANCE:
                if rid not in self.wedge_reported:
                    self.wedge_reported.add(rid)
                    self.get_logger().warn(
                        f"{rid} moved {moved:.2f} m in {WEDGE_WINDOW:.0f} s while "
                        f"being driven: wedged, backing out"
                    )
                return True
        return False

    def step(self) -> None:
        if not self.running:
            return
        now = time.monotonic()
        self._advance_cycle()
        for rid, pub in self.pubs.items():
            if self.scan.get(rid) is None:
                continue
            self._advance_phase(rid, now)

            # A wedged robot outranks every other behaviour: it cannot explore,
            # cannot map, and — because it stops producing keyframes — silently
            # stalls the fleet's collaborative optimisation if it happens to be
            # the lowest-ID robot. Back out and turn hard before doing anything
            # else.
            if now < self.wedge_until.get(rid, 0.0):
                escape = Twist()
                escape.linear.x = -0.22
                escape.angular.z = 1.0 * self.turn_dir[rid]
                pub.publish(escape)
                continue
            if self._wedged(rid, now):
                self.wedge_until[rid] = now + WEDGE_ESCAPE
                self.turn_dir[rid] = -self.turn_dir[rid]
                continue

            front = self._nearest(rid, 0, 25)
            left = self._nearest(rid, 55, 30)
            right = self._nearest(rid, -55, 30)

            # Per platform, from its own chassis. A Bunker needs to begin
            # turning a third of a metre earlier than a Scout Mini does.
            radius = self.radius.get(rid, DEFAULT_RADIUS)
            stop_dist = radius + STOP_CLEARANCE
            clear_dist = radius + OPEN_CLEARANCE
            turn_dist = radius + TURN_CLEARANCE

            cmd = Twist()
            if front > clear_dist:
                # Open ahead: run, with a gentle bias away from the nearer side.
                cmd.linear.x = MAX_LIN
                bias = 0.0
                if min(left, right) < clear_dist:
                    bias = -0.5 if left < right else 0.5
                cmd.angular.z = bias + self.rng.uniform(-0.06, 0.06)
                self.stuck_since[rid] = 0.0

                # Homing only ever steers where the reactive controller has
                # already decided it is safe to drive — the wall-avoidance bias
                # stays in the sum, so a robot heading home still gives way to
                # whatever is beside it.
                if self.phase[rid] == "home":
                    error = self._home_error(rid)
                elif self.phase[rid] == "rendezvous":
                    error = self._rendezvous_error(rid)
                else:
                    error = None
                if error is not None:
                    _, bearing = error
                    cmd.angular.z = max(
                        -MAX_ANG, min(MAX_ANG, bias + HOME_GAIN * bearing)
                    )
                    if abs(bearing) > 0.8:
                        cmd.linear.x = 0.2
            elif front > stop_dist:
                cmd.linear.x = 0.18
                cmd.angular.z = MAX_ANG * (1.0 if left > right else -1.0)
            else:
                # Blocked: rotate in place toward the more open side. Commit to a
                # direction so we do not oscillate in a corner.
                now = time.monotonic()
                if self.stuck_since[rid] == 0.0:
                    self.stuck_since[rid] = now
                    self.turn_dir[rid] = 1.0 if left > right else -1.0

                # But only if there is room to sweep into. Rotating in place
                # carves out the circumscribed radius on BOTH sides, so a robot
                # boxed in laterally that turns anyway drives its own corner
                # into whatever stopped it — which is how two robots that had
                # each correctly stopped for the other ended up grinding
                # together. With no room, reverse out first and turn after.
                if max(left, right) < turn_dist:
                    cmd.linear.x = -0.15
                    cmd.angular.z = 0.0
                else:
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
    ap.add_argument(
        "--loop-period",
        type=float,
        default=90.0,
        help="seconds of exploring between legs back to the start "
        "pose, which is what guarantees a loop closure per "
        "cycle. 0 disables homing and wanders throughout.",
    )
    ap.add_argument(
        "--start-poses",
        default="",
        help="JSON {robot_id: {x, y, yaw}} of world-frame spawn "
        "poses. Enables scheduled pair rendezvous, which is "
        "what makes INTER-robot loop closure reliable rather "
        "than incidental. Without it only homing runs.",
    )
    ap.add_argument(
        "--radii",
        default="",
        help="JSON {robot_id: circumscribed_radius_m}. Every "
        "clearance this node uses is measured from the "
        "chassis, so a mixed fleet needs one per robot. "
        "Omitted robots fall back to the Duckiebot's 0.27 m.",
    )
    args = ap.parse_args()

    radii: dict[str, float] = {}
    if args.radii:
        try:
            radii = {rid: float(r) for rid, r in json.loads(args.radii).items()}
        except (ValueError, TypeError, AttributeError):
            radii = {}

    starts: dict[str, tuple[float, float, float]] = {}
    if args.start_poses:
        try:
            for rid, pose in json.loads(args.start_poses).items():
                starts[rid] = (
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("yaw", 0.0)),
                )
        except (ValueError, TypeError, AttributeError):
            starts = {}

    rclpy.init()
    ids = [f"{args.prefix}{i}" for i in range(args.robots)]
    starts = {rid: pose for rid, pose in starts.items() if rid in ids}
    node = Explorer(
        ids,
        seed=args.seed,
        loop_period=args.loop_period,
        starts=starts,
        radii={rid: r for rid, r in radii.items() if rid in ids},
    )
    node._set_rendezvous(node.cycle)
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
