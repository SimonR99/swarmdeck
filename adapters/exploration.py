"""MGG ROS 2 PCI control shared by simulation and hardware adapters.

MGG selects exploration goals; the robot's existing navigation stack executes
and collision-checks them. No planner output is accepted outside an explicit
operator exploration session.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time


@dataclass(frozen=True)
class PlannerPose:
    """One MGG pose, retained in full for the controller boundary."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class PlannerPath:
    """A validated path whose source timestamp is its execution revision."""

    frame_id: str
    revision_ns: int
    poses: tuple[PlannerPose, ...]


def planner_path(
    path,
    expected_frame: str,
    planar_tolerance_m: float,
    max_inclination_rad: float = math.radians(30.0),
) -> PlannerPath:
    """Validate a ROS-like MGG path without importing ROS.

    Nav2's FollowPath controller is a planar backend. XYZ stays intact at this
    boundary, but the controller does not itself establish 3D feasibility.
    Accept only yaw orientations and terrain segments within the configured
    ground robot step or grade capability.
    """
    frame = str(path.header.frame_id).lstrip("/")
    if frame != str(expected_frame).lstrip("/"):
        raise ValueError(
            f"path frame {path.header.frame_id!r} differs from {expected_frame!r}"
        )
    stamp = path.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("path has an invalid timestamp")
    revision_ns = sec * 1_000_000_000 + nanosec
    tolerance = max(0.0, float(planar_tolerance_m))
    max_inclination = max(0.0, float(max_inclination_rad))
    poses: list[PlannerPose] = []
    for index, stamped in enumerate(path.poses):
        pose_frame = str(getattr(getattr(stamped, "header", None), "frame_id", ""))
        if pose_frame and pose_frame.lstrip("/") != frame:
            raise ValueError(
                f"path pose {index} frame {pose_frame!r} differs from {frame!r}"
            )
        pose = stamped.pose
        p, q = pose.position, pose.orientation
        values = (
            float(p.x),
            float(p.y),
            float(getattr(p, "z", 0.0)),
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"path pose {index} contains a nonfinite value")
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm < 1e-9:
            raise ValueError(f"path pose {index} has an invalid orientation")
        qx, qy, qz, qw = (value / norm for value in values[3:])
        # A planar quaternion can have yaw, but no roll or pitch. These two
        # rotation-matrix terms are zero for every yaw-only orientation.
        tilt_x = 2.0 * (qx * qz - qw * qy)
        tilt_y = 2.0 * (qy * qz + qw * qx)
        z_alignment = 1.0 - 2.0 * (qx * qx + qy * qy)
        if math.atan2(math.hypot(tilt_x, tilt_y), z_alignment) > 1e-4:
            raise ValueError(f"path pose {index} is not planar")
        poses.append(PlannerPose(*values[:3], qx, qy, qz, qw))
    if not poses:
        raise ValueError("path is empty")
    for previous, current in zip(poses, poses[1:]):
        rise = abs(current.z - previous.z)
        run = math.hypot(current.x - previous.x, current.y - previous.y)
        inclination = math.atan2(rise, run)
        # Ground control supports bounded steps and continuous ramps. Total
        # elevation is irrelevant; a long gentle ramp is still planar motion.
        if rise > tolerance and inclination > max_inclination:
            raise ValueError(
                f"path segment rises {rise:.3f} m at {inclination:.3f} rad; "
                "the planar ground controller cannot traverse it"
            )
    return PlannerPath(frame, revision_ns, tuple(poses))


def follow_path_goal(plan: PlannerPath):
    """Convert the transport-free path to a Nav2 FollowPath goal."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from nav2_msgs.action import FollowPath

    if not plan.poses:
        raise ValueError("path is empty")
    goal = FollowPath.Goal()
    goal.path = Path()
    goal.path.poses = []
    goal.path.header.frame_id = plan.frame_id
    goal.path.header.stamp.sec = plan.revision_ns // 1_000_000_000
    goal.path.header.stamp.nanosec = plan.revision_ns % 1_000_000_000
    for source in plan.poses:
        pose = PoseStamped()
        pose.header = goal.path.header
        pose.pose.position.x = source.x
        pose.pose.position.y = source.y
        pose.pose.position.z = source.z
        pose.pose.orientation.x = source.qx
        pose.pose.orientation.y = source.qy
        pose.pose.orientation.z = source.qz
        pose.pose.orientation.w = source.qw
        goal.path.poses.append(pose)
    return goal


class MggExploration:
    def __init__(self, bridge, config):
        from std_srvs.srv import Trigger
        from nav_msgs.msg import Path
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy

        self.bridge = bridge
        self.awaiting_terminal = False
        self.status = "idle"
        self.active = False
        self.generation = 0
        self.started_ns = 0
        self.last_path_revision_ns = 0
        self.pending_plan = None
        self.executing_plan = None
        self.replan_requested_generation = -1
        self.pending = None
        self.pending_stop = None
        self.deadline = 0.0
        self.stop_deadline = 0.0
        self.last_link = time.monotonic()
        self.request_type = Trigger.Request
        self.frame = str(
            config.get("frame")
            or getattr(bridge, "map_frame", f"{bridge.id}/map_frame")
        ).lstrip("/")
        self.planar_tolerance_m = max(
            0.0, float(config.get("planar_tolerance_m", 0.10))
        )
        self.max_inclination_rad = max(
            0.0, float(config.get("max_inclination_rad", math.radians(30.0)))
        )
        self.coordinator = None
        if config.get("peer_coordination", False):
            from adapters.peer_coordination import PeerCoordinator

            self.coordinator = PeerCoordinator(bridge, config)
        navigation_frame = str(
            getattr(bridge, "map_frame", f"{bridge.id}/map_frame")
        ).lstrip("/")
        if self.frame != navigation_frame:
            raise ValueError(
                "exploration.frame must match the adapter navigation map frame"
            )
        namespace = str(config.get("namespace") or f"/{bridge.id}/mgg").rstrip("/")
        self.start_client = bridge.node.create_client(
            Trigger, f"{namespace}/pci_trigger"
        )
        self.replan_client = bridge.node.create_client(
            Trigger, f"{namespace}/pci_replan"
        )
        self.stop_client = bridge.node.create_client(Trigger, f"{namespace}/pci_stop")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = bridge.node.create_subscription(
            Path, f"{namespace}/command_path", self.on_path, qos
        )
        self.status_subscription = bridge.node.create_subscription(
            String, f"{namespace}/status", self.on_status, qos
        )
        self.timer = bridge.node.create_timer(0.2, self.tick)

    def warn(self, text):
        self.bridge.node.get_logger().warning(f"[{self.bridge.id}] exploration: {text}")

    def start(self):
        if self.active:
            return
        if any(
            future is not None and not future.done()
            for future in (self.pending, self.pending_stop)
        ):
            self.warn("previous MGG request is still finishing; retry Explore shortly")
            return
        if (
            not self.start_client.service_is_ready()
            or not self.stop_client.service_is_ready()
        ):
            self.warn("MGG start/stop services are unavailable")
            return
        self.bridge.cancel_goal()
        self.generation += 1
        generation = self.generation
        self.started_ns = self.bridge.node.get_clock().now().nanoseconds
        self.last_path_revision_ns = 0
        self.pending_plan = None
        self.executing_plan = None
        self.replan_requested_generation = -1
        self.awaiting_terminal = False
        self.status = "starting"
        self.active = True
        self.deadline = time.monotonic() + 30.0
        try:
            self.pending = self.start_client.call_async(self.request_type())
        except Exception as exc:
            self.warn(f"start request failed: {exc}")
            self.stop()
            return
        self.pending.add_done_callback(lambda future: self.started(future, generation))

    def started(self, future, generation):
        if self.pending is future:
            self.pending = None
        if generation != self.generation:
            return
        try:
            response = future.result()
            if not response.success:
                self.warn(response.message or "MGG rejected start")
                self.stop()
                return
            if self.replan_requested_generation == generation:
                self.replan_requested_generation = -1
                self._request_replan(generation)
        except Exception as exc:
            self.warn(f"start failed: {exc}")
            self.stop()

    def stop(self, *, notify_planner=True, status="stopped"):
        self.status = status
        self.awaiting_terminal = False
        # Disable intake before touching ROS: a late service response or path
        # cannot re-arm exploration after Stop All or manual control.
        was_active = self.active
        self.active = False
        self.generation += 1
        self.pending_plan = None
        self.executing_plan = None
        self.replan_requested_generation = -1
        if self.coordinator is not None:
            self.coordinator.release(self.generation)
        if was_active:
            try:
                if not notify_planner:
                    pass
                elif self.stop_client.service_is_ready():
                    self.pending_stop = self.stop_client.call_async(self.request_type())
                    self.stop_deadline = time.monotonic() + 30.0
                else:
                    self.warn(
                        "MGG stop service unavailable; navigation stopped locally"
                    )
            except Exception as exc:
                self.warn(f"stop request failed; navigation stopped locally: {exc}")
            self.bridge.cancel_goal()
            self.bridge.drive(0.0, 0.0)

    def tick(self):
        # A planner restart can orphan a DDS request forever. Retire expired
        # futures even after Stop, so a later operator start can recover.
        now = time.monotonic()
        if self.coordinator is not None:
            self.coordinator.tick()
            candidate = self.executing_plan or self.pending_plan
            if self.active and candidate is not None:
                plan, generation = candidate
                decision = self.coordinator.reserve(plan, generation)
                if decision == "granted" and self.pending_plan == candidate:
                    self.pending_plan = None
                    self._execute(plan, generation)
                elif decision == "rejected" and self.pending_plan == candidate:
                    self.pending_plan = None
                    self.status = "waiting"
                    self.coordinator.release(generation)
                    self._request_replan(generation)
                elif decision != "granted" and self.executing_plan == candidate:
                    self.warn("peer reservation lost; stopping path")
                    self.bridge.cancel_goal()
                    self.executing_plan = None
                    self.status = "waiting"
                    if decision == "pending":
                        self.pending_plan = candidate
                    else:
                        self.coordinator.release(generation)
                        self._request_replan(generation)
        if self.pending is not None and now > self.deadline:
            future, self.pending = self.pending, None
            if self.active:
                self.warn("MGG start timed out")
                self.stop()
            self.start_client.remove_pending_request(future)
            future.cancel()
        if self.pending_stop is not None and now > self.stop_deadline:
            future, self.pending_stop = self.pending_stop, None
            self.stop_client.remove_pending_request(future)
            future.cancel()
        if self.active and time.monotonic() - self.last_link > float(
            self.bridge.cfg.get("link_timeout_s", 5.0)
        ):
            self.warn("operator link lost")
            self.stop()

    def on_status(self, message):
        try:
            report = json.loads(message.data)
            stamp = int(report["stamp_ns"])
            state = report["state"]
        except (ValueError, TypeError, KeyError):
            return
        if stamp < self.started_ns or not self.started_ns:
            return
        if state in ("complete", "blocked"):
            # Empty path and status are separate DDS topics and may arrive in
            # either order. A manual stop increments generation and must win.
            if self.active or getattr(self, "awaiting_terminal", False):
                terminal = (
                    "locally_exhausted"
                    if state == "complete" and self.coordinator is not None
                    else state
                )
                self.stop(notify_planner=False, status=terminal)
                self.awaiting_terminal = False
        elif (
            state in ("starting", "exploring")
            and self.active
            and self.pending_plan is None
        ):
            self.status = state

    def on_path(self, path):
        if not self.active:
            return
        sec = int(path.header.stamp.sec)
        nanosec = int(path.header.stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            self.warn("unsupported planner path: invalid timestamp")
            self.stop()
            return
        stamp = sec * 1_000_000_000 + nanosec
        if stamp < self.started_ns:
            return  # Discard the latched path from a previous session.
        if stamp <= self.last_path_revision_ns:
            return  # Duplicate or reordered path within this operator session.
        if not path.poses:
            # Stop immediately without sending pci_stop back: the accompanying
            # status topic distinguishes exhaustion from blocked planning.
            self.stop(notify_planner=False, status="stopped")
            self.awaiting_terminal = True
            return
        try:
            plan = planner_path(
                path,
                self.frame,
                self.planar_tolerance_m,
                self.max_inclination_rad,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            self.warn(f"unsupported planner path: {exc}")
            self.stop()
            return
        self.last_path_revision_ns = plan.revision_ns
        generation = self.generation
        if not self.active:
            return
        if self.coordinator is not None:
            if self.executing_plan is not None:
                self.bridge.cancel_goal()
                self.executing_plan = None
            decision = self.coordinator.reserve(plan, generation)
            if decision == "pending":
                self.pending_plan = (plan, generation)
                self.status = "waiting"
                return
            if decision == "rejected":
                self.bridge.cancel_goal()
                self.status = "waiting"
                self.coordinator.release(generation)
                self._request_replan(generation)
                return
        self._execute(plan, generation)

    def _execute(self, plan, generation):
        if not self.active or generation != self.generation:
            return
        try:
            self.status = "exploring"
            accepted = self.bridge.follow_path(plan)
            if accepted is False:
                self.warn("full-path controller unavailable")
                self.stop()
        except Exception as exc:
            self.warn(f"path execution failed: {exc}")
            self.stop()
        # Some hardware service calls block briefly. Stop still wins if it
        # arrives while the navigation request is being prepared.
        if not self.active or generation != self.generation:
            self.bridge.cancel_goal()
            self.bridge.drive(0.0, 0.0)
            return
        self.executing_plan = (plan, generation)

    def _request_replan(self, generation):
        if not self.active or generation != self.generation:
            return
        if self.pending is not None and not self.pending.done():
            self.replan_requested_generation = generation
            return
        self.replan_requested_generation = -1
        if not self.replan_client.service_is_ready():
            self.warn("MGG replan service is unavailable")
            self.stop(status="blocked")
            return
        try:
            self.pending = self.replan_client.call_async(self.request_type())
            self.deadline = time.monotonic() + 30.0
        except Exception as exc:
            self.warn(f"replan request failed: {exc}")
            self.stop(status="blocked")
            return
        self.pending.add_done_callback(lambda future: self.started(future, generation))


def configure_exploration(bridge):
    config = (bridge.cfg or {}).get("exploration") or {}
    enabled = bool(config.get("enabled", False))
    if enabled and getattr(bridge, "path_client", None) is None:
        bridge.node.get_logger().warning(
            f"[{bridge.id}] exploration disabled: no full-path controller configured"
        )
        enabled = False
    bridge.exploration = MggExploration(bridge, config) if enabled else None
