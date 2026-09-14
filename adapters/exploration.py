"""MGG ROS 2 PCI control shared by simulation and hardware adapters.

MGG selects exploration goals; the robot's existing navigation stack executes
and collision-checks them. No planner output is accepted outside an explicit
operator exploration session.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import time


def is_physical_no_progress_failure(reason) -> bool:
    """Recognize the controller's explicit progress-checker terminal result."""

    return bool(
        isinstance(reason, str)
        and re.search(r"\bfailed to make progress\b", reason, re.IGNORECASE)
    )


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
        # Match native ground projection's numeric epsilon at the step cap.
        # Subtracting two voxel heights can make a 0.10 m step slightly larger
        # than 0.10 in floating point even though the native route admitted it.
        if rise > tolerance + 1e-6 and inclination > max_inclination:
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
        from adapters.mapping_authority import planning_frame

        self.bridge = bridge
        self.awaiting_terminal = False
        self.status = "idle"
        self.reason = None
        self.active = False
        self.generation = 0
        self.started_ns = 0
        self.last_path_revision_ns = 0
        self.pending_plan = None
        self.pending_authority_replan = None
        self.executing_plan = None
        self.executing_goal_generation = None
        self.completed_goal_generation = None
        self.replan_requested_generation = -1
        self.replan_requested_recovery = False
        self.controller_replan_generation = -1
        self.controller_goal_generation = None
        self.controller_replan_attempts = 0
        self.controller_replan_deadline = 0.0
        self.controller_replan_due = 0.0
        self.awaiting_replan_path = False
        self.pending = None
        self.pending_kind = None
        self.pending_recovery = False
        self.pending_stop = None
        self.deadline = 0.0
        self.stop_deadline = 0.0
        self.last_link = time.monotonic()
        self.request_type = Trigger.Request
        self.frame = planning_frame(bridge)
        configured_frame = str(config.get("frame") or "").lstrip("/")
        if configured_frame and configured_frame != self.frame:
            raise ValueError(
                "exploration.frame must match the configured MGG planning frame"
            )
        self.planar_tolerance_m = max(
            0.0, float(config.get("planar_tolerance_m", 0.10))
        )
        self.max_inclination_rad = max(
            0.0, float(config.get("max_inclination_rad", math.radians(30.0)))
        )
        self.controller_replan_max_attempts = self._bounded_int(
            # Two replacement paths allow three failed movement attempts total.
            config.get("controller_replan_max_attempts", 2),
            0,
            20,
            2,
        )
        self.controller_replan_deadline_s = self._bounded_float(
            config.get("controller_replan_deadline_s", 15.0), 0.1, 300.0, 15.0
        )
        self.controller_replan_backoff_s = self._bounded_float(
            config.get("controller_replan_backoff_s", 0.25), 0.0, 10.0, 0.25
        )
        self.coordinator = None
        if config.get("peer_coordination", False):
            from adapters.peer_coordination import PeerCoordinator

            self.coordinator = PeerCoordinator(bridge, config)
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
        self.reason = str(text).strip()[:512] or None
        self.bridge.node.get_logger().warning(f"[{self.bridge.id}] exploration: {text}")

    @staticmethod
    def _bounded_float(value, lower, upper, default):
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, parsed)) if math.isfinite(parsed) else default

    @staticmethod
    def _bounded_int(value, lower, upper, default):
        try:
            parsed = int(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, parsed))

    def start(self):
        if self.active:
            return
        self.reason = None
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
            self.status = "blocked"
            return
        self.bridge.cancel_goal()
        self.generation += 1
        generation = self.generation
        self.started_ns = self.bridge.node.get_clock().now().nanoseconds
        self.last_path_revision_ns = 0
        self.pending_plan = None
        self.pending_authority_replan = None
        self.executing_plan = None
        self.executing_goal_generation = None
        self.completed_goal_generation = None
        self.replan_requested_generation = -1
        self.replan_requested_recovery = False
        self._clear_controller_replan()
        self.awaiting_terminal = False
        self.status = "starting"
        self.active = True
        self.deadline = time.monotonic() + 30.0
        try:
            self.pending = self.start_client.call_async(self.request_type())
            self.pending_kind = "start"
            self.pending_recovery = False
        except Exception as exc:
            self.warn(f"start request failed: {exc}")
            self.stop()
            return
        self.pending.add_done_callback(
            lambda future: self.started(future, generation, "start", False)
        )

    def started(self, future, generation, request_kind="start", recovery=False):
        if self.pending is future:
            self.pending = None
            self.pending_kind = None
            self.pending_recovery = False
        if generation != self.generation:
            return
        if self._completed_goal_superseded():
            return
        try:
            if recovery and self.controller_replan_generation != generation:
                return
            if (
                recovery
                and self.controller_goal_generation != self.bridge._goal_generation
            ):
                self.warn("controller goal ownership changed before replan reply")
                self._stop_without_motion(status="stopped")
                return
            if request_kind == "replan":
                # Path delivery and its peer decision can precede the Trigger
                # reply. Preserve that newer work even if the reply fails.
                if self.executing_plan is not None or self.pending_plan is not None:
                    return
                if self.replan_requested_generation == generation:
                    requested_recovery = self.replan_requested_recovery
                    self.replan_requested_generation = -1
                    self.replan_requested_recovery = False
                    self._request_replan(generation, recovery=requested_recovery)
                    return
            response = future.result()
            if not response.success:
                self.warn(response.message or f"MGG rejected {request_kind}")
                self.stop(status="blocked" if request_kind == "replan" else "stopped")
                return
            if recovery and self.executing_plan is None:
                self.awaiting_replan_path = True
            if self.replan_requested_generation == generation:
                requested_recovery = self.replan_requested_recovery
                self.replan_requested_generation = -1
                self.replan_requested_recovery = False
                self._request_replan(generation, recovery=requested_recovery)
        except Exception as exc:
            self.warn(f"{request_kind} failed: {exc}")
            self.stop(status="blocked" if request_kind == "replan" else "stopped")

    def stop(self, *, notify_planner=True, status="stopped", cancel_navigation=True):
        self.status = status
        if status in ("complete", "locally_exhausted"):
            self.reason = None
        self.awaiting_terminal = False
        # Disable intake before touching ROS: a late service response or path
        # cannot re-arm exploration after Stop All or manual control.
        was_active = self.active
        self.active = False
        self.generation += 1
        self.pending_plan = None
        self.pending_authority_replan = None
        self.executing_plan = None
        self.executing_goal_generation = None
        self.completed_goal_generation = None
        self.replan_requested_generation = -1
        self.replan_requested_recovery = False
        self._clear_controller_replan()
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
            if cancel_navigation:
                self.bridge.cancel_goal()
                self.bridge.drive(0.0, 0.0)

    def tick(self):
        # A planner restart can orphan a DDS request forever. Retire expired
        # futures even after Stop, so a later operator start can recover.
        now = time.monotonic()
        self._completed_goal_superseded()
        self._check_controller_result(now)
        self._drive_controller_replan(now)
        if self.coordinator is not None:
            self.coordinator.tick()
            candidate = self.executing_plan or self.pending_plan
            if self.active and candidate is not None:
                plan, generation = candidate
                decision = self.coordinator.reserve(plan, generation)
                if decision == "granted" and self.pending_plan == candidate:
                    self.pending_plan = None
                    if self.pending_authority_replan is candidate:
                        # The robot may have advanced before this executing
                        # path lost authority. Use the retained plan only to
                        # finish the reservation decision, then replan from
                        # the current pose so Nav2 never receives a stale,
                        # fully-pruned path.
                        self.pending_authority_replan = None
                        self.coordinator.release(generation)
                        self._request_replan(
                            generation,
                            recovery=(
                                self.controller_replan_generation == generation
                            ),
                        )
                    else:
                        self._execute(plan, generation)
                elif decision == "rejected" and self.pending_plan == candidate:
                    self.pending_plan = None
                    self.pending_authority_replan = None
                    self.status = "waiting"
                    self.reason = getattr(self.coordinator, "last_decision_reason", "Waiting for another exploration route")
                    self.coordinator.release(generation)
                    self._request_replan(
                        generation,
                        recovery=self.controller_replan_generation == generation,
                    )
                elif decision != "granted" and self.executing_plan == candidate:
                    reason = getattr(
                        self.coordinator, "last_decision_reason", decision
                    )
                    self.warn(f"peer reservation lost ({reason}); stopping path")
                    owner = self.executing_goal_generation
                    cancel_if_current = getattr(
                        self.bridge, "cancel_goal_if_current", None
                    )
                    cancelled_generation = (
                        cancel_if_current(owner)
                        if callable(cancel_if_current) and type(owner) is int
                        else None
                    )
                    if cancelled_generation is None:
                        self.warn(
                            "controller goal ownership changed during peer "
                            "authority loss"
                        )
                        self._stop_without_motion(status="stopped")
                        return
                    # A non-recovery path has no controller recovery owner.
                    # Retain the generation created by this cancellation so a
                    # later reservation grant cannot overwrite an operator
                    # command issued while peer authority was pending.
                    self.completed_goal_generation = cancelled_generation
                    self.executing_plan = None
                    self.executing_goal_generation = None
                    self.status = "waiting"
                    recovering = self.controller_replan_generation == generation
                    if recovering:
                        # This replacement already survived planning and began
                        # physical execution. Losing its peer authority starts
                        # a distinct bounded wait; an expired planner deadline
                        # from the preceding controller failure must not retire
                        # it on the next tick. Cancellation changes controller
                        # ownership, while the physical-attempt count remains.
                        self.controller_goal_generation = cancelled_generation
                        self.controller_replan_deadline = (
                            now + self.controller_replan_deadline_s
                        )
                        self.controller_replan_due = math.inf
                        self.awaiting_replan_path = True
                    if decision == "pending":
                        self.pending_plan = candidate
                        self.pending_authority_replan = candidate
                    else:
                        self.coordinator.release(generation)
                        self._request_replan(generation, recovery=recovering)
        if self.pending is not None and now > self.deadline:
            future, self.pending = self.pending, None
            request_kind, self.pending_kind = self.pending_kind or "start", None
            recovery, self.pending_recovery = self.pending_recovery, False
            if self.active:
                self.warn(f"MGG {request_kind} timed out")
                timeout_status = (
                    "blocked" if request_kind == "replan" or recovery else "stopped"
                )
                if (
                    (request_kind == "replan" or recovery)
                    and self.executing_plan is None
                ):
                    self._stop_without_motion(status=timeout_status)
                else:
                    self.stop(status=timeout_status)
            client = (
                self.replan_client if request_kind == "replan" else self.start_client
            )
            client.remove_pending_request(future)
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

    def _check_controller_result(self, now):
        if not self.active or self.executing_plan is None:
            return
        session_generation = self.executing_plan[1]
        goal_generation = self.executing_goal_generation
        if session_generation != self.generation:
            return
        if (
            type(goal_generation) is not int
            or goal_generation != self.bridge._goal_generation
        ):
            self.warn("controller goal ownership changed; stopping MGG session")
            self._stop_without_motion(status="stopped")
            return
        nav_status = self.bridge.nav_status
        if nav_status == "succeeded":
            self.executing_plan = None
            self.executing_goal_generation = None
            self.completed_goal_generation = goal_generation
            self._clear_controller_replan()
            if self.coordinator is not None:
                self.coordinator.release(self.generation)
            # PCI's external-execution mode leaves arrival to FollowPath. A
            # waypoint proximity check must never replace an unfinished path.
            self.status = "waiting"
            self._request_replan(self.generation)
            return
        if nav_status != "failed":
            return
        failure_reason = getattr(self.bridge, "_nav_failure_reason", None)
        if not is_physical_no_progress_failure(failure_reason):
            detail = (
                str(failure_reason).strip()
                if isinstance(failure_reason, str) and failure_reason.strip()
                else "reason unavailable"
            )
            self.warn(f"controller failed without no-progress evidence: {detail}")
            # The controller is already terminal. Retain its exact failure
            # reason in robot telemetry while stopping the planner session.
            self._stop_without_motion(status="blocked")
            return
        self.executing_plan = None
        self.executing_goal_generation = None
        if self.coordinator is not None:
            self.coordinator.release(self.generation)
        if self.controller_replan_generation != self.generation:
            self.controller_replan_generation = self.generation
            self.controller_goal_generation = goal_generation
            self.controller_replan_attempts = 0
        # This deadline bounds production of the next replacement path. Time
        # spent physically attempting the current path does not consume the
        # next recovery window.
        self.controller_replan_deadline = now + self.controller_replan_deadline_s
        if self.controller_replan_attempts >= self.controller_replan_max_attempts:
            self.warn(
                f"controller recovery exhausted after "
                f"{self.controller_replan_attempts + 1} failed movement attempts; "
                "exploration is blocked"
            )
            self._stop_without_motion(status="blocked")
            return
        self.status = "waiting"
        self.awaiting_replan_path = False
        self.controller_replan_due = now + self.controller_replan_backoff_s

    def _drive_controller_replan(self, now):
        if (
            not self.active
            or self.controller_replan_generation != self.generation
            or self.executing_plan is not None
        ):
            return
        if now >= self.controller_replan_deadline:
            self.warn("controller recovery timed out waiting for a replacement path")
            # The failed/cancelled controller path no longer owns motion here.
            # Retire recovery state without risking cancellation of a newer
            # operator goal that raced this watchdog tick.
            self._stop_without_motion(status="blocked")
            return
        if self.controller_goal_generation != self.bridge._goal_generation:
            self.warn("controller goal ownership changed during recovery")
            self._stop_without_motion(status="stopped")
            return
        if self.awaiting_replan_path or now < self.controller_replan_due:
            return
        self.controller_replan_attempts += 1
        self.controller_replan_due = math.inf
        self._request_replan(self.generation, recovery=True)

    def _clear_controller_replan(self):
        self.controller_replan_generation = -1
        self.controller_goal_generation = None
        self.controller_replan_attempts = 0
        self.controller_replan_deadline = 0.0
        self.controller_replan_due = 0.0
        self.awaiting_replan_path = False

    def _stop_without_motion(self, *, status):
        """Retire MGG after another command has taken bridge goal ownership."""
        self.stop(status=status, cancel_navigation=False)

    def _completed_goal_superseded(self):
        """A manual command also wins while the next plan is being computed."""
        expected = self.completed_goal_generation
        if expected is None or expected == self.bridge._goal_generation:
            return False
        self._stop_without_motion(status="stopped")
        return True

    def on_status(self, message):
        if self._completed_goal_superseded():
            return
        try:
            report = json.loads(message.data)
            stamp = int(report["stamp_ns"])
            state = report["state"]
        except (ValueError, TypeError, KeyError):
            return
        if stamp < self.started_ns or not self.started_ns:
            return
        if state in ("complete", "blocked"):
            if self.executing_plan is not None:
                self.warn(
                    f"ignoring terminal planner status {state!r} while "
                    "controller goal is active"
                )
                return
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
            state in ("starting", "exploring", "waiting")
            and self.active
            and self.pending_plan is None
            and self.executing_plan is None
        ):
            self.status = state
            detail = report.get("reason")
            self.reason = (
                detail.strip()[:512]
                if isinstance(detail, str) and detail.strip()
                else "MGG is searching for a traversable exploration route"
                if state == "waiting"
                else None
            )

    def on_path(self, path):
        if not self.active:
            return
        if self._completed_goal_superseded():
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
        if self.executing_plan is not None:
            # PCI external-execution mode must wait for an explicit replan
            # request after FollowPath reaches a terminal result. Treat any
            # newer topic value received while the controller still owns the
            # current path as unsolicited: submitting it would preempt the
            # active FollowPath goal before the robot reached its endpoint.
            # Consume its revision as well, so a duplicate cannot become
            # executable after the current controller goal finishes.
            self.last_path_revision_ns = stamp
            self.warn("ignoring replacement path while controller goal is active")
            return
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
        # A newer planner route supersedes any cancelled path retained only as
        # a peer-reservation token. Its same-session generation must not make
        # the fresh route inherit the old route's mandatory-replan marker.
        self.pending_plan = None
        self.pending_authority_replan = None
        if (
            self.controller_replan_generation == generation
            and self.controller_goal_generation != self.bridge._goal_generation
        ):
            self.warn("controller goal ownership changed before replacement path")
            self._stop_without_motion(status="stopped")
            return
        self.awaiting_replan_path = False
        self.controller_replan_due = math.inf
        if not self.active:
            return
        if self.coordinator is not None:
            if self.executing_plan is not None:
                self.bridge.cancel_goal()
                self.executing_plan = None
                self.executing_goal_generation = None
            decision = self.coordinator.reserve(plan, generation)
            if decision == "pending":
                self.pending_plan = (plan, generation)
                self.status = "waiting"
                self.reason = getattr(self.coordinator, "last_decision_reason", "Waiting for peer coordination")
                return
            if decision == "rejected":
                # Any previous executing path was cancelled above. Cancelling
                # again also changes ownership of an already-completed goal.
                self.status = "waiting"
                self.coordinator.release(generation)
                self._request_replan(generation)
                return
        self._execute(plan, generation)

    def _execute(self, plan, generation):
        if not self.active or generation != self.generation:
            return
        self.pending_authority_replan = None
        try:
            self.status = "exploring"
            self.reason = None
            recovering = self.controller_replan_generation == generation
            expected = (
                self.controller_goal_generation
                if recovering
                else self.completed_goal_generation
            )
            accepted = (
                self.bridge.follow_path(plan, expected_generation=expected)
                if expected is not None
                else self.bridge.follow_path(plan)
            )
            if type(accepted) is not int:
                self.warn("full-path controller unavailable")
                if expected is not None and accepted is None:
                    self._stop_without_motion(status="stopped")
                else:
                    self.stop(status="blocked" if recovering else "stopped")
                return
        except Exception as exc:
            self.warn(f"path execution failed: {exc}")
            self.stop()
        # Some hardware service calls block briefly. Stop still wins if it
        # arrives while the navigation request is being prepared.
        if not self.active or generation != self.generation:
            self.bridge.cancel_goal()
            self.bridge.drive(0.0, 0.0)
            return
        if accepted != self.bridge._goal_generation:
            self.warn("controller goal ownership changed during path submission")
            self._stop_without_motion(status="stopped")
            return
        self.executing_plan = (plan, generation)
        self.executing_goal_generation = accepted
        self.completed_goal_generation = None
        self.awaiting_replan_path = False

    def _request_replan(self, generation, *, recovery=False):
        if not self.active or generation != self.generation:
            return
        if recovery and self.controller_goal_generation != self.bridge._goal_generation:
            self.warn("controller goal ownership changed before replan")
            self._stop_without_motion(status="stopped")
            return
        if self.pending is not None:
            self.replan_requested_generation = generation
            self.replan_requested_recovery = self.replan_requested_recovery or recovery
            return
        self.replan_requested_generation = -1
        if not self.replan_client.service_is_ready():
            self.warn("MGG replan service is unavailable")
            self.stop(status="blocked")
            return
        try:
            self.pending = self.replan_client.call_async(self.request_type())
            self.pending_kind = "replan"
            self.pending_recovery = recovery
            request_deadline = time.monotonic() + 30.0
            self.deadline = (
                min(request_deadline, self.controller_replan_deadline)
                if recovery
                else request_deadline
            )
        except Exception as exc:
            self.warn(f"replan request failed: {exc}")
            self.stop(status="blocked")
            return
        self.pending.add_done_callback(
            lambda future: self.started(future, generation, "replan", recovery)
        )


def configure_exploration(bridge):
    config = (bridge.cfg or {}).get("exploration") or {}
    enabled = bool(config.get("enabled", False))
    if enabled and getattr(bridge, "path_client", None) is None:
        bridge.node.get_logger().warning(
            f"[{bridge.id}] exploration disabled: no full-path controller configured"
        )
        enabled = False
    bridge.exploration = MggExploration(bridge, config) if enabled else None
