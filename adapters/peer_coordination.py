"""Optional ROS peer exploration reservations over verified map authorities."""

from dataclasses import asdict
import json
import math
import time

import numpy as np

from autonomy.contracts import KeyframeId, validate_se3
from autonomy.coordination import (
    CompletionTracker,
    ExplorationReport,
    Intention,
    LeaseArbiter,
)
from adapters.mapping_authority import (
    accepts_authority_update,
    get_mapping_authority,
    transform_change_squared,
)


class PeerCoordinator:
    """Nonblocking reservation API used by MggExploration's command arbiter.

    reserve(plan, generation) -> granted/pending/rejected
    release(generation) and tick() are idempotent. Missing/stale authority rejects
    spatial assignment; independent MGG remains available with this feature off.
    """

    def __init__(self, bridge, config):
        from std_msgs.msg import String
        from geometry_msgs.msg import Pose, PoseArray

        self.bridge, self.msg_type = bridge, String
        self.mapping_authority = get_mapping_authority(bridge)
        self.pose_type, self.exclusions_type = Pose, PoseArray
        self.clock = time.monotonic
        self.authority, self.received_at = None, 0.0
        self.arbiter = None
        self.token, self.generation, self.invalid_token = None, -1, None
        self.last_publish = 0.0
        self.radius = float(config.get("reservation_radius_m", 2.0))
        self.run_id, self.completion = None, None
        self.exhaustion_signature = None
        self.report_sequence, self.reported_at = 0, 0.0
        self.report_heads, self.report_components = {}, {}
        self.report_publisher = bridge.node.create_publisher(
            String, "/swarmdeck/exploration_reports", 20
        )
        self.report_subscription = bridge.node.create_subscription(
            String, "/swarmdeck/exploration_reports", self.receive_report, 20
        )
        self.exclusions_publisher = bridge.node.create_publisher(
            PoseArray, f"/{bridge.id}/mgg/coordination_exclusions", 5
        )
        self.exclusions_at = 0.0
        self.publisher = bridge.node.create_publisher(
            String, "/swarmdeck/intentions", 20
        )
        self.subscription = bridge.node.create_subscription(
            String, "/swarmdeck/intentions", self.receive, 20
        )
        self.authority_subscription = bridge.node.create_subscription(
            String, f"/{bridge.id}/map_authority", self.on_authority, 5
        )

    def publish(self, intention):
        if intention is not None:
            self.publisher.publish(
                self.msg_type(data=json.dumps(asdict(intention), allow_nan=False))
            )
            self.last_publish = self.clock()

    def on_authority(self, msg):
        try:
            if len(msg.data) > 32_768:
                return
            value = json.loads(msg.data)
            if value["robot_id"] != self.bridge.id:
                return
            frame = str(value["navigation_frame"]).lstrip("/")
            if frame != self.bridge.map_frame.lstrip("/"):
                return
            if not accepts_authority_update(
                value, self.authority, self.mapping_authority.expected_mission
            ):
                return
            transform = validate_se3(value["T_component_navigation"])
            changed = (
                transform_change_squared(
                    transform, self.authority["T_component_navigation"]
                )
                if self.authority
                else 0.0
            )
            correction = value.get("correction_revision")
            if "solution_order" not in value:
                return
            signature = (
                value["mission_id"],
                value["component_id"],
                (
                    correction
                    if correction is not None
                    else tuple(value["solution_order"])
                ),
            )
            if self.authority:
                old = self.authority
                if signature != old["signature"] or changed > 0.01:
                    self.invalid_token = self.token
                    self.release(self.generation)
            if self.arbiter is None or self.arbiter.session_id != value["mission_id"]:
                self.arbiter = LeaseArbiter(
                    self.bridge.id,
                    value["mission_id"],
                    set(value["participants"]),
                    clock=self.clock,
                )
            self.arbiter.set_component(value["component_id"])
            self.authority = {
                **value,
                "signature": signature,
                "T_component_navigation": transform,
            }
            self.received_at = self.clock()
        except (ValueError, KeyError, TypeError):
            return

    def receive(self, msg):
        if self.arbiter is None or len(msg.data) > 4096:
            return
        try:
            self.arbiter.receive(Intention(**json.loads(msg.data)))
        except (ValueError, KeyError, TypeError):
            return

    def reserve(self, plan, generation):
        token = (generation, plan.revision_ns)
        if token == self.invalid_token or generation < self.generation:
            return "rejected"
        if self.authority is None or self.clock() - self.received_at > 3.0:
            self.release(generation)
            return "pending"
        if (
            plan.frame_id.lstrip("/") != self.authority["navigation_frame"].lstrip("/")
            or not plan.poses
        ):
            return "rejected"
        if token != self.token:
            self.release(generation)
            self.generation, self.token = generation, token
            final = plan.poses[-1]
            T = np.asarray(self.authority["T_component_navigation"])
            target = (T @ np.array([final.x, final.y, final.z, 1.0]))[:3]
            cost = sum(
                math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(plan.poses, plan.poses[1:])
            )
            self.publish(self.arbiter.propose(target, radius_m=self.radius, cost=cost))
        self.tick()
        return {
            "granted": "granted",
            "pending": "pending",
            "conflict": "rejected",
            "expired": "pending",
            "unassigned": "pending",
        }[self.arbiter.decision()]

    def publish_exclusions(self):
        message = self.exclusions_type()
        message.header.frame_id = self.bridge.map_frame
        if self.authority and self.arbiter and self.clock() - self.received_at <= 3.0:
            inverse = np.linalg.inv(self.authority["T_component_navigation"])
            local = self.arbiter.local
            for robot, (claim, expires) in self.arbiter.leases.items():
                if robot == self.bridge.id or expires <= self.clock():
                    continue
                if local and (claim.cost, robot) >= (local.cost, self.bridge.id):
                    continue
                xyz = inverse @ np.array([*claim.target, 1.0])
                pose = self.pose_type()
                pose.position.x, pose.position.y, pose.position.z = map(float, xyz[:3])
                pose.orientation.w = 1.0
                message.poses.append(pose)
        self.exclusions_publisher.publish(message)
        self.exclusions_at = self.clock()

    def tick(self):
        if self.clock() - self.reported_at >= 1.0:
            self.report_progress()
        if self.clock() - self.exclusions_at >= 0.5:
            self.publish_exclusions()
        if self.arbiter is None or self.arbiter.local is None:
            return
        if self.clock() - self.received_at > 3.0:
            self.release(self.generation)
            return
        if self.clock() - self.last_publish >= 1.0:
            claim = self.arbiter.local
            self.publish(
                self.arbiter.propose(
                    claim.target, radius_m=claim.radius_m, cost=claim.cost
                )
            )

    def release(self, generation):
        if generation < self.generation:
            return
        if self.arbiter:
            self.publish(self.arbiter.release())
        self.token = None
        self.generation = max(self.generation, generation)

    def begin_run(self, run_id, participants):
        """Bind completion to one operator command, not the estimator lifetime."""
        self.run_id, self.completion = None, None
        self.exhaustion_signature = None
        self.report_heads, self.report_components = {}, {}
        if run_id is None:  # Legacy command: no evidence for fleet completion.
            return
        KeyframeId(self.bridge.id, run_id, 0)
        if (
            not isinstance(participants, list)
            or not participants
            or len(participants) > 256
        ):
            raise ValueError("Exploration participants must be a bounded robot list")
        for robot in participants:
            KeyframeId(robot, run_id, 0)
        if self.bridge.id not in participants or len(set(participants)) != len(
            participants
        ):
            raise ValueError(
                "Exploration participants must include this robot exactly once"
            )
        self.run_id = run_id
        self.completion = CompletionTracker(
            set(participants), clock=lambda: self.clock()
        )
        self.report_sequence = 0

    def receive_report(self, message):
        if (
            self.completion is None
            or self.authority is None
            or len(message.data) > 8192
        ):
            return
        try:
            value = json.loads(message.data)
            if (
                value["run_id"] != self.run_id
                or value["mission_id"] != self.authority["mission_id"]
                or set(value["participants"]) != self.completion.participants
            ):
                return
            robot, sequence = value["robot_id"], value["sequence"]
            if (
                type(sequence) is not int
                or sequence < 0
                or sequence <= self.report_heads.get(robot, -1)
            ):
                return
            if not isinstance(value["component_id"], str) or not value["component_id"]:
                return
            if (
                type(value["coverage_met"]) is not bool
                or type(value["active_assignments"]) is not int
            ):
                return
            report = ExplorationReport(
                robot,
                value["state"],
                value["active_assignments"],
                value["coverage_met"],
            )
            if self.completion.receive(report):
                self.report_heads[robot] = sequence
                self.report_components[robot] = value["component_id"]
        except (ValueError, TypeError, KeyError):
            return

    def report_progress(self):
        self.reported_at = self.clock()
        if self.completion is None or self.authority is None:
            return
        explorer = getattr(self.bridge, "exploration", None)
        status = getattr(explorer, "status", "idle")
        fresh = self.clock() - self.received_at <= 3.0
        signature = (
            self.authority["mission_id"],
            self.authority["component_id"],
            self.authority.get("correction_revision", 0),
        )
        if status != "locally_exhausted":
            self.exhaustion_signature = None
        elif self.exhaustion_signature is None:
            self.exhaustion_signature = signature
        exhausted = (
            status == "locally_exhausted" and self.exhaustion_signature == signature
        )
        state = (
            "locally_exhausted"
            if exhausted and fresh
            else (
                "exploring"
                if getattr(explorer, "active", False) and fresh
                else "waiting_for_map" if not fresh else "blocked"
            )
        )
        assignments = int(
            bool(
                getattr(explorer, "pending_plan", None)
                or getattr(explorer, "executing_plan", None)
                or (self.arbiter and self.arbiter.local)
            )
        )
        self.report_sequence += 1
        message = self.msg_type(
            data=json.dumps(
                {
                    "run_id": self.run_id,
                    "mission_id": self.authority["mission_id"],
                    "participants": sorted(self.completion.participants),
                    "robot_id": self.bridge.id,
                    "component_id": self.authority["component_id"],
                    "sequence": self.report_sequence,
                    "state": state,
                    "active_assignments": assignments,
                    "coverage_met": state == "locally_exhausted",
                }
            )
        )
        self.receive_report(message)
        self.report_publisher.publish(message)

    @property
    def completion_state(self):
        if (
            self.completion is None
            or self.authority is None
            or self.clock() - self.received_at > 3.0
        ):
            return "unknown"
        state = self.completion.state()
        if state == "complete":
            signature = (
                self.authority["mission_id"],
                self.authority["component_id"],
                self.authority.get("correction_revision", 0),
            )
            if self.exhaustion_signature != signature or set(
                self.report_components.values()
            ) != {self.authority["component_id"]}:
                return "incomplete"
        return state
