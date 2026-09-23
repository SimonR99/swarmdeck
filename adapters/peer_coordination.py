"""Optional ROS peer exploration reservations over verified map authorities."""

from dataclasses import asdict
import json
import math
import time

import numpy as np

from autonomy.contracts import KeyframeId
from autonomy.coordination import (
    CompletionTracker,
    ExplorationReport,
    Intention,
    LeaseArbiter,
)
from adapters.mapping_authority import (
    accepts_authority_update,
    authority_for_frame,
    get_mapping_authority,
    planning_frame,
    shared_subscription,
    transform_change_squared,
)


def deployment_transform(pose):
    """Deployment-from-map transform of a surveyed start pose, or None."""
    if not isinstance(pose, dict):
        return None
    try:
        x, y = float(pose["x"]), float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[c, -s, 0.0, x], [s, c, 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
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
        self.frame = planning_frame(bridge)
        configured_frame = str(config.get("frame") or "").lstrip("/")
        if configured_frame and configured_frame != self.frame:
            raise ValueError(
                "peer coordination frame must match the configured MGG planning frame"
            )
        self.pose_type, self.exclusions_type = Pose, PoseArray
        self.clock = time.monotonic
        self.authority, self.raw_authority, self.received_at = None, None, 0.0
        self.arbiter = None
        self.token, self.generation, self.invalid_token = None, -1, None
        self.invalid_token_reason = None
        self.last_decision_reason = "unassigned"
        self.decisions = {"granted": 0, "conflict": 0, "pending": 0}
        self.reservation_transform = None
        self.reservation_signature = None
        self.last_publish = 0.0
        self.radius = float(config.get("reservation_radius_m", 2.0))
        # Leases are arbitrated inside one verified map component, so a fleet
        # whose robots each hold their own component (inter-robot closures off)
        # assigns no frontiers at all. A deployment that surveyed where every
        # robot starts can arbitrate in that shared frame instead. It is used
        # for reservations only, whose radius absorbs metres of drift; no map
        # geometry ever passes through it.
        self.deployment_from_map = deployment_transform(
            config.get("deployment_start_pose")
        )
        self.run_id, self.completion = None, None
        self.exhaustion_signature = None
        self.report_sequence, self.reported_at = 0, 0.0
        self.report_heads, self.report_components = {}, {}
        self.report_publisher = bridge.node.create_publisher(
            String, "/swarmdeck/exploration_reports", 20
        )
        self.report_subscription = shared_subscription(
            bridge.node,
            String,
            "/swarmdeck/exploration_reports",
            self.receive_report,
            20,
        )
        self.exclusions_publisher = bridge.node.create_publisher(
            PoseArray, f"/{bridge.id}/mgg/coordination_exclusions", 5
        )
        self.exclusions_at = 0.0
        self.publisher = bridge.node.create_publisher(
            String, "/swarmdeck/intentions", 20
        )
        self.subscription = shared_subscription(
            bridge.node, String, "/swarmdeck/intentions", self.receive, 20
        )
        self.authority_subscription = shared_subscription(
            bridge.node, String, f"/{bridge.id}/map_authority", self.on_authority, 5
        )
        # Where the other robots stand. They are masked out of every map, so
        # the planner learns about them here: each robot reports its position
        # in the deployment frame and hands the others' to its own MGG, which
        # keeps a disc around each clear of routes for as long as reports come.
        self.peer_positions = {}
        self.peer_bodies_at = 0.0
        self.peer_pose_publisher = self.peer_pose_subscription = None
        self.peer_bodies_publisher = None
        if self.deployment_from_map is not None:
            self.peer_pose_publisher = bridge.node.create_publisher(
                String, "/swarmdeck/peer_poses", 20
            )
            self.peer_pose_subscription = shared_subscription(
                bridge.node, String, "/swarmdeck/peer_poses", self.receive_peer_pose, 20
            )
            self.peer_bodies_publisher = bridge.node.create_publisher(
                PoseArray, f"/{bridge.id}/mgg/peer_bodies", 5
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
            if frame != self.bridge.navigation_frame.lstrip("/"):
                return
            if not accepts_authority_update(
                value, self.raw_authority, self.mapping_authority.expected_mission
            ):
                return
            selected = authority_for_frame(value, self.frame)
            transform = selected["T_component_navigation"]
            if "solution_order" not in value:
                return
            component = value["component_id"]
            # What revokes a granted reservation is a correction of the robot's
            # own map, judged on the component transform. The deployment
            # transform also carries map-from-odometry drift, which moves a
            # target by centimetres and is no reason to stop a path (Spot lost
            # its reservation every 30 s to it, 2026-09-17).
            guard = transform
            if self.deployment_from_map is not None:
                map_from_planning = np.linalg.solve(
                    np.asarray(value["T_component_navigation"], dtype=float),
                    np.asarray(transform, dtype=float),
                )
                transform = (self.deployment_from_map @ map_from_planning).tolist()
                component = f"deployment:{value['mission_id']}"
            signature = (value["mission_id"], component)
            if self.authority:
                reservation_changed = (
                    self.token is not None
                    and self.reservation_transform is not None
                    and transform_change_squared(guard, self.reservation_transform)
                    > 0.01
                )
                component_changed = (
                    self.token is not None
                    and self.reservation_signature is not None
                    and signature != self.reservation_signature
                )
                if component_changed or reservation_changed:
                    self.invalid_token = self.token
                    self.invalid_token_reason = (
                        "map component changed"
                        if component_changed
                        else "material map correction"
                    )
                    self.release(self.generation)
            if self.arbiter is None or self.arbiter.session_id != value["mission_id"]:
                self.arbiter = LeaseArbiter(
                    self.bridge.id,
                    value["mission_id"],
                    set(value["participants"]),
                    clock=self.clock,
                )
            self.arbiter.set_component(component)
            self.raw_authority = value
            self.authority = {
                **selected,
                "signature": signature,
                "T_component_navigation": transform,
                "reservation_guard": guard,
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
        if token == self.invalid_token:
            self.last_decision_reason = f"reservation invalidated by {self.invalid_token_reason or 'map correction'}"
            return "rejected"
        if generation < self.generation:
            self.last_decision_reason = "stale exploration generation"
            return "rejected"
        if self.authority is None or self.clock() - self.received_at > 3.0:
            self.last_decision_reason = "map authority unavailable or stale"
            preserve = generation == self.generation and token == self.token
            self._release_lease(generation, preserve_binding=preserve)
            return "pending"
        if (
            plan.frame_id.lstrip("/") != self.authority["navigation_frame"].lstrip("/")
            or not plan.poses
        ):
            self.last_decision_reason = "path does not match verified map authority"
            return "rejected"
        if token != self.token or self.arbiter.local is None:
            self.release(generation)
            self.generation, self.token = generation, token
            final = plan.poses[-1]
            T = np.asarray(self.authority["T_component_navigation"])
            self.reservation_transform = self.authority["reservation_guard"]
            self.reservation_signature = self.authority["signature"]
            target = (T @ np.array([final.x, final.y, final.z, 1.0]))[:3]
            cost = sum(
                math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(plan.poses, plan.poses[1:])
            )
            self.publish(self.arbiter.propose(target, radius_m=self.radius, cost=cost))
        self.tick()
        decision, winner = self.arbiter.decision_with_winner()
        result = {
            "granted": "granted",
            "pending": "pending",
            "conflict": "rejected",
            "expired": "pending",
            "unassigned": "pending",
        }[decision]
        self.decisions[decision if decision in self.decisions else "pending"] += 1
        if decision == "conflict":
            self.last_decision_reason = f"conflict won by {winner}"
        else:
            self.last_decision_reason = {
                "granted": "reservation granted",
                "pending": "reservation settling",
                "expired": "local reservation lease expired",
                "unassigned": "local reservation is unassigned",
            }[decision]
        return result

    def settle_remaining_s(self):
        """Seconds until the local claim may be granted, or None if not settling."""
        arbiter = self.arbiter
        if arbiter is None or arbiter.local is None:
            return None
        remaining = arbiter.proposed_at + arbiter.settle_s - self.clock()
        return remaining if remaining > 0.0 else None

    def publish_exclusions(self):
        message = self.exclusions_type()
        message.header.frame_id = self.frame
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

    def receive_peer_pose(self, msg):
        try:
            if len(msg.data) > 512:
                return
            value = json.loads(msg.data)
            robot, x, y = str(value["robot_id"]), float(value["x"]), float(value["y"])
            if (
                robot == self.bridge.id
                or not self.raw_authority
                or value["session_id"] != self.raw_authority["mission_id"]
                or robot not in self.raw_authority["participants"]
                or not (math.isfinite(x) and math.isfinite(y))
            ):
                return
            self.peer_positions[robot] = (x, y, self.clock())
        except (ValueError, KeyError, TypeError):
            return

    def publish_peer_bodies(self):
        """Report this robot's position and hand the peers' to the planner."""
        self.peer_bodies_at = self.clock()
        if self.peer_bodies_publisher is None or not self.authority:
            return
        fresh = self.clock() - self.received_at <= 3.0
        pose = getattr(self.bridge, "map_pose", None)
        try:
            pose = pose() if callable(pose) else None
            if fresh and pose is not None:
                here = self.deployment_from_map @ np.array(
                    [float(pose["x"]), float(pose["y"]), 0.0, 1.0]
                )
                if np.all(np.isfinite(here)):
                    self.peer_pose_publisher.publish(
                        self.msg_type(
                            data=json.dumps(
                                {
                                    "robot_id": self.bridge.id,
                                    "session_id": self.raw_authority["mission_id"],
                                    "x": float(here[0]),
                                    "y": float(here[1]),
                                }
                            )
                        )
                    )
        except (KeyError, TypeError, ValueError):
            pass
        message = self.exclusions_type()
        message.header.frame_id = self.frame
        if fresh:
            planning_from_deployment = np.linalg.inv(
                self.authority["T_component_navigation"]
            )
            for robot, (x, y, seen) in self.peer_positions.items():
                if self.clock() - seen > 3.0:
                    continue
                xyz = planning_from_deployment @ np.array([x, y, 0.0, 1.0])
                body = self.pose_type()
                body.position.x, body.position.y = float(xyz[0]), float(xyz[1])
                body.position.z = 0.0
                body.orientation.w = 1.0
                message.poses.append(body)
        self.peer_bodies_publisher.publish(message)

    def tick(self):
        if self.clock() - self.reported_at >= 1.0:
            self.report_progress()
        if self.clock() - self.peer_bodies_at >= 0.5:
            self.publish_peer_bodies()
        if self.clock() - self.exclusions_at >= 0.5:
            self.publish_exclusions()
        if self.arbiter is None or self.arbiter.local is None:
            return
        if self.clock() - self.received_at > 3.0:
            self._release_lease(self.generation, preserve_binding=True)
            return
        if self.clock() - self.last_publish >= 1.0:
            claim = self.arbiter.local
            self.publish(
                self.arbiter.propose(
                    claim.target, radius_m=claim.radius_m, cost=claim.cost
                )
            )

    def release(self, generation):
        self._release_lease(generation, preserve_binding=False)

    def _release_lease(self, generation, *, preserve_binding):
        if generation < self.generation:
            return
        if self.arbiter:
            self.publish(self.arbiter.release())
        if not preserve_binding:
            self.token = None
            self.reservation_transform = None
            self.reservation_signature = None
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

    def summary(self):
        """What the operator can act on: the frame arbitration runs in, who is
        heard, the local claim and how it fared, and the peers' progress."""
        now = self.clock()
        arbiter = self.arbiter
        authority_fresh = self.authority is not None and now - self.received_at <= 3.0
        local = None
        if arbiter is not None and arbiter.local is not None:
            decision, winner = arbiter.decision_with_winner()
            local = {
                "target": [round(float(v), 2) for v in arbiter.local.target],
                "radius_m": arbiter.local.radius_m,
                "decision": decision,
                "winner": winner,
            }
        leases = {}
        if arbiter is not None:
            for robot, (claim, expires_at) in arbiter.leases.items():
                if robot != self.bridge.id and expires_at > now:
                    leases[robot] = [round(float(v), 2) for v in claim.target]
        return {
            "frame": (
                None
                if not authority_fresh
                else (
                    "deployment"
                    if self.deployment_from_map is not None
                    else "component"
                )
            ),
            "component_id": arbiter.component_id if arbiter is not None else None,
            "peers_heard": sorted(arbiter.heads) if arbiter is not None else [],
            "peer_leases": leases,
            "peer_bodies": sorted(
                robot
                for robot, (_x, _y, at) in self.peer_positions.items()
                if now - at <= 3.0
            ),
            "local": local,
            "last_decision": self.last_decision_reason,
            "decisions": dict(self.decisions),
            "reports": (
                {
                    robot: report.state
                    for robot, (report, _at) in self.completion.reports.items()
                }
                if self.completion is not None
                else {}
            ),
            "completion": self.completion_state,
        }
