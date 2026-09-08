"""MGG ROS 2 PCI control shared by simulation and hardware adapters.

MGG selects exploration goals; the robot's existing navigation stack executes
and collision-checks them. No planner output is accepted outside an explicit
operator exploration session.
"""

from __future__ import annotations

import math
import time


class MggExploration:
    def __init__(self, bridge, config):
        from std_srvs.srv import Trigger
        from nav_msgs.msg import Path
        from rclpy.qos import QoSProfile, DurabilityPolicy

        self.bridge = bridge
        self.active = False
        self.generation = 0
        self.started_ns = 0
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
        self.stop_client = bridge.node.create_client(Trigger, f"{namespace}/pci_stop")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = bridge.node.create_subscription(
            Path, f"{namespace}/command_path", self.on_path, qos
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
        except Exception as exc:
            self.warn(f"start failed: {exc}")
            self.stop()

    def stop(self):
        # Disable intake before touching ROS: a late service response or path
        # cannot re-arm exploration after Stop All or manual control.
        was_active = self.active
        self.active = False
        self.generation += 1
        if was_active:
            try:
                if self.stop_client.service_is_ready():
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

    def on_path(self, path):
        if not self.active:
            return
        stamp = path.header.stamp.sec * 1_000_000_000 + path.header.stamp.nanosec
        if stamp < self.started_ns:
            return  # Discard the latched path from a previous session.
        if not path.poses:
            self.stop()  # MGG publishes an empty path on stop/completion.
            return
        if path.header.frame_id.lstrip("/") != self.frame:
            self.warn(
                f"path frame {path.header.frame_id!r} differs from {self.frame!r}"
            )
            self.stop()
            return
        pose = path.poses[-1].pose
        q = pose.orientation
        goal = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            ),
        }
        if not all(math.isfinite(v) for v in goal.values()):
            self.warn("invalid planner goal")
            self.stop()
            return
        generation = self.generation
        if not self.active:
            return
        try:
            self.bridge.navigate_to(goal)
        except Exception as exc:
            self.warn(f"navigation failed: {exc}")
            self.stop()
        # Some hardware service calls block briefly. Stop still wins if it
        # arrives while the navigation request is being prepared.
        if not self.active or generation != self.generation:
            self.bridge.cancel_goal()
            self.bridge.drive(0.0, 0.0)


def configure_exploration(bridge):
    config = (bridge.cfg or {}).get("exploration") or {}
    bridge.exploration = (
        MggExploration(bridge, config) if config.get("enabled", False) else None
    )
