#!/usr/bin/env python3
"""Clear Spot tablet keepalive policies that hold the motors off.

The Boston Dynamics tablet app publishes a `tablet-stop` Keepalive policy with
an immediate `controlled_motors_off` action. Claim then succeeds (the ROS
driver has the lease and the software e-stop is released) but `/power_on`
returns KeepaliveMotorsOffError, so the GUI Claim/Stand buttons look dead.

This node is a std_srvs/Trigger that drops those tablet policies. The adapter
calls it after `/claim`. Lease and e-stop autopolicy entries are left alone.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Iterable

import yaml


def load_spot_login(path: Path) -> tuple[str, str, str]:
    data = yaml.safe_load(path.read_text()) or {}
    params = _ros_params(data)
    try:
        return str(params["hostname"]), str(params["username"]), str(params["password"])
    except KeyError as exc:
        raise SystemExit(f"{path}: missing {exc} under ros__parameters") from exc


def _ros_params(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict):
        return {}
    if "username" in data and "hostname" in data:
        return data
    nested = data.get("ros__parameters")
    if isinstance(nested, dict):
        return nested
    for value in data.values():
        found = _ros_params(value)
        if found:
            return found
    return {}


def tablet_policy_ids(statuses: Iterable[Any]) -> list[int]:
    """Ids of keepalive policies the tablet uses to hold motors off."""
    ids: list[int] = []
    for status in statuses:
        policy = getattr(status, "policy", None)
        if policy is None:
            continue
        name = str(getattr(policy, "name", "") or "")
        user_id = str(getattr(policy, "user_id", "") or "")
        if name == "tablet-stop" or user_id.startswith("bosdyn.android"):
            ids.append(int(status.policy_id))
    return ids


def _clear_once(hostname: str, username: str, password: str) -> tuple[bool, str]:
    import bosdyn.client
    from bosdyn.client.keepalive import KeepaliveClient

    sdk = bosdyn.client.create_standard_sdk("swarmdeck-clear-keepalive")
    robot = sdk.create_robot(hostname)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()
    client = robot.ensure_client(KeepaliveClient.default_service_name)
    status = client.get_status()
    remove = tablet_policy_ids(status.status)
    if not remove:
        return True, "no tablet keepalive"
    client.modify_policy(policy_ids_to_remove=remove)
    return True, f"removed {len(remove)} tablet keepalive policy(ies)"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--params", required=True, type=Path)
    args = parser.parse_args()
    hostname, username, password = load_spot_login(args.params)

    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class ClearKeepalive(Node):
        def __init__(self) -> None:
            super().__init__("spot_clear_keepalive")
            self.create_service(Trigger, "clear_keepalive", self._on_clear)

        def _on_clear(
            self, _req: Trigger.Request, resp: Trigger.Response
        ) -> Trigger.Response:
            try:
                ok, message = _clear_once(hostname, username, password)
            except Exception as exc:
                resp.success = False
                resp.message = str(exc)
                self.get_logger().warn(f"clear_keepalive failed: {exc}")
                return resp
            resp.success = ok
            resp.message = message
            self.get_logger().info(f"clear_keepalive: {message}")
            return resp

    rclpy.init()
    node = ClearKeepalive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
