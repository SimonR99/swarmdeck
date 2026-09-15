#!/usr/bin/env python3
"""Exercise the persisted map -> ROS QueryMapBatch boundary in the mapping image.

The fixture is intentionally tiny. It proves that the production server can
decode the canonical snapshot, advertise the generated service type, and send
all response arrays through DDS without callback/type-conversion errors.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from geometry_msgs.msg import Point
from mgg_msgs.srv import QueryMapBatch

from autonomy.contracts import (
    IDENTITY_SE3,
    KeyframeId,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.mapping import SubmapStore


def require(predicate: bool, detail: str) -> None:
    if not predicate:
        raise RuntimeError(detail)


def make_request(manifest, component_id: str, *, stamp_ns: int, revision=None):
    request = QueryMapBatch.Request()
    request.component_id = component_id
    request.epoch = manifest.graph_revision.epoch
    request.graph_revision = (
        manifest.graph_revision.revision if revision is None else revision
    )
    request.geometry_revision = manifest.geometry_revision
    request.source_stamp.sec = stamp_ns // 1_000_000_000
    request.source_stamp.nanosec = stamp_ns % 1_000_000_000
    request.samples = [
        Point(x=0.1, y=0.1, z=0.5),
        Point(x=2.1, y=0.1, z=0.5),
        Point(x=0.1, y=2.1, z=0.5),
    ]
    request.body_size.x = 0.1
    request.body_size.y = 0.1
    request.body_size.z = 0.1
    request.max_step_m = 0.15
    request.max_drop_m = 0.15
    request.stop_at_unknown = False
    return request


def call(node, client, request, timeout_s: float = 10.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    require(future.done(), "QueryMapBatch call timed out")
    error = future.exception()
    require(error is None, f"QueryMapBatch call failed: {error}")
    response = future.result()
    require(response is not None, "QueryMapBatch returned no response")
    return response


def main() -> None:
    mission_id = str(uuid.uuid4())
    robot_id = "indexed_probe"
    observed_at_ns = 3_000_000_123
    with tempfile.TemporaryDirectory(prefix="indexed-map-smoke-") as directory:
        maps_root = Path(directory)
        peer_root = maps_root / mission_id / robot_id
        store = SubmapStore(peer_root / "geometry")
        keyframe = KeyframeId(robot_id, mission_id, 0)
        store.add_submap(
            SubmapId.from_keyframe(keyframe),
            [[2.1, 0.1, 0.5]],
            keyframe_poses_local={keyframe: IDENTITY_SE3},
            sensor_origins_local=((-0.1, 0.1, 0.5),),
            resolution_m=0.2,
            observed_at_ns=observed_at_ns,
        )
        snapshot = store.snapshot()
        require(len(snapshot.manifests) == 1, "fixture did not produce one manifest")
        manifest = snapshot.manifests[0]
        component_id = component_id_for_anchor(keyframe)
        peer_root.mkdir(parents=True, exist_ok=True)
        (peer_root / "snapshot.json").write_text(
            json.dumps(snapshot.to_dict(), allow_nan=False)
        )

        process = subprocess.Popen(
            [
                "swarmdeck-indexed-map-server",
                "--maps-root",
                str(maps_root),
                "--mission-id",
                mission_id,
                "--poll-s",
                "0.1",
                "--max-snapshot-age-s",
                "10",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node(f"indexed_map_smoke_{os.getpid()}")
        client = node.create_client(QueryMapBatch, f"/{robot_id}/mapping/query_batch")
        logs = ""
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and not client.wait_for_service(
                timeout_sec=0.2
            ):
                require(
                    process.poll() is None,
                    "indexed map server exited before advertising its service",
                )
            require(client.service_is_ready(), "indexed map service was not discovered")

            response = call(
                node,
                client,
                make_request(manifest, component_id, stamp_ns=observed_at_ns),
            )
            require(response.status == QueryMapBatch.Response.OK, response.detail)
            require(response.component_id == component_id, "component echo changed")
            require(
                response.epoch == manifest.graph_revision.epoch, "epoch echo changed"
            )
            require(
                response.graph_revision == manifest.graph_revision.revision,
                "graph revision echo changed",
            )
            require(
                response.geometry_revision == manifest.geometry_revision,
                "geometry revision echo changed",
            )
            require(
                list(response.occupancy)
                == [
                    QueryMapBatch.Response.FREE,
                    QueryMapBatch.Response.OCCUPIED,
                    QueryMapBatch.Response.UNKNOWN,
                ],
                f"unexpected occupancy array: {list(response.occupancy)}",
            )
            for name in ("ground_z", "roughness", "clearance", "step", "drop"):
                require(len(getattr(response, name)) == 3, f"{name} array is malformed")

            stale_stamp = call(
                node,
                client,
                make_request(manifest, component_id, stamp_ns=observed_at_ns + 1),
            )
            require(
                stale_stamp.status == QueryMapBatch.Response.STALE,
                "source stamp mismatch was not rejected as STALE",
            )
            require(
                stale_stamp.component_id == component_id
                and stale_stamp.graph_revision == manifest.graph_revision.revision,
                "stale response did not echo the current indexed key",
            )

            stale_key = call(
                node,
                client,
                make_request(
                    manifest,
                    component_id,
                    stamp_ns=observed_at_ns,
                    revision=manifest.graph_revision.revision + 1,
                ),
            )
            require(
                stale_key.status == QueryMapBatch.Response.STALE,
                "graph key mismatch was not rejected as STALE",
            )
            require(
                stale_key.graph_revision == manifest.graph_revision.revision,
                "stale-key response did not echo the current graph revision",
            )
            print(
                "PASS: canonical snapshot -> indexed server -> QueryMapBatch; "
                "FREE/OCCUPIED/UNKNOWN arrays and exact key/stamp fencing",
                flush=True,
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
            try:
                logs = process.communicate(timeout=5)[0] or ""
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                logs = process.communicate(timeout=5)[0] or ""
            if process.returncode not in (0, -signal.SIGTERM):
                print(logs, flush=True)
                raise RuntimeError(
                    f"indexed map server exited with {process.returncode}"
                )


if __name__ == "__main__":
    main()
