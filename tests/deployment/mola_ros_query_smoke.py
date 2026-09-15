#!/usr/bin/env python3
"""Bounded native MOLA -> ROS QueryMapBatch acceptance with no motion.

Run in the mapping image after sourcing the generated messages::

    source /opt/swarmdeck-mgg-msgs/local_setup.bash
    timeout 45s env PYTHONPATH=/opt/swarmdeck python3 /usr/local/bin/mola-ros-query-smoke.py
"""

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import ModuleType

import rclpy
from geometry_msgs.msg import Point
from mgg_msgs.srv import QueryMapBatch

INTERNAL_TIMEOUT_S = 35.0


def require(predicate: bool, detail: str) -> None:
    if not predicate:
        raise RuntimeError(detail)


def check_deadline(deadline: float) -> None:
    if time.monotonic() >= deadline:
        raise TimeoutError("MOLA ROS query smoke exceeded its internal deadline")


def load_fixture(path: Path) -> ModuleType:
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"fixture script is unavailable: {path}")
    name = "_swarmdeck_mola_planner_acceptance"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load fixture script: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    for member in ("MISSION", "_fixture", "_materialize", "MolaWorker"):
        if not hasattr(module, member):
            raise RuntimeError(f"fixture script is missing {member}")
    return module


def make_request(
    manifest: dict[str, object],
    *,
    source_stamp_ns: int,
    revision: int | None = None,
) -> QueryMapBatch.Request:
    graph = manifest["graph_revision"]
    assert isinstance(graph, dict)
    request = QueryMapBatch.Request()
    request.component_id = str(graph["component_id"])
    request.epoch = int(graph["epoch"])
    request.graph_revision = int(graph["revision"] if revision is None else revision)
    request.geometry_revision = str(manifest["geometry_revision"])
    request.source_stamp.sec = source_stamp_ns // 1_000_000_000
    request.source_stamp.nanosec = source_stamp_ns % 1_000_000_000
    request.samples = [
        Point(x=2.0, y=0.0, z=0.6),
        Point(x=3.0, y=0.0, z=0.6),
        Point(x=4.0, y=0.0, z=0.6),
    ]
    request.body_size.x = 0.1
    request.body_size.y = 0.1
    request.body_size.z = 0.1
    request.max_step_m = 0.15
    request.max_drop_m = 0.15
    request.stop_at_unknown = False
    return request


def call(node, client, request, deadline: float):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("QueryMapBatch call exceeded the smoke deadline")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=min(3.0, remaining))
    if not future.done():
        future.cancel()
        raise TimeoutError("QueryMapBatch call timed out")
    error = future.exception()
    require(error is None, f"QueryMapBatch call failed: {error}")
    response = future.result()
    require(response is not None, "QueryMapBatch returned no response")
    return response


def wait_for_status(node, client, request, status: int, process, deadline: float):
    last = None
    while time.monotonic() < deadline:
        require(
            process.poll() is None,
            "indexed map server exited while waiting for query status",
        )
        last = call(node, client, request, deadline)
        if last.status == status:
            return last
        time.sleep(0.1)
    detail = getattr(last, "detail", "no response")
    raise TimeoutError(f"QueryMapBatch did not reach status {status}: {detail}")


def source_stamp(manifest: dict[str, object]) -> int:
    submaps = manifest.get("submaps")
    if not isinstance(submaps, list):
        raise RuntimeError("fixture manifest has no submaps")
    stamps = [
        item.get("observed_at_ns", 0) for item in submaps if isinstance(item, dict)
    ]
    if not stamps or any(type(value) is not int or value < 0 for value in stamps):
        raise RuntimeError("fixture manifest has invalid observation stamps")
    return max(stamps)


def stop_process(process: subprocess.Popen[str], log_file) -> str:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)
    log_file.seek(0)
    return log_file.read()


def run(args: argparse.Namespace) -> None:
    deadline = time.monotonic() + args.timeout
    fixture = load_fixture(args.fixture_script)
    importer = args.binary.resolve()
    if not importer.is_file() or not os.access(importer, os.X_OK):
        raise FileNotFoundError(f"native MOLA importer is not executable: {importer}")

    worker = None
    mapper = None
    process = None
    node = None
    rclpy_started = False
    log_file = tempfile.TemporaryFile(mode="w+t")
    failed = False
    with tempfile.TemporaryDirectory(prefix="swarmdeck-mola-ros-query-") as temporary:
        root = Path(temporary)
        maps_root = root / "maps"
        peer = maps_root / fixture.MISSION / "planner"
        try:
            mapper, _, _ = fixture._fixture(root / "store")
            snapshot = fixture._materialize(mapper, peer)
            manifests = snapshot.get("manifests")
            require(
                isinstance(manifests, list) and len(manifests) == 1,
                "fixture must publish exactly one component",
            )
            manifest = manifests[0]
            require(isinstance(manifest, dict), "fixture manifest is invalid")
            stamp_ns = source_stamp(manifest)

            worker = fixture.MolaWorker(
                maps_root,
                importer=importer,
                timeout_s=min(6.0, args.timeout),
                retry_s=0.0,
                planner_maps=True,
                mission_id=fixture.MISSION,
                max_output_bytes=64 * 1024 * 1024,
                max_points_per_map=100_000,
                max_resident_points=200_000,
                max_maps=8,
            )
            published = worker.process_peer(peer)
            require(
                published.published, "MOLA worker discarded the fixture publication"
            )
            worker.close()
            worker = None
            check_deadline(deadline)

            os.environ["ROS_DOMAIN_ID"] = "197"
            os.environ["ROS_LOCALHOST_ONLY"] = "1"
            environment = os.environ.copy()
            process = subprocess.Popen(
                [
                    args.server,
                    "--map-provider",
                    "mola",
                    "--maps-root",
                    str(maps_root),
                    "--mission-id",
                    fixture.MISSION,
                    "--poll-s",
                    "0.1",
                    "--max-snapshot-age-s",
                    "30",
                ],
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                env=environment,
                start_new_session=True,
            )

            rclpy.init()
            rclpy_started = True
            node = rclpy.create_node(f"mola_ros_query_smoke_{os.getpid()}")
            client = node.create_client(QueryMapBatch, "/planner/mapping/query_batch")
            discovery_deadline = min(deadline, time.monotonic() + 8.0)
            while time.monotonic() < discovery_deadline and not client.wait_for_service(
                timeout_sec=0.2
            ):
                require(
                    process.poll() is None,
                    "indexed map server exited before advertising its service",
                )
            require(
                client.service_is_ready(), "MOLA QueryMapBatch service was not found"
            )

            request = make_request(manifest, source_stamp_ns=stamp_ns)
            response = call(node, client, request, deadline)
            require(response.status == QueryMapBatch.Response.OK, response.detail)
            require(
                list(response.occupancy)
                == [
                    QueryMapBatch.Response.FREE,
                    QueryMapBatch.Response.OCCUPIED,
                    QueryMapBatch.Response.UNKNOWN,
                ],
                f"unexpected MOLA occupancy: {list(response.occupancy)}",
            )
            for name in ("ground_z", "roughness", "clearance", "step", "drop"):
                require(len(getattr(response, name)) == 3, f"{name} array is malformed")

            invalid_limits = make_request(manifest, source_stamp_ns=stamp_ns)
            invalid_limits.max_step_m = 0.0
            invalid = call(node, client, invalid_limits, deadline)
            require(
                invalid.status == QueryMapBatch.Response.UNAVAILABLE
                and "max_step_m" in invalid.detail,
                "invalid platform terrain limits did not fail closed",
            )

            graph = manifest["graph_revision"]
            assert isinstance(graph, dict)
            stale = call(
                node,
                client,
                make_request(
                    manifest,
                    source_stamp_ns=stamp_ns,
                    revision=int(graph["revision"]) + 1,
                ),
                deadline,
            )
            require(
                stale.status == QueryMapBatch.Response.STALE,
                "newer requested graph revision was not fenced as STALE",
            )

            (peer / "mola" / "index.json").write_text('{"version":')
            unavailable = wait_for_status(
                node,
                client,
                request,
                QueryMapBatch.Response.UNAVAILABLE,
                process,
                min(deadline, time.monotonic() + 5.0),
            )
            require(
                "validation failed" in unavailable.detail,
                f"corrupt index returned unexpected detail: {unavailable.detail}",
            )
            print(
                "PASS: native MOLA worker -> immutable grid -> ROS QueryMapBatch; "
                "FREE/OCCUPIED/UNKNOWN, platform limits, stale revision, "
                "corrupt-index fail-closed",
                flush=True,
            )
        except BaseException:
            failed = True
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy_started and rclpy.ok():
                rclpy.try_shutdown()
            if worker is not None:
                worker.close()
            if mapper is not None:
                mapper.store.close()
            if process is not None:
                logs = stop_process(process, log_file)
                if failed or process.returncode not in (0, -signal.SIGTERM):
                    print(logs, file=sys.stderr, flush=True)
                if not failed and process.returncode not in (0, -signal.SIGTERM):
                    raise RuntimeError(
                        f"indexed map server exited with {process.returncode}"
                    )
            log_file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixture-script",
        type=Path,
        default=Path("/usr/local/bin/mola-planner-acceptance.py"),
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path(
            os.environ.get(
                "SWARMDECK_MOLA_IMPORTER",
                "/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import",
            )
        ),
    )
    parser.add_argument(
        "--server",
        default="swarmdeck-indexed-map-server",
        help="installed indexed-map server command",
    )
    parser.add_argument("--timeout", type=float, default=INTERNAL_TIMEOUT_S)
    args = parser.parse_args()
    if args.timeout <= 0 or args.timeout > INTERNAL_TIMEOUT_S:
        parser.error(f"--timeout must be in (0, {INTERNAL_TIMEOUT_S:g}]")
    run(args)


if __name__ == "__main__":
    main()
