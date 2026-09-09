#!/usr/bin/env python3
"""ROS 2 batch-query adapter for :mod:`autonomy.indexed_mapping`.

The server watches ``/maps/<mission>/<robot>/snapshot.json`` off the executor
thread. Service callbacks only read an immutable published index, so MGG can
call it from a reentrant callback group without a callback cycle.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

from autonomy.indexed_mapping import (
    IndexedMapView,
    QueryRequest,
    QueryResult,
    QueryStatus,
    SnapshotDirectorySource,
    SnapshotKey,
)

MAX_SNAPSHOT_BYTES = 4 * 1024 * 1024


def _component_ids(snapshot_path: Path) -> tuple[str, ...]:
    size = snapshot_path.stat().st_size
    if size <= 0 or size > MAX_SNAPSHOT_BYTES:
        raise ValueError("snapshot file has invalid size")

    def reject_constant(value: str) -> None:
        raise ValueError(f"nonfinite JSON constant: {value}")

    value = json.loads(snapshot_path.read_bytes(), parse_constant=reject_constant)
    manifests = value.get("manifests") if isinstance(value, dict) else None
    if not isinstance(manifests, list):
        raise ValueError("snapshot manifests are missing")
    result: list[str] = []
    for manifest in manifests:
        revision = (
            manifest.get("graph_revision") if isinstance(manifest, dict) else None
        )
        component = revision.get("component_id") if isinstance(revision, dict) else None
        if not isinstance(component, str) or not component:
            raise ValueError("manifest component_id is invalid")
        result.append(component)
    if len(set(result)) != len(result):
        raise ValueError("snapshot repeats a component_id")
    return tuple(result)


def _stamp_ns(stamp: object) -> int:
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if (
        not isinstance(seconds, int)
        or isinstance(seconds, bool)
        or not isinstance(nanoseconds, int)
        or isinstance(nanoseconds, bool)
        or seconds < 0
        or nanoseconds < 0
        or nanoseconds >= 1_000_000_000
    ):
        raise ValueError("source_stamp is invalid")
    return seconds * 1_000_000_000 + nanoseconds


def _unavailable(detail: str) -> QueryResult:
    return QueryResult(QueryStatus.UNAVAILABLE, None, detail=detail)


class IndexRegistry:
    """Filesystem monitor independent of ROS, with one view per component."""

    def __init__(
        self,
        maps_root: Path,
        mission_id: str,
        *,
        snapshot_age_s: float,
        poll_s: float,
        clock=time.monotonic,
    ):
        try:
            mission = str(uuid.UUID(mission_id))
        except (ValueError, AttributeError) as exc:
            raise ValueError("mission_id must be a canonical UUID") from exc
        if mission != mission_id:
            raise ValueError("mission_id must use canonical UUID spelling")
        self.maps_root = maps_root
        self.mission_id = mission
        self.max_snapshot_age_ns = int(snapshot_age_s * 1e9)
        self.poll_s = poll_s
        self._clock = clock
        self._lock = threading.RLock()
        self._views: dict[tuple[str, str], IndexedMapView] = {}
        self._roots: dict[tuple[str, str], Path] = {}
        # A failed source is retried exponentially, while a changed snapshot
        # identity gets an immediate attempt. Keep this keyed by root rather
        # than component so one bad publication cannot spin all its views.
        self._failed_sources: dict[
            Path, tuple[tuple[int, int, int, int], int, float]
        ] = {}
        self._robots: set[str] = set()
        self._ambiguous: set[str] = set()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="map-index-refresh", daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.poll_s * 2))

    def robots(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(sorted(self._robots))

    def query(self, robot: str, request: QueryRequest) -> QueryResult:
        with self._lock:
            if robot in self._ambiguous:
                return _unavailable("multiple missions expose the same robot map")
            view = self._views.get((robot, request.key.component_id))
        if view is None:
            return _unavailable("component index is unavailable")
        return view.query(request)

    @staticmethod
    def _snapshot_signature(path: Path) -> tuple[int, int, int, int]:
        stat = path.stat()
        return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)

    def _invalidate_root(self, root: Path, detail: str) -> None:
        with self._lock:
            affected = [
                view
                for identity, view in self._views.items()
                if self._roots.get(identity) == root
            ]
        for view in affected:
            view.invalidate(detail)

    def _record_failure(
        self, root: Path, signature: tuple[int, int, int, int], now: float
    ) -> None:
        previous = self._failed_sources.get(root)
        attempts = previous[1] + 1 if previous and previous[0] == signature else 1
        # Seven attempts reach the 60 second ceiling. Keep the counter capped
        # as well as the delay so an always-broken source cannot overflow the
        # exponent after a long-lived process has retried it many times.
        attempts = min(attempts, 7)
        delay = min(60.0, 2.0 ** (attempts - 1))
        self._failed_sources[root] = (signature, attempts, now + delay)

    def _retry_allowed(
        self, root: Path, signature: tuple[int, int, int, int], now: float
    ) -> bool:
        previous = self._failed_sources.get(root)
        if previous is None:
            return True
        if previous[0] != signature:
            # Atomic replacement, mtime/size change, or a new inode means a
            # new source revision. Do not carry failure backoff across it.
            self._failed_sources.pop(root, None)
            return True
        return now >= previous[2]

    def refresh_once(self) -> None:
        paths = tuple(
            sorted((self.maps_root / self.mission_id).glob("*/snapshot.json"))
        )
        robots_to_roots: dict[str, set[Path]] = {}
        discovered: list[tuple[str, str, Path, tuple[int, int, int, int]]] = []
        preserve_roots: set[Path] = set()
        validated_roots: set[Path] = set()
        now = self._clock()
        for path in paths:
            robot = path.parent.name
            root = path.parent
            robots_to_roots.setdefault(robot, set()).add(root)
            try:
                signature = self._snapshot_signature(path)
            except OSError as exc:
                self._invalidate_root(root, f"snapshot validation failed: {exc}")
                continue
            if not self._retry_allowed(root, signature, now):
                preserve_roots.add(root)
                continue
            try:
                components = _component_ids(path)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self._invalidate_root(root, f"snapshot validation failed: {exc}")
                self._record_failure(root, signature, now)
                preserve_roots.add(root)
                continue
            validated_roots.add(root)
            discovered.extend(
                (robot, component, root, signature) for component in components
            )
        with self._lock:
            self._robots = set(robots_to_roots)
            self._ambiguous = {
                robot for robot, roots in robots_to_roots.items() if len(roots) != 1
            }
        failed_roots: set[Path] = set()
        for robot, component, root, signature in discovered:
            if root in failed_roots:
                continue
            if robot in self._ambiguous:
                continue
            identity = (robot, component)
            with self._lock:
                view = self._views.setdefault(identity, IndexedMapView())
                self._roots[identity] = root
            source = SnapshotDirectorySource(root)
            try:
                source.refresh(view, component)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                # IndexedMapView invalidates its old publication on every
                # extraction, integrity, or build failure.
                # Snapshot parsing and chunk I/O can fail before the view is
                # entered, however, so fail closed here as well. This keeps a
                # previously published index from being served while the
                # source is in the retry backoff window.
                self._invalidate_root(root, f"index refresh failed: {exc}")
                try:
                    latest_signature = self._snapshot_signature(source.snapshot_path)
                except OSError:
                    latest_signature = signature
                if latest_signature == signature:
                    self._record_failure(root, signature, self._clock())
                else:
                    self._failed_sources.pop(root, None)
                failed_roots.add(root)
                continue
        # A source failure is recorded once for the whole source refresh. Only
        # a complete pass over all of its components clears an older failure.
        nonambiguous_roots = {
            root
            for robot, roots in robots_to_roots.items()
            if robot not in self._ambiguous
            for root in roots
        }
        successful_roots = validated_roots & nonambiguous_roots - failed_roots
        for root in successful_roots:
            self._failed_sources.pop(root, None)
        current = {(robot, component) for robot, component, _, _ in discovered}
        with self._lock:
            current.update(
                identity
                for identity, root in self._roots.items()
                if root in preserve_roots
            )
            removed = [
                view
                for identity, view in self._views.items()
                if identity not in current
            ]
            removed_identities = [
                identity
                for identity in self._views
                if identity not in current
            ]
            for identity in removed_identities:
                self._views.pop(identity, None)
                self._roots.pop(identity, None)
            active_roots = {path.parent for path in paths}
            for root in tuple(self._failed_sources):
                if root not in active_roots:
                    self._failed_sources.pop(root, None)
        for view in removed:
            view.invalidate("component is absent from the current coherent snapshot")

    def _run(self) -> None:
        while not self._stop.is_set():
            self.refresh_once()
            self._stop.wait(self.poll_s)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-root", type=Path, default=Path("/maps"))
    parser.add_argument("--mission-id", default=os.environ.get("SWARMDECK_MISSION_ID"))
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--max-snapshot-age-s", type=float, default=3.0)
    args = parser.parse_args()
    if not args.mission_id:
        parser.error("--mission-id or SWARMDECK_MISSION_ID is required")
    if args.poll_s <= 0 or args.max_snapshot_age_s <= 0:
        parser.error("poll and source age must be positive")

    import rclpy
    from mgg_msgs.srv import QueryMapBatch
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    registry = IndexRegistry(
        args.maps_root,
        args.mission_id,
        snapshot_age_s=args.max_snapshot_age_s,
        poll_s=args.poll_s,
    )

    class QueryNode(Node):
        def __init__(self) -> None:
            super().__init__("swarmdeck_indexed_map_server")
            # Node._services is rclpy's internal list. Keep our robot lookup
            # separate so create_service() can append to that list normally.
            self._robot_services: dict[str, object] = {}
            self._callbacks = ReentrantCallbackGroup()
            # Discovery mutates the service table. Keep its timer in the
            # default mutually exclusive group; queries may run concurrently.
            self.create_timer(0.25, self._discover_services)

        def _discover_services(self) -> None:
            for robot in registry.robots():
                if robot in self._robot_services:
                    continue
                name = f"/{robot}/mapping/query_batch"
                self._robot_services[robot] = self.create_service(
                    QueryMapBatch,
                    name,
                    lambda request, response, robot=robot: self._query(
                        robot, request, response
                    ),
                    callback_group=self._callbacks,
                )
                self.get_logger().info(f"serving {name}")

        def _query(self, robot, request, response):
            try:
                key = SnapshotKey(
                    request.component_id,
                    request.epoch,
                    request.graph_revision,
                    request.geometry_revision,
                )
                samples = tuple((p.x, p.y, p.z) for p in request.samples)
                body = (request.body_size.x, request.body_size.y, request.body_size.z)
                result = registry.query(
                    robot,
                    QueryRequest(
                        key,
                        samples,
                        body,
                        stop_at_unknown=request.stop_at_unknown,
                        source_stamp_ns=_stamp_ns(request.source_stamp),
                        now_monotonic_ns=time.monotonic_ns(),
                        max_snapshot_age_ns=registry.max_snapshot_age_ns,
                    ),
                )
            except (AttributeError, TypeError, ValueError) as exc:
                result = _unavailable(str(exc))
            response.status = int(result.status)
            response.component_id = result.key.component_id if result.key else ""
            response.epoch = result.key.epoch if result.key else 0
            response.graph_revision = result.key.graph_revision if result.key else 0
            response.geometry_revision = (
                result.key.geometry_revision if result.key else ""
            )
            response.occupancy = list(result.occupancy)
            response.ground_z = list(result.ground_z)
            response.roughness = list(result.roughness)
            response.clearance = list(result.clearance)
            response.step = list(result.step)
            response.drop = list(result.drop)
            response.detail = result.detail
            return response

    rclpy.init()
    node = QueryNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    registry.refresh_once()
    registry.start()
    try:
        executor.spin()
    finally:
        registry.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
