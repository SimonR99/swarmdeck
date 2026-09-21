#!/usr/bin/env python3
"""ROS 2 batch-query adapter for planner-ready immutable map grids.

The selected provider is refreshed off the executor thread. Service callbacks
only read an immutable published index, so MGG can call it from a reentrant
callback group without a callback cycle.
"""

from __future__ import annotations

import argparse
from functools import partial
import logging
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
from autonomy.map_provider import (
    MapProvider,
    MapProviderFactory,
    PublicationPending,
)
from autonomy.map_epochs import assert_map_epoch_dependencies, read_map_epoch

LOGGER = logging.getLogger("swarmdeck.indexed_map_server")


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


def provider_factory(name: str) -> MapProviderFactory:
    if name == "indexed":
        return SnapshotDirectorySource
    if name == "mola":
        from autonomy.mola_mapping import MolaDirectorySource

        return MolaDirectorySource
    raise ValueError(f"unknown map provider: {name}")


class IndexRegistry:
    """Filesystem monitor independent of ROS, with one view per component."""

    def __init__(
        self,
        maps_root: Path,
        mission_id: str,
        *,
        snapshot_age_s: float,
        poll_s: float,
        superseded_grace_s: float = 15.0,
        clock=time.monotonic,
        provider_factory: MapProviderFactory = SnapshotDirectorySource,
    ):
        try:
            mission = str(uuid.UUID(mission_id))
        except (ValueError, AttributeError) as exc:
            raise ValueError("mission_id must be a canonical UUID") from exc
        if mission != mission_id:
            raise ValueError("mission_id must use canonical UUID spelling")
        if not math.isfinite(superseded_grace_s) or superseded_grace_s < 0:
            raise ValueError("superseded_grace_s must be finite and non-negative")
        self.maps_root = maps_root
        self.mission_id = mission
        self.max_snapshot_age_ns = int(snapshot_age_s * 1e9)
        # A view keeps answering a key it served until this long after that
        # key was superseded; see IndexedMapView.
        self.superseded_grace_ns = int(superseded_grace_s * 1e9)
        self.poll_s = poll_s
        self._clock = clock
        self._provider_factory = provider_factory
        self._lock = threading.RLock()
        self._views: dict[tuple[str, str], IndexedMapView] = {}
        self._roots: dict[tuple[str, str], Path] = {}
        self._providers: dict[Path, MapProvider] = {}
        self._map_epochs: dict[Path, dict | None] = {}
        # A failed source is retried exponentially, while a changed snapshot
        # identity gets an immediate attempt. Keep this keyed by root rather
        # than component so one bad publication cannot spin all its views.
        self._failed_sources: dict[Path, tuple[tuple[int, ...], int, float]] = {}
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
            root = self._roots.get((robot, request.key.component_id))
            epoch = self._map_epochs.get(root)
        if view is None or root is None:
            return _unavailable("component index is unavailable")
        # Reset is a lifetime fence, not a product transition. Check on the
        # request path as well as the polling thread so an old immutable view
        # cannot answer during the interval before the next refresh.
        try:
            if read_map_epoch(root) != epoch:
                view.invalidate("robot map lifetime changed")
                return _unavailable("robot map lifetime changed")
            result = view.query(request)
            if read_map_epoch(root) != epoch:
                view.invalidate("robot map lifetime changed during query")
                return _unavailable("robot map lifetime changed during query")
            return result
        except (OSError, ValueError) as exc:
            view.invalidate(f"robot map lifetime claim is invalid: {exc}")
            return _unavailable("robot map lifetime claim is invalid")

    def _invalidate_root(self, root: Path, detail: str) -> None:
        # Failing closed refuses every query for this robot until the next
        # good refresh. The retry backoff bounds how often this is reached,
        # so say why each time: a silent refusal is undiagnosable live.
        LOGGER.warning("%s: %s", root.name, detail)
        with self._lock:
            affected = [
                view
                for identity, view in self._views.items()
                if self._roots.get(identity) == root
            ]
        for view in affected:
            view.invalidate(detail)

    def _record_failure(
        self, root: Path, signature: tuple[int, ...], now: float
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
        self, root: Path, signature: tuple[int, ...], now: float
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
        mission_root = self.maps_root / self.mission_id
        roots = tuple(sorted(path for path in mission_root.glob("*") if path.is_dir()))
        sources: list[tuple[Path, MapProvider]] = []
        for root in roots:
            try:
                epoch = read_map_epoch(root)
                previous = self._map_epochs.get(root)
                if previous is not None and epoch is None:
                    raise ValueError("robot map lifetime claim disappeared")
            except (OSError, ValueError) as exc:
                self._invalidate_root(
                    root, f"robot map lifetime claim is invalid: {exc}"
                )
                continue
            if epoch != previous:
                self._invalidate_root(root, "robot map lifetime changed")
                with self._lock:
                    for identity in tuple(self._views):
                        if self._roots.get(identity) == root:
                            self._views.pop(identity)
                            self._roots.pop(identity, None)
                    self._providers.pop(root, None)
                    self._failed_sources.pop(root, None)
            with self._lock:
                self._map_epochs[root] = epoch
            source = self._providers.get(root)
            if source is None:
                source = self._provider_factory(root)
                self._providers[root] = source
            if source.publication_path.is_file():
                sources.append((root, source))
        robots_to_roots: dict[str, set[Path]] = {}
        discovered: list[tuple[str, str, Path, MapProvider, tuple[int, ...]]] = []
        preserve_roots: set[Path] = set()
        validated_roots: set[Path] = set()
        now = self._clock()
        for root, source in sources:
            robot = root.name
            robots_to_roots.setdefault(robot, set()).add(root)
            try:
                signature = source.signature()
            except OSError as exc:
                # The provider stats its own publication files (for MOLA,
                # mola/source.json and mola/index.json; never the bridge's
                # snapshot.json), so a missing one is a publication problem.
                self._invalidate_root(root, f"publication signature failed: {exc}")
                continue
            if not self._retry_allowed(root, signature, now):
                preserve_roots.add(root)
                continue
            try:
                components = source.component_ids()
            except PublicationPending:
                # Between two publications: keep serving the indexed one for
                # its own key and look again on the next poll.
                preserve_roots.add(root)
                continue
            except (OSError, ValueError) as exc:
                self._invalidate_root(root, f"map publication validation failed: {exc}")
                self._record_failure(root, signature, now)
                preserve_roots.add(root)
                continue
            validated_roots.add(root)
            discovered.extend(
                (robot, component, root, source, signature) for component in components
            )
        with self._lock:
            self._robots = set(robots_to_roots)
            self._ambiguous = {
                robot for robot, roots in robots_to_roots.items() if len(roots) != 1
            }
        failed_roots: set[Path] = set()
        for robot, component, root, source, signature in discovered:
            if root in failed_roots:
                continue
            if robot in self._ambiguous:
                continue
            identity = (robot, component)
            with self._lock:
                view = self._views.get(identity)
                if view is None:
                    view = IndexedMapView(
                        superseded_grace_ns=self.superseded_grace_ns,
                        dependency_validator=partial(
                            assert_map_epoch_dependencies, root
                        ),
                    )
                    self._views[identity] = view
                self._roots[identity] = root
            try:
                source.refresh(view, component)
            except PublicationPending:
                failed_roots.add(root)
                continue
            except (OSError, ValueError) as exc:
                # IndexedMapView invalidates its old publication on every
                # extraction, integrity, or build failure.
                # Snapshot parsing and chunk I/O can fail before the view is
                # entered, however, so fail closed here as well. This keeps a
                # previously published index from being served while the
                # source is in the retry backoff window.
                self._invalidate_root(root, f"index refresh failed: {exc}")
                try:
                    latest_signature = source.signature()
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
        current = {(robot, component) for robot, component, _, _, _ in discovered}
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
                identity for identity in self._views if identity not in current
            ]
            for identity in removed_identities:
                self._views.pop(identity, None)
                self._roots.pop(identity, None)
            active_roots = {root for root, _ in sources}
            for root in tuple(self._failed_sources):
                if root not in active_roots:
                    self._failed_sources.pop(root, None)
            for root in tuple(self._providers):
                if root not in roots:
                    self._providers.pop(root, None)
        for view in removed:
            view.invalidate("component is absent from the current coherent snapshot")

    def _run(self) -> None:
        while not self._stop.is_set():
            self.refresh_once()
            self._stop.wait(self.poll_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps-root", type=Path, default=Path("/maps"))
    parser.add_argument("--mission-id", default=os.environ.get("SWARMDECK_MISSION_ID"))
    parser.add_argument("--poll-s", type=float, default=0.5)
    # See docker-compose.mapping.yml: a serial decode of every peer's new
    # product must fit inside this bound.
    parser.add_argument("--max-snapshot-age-s", type=float, default=15.0)
    # A string default is parsed like a command-line value, so a malformed
    # environment value is reported by argparse rather than as a traceback.
    parser.add_argument(
        "--superseded-grace-s",
        type=float,
        default=os.environ.get("SWARMDECK_MAP_QUERY_SUPERSEDED_GRACE_S", "15"),
        help=(
            "keep answering a snapshot key for this many seconds after a newer "
            "product replaced it, so a route planned under that key can still "
            "be validated against the geometry it was planned on; 0 disables"
        ),
    )
    parser.add_argument(
        "--map-provider",
        choices=("indexed", "mola"),
        default=os.environ.get("SWARMDECK_PLANNER_MAP_PROVIDER", "indexed"),
        help="planner grid provider; MOLA is explicit opt-in and never falls back",
    )
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.mission_id:
        parser.error("--mission-id or SWARMDECK_MISSION_ID is required")
    if args.poll_s <= 0 or args.max_snapshot_age_s <= 0:
        parser.error("poll and source age must be positive")
    if not math.isfinite(args.superseded_grace_s) or args.superseded_grace_s < 0:
        parser.error("superseded grace must be finite and non-negative")
    return args


def main() -> None:
    args = parse_args()

    import rclpy
    from mgg_msgs.srv import QueryMapBatch
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    selected_provider = provider_factory(args.map_provider)
    registry = IndexRegistry(
        args.maps_root,
        args.mission_id,
        snapshot_age_s=args.max_snapshot_age_s,
        poll_s=args.poll_s,
        superseded_grace_s=args.superseded_grace_s,
        provider_factory=selected_provider,
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
                        max_step_m=request.max_step_m,
                        max_drop_m=request.max_drop_m,
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
