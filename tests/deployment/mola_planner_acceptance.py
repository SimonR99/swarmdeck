#!/usr/bin/env python3
"""Bounded, no-motion acceptance for the native MOLA planner grid.

The fixture is deliberately built through :class:`SubmapStore` and
``CorrectionAwareMapper``.  The worker is then the only producer of the
native SDMGRID1 artifact; planner queries go through ``MolaDirectorySource``
and ``IndexedMapView`` just as the deployment provider does.

Run this from the repository root (or from a mapping image)::

    PYTHONPATH=. python3 tests/deployment/mola_planner_acceptance.py \
      --binary /mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import

The whole fixture has a hard 45 second deadline and performs no ROS or motion
work.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import struct
import tempfile
import time
import uuid

import numpy as np

from autonomy.contracts import (
    IDENTITY_SE3,
    ZERO_COVARIANCE,
    Calibration,
    CalibratedCapture,
    ComponentRevision,
    DeskewStatus,
    GraphSolution,
    KeyframeId,
    RayEvidence,
    RayReturnSemantics,
    SubmapId,
    component_id_for_anchor,
)
from autonomy.indexed_mapping import (
    IndexedMapView,
    QueryRequest,
    QueryStatus,
    VoxelOccupancy,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from autonomy.mola_mapping import MolaDirectorySource
from deploy.autonomy.mola_worker import MolaWorker

SESSION = str(uuid.UUID("f4c6a20c-0f0c-4dd7-8f1e-5f84b9a7c041"))
MISSION = str(uuid.UUID("0b0b4de4-4a21-47a5-96b9-e27b1e336d44"))
DEADLINE_S = 45.0


def _check_deadline(deadline: float) -> None:
    if time.monotonic() >= deadline:
        raise TimeoutError("native MOLA planner acceptance exceeded 45 seconds")


def _yaw_pose(
    yaw: float, x: float, y: float, z: float = 0.0
) -> tuple[tuple[float, ...], ...]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        (c, -s, 0.0, x),
        (s, c, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _capture(keyframe: KeyframeId, observed_at_ns: int) -> CalibratedCapture:
    """Make explicit synthetic FIRST_RETURN + DESKEWED capture evidence."""

    return CalibratedCapture(
        keyframe,
        observed_at_ns - 1,
        observed_at_ns,
        "synthetic/lidar",
        "synthetic-lidar-v1",
        IDENTITY_SE3,
        ZERO_COVARIANCE,
        DeskewStatus.DESKEWED,
        ray_return_semantics=RayReturnSemantics.FIRST_RETURN,
    )


def _calibration() -> Calibration:
    return Calibration(
        "synthetic-lidar-v1",
        "synthetic/lidar",
        "x-forward/y-left/z-up",
        (),
        "none",
        (),
        IDENTITY_SE3,
    )


def _patches(center_x: float, center_y: float, z: float) -> list[list[float]]:
    return [
        [center_x + dx, center_y + dy, z]
        for dx in (-0.2, 0.0, 0.2)
        for dy in (-0.2, 0.0, 0.2)
    ]


def _fixture(
    store_root: Path,
) -> tuple[CorrectionAwareMapper, tuple[KeyframeId, ...], KeyframeId]:
    """Build one component containing geometry and provenance edge cases."""

    store = SubmapStore(store_root)
    mapper = CorrectionAwareMapper(store, resolution_m=0.2)
    calibration = _calibration()
    wall = KeyframeId("planner", SESSION, 0)
    terrain = KeyframeId("planner", SESSION, 1)
    missing = KeyframeId("planner", SESSION, 2)
    partial = KeyframeId("planner", SESSION, 3)

    # A qualified wall capture.  The one sensor origin is intentionally at
    # (0, 0, 0), so the cells before the first return are measured free.
    wall_points = [
        [3.0, y, z]
        for y in np.arange(-1.0, 1.01, 0.2)
        for z in np.arange(0.2, 1.41, 0.2)
    ]
    mapper.add_capture(_capture(wall, 100), calibration, wall_points)

    # Ground, a curb/step, a down-step, and stacked floors.  These are all
    # actual point returns, rather than fabricated planner voxels.
    floor = [
        [x, y, 0.0]
        for x in np.arange(-1.0, 0.81, 0.2)
        for y in np.arange(-1.0, 1.01, 0.2)
    ]
    curb = [
        [x, y, 0.4]
        for x in np.arange(1.2, 2.41, 0.2)
        for y in np.arange(-1.0, 1.01, 0.2)
    ]
    # Keep support under the wall and beyond it, while leaving the queried
    # z=.6 body cells free/unknown as appropriate.
    floor += [
        [x, y, 0.0]
        for x in np.arange(2.6, 7.01, 0.2)
        for y in np.arange(-1.0, 1.01, 0.2)
    ]
    drop_surfaces = _patches(0.1, 2.0, 0.4) + _patches(1.7, 2.0, -0.4)
    stacked = [[x, y, z] for z in (0.0, 2.0, 4.0) for x, y in _grid_xy(5.0, -2.0)]
    mapper.add_capture(
        _capture(terrain, 101), calibration, floor + curb + drop_surfaces + stacked
    )

    # These captures carry explicit synthetic qualified capture records, but
    # their submaps intentionally omit the certificate.  The native planner
    # must therefore publish occupied endpoints while keeping their preceding
    # and trailing cells unknown.
    missing_capture = _capture(missing, 102)
    store.record_capture(missing_capture, calibration)
    store.add_submap(
        SubmapId.from_keyframe(missing),
        [[6.0, 4.0, 0.5]],
        keyframe_poses_local={missing: IDENTITY_SE3},
        sensor_origins_local=(),
        resolution_m=0.2,
        observed_at_ns=102,
        ray_evidence=RayEvidence(),
    )
    partial_capture = _capture(partial, 103)
    store.record_capture(partial_capture, calibration)
    store.add_submap(
        SubmapId.from_keyframe(partial),
        [[6.0, 5.0, 0.5]],
        keyframe_poses_local={partial: IDENTITY_SE3},
        sensor_origins_local=((0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        resolution_m=0.2,
        observed_at_ns=103,
        ray_evidence=RayEvidence(),
    )

    members = (wall, terrain, missing, partial)
    component = component_id_for_anchor(wall)
    mapper.apply_solution(
        GraphSolution(
            ComponentRevision(component, 0, 1),
            wall,
            members,
            {key: IDENTITY_SE3 for key in members},
        )
    )
    return mapper, members, wall


def _grid_xy(center_x: float, center_y: float) -> list[tuple[float, float]]:
    return [
        (center_x + dx, center_y + dy)
        for dx in (-0.3, -0.1, 0.1, 0.3)
        for dy in (-0.3, -0.1, 0.1, 0.3)
    ]


def _materialize(mapper: CorrectionAwareMapper, peer: Path) -> dict[str, object]:
    """Write only the store's canonical snapshot and content-addressed chunks."""

    snapshot = mapper.store.snapshot()
    peer.mkdir(parents=True, exist_ok=True)
    chunks = peer / "geometry" / "chunks"
    chunks.mkdir(parents=True, exist_ok=True)
    for manifest in snapshot.manifests:
        for chunk in manifest.chunks:
            path = chunks / chunk.sha256
            if not path.exists():
                path.write_bytes(mapper.store.get_chunk(chunk.sha256))
    # Snapshot identity and geometry revisions are produced by SubmapStore.
    # Do not synthesize or patch hashes in this acceptance fixture.
    (peer / "snapshot.json").write_text(snapshot.to_json() + "\n")
    return snapshot.to_dict()


def _query(
    view: IndexedMapView,
    key: object,
    samples: tuple[tuple[float, float, float], ...],
    body: tuple[float, float, float],
    *,
    stop_at_unknown: bool = False,
):
    result = view.query(
        QueryRequest(
            key,  # type: ignore[arg-type]
            samples,
            body,
            stop_at_unknown=stop_at_unknown,
        )
    )
    assert result.status is QueryStatus.OK, result.detail
    return result


def _publish_and_refresh(
    worker: MolaWorker,
    mapper: CorrectionAwareMapper,
    peer: Path,
    source: MolaDirectorySource,
    view: IndexedMapView,
    component: str,
    deadline: float,
) -> object:
    _check_deadline(deadline)
    _materialize(mapper, peer)
    result = worker.process_peer(peer)
    assert result.published, result
    _check_deadline(deadline)
    return source.refresh(view, component)


def _replay_manifest(
    peer: Path,
) -> tuple[dict[str, object], dict[str, dict[str, object]]]:
    raw = (peer / "snapshot.json").read_bytes()
    value = json.loads(raw)
    manifests = value.get("manifests")
    if not isinstance(manifests, list):
        raise ValueError(f"{peer}: snapshot manifests are missing")
    by_component: dict[str, dict[str, object]] = {}
    for manifest in manifests:
        if not isinstance(manifest, dict):
            raise ValueError(f"{peer}: snapshot manifest is invalid")
        revision = manifest.get("graph_revision")
        if not isinstance(revision, dict) or not isinstance(
            revision.get("component_id"), str
        ):
            raise ValueError(f"{peer}: snapshot component is invalid")
        component = str(revision["component_id"])
        if component in by_component:
            raise ValueError(f"{peer}: snapshot repeats {component}")
        by_component[component] = manifest
    return value, by_component


def _qualified_count(manifest: dict[str, object]) -> int:
    count = 0
    submaps = manifest.get("submaps")
    if not isinstance(submaps, list):
        raise ValueError("manifest submaps are invalid")
    for submap in submaps:
        if not isinstance(submap, dict):
            raise ValueError("manifest submap is invalid")
        evidence = submap.get("ray_evidence")
        origins = submap.get("sensor_origins")
        if (
            isinstance(evidence, dict)
            and evidence.get("return_semantics") == "first_return"
            and evidence.get("deskew") in ("deskewed", "not_required")
            and evidence.get("origin_association") == "single_capture"
            and isinstance(origins, list)
            and len(origins) == 1
        ):
            count += 1
    return count


def _planner_header(peer: Path, artifact: dict[str, object]) -> dict[str, object]:
    planner = artifact.get("planner")
    if not isinstance(planner, dict) or not isinstance(planner.get("path"), str):
        raise ValueError(f"{peer}: planner artifact is missing")
    raw = (peer / "mola" / str(planner["path"])).read_bytes()
    if len(raw) < 12 or raw[:8] != b"SDMGRID1":
        raise ValueError(f"{peer}: planner artifact magic is invalid")
    metadata_size = struct.unpack_from("<I", raw, 8)[0]
    if metadata_size <= 0 or 12 + metadata_size > len(raw):
        raise ValueError(f"{peer}: planner metadata is invalid")
    metadata = json.loads(raw[12 : 12 + metadata_size])
    if not isinstance(metadata, dict):
        raise ValueError(f"{peer}: planner metadata is invalid")
    return metadata


def _rss_kib(worker: MolaWorker) -> int | None:
    runtime = getattr(worker, "_runtime", None)
    process = getattr(runtime, "_process", None)
    pid = getattr(process, "pid", None)
    if not isinstance(pid, int):
        return None
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        return None
    return None


def _point_samples(
    manifest: dict[str, object],
) -> tuple[tuple[float, float, float], ...]:
    samples: list[tuple[float, float, float]] = []
    submaps = manifest.get("submaps")
    if not isinstance(submaps, list):
        return ()
    for submap in submaps:
        if not isinstance(submap, dict):
            continue
        bounds = submap.get("bounds")
        pose = submap.get("T_component_submap")
        if (
            not isinstance(bounds, list)
            or len(bounds) != 2
            or not all(isinstance(item, list) and len(item) == 3 for item in bounds)
            or not isinstance(pose, list)
            or len(pose) != 4
        ):
            continue
        local = tuple(
            (float(bounds[0][axis]) + float(bounds[1][axis])) / 2 for axis in range(3)
        )
        rotation = tuple(tuple(float(value) for value in row) for row in pose)
        samples.append(
            tuple(
                sum(rotation[row][axis] * local[axis] for axis in range(3))
                + rotation[row][3]
                for row in range(3)
            )
        )
        if len(samples) >= 8:
            break
    return tuple(samples)


def _require_writable_copy(maps_root: Path, mission_id: str) -> tuple[Path, ...]:
    if not maps_root.is_dir() or not maps_root.is_absolute():
        raise ValueError(
            "--maps-root must be an existing absolute copied-map directory"
        )
    try:
        with tempfile.NamedTemporaryFile(
            prefix=".mola-planner-acceptance-", dir=maps_root, delete=True
        ):
            pass
    except OSError as exc:
        raise ValueError("--maps-root must be writable copied map data") from exc
    mission_root = maps_root / mission_id
    peers = tuple(sorted(path.parent for path in mission_root.glob("*/snapshot.json")))
    if not peers:
        raise ValueError(f"no copied peer snapshots found below {mission_root}")
    return peers


def run_replay(binary: Path, maps_root: Path, mission_id: str) -> dict[str, object]:
    """Replay copied peer maps and report bounded native/provider timings."""

    try:
        canonical_mission = str(uuid.UUID(mission_id))
    except (ValueError, AttributeError) as exc:
        raise ValueError("--mission-id must be a canonical UUID") from exc
    if canonical_mission != mission_id:
        raise ValueError("--mission-id must use canonical UUID spelling")
    peers = _require_writable_copy(maps_root, mission_id)
    worker = MolaWorker(
        maps_root,
        importer=binary,
        timeout_s=30.0,
        retry_s=0.0,
        planner_maps=True,
        mission_id=mission_id,
        max_output_bytes=256 * 1024 * 1024,
    )
    view = IndexedMapView(max_build_s=8.0, max_query_s=0.5)
    started = time.perf_counter()
    reports: list[dict[str, object]] = []
    max_rss = 0
    try:
        for peer in peers:
            publication_started = time.perf_counter()
            result = worker.process_peer(peer)
            publication_s = time.perf_counter() - publication_started
            if not result.published:
                raise RuntimeError(f"{peer}: worker discarded its publication")
            snapshot, manifests = _replay_manifest(peer)
            index = json.loads((peer / "mola" / "index.json").read_text())
            artifacts = {
                str(item["component_id"]): item
                for item in index.get("artifacts", [])
                if isinstance(item, dict) and isinstance(item.get("component_id"), str)
            }
            if set(artifacts) != set(manifests):
                raise RuntimeError(f"{peer}: worker index does not cover its snapshot")
            source = MolaDirectorySource(peer)
            first_refresh_s = 0.0
            warm_samples: list[float] = []
            point_count = 0
            query_statuses: list[str] = []
            query_elapsed_s = 0.0
            peer_rss = 0
            for component, manifest in sorted(manifests.items()):
                expected_points = sum(
                    int(chunk["point_count"])
                    for chunk in manifest.get("chunks", [])
                    if isinstance(chunk, dict)
                )
                metadata = _planner_header(peer, artifacts[component])
                qualified = _qualified_count(manifest)
                if metadata.get("point_count") != expected_points:
                    raise RuntimeError(
                        f"{peer}/{component}: planner point count mismatch"
                    )
                if metadata.get("qualified_ray_keyframes") != qualified:
                    raise RuntimeError(
                        f"{peer}/{component}: qualified-ray count mismatch"
                    )
                if qualified == 0 and metadata.get("free_count") != 0:
                    raise RuntimeError(
                        f"{peer}/{component}: unqualified map has free voxels"
                    )

                first_started = time.perf_counter()
                key = source.refresh(view, component)
                first_refresh_s += time.perf_counter() - first_started
                if view.stats.point_count != expected_points:
                    raise RuntimeError(
                        f"{peer}/{component}: provider point count mismatch"
                    )
                point_count += expected_points
                for _ in range(5):
                    warm_started = time.perf_counter()
                    source.refresh(view, component)
                    warm_samples.append(time.perf_counter() - warm_started)
                samples = _point_samples(manifest)
                if samples:
                    batch8 = (samples * ((8 + len(samples) - 1) // len(samples)))[:8]
                    query_started = time.perf_counter()
                    queried = view.query(
                        QueryRequest(
                            key, batch8, (0.2, 0.2, 0.6), stop_at_unknown=False
                        )
                    )
                    query_elapsed_s += time.perf_counter() - query_started
                    query_statuses.append(queried.status.name)
                rss = _rss_kib(worker)
                if rss is not None:
                    peer_rss = max(peer_rss, rss)
                    max_rss = max(max_rss, rss)
            reports.append(
                {
                    "peer": str(peer.relative_to(maps_root)),
                    "components": len(manifests),
                    "points": point_count,
                    "publication_s": round(publication_s, 6),
                    "first_refresh_s": round(first_refresh_s, 6),
                    "warm_refresh_median_s": (
                        round(statistics.median(warm_samples), 6)
                        if warm_samples
                        else None
                    ),
                    "query_batch8_s": round(query_elapsed_s, 6),
                    "query_statuses": query_statuses,
                    "native_rss_kib": peer_rss or None,
                }
            )
    finally:
        worker.close()
    return {
        "mode": "replay",
        "mission_id": mission_id,
        "peers": reports,
        "max_native_rss_kib": max_rss or None,
        "elapsed_s": round(time.perf_counter() - started, 3),
    }


def run(binary: Path) -> dict[str, object]:
    started = time.monotonic()
    deadline = started + DEADLINE_S
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"native MOLA importer is not executable: {binary}")

    with tempfile.TemporaryDirectory(prefix="swarmdeck-mola-planner-") as temporary:
        root = Path(temporary)
        mapper, members, wall = _fixture(root / "store")
        peer = root / "maps" / MISSION / "planner"
        component = component_id_for_anchor(wall)
        source = MolaDirectorySource(peer)
        view = IndexedMapView(
            resolution_m=0.2,
            max_build_s=4.0,
            max_query_s=0.5,
            max_step_m=0.2,
            max_drop_m=0.2,
        )
        worker = MolaWorker(
            root / "maps",
            importer=binary,
            # Four bounded native publications fit comfortably inside the
            # fixture's 45 second wall clock budget.
            timeout_s=6.0,
            retry_s=0.0,
            planner_maps=True,
            mission_id=MISSION,
            max_output_bytes=64 * 1024 * 1024,
            max_points_per_map=100_000,
            max_resident_points=200_000,
            max_maps=8,
        )
        try:
            key = _publish_and_refresh(
                worker, mapper, peer, source, view, component, deadline
            )
            initial = _query(
                view,
                key,
                ((2.0, 0.0, 0.6), (3.0, 0.0, 0.6), (4.0, 0.0, 0.6)),
                (0.1, 0.1, 0.1),
            )
            assert initial.occupancy == (
                VoxelOccupancy.FREE,
                VoxelOccupancy.OCCUPIED,
                VoxelOccupancy.UNKNOWN,
            ), initial

            missing = _query(
                view,
                key,
                ((5.5, 4.0, 0.5), (6.0, 4.0, 0.5), (5.5, 5.0, 0.5)),
                (0.1, 0.1, 0.1),
            )
            assert missing.occupancy == (
                VoxelOccupancy.UNKNOWN,
                VoxelOccupancy.OCCUPIED,
                VoxelOccupancy.UNKNOWN,
            ), missing

            step = _query(
                view,
                key,
                ((0.0, 0.0, 0.5), (1.8, 0.0, 0.9)),
                (0.2, 0.2, 0.4),
            )
            assert any(step.step), step
            drop = _query(
                view,
                key,
                ((0.1, 2.0, 0.9), (1.7, 2.0, 0.1), (3.5, 2.0, 0.1)),
                (0.2, 0.2, 0.4),
            )
            assert drop.drop[1] and math.isnan(drop.ground_z[2]), drop
            stacked = _query(
                view,
                key,
                ((5.0, -2.0, 0.6), (5.0, -2.0, 2.6)),
                (0.2, 0.2, 0.6),
            )
            assert all(
                abs(actual - expected) <= 0.05
                for actual, expected in zip(stacked.ground_z, (0.0, 2.0))
            ), stacked
            assert all(
                abs(actual - expected) <= 0.05
                for actual, expected in zip(stacked.clearance, (2.0, 2.0))
            ), stacked

            # A rigid translation plus yaw must move both the origin-derived
            # free ray and the endpoint wall. The original wall contribution
            # must not remain in the corrected planner grid.
            corrected = _yaw_pose(math.pi / 2.0, 1.0, 2.0)
            mapper.apply_solution(
                GraphSolution(
                    ComponentRevision(component, 0, 2),
                    wall,
                    members,
                    {keyframe: corrected for keyframe in members},
                )
            )
            key = _publish_and_refresh(
                worker, mapper, peer, source, view, component, deadline
            )
            corrected_queries = _query(
                view,
                key,
                ((1.0, 3.5, 0.6), (1.0, 5.0, 0.6), (3.0, 0.0, 0.6)),
                (0.1, 0.1, 0.1),
            )
            assert corrected_queries.occupancy == (
                VoxelOccupancy.FREE,
                VoxelOccupancy.OCCUPIED,
                VoxelOccupancy.UNKNOWN,
            ), corrected_queries

            # Replacing geometry must remove the old wall before publishing
            # the replacement. The retained corrected pose is applied by the
            # store to the new local point.
            mapper.replace_submap_geometry(
                SubmapId.from_keyframe(wall),
                [[1.0, 0.0, 0.6]],
                keyframe_poses_local={wall: IDENTITY_SE3},
                sensor_origins_local=(),
                observed_at_ns=200,
            )
            key = _publish_and_refresh(
                worker, mapper, peer, source, view, component, deadline
            )
            replaced = _query(
                view,
                key,
                ((1.0, 3.0, 0.6), (1.0, 5.0, 0.6)),
                (0.1, 0.1, 0.1),
            )
            assert replaced.occupancy == (
                VoxelOccupancy.OCCUPIED,
                VoxelOccupancy.UNKNOWN,
            ), replaced

            # Retraction must remove the replacement as well, including its
            # old resident native contribution.
            remaining = tuple(keyframe for keyframe in members if keyframe != wall)
            mapper.apply_solution(
                GraphSolution(
                    ComponentRevision(component, 0, 3),
                    members[1],
                    remaining,
                    {keyframe: corrected for keyframe in remaining},
                    retracted_keyframes=(wall,),
                )
            )
            key = _publish_and_refresh(
                worker, mapper, peer, source, view, component, deadline
            )
            retracted = _query(
                view,
                key,
                ((1.0, 3.0, 0.6), (1.0, 5.0, 0.6)),
                (0.1, 0.1, 0.1),
            )
            assert retracted.occupancy == (
                VoxelOccupancy.UNKNOWN,
                VoxelOccupancy.UNKNOWN,
            ), retracted
            return {
                "elapsed_s": round(time.monotonic() - started, 3),
                "component": component,
                "revisions": 4,
                "native_planner": "SDMGRID1",
                "qualified_captures": 2,
                "unqualified_submaps": 2,
            }
        finally:
            worker.close()
            mapper.store.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
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
        "--maps-root",
        type=Path,
        help="explicit absolute writable copied-map root for replay mode",
    )
    parser.add_argument(
        "--mission-id",
        help="canonical mission UUID for replay mode",
    )
    args = parser.parse_args()
    if (args.maps_root is None) != (args.mission_id is None):
        parser.error("--maps-root and --mission-id must be supplied together")
    result = (
        run_replay(args.binary, args.maps_root, args.mission_id)
        if args.maps_root is not None
        else run(args.binary)
    )
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
