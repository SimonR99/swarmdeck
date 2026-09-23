"""Read-only inspection views over durable onboard map replicas.

This endpoint returns manifest metadata and immutable chunk references. The UI
chooses one component and applies each submap's ``T_component_submap`` locally;
the server never invents a transform between disconnected components.

The one composition this module does perform is the deployment composite
(``deployment:<session>``): every replicated single-robot component of the
active mission placed in the surveyed deployment frame that the 2D fleet map
already uses. It is a display and goal-entry composition, not a verified merge;
see ``deployment_view``.
"""

from __future__ import annotations

import asyncio
import math
import os
from collections import OrderedDict
from threading import Lock
from typing import Any, Mapping
import time
from weakref import WeakKeyDictionary

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from autonomy.contracts import IDENTITY_SE3, validate_se3
from ..fleet.registry import registry
from ..mapsvc.service import map_service
from .autonomy_routes import store
from .replica_components import (
    ComponentCatalogue,
    MAX_ENVELOPES,
    MAX_SOURCE_SUBMAPS,
    MAX_SUBMAPS,
    XYZ_ENCODING,
    XYZRGBA_ENCODING,
    _normalize_envelope,
    digest,
)
from .replica_live import router as live_router, view_solution_order

router = APIRouter(prefix="/api/autonomy/replicas", tags=["autonomy replica views"])
router.include_router(live_router)
_catalogues = WeakKeyDictionary()
_catalogue_lock = Lock()
_composites = WeakKeyDictionary()

DEPLOYMENT_FRAME = "deployment"
DEPLOYMENT_PREFIX = "deployment:"


class _CatalogueState:
    def __init__(self):
        self.lock = Lock()
        self.catalogues = OrderedDict()
        self.normalized = OrderedDict()


def _state(replica_store):
    with _catalogue_lock:
        return _catalogues.setdefault(replica_store, _CatalogueState())


def current_catalogue(session_id: str | None):
    replica_store = store()
    state = _state(replica_store)
    # Serialize misses for one store. Without this recheck, simultaneous UI and
    # live-overlay reads normalize and hash the same immutable manifests twice.
    with state.lock:
        versions = replica_store.snapshot_versions(session_id)
        tombstones = replica_store.tombstones(session_id)
        previous = state.catalogues.get(session_id)
        if previous is not None and previous[0] == (versions, tombstones):
            state.catalogues.move_to_end(session_id)
            return previous[1]

        snapshots = replica_store.snapshots(session_id)
        # A publication may have advanced during the first version read. Associate
        # the cache with the actual coherent snapshot used to construct the view.
        versions = tuple(
            (e["robot_id"], e["session_id"], e["revision"], e["map_epoch"], e["run_id"])
            for e in snapshots
        )

        def normalize(envelope):
            owner = (envelope["session_id"], envelope["robot_id"])
            revision = (envelope["revision"], envelope["map_epoch"], envelope["run_id"])
            cached = state.normalized.get(owner)
            if cached is not None and cached[0] == revision:
                state.normalized.move_to_end(owner)
                return cached[1]
            source = _normalize_envelope(envelope)
            state.normalized[owner] = (revision, source)
            state.normalized.move_to_end(owner)
            while len(state.normalized) > MAX_ENVELOPES:
                state.normalized.popitem(last=False)
            return source

        tombstones = replica_store.tombstones(session_id)
        catalogue = ComponentCatalogue(
            snapshots, normalizer=normalize, tombstones=tombstones
        )
        state.catalogues[session_id] = ((versions, tombstones), catalogue)
        state.catalogues.move_to_end(session_id)
        while len(state.catalogues) > 2:
            state.catalogues.popitem(last=False)
        retained_revisions = {
            ((source_session, robot_id), (revision, epoch, run))
            for (cached_versions, _), _ in state.catalogues.values()
            for robot_id, source_session, revision, epoch, run in cached_versions
        }
        for owner, cached in tuple(state.normalized.items()):
            if (owner, cached[0]) not in retained_revisions:
                del state.normalized[owner]
        return catalogue


# Transforms follow the autonomy contract: ``T_a_b`` maps frame ``b`` into
# frame ``a``. The map service places each robot's navigation frame in the
# merged world frame with a surveyed SE(3) translation (yaw-only rotation);
# the 2D fleet map projects the same placement to SE(2). A robot's live
# authority places that navigation frame in its component frame with
# ``T_component_navigation``. Their composition places the component:
#
#     T_world_component = T_world_navigation @ inv(T_component_navigation)


def deployment_component_id(session_id: str) -> str:
    return f"{DEPLOYMENT_PREFIX}{session_id}"


def is_deployment_component(component_id: Any) -> bool:
    return isinstance(component_id, str) and component_id.startswith(DEPLOYMENT_PREFIX)


def _se3_matrix(
    x: float, y: float, z: float, yaw: float
) -> tuple[tuple[float, ...], ...]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        (c, -s, 0.0, float(x)),
        (s, c, 0.0, float(y)),
        (0.0, 0.0, 1.0, float(z)),
        (0.0, 0.0, 0.0, 1.0),
    )


def _placement_matrix(values) -> tuple[tuple[float, ...], ...]:
    """Build the world placement matrix from canonical ``(x, y, z, yaw)``."""
    x, y, z, yaw = values
    return _se3_matrix(float(x), float(y), float(z), float(yaw))


def _matmul(a, b) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def _invert_se3(matrix) -> tuple[tuple[float, ...], ...]:
    """Invert a rigid transform by transposing its rotation."""
    rotation_t = [[matrix[j][i] for j in range(3)] for i in range(3)]
    translation = [
        -sum(rotation_t[i][k] * matrix[k][3] for k in range(3)) for i in range(3)
    ]
    return tuple(tuple(rotation_t[i] + [translation[i]]) for i in range(3)) + (
        (0.0, 0.0, 0.0, 1.0),
    )


def _rows(matrix) -> list[list[float]]:
    return [[float(value) for value in row] for row in matrix]


def deployment_placements(session_id: str | None) -> dict[str, dict[str, Any]]:
    """Each robot's live component and where the deployment frame places it.

    A robot is placed only when both halves are known: the map service holds a
    transform for its navigation frame (a surveyed start pose, or an accepted
    deployment placement) and its latest live mapping authority names the
    component that frame is expressed in. The latest authority is used without
    a freshness cut so a momentary telemetry gap does not move or remove a
    member's geometry; the live overlay applies its own freshness budget to
    robot poses. A robot whose adapter already reports in the merged frame has
    the identity placement. This assumes the live ``navigation_frame`` is the
    frame the map service's transform describes, which is how the 2D fleet map
    draws the same robot.
    """
    if not session_id:
        return {}
    with map_service._state_lock:
        transforms = dict(map_service.transforms)
    placements: dict[str, dict[str, Any]] = {}
    for robot in list(registry.robots.values()):
        live = robot.live_mapping
        if live is None or live["mission_id"] != session_id:
            continue
        if robot.coordinate_frame == "merged":
            world_navigation = IDENTITY_SE3
        else:
            transform = transforms.get(robot.robot_id)
            if transform is None:
                continue
            world_navigation = _placement_matrix(transform)
        try:
            component_navigation = validate_se3(
                live["T_component_navigation"], "T_component_navigation"
            )
        except ValueError:
            continue
        placements[robot.robot_id] = {
            "robot_id": robot.robot_id,
            "component_id": live["component_id"],
            "solution_order": tuple(live["solution_order"]),
            "T_world_navigation": world_navigation,
            "T_component_navigation": component_navigation,
            "T_world_component": _matmul(
                world_navigation, _invert_se3(component_navigation)
            ),
        }
    return placements


def deployment_members(
    catalogue: ComponentCatalogue,
    session_id: str,
    placements: Mapping[str, Mapping[str, Any]],
) -> list[tuple[str, Mapping[str, Any], dict]]:
    """Placed robots whose live component is a ready single-robot component.

    A component published by more than one robot is a verified merge and is
    never composed: the UI prefers it on its own, and two publishers could
    disagree on where the deployment frame places it.

    The member's replica may carry a newer solution order than its live
    authority: the authority names the frame of the last published MOLA
    product (every 10 to 27 s), the replica follows every solver result that
    moves a pose, and a robot with intra-robot loop closures runs ahead of
    its product most of the time. Requiring the two to agree dropped the
    robot with the largest map from the fleet composite (benchbot
    2026-09-19: robot_0 at replica order 1166 was absent from both the 3D
    view and the 2D raster). The composite is a display placement, so the
    member is placed with the authority's ``T_component_navigation`` and its
    geometry may sit off by the solver's correction since that product; the
    live overlay and goal dispatch still fence the robot's own frame
    revision through the member record.
    """
    members = []
    for robot_id in sorted(placements):
        placement = placements[robot_id]
        key = (session_id, placement["component_id"])
        sources = catalogue.groups.get(key)
        if not sources or [source["robot_id"] for source in sources] != [robot_id]:
            continue
        try:
            view = catalogue.view(session_id, placement["component_id"])
        except (LookupError, TypeError, ValueError):
            continue
        if view.get("solution_order_known") is not True:
            continue
        members.append((robot_id, placement, view))
    return members


def deployment_composite(
    session_id: str, members: list[tuple[str, Mapping[str, Any], dict]]
) -> dict[str, Any]:
    """Union the members' component views, re-expressed in the deployment frame.

    Chunks are immutable and pass through untouched (the browser caches them
    by hash); only each submap's ``T_component_submap`` becomes
    ``T_world_component @ T_component_submap``. The per-component budgets
    bound the union; exceeding them is an ``OverflowError`` so the routes
    answer 413 exactly as an oversized catalogue does.
    """
    submaps: dict[str, dict] = {}
    chunks: dict[str, dict] = {}
    tombstones: set[str] = set()
    publications: list[dict] = []
    generated_at_ns = 0
    for robot_id, placement, view in members:
        selected = view["selected"]
        world_component = placement["T_world_component"]
        for submap in selected["submaps"]:
            submap_id = submap["submap_id"]
            if submap_id in submaps:
                raise ValueError("Deployment members repeat an active submap")
            submaps[submap_id] = {
                **submap,
                "T_component_submap": _rows(
                    _matmul(world_component, submap["T_component_submap"])
                ),
            }
            if len(submaps) > MAX_SUBMAPS:
                raise OverflowError("Deployment composite exceeds the submap budget")
        for chunk in view["chunks"]:
            previous = chunks.get(chunk["sha256"])
            if previous is not None and previous != chunk:
                raise ValueError("One chunk hash has conflicting descriptors")
            chunks[chunk["sha256"]] = chunk
        tombstones.update(selected["tombstones"])
        if len(submaps) + len(tombstones) > MAX_SOURCE_SUBMAPS:
            raise OverflowError("Deployment composite exceeds the source budget")
        publications.extend(view["sources"])
        generated_at_ns = max(generated_at_ns, view["generated_at_ns"])
    publications.sort(key=lambda source: source["robot_id"])
    member_records = [
        {
            "robot_id": robot_id,
            "component_id": placement["component_id"],
            "solution_order": list(placement["solution_order"]),
            "snapshot_id": view["snapshot_id"],
            "T_world_component": _rows(placement["T_world_component"]),
            "T_world_navigation": _rows(placement["T_world_navigation"]),
        }
        for robot_id, placement, view in members
    ]
    component_id = deployment_component_id(session_id)
    component = {
        "component_id": component_id,
        "frame_id": DEPLOYMENT_FRAME,
        "graph_revision": None,
        "geometry_revision": digest(
            [
                (robot_id, view["selected"]["geometry_revision"])
                for robot_id, _, view in members
            ]
        ),
        "submaps": [submaps[name] for name in sorted(submaps)],
        "tombstones": sorted(tombstones),
    }
    return {
        "version": 1,
        "scope": "fleet",
        "robot_id": "fleet",
        "session_id": session_id,
        "component_id": component_id,
        "revision": None,
        # Placement is part of the publication identity: a moved member must
        # re-render even when no member replica changed.
        "snapshot_id": digest([DEPLOYMENT_FRAME, session_id, member_records]),
        "generated_at_ns": generated_at_ns,
        # No solver produced this frame. ``None`` is the catalogue's spelling of
        # the pre-optimizer sentinel (0, -1); the live routes and the browser
        # both read it that way, and each member keeps its own frame revision.
        "solution_order": None,
        "solution_order_known": True,
        "components": [component],
        "selected": component,
        "chunks": [chunks[name] for name in sorted(chunks)],
        "source_age_s": None,
        "age_clock": "unknown",
        "sources": publications,
        "geometry_encodings": [XYZ_ENCODING, XYZRGBA_ENCODING],
        "reconstruction": None,
        "composite": True,
        "members": member_records,
    }


def _cached_composite(
    catalogue: ComponentCatalogue,
    session_id: str,
    members: list[tuple[str, Mapping[str, Any], dict]],
) -> dict[str, Any]:
    """The browser polls every second; reuse the union until a member moves."""
    cache_key = tuple(
        (
            robot_id,
            view["snapshot_id"],
            placement["T_world_component"],
            placement["T_world_navigation"],
        )
        for robot_id, placement, view in members
    )
    with _catalogue_lock:
        cached = _composites.get(catalogue, {}).get(session_id)
    if cached is not None and cached[0] == cache_key:
        return cached[1]
    view = deployment_composite(session_id, members)
    with _catalogue_lock:
        _composites.setdefault(catalogue, {})[session_id] = (cache_key, view)
    return view


def deployment_view(
    catalogue: ComponentCatalogue,
    session_id: str,
    placements: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any] | None:
    """The composite view for one catalogue, or ``None`` below two members."""
    members = deployment_members(catalogue, session_id, placements)
    if len(members) < 2:
        return None
    return _cached_composite(catalogue, session_id, members)


def deployment_entry(
    catalogue: ComponentCatalogue,
    session_id: str,
    placements: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any] | None:
    """The composite's catalogue entry, in the shape of every other entry."""
    members = deployment_members(catalogue, session_id, placements)
    if len(members) < 2:
        return None
    entry = {
        "session_id": session_id,
        "component_id": deployment_component_id(session_id),
        "frame_id": DEPLOYMENT_FRAME,
        "robot_ids": [robot_id for robot_id, _, _ in members],
        "source_count": len(members),
        "point_count": 0,
        "submap_count": 0,
        "available": False,
        "status": "conflict",
        "detail": "",
        "solution_order": None,
        "solution_order_known": True,
        "sources": sorted(
            (source for _, _, view in members for source in view["sources"]),
            key=lambda source: source["robot_id"],
        ),
        "composite": True,
    }
    try:
        view = _cached_composite(catalogue, session_id, members)
    except (OverflowError, ValueError, KeyError, TypeError) as exc:
        entry["detail"] = str(exc)
        return entry
    entry.update(
        available=True,
        status="ready",
        sources=view["sources"],
        submap_count=len(view["selected"]["submaps"]),
        point_count=sum(
            chunk["point_count"]
            for submap in view["selected"]["submaps"]
            for chunk in submap["chunks"]
        ),
    )
    return entry


@router.get("/components")
async def component_catalogue(session_id: str | None = None):
    active_session_id = os.environ.get("SWARMDECK_MISSION_ID") or None
    # Registry and map service state belong to the event loop; snapshot the
    # placements here rather than in the worker thread.
    placements = (
        deployment_placements(active_session_id)
        if session_id in (None, active_session_id)
        else {}
    )
    try:

        def read():
            try:
                catalogue = current_catalogue(session_id)
            except OverflowError:
                # Preserve the complete historical catalogue while it fits its
                # input budget. Once it does not, the unscoped UI bootstrap can
                # still inspect the configured live mission without retiring
                # any historical replica manifests.
                if session_id is not None or active_session_id is None:
                    raise
                catalogue = current_catalogue(active_session_id)
            index = catalogue.index()
            if active_session_id is not None and placements:
                entry = deployment_entry(catalogue, active_session_id, placements)
                if entry is not None:
                    index["components"].append(entry)
            return {
                **index,
                # Deployment identity selects live data explicitly; UUID order
                # and independent per-robot revisions do not imply recency.
                "active_session_id": active_session_id,
            }

        return await asyncio.to_thread(read)
    except OverflowError as exc:
        return JSONResponse({"error": str(exc)}, status_code=413)
    except (KeyError, TypeError, ValueError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)


@router.get("/components/view/{session_id}")
async def component_view(session_id: str, component_id: str):
    try:
        if is_deployment_component(component_id):
            active_session_id = os.environ.get("SWARMDECK_MISSION_ID") or None
            if (
                session_id != active_session_id
                or component_id != deployment_component_id(session_id)
            ):
                raise KeyError("Replica component not found")
            placements = deployment_placements(session_id)

            def read_composite():
                view = deployment_view(
                    current_catalogue(session_id), session_id, placements
                )
                if view is None:
                    raise KeyError("Replica component not found")
                return view

            return await asyncio.to_thread(read_composite)

        def read():
            return current_catalogue(session_id).view(session_id, component_id)

        return await asyncio.to_thread(read)
    except KeyError as exc:
        return JSONResponse({"error": str(exc)}, status_code=404)
    except OverflowError as exc:
        return JSONResponse({"error": str(exc)}, status_code=413)
    except (LookupError, TypeError, ValueError) as exc:
        # The browser keeps its previous coherent publication while peers
        # converge on a common solution or replace their geometry.
        return JSONResponse({"error": str(exc)}, status_code=409)


def _component_id(manifest: Mapping[str, Any]) -> str:
    revision = manifest.get("graph_revision")
    if isinstance(revision, Mapping) and revision.get("component_id"):
        return str(revision["component_id"])
    return str(manifest.get("frame_id", ""))


def _submap_id(value: Any) -> str:
    if isinstance(value, Mapping):
        robot = value.get("robot_id", "")
        session = value.get("session_id", "")
        seq = value.get("seq", "")
        return f"{robot}/{session}/submap/{seq}"
    return str(value)


def build_view(
    envelope: Mapping[str, Any], *, component_id: str | None = None, tombstones=()
) -> dict[str, Any]:
    """Build a bounded, component-separated inspection payload.

    This pure function is intentionally usable by route tests and future
    transports. It preserves pose-only updates while exposing stable chunk
    hashes for browser-side caching.
    """

    snapshot = envelope.get("snapshot")
    if not isinstance(snapshot, Mapping):
        raise ValueError("replica snapshot is missing")
    manifests = snapshot.get("manifests")
    if not isinstance(manifests, list):
        raise ValueError("replica snapshot manifests are invalid")
    components: dict[str, dict[str, Any]] = {}
    retired = tuple(
        f"{robot}/{run}/"
        for robot, session, run in tombstones
        if session == envelope.get("session_id")
    )
    for manifest in manifests:
        if not isinstance(manifest, Mapping):
            raise ValueError("replica manifest is invalid")
        current = _component_id(manifest)
        if not current:
            continue
        component = components.setdefault(
            current,
            {
                "component_id": current,
                "frame_id": manifest.get("frame_id", current),
                "graph_revision": manifest.get("graph_revision"),
                "geometry_revision": manifest.get("geometry_revision"),
                "submaps": [],
                "tombstones": [],
            },
        )
        component["geometry_revision"] = manifest.get(
            "geometry_revision", component["geometry_revision"]
        )
        component["tombstones"].extend(manifest.get("tombstones", []))
        for submap in manifest.get("submaps", []):
            if not isinstance(submap, Mapping):
                raise ValueError("replica submap is invalid")
            if retired and _submap_id(submap.get("submap_id")).startswith(retired):
                continue
            transform = validate_se3(
                submap.get("T_component_submap"), "T_component_submap"
            )
            # Keep the transform exactly as supplied by the mapper. A selected
            # component is the only frame in which this view is rendered.
            component["submaps"].append(
                {
                    "submap_id": _submap_id(submap.get("submap_id")),
                    "geometry_revision": submap.get("geometry_revision"),
                    "pose_revision": submap.get("pose_revision"),
                    "T_component_submap": transform,
                    "bounds": submap.get("bounds"),
                    "resolution_m": submap.get("resolution_m"),
                    "replaces_geometry_revision": submap.get(
                        "replaces_geometry_revision"
                    ),
                    "observed_at_ns": submap.get("observed_at_ns", 0),
                    "sensor_origins": submap.get("sensor_origins", []),
                    "chunks": [
                        {
                            "sha256": chunk.get("sha256"),
                            "encoding": chunk.get("encoding"),
                            "size_bytes": chunk.get("size_bytes"),
                            "point_count": chunk.get("point_count"),
                            "bounds": chunk.get("bounds"),
                        }
                        for chunk in submap.get("chunks", [])
                        if isinstance(chunk, Mapping)
                    ],
                }
            )

    ordered = [components[key] for key in sorted(components)]
    selected = component_id
    if selected is None and len(ordered) == 1:
        selected = ordered[0]["component_id"]
    if selected is not None and selected not in components:
        raise KeyError(f"unknown replica component: {selected}")
    selected_components = [components[selected]] if selected else []
    chunks: dict[str, dict[str, Any]] = {}
    for component in selected_components:
        for submap in component["submaps"]:
            for chunk in submap["chunks"]:
                chunk_digest = chunk.get("sha256")
                if chunk_digest:
                    chunks.setdefault(chunk_digest, chunk)
    # ``generated_at_ns`` and ``observed_at_ns`` are mapper/ROS timestamps.
    # Treating either as wall time makes a simulation clock (or another host's
    # clock) look fresh or ancient.  Producers may opt in to a wall timestamp
    # when they can establish that clock domain; otherwise age is explicit.
    wall_generated = snapshot.get(
        "generated_at_wall_ns", envelope.get("generated_at_wall_ns")
    )
    age = (
        max(0.0, time.time() - int(wall_generated) / 1e9)
        if isinstance(wall_generated, (int, float)) and wall_generated > 0
        else None
    )
    return {
        "version": 1,
        "robot_id": envelope.get("robot_id"),
        "session_id": envelope.get("session_id"),
        "revision": envelope.get("revision"),
        "solution_order": envelope.get("solution_order"),
        "solution_order_known": (
            "solution_order" in envelope and envelope["solution_order"] is not None
        ),
        "snapshot_id": (
            digest([snapshot.get("snapshot_id"), sorted(retired)])
            if retired
            else snapshot.get("snapshot_id")
        ),
        "generated_at_ns": snapshot.get("generated_at_ns"),
        "component_id": selected,
        "components": ordered,
        "selected": selected_components[0] if selected_components else None,
        "chunks": list(chunks.values()),
        "source_age_s": age,
        "age_clock": "wall" if age is not None else "unknown",
        "geometry_encodings": [
            "application/vnd.swarmdeck.xyz-f32.v1",
            "application/vnd.swarmdeck.xyzrgba-f32-u8.v1",
        ],
        "reconstruction": envelope.get(
            "reconstruction", snapshot.get("reconstruction")
        ),
    }


@router.get("/view/{robot_id}/{session_id}")
async def replica_view(robot_id: str, session_id: str, component_id: str | None = None):
    try:
        envelope = await asyncio.to_thread(store().get, robot_id, session_id)
        if envelope is None:
            return JSONResponse({"error": "replica not found"}, status_code=404)
        return build_view(
            envelope,
            component_id=component_id,
            tombstones=await asyncio.to_thread(store().tombstones, session_id),
        )
    except KeyError as exc:
        return JSONResponse({"error": str(exc)}, status_code=404)
    except (TypeError, ValueError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
