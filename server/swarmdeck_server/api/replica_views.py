"""Read-only inspection views over durable onboard map replicas.

This endpoint returns manifest metadata and immutable chunk references. The UI
chooses one component and applies each submap's ``T_component_submap`` locally;
the server never invents a transform between disconnected components.
"""

from __future__ import annotations

import asyncio
import os
from collections import OrderedDict
from threading import Lock
from typing import Any, Mapping
import time
from weakref import WeakKeyDictionary

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from autonomy.contracts import validate_se3
from .autonomy_routes import store
from .replica_components import ComponentCatalogue, MAX_ENVELOPES, _normalize_envelope
from .replica_live import router as live_router

router = APIRouter(prefix="/api/autonomy/replicas", tags=["autonomy replica views"])
router.include_router(live_router)
_catalogues = WeakKeyDictionary()
_catalogue_lock = Lock()


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
        previous = state.catalogues.get(session_id)
        if previous is not None and previous[0] == versions:
            state.catalogues.move_to_end(session_id)
            return previous[1]

        snapshots = replica_store.snapshots(session_id)
        # A publication may have advanced during the first version read. Associate
        # the cache with the actual coherent snapshot used to construct the view.
        versions = tuple(
            (e["robot_id"], e["session_id"], e["revision"]) for e in snapshots
        )

        def normalize(envelope):
            owner = (envelope["session_id"], envelope["robot_id"])
            revision = envelope["revision"]
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

        catalogue = ComponentCatalogue(snapshots, normalizer=normalize)
        state.catalogues[session_id] = (versions, catalogue)
        state.catalogues.move_to_end(session_id)
        while len(state.catalogues) > 2:
            state.catalogues.popitem(last=False)
        retained_revisions = {
            ((source_session, robot_id), revision)
            for cached_versions, _ in state.catalogues.values()
            for robot_id, source_session, revision in cached_versions
        }
        for owner, cached in tuple(state.normalized.items()):
            if (owner, cached[0]) not in retained_revisions:
                del state.normalized[owner]
        return catalogue


@router.get("/components")
async def component_catalogue(session_id: str | None = None):
    try:

        def read():
            active_session_id = os.environ.get("SWARMDECK_MISSION_ID") or None
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
            return {
                **catalogue.index(),
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
    envelope: Mapping[str, Any], *, component_id: str | None = None
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
                digest = chunk.get("sha256")
                if digest:
                    chunks.setdefault(digest, chunk)
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
        "snapshot_id": snapshot.get("snapshot_id"),
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
        return build_view(envelope, component_id=component_id)
    except KeyError as exc:
        return JSONResponse({"error": str(exc)}, status_code=404)
    except (TypeError, ValueError) as exc:
        return JSONResponse({"error": str(exc)}, status_code=400)
