"""Read-only inspection views over durable onboard map replicas.

This endpoint returns manifest metadata and immutable chunk references. The UI
chooses one component and applies each submap's ``T_component_submap`` locally;
the server never invents a transform between disconnected components.
"""

from __future__ import annotations

import asyncio
from typing import Any, Mapping
import time

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from autonomy.contracts import validate_se3
from .autonomy_routes import store


router = APIRouter(prefix="/api/autonomy/replicas", tags=["autonomy replica views"])


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
    wall_generated = snapshot.get("generated_at_wall_ns", envelope.get("generated_at_wall_ns"))
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
        "snapshot_id": snapshot.get("snapshot_id"),
        "generated_at_ns": snapshot.get("generated_at_ns"),
        "component_id": selected,
        "components": ordered,
        "selected": selected_components[0] if selected_components else None,
        "chunks": list(chunks.values()),
        "source_age_s": age,
        "age_clock": "wall" if age is not None else "unknown",
        "geometry_encoding": "application/vnd.swarmdeck.xyz-f32.v1",
        "reconstruction": envelope.get("reconstruction", snapshot.get("reconstruction")),
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
