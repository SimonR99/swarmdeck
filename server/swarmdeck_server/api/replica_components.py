"""Fleet display assembly from compatible, already accepted peer results.

This module never registers maps or estimates transforms. Component IDs identify
frames; a common Swarm-SLAM solution order identifies the corrected snapshot.
Local graph epochs/revisions are deliberately not treated as fleet counters.
"""

from __future__ import annotations

from collections import defaultdict
import hashlib
from typing import Any

from autonomy.contracts import (
    ChunkRef,
    ComponentRevision,
    KeyframeId,
    MapManifest,
    MapSnapshot,
    SubmapId,
    SubmapRevision,
)
from autonomy.replication import MAX_CHUNK_BYTES, canonical, chunk_hash, identity

MAX_ENVELOPES = 128
MAX_SUBMAPS = 16_384
MAX_SOURCE_SUBMAPS = 65_536
MAX_CHUNKS = 4_096
XYZ_ENCODING = "application/vnd.swarmdeck.xyz-f32.v1"
XYZRGBA_ENCODING = "application/vnd.swarmdeck.xyzrgba-f32-u8.v1"


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


def uint(value: Any, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"Invalid {field}")
    return value


def solution_order(envelope: dict) -> tuple[int, int] | None:
    value = envelope.get("solution_order")
    if value is None or value == [0, -1]:
        return None
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("Invalid accepted solution order")
    return (uint(value[0], "solution clock"), uint(value[1], "optimizer ID"))


def _revision(value: Any) -> ComponentRevision:
    if not isinstance(value, dict):
        raise ValueError("Invalid component graph revision")
    try:
        revision = ComponentRevision(
            value["component_id"], value["epoch"], value["revision"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid component graph revision") from exc
    if value != {
        "component_id": revision.component_id,
        "epoch": revision.epoch,
        "revision": revision.revision,
    }:
        raise ValueError("Invalid component graph revision")
    return revision


def _chunk(value: Any) -> ChunkRef:
    if not isinstance(value, dict):
        raise ValueError("Invalid geometry chunk")
    try:
        chunk = ChunkRef(
            sha256=value["sha256"],
            encoding=value["encoding"],
            size_bytes=value["size_bytes"],
            bounds=value["bounds"],
            point_count=value["point_count"],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid geometry chunk") from exc
    if (
        chunk.encoding not in {XYZ_ENCODING, XYZRGBA_ENCODING}
        or chunk.size_bytes
        != 16 + (12 if chunk.encoding == XYZ_ENCODING else 16) * chunk.point_count
    ):
        raise ValueError("Invalid geometry chunk")
    return chunk


def _submap(value: Any) -> SubmapRevision:
    if not isinstance(value, dict):
        raise ValueError("Invalid submap revision")
    try:
        chunks = tuple(_chunk(item) for item in value["chunks"])
        if len({chunk.sha256 for chunk in chunks}) != len(chunks):
            raise ValueError("Submap repeats a geometry chunk")
        submap = SubmapRevision(
            submap_id=SubmapId.from_dict(value["submap_id"]),
            geometry_revision=value["geometry_revision"],
            pose_revision=_revision(value["pose_revision"]),
            T_component_submap=value["T_component_submap"],
            keyframes=tuple(KeyframeId.from_dict(item) for item in value["keyframes"]),
            chunks=chunks,
            bounds=value["bounds"],
            resolution_m=value["resolution_m"],
            replaces_geometry_revision=value.get("replaces_geometry_revision"),
            observed_at_ns=value.get("observed_at_ns", 0),
            sensor_origins=tuple(value.get("sensor_origins", ())),
            ray_evidence=value.get("ray_evidence", {}),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid submap revision") from exc
    if len(set(submap.keyframes)) != len(submap.keyframes):
        raise ValueError("Submap repeats a keyframe")
    return submap


def _geometry_digest(submaps: tuple[SubmapRevision, ...]) -> str:
    content = [
        (
            submap.submap_id.stable_id,
            submap.geometry_revision,
            [chunk.sha256 for chunk in submap.chunks],
        )
        for submap in sorted(submaps, key=lambda item: item.submap_id.stable_id)
    ]
    return digest(content)


def _manifest(value: Any) -> MapManifest:
    if not isinstance(value, dict):
        raise ValueError("Invalid map manifest")
    try:
        submaps = tuple(_submap(item) for item in value["submaps"])
        chunks = tuple(_chunk(item) for item in value["chunks"])
        tombstones = tuple(value["tombstones"])
        if any(not isinstance(item, str) or not item for item in tombstones):
            raise ValueError("Invalid tombstone")
        manifest = MapManifest(
            map_id=value["map_id"],
            layer_id=value["layer_id"],
            frame_id=value["frame_id"],
            graph_revision=_revision(value["graph_revision"]),
            geometry_revision=value["geometry_revision"],
            submaps=submaps,
            chunks=chunks,
            tombstones=tombstones,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid map manifest") from exc
    encoded = manifest.to_dict()
    encoded.pop("schema")
    if encoded != value:
        raise ValueError("Map manifest differs from the autonomy schema")
    if manifest.geometry_revision != _geometry_digest(manifest.submaps):
        raise ValueError("Manifest geometry revision does not match its submaps")
    return manifest


def _tombstone(value: str) -> tuple[str, str]:
    stable_id, separator, reason = value.rpartition(":")
    parts = stable_id.split("/")
    if not separator or not reason or len(parts) != 4 or parts[2] != "submap":
        raise ValueError("Invalid submap tombstone")
    try:
        key = SubmapId(parts[0], parts[1], int(parts[3]))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid submap tombstone") from exc
    if key.stable_id != stable_id:
        raise ValueError("Invalid submap tombstone")
    return stable_id, key.robot_id


def _chunk_value(chunk: ChunkRef) -> dict:
    return {
        "sha256": chunk.sha256,
        "encoding": chunk.encoding,
        "size_bytes": chunk.size_bytes,
        "point_count": chunk.point_count,
        "bounds": chunk.bounds,
    }


def _submap_value(submap: SubmapRevision, declared: dict[str, int]) -> dict:
    if any(declared.get(chunk.sha256) != chunk.size_bytes for chunk in submap.chunks):
        raise ValueError("Submap chunk is not in the committed geometry publication")
    return {
        "submap_id": submap.submap_id.stable_id,
        "geometry_revision": submap.geometry_revision,
        "pose_revision": {
            "component_id": submap.pose_revision.component_id,
            "epoch": submap.pose_revision.epoch,
            "revision": submap.pose_revision.revision,
        },
        "T_component_submap": submap.T_component_submap,
        "bounds": submap.bounds,
        "resolution_m": submap.resolution_m,
        "replaces_geometry_revision": submap.replaces_geometry_revision,
        "observed_at_ns": submap.observed_at_ns,
        "chunks": [_chunk_value(chunk) for chunk in submap.chunks],
    }


def geometry_identity(submap: dict) -> dict:
    return {
        key: value
        for key, value in submap.items()
        if key not in {"pose_revision", "T_component_submap"}
    }


def _publications(sources: list[dict]) -> list[dict]:
    publications = [
        {
            "robot_id": source["robot_id"],
            "session_id": source["session_id"],
            "revision": source["revision"],
            "snapshot_id": source["snapshot_id"],
        }
        for source in sources
    ]
    publications.sort(key=lambda source: source["robot_id"])
    return publications


def _normalize_envelope(envelope: dict) -> dict:
    if not isinstance(envelope, dict) or envelope.get("version") != 1:
        raise ValueError("Invalid replica envelope")
    robot_id, session_id = envelope["robot_id"], envelope["session_id"]
    identity(robot_id, session_id)
    revision = uint(envelope["revision"], "replica revision")
    order_known = (
        "solution_order" in envelope and envelope["solution_order"] is not None
    )
    order = solution_order(envelope)
    raw_declared = envelope.get("chunks")
    if not isinstance(raw_declared, list) or len(raw_declared) > MAX_CHUNKS:
        raise ValueError("Invalid replica chunk declarations")
    declared: dict[str, int] = {}
    for item in raw_declared:
        if not isinstance(item, dict) or set(item) != {"sha256", "size"}:
            raise ValueError("Invalid replica chunk declaration")
        chunk_hash(item["sha256"])
        size = uint(item["size"], "replica chunk size")
        if size > MAX_CHUNK_BYTES:
            raise ValueError("Replica chunk exceeds the storage limit")
        if item["sha256"] in declared:
            raise ValueError("Repeated replica chunk declaration")
        declared[item["sha256"]] = size

    snapshot_value = envelope.get("snapshot")
    if not isinstance(snapshot_value, dict):
        raise ValueError("Invalid replica snapshot")
    raw_manifests = snapshot_value.get("manifests")
    if not isinstance(raw_manifests, list):
        raise ValueError("Invalid replica snapshot manifests")
    manifests = tuple(_manifest(value) for value in raw_manifests)
    try:
        snapshot = MapSnapshot(
            snapshot_value["snapshot_id"],
            snapshot_value["generated_at_ns"],
            manifests,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid replica snapshot") from exc
    if snapshot.to_dict() != snapshot_value:
        raise ValueError("Replica snapshot differs from the autonomy schema")
    manifest_descriptors = {}
    for manifest in manifests:
        for chunk in manifest.chunks:
            descriptor = _chunk_value(chunk)
            previous = manifest_descriptors.get(chunk.sha256)
            if previous is not None and previous != descriptor:
                raise ValueError("One chunk hash has conflicting descriptors")
            manifest_descriptors[chunk.sha256] = descriptor
    manifest_chunks = {
        name: descriptor["size_bytes"]
        for name, descriptor in manifest_descriptors.items()
    }
    if manifest_chunks != declared:
        raise ValueError("Replica chunk declarations do not exactly cover the snapshot")

    grouped: dict[str, list[MapManifest]] = defaultdict(list)
    for manifest in manifests:
        grouped[manifest.graph_revision.component_id].append(manifest)
    components = {}
    source_submaps = 0
    for component_id, component_manifests in grouped.items():
        revisions = [manifest.graph_revision for manifest in component_manifests]
        if len(set(revisions)) != len(revisions):
            raise ValueError("Repeated component graph revision")
        head = max(revisions)
        frames = {manifest.frame_id for manifest in component_manifests}
        if len(frames) != 1:
            raise ValueError("One publisher disagrees on the component frame")
        if any(
            manifest.submaps and manifest.graph_revision != head
            for manifest in component_manifests
        ):
            raise ValueError("Historical component manifest contains active submaps")
        submaps = tuple(
            submap for manifest in component_manifests for submap in manifest.submaps
        )
        source_submaps += len(submaps)
        if len(submaps) > MAX_SUBMAPS or source_submaps > MAX_SOURCE_SUBMAPS:
            raise ValueError("Source submap budget exceeded")
        values = tuple(
            (submap.submap_id.robot_id, _submap_value(submap, declared))
            for submap in submaps
        )
        if len({value[1]["submap_id"] for value in values}) != len(values):
            raise ValueError("Publisher repeated an active submap")
        tombstones = []
        tombstoned_ids = set()
        for manifest in component_manifests:
            for text in manifest.tombstones:
                stable_id, owner = _tombstone(text)
                if stable_id in tombstoned_ids:
                    raise ValueError("Publisher repeated a submap tombstone")
                tombstones.append((robot_id, owner, stable_id, text))
                tombstoned_ids.add(stable_id)
        if len(tombstones) > MAX_SUBMAPS:
            raise ValueError("Source tombstone budget exceeded")
        if tombstoned_ids & {value[1]["submap_id"] for value in values}:
            raise ValueError("Publisher marks one submap active and tombstoned")
        components[component_id] = {
            "frame_id": next(iter(frames)),
            "submaps": values,
            "tombstones": tuple(tombstones),
        }
    return {
        "robot_id": robot_id,
        "session_id": session_id,
        "revision": revision,
        "snapshot_id": snapshot.snapshot_id,
        "generated_at_ns": snapshot.generated_at_ns,
        "solution_order": order,
        "solution_order_known": order_known,
        "components": components,
    }


class ComponentCatalogue:
    """One immutable database snapshot, with lazy component view assembly."""

    def __init__(self, envelopes: list[dict], *, normalizer=_normalize_envelope):
        if not isinstance(envelopes, list) or len(envelopes) > MAX_ENVELOPES:
            raise ValueError("Replica source budget exceeded")
        self.groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
        self.owners: set[tuple[str, str]] = set()
        self.invalid_owners: set[tuple[str, str]] = set()
        self.source_errors: list[dict] = []
        self.direct: dict[tuple[str, str], tuple[str, dict]] = {}
        self._views: dict[tuple[str, str], dict] = {}

        for envelope in envelopes:
            if not isinstance(envelope, dict):
                raise ValueError("Invalid replica envelope")
            try:
                robot_id, session_id = envelope["robot_id"], envelope["session_id"]
                identity(robot_id, session_id)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Invalid replica owner identity") from exc
            owner = (session_id, robot_id)
            if owner in self.owners:
                raise ValueError("Repeated replica owner")
            self.owners.add(owner)
            try:
                source = normalizer(envelope)
            except (KeyError, TypeError, ValueError) as exc:
                self.invalid_owners.add(owner)
                self.source_errors.append(
                    {"session_id": session_id, "robot_id": robot_id, "detail": str(exc)}
                )
                continue
            for component_id, component in source["components"].items():
                key = (session_id, component_id)
                self.groups[key].append(source)
                for submap_owner, value in component["submaps"]:
                    if submap_owner != robot_id:
                        continue
                    direct_key = (session_id, value["submap_id"])
                    if direct_key in self.direct:
                        raise ValueError("Owner repeated an active submap")
                    self.direct[direct_key] = (component_id, value)

    def _assemble(self, key: tuple[str, str]) -> dict:
        if key in self._views:
            return self._views[key]
        sources = self.groups[key]
        frames = {source["components"][key[1]]["frame_id"] for source in sources}
        if len(frames) != 1:
            raise ValueError("Component publishers disagree on the coordinate frame")
        orders = {source["solution_order"] for source in sources}
        order_known = all(source["solution_order_known"] for source in sources)
        if len(sources) > 1 and (not order_known or None in orders or len(orders) != 1):
            raise LookupError("Waiting for a common accepted Swarm-SLAM solution")
        order = next(iter(orders))
        source_count = sum(
            len(source["components"][key[1]]["submaps"])
            + len(source["components"][key[1]]["tombstones"])
            for source in sources
        )
        if source_count > MAX_SOURCE_SUBMAPS:
            raise ValueError("Component source record budget exceeded")

        active: dict[str, dict] = {}
        tombstone_candidates = []
        for source in sources:
            component = source["components"][key[1]]
            tombstone_candidates.extend(component["tombstones"])
            for owner, candidate in component["submaps"]:
                submap_id = candidate["submap_id"]
                owner_key = (key[0], owner)
                if owner_key in self.invalid_owners:
                    raise ValueError("Submap owner's direct publication is invalid")
                if owner_key in self.owners:
                    direct = self.direct.get((key[0], submap_id))
                    if direct is None or direct[0] != key[1]:
                        continue
                    candidate = direct[1]
                previous = active.get(submap_id)
                if previous is not None and previous != candidate:
                    raise ValueError(
                        "Relayed submap copies disagree on geometry or pose"
                    )
                active[submap_id] = candidate
                if len(active) > MAX_SUBMAPS:
                    raise ValueError("Component submap budget exceeded")

        tombstones: dict[str, str] = {}
        for publisher, owner, submap_id, text in tombstone_candidates:
            owner_key = (key[0], owner)
            if owner_key in self.invalid_owners:
                raise ValueError("Tombstone owner's direct publication is invalid")
            if owner_key in self.owners:
                if publisher != owner:
                    continue
                if (key[0], submap_id) in self.direct:
                    raise ValueError("Owner marks one submap active and tombstoned")
            elif submap_id in active:
                raise ValueError("Relayed active submap conflicts with a tombstone")
            previous = tombstones.get(submap_id)
            if previous is not None and previous != text:
                raise ValueError("Relayed tombstones disagree")
            tombstones[submap_id] = text

        submaps = [active[name] for name in sorted(active)]
        geometry_revision = digest([geometry_identity(value) for value in submaps])
        publications = _publications(sources)
        component = {
            "component_id": key[1],
            "frame_id": next(iter(frames)),
            "graph_revision": None,
            "geometry_revision": geometry_revision,
            "submaps": submaps,
            "tombstones": [tombstones[name] for name in sorted(tombstones)],
        }
        chunks: dict[str, dict] = {}
        for submap in submaps:
            for chunk in submap["chunks"]:
                previous = chunks.get(chunk["sha256"])
                if previous is not None and previous != chunk:
                    raise ValueError("One chunk hash has conflicting descriptors")
                chunks[chunk["sha256"]] = chunk
        view = {
            "version": 1,
            "scope": "fleet",
            "robot_id": "fleet",
            "session_id": key[0],
            "component_id": key[1],
            "revision": None,
            "snapshot_id": digest([key, publications, order, component]),
            "generated_at_ns": max(source["generated_at_ns"] for source in sources),
            "solution_order": order,
            "solution_order_known": order_known,
            "components": [component],
            "selected": component,
            "chunks": [chunks[name] for name in sorted(chunks)],
            "source_age_s": None,
            "age_clock": "unknown",
            "sources": publications,
            "geometry_encodings": [XYZ_ENCODING, XYZRGBA_ENCODING],
            "reconstruction": None,
        }
        self._views[key] = view
        return view

    def index(self) -> dict:
        entries = []
        for key, sources in sorted(self.groups.items()):
            entry = {
                "session_id": key[0],
                "component_id": key[1],
                "frame_id": sources[0]["components"][key[1]]["frame_id"],
                "robot_ids": sorted(source["robot_id"] for source in sources),
                "source_count": len(sources),
                "point_count": 0,
                "submap_count": 0,
                "available": False,
                "status": "conflict",
                "detail": "",
                "solution_order": None,
                "solution_order_known": False,
                "sources": _publications(sources),
            }
            try:
                view = self._assemble(key)
                entry.update(
                    available=True,
                    status="ready",
                    sources=view["sources"],
                    solution_order=view["solution_order"],
                    solution_order_known=view["solution_order_known"],
                    submap_count=len(view["selected"]["submaps"]),
                    point_count=sum(
                        chunk["point_count"]
                        for submap in view["selected"]["submaps"]
                        for chunk in submap["chunks"]
                    ),
                )
            except LookupError as exc:
                entry.update(status="syncing", detail=str(exc))
            except (ValueError, KeyError, TypeError) as exc:
                entry["detail"] = str(exc)
            entries.append(entry)
        return {
            "version": 1,
            "components": entries,
            "source_errors": self.source_errors,
        }

    def view(self, session_id: str, component_id: str) -> dict:
        key = (session_id, component_id)
        if key not in self.groups:
            raise KeyError("Replica component not found")
        return self._assemble(key)
