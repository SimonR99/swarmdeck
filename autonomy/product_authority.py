"""Product-gated map authority: advertise only a published MOLA product.

The bridge's map authority is the key every indexed-map consumer (the MGG
loader, the index server, the adapter) must find on disk as
``<peer>/mola/index.json``. The MOLA worker builds a product a few seconds
behind the pose graph, so an authority taken from the current revision has no
product while the robot drives. This module reads the newest product the
worker has published and pairs it with the frame state that was in effect at
the revision it was built from (``CslamMapper.frame_history``), so the
advertised key always has a product and its geometry is placed with the
correction it was built under.

Publication protocol: the worker writes ``mola/source.json`` (the exact bytes
of the ``snapshot.json`` it built from) and then ``mola/index.json`` whose
``source_sha256`` and ``source_snapshot_id`` name those bytes. A reader reads
the index, then the source, and treats a mismatch as a pair caught between two
publications: retry a few times, then give up for this tick. Readers never
read ``snapshot.json``.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import time
from typing import TYPE_CHECKING, Callable, Mapping, Sequence

import numpy as np

from .contracts import SCHEMA_VERSION, validate_se3

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .cslam import FrameState

MAX_PRODUCT_BYTES = 4 * 1024 * 1024
MAX_PRODUCT_COMPONENTS = 256
DEFAULT_READ_ATTEMPTS = 3
READ_RETRY_PAUSE_S = 0.005
_SHA_RE = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True)
class ProductArtifact:
    """One component of a published product and the source it was built from."""

    component_id: str
    epoch: int
    revision: int
    geometry_revision: str
    manifest_sha256: str
    source_stamp_ns: int


@dataclass(frozen=True)
class PublishedProduct:
    """A coherent ``index.json`` and ``source.json`` pair."""

    snapshot_id: str
    generated_at_ns: int
    artifacts: tuple[ProductArtifact, ...]


def _uint(value: object, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _sha(value: object, field: str) -> str:
    if not isinstance(value, str) or not _SHA_RE.fullmatch(value):
        raise ValueError(f"{field} must be lowercase SHA-256")
    return value


def _text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError(f"{field} must be a nonempty bounded string")
    return value


def _object(value: object, field: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _bounded_bytes(path: Path, maximum: int) -> bytes | None:
    """Return the file's bytes, or None when absent, oversized or unreadable."""

    try:
        if path.stat().st_size > maximum:
            return None
        with path.open("rb") as stream:
            raw = stream.read(maximum + 1)
    except OSError:
        return None
    if len(raw) > maximum:
        return None
    return raw


def _json(raw: bytes, field: str) -> dict[str, object]:
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field} is not valid JSON") from exc
    return _object(value, field)


def _source_manifests(source: Mapping[str, object]) -> dict[str, dict[str, object]]:
    if source.get("schema") != SCHEMA_VERSION:
        raise ValueError("unsupported source snapshot schema")
    manifests = source.get("manifests")
    if not isinstance(manifests, list) or len(manifests) > MAX_PRODUCT_COMPONENTS:
        raise ValueError("source manifests must be a bounded list")
    by_component: dict[str, dict[str, object]] = {}
    for raw_manifest in manifests:
        manifest = _object(raw_manifest, "manifest")
        revision = _object(manifest.get("graph_revision"), "graph_revision")
        component = _text(revision.get("component_id"), "component_id")
        if component in by_component:
            raise ValueError("source repeats a component_id")
        _uint(revision.get("epoch"), "manifest epoch")
        _uint(revision.get("revision"), "manifest revision")
        _sha(manifest.get("geometry_revision"), "manifest geometry_revision")
        submaps = manifest.get("submaps")
        if not isinstance(submaps, list):
            raise ValueError("manifest submaps must be a list")
        for raw_submap in submaps:
            submap = _object(raw_submap, "submap")
            _uint(submap.get("observed_at_ns", 0), "submap observed_at_ns")
        by_component[component] = manifest
    return by_component


def _source_stamp_ns(manifest: Mapping[str, object]) -> int:
    submaps = manifest.get("submaps")
    assert isinstance(submaps, list)  # narrowed by _source_manifests
    return max((int(submap.get("observed_at_ns", 0)) for submap in submaps), default=0)


def _artifacts(
    index: Mapping[str, object], manifests: Mapping[str, Mapping[str, object]]
) -> tuple[ProductArtifact, ...]:
    raw_artifacts = index.get("artifacts")
    if (
        not isinstance(raw_artifacts, list)
        or len(raw_artifacts) > MAX_PRODUCT_COMPONENTS
    ):
        raise ValueError("index artifacts must be a bounded list")
    artifacts: list[ProductArtifact] = []
    seen: set[str] = set()
    for raw_item in raw_artifacts:
        item = _object(raw_item, "artifact")
        component = _text(item.get("component_id"), "artifact component_id")
        if component in seen:
            raise ValueError("index repeats a component_id")
        seen.add(component)
        manifest = manifests.get(component)
        if manifest is None:
            raise ValueError("artifact names a component absent from its source")
        revision = _object(manifest.get("graph_revision"), "graph_revision")
        artifact = ProductArtifact(
            component,
            _uint(item.get("epoch"), "artifact epoch"),
            _uint(item.get("revision"), "artifact revision"),
            _sha(item.get("geometry_revision"), "artifact geometry_revision"),
            _sha(item.get("manifest_sha256"), "artifact manifest_sha256"),
            _source_stamp_ns(manifest),
        )
        if (
            artifact.epoch != revision["epoch"]
            or artifact.revision != revision["revision"]
            or artifact.geometry_revision != manifest["geometry_revision"]
        ):
            raise ValueError("artifact does not match its source manifest")
        artifacts.append(artifact)
    if seen != set(manifests):
        raise ValueError("index does not cover the source components")
    return tuple(artifacts)


def _file_identity(path: Path) -> tuple[int, int, int, int] | None:
    """Fields an atomic replacement changes, or None when the file is absent."""

    try:
        value = path.stat()
    except OSError:
        return None
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)


def read_published_product(
    peer_root: str | Path,
    *,
    max_bytes: int = MAX_PRODUCT_BYTES,
    attempts: int = DEFAULT_READ_ATTEMPTS,
    retry_pause_s: float = READ_RETRY_PAUSE_S,
    sleep: Callable[[float], object] = time.sleep,
    memo: dict[str, object] | None = None,
) -> PublishedProduct | None:
    """Return the product the worker has published for ``peer_root``, or None.

    None means there is nothing to advertise this tick: a file is absent,
    oversized or invalid, or the index and source pair stayed incoherent for
    ``attempts`` reads (a publication in progress).

    ``memo`` is an optional caller-owned dict. The result is remembered under
    the file identities it was read from, so a caller polling every second
    does not parse an unchanged multi-megabyte source again; the worker
    replaces both files atomically, which changes their identity.
    """

    mola_root = Path(peer_root) / "mola"
    index_path, source_path = mola_root / "index.json", mola_root / "source.json"
    identity = (_file_identity(index_path), _file_identity(source_path))
    if memo is not None and None not in identity and memo.get("identity") == identity:
        return memo.get("product")  # type: ignore[return-value]
    product = _read_published_product(
        index_path, source_path, max_bytes, attempts, retry_pause_s, sleep
    )
    if memo is not None:
        if None not in identity and identity == (
            _file_identity(index_path),
            _file_identity(source_path),
        ):
            memo["identity"], memo["product"] = identity, product
        else:
            memo.clear()
    return product


def _read_published_product(
    index_path: Path,
    source_path: Path,
    max_bytes: int,
    attempts: int,
    retry_pause_s: float,
    sleep: Callable[[float], object],
) -> PublishedProduct | None:
    for attempt in range(max(1, int(attempts))):
        index_raw = _bounded_bytes(index_path, max_bytes)
        if index_raw is None:
            return None
        source_raw = _bounded_bytes(source_path, max_bytes)
        if source_raw is None:
            return None
        try:
            index = _json(index_raw, "index")
            if index.get("version") != 1:
                raise ValueError("unsupported index version")
            source_sha = _sha(index.get("source_sha256"), "index source_sha256")
            snapshot_id = _sha(index.get("source_snapshot_id"), "index snapshot id")
            generated_at_ns = _uint(index.get("generated_at_ns"), "generated_at_ns")
            source = _json(source_raw, "source")
        except ValueError:
            return None
        if source_sha != hashlib.sha256(
            source_raw
        ).hexdigest() or snapshot_id != source.get("snapshot_id"):
            # The pair was caught between two publications: the worker
            # replaces the source first and the index right after it.
            if attempt + 1 < attempts:
                sleep(retry_pause_s)
            continue
        try:
            manifests = _source_manifests(source)
            artifacts = _artifacts(index, manifests)
        except ValueError:
            return None
        return PublishedProduct(snapshot_id, generated_at_ns, artifacts)
    return None


def product_authority_key(
    core, product: PublishedProduct | None
) -> tuple[ProductArtifact, FrameState] | None:
    """Pair the newest advertisable artifact with the frame it was built under.

    An artifact is advertisable when its revision is not ahead of the core and
    the core still remembers that revision's frame for the same component and
    epoch. Anything else (no product, no history, a product from a revision the
    core has not applied, a component the frame at that revision did not have)
    leaves the authority unadvertised rather than pairing geometry with a
    frame it was not placed in.
    """

    if product is None:
        return None
    history = getattr(core, "frame_history", None)
    if not history:
        return None
    revision = int(getattr(core, "revision", 0))
    selected: tuple[ProductArtifact, FrameState] | None = None
    for artifact in product.artifacts:
        if artifact.revision > revision:
            continue
        frame = history.get(artifact.revision)
        if (
            frame is None
            or frame.component_id != artifact.component_id
            or frame.epoch != artifact.epoch
        ):
            continue
        if selected is None or artifact.revision > selected[0].revision:
            selected = (artifact, frame)
    return selected


def build_authority(
    artifact: ProductArtifact,
    frame: FrameState,
    *,
    robot_id: str,
    mission_id: str,
    participants: Sequence[str],
    navigation_frame: str,
    planning_frame: str,
    T_local_navigation,
    home_keyframe_id: str,
    peer_slam: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Build the authority message for one product and its frame state."""

    T_component_local = np.asarray(frame.T_component_local, dtype=np.float64)
    local_navigation = np.asarray(
        validate_se3(T_local_navigation, "T_local_navigation"), dtype=np.float64
    )
    T_component_navigation = T_component_local @ local_navigation
    stamp_ns = int(artifact.source_stamp_ns)
    authority: dict[str, object] = {
        "robot_id": robot_id,
        "mission_id": mission_id,
        "participants": list(participants),
        "component_id": artifact.component_id,
        "solution_order": list(frame.solution_order),
        "correction_revision": int(frame.correction_revision),
        "map_epoch": int(artifact.epoch),
        "mapping_graph_revision": int(artifact.revision),
        "geometry_revision": artifact.geometry_revision,
        "map_source_stamp": {
            "sec": stamp_ns // 1_000_000_000,
            "nanosec": stamp_ns % 1_000_000_000,
        },
        "navigation_frame": navigation_frame,
        "T_component_navigation": T_component_navigation.tolist(),
        # MGG may use continuous odometry while the UI remains in the SLAM
        # navigation frame. Publish both pairs from one frame state so no
        # consumer composes different revisions.
        "planning_frame": planning_frame,
        "T_component_planning": T_component_local.tolist(),
    }
    if peer_slam is not None:
        authority["peer_slam"] = dict(peer_slam)
    if frame.T_component_home is not None:
        T_navigation_component = np.linalg.inv(T_component_navigation)
        authority["home"] = {
            "keyframe_id": home_keyframe_id,
            "T_navigation_home": (
                T_navigation_component
                @ np.asarray(frame.T_component_home, dtype=np.float64)
            ).tolist(),
        }
    return authority
