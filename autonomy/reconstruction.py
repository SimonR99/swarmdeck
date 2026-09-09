"""Durable, backend-neutral jobs for optional Gaussian reconstruction.

The runner deliberately owns job identity, input snapshots, cancellation, and
publication.  A backend only prepares a dataset, runs a bounded command, and
returns a staged artifact.  This keeps a private trainer (such as UMAMI) out of
the autonomy and server processes and prevents a late job from replacing a
newer pose revision.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
from enum import Enum
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from contextlib import contextmanager
from typing import Any, Callable, Iterable, Mapping, Sequence

from .contracts import ComponentRevision, GraphSolution, KeyframeId, validate_se3

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts" / "reconstruction"
DEFAULT_UMAMI_SCRIPT = SCRIPT_DIR / "umami.py"


class ReconstructionError(RuntimeError):
    """Base class for deterministic job failures."""


class InputUnavailable(ReconstructionError):
    pass


class ResourceBudgetError(ReconstructionError):
    pass


class JobCanceled(ReconstructionError):
    pass


class StaleResult(ReconstructionError):
    pass


class JobState(str, Enum):
    QUEUED = "queued"
    WAITING_FOR_INPUT = "waiting-for-input"
    TRAINING = "training"
    READY = "ready"
    STALE = "stale"
    CANCELED = "canceled"
    FAILED = "failed"


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Enum):
        return value.value
    raise TypeError(f"cannot encode {type(value).__name__}")


def _canonical(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        default=_json_default,
        allow_nan=False,
    )


def _digest(value: Any) -> str:
    return hashlib.sha256(_canonical(value).encode("utf-8")).hexdigest()


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        json.dump(value, stream, indent=2, sort_keys=True, default=_json_default)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _atomic_bytes(path: Path, value: bytes) -> None:
    """Install immutable job input bytes without exposing a partial file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("wb", dir=path.parent, delete=False) as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _safe_rmtree(path: Path, jobs_root: Path) -> None:
    """Remove a job-owned work directory after checking its scope."""
    resolved = path.resolve()
    jobs_root = jobs_root.resolve()
    try:
        if os.path.commonpath((str(resolved), str(jobs_root))) != str(jobs_root):
            raise ReconstructionError(
                f"refusing to remove path outside jobs root: {path}"
            )
    except ValueError as exc:
        raise ReconstructionError(f"invalid work path: {path}") from exc
    if resolved.exists() and resolved.is_dir():
        shutil.rmtree(resolved)


@dataclass(frozen=True)
class PoseRevision:
    """The externally supplied pose snapshot used by one training job."""

    graph_revision: str = "unknown"
    pose_revision: str = "unknown"
    geometry_revision: str = "unknown"
    component_id: str = ""
    snapshot_digest: str = ""

    @classmethod
    def from_file(
        cls,
        path: Path,
        *,
        graph_revision: str = "unknown",
        pose_revision: str = "unknown",
        geometry_revision: str = "unknown",
        component_id: str = "",
    ) -> "PoseRevision":
        return cls(
            graph_revision=graph_revision,
            pose_revision=pose_revision,
            geometry_revision=geometry_revision,
            component_id=component_id,
            snapshot_digest=_file_digest(path),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "graph_revision": self.graph_revision,
            "pose_revision": self.pose_revision,
            "geometry_revision": self.geometry_revision,
            "component_id": self.component_id,
            "snapshot_digest": self.snapshot_digest,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "PoseRevision":
        return cls(
            graph_revision=str(value.get("graph_revision", "unknown")),
            pose_revision=str(value.get("pose_revision", "unknown")),
            geometry_revision=str(value.get("geometry_revision", "unknown")),
            component_id=str(value.get("component_id", "")),
            snapshot_digest=str(value.get("snapshot_digest", "")),
        )

    @property
    def fingerprint(self) -> str:
        return _digest(self.to_dict())


def _compose_se3(
    a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]
) -> tuple[tuple[float, ...], ...]:
    left = validate_se3(a, "T_component_keyframe")
    right = validate_se3(b, "T_keyframe_camera")
    result = tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )
    return validate_se3(result, "T_component_camera")


@dataclass(frozen=True)
class PoseSnapshot:
    """Immutable, validated graph solution used to re-pose RGB-D frames."""

    solution: GraphSolution
    source_digest: str

    @classmethod
    def from_file(cls, path: Path) -> "PoseSnapshot":
        source = Path(path).expanduser().resolve()
        try:
            raw_bytes = source.read_bytes()
            value = json.loads(raw_bytes.decode("utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise InputUnavailable(f"invalid pose snapshot {source}: {exc}") from exc
        if not isinstance(value, Mapping):
            raise InputUnavailable(f"pose snapshot {source} is not an object")
        payload = value.get("solution", value)
        if not isinstance(payload, Mapping):
            raise InputUnavailable(f"pose snapshot {source} has no graph solution")
        try:
            revision_value = payload["revision"]
            revision = ComponentRevision(
                str(revision_value["component_id"]),
                int(revision_value["epoch"]),
                int(revision_value["revision"]),
            )
            anchor = KeyframeId.from_dict(payload["anchor"])
            membership = tuple(
                KeyframeId.from_dict(item) for item in payload["membership"]
            )
            poses = {
                KeyframeId.from_dict(item["keyframe_id"]): validate_se3(
                    item["T_component_keyframe"], "T_component_keyframe"
                )
                for item in payload["poses"]
            }
            solution = GraphSolution(
                revision,
                anchor,
                membership,
                poses,
                tuple(str(item) for item in payload.get("retracted_constraints", ())),
                tuple(
                    KeyframeId.from_dict(item)
                    for item in payload.get("retracted_keyframes", ())
                ),
            )
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise InputUnavailable(
                f"invalid graph solution in {source}: {exc}"
            ) from exc
        return cls(solution, hashlib.sha256(raw_bytes).hexdigest())

    @property
    def component_id(self) -> str:
        return self.solution.revision.component_id

    @property
    def digest(self) -> str:
        return self.source_digest

    def pose_for(self, keyframe: KeyframeId) -> tuple[tuple[float, ...], ...]:
        try:
            return self.solution.poses[keyframe]
        except KeyError as exc:
            raise InputUnavailable(
                f"pose snapshot {self.digest} does not contain keyframe {keyframe.stable_id}"
            ) from exc

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "swarmdeck.pose-snapshot.v1",
            "solution": self.solution.canonical_dict(),
        }


@dataclass(frozen=True)
class FrameRecord:
    relative_path: str
    size_bytes: int
    sha256: str
    width: int | None = None
    height: int | None = None
    keyframe_id: str = ""
    calibration_version: str = ""

    def __post_init__(self) -> None:
        # Frame records are relative names produced by the capture scanner. Do
        # not let a hand-edited journal turn one into an arbitrary filesystem
        # path when a backend consumes the manifest.
        if (
            not self.relative_path
            or Path(self.relative_path).name != self.relative_path
            or self.relative_path in {".", ".."}
        ):
            raise ValueError(
                f"frame path must be a relative file name: {self.relative_path!r}"
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "width": self.width,
            "height": self.height,
            "keyframe_id": self.keyframe_id,
            "calibration_version": self.calibration_version,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "FrameRecord":
        return cls(
            relative_path=str(value["relative_path"]),
            size_bytes=int(value["size_bytes"]),
            sha256=str(value["sha256"]),
            width=int(value["width"]) if value.get("width") is not None else None,
            height=int(value["height"]) if value.get("height") is not None else None,
            keyframe_id=str(value.get("keyframe_id", "")),
            calibration_version=str(value.get("calibration_version", "")),
        )


@dataclass(frozen=True)
class InputManifest:
    """Immutable content and identity snapshot for a reconstruction job."""

    capture_root: str
    capture_id: str = ""
    robot_id: str = ""
    session_id: str = ""
    submap_id: str = ""
    calibration_version: str = ""
    optical_frame: str = ""
    frame_records: tuple[FrameRecord, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)
    pose_revision: PoseRevision = field(default_factory=PoseRevision)
    # The operator/source path is retained for stale-input checks.  The copy
    # is created by submit() and is the only path passed to a backend.
    pose_snapshot_path: str = ""
    pose_snapshot_copy_path: str = ""

    @classmethod
    def from_capture(
        cls,
        capture_root: Path,
        *,
        pose_revision: PoseRevision | None = None,
        pose_snapshot: Path | None = None,
        allow_missing: bool = True,
    ) -> "InputManifest":
        root = Path(capture_root).expanduser().resolve()
        metadata: dict[str, Any] = {}
        metadata_path = root / "capture_manifest.json"
        if metadata_path.exists():
            try:
                value = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise InputUnavailable(
                    f"invalid capture manifest {metadata_path}: {exc}"
                ) from exc
            if not isinstance(value, dict):
                raise InputUnavailable(
                    f"capture manifest {metadata_path} is not an object"
                )
            metadata = value
        elif not root.exists() and not allow_missing:
            raise InputUnavailable(f"capture directory does not exist: {root}")

        records: list[FrameRecord] = []
        if root.exists():
            for path in sorted(root.glob("*.npz")):
                width = height = None
                keyframe_id = ""
                calibration_version = str(metadata.get("calibration_version", ""))
                try:
                    import numpy as np

                    with np.load(path, allow_pickle=False) as frame:
                        if "rgb" in frame.files:
                            height, width = map(int, frame["rgb"].shape[:2])
                        if "keyframe_id" in frame.files:
                            keyframe_id = str(frame["keyframe_id"].item())
                        if "calibration_version" in frame.files:
                            calibration_version = str(
                                frame["calibration_version"].item()
                            )
                except Exception:
                    # The hash still makes the input immutable; backend validation
                    # reports malformed frames with a useful error later.
                    pass
                records.append(
                    FrameRecord(
                        relative_path=path.name,
                        size_bytes=path.stat().st_size,
                        sha256=_file_digest(path),
                        width=width,
                        height=height,
                        keyframe_id=keyframe_id,
                        calibration_version=calibration_version,
                    )
                )
        if not records and root.exists() and not allow_missing:
            raise InputUnavailable(f"capture contains no RGB-D frames: {root}")
        snapshot_path = ""
        pose = pose_revision or PoseRevision()
        if pose_snapshot is not None:
            snapshot_path = str(Path(pose_snapshot).expanduser().resolve())
            snapshot = PoseSnapshot.from_file(Path(snapshot_path))
            pose = PoseRevision(
                graph_revision=f"{snapshot.solution.revision.epoch}",
                pose_revision=f"{snapshot.solution.revision.revision}",
                component_id=snapshot.component_id,
                snapshot_digest=snapshot.digest,
            )
        return cls(
            capture_root=str(root),
            capture_id=str(metadata.get("capture_id", "")),
            robot_id=str(metadata.get("robot_id", "")),
            session_id=str(metadata.get("session_id", "")),
            submap_id=str(metadata.get("submap_id", "")),
            calibration_version=str(metadata.get("calibration_version", "")),
            optical_frame=str(metadata.get("optical_frame", "")),
            frame_records=tuple(records),
            metadata=metadata,
            pose_revision=pose,
            pose_snapshot_path=snapshot_path,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "capture_root": self.capture_root,
            "capture_id": self.capture_id,
            "robot_id": self.robot_id,
            "session_id": self.session_id,
            "submap_id": self.submap_id,
            "calibration_version": self.calibration_version,
            "optical_frame": self.optical_frame,
            "frame_records": [record.to_dict() for record in self.frame_records],
            "metadata": dict(self.metadata),
            "pose_revision": self.pose_revision.to_dict(),
            "pose_snapshot_path": self.pose_snapshot_path,
            "pose_snapshot_copy_path": self.pose_snapshot_copy_path,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "InputManifest":
        return cls(
            capture_root=str(value["capture_root"]),
            capture_id=str(value.get("capture_id", "")),
            robot_id=str(value.get("robot_id", "")),
            session_id=str(value.get("session_id", "")),
            submap_id=str(value.get("submap_id", "")),
            calibration_version=str(value.get("calibration_version", "")),
            optical_frame=str(value.get("optical_frame", "")),
            frame_records=tuple(
                FrameRecord.from_dict(item) for item in value.get("frame_records", [])
            ),
            metadata=dict(value.get("metadata", {})),
            pose_revision=PoseRevision.from_dict(value.get("pose_revision", {})),
            pose_snapshot_path=str(value.get("pose_snapshot_path", "")),
            pose_snapshot_copy_path=str(value.get("pose_snapshot_copy_path", "")),
        )

    @property
    def fingerprint(self) -> str:
        return _digest(self.to_dict())

    @property
    def total_bytes(self) -> int:
        return sum(record.size_bytes for record in self.frame_records)

    @property
    def available(self) -> bool:
        root = Path(self.capture_root)
        return root.is_dir() and bool(self.frame_records)


@dataclass(frozen=True)
class ResourceBudget:
    max_frames: int = 2000
    max_width: int | None = None
    max_height: int | None = None
    max_gaussians: int = 150000
    max_gpu_memory_mb: int | None = None
    max_training_seconds: float = 3600.0
    max_disk_bytes: int = 20 * 1024**3
    max_upload_bytes: int | None = None

    def validate(self, manifest: InputManifest) -> None:
        if self.max_frames < 1 or len(manifest.frame_records) > self.max_frames:
            raise ResourceBudgetError(
                f"input has {len(manifest.frame_records)} frames; budget allows {self.max_frames}"
            )
        if self.max_gaussians < 1:
            raise ResourceBudgetError("max_gaussians must be positive")
        if self.max_training_seconds <= 0 or self.max_disk_bytes < 1:
            raise ResourceBudgetError("training and disk budgets must be positive")
        if (
            self.max_upload_bytes is not None
            and manifest.total_bytes > self.max_upload_bytes
        ):
            raise ResourceBudgetError("input exceeds upload byte budget")
        dimensions = [
            (record.width, record.height)
            for record in manifest.frame_records
            if record.width is not None and record.height is not None
        ]
        if (self.max_width is not None or self.max_height is not None) and len(
            dimensions
        ) != len(manifest.frame_records):
            raise ResourceBudgetError(
                "cannot enforce resolution budget for a frame with unreadable dimensions"
            )
        if self.max_width is not None and any(
            width > self.max_width for width, _ in dimensions
        ):
            raise ResourceBudgetError("input width exceeds resource budget")
        if self.max_height is not None and any(
            height > self.max_height for _, height in dimensions
        ):
            raise ResourceBudgetError("input height exceeds resource budget")

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_frames": self.max_frames,
            "max_width": self.max_width,
            "max_height": self.max_height,
            "max_gaussians": self.max_gaussians,
            "max_gpu_memory_mb": self.max_gpu_memory_mb,
            "max_training_seconds": self.max_training_seconds,
            "max_disk_bytes": self.max_disk_bytes,
            "max_upload_bytes": self.max_upload_bytes,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ResourceBudget":
        return cls(
            **{key: value[key] for key in cls.__dataclass_fields__ if key in value}
        )


@dataclass(frozen=True)
class BackendCapabilities:
    """Capabilities are declarations tested by the scheduler, not guesses."""

    fixed_camera_poses: bool = False
    pose_refinement: bool = False
    checkpoint_resume: bool = False
    incremental_updates: bool = False
    external_pose_constraints: bool = False
    depth_loss: bool = False
    batch: bool = True

    def to_dict(self) -> dict[str, bool]:
        return {key: bool(getattr(self, key)) for key in self.__dataclass_fields__}


@dataclass
class ReconstructionJob:
    job_id: str
    backend: str
    backend_version: str
    input_manifest: InputManifest
    budget: ResourceBudget
    artifact_target: str | None = None
    config: dict[str, Any] = field(default_factory=dict)
    required_capabilities: tuple[str, ...] = ()
    state: JobState = JobState.QUEUED
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)
    cancel_requested: bool = False
    prepared: bool = False
    staged_artifact: str | None = None
    published_at: float | None = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "job_id": self.job_id,
            "backend": self.backend,
            "backend_version": self.backend_version,
            "input_manifest": self.input_manifest.to_dict(),
            "input_fingerprint": self.input_manifest.fingerprint,
            "budget": self.budget.to_dict(),
            "artifact_target": self.artifact_target,
            "config": self.config,
            "required_capabilities": list(self.required_capabilities),
            "state": self.state.value,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "cancel_requested": self.cancel_requested,
            "prepared": self.prepared,
            "staged_artifact": self.staged_artifact,
            "published_at": self.published_at,
            "error": self.error,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ReconstructionJob":
        job_id = str(value["job_id"])
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", job_id):
            raise ValueError("invalid job id in journal")
        manifest = InputManifest.from_dict(value["input_manifest"])
        recorded_fingerprint = value.get("input_fingerprint")
        if recorded_fingerprint and recorded_fingerprint != manifest.fingerprint:
            raise ValueError(
                "job journal input manifest fingerprint does not match contents"
            )
        return cls(
            job_id=job_id,
            backend=str(value["backend"]),
            backend_version=str(value.get("backend_version", "unknown")),
            input_manifest=manifest,
            budget=ResourceBudget.from_dict(value.get("budget", {})),
            artifact_target=value.get("artifact_target"),
            config=dict(value.get("config", {})),
            required_capabilities=tuple(value.get("required_capabilities", [])),
            state=JobState(value.get("state", JobState.QUEUED.value)),
            created_at=float(value.get("created_at", time.time())),
            updated_at=float(value.get("updated_at", time.time())),
            cancel_requested=bool(value.get("cancel_requested", False)),
            prepared=bool(value.get("prepared", False)),
            staged_artifact=value.get("staged_artifact"),
            published_at=value.get("published_at"),
            error=value.get("error"),
        )


CommandRunner = Callable[[Sequence[str], Path, threading.Event, float | None], None]


class ReconstructionBackend:
    name = "backend"
    version = "unknown"
    capabilities = BackendCapabilities()

    def validate(self, job: ReconstructionJob) -> None:
        """Validate installation and the backend-specific job configuration."""

    def prepare(
        self,
        job: ReconstructionJob,
        workdir: Path,
        run_command: CommandRunner,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Prepare a backend dataset in ``workdir``."""

    def run(
        self,
        job: ReconstructionJob,
        workdir: Path,
        cancel_event: threading.Event,
        run_command: CommandRunner,
    ) -> Path:
        raise NotImplementedError


@dataclass
class UmamiBackend(ReconstructionBackend):
    """Fixed-pose batch adapter around the existing UMAMI script."""

    umami_root: Path
    config: Path
    script: Path = DEFAULT_UMAMI_SCRIPT
    python_executable: str = sys.executable
    export_stride: int = 8
    name: str = "umami-fixed-pose-batch"
    version: str = "b1251d435b09f4298a414dbc1151c9bae42c3c37"
    capabilities: BackendCapabilities = field(
        default_factory=lambda: BackendCapabilities(
            fixed_camera_poses=True,
            pose_refinement=False,
            checkpoint_resume=False,
            incremental_updates=False,
            external_pose_constraints=False,
            depth_loss=False,
            batch=True,
        )
    )

    def validate(self, job: ReconstructionJob) -> None:
        if not self.umami_root.is_dir():
            raise InputUnavailable(
                f"UMAMI installation is unavailable: {self.umami_root}"
            )
        trainer = self.umami_root / "bin" / "train_colmap"
        if not trainer.exists() or not os.access(trainer, os.X_OK):
            raise InputUnavailable(f"UMAMI trainer is unavailable: {trainer}")
        if not self.script.is_file():
            raise InputUnavailable(
                f"reconstruction script is unavailable: {self.script}"
            )
        if not self.config.is_file():
            raise InputUnavailable(f"UMAMI config is unavailable: {self.config}")
        if not job.input_manifest.available:
            raise InputUnavailable(
                f"RGB-D input is unavailable: {job.input_manifest.capture_root}"
            )
        if job.input_manifest.pose_snapshot_copy_path:
            PoseSnapshot.from_file(Path(job.input_manifest.pose_snapshot_copy_path))
        for capability in job.required_capabilities:
            if not getattr(self.capabilities, capability, False):
                raise ReconstructionError(
                    f"backend {self.name} does not declare capability {capability}"
                )

    def prepare(
        self,
        job: ReconstructionJob,
        workdir: Path,
        run_command: CommandRunner,
        cancel_event: threading.Event | None = None,
    ) -> None:
        dataset = workdir / "dataset"
        command = [
            self.python_executable,
            str(self.script),
            "export",
            job.input_manifest.capture_root,
            str(dataset),
            "--stride",
            str(self.export_stride),
            "--max-points",
            str(job.budget.max_gaussians),
        ]
        pose_snapshot = (
            job.input_manifest.pose_snapshot_copy_path
            or job.input_manifest.pose_snapshot_path
        )
        if pose_snapshot:
            command.extend(["--pose-snapshot", pose_snapshot])
        run_command(
            command,
            workdir,
            cancel_event or threading.Event(),
            None,
        )

    def run(
        self,
        job: ReconstructionJob,
        workdir: Path,
        cancel_event: threading.Event,
        run_command: CommandRunner,
    ) -> Path:
        output = workdir / "trained"
        staged = workdir / "artifact.swgs"
        command = [
            self.python_executable,
            str(self.script),
            "train",
            "--umami",
            str(self.umami_root),
            "--config",
            str(self.config),
            "--dataset",
            str(workdir / "dataset"),
            "--output",
            str(output),
            "--publish",
            str(staged),
            "--budget",
            str(job.budget.max_gaussians),
        ]
        run_command(command, workdir, cancel_event, job.budget.max_training_seconds)
        if not staged.is_file() or staged.stat().st_size < 16:
            raise ReconstructionError("UMAMI did not produce a compact SWGS artifact")
        return staged


class DurableJobRunner:
    """Journaled runner with atomic, revision-checked artifact publication."""

    def __init__(
        self,
        root: Path,
        backends: Iterable[ReconstructionBackend] | None = None,
        *,
        gpu_memory_reader: Callable[[], int] | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self.jobs_dir = self.root / "jobs"
        self.jobs_dir.mkdir(parents=True, exist_ok=True)
        self._backends = {backend.name: backend for backend in (backends or ())}
        self._events: dict[str, threading.Event] = {}
        self._lock = threading.RLock()
        self._gpu_memory_reader = gpu_memory_reader
        self._latest_path = self.root / "latest.json"
        self._latest: dict[str, str] = {}
        if self._latest_path.exists():
            try:
                self._latest = json.loads(self._latest_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                self._latest = {}
        self._jobs: dict[str, ReconstructionJob] = {}
        for path in sorted(self.jobs_dir.glob("*.json")):
            try:
                job = ReconstructionJob.from_dict(
                    json.loads(path.read_text(encoding="utf-8"))
                )
            except (OSError, ValueError, KeyError, TypeError):
                continue
            if job.state == JobState.TRAINING and not self._job_is_running(job.job_id):
                job.state = JobState.QUEUED
                job.prepared = False
                job.error = "recovered after runner restart"
                self._write(job)
            self._jobs[job.job_id] = job

    def register_backend(self, backend: ReconstructionBackend) -> None:
        self._backends[backend.name] = backend

    def _job_is_running(self, job_id: str) -> bool:
        with self._job_process_lock(job_id) as acquired:
            return not acquired

    def _job_path(self, job_id: str) -> Path:
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", job_id or ""):
            raise ValueError("invalid job id")
        return self.jobs_dir / f"{job_id}.json"

    @contextmanager
    def _job_process_lock(self, job_id: str):
        """Coordinate workers across CLI processes without blocking cancel/status."""
        lock_path = self._job_path(job_id).with_suffix(".lock")
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        handle = lock_path.open("a+")
        acquired = False
        try:
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                acquired = True
            except BlockingIOError:
                pass
            yield acquired
        finally:
            if acquired:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()

    def _workdir(self, job_id: str) -> Path:
        # Validate both the journal key and the path's resolved containment.
        path = (self.jobs_dir / job_id / "work").resolve()
        jobs_root = self.jobs_dir.resolve()
        if os.path.commonpath((str(path), str(jobs_root))) != str(jobs_root):
            raise ReconstructionError("job work directory escapes the journal root")
        return path

    @staticmethod
    def _within(path: Path, root: Path) -> bool:
        try:
            return os.path.commonpath(
                (str(path.resolve()), str(root.resolve()))
            ) == str(root.resolve())
        except ValueError:
            return False

    def _cancel_marker(self, job_id: str) -> Path:
        return self._job_path(job_id).with_suffix(".cancel")

    def _disk_cancel_requested(self, job: ReconstructionJob) -> bool:
        marker = self._cancel_marker(job.job_id)
        if marker.exists():
            return True
        try:
            value = json.loads(self._job_path(job.job_id).read_text(encoding="utf-8"))
            return bool(value.get("cancel_requested", False))
        except (OSError, json.JSONDecodeError, ValueError, AttributeError):
            return False

    def _write(self, job: ReconstructionJob) -> None:
        # A separate ``cancel`` CLI writes a marker while a worker may be
        # updating the journal. The marker is authoritative until the worker
        # records a terminal canceled state.
        try:
            if self._cancel_marker(job.job_id).exists():
                job.cancel_requested = True
        except ValueError:
            pass
        job.updated_at = time.time()
        _atomic_json(self._job_path(job.job_id), job.to_dict())

    def _backend(self, job: ReconstructionJob) -> ReconstructionBackend:
        try:
            return self._backends[job.backend]
        except KeyError as exc:
            raise ReconstructionError(
                f"backend is not registered: {job.backend}"
            ) from exc

    def _set_latest(self, target: str, job_id: str) -> None:
        with self._latest_lock():
            self._set_latest_unlocked(target, job_id)

    @contextmanager
    def _latest_lock(self):
        lock_path = self.root / "latest.lock"
        with lock_path.open("a+") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)

    def _set_latest_unlocked(self, target: str, job_id: str) -> None:
        try:
            latest = json.loads(self._latest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            latest = {}
        latest[str(Path(target).expanduser().resolve())] = job_id
        _atomic_json(self._latest_path, latest)
        self._latest = latest

    def _latest_job(self, target: str) -> str | None:
        try:
            latest = json.loads(self._latest_path.read_text(encoding="utf-8"))
            if isinstance(latest, dict):
                return latest.get(str(Path(target).expanduser().resolve()))
        except (OSError, json.JSONDecodeError):
            pass
        return self._latest.get(str(Path(target).expanduser().resolve()))

    def submit(
        self,
        input_manifest: InputManifest | Path,
        *,
        backend: str,
        budget: ResourceBudget | None = None,
        artifact_target: Path | None = None,
        config: Mapping[str, Any] | None = None,
        required_capabilities: Iterable[str] = (),
        job_id: str | None = None,
    ) -> ReconstructionJob:
        with self._lock:
            if isinstance(input_manifest, (str, Path)):
                input_manifest = InputManifest.from_capture(Path(input_manifest))
            if backend not in self._backends:
                raise ReconstructionError(f"backend is not registered: {backend}")
            selected_budget = budget or ResourceBudget()
            selected_budget.validate(input_manifest)
            backend_impl = self._backends[backend]
            resolved_target = (
                Path(artifact_target).expanduser().resolve()
                if artifact_target
                else None
            )
            if resolved_target is not None:
                capture_root = Path(input_manifest.capture_root).resolve()
                if self._within(resolved_target, capture_root) or self._within(
                    resolved_target, self.jobs_dir
                ):
                    raise ReconstructionError(
                        "artifact target overlaps capture input or job journal"
                    )
            selected_job_id = job_id or uuid.uuid4().hex
            self._job_path(selected_job_id)
            if (
                selected_job_id in self._jobs
                or self._job_path(selected_job_id).exists()
            ):
                raise ReconstructionError(f"job already exists: {selected_job_id}")

            # A graph solution is a point-in-time input.  Copy it into the
            # journal before the job becomes visible so prepare/train cannot
            # observe a later replacement at the operator-supplied path (the
            # ABA case).  The digest is checked against the manifest captured
            # above, so a source changing during submission fails closed.
            if input_manifest.pose_snapshot_path:
                source = Path(input_manifest.pose_snapshot_path).expanduser().resolve()
                try:
                    snapshot_bytes = source.read_bytes()
                except OSError as exc:
                    raise InputUnavailable(
                        f"pose snapshot disappeared during submission: {source}"
                    ) from exc
                if (
                    hashlib.sha256(snapshot_bytes).hexdigest()
                    != input_manifest.pose_revision.snapshot_digest
                ):
                    raise InputUnavailable(
                        f"pose snapshot changed during submission: {source}"
                    )
                copied = (
                    self.jobs_dir / selected_job_id / "inputs" / "graph_solution.json"
                )
                _atomic_bytes(copied, snapshot_bytes)
                input_manifest = replace(
                    input_manifest, pose_snapshot_copy_path=str(copied.resolve())
                )
            job = ReconstructionJob(
                job_id=selected_job_id,
                backend=backend_impl.name,
                backend_version=backend_impl.version,
                input_manifest=input_manifest,
                budget=selected_budget,
                artifact_target=str(resolved_target) if resolved_target else None,
                config=dict(config or {}),
                required_capabilities=tuple(required_capabilities),
            )
            if not input_manifest.available:
                job.state = JobState.WAITING_FOR_INPUT
            self._jobs[job.job_id] = job
            self._write(job)
            if job.artifact_target:
                self._set_latest(job.artifact_target, job.job_id)
            return job

    def get(self, job_id: str) -> ReconstructionJob:
        # Reads intentionally do not take the runner lock: status and cancel
        # must remain responsive while a trainer subprocess is running.
        if job_id in self._jobs:
            return self._jobs[job_id]
        path = self._job_path(job_id)
        if not path.exists():
            raise KeyError(job_id)
        job = ReconstructionJob.from_dict(json.loads(path.read_text(encoding="utf-8")))
        self._jobs[job_id] = job
        return job

    def status(
        self, job_id: str | None = None
    ) -> dict[str, Any] | list[dict[str, Any]]:
        if job_id is not None:
            return self.get(job_id).to_dict()
        return [self._jobs[key].to_dict() for key in sorted(self._jobs)]

    def query_status(
        self, job_id: str | None = None
    ) -> dict[str, Any] | list[dict[str, Any]]:
        """Explicit operation name for service adapters and CLI callers."""
        return self.status(job_id)

    def validate(self, job_id: str) -> dict[str, Any]:
        with self._lock:
            job = self.get(job_id)
            backend = self._backend(job)
            job.budget.validate(job.input_manifest)
            try:
                backend.validate(job)
            except InputUnavailable:
                job.state = JobState.WAITING_FOR_INPUT
                self._write(job)
                raise
            if job.state == JobState.WAITING_FOR_INPUT:
                job.state = JobState.QUEUED
            self._write(job)
            return {
                "job_id": job.job_id,
                "backend": backend.name,
                "backend_version": backend.version,
                "capabilities": backend.capabilities.to_dict(),
                "input_fingerprint": job.input_manifest.fingerprint,
            }

    def _disk_usage(self, path: Path) -> int:
        if not path.exists():
            return 0
        if path.is_file():
            return path.stat().st_size
        return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())

    def _check_limits(
        self, job: ReconstructionJob, workdir: Path, started: float
    ) -> None:
        if time.monotonic() - started > job.budget.max_training_seconds:
            raise ResourceBudgetError("training time budget exceeded")
        if self._disk_usage(workdir) > job.budget.max_disk_bytes:
            raise ResourceBudgetError("job disk budget exceeded")
        if job.budget.max_gpu_memory_mb is not None:
            if self._gpu_memory_reader is None:
                raise ResourceBudgetError(
                    "GPU memory budget requested but no GPU memory monitor is configured"
                )
            if self._gpu_memory_reader() > job.budget.max_gpu_memory_mb:
                raise ResourceBudgetError("GPU memory budget exceeded")

    def _run_command(
        self,
        command: Sequence[str],
        cwd: Path,
        cancel_event: threading.Event,
        timeout: float | None,
        job: ReconstructionJob,
    ) -> None:
        cwd.mkdir(parents=True, exist_ok=True)
        log_path = cwd / "commands.log"
        started = time.monotonic()
        with log_path.open("ab") as log:
            log.write(("$ " + " ".join(map(str, command)) + "\n").encode())
            process = subprocess.Popen(
                list(map(str, command)),
                cwd=cwd,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            process_group = os.getpgid(process.pid)

            def terminate_group() -> None:
                # The trainer may spawn workers.  Always terminate its process
                # group when this command exits through cancellation, a budget
                # exception, or another unexpected exception.
                try:
                    os.killpg(process_group, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process_group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()

            try:
                while process.poll() is None:
                    if (
                        cancel_event.is_set()
                        or job.cancel_requested
                        or self._disk_cancel_requested(job)
                    ):
                        terminate_group()
                        raise JobCanceled("reconstruction command canceled")
                    self._check_limits(job, cwd, started)
                    if timeout is not None and time.monotonic() - started > timeout:
                        terminate_group()
                        raise ResourceBudgetError("reconstruction command timed out")
                    time.sleep(0.05)
                if process.returncode:
                    raise ReconstructionError(
                        f"reconstruction command exited with {process.returncode}; see {log_path}"
                    )
            finally:
                terminate_group()

    def _prepare_impl(self, job_id: str) -> ReconstructionJob:
        job = self.get(job_id)
        if job.cancel_requested or self._disk_cancel_requested(job):
            job.state = JobState.CANCELED
            self._write(job)
            raise JobCanceled(job_id)
        self.validate(job_id)
        job = self.get(job_id)
        workdir = self._workdir(job.job_id)
        workdir.mkdir(parents=True, exist_ok=True)
        # A backend without checkpoint support always starts a bounded job
        # from a clean preparation directory.
        if not self._backend(job).capabilities.checkpoint_resume and job.prepared:
            _safe_rmtree(workdir, self.jobs_dir)
            workdir.mkdir(parents=True, exist_ok=True)
            job.prepared = False
        event = self._events.setdefault(job.job_id, threading.Event())
        self._backend(job).prepare(
            job,
            workdir,
            lambda command, cwd, event, timeout: self._run_command(
                command, cwd, event, timeout, job
            ),
            event,
        )
        self._check_limits(job, workdir, time.monotonic())
        job.prepared = True
        job.state = JobState.QUEUED
        self._write(job)
        return job

    def prepare(self, job_id: str) -> ReconstructionJob:
        with self._job_process_lock(job_id) as acquired:
            if not acquired:
                return self.get(job_id)
            return self._prepare_impl(job_id)

    def _current_manifest(self, job: ReconstructionJob) -> InputManifest:
        return InputManifest.from_capture(
            Path(job.input_manifest.capture_root),
            pose_revision=job.input_manifest.pose_revision,
            pose_snapshot=(
                Path(job.input_manifest.pose_snapshot_path)
                if job.input_manifest.pose_snapshot_path
                else None
            ),
        )

    def _is_current(self, job: ReconstructionJob) -> bool:
        if not job.artifact_target:
            return True
        if self._latest_job(job.artifact_target) != job.job_id:
            return False
        try:
            current = self._current_manifest(job)
            # Freshness is checked against the retained live source path. The
            # copied path is restored only to compare equivalent journal
            # manifests; it remains immutable and is still the backend input.
            if job.input_manifest.pose_snapshot_copy_path:
                current = replace(
                    current,
                    pose_snapshot_copy_path=job.input_manifest.pose_snapshot_copy_path,
                )
            return current.fingerprint == job.input_manifest.fingerprint
        except InputUnavailable:
            return False

    def _export_impl(
        self, job_id: str, target: Path | None = None
    ) -> ReconstructionJob:
        job = self.get(job_id)
        event = self._events.get(job.job_id)
        if (
            job.cancel_requested
            or self._disk_cancel_requested(job)
            or (event is not None and event.is_set())
        ):
            job.state = JobState.CANCELED
            job.error = "publication canceled"
            self._write(job)
            raise JobCanceled(job.error)
        with self._latest_lock():
            if target is not None:
                resolved = str(Path(target).expanduser().resolve())
                if (
                    job.artifact_target
                    and str(Path(job.artifact_target).resolve()) != resolved
                ):
                    raise ReconstructionError(
                        "export target differs from immutable job target"
                    )
                if not job.artifact_target:
                    job.artifact_target = resolved
                    self._set_latest_unlocked(resolved, job.job_id)
            if not job.staged_artifact or not Path(job.staged_artifact).is_file():
                raise ReconstructionError("job has no staged artifact to export")
            staged = Path(job.staged_artifact).resolve()
            if not self._within(staged, self._workdir(job.job_id)):
                raise ReconstructionError(
                    "staged artifact escapes the job work directory"
                )
            if not self._is_current(job):
                job.state = JobState.STALE
                job.error = "input or pose revision was superseded before publication"
                self._write(job)
                raise StaleResult(job.error)
            if not job.artifact_target:
                raise ReconstructionError("job has no artifact target")
            destination = Path(job.artifact_target).expanduser().resolve()
            capture_root = Path(job.input_manifest.capture_root).resolve()
            if self._within(destination, capture_root) or self._within(
                destination, self.jobs_dir
            ):
                raise ReconstructionError(
                    "artifact target overlaps capture input or job journal"
                )
            destination.parent.mkdir(parents=True, exist_ok=True)

            # Each publication has an immutable filename.  The sidecar is an
            # atomic pointer to that filename, so readers never pair a new
            # artifact with an older manifest.  The requested target remains a
            # compatibility copy for existing callers; new readers should use
            # the ``artifact`` path from the pointer manifest.
            versioned = destination.with_name(f"{destination.name}.{job.job_id}")
            os.replace(staged, versioned)
            published_at = time.time()
            artifact_size = versioned.stat().st_size
            artifact_sha256 = _file_digest(versioned)
            _atomic_json(
                destination.with_name(destination.name + ".manifest.json"),
                {
                    "schema_version": 2,
                    "version": job.job_id,
                    "state": JobState.READY.value,
                    "job_id": job.job_id,
                    "backend": job.backend,
                    "backend_version": job.backend_version,
                    "input_fingerprint": job.input_manifest.fingerprint,
                    "pose_revision": job.input_manifest.pose_revision.to_dict(),
                    "artifact": str(versioned),
                    "artifact_size_bytes": artifact_size,
                    "artifact_sha256": artifact_sha256,
                    "compatibility_artifact": str(destination),
                    "published_at": published_at,
                },
            )
            compatibility_temporary = destination.with_name(
                f".{destination.name}.{job.job_id}.compatibility"
            )
            shutil.copyfile(versioned, compatibility_temporary)
            os.replace(compatibility_temporary, destination)
            job.published_at = published_at
            job.state = JobState.READY
            self._write(job)
            return job

    def export(self, job_id: str, target: Path | None = None) -> ReconstructionJob:
        with self._job_process_lock(job_id) as acquired:
            if not acquired:
                raise ReconstructionError("job is currently running in another process")
            return self._export_impl(job_id, target)

    def run(self, job_id: str, *, publish: bool = True) -> ReconstructionJob:
        with self._job_process_lock(job_id) as acquired:
            if not acquired:
                return self.get(job_id)
            job = self.get(job_id)
            if job.state in (JobState.CANCELED, JobState.STALE):
                return job
            if job.state == JobState.TRAINING:
                return job
            event = self._events.setdefault(job.job_id, threading.Event())
            if job.cancel_requested or self._disk_cancel_requested(job):
                event.set()
                job.state = JobState.CANCELED
                self._write(job)
                return job
            try:
                self.validate(job_id)
                job = self.get(job_id)
                if not job.prepared:
                    self._prepare_impl(job_id)
                    job = self.get(job_id)
                job.state = JobState.TRAINING
                self._write(job)
                workdir = self._workdir(job.job_id)
                artifact = self._backend(job).run(
                    job,
                    workdir,
                    event,
                    lambda command, cwd, command_event, timeout: self._run_command(
                        command, cwd, command_event, timeout, job
                    ),
                )
                self._check_limits(job, workdir, time.monotonic())
                if (
                    event.is_set()
                    or job.cancel_requested
                    or self._disk_cancel_requested(job)
                ):
                    raise JobCanceled("reconstruction canceled")
                staged = Path(artifact).resolve()
                if not self._within(staged, workdir):
                    raise ReconstructionError(
                        "backend artifact escapes the job work directory"
                    )
                job.staged_artifact = str(staged)
                job.state = JobState.READY
                self._write(job)
                if publish:
                    return self._export_impl(job_id)
                return job
            except JobCanceled as exc:
                job.state = JobState.CANCELED
                job.error = str(exc)
                self._write(job)
                return job
            except StaleResult:
                return self.get(job_id)
            except InputUnavailable as exc:
                job.state = JobState.WAITING_FOR_INPUT
                job.error = str(exc)
                self._write(job)
                return job
            except Exception as exc:  # journal failures; callers can inspect status
                job.state = JobState.FAILED
                job.error = f"{type(exc).__name__}: {exc}"
                self._write(job)
                return job
            finally:
                if job.state == JobState.CANCELED:
                    try:
                        self._cancel_marker(job_id).unlink()
                    except FileNotFoundError:
                        pass
                self._events.pop(job_id, None)

    def run_async(self, job_id: str, *, publish: bool = True) -> threading.Thread:
        thread = threading.Thread(
            target=self.run, args=(job_id,), kwargs={"publish": publish}, daemon=True
        )
        thread.start()
        return thread

    def cancel(self, job_id: str) -> ReconstructionJob:
        # Set the event before acquiring the journal lock.  A running trainer
        # may be inside a long subprocess while holding the runner lock; the
        # process poll must still observe cancellation immediately.
        event = self._events.setdefault(job_id, threading.Event())
        event.set()
        marker = self._cancel_marker(job_id)
        marker.touch(exist_ok=True)
        with self._lock:
            job = self.get(job_id)
            if job.state == JobState.READY and job.published_at is not None:
                # A completed publication cannot be rolled back by a late
                # cancel request; leave its manifest and state coherent.
                marker.unlink(missing_ok=True)
                event.clear()
                return job
            job.cancel_requested = True
            if job.state in (JobState.QUEUED, JobState.WAITING_FOR_INPUT):
                job.state = JobState.CANCELED
            self._write(job)
            if job.state == JobState.CANCELED:
                marker.unlink(missing_ok=True)
            return job


def _build_cli_runner(args: argparse.Namespace) -> DurableJobRunner:
    runner = DurableJobRunner(Path(args.store))
    if getattr(args, "umami", None) and getattr(args, "config", None):
        runner.register_backend(
            UmamiBackend(
                umami_root=Path(args.umami),
                config=Path(args.config),
                script=Path(getattr(args, "script", DEFAULT_UMAMI_SCRIPT)),
            )
        )
    return runner


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Durable Gaussian reconstruction jobs")
    parser.add_argument("--store", type=Path, default=Path(".reconstruction-jobs"))
    sub = parser.add_subparsers(dest="command", required=True)
    status = sub.add_parser("status")
    status.add_argument("--store", type=Path, default=argparse.SUPPRESS)
    status.add_argument("job_id", nargs="?")
    cancel = sub.add_parser("cancel")
    cancel.add_argument("--store", type=Path, default=argparse.SUPPRESS)
    cancel.add_argument("job_id")
    run = sub.add_parser("run")
    run.add_argument("--store", type=Path, default=argparse.SUPPRESS)
    run.add_argument("job_id")
    run.add_argument("--umami", type=Path)
    run.add_argument("--config", type=Path)
    run.add_argument("--script", type=Path, default=DEFAULT_UMAMI_SCRIPT)
    submit = sub.add_parser("submit")
    submit.add_argument("--store", type=Path, default=argparse.SUPPRESS)
    submit.add_argument("capture", type=Path)
    submit.add_argument("--artifact", type=Path, required=True)
    submit.add_argument("--umami", type=Path, required=True)
    submit.add_argument("--config", type=Path, required=True)
    submit.add_argument("--script", type=Path, default=DEFAULT_UMAMI_SCRIPT)
    submit.add_argument("--pose-snapshot", type=Path)
    submit.add_argument("--max-frames", type=int, default=2000)
    args = parser.parse_args(argv)
    runner = _build_cli_runner(args)
    if args.command == "status":
        print(
            json.dumps(
                runner.status(args.job_id),
                indent=2,
                sort_keys=True,
                default=_json_default,
            )
        )
        return 0
    if args.command == "cancel":
        print(
            json.dumps(runner.cancel(args.job_id).to_dict(), indent=2, sort_keys=True)
        )
        return 0
    if args.command == "run":
        print(json.dumps(runner.run(args.job_id).to_dict(), indent=2, sort_keys=True))
        return 0
    backend = runner._backends["umami-fixed-pose-batch"]
    input_manifest = InputManifest.from_capture(
        args.capture, pose_snapshot=args.pose_snapshot
    )
    job = runner.submit(
        input_manifest,
        backend=backend.name,
        budget=ResourceBudget(max_frames=args.max_frames),
        artifact_target=args.artifact,
    )
    print(json.dumps(job.to_dict(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
