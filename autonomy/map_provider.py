"""Provider boundary for planner-ready immutable map publications."""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Protocol

from .indexed_mapping import IndexedMapView, SnapshotKey


class PublicationPending(ValueError):
    """The source is between two coherent publications.

    A writer replaces the files of one publication one after the other (the
    MOLA worker publishes ``source.json`` and then ``index.json``), and any
    file may be replaced during a read, so a reader can observe a pair whose
    digests disagree. Neither says anything about the publication already
    indexed: that one stays coherent with its own snapshot key, so it keeps
    serving requests for that key until the next refresh, bounded by the
    coherent-read age.
    """


class MapProvider(Protocol):
    """One peer's source of verified planner-grid publications."""

    peer_root: Path
    publication_path: Path

    def component_ids(self) -> tuple[str, ...]:
        """Return the components in the current atomic publication."""

    def signature(self) -> tuple[int, ...]:
        """Return source identities used to reset retry backoff on change."""

    def refresh(self, view: IndexedMapView, component_id: str) -> SnapshotKey:
        """Verify and publish the selected component into ``view``."""


MapProviderFactory = Callable[[Path], MapProvider]
