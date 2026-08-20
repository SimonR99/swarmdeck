"""What SwarmDeck's open-vocabulary detector is allowed to recognise.

YOLOE is prompted with free text, so a "class" here is three things at once:
the name the protocol carries, the prompts that summon it, and the score below
which we do not believe it.  The sidecar, every adapter and the dashboard all
import this one table, so a class an operator switches off and a box arriving
over the websocket are always talking about the same object.

Deliberately dependency-free -- the server imports it to describe the catalog
to the browser, and it must not drag numpy or OpenCV into that process.

`prompts` are measured, not chosen by taste.  YOLOE's text encoder is literal
in ways that are hard to guess: on the reference photograph in `tests/perception/fixtures/`,
"traffic cone" and "sports cone" score *nothing* on a flat sports saucer that
"orange plastic saucer" finds at 0.70, because the word "cone" pulls the
embedding towards tall highway cones.  Re-run `tests/perception/test_catalog_recall.py`
after editing any prompt.

`min_score` is per class for the same reason.  A bare wooden block is a
textbook object and lands at 0.97; a spool of black filament against a dark
desk peaks near 0.37.  A single global floor either drowns in blocks or never
sees a spool, so each class carries the floor its own evidence justifies.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TargetClass:
    """One recognisable object: protocol name, prompts, and its score floor."""

    name: str
    label: str
    prompts: tuple[str, ...]
    min_score: float

    def __post_init__(self) -> None:
        if not self.name or not self.prompts:
            raise ValueError(f"target class {self.name!r} needs a name and prompts")


CATALOG: tuple[TargetClass, ...] = (
    TargetClass(
        name="rubber_duck",
        label="Rubber duck",
        prompts=("yellow rubber duck toy", "rubber duck"),
        min_score=0.25,
    ),
    TargetClass(
        name="wooden_block",
        label="Wooden block",
        # "wooden block" outscores "wooden cube" (0.97 vs 0.86) and generalises
        # to the non-cubic offcuts on the same bench.
        prompts=("wooden block", "wooden toy block"),
        min_score=0.35,
    ),
    TargetClass(
        name="disc_cone",
        label="Disc cone",
        # Every prompt containing "cone" scores at or below 0.10 here; the
        # saucer/disc wording is what actually finds a flat sports marker.
        prompts=("orange plastic saucer", "orange plastic disc", "saucer cone"),
        min_score=0.20,
    ),
    TargetClass(
        name="filament_spool",
        label="Filament spool",
        # The hardest of the five: dark filament on a dark desk, and the spool
        # rim reads as a separate object from the wound material.  Its floor is
        # the one real trade-off in this table -- it peaks around 0.37 where a
        # block reaches 0.97, so it sits as low as it can without labelling
        # every dark desk object a spool.
        prompts=(
            "roll of black filament",
            "spool of black plastic wire",
            "filament spool",
        ),
        min_score=0.25,
    ),
    TargetClass(
        name="pool_noodle",
        label="Pool noodle",
        prompts=("pool noodle", "swimming pool noodle", "blue foam tube"),
        min_score=0.25,
    ),
)

CLASS_NAMES: tuple[str, ...] = tuple(target.name for target in CATALOG)

#: Confidence a sensitivity of 0.55 maps to; the point where each class uses
#: exactly its own calibrated ``min_score``.  See ``ObjectDetector``.
CALIBRATED_CONFIDENCE = 0.25


def by_name(name: str) -> TargetClass | None:
    for target in CATALOG:
        if target.name == name:
            return target
    return None


def resolve(names: object) -> tuple[TargetClass, ...]:
    """Return the requested classes, or the whole catalog when unspecified.

    Unknown names are dropped rather than raising: the list arrives from
    persisted operator settings, and a class renamed in a later version must
    not take the detector down with it.  An empty result means "everything",
    because a detector that silently recognises nothing is the one failure an
    operator cannot see.
    """
    if not isinstance(names, (list, tuple, set, frozenset)):
        return CATALOG
    requested = {str(name).strip() for name in names}
    return tuple(target for target in CATALOG if target.name in requested) or CATALOG


def prompt_bindings(
    classes: tuple[TargetClass, ...] = CATALOG,
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Flatten classes into ``(prompts, owning class name per prompt)``.

    YOLOE binds a flat list of texts and reports the index it matched, so this
    pairing is how a model answer becomes a protocol class again.
    """
    prompts: list[str] = []
    owners: list[str] = []
    for target in classes:
        for prompt in target.prompts:
            prompts.append(prompt)
            owners.append(target.name)
    return tuple(prompts), tuple(owners)
