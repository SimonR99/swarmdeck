"""The detector's class catalog, as the backend sees it.

The catalog itself lives with the perception code that measured it
(``adapters/perception/catalog.py``); the backend only needs enough of it to
validate an operator's selection and to describe the choices to the dashboard.

Imported defensively because the two ship separately: the backend image copies
``adapters/`` today, but a deployment that trims it should lose the settings
*toggles*, not the server.  Falling back to an empty catalog means "accept
whatever the adapters accept", which is exactly what the old single-class
behaviour was.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# Resolved from this file rather than the working directory: the backend runs
# from the repo root in Docker and from ``server/`` under pytest, and the
# catalog has to be the same one either way.
REPO = Path(__file__).resolve().parents[3]
if (REPO / "adapters").is_dir() and str(REPO) not in sys.path:
    sys.path.append(str(REPO))

try:  # pragma: no cover - exercised by the absence of adapters/, not by tests
    from adapters.perception.catalog import CATALOG as _CATALOG
except ImportError:  # pragma: no cover
    _CATALOG = ()


DETECTION_CLASSES: list[dict[str, Any]] = [
    {
        "name": target.name,
        "label": target.label,
        "min_score": target.min_score,
    }
    for target in _CATALOG
]

DETECTION_CLASS_NAMES: list[str] = [target["name"] for target in DETECTION_CLASSES]
DETECTION_CLASS_FLOORS: dict[str, float] = {
    target["name"]: float(target["min_score"]) for target in DETECTION_CLASSES
}

FLOOR_MIN = 0.05
FLOOR_MAX = 0.95

#: Ceiling on per-robot override entries, mirroring ``settings.MAX_ROBOTS``.
#: Duplicated rather than imported because ``settings`` imports this module.
MAX_ROBOT_OVERRIDES = 8


def _clamp_floor(value: Any) -> float | None:
    """A stored floor, or ``None`` when the value was not a usable number."""
    try:
        return round(max(FLOOR_MIN, min(FLOOR_MAX, float(value))), 2)
    except (TypeError, ValueError):
        return None


def validate_classes(raw: Any) -> list[str]:
    """Normalize a requested class selection into catalog order.

    An empty or unrecognisable selection returns every class.  Detection is
    switched off with ``detection_enabled``; an empty class list reaching the
    adapters would be a second, invisible off switch that no part of the
    dashboard shows.
    """
    if not DETECTION_CLASS_NAMES:
        return (
            [str(name).strip()[:48] for name in raw][:32]
            if isinstance(raw, list)
            else []
        )
    if not isinstance(raw, list):
        return list(DETECTION_CLASS_NAMES)
    requested = {str(name).strip() for name in raw}
    selected = [name for name in DETECTION_CLASS_NAMES if name in requested]
    return selected or list(DETECTION_CLASS_NAMES)


def validate_class_floors(raw: Any) -> dict[str, float]:
    """Fill every catalog class with a clamped fleet-wide operator floor.

    Missing keys keep the catalog default so a save never silently drops a
    class the dashboard still shows a slider for.  Unknown names are ignored.

    This is a *display* floor, enforced by the backend against stored
    detections (see ``floor_for``), not something a robot is asked to obey.
    What the robots capture is derived separately by ``capture_floors``.
    """
    if not DETECTION_CLASS_FLOORS:
        if not isinstance(raw, dict):
            return {}
        floors: dict[str, float] = {}
        for name, value in list(raw.items())[:32]:
            key = str(name).strip()[:48]
            floor = _clamp_floor(value)
            if not key or floor is None:
                continue
            floors[key] = floor
        return floors
    floors = dict(DETECTION_CLASS_FLOORS)
    if not isinstance(raw, dict):
        return floors
    for name, value in raw.items():
        key = str(name).strip()
        floor = _clamp_floor(value)
        if key not in floors or floor is None:
            continue
        floors[key] = floor
    return floors


def validate_robot_floors(raw: Any) -> dict[str, dict[str, float]]:
    """Per-robot overrides of the fleet floors, stored sparsely.

    Only the classes an operator actually moved *for that robot* are kept, and
    a robot left with no overrides is dropped entirely.  That sparseness is the
    point: a robot that merely agrees with the fleet today keeps falling
    through to the fleet value, so changing the fleet default later still
    reaches it.  A dense copy would freeze every robot at whatever the fleet
    happened to be on the day its row was written.
    """
    if not isinstance(raw, dict):
        return {}
    robots: dict[str, dict[str, float]] = {}
    for robot_id, floors in list(raw.items())[:MAX_ROBOT_OVERRIDES]:
        key = str(robot_id).strip()[:48]
        if not key or not isinstance(floors, dict):
            continue
        clean: dict[str, float] = {}
        for name, value in list(floors.items())[:32]:
            class_name = str(name).strip()[:48]
            floor = _clamp_floor(value)
            if not class_name or floor is None:
                continue
            # Same compatibility rule as validate_class_floors: with no
            # catalog available the adapters own the class list.
            if DETECTION_CLASS_FLOORS and class_name not in DETECTION_CLASS_FLOORS:
                continue
            clean[class_name] = floor
        if clean:
            robots[key] = clean
    return robots


def floor_for(settings: dict[str, Any], robot_id: str, class_name: str) -> float:
    """The score this robot's detections of this class must reach to be shown.

    Robot override, then fleet floor, then the catalog default.  An unknown
    class returns 0.0 rather than a guess -- in the no-catalog compatibility
    mode the backend has no basis to hide anything, and hiding a detection the
    operator cannot see a slider for would be unexplainable from the dashboard.
    """
    override = (settings.get("detection_robot_floors") or {}).get(robot_id)
    if isinstance(override, dict) and class_name in override:
        return float(override[class_name])
    floors = settings.get("detection_class_floors") or {}
    if class_name in floors:
        return float(floors[class_name])
    return float(DETECTION_CLASS_FLOORS.get(class_name, 0.0))


def capture_floors(settings: dict[str, Any]) -> dict[str, float]:
    """What the robots must actually capture at to satisfy every display floor.

    Derived, never operator-set.  The sidecar cannot show what it never
    returned, so the fleet has to run at the LOWEST floor anyone asked for --
    the catalog default, or lower still if an operator dragged a slider below
    it.  Raising a floor therefore never changes this, which is exactly why
    raising one takes effect instantly and can be undone instantly: the
    evidence between the capture floor and the display floor is still arriving,
    just hidden.

    Lowering a floor *below* the capture floor is the one change that must
    reach the robots, and it is also the one change that cannot be retroactive
    however it is implemented -- those frames were never inferred on.
    """
    fleet = settings.get("detection_class_floors") or {}
    overrides = settings.get("detection_robot_floors") or {}

    names = set(DETECTION_CLASS_FLOORS) | set(fleet)
    for floors in overrides.values():
        if isinstance(floors, dict):
            names |= set(floors)

    captured: dict[str, float] = {}
    for name in names:
        candidates = [
            value
            for value in (
                DETECTION_CLASS_FLOORS.get(name),
                fleet.get(name),
                *(
                    floors.get(name)
                    for floors in overrides.values()
                    if isinstance(floors, dict)
                ),
            )
            if isinstance(value, (int, float))
        ]
        if candidates:
            captured[name] = round(min(float(value) for value in candidates), 2)
    return captured
