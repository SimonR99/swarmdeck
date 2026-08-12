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


def validate_classes(raw: Any) -> list[str]:
    """Normalize a requested class selection into catalog order.

    An empty or unrecognisable selection returns every class.  Detection is
    switched off with ``detection_enabled``; an empty class list reaching the
    adapters would be a second, invisible off switch that no part of the
    dashboard shows.
    """
    if not DETECTION_CLASS_NAMES:
        return [str(name).strip()[:48] for name in raw][:32] if isinstance(raw, list) else []
    if not isinstance(raw, list):
        return list(DETECTION_CLASS_NAMES)
    requested = {str(name).strip() for name in raw}
    selected = [name for name in DETECTION_CLASS_NAMES if name in requested]
    return selected or list(DETECTION_CLASS_NAMES)


def validate_class_floors(raw: Any) -> dict[str, float]:
    """Fill every catalog class with a clamped operator floor.

    Missing keys keep the catalog default so a save never silently drops a
    class the dashboard still shows a slider for.  Unknown names are ignored.
    """
    if not DETECTION_CLASS_FLOORS:
        if not isinstance(raw, dict):
            return {}
        floors: dict[str, float] = {}
        for name, value in list(raw.items())[:32]:
            key = str(name).strip()[:48]
            if not key:
                continue
            try:
                floors[key] = round(max(0.05, min(0.95, float(value))), 2)
            except (TypeError, ValueError):
                continue
        return floors
    floors = dict(DETECTION_CLASS_FLOORS)
    if not isinstance(raw, dict):
        return floors
    for name, value in raw.items():
        key = str(name).strip()
        if key not in floors:
            continue
        try:
            floors[key] = round(max(0.05, min(0.95, float(value))), 2)
        except (TypeError, ValueError):
            pass
    return floors
