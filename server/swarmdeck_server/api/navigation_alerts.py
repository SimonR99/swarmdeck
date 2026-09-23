"""Tell the operator why a navigation goal failed, in words they can act on.

The robot reports the planner's or controller's own reason
(``nav_failure_reason``); the dashboard used to show only "NAV FAILED" with
that text in a tooltip. Each failure now raises an alert whose message says
what happened and what to do, with the raw reason kept as its detail.
"""

from __future__ import annotations

# Checked in order: the first group with a phrase in the reason explains it.
_EXPLANATIONS: tuple[tuple[tuple[str, ...], str], ...] = (
    (
        ("exploring toward it ended",),
        "Explored toward the goal until there was nothing left to explore, "
        "without finding a route to it. It may be unreachable for this robot.",
    ),
    (
        (
            "goal cannot be linked",
            "no route over the global graph reaches",
            "no route through the local lattice reaches",
        ),
        "No known route to the goal: the explored map does not connect it to "
        'where the robot can go. Send it with "Explore if unknown" to explore '
        "toward it, or pick a goal in mapped space.",
    ),
    (
        ("no mapped ground under the goal",),
        "The goal is not on mapped ground: that area is unexplored, or the "
        "point is not floor.",
    ),
    (
        (
            "current pose cannot be linked",
            "the global graph is empty",
            "the local lattice holds no admissible cell",
        ),
        "The planner cannot place the robot on its own map yet. Let it map "
        "its surroundings, then retry.",
    ),
    (("already at the goal",), "The robot is already at the goal."),
    (
        (
            "stale map identity",
            "map identity stayed stale",
            "requested map is not the one in service",
        ),
        "The map changed while the route was being planned. Retry.",
    ),
    (
        (
            "odometry is stale",
            "map unavailable",
            "no odometry",
            "objective service is unavailable",
            "objective request timed out",
            "objective request failed",
        ),
        "The planner is not ready or not answering. Retry in a moment.",
    ),
    (
        (
            "failed to make progress",
            "made no progress",
            "controller recovery",
            "controller failed",
            "path execution failed",
        ),
        "The robot could not follow the route: it was blocked or stuck, and "
        "recovery did not free it.",
    ),
)


def explain_navigation_failure(reason: str | None) -> str:
    """One operator-facing sentence for a robot's navigation failure."""
    text = (reason or "").lower()
    for phrases, explanation in _EXPLANATIONS:
        if any(phrase.lower() in text for phrase in phrases):
            return explanation
    if text:
        return "Navigation failed; see the planner's reason below."
    return "Navigation failed without a reported reason."
