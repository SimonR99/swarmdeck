"""Progress along the controller's route, shared by every bridge.

Nav2's ``SimpleProgressChecker`` measures displacement from a reference pose
(``required_movement_radius`` 0.2 m within ``movement_time_allowance`` 20 s).
A robot rocking 0.3 m back and forth on a step it cannot climb satisfies it
forever, so FollowPath never reports ``Failed to make progress`` and none of
the adapter recovery ever runs. Measured 2026-09-17 in the Bistro simulation:
robot_1 dithered on a 7 cm ridge for three minutes with ``nav_status`` still
``active`` in three consecutive exploration trials.

This module measures progress along the path the controller was given: the
arc length of the closest route point to the robot, kept monotone, so that
oscillating on one spot counts as standing still. When that progress has not
advanced by ``min_progress_m`` within ``timeout_s`` while a FollowPath goal is
active, the bridge cancels the goal and records the same kind of failure the
controller's own progress checker would have, so the existing exploration and
objective recovery paths run unchanged.

Nothing here imports ROS. Each bridge supplies two small hooks:

``_route_progress_pose(frame)``
    The robot's XY pose expressed in ``frame`` (the plan's frame), or ``None``
    when that frame cannot be read. Without a pose the watchdog gathers no
    evidence and never fires.
``_fail_route_progress(generation, reason)``
    Cancel the controller goal owning ``generation`` and leave the bridge in
    the terminal state a controller no-progress failure would have produced.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

DEFAULT_TIMEOUT_S = 30.0
DEFAULT_MIN_PROGRESS_M = 0.5
# Arc length beyond the progress reached so far within which the closest route
# point is searched. Between observations the robot moves centimetres, so this
# only stops a self-crossing route from teleporting progress onto a later leg,
# and lets a route that starts behind the robot be joined where the robot is.
SEARCH_WINDOW_M = 3.0
# The bounded failure reason. ``is_physical_no_progress_failure`` matches it.
REASON_PATTERN = r"\bno progress along the route\b"
REASON_LIMIT = 160


@dataclass(frozen=True)
class RouteProgress:
    """How far along a route the robot has been observed so far."""

    index: int  # route segment reached so far, monotone
    distance_m: float  # arc length along the route reached so far, monotone
    offset_m: float  # distance from the robot to the route at that point


def _xy(pose):
    if pose is None:
        return None
    try:
        if isinstance(pose, dict):
            x, y = float(pose["x"]), float(pose["y"])
        else:
            x, y = float(pose[0]), float(pose[1])
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def route_points(path) -> tuple[tuple[float, float], ...]:
    """Distinct finite XY points of a controller path, in order.

    Accepts a ``PlannerPath`` (poses with ``x``/``y``), or a sequence of dicts
    or pairs. Anything unusable yields an empty tuple, which leaves the route
    unsupervised rather than raising inside a bridge tick.
    """
    poses = getattr(path, "poses", path)
    points: list[tuple[float, float]] = []
    try:
        for pose in poses or ():
            if isinstance(pose, dict) or not hasattr(pose, "x"):
                point = _xy(pose)
            else:
                point = _xy((pose.x, pose.y))
            if point is None:
                return ()
            if not points or math.dist(points[-1], point) > 1e-6:
                points.append(point)
    except TypeError:
        return ()
    return tuple(points)


def advance_route_progress(
    points,
    pose,
    previous: RouteProgress | None = None,
    window_m: float = SEARCH_WINDOW_M,
) -> RouteProgress | None:
    """Monotone progress of ``pose`` along ``points``.

    The closest route point is searched from the segment reached so far up to
    ``window_m`` of arc length beyond the progress reached so far, and never
    backwards. A robot rocking on one spot therefore keeps the furthest arc
    length it reached, and a route that starts behind the robot is joined at
    the robot's actual position rather than at the route's first pose. Ties
    prefer the earlier segment. Returns ``None`` for a route with fewer than
    two distinct points or an unusable pose.
    """
    position = _xy(pose)
    count = len(points)
    if position is None or count < 2:
        return None
    if previous is not None and not (0 <= previous.index < count - 1):
        previous = None
    x, y = position
    cumulative = [0.0]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(a, b))
    floor = previous.distance_m if previous is not None else 0.0
    floor = min(max(0.0, floor), cumulative[-1])
    ceiling = floor + max(0.0, float(window_m))
    best: tuple[float, float, int] | None = None
    for index in range(previous.index if previous is not None else 0, count - 1):
        start = cumulative[index]
        if start > ceiling:
            break
        end = cumulative[index + 1]
        length = end - start
        if length <= 0.0:
            continue
        (ax, ay), (bx, by) = points[index], points[index + 1]
        projection = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / length
        arc = start + min(length, max(0.0, projection))
        arc = min(ceiling, max(floor, arc))
        fraction = (arc - start) / length
        px, py = ax + fraction * (bx - ax), ay + fraction * (by - ay)
        offset = math.hypot(x - px, y - py)
        if best is None or offset < best[0] - 1e-9:
            best = (offset, arc, index)
    if best is None:
        return previous
    offset, arc, index = best
    return RouteProgress(index=index, distance_m=max(floor, arc), offset_m=offset)


def _bounded(value, default, *, minimum=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return default
    if not math.isfinite(parsed) or parsed < minimum:
        return default
    return parsed


class RouteProgressWatchdog:
    """Fires once when route progress stalls while a controller goal is active.

    ``observe`` must be called periodically. It resets on every new path
    object, whenever the goal is not active, and on ``reset``, so a stall can
    never be inherited by a later goal. A timeout of zero disables it.
    """

    def __init__(
        self,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        min_progress_m: float = DEFAULT_MIN_PROGRESS_M,
        window_m: float = SEARCH_WINDOW_M,
    ):
        self.timeout_s = _bounded(timeout_s, DEFAULT_TIMEOUT_S)
        self.min_progress_m = _bounded(min_progress_m, DEFAULT_MIN_PROGRESS_M)
        if self.min_progress_m <= 0.0:
            self.min_progress_m = DEFAULT_MIN_PROGRESS_M
        self.window_m = _bounded(window_m, SEARCH_WINDOW_M)
        if self.window_m <= 0.0:
            self.window_m = SEARCH_WINDOW_M
        self.reset()

    @property
    def enabled(self) -> bool:
        return self.timeout_s > 0.0

    def reset(self) -> None:
        self._path = None
        self._points: tuple[tuple[float, float], ...] = ()
        self.progress: RouteProgress | None = None
        self._anchor_distance = 0.0
        self._anchor_time: float | None = None
        self._fired = False

    def observe(self, now: float, pose, path, active: bool) -> bool:
        """Record one observation; return True once when the route stalled.

        ``pose`` is the robot in the path's frame (``None`` gathers no
        evidence). ``path`` is the object the controller executes; a new
        object restarts the clock. ``active`` says whether that controller
        goal is still running; anything else resets the watchdog.
        """
        if not self.enabled or not active or path is None:
            self.reset()
            return False
        if path is not self._path:
            self.reset()
            self._path = path
            self._points = route_points(path)
        if not self._points:
            return False
        progress = advance_route_progress(
            self._points, pose, self.progress, self.window_m
        )
        if progress is None:
            return False
        self.progress = progress
        if (
            self._anchor_time is None
            or progress.distance_m - self._anchor_distance >= self.min_progress_m
        ):
            self._anchor_time = now
            self._anchor_distance = progress.distance_m
        if self._fired:
            return False
        if now - self._anchor_time >= self.timeout_s:
            self._fired = True
            return True
        return False

    def reason(self) -> str:
        """The bounded failure reason recorded on the bridge when it fires."""
        text = f"no progress along the route for {self.timeout_s:.0f} s"
        progress = self.progress
        if progress is not None and self._points:
            total = 0.0
            for a, b in zip(self._points, self._points[1:]):
                total += math.dist(a, b)
            text += (
                f" (reached {progress.distance_m:.2f} m of {total:.2f} m,"
                f" {progress.offset_m:.2f} m off the route)"
            )
        return text[:REASON_LIMIT]


def bridge_route_watchdog(bridge) -> RouteProgressWatchdog:
    """The bridge's watchdog, built from its transport config on first use."""
    watchdog = getattr(bridge, "_route_watchdog", None)
    if watchdog is None:
        cfg = getattr(bridge, "cfg", None) or {}
        watchdog = RouteProgressWatchdog(
            cfg.get("route_progress_timeout_s", DEFAULT_TIMEOUT_S),
            cfg.get("route_progress_min_m", DEFAULT_MIN_PROGRESS_M),
        )
        bridge._route_watchdog = watchdog
    return watchdog


def _warn(bridge, message: str) -> None:
    node = getattr(bridge, "node", None)
    try:
        node.get_logger().warn(message)
    except Exception:
        pass


def route_progress_tick(bridge, now: float | None = None) -> bool:
    """One watchdog observation for ``bridge``; True when it cancelled the goal.

    Called from each bridge's periodic loop. The route is the retained
    FollowPath plan (``_follow_path_display``) and counts as active only while
    it still owns the current goal generation, ``nav_status`` is ``active``
    and the controller has accepted the goal. Anything else, including an
    operator PointGoalAction goal, leaves the watchdog reset.
    """
    watchdog = bridge_route_watchdog(bridge)
    if not watchdog.enabled:
        return False
    if now is None:
        now = time.monotonic()
    retained = getattr(bridge, "_follow_path_display", None)
    generation = getattr(bridge, "_goal_generation", None)
    active = (
        isinstance(retained, tuple)
        and len(retained) == 2
        and generation is not None
        and retained[0] == generation
        and getattr(bridge, "nav_status", None) == "active"
        and getattr(bridge, "_goal_handle", None) is not None
    )
    if not active:
        watchdog.observe(now, None, None, False)
        return False
    plan = retained[1]
    frame = getattr(plan, "frame_id", None)
    pose = bridge._route_progress_pose(frame)
    if pose is None and getattr(bridge, "_route_pose_warned_plan", None) is not plan:
        bridge._route_pose_warned_plan = plan
        _warn(
            bridge,
            f"[{getattr(bridge, 'id', '?')}] route progress watchdog cannot read "
            f"the robot pose in frame {frame!r}; this route is not supervised",
        )
    if not watchdog.observe(now, pose, plan, True):
        return False
    reason = watchdog.reason()
    return bool(bridge._fail_route_progress(generation, reason))
