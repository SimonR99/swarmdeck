"""Attach live navigation-frame state without creating another ROS reader."""


def _display_navigation(bridge, state, authority):
    """Render a bounded copy of an owned route in the current UI map frame.

    FollowPath keeps its original stable-frame poses. Display transforms come
    from one atomic authority envelope, never separately sampled TF links.
    """
    import math
    import numpy as np
    from adapters.mapping_authority import authority_for_frame
    from autonomy.live_mapping import display_path

    state = dict(state)
    target = bridge.map_frame.lstrip("/")
    transforms = {}

    def matrix(frame):
        frame = frame.lstrip("/")
        if frame == target:
            return np.eye(4)
        if frame not in transforms:
            if not authority or authority["navigation_frame"].lstrip("/") != target:
                raise ValueError("No fresh display transform")
            source = authority_for_frame(authority, frame)
            if source is None:
                raise ValueError("No qualified planning transform")
            transforms[frame] = np.linalg.solve(
                np.asarray(authority["T_component_navigation"]),
                np.asarray(source["T_component_navigation"]),
            )
        return transforms[frame]

    def point(value, transform):
        xyz = transform @ np.array([value["x"], value["y"], value.get("z", 0), 1.0])
        result = dict(value, **dict(zip(("x", "y", "z"), map(float, xyz[:3]))))
        if "yaw" in value:
            heading = transform[:3, :3] @ np.array([
                math.cos(value["yaw"]), math.sin(value["yaw"]), 0,
            ])
            result["yaw"] = math.atan2(heading[1], heading[0])
        if "frame_id" in value:
            result["frame_id"] = target
        return result

    goal = state.get("goal")
    if isinstance(goal, dict) and goal.get("frame_id", target).lstrip("/") != target:
        try:
            state["goal"] = point(goal, matrix(goal["frame_id"]))
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError):
            state["goal"] = None
    retained = getattr(bridge, "_follow_path_display", None)
    if (
        retained is not None
        and retained[0] == getattr(bridge, "_goal_generation", None)
        and state.get("nav_status") == "active"
    ):
        plan = retained[1]
        try:
            transform = matrix(plan.frame_id)
            # Bound the display copy before transforming. Controller poses and
            # their exact endpoint remain untouched.
            points = [
                dict(x=p.x, y=p.y, z=p.z) for p in display_path(plan.poses)
            ]
            points = [point(p, transform) for p in points]
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError):
            points = []
        state["planned_path"] = points
        state["global_planned_path"] = points
    return state


def live_state(bridge):
    state = bridge.state()
    planner = getattr(bridge, "objective_planner", None)
    decorate = getattr(planner, "decorate_state", None)
    if callable(decorate):
        state = decorate(state)
    reader = getattr(bridge, "_mapping_authority", None)
    authority = reader.current() if reader is not None else None
    # Import lazily: legacy ROS 1 deployments need no planning-frame support.
    if getattr(bridge, "_follow_path_display", None) is not None or (
        isinstance(state.get("goal"), dict)
        and state["goal"].get("frame_id", bridge.map_frame).lstrip("/")
        != bridge.map_frame.lstrip("/")
    ):
        state = _display_navigation(bridge, state, authority)
    state = dict(state, live_mapping=None, peer_slam=None)
    if authority is None:
        return state
    if authority.get("peer_slam") is not None:
        from autonomy.slam_status import peer_status
        try:
            state["peer_slam"] = peer_status(
                authority["peer_slam"], bridge.id, authority["mission_id"]
            )
        except (KeyError, ValueError, TypeError):
            pass
    # ROS 1 images do not carry the onboard mapping package.
    from autonomy.live_mapping import PATH_FIELDS, display_path, validate_live_mapping

    try:
        if authority["navigation_frame"].lstrip("/") != bridge.map_frame.lstrip("/"):
            return state
        payload = {
            **authority,
            "authority_age_s": max(0.0, reader.clock() - reader.received_at),
            "pose": state["pose"],
            "goal": state.get("goal"),
            **{name: display_path(state.get(name) or []) for name in PATH_FIELDS},
        }
        state["live_mapping"] = validate_live_mapping(payload, bridge.id)
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
        # Legacy adapters remain useful without qualified replica overlays.
        pass
    return state
