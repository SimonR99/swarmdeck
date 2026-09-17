"""Attach live navigation-frame state without creating another ROS reader."""


def _display_navigation(bridge, state, authority, anchored=True):
    """Render a bounded copy of an owned route in the current UI map frame.

    FollowPath keeps its original stable-frame poses. Display transforms come
    from one atomic authority envelope, never separately sampled TF links.

    The 2D map is drawn in the navigation frame, where the ground is fixed and
    the planning frame drifts. An anchored display therefore holds the
    navigation<-planning transform of a route's first display until that route
    is replaced; re-sampling it every tick slides the whole route against the
    robot's motion as odometry drift accrues. ``anchored=False`` keeps the
    fresh transform for the live envelope, whose consumers project it back
    through the same authority. Returns the state and whether the two differ.
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

    planner = getattr(bridge, "objective_planner", None)
    global_route = getattr(planner, "global_display_plan", None)
    global_plan = global_route() if callable(global_route) else None
    retained = getattr(bridge, "_follow_path_display", None)
    generation = getattr(bridge, "_goal_generation", None)
    goal = state.get("goal")
    # One owner per tick keeps the goal marker on the end of its route.
    if global_plan is not None:
        owner = global_plan
    elif retained is not None and retained[0] == generation:
        owner = retained[1]
    elif isinstance(goal, dict):
        owner = (generation, goal.get("frame_id"), goal.get("x"), goal.get("y"))
    else:
        owner = None
    drifted = False

    def display(frame):
        nonlocal drifted
        # Always qualify against the fresh authority so display fails closed.
        fresh = matrix(frame)
        if not anchored or frame.lstrip("/") == target:
            return fresh
        key = (owner, frame.lstrip("/"))
        held = getattr(bridge, "_display_anchor", None)
        if held is None or held[0] != key:
            held = bridge._display_anchor = (key, fresh)
        drifted = drifted or not np.array_equal(held[1], fresh)
        return held[1]

    def point(value, transform):
        xyz = transform @ np.array([value["x"], value["y"], value.get("z", 0), 1.0])
        result = dict(value, **dict(zip(("x", "y", "z"), map(float, xyz[:3]))))
        if "yaw" in value:
            heading = transform[:3, :3] @ np.array(
                [
                    math.cos(value["yaw"]),
                    math.sin(value["yaw"]),
                    0,
                ]
            )
            result["yaw"] = math.atan2(heading[1], heading[0])
        if "frame_id" in value:
            result["frame_id"] = target
        return result

    if isinstance(goal, dict) and goal.get("frame_id", target).lstrip("/") != target:
        try:
            state["goal"] = point(goal, display(goal["frame_id"]))
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError):
            state["goal"] = None
    local_points = []
    if (
        retained is not None
        and retained[0] == generation
        and state.get("nav_status") == "active"
        and not (
            global_plan is not None
            and (state.get("objective_continuation") or {}).get("phase") == "planning"
        )
    ):
        plan = retained[1]
        try:
            transform = display(plan.frame_id)
            # Bound the display copy before transforming. Controller poses and
            # their exact endpoint remain untouched.
            points = [dict(x=p.x, y=p.y, z=p.z) for p in display_path(plan.poses)]
            local_points = [point(p, transform) for p in points]
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError):
            local_points = []
        state["planned_path"] = local_points
    if global_plan is not None:
        try:
            transform = display(global_plan.frame_id)
            points = [
                dict(x=p.x, y=p.y, z=p.z) for p in display_path(global_plan.poses)
            ]
            global_points = [point(p, transform) for p in points]
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError):
            global_points = []
        state["global_planned_path"] = global_points
        state["local_planned_path"] = local_points
        # Bridge state may still retain the controller's completed chunk while
        # the next native refinement RPC is pending. Keep the compatibility
        # path aligned with the executable local window instead of replaying
        # that stale chunk beside the persistent global route.
        state["planned_path"] = local_points
    elif retained is not None:
        # Legacy full FollowPath routes remain both the compatibility and
        # global display path.
        state["global_planned_path"] = local_points
    return state, drifted


def live_state(bridge):
    state = bridge.state()
    planner = getattr(bridge, "objective_planner", None)
    decorate = getattr(planner, "decorate_state", None)
    if callable(decorate):
        state = decorate(state)
    reader = getattr(bridge, "_mapping_authority", None)
    authority = reader.current() if reader is not None else None
    envelope = None
    # Import lazily: legacy ROS 1 deployments need no planning-frame support.
    if getattr(bridge, "_follow_path_display", None) is not None or (
        isinstance(state.get("goal"), dict)
        and state["goal"].get("frame_id", bridge.map_frame).lstrip("/")
        != bridge.map_frame.lstrip("/")
    ):
        source = state
        state, drifted = _display_navigation(bridge, source, authority)
        if drifted:
            # The envelope is projected through this same authority, so its
            # route must use the fresh transform rather than the 2D anchor.
            envelope = _display_navigation(bridge, source, authority, False)[0]
    if envelope is None:
        envelope = state
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
            "goal": envelope.get("goal"),
            **{name: display_path(envelope.get(name) or []) for name in PATH_FIELDS},
        }
        state["live_mapping"] = validate_live_mapping(payload, bridge.id)
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
        # Legacy adapters remain useful without qualified replica overlays.
        pass
    return state
