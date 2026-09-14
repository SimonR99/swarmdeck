# Shared route tracking

`adapters.route_tracking.RouteTracker` follows a polyline by distance along the
route. Both accepted progress and target distance are monotonic. A bounded local
projection prevents jumping onto a distant return leg of a looping route. The
lookahead can cross a vertex; the robot need not return to visit the vertex after
local obstacle avoidance.

Every proposed connection from the live pose to the lookahead passes an occupancy
check. `GridCollisionChecker` uses the original downloaded map in the robot's
navigation frame, with conservative square clearance matching the grid planner.
Unknown space and boundaries block shortcuts. No occupied cells are cleared from
this map, including beneath the robot. The native local collision controller
continues checking live sensors.

All ROS 1, ROS 2, and simulator adapters inherit
`navigation_route_target(path, pose)` from `AdapterTelemetryMixin`. It returns an
`{x, y}` target or `None` when no checked connection is available. Callers must
hold motion on `None`; the shared native-velocity relay also observes the hold.
Set `_nav_route_key = None` when beginning a new goal, even if its path matches the
previous goal. Both path and pose must use the downloaded map's navigation frame.

Scout uses this API for its joystick-driven local planner. Nav2 robots continue
using their native path followers; their adapters have the shared API available
for future waypoint-based controllers without replacing Nav2's recovery behavior.

Defaults are 0.8 m lookahead and the robot's configured footprint radius for
clearance. `nav_route_lookahead_m` and `nav_route_clearance_m` override those values.
Scout uses the same 0.35 m corridor clearance as its existing global planner.
Maps must have been successfully fetched or revalidated within ten seconds.
Missing/stale maps or blocked connections hold Scout, retrying on later state
updates; they do not trigger an unchecked shortcut or a reverse-to-vertex fallback.
A persistent hold requires a new route or changed map; this tracker does not run
an A* replan itself.

Regression coverage includes recorded Scout doorway poses with reconstructed
route geometry on a free test grid, occupied/unknown corners, footprint clearance,
map boundaries, repeated points, looping routes, stale maps, and suppression of
native velocity while held. The replay does not include the historical sensor map
and cannot substitute for a physical doorway test.
