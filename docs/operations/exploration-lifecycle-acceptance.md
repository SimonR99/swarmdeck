# Exploration execution and recovery acceptance

This change addresses the Bistro deployment previously served on port 15173.
The old simulation logs showed paths cancelled by peer reservation loss every
7–10 seconds, PCI replanning before Nav2 had finished, and R3 repeatedly failing
Nav2's progress check. R3's escape also reported no progress against geometry;
these logs do not identify the particular mesh or prop.

## Corrections

- Map correction metadata no longer invalidates an unchanged reservation.
  Component changes, stale authority, and material cumulative transform changes
  still invalidate it.
- SwarmDeck enables PCI external execution. The full-path controller owns arrival
  and movement failure; PCI cannot replace an active path on proximity alone.
- Actual controller success requests the next path. Late service replies cannot
  discard newer executing paths or paths awaiting peer arbitration. Manual
  commands and Stop All retain ownership while the next plan is pending.
- The default allows two replacement paths: at most three consecutive failed
  movement attempts. Replacement path revisions do not replenish the budget.
  The recovery planning window is 15 seconds; an executing action may finish.
- Empty or near-terminal planner paths wait with capped exponential retry delays
  instead of declaring local exploration complete. This does not implement a
  proof of full-map coverage or guarantee that a distant frontier is reachable.
- Concurrent detector requests fail promptly with HTTP 503 while inference is
  busy, preventing timed-out camera requests from accumulating behind the model.

## Regression validation

The focused adapter, ROS 2, coordination, objective, detector, and launch-config
suite passed **205 tests**. The exact benchbot MGG image passed **8 native tests**,
including an accepted path whose robot moves inside PCI's old arrival radius
without generating another plan until explicitly requested, repeated empty
plans, near-terminal filtering, and Stop followed by a rejected late replan.

The planner image is `swarmdeck-mgg:exploration-lifecycle-review`, built directly
on benchbot from `swarmdeck-mgg:diagnostics-10` with networking disabled. Image ID:
`sha256:e95b853b276baeb867f96dc9fb3646fc1e554d49ec6f2939f72a6b70f974cb1e`.
The native patch SHA-256 is
`7024661dd03000d66b10e78e26a8aa5dda5ef1667a4e401261c5043f80aebda0`.

## Bistro run and deployment

A 120-second Fleet Explore run used mission
`7fb45d65-7594-4f59-b85d-ddd476639e70`, ROS domain 201, native MOLA mapping,
and the patched PCI. All four robots accepted exploration. R0 and R2 ended
about 25 m from their initial positions; R3 moved about 19 m before controller
failures exhausted its recovery window after two failed attempts. R1 completed
one short path, then remained waiting because its local grid found no reachable
continuation. This remains a planner/terrain limitation; full coverage was not
achieved. Stop All stopped all four robots.

The ROS trace recorded 15 nonempty paths, with 8–36 ordered poses. R0 and R2
completed seven controller goals between them. Some reservation cancellations
remained (three for R3 and two for R0); the run does not establish their exact
cause. The metadata-only regression is covered by unit tests, and an additional
idle authority trace showed heartbeat gaps below 1.32 seconds for all robots.
Further investigation should record authority transitions during motion.

The retained deployment is in
`/home/sroy/workspaces/swarmdeck-exploration-review` on benchbot, Compose project
`planning-next`, with UI **15173**, API **18080**, SLAM **18090**, and media **8190**.
`bash compose.sh up -d --no-build` operates on that deployment. All older
`planning` and `swarmdeck` services and the obsolete build helper were stopped;
source checkouts, sessions, and volumes were preserved.
