# Coordinated multi-robot exploration

The default Gazebo exploration path is a joint frontier planner, implemented as
a thin ROS node in `coordinated_explore.py` over deterministic, ROS-free
primitives in `frontier_planner.py`. The original reactive wanderer remains an
explicit A/B baseline (`explore_strategy:=reactive`).

## Information boundary

The planner subscribes to each robot's occupancy map and resolves
`map -> odom -> base_link` through TF. It does not subscribe to Gazebo model
states, ground truth, or raw wheel odometry. Configured start poses are coarse
survey data used for the initial rendezvous and common grid coordinates; the
SLAM service must geometrically verify a shared component before exploration
can leave the rendezvous phase. Ground truth is recorded and read only by
post-run evaluation tools.

This boundary matters on hardware: a noisy encoder can disturb the robot's
local navigation stack, but it cannot directly assign a global frontier, merge
two maps, or become a collaborative pose-graph factor.

## Planner

1. A short, bounded rendezvous creates overlapping observations. Up to three
   observations are used when needed; exploration begins as soon as the SLAM
   status reports one verified four-robot component.
2. Per-robot maps are projected into a shared grid. Conservative free space is
   used for frontier viewpoints and reservations; an optimistic free-space
   union is used only for coarse reachability so one robot's stale unknown cells
   do not veto a corridor another robot has mapped.
3. Frontiers are connected components of known free cells adjacent to unknown
   space. Viewpoints are eroded by the rectangular robot half-width plus 0.12 m,
   while Nav2 validates the full footprint and local obstacles.
4. A joint assignment minimizes geodesic travel and rewards information gain.
   Active goals reserve their sensing discs, preventing teammates from mapping
   the same patch. An idle robot defers a long deadhead when a teammate already
   approaching that region can inherit it within 4 m.
5. Goals are cancelled when a teammate observes their frontier, failed goals
   are temporarily quarantined, and exploration terminates only after no
   reachable frontier remains for a dwell period.

This follows the same broad decomposition used by recent multi-robot work:
global region/frontier allocation plus local collision-safe execution, explicit
communication/goal coordination, and loop-closure-aware rendezvous. Relevant
references are [MGG Planner](https://mistlab.ca/MGGPlanner/),
[MUI-TARE](https://arxiv.org/abs/2209.10775),
[RACER](https://arxiv.org/abs/2209.08533), and the 2026 LPFE collaborative-SLAM
work. The implementation here is deliberately smaller and fully auditable; it
does not claim those papers' algorithms as its own.

## Measured four-robot run

`coordinated-gt-run-10` completed in 222.4 s with one unreachable residual
frontier:

| Metric | Result |
|---|---:|
| Known union | 512.763 m² |
| Aggregate path | 85.819 m |
| Coverage efficiency | 5.975 m²/m |
| Redundant known fraction | 0.5774 |
| Longest robot path | 28.487 m |
| Failed goals | 1 |

Against the preceding coordinated run, aggregate path fell from 90.63 m to
85.82 m, completion time from 324.3 s to 222.4 s, and redundant fraction from
0.7325 to 0.5774 while mapped union increased from 496.7 m² to 512.8 m². The
handoff rule removed the observed 22.6 m and 18.1 m end-of-run deadheads.

The accompanying reconstruction places all 288 keyframes in one four-robot
component with 2.50 cm joint translation RMSE. See
[Odometry-free reconstruction](odometry-free-keyframe-reconstruction.md) for
the complete pose, map, and odometry-removal evidence.

## Run and evaluate

The Compose/launch default is coordinated exploration:

```bash
EXPLORE_STRATEGY=coordinated EXPLORE_SECONDS=600 make up-sim
```

Metrics are written atomically to `sessions/exploration-latest.json`. For a
labelled simulation session, run `scripts/record_ground_truth.py` as a separate
evaluation process and score the saved reconstruction with
`slam/tools/evaluate_odom_free.py` and `slam/tools/score_map_vs_world.py`.

Hardware release still requires time-synchronized bags and surveyed or mocap
truth. The Gazebo result demonstrates the architecture and catches aliases; it
does not establish real-world SOTA performance by itself.
