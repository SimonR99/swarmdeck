# Archive

These documents are kept for their evidence and their reasoning. Their content
is unchanged. They are superseded by [the plan](../plan.md) and the
[acceptance log](../operations/acceptance-log.md), which are the current
references. Do not update the files here; add to the plan or the log instead.

Because their text is unchanged, their relative links still point at the
locations these files had before the move, so 11 of them no longer resolve. The
table below says where each document went and what replaces it. A repository
link check should exclude this directory.

| Document | What it was | Superseded by |
| --- | --- | --- |
| [decentralized-autonomy-plan.md](decentralized-autonomy-plan.md) | The full decentralized autonomy design: contracts, frames, mapping products, MGG refactor, peer collaboration, replication, and phases 0 to 7 with 6G and 6N | `docs/plan.md` |
| [planning-refactor-remaining.md](planning-refactor-remaining.md) | The live tracker: the priority 1 to 6 order of implementation, work split, capture provenance gate and remaining qualification | `docs/plan.md` |
| [navigation-mapping-issue-plan.md](navigation-mapping-issue-plan.md) | The September 13 fleet regression plan: parallel lanes, the NAV, HOME, MAP and RECON issue register, the acceptance matrix, and a long debugging chronology | `docs/plan.md` and the acceptance log |
| [roadmap.md](roadmap.md) | Implementation status and priority work from the Gazebo and central SLAM era | `docs/plan.md` |
| [PROGRESS.md](PROGRESS.md) | The August central pose-graph optimizer design, component ports, fleet profiles and verification commands | `docs/plan.md` |
| [collaborative-mapping-plan.md](collaborative-mapping-plan.md) | The central GTSAM pose-graph service design behind `merge_mode: graph` | `docs/plan.md`; the service remains at `:8090` as diagnostics only |
| [collaborative-slam.md](collaborative-slam.md) | The initial grid-registration and Swarm-SLAM integration analysis, recording why those approaches were superseded | `docs/plan.md` |
| [coordinated-exploration.md](coordinated-exploration.md) | The joint frontier planner for the retired Gazebo simulation path, with its measured four-robot run | `docs/plan.md`; the live simulator is ARGoS |
| [decentralized-autonomy.md](decentralized-autonomy.md) | The onboard pipeline build, test, deployment and replication record, with its validation chronology | Operational parts by `docs/operations/current-stack.md` and `docs/operations/mola-runtime.md`; trials by the acceptance log |
| [navigation-live-map-acceptance.md](navigation-live-map-acceptance.md) | The navigation and live-map deployment acceptance record: every Benchbot mission, image identity, probe and arrival result from 2026-09-10 to 2026-09-15 | The acceptance log |
| [planning-parallel-acceptance.md](planning-parallel-acceptance.md) | The 2026-09-10 mapping, exploration and display acceptance record | The acceptance log |
| [exploration-lifecycle-acceptance.md](exploration-lifecycle-acceptance.md) | The exploration execution and recovery acceptance record, with its Bistro run and retained deployment | The acceptance log |
