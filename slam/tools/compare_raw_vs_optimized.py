"""Base SLAM vs optimized: does the pose graph IMPROVE the robot's own solution?

    cd slam && .venv/bin/python tools/compare_raw_vs_optimized.py [dataset] [last_n]

Needs no ground truth, which is what makes it usable on hardware captures. It
asks the only question that matters when a robot's own SLAM is already good:
how far does the graph MOVE it? A pose graph that displaces a working solution
by metres is not refining it.

This exists because a gate was once tuned by counting how many loop closures it
accepted, without checking whether they were right. Closure count went up and
the map was destroyed. Validate against pose displacement, not recall.

Raw = the robot's own SuperOdometry pose (t_odom_base, i.e. T_map_base).
Optimized = what the pose graph produces. If raw is good and optimized is not,
the graph is corrupting a working solution and the question is which gate let
that happen.

Measured PER TRAJECTORY, against that trajectory's own fitted T_world_map. The
graph is free to place a whole segment anywhere -- gauge is arbitrary, and a
segment that arrived after a reboot has no relation to the previous one's frame
at all -- so a displacement measured against a single frame for the whole robot
reports the gauge choice rather than any deformation of the solution. On a
capture spanning a reboot that difference is tens of metres of pure noise, and
it points the wrong way: it makes a correct re-merge look like a destroyed map.

Map sharpness proxy: occupied cell count at fixed resolution. A trajectory that
folds onto itself smears one wall into many, so occupied cells GROW while the
true structure does not.
"""

import collections
import pathlib
import sys

import numpy as np
from dataclasses import replace
from swarmdeck_protocol import decode_keyframe
from swarmdeck_slam.backend import CollaborativeBackend, PRODUCTION_VERIFY
from swarmdeck_slam.render import RenderConfig, render_occupancy, OCCUPIED
from swarmdeck_slam.types import Component, EdgeKind, OptimizedGraph, se3_distance

DATASET = sys.argv[1] if len(sys.argv) > 1 else "../sessions/captures/hw-run-02"
LAST_N = int(sys.argv[2]) if len(sys.argv) > 2 else 145
blobs = sorted(pathlib.Path(DATASET, "keyframes").glob("*.kf"))[-LAST_N:]
packets = []
for f in blobs:
    try:
        packets.append(decode_keyframe(f.read_bytes()))
    except Exception:
        pass
seqs = [p.seq for p in packets]
print(
    f"  {len(packets)} blobs, robots={sorted({p.robot_id for p in packets})}, "
    f"seq {min(seqs)}..{max(seqs)}"
)

RENDER = RenderConfig(floor_z=0.0, min_z=0.08, max_z=2.20, native_map_resolution=0.05)


def occupied(graph, kfs):
    grids = render_occupancy(graph, kfs, RENDER)
    g = max(grids.values(), key=lambda x: x.width * x.height)
    return int((g.cells == OCCUPIED).sum()), g.width, g.height


def displacement(backend, graph):
    """Per trajectory: how far the solver moved each pose from the robot's own
    solution, with that trajectory's own frame taken out first."""
    by_trajectory = collections.defaultdict(list)
    for kf_id in graph.poses:
        by_trajectory[kf_id.trajectory].append(kf_id)
    for trajectory in sorted(by_trajectory):
        frame = graph.t_world_trajectory[trajectory]
        deltas = [
            se3_distance(graph.poses[k], frame @ backend._keyframes[k].t_odom_base)
            for k in by_trajectory[trajectory]
        ]
        translations = [d[0] for d in deltas]
        rotations = [np.degrees(d[1]) for d in deltas]
        yield trajectory, len(deltas), translations, rotations


for mme in (1.0, 0.15):
    be = CollaborativeBackend(
        verify=replace(PRODUCTION_VERIFY, max_mean_error=mme), render=RENDER
    )
    for i, p in enumerate(packets, 1):
        be.ingest_packet(p)
        if i % 25 == 0:
            be.optimize_and_render()
    snap = be.optimize_and_render()
    opt = snap.optimized
    kfs = list(be._keyframes.values())

    # RAW: the robot's own SLAM poses, as a graph. One component per
    # trajectory, because two segments of one robot are exactly as unrelated
    # as two robots until something proves otherwise.
    trajectories = sorted({k.id.trajectory for k in kfs})
    raw = OptimizedGraph(
        poses={k.id: k.t_odom_base for k in kfs},
        components=[
            Component(
                i,
                frozenset({t.robot_id}),
                min(k.id for k in kfs if k.id.trajectory == t),
                frozenset({t}),
            )
            for i, t in enumerate(trajectories)
        ],
    )
    nloop = sum(1 for e in be._graph._edges if e.kind is not EdgeKind.ODOMETRY)
    o_opt, w1, h1 = occupied(opt, kfs)
    o_raw, w2, h2 = occupied(raw, kfs)
    print(f"\n  max_mean_error={mme}")
    print(f"    trajectories         : {[str(t) for t in trajectories]}")
    print(
        f"    loop closures        : {nloop}   rejected by graph: {len(opt.rejected_edges)}"
    )
    for trajectory, n, trans, rot in displacement(be, opt):
        print(
            f"    moved {str(trajectory):<24} n={n:>4}  median {np.median(trans):.2f} m / "
            f"{np.median(rot):.1f} deg, max {max(trans):.2f} m / {max(rot):.1f} deg"
        )
    print(f"    occupied cells  RAW  : {o_raw:>7}  ({w2}x{h2})")
    print(
        f"    occupied cells  OPT  : {o_opt:>7}  ({w1}x{h1})   ratio {o_opt/max(o_raw,1):.2f}x"
    )
