# Contributing to SwarmDeck

Start with the [README](README.md) for setup and the
[documentation index](docs/README.md) for subsystem guides.

## Working on a change

Keep changes focused and explain the observable behavior they improve. Use a
separate branch or worktree when developing alongside another contributor.
Run commands from that checkout's root so Compose builds the intended source.
Do not commit generated sessions, local deployment credentials, model weights,
or private reconstruction dependencies.

The server and adapter protocol run without ROS. Keep ROS imports in the robot
bridges. Keep the Python 3.12 / NumPy < 2 SLAM environment separate from the
server environment. UI packages are locked in `ui/package-lock.json`.

For mapping changes, preserve coordinate-frame and timestamp contracts. Robot
local maps and world-aligned SLAM clouds are different inputs; the tactical view
uses world coordinates. Pair captures with historical poses and keep optional
color/reconstruction failures independent of geometry capture. See
[capture timing](docs/operations/keyframe-yaw.md) and
[3D mapping](docs/operations/tactical-3d-map.md).

## Validation

Choose tests for the behavior being changed:

```bash
# Frontend types, terrain preparation, frame transforms, and buffer reuse
make test-ui
make ui-build

# ROS-free capture and pose regressions
server/.venv/bin/python -m pytest \
  adapters/test/test_adapter_sim_pose.py \
  adapters/test/test_adapter_sim_keyframes.py \
  adapters/test/test_keyframe_producer.py -q

# Cloud transport, calibrated projection, and reconstruction export
server/.venv/bin/python -m pytest server/tests/test_reconstruction.py -q

# Collaborative SLAM (its own environment)
make test-slam
```

`make test` runs the default Python test selection, the SLAM suite, and frontend
checks/tests. ROS launch checks are available through `make docker-test-launch`;
end-to-end simulation checks are in `tests/integration/`.

Rendering changes also need a browser check. Exercise 2D/3D switching, local and
global maps, ceiling clipping, and each affected representation. Check behavior
with missing RGB or Gaussian data. Workload limits and synthetic test timings
are not evidence of sustained performance on a particular GPU.

## Diagnostics and upstream patches

Keep operational warnings actionable and rate-limited. Avoid per-frame logging
in sensor, map, and rendering loops; use focused regression tests or the
[simulation benchmark](scripts/benchmark-sim.py) for investigations. Keep local
captures and temporary reproductions outside tracked source. Document current
contracts and reproducible commands; Git history holds resolved incident detail.

Keep command-line scripts import-safe, with argument parsing in `main()` and
reusable processing functions. Upstream source changes belong in
`deploy/patches/` and must match the revisions pinned in the Dockerfiles. See the
[ARGoS patch guide](deploy/patches/argos/README.md) for physics checks and
[MGG exploration](docs/operations/mgg-exploration.md) for planner lifecycle checks.

## Pull requests

Describe the problem, the resulting behavior, and the tests you ran. Include
screenshots for visible UI changes and relevant timing measurements for
performance changes. Update documentation when commands, frame contracts,
configuration, or operator behavior change. Report hardware-dependent checks
that you could not run.
