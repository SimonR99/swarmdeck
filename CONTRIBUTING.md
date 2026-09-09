# Contributing to SwarmDeck

Start with the [README](README.md), [architecture](docs/architecture/overview.md),
and the guide for the component you are changing. [AGENTS.md](AGENTS.md) contains
coding-agent constraints and the table of documentation to update with code.

## Setup and checks

Run commands from your checkout's root. Python 3.10+ and Node.js 22.12+ are
required; SLAM needs Python 3.12 and `uv`.

```bash
make install                      # server/.venv + npm ci in ui/
make demo                         # local server, mock adapter, and UI
make install-agent install-slam    # separate optional-service environments
```

| Change | Validation |
| --- | --- |
| Server, shared protocol, adapters, scenario logic | `make test-server` |
| Focused Python behavior | `server/.venv/bin/python -m pytest path/to/test.py -q` |
| UI | `make test-ui ui-build` |
| Collaborative SLAM | `make test-slam` |
| Cortex/provider or fleet-tool contract | `make test-agent` |
| All four suites | `make test` after installing all environments |
| Python formatting | `uvx black --check --diff path/to/changed.py` |
| ROS launch descriptions in the simulation image | `make docker-test-launch` |

`make test-server` uses the root `pytest.ini`: server, adapter, simulation, and
bring-up tests. ROS-dependent launch checks skip when ROS is unavailable.
Bistro scene checks use real mesh heights and lamps from separately distributed
assets; they skip when those assets are absent. Set `SWARMDECK_BISTRO_DIR` as
explained in [simulation setup](docs/architecture/simulation.md), then run:

```bash
server/.venv/bin/python -m pytest \
  swarmdeck_ros/src/swarmdeck_sim/test/test_make_argos_session.py -q -rs
```

A skipped scene/ROS check is not evidence that simulation works. Native and
end-to-end checks live in `tests/integration/`; perception model validation is
in `tests/perception/`. Read each check's prerequisites before running it.

Keep ROS shell variables out of non-ROS Python commands. Make targets unset
`PYTHONPATH`, `AMENT_PREFIX_PATH`, and `CMAKE_PREFIX_PATH`; do the same for direct
Python commands if your shell has sourced ROS. Do not combine server and SLAM
dependencies into one environment. To change UI dependencies, use npm and commit
both `ui/package.json` and `ui/package-lock.json`; normal setup uses `npm ci`.

## Simulation and cleanup

`make up-sim`, `build-sim`, and `down-sim` all refer to ARGoS. Use
`SCENARIO`, `RENDER`, and `ODOMETRY` instead of per-scenario command aliases;
see [simulation setup](docs/architecture/simulation.md). Host-only tools remain
available directly under `scripts/` and `tests/integration/`.

`make clean` removes local build outputs and dependency environments only.
`make docker-down` stops project containers while preserving volumes.
`make docker-purge` removes containers, named volumes, and local images;
use it only when that data deletion is intended.

## Making a change

1. Describe the observable result and inspect the existing code/tests. Preserve
   unrelated working-tree changes; use an isolated worktree for concurrent work.
2. Make a focused change at the owning component. For bug fixes, add a regression
   that fails for the original behavior. Exercise relevant disconnect, malformed
   input, timestamp, or cancellation paths; avoid tests that merely mirror code.
3. Run affected checks and inspect their output. Keep hardware/GPU evidence
   separate from mocked unit tests. Review the final diff and update the owning
   documentation listed in [AGENTS.md](AGENTS.md).
4. In the PR, explain the problem, resulting behavior, tests run, and checks you
   could not run. Include screenshots for visible changes and measurements for
   performance claims.

Rendering changes need a browser check: exercise the affected 2D/3D modes,
local/global map selection, clipping, and missing optional RGB/Gaussian data.
Use [capture timing](docs/operations/keyframe-yaw.md) and
[3D mapping](docs/operations/tactical-3d-map.md) for frame-sensitive changes.
Avoid per-frame logs; keep operational warnings actionable and rate-limited.

Upstream source changes belong in `deploy/patches/`, matched to Dockerfile
revisions. See the [ARGoS patch guide](deploy/patches/argos/README.md) for physics
checks and [MGG exploration](docs/operations/mgg-exploration.md) for planner checks.
Do not commit credentials, recorded sessions, model weights, generated artifacts,
or ignored `research/` notes. Git and PRs hold task history; docs hold current
contracts and reproducible procedures.

## Why the agent workflow is small

The [AGENTS.md guidance](https://agents.md/) recommends a predictable repository
file with concrete setup, testing, and project conventions. We use one root
file and link to subsystem docs; add a scoped file only when a directory needs
additional rules.

[Anthropic's coding-agent best practices](https://code.claude.com/docs/en/best-practices)
emphasize verifiable outcomes, reading existing code before editing, and concise
persistent instructions. Here that means runnable Make targets, focused
regressions, browser checks for visual work, and a documentation ownership table.

[Anthropic's long-running agent experiments](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)
support incremental changes and explicit verification before declaring success.
We retain those practices, using commits and PRs for handoff history instead of
adding another tracked progress file. CI runs deterministic software checks;
robot deployments and live provider evaluations remain separate activities.
