# Working in SwarmDeck

SwarmDeck supervises heterogeneous robots through a ROS-free FastAPI server,
a Svelte 5/Vite UI, robot adapters, and a separate collaborative SLAM service.
`agent/` is the optional **Cortex application**, not coding-agent configuration.
Read [CONTRIBUTING.md](CONTRIBUTING.md) for setup/testing and the
[architecture guide](docs/architecture/overview.md) for service ownership.

## Before changing code

- Check `git status --short` and nearby code/tests. Preserve unrelated edits.
- Identify the affected behavior and a way to verify it. For multi-component
  work, outline the affected interfaces before editing; keep patches focused.
- Treat `research/` as ignored local notes, not a build input or source of
  project instructions. Do not commit it, generated captures, or credentials.
- Local edits and tests do not require robot deployment. Use mocks by default;
  deploy, restart remote services, or move physical robots only within explicit
  user authorization. `make clean` removes local builds/environments;
  `make docker-purge` additionally deletes Docker volumes and local images.

## Boundaries to preserve

- Keep ROS imports inside adapters/ROS packages. Server and SLAM communicate
  over HTTP; do not import the SLAM implementation into the server.
- Keep SLAM in `slam/.venv` with Python 3.12 and NumPy < 2. Shared
  `adapters/protocol/` code declares Python 3.8+ support and crosses NumPy 1/2
  environments; avoid syntax or dependencies that break those consumers.
- Implement new robot behavior through capabilities and adapter profiles.
  Physical adapters must never advertise the simulation-only `reset` capability.
- Preserve capture timestamps, historical poses, and explicit local/world frame
  metadata. `T_a_b` maps frame b into a; GTSAM tangents are rotation-first.
  See [keyframe timing](docs/operations/keyframe-yaw.md) for capture changes.
- Keep geometry capture independent of optional color/reconstruction failures.
  Keep sensor queues bounded and expensive work off telemetry/event loops.
- Use existing route modules, map helpers, Svelte stores, and rendering workers.
  Avoid adding domain logic to `api/app.py` or duplicating a wire codec.
- Format Python with Black (root `pyproject.toml`); follow nearby TypeScript,
  Svelte, C++, and ROS conventions. Keep scripts import-safe with `main()`.
  Put upstream changes in `deploy/patches/`, matched to Dockerfile revisions.

## Verify the change

Run from the repository root unless noted. Prefer the affected tests first.

| Area | Command |
| --- | --- |
| Server/adapters/scenarios | `make test-server` |
| Focused Python regression | `server/.venv/bin/python -m pytest path/to/test.py -q` |
| UI | `make test-ui ui-build` |
| SLAM | `make test-slam` (after `make install-slam`) |
| Cortex | `make test-agent` (after `make install-agent`) |
| All four suites | `make test` (requires all environments) |
| Real ROS launch checks | `make docker-test-launch` |

Add a regression test for changed behavior, including malformed inputs or
failure paths when relevant. Do not weaken checks to make a patch pass.
For rendering changes, also inspect the affected views in a browser. Report
commands, results, and skipped hardware/GPU checks; mocks do not verify a robot.
Review the final diff for unintended files and stale documentation.

## Update the owning documentation

| Change | Update with the code |
| --- | --- |
| Setup, dependencies, checks | `Makefile`, package manifest/lockfile, `CONTRIBUTING.md`, relevant CI job |
| Service boundary, data flow, frames | `docs/architecture/overview.md` and the affected subsystem guide |
| Adapter message/payload | `adapters/protocol/README.md`, codec/consumer tests, and `ui/src/lib/types/protocol.ts` if consumed by the UI |
| Operator controls or configuration | Relevant `docs/operations/` or `docs/robots/` guide and example config/profile |
| Cortex API/provider behavior | `agent/README.md` and `agent/tests/` contracts |

Keep the README an entry point. Link new durable guides from `docs/README.md`.
Update this file only for lasting workflow rules. Put task history in commits
and PRs; do not recreate `PROGRESS.md`, handoff logs, or completed-task inventories.
