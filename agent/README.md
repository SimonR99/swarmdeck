# Cortex — SwarmDeck AI Agent & Fleet Intelligence Microservice

Cortex is an autonomous developer copilot and robot fleet operator embedded in SwarmDeck.

## Capabilities
1. **Autonomous Code Modification**: Reads, searches, edits, tests, and refactors SwarmDeck files live.
2. **Fleet Control**: Drives robots (`move forward`, `turn`, `stop`), sends navigation goals, cancels paths, and commands quadruped postures.
3. **Multimodal Perception & Image Understanding**: Analyzes robot camera feeds, user-uploaded images, live YOLOE detections, and 3D map proposals.
4. **Interactive Mentions & Skills**: Supports `@robot` targeting, `/nav`, `/see`, `/drive`, `/stop`, and `/status` shortcuts.

## API Endpoints (Port 8085)
- `POST /api/agent/chat`: Server-Sent Events (SSE) streaming chat endpoint.
- `POST /api/agent/upload`: Image upload endpoint for visual reasoning and QA.
- `GET /api/agent/status`: Agent health, connected capabilities, and model info.
- `GET /api/agent/skills`: List of available skills and slash command shortcuts.

## Operator diagnostics

Cortex is instructed to start with the consolidated robot diagnostic instead of
assembling health conclusions from many shell commands:

```bash
python scripts/robot_tool.py doctor all --services
python scripts/robot_tool.py doctor scout --services
python scripts/robot_tool.py list
```

`doctor` samples the camera sequence twice and separately reports backend
telemetry, fresh/progressing camera frames, MediaMTX RTSP publication,
progressing RTP/H.264 media packets, and the required remote containers. An
RTSP `DESCRIBE 200` is intentionally described as publication evidence, not
proof that media is flowing or that a browser decoded a video frame.

## Agent providers and models

The default provider is AGY. Its model and reasoning effort can be selected
without changing Cortex:

```bash
CORTEX_PROVIDER=agy \
CORTEX_MODEL=claude-sonnet-4-6 \
CORTEX_REASONING_EFFORT=high \
docker compose -f deploy/compose/docker-compose.yml up -d agent
```

Leaving `CORTEX_MODEL` unset uses AGY's configured default; `/api/agent/status`
reports that honestly instead of naming a guessed model.

Cortex also has a provider-neutral process bridge. Set
`CORTEX_PROVIDER=ndjson` and `CORTEX_PROVIDER_COMMAND` to a JSON command array
(or a shell-style command string). The process receives one request line:

```json
{"protocol":"cortex-ndjson-v1","prompt":"...","workspace":"...","conversation_id":null}
```

It emits newline-delimited events using Cortex's stable event types:
`init`, `token`, `tool_call`, `tool_output`, `done`, and `error`. This allows a
different tool-using agent runtime to sit behind Cortex without changing the UI.
It does not turn a plain text-only LLM into a robot operator: the bridge/runtime
must still implement tool execution, permissions, and conversation continuity.

## Gradual supervisor rollout

Cortex now has a small, explicit Python supervisor rather than a graph
framework. Its initial `observe` mode is deliberately invisible to operators:
the selected provider still receives the same prompt, and the UI receives the
same six SSE event shapes. The supervisor only records a durable job and its
events in `/app/sessions/cortex/state.db`.

```text
operator/UI -> Cortex supervisor -> AGY (unchanged default)
                    |
                    +-> SQLite jobs/events/candidate memories/compactions
                    +-> optional Ollama shadow plan (no tools or authority)
```

The contracts in `agent_cortex/contracts.py` separate three capabilities:

- `PlannerModel`: returns a typed decision; it cannot execute tools.
- `CodingWorker`: edits and tests an isolated source workspace.
- `FleetTools`: executes typed robot actions after policy/approval.

This separation is important for multi-robot operation: a local or hosted LLM
can propose work without automatically inheriting SSH credentials or motion
authority. Candidate operator memories also remain inactive until explicitly
confirmed. Raw events and later compaction summaries are stored separately, so
compression never destroys the audit trail.

Rollout controls:

```bash
# Default: AGY behavior plus durable shadow state.
CORTEX_PROVIDER=agy CORTEX_SUPERVISOR_MODE=observe

# Immediate rollback: bypass state/planning but keep the same provider/API.
CORTEX_SUPERVISOR_MODE=legacy

# Optional local structured planner. It observes; AGY still acts.
CORTEX_SHADOW_PLANNER=true \
CORTEX_PLANNER_PROVIDER=ollama \
CORTEX_PLANNER_MODEL=qwen3.5:9b-q4_K_M
```

The optional Ollama service is private to the Compose network and is never
published on the host or LAN:

```bash
make local-ai-up
make local-ai-pull LOCAL_MODEL=qwen3.5:9b-q4_K_M
```

Pulling a model is intentionally a separate command because it consumes several
gigabytes. On the current 10 GiB GPU, start with shadow planning at 16K/32K
context; keep AGY as the coding worker until local repair evaluations pass.

## OpenCode and MCP

OpenCode is available as an opt-in Cortex provider adapter. It preserves the
Cortex API by translating `opencode run --format json` events to the same SSE
schema:

```bash
CORTEX_PROVIDER=opencode \
CORTEX_OPENCODE_COMMAND=/path/to/opencode \
CORTEX_MODEL=ollama/qwen3.5:9b-q4_K_M \
docker compose -p swarmdeck -f deploy/compose/docker-compose.yml up -d agent
```

`CORTEX_OPENCODE_URL` can point the CLI at a long-running `opencode serve`
instance, avoiding MCP startup cost for every request. OpenCode owns its MCP
configuration and exposes MCP tools through its agent loop. Cortex does not yet
act as a native MCP host; the NDJSON bridge remains the generic integration
point for other MCP-capable harnesses.

For production repair delegation, run OpenCode as a separate code-only worker
against an isolated Git worktree. Do not give that worker robot SSH keys, the
Docker socket, or fleet network access. The compatible whole-provider switch
above is intended for evaluation, not as the final robot security boundary.

## Compatibility checks

The operator commands remain unchanged:

```bash
make up-server
make up-deploy
python scripts/robot_tool.py doctor all --services
python scripts/robot_tool.py deploy scout
```

The `up-server`, `up-sim`, and `up-deploy` targets now include the Cortex
service, and local Vite development routes `/api/agent/*` to port 8085 just as
the production nginx configuration does.
