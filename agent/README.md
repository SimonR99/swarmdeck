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
