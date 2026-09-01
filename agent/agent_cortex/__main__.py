"""Cortex Microservice CLI Entry Point."""

from __future__ import annotations

import argparse
import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="SwarmDeck Cortex AI Agent Microservice")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8085, help="Bind port (default: 8085)")
    parser.add_argument("--reload", action="store_true", help="Enable live code reload")

    args = parser.parse_args()
    uvicorn.run("agent_cortex.server:app", host=args.host, port=args.port, reload=args.reload)


if __name__ == "__main__":
    main()
