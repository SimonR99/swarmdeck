"""Entry point: python -m swarmdeck_server [--config configs/4robot.yaml]"""
from __future__ import annotations

import argparse

import uvicorn

from .api.app import app, load_config


def main() -> None:
    ap = argparse.ArgumentParser(prog="swarmdeck_server")
    ap.add_argument("--config", default=None, help="path to a fleet config yaml")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    cfg = load_config(args.config)
    print(f"[swarmdeck] config: {cfg.get('name', '(none)')}  robots={cfg.get('fleet', {}).get('robot_count')}")
    print(f"[swarmdeck] http://{args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
