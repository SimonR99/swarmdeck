"""Inspect or collect expired unreferenced replica chunks without retiring maps."""

import argparse
import json

from .replication import ReplicaStore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root", help="server replica directory, containing replicas.sqlite"
    )
    parser.add_argument("--retention-seconds", type=float, default=3600)
    parser.add_argument(
        "--limit", type=int, default=256, help="maximum chunks in this batch (1–4096)"
    )
    parser.add_argument(
        "--collect",
        action="store_true",
        help="delete eligible chunks; default is a dry run",
    )
    args = parser.parse_args()
    store = ReplicaStore(args.root, retention_s=args.retention_seconds)
    try:
        print(
            json.dumps(
                store.collect_unreferenced(limit=args.limit, dry_run=not args.collect)
            )
        )
    finally:
        store.close()


if __name__ == "__main__":
    main()
