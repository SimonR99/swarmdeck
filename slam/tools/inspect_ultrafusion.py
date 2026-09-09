#!/usr/bin/env python3
"""Record static evidence from the pinned Ultra-Fusion ROS 2 release.

Uses dpkg-deb and GNU binutils; never installs or executes the release binaries.
This is an inspection tool, not a decompiler or an odometry implementation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

RELEASE_URL = (
    "https://github.com/sjtuyinjie/Ultra-Fusion/releases/download/v0.2.2/"
    "ultrafusion-ros2_0.2.2_amd64.deb"
)
RELEASE_SHA256 = "243f88fa5e3d87fcd96a2b02c8561fca4d5e56419ab72ec0a3a731b0ea34cccc"
ARTIFACTS = ("bin/uf_node", "bin/uf_ros2_adapter", "lib/libultra_lib.so")


def run(*args: str) -> str:
    return subprocess.check_output(
        args, text=True, errors="replace", env={**os.environ, "LC_ALL": "C"}
    )


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def inspect(package: Path, output: Path) -> None:
    package_hash = digest(package)
    if package_hash != RELEASE_SHA256:
        raise ValueError(
            "package SHA256 does not match the pinned ROS 2 v0.2.2 release"
        )
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"release_url": RELEASE_URL, "sha256": package_hash, "artifacts": []}
    with tempfile.TemporaryDirectory(prefix="swarmdeck-uf-inspection-") as extracted:
        run("dpkg-deb", "-x", str(package.resolve()), extracted)
        root = Path(extracted) / "opt/ultrafusion"
        for relative in ARTIFACTS:
            artifact = root / relative
            header = run("readelf", "-hW", str(artifact))
            sections = run("readelf", "-SW", str(artifact))
            dynamic = run("readelf", "-dW", str(artifact))
            symbols = run("nm", "-D", "-C", "--defined-only", str(artifact))
            strings = run("strings", "-a", str(artifact))
            for suffix, content in (
                ("header.txt", header),
                ("sections.txt", sections),
                ("dynamic.txt", dynamic),
                ("symbols.txt", symbols),
            ):
                (output / f"{artifact.name}.{suffix}").write_text(content)
            source_paths = sorted(
                {
                    line.split("/Ultra-Fusion/", 1)[1]
                    for line in strings.splitlines()
                    if "/Ultra-Fusion/" in line
                    and line.endswith((".cpp", ".hpp", ".h"))
                }
            )
            function_count = sum(
                bool(re.match(r"^[0-9a-f]+ [TW] ", line))
                for line in symbols.splitlines()
            )
            manifest["artifacts"].append(
                {
                    "path": relative,
                    "bytes": artifact.stat().st_size,
                    "sha256": digest(artifact),
                    "machine": re.search(r"Machine:\s+(.+)", header).group(1),
                    "has_debug_info": bool(re.search(r"\s\.debug_info\s", sections)),
                    "has_static_symbol_table": bool(
                        re.search(r"\s\.symtab\s", sections)
                    ),
                    "exported_function_symbols_T_W": function_count,
                    "needed": re.findall(r"\(NEEDED\).*\[(.*?)\]", dynamic),
                    "embedded_source_paths": source_paths,
                }
            )
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    parser.add_argument("output", type=Path, help="new directory for evidence files")
    args = parser.parse_args()
    try:
        inspect(args.package, args.output)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"inspection failed: {error}\n")


if __name__ == "__main__":
    main()
