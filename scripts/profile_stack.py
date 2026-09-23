#!/usr/bin/env python3
"""Measure a running SwarmDeck simulation stack and print a markdown report.

Run on the host that runs the stack, while it runs:

    scripts/profile_stack.py                 # 30 s window, no profiles
    scripts/profile_stack.py --profiles      # plus py-spy of the Python hot spots
    scripts/profile_stack.py --window 60 --label "after lattice cache"

It reports what the clean-up plan (docs/superpowers/plans/
2026-09-23-cleanup-optimization.md) is measured by:

- simulation real-time factor: /clock against wall time;
- CPU per container and per process (top), for the stack's processes only;
- MGG plan cycles: wall time and its lattice ("grid") and gain parts, from
  the planner's own log lines within the window;
- GUI WebSocket traffic: messages and bytes per second by type;
- optionally, py-spy profiles of the Python processes, taken from a
  throwaway sidecar container that joins each container's PID namespace,
  so nothing in the stack is modified.

Only docker and python3 are needed on the host.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import re
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PROJECT = "swarmdeck"
PYSPY_IMAGE = "python:3.12-slim"

# name, container, command-line pattern
PYTHON_TARGETS = (
    ("bridge", "sim", "swarmdeck_argos_bridge.py"),
    ("adapter_sim", "sim", "adapter_sim.py"),
    ("server", "server", "swarmdeck_server"),
    ("cslam_bridge", "peer0", "cslam_bridge.py"),
    ("lidar_handler", "peer0", "lidar_handler_node.py"),
    ("loop_closure", "peer0", "loop_closure_detection_node.py"),
    ("mgg_supervisor", "mgg", "supervisor.py"),
)

PLAN_LINE = re.compile(
    r"\[(robot_\d+)\.mgg\.mggplanner_node\]: .*grid graph: (\d+) free cells, "
    r"(\d+) vertices, (\d+) edges.*?; (\d+) ms \(global (\d+), grid (\d+), "
    r"gain (\d+), select (\d+)\)"
)


def sh(args, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(args, capture_output=True, text=True, **kwargs)


def container(name: str) -> str:
    return f"{PROJECT}-{name}-1"


def running_containers() -> list[str]:
    out = sh(["docker", "ps", "--format", "{{.Names}}"]).stdout.split()
    return sorted(c for c in out if c.startswith(f"{PROJECT}-"))


# -------------------------------------------------------------- real time


RTF_PROBE = r"""
import time, rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); node = rclpy.create_node("swarmdeck_rtf_probe")
seen = []
node.create_subscription(
    Clock, "/clock",
    lambda m: seen.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)),
    qos_profile_sensor_data)
end = time.monotonic() + WINDOW
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
print(len(seen), seen[0][0] if seen else 0, seen[0][1] if seen else 0,
      seen[-1][0] if seen else 0, seen[-1][1] if seen else 0)
"""


def real_time_factor(window: float) -> dict:
    probe = RTF_PROBE.replace("WINDOW", str(window))
    result = sh(
        [
            "docker",
            "exec",
            "-i",
            container("sim"),
            "bash",
            "-c",
            "source /opt/ros/jazzy/setup.bash && python3 -",
        ],
        input=probe,
        timeout=window + 60,
    )
    try:
        n, w0, s0, w1, s1 = result.stdout.split()[-5:]
        n = int(n)
        wall, sim = float(w1) - float(w0), float(s1) - float(s0)
    except ValueError:
        return {"error": (result.stderr or result.stdout).strip()[-300:]}
    if n < 2 or wall <= 0:
        return {"error": f"only {n} /clock messages"}
    return {"rtf": sim / wall, "clock_hz": (n - 1) / wall, "sim_s": sim, "wall_s": wall}


# -------------------------------------------------------------------- CPU


def stack_pids() -> dict[int, str]:
    pids = {}
    for name in running_containers():
        out = sh(["docker", "top", name, "-eo", "pid"]).stdout.split()[1:]
        for pid in out:
            if pid.isdigit():
                pids[int(pid)] = name.removeprefix(f"{PROJECT}-").removesuffix("-1")
    return pids


def cpu_sample(seconds: float) -> tuple[dict[str, float], list[tuple[float, str, str]]]:
    """(per-container CPU %, [(cpu %, container, command)] for stack processes)."""
    pids = stack_pids()
    out = sh(
        ["top", "-b", "-d", str(seconds), "-n", "2", "-w", "512", "-c"],
        timeout=seconds + 30,
    ).stdout
    last = out.split("\ntop - ")[-1].splitlines()
    try:
        header = next(
            i for i, line in enumerate(last) if line.lstrip().startswith("PID")
        )
    except StopIteration:
        return {}, []
    per_container: dict[str, float] = collections.defaultdict(float)
    processes = []
    for line in last[header + 1 :]:
        fields = line.split(None, 11)
        if len(fields) < 12 or not fields[0].isdigit():
            continue
        pid, cpu, command = int(fields[0]), float(fields[8]), fields[11]
        name = pids.get(pid)
        if name is None:
            continue
        per_container[name] += cpu
        processes.append((cpu, name, command))
    processes.sort(reverse=True)
    return dict(per_container), processes


def short_command(command: str) -> str:
    words = command.split()
    for word in words:
        if (
            word.endswith(".py")
            or "/lib/" in word
            or word.startswith("/opt/")
            or "/bin/" in word
        ):
            base = word.rsplit("/", 1)[-1]
            ns = re.search(r"__ns:=(\S+)", command)
            return f"{base} {ns.group(1)}" if ns else base
    return words[0].rsplit("/", 1)[-1] if words else command


# ------------------------------------------------------------------ MGG


def mgg_cycles(since_s: float) -> list[dict]:
    logs = sh(
        ["docker", "logs", "--since", f"{int(since_s) + 1}s", container("mgg")],
    )
    cycles = []
    for line in (logs.stdout + logs.stderr).splitlines():
        match = PLAN_LINE.search(line)
        if match:
            robot, cells, vertices, edges, total, _glob, grid, gain, select = (
                match.groups()
            )
            cycles.append(
                {
                    "robot": robot,
                    "cells": int(cells),
                    "vertices": int(vertices),
                    "edges": int(edges),
                    "total": int(total),
                    "grid": int(grid),
                    "gain": int(gain),
                    "select": int(select),
                }
            )
    return cycles


# ------------------------------------------------------------ websocket


WS_PROBE = r"""
import asyncio, json, time, collections, websockets
async def main():
    sizes = collections.defaultdict(list); t0 = time.time()
    async with websockets.connect("ws://localhost:8080/ws", max_size=None) as ws:
        while time.time() - t0 < WINDOW:
            try:
                m = await asyncio.wait_for(ws.recv(), 1)
            except asyncio.TimeoutError:
                continue
            try:
                kind = json.loads(m).get("type")
            except Exception:
                kind = "?"
            sizes[kind].append(len(m))
    print(json.dumps({k: [len(v), sum(v)] for k, v in sizes.items()}))
asyncio.run(main())
"""


def websocket_traffic(window: float) -> dict:
    result = sh(
        ["docker", "exec", "-i", container("server"), "python", "-"],
        input=WS_PROBE.replace("WINDOW", str(window)),
        timeout=window + 30,
    )
    try:
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        return {"error": [0, 0], "detail": result.stderr[-300:]}


# -------------------------------------------------------------- py-spy


PYSPY_SCRIPT = r"""
pip install -q py-spy >/dev/null 2>&1
pid=$(for p in /proc/[0-9]*; do tr '\0' ' ' < $p/cmdline 2>/dev/null | grep -q -- "$PATTERN" && echo ${p#/proc/}; done | sort -n | head -1)
[ -n "$pid" ] && py-spy record -p $pid -d $SECONDS -r 50 -f raw -o /out/profile.txt --nonblocking >/dev/null 2>&1
chmod a+r /out/profile.txt 2>/dev/null
"""


def pyspy(target_container: str, pattern: str, seconds: int) -> list[tuple[float, str]]:
    with tempfile.TemporaryDirectory() as out:
        script = PYSPY_SCRIPT.replace("$PATTERN", pattern).replace(
            "$SECONDS", str(seconds)
        )
        sh(
            [
                "docker",
                "run",
                "--rm",
                f"--pid=container:{target_container}",
                "--cap-add",
                "SYS_PTRACE",
                "-v",
                f"{out}:/out",
                PYSPY_IMAGE,
                "sh",
                "-c",
                script,
            ],
            timeout=seconds + 180,
        )
        path = Path(out) / "profile.txt"
        if not path.exists():
            return []
        inclusive: collections.Counter = collections.Counter()
        total = 0
        for line in path.read_text(errors="replace").splitlines():
            stack, _, count = line.rpartition(" ")
            if not count.isdigit():
                continue
            total += int(count)
            for frame in set(stack.split(";")):
                if "threading.py" in frame or "<module>" in frame or "runpy" in frame:
                    continue
                inclusive[frame[:110]] += int(count)
        return (
            [(100.0 * c / total, f) for f, c in inclusive.most_common(8)]
            if total
            else []
        )


# ---------------------------------------------------------------- report


def report(args) -> str:
    started = time.monotonic()
    lines = [
        f"## {dt.datetime.now().isoformat(timespec='minutes')}"
        + (f" - {args.label}" if args.label else ""),
        "",
    ]
    containers = running_containers()
    if not containers:
        return "No swarmdeck containers are running."

    rtf = real_time_factor(args.window)
    if "rtf" in rtf:
        lines.append(
            f"- **Real-time factor {rtf['rtf']:.2f}** ({rtf['sim_s']:.1f} s simulated in "
            f"{rtf['wall_s']:.1f} s; /clock {rtf['clock_hz']:.1f} Hz)"
        )
    else:
        lines.append(f"- Real-time factor: not measured ({rtf['error']})")

    per_container, processes = cpu_sample(min(10.0, args.window))
    lines += ["", "| Container | CPU % |", "|---|---:|"]
    for name, cpu in sorted(per_container.items(), key=lambda kv: -kv[1]):
        lines.append(f"| {name} | {cpu:.0f} |")
    lines.append(f"| **total** | **{sum(per_container.values()):.0f}** |")
    lines += ["", "| Process | Container | CPU % |", "|---|---|---:|"]
    for cpu, name, command in processes[: args.top]:
        lines.append(f"| {short_command(command)} | {name} | {cpu:.0f} |")

    cycles = mgg_cycles(time.monotonic() - started + args.window)
    lines.append("")
    if cycles:
        totals = [c["total"] for c in cycles]
        grids = [c["grid"] for c in cycles]
        gains = [c["gain"] for c in cycles]
        lines.append(
            f"- **MGG plan cycles: {len(cycles)}**, wall ms median {statistics.median(totals):.0f} "
            f"(max {max(totals)}); lattice median {statistics.median(grids):.0f}, "
            f"gain median {statistics.median(gains):.0f}; "
            f"median {statistics.median(c['vertices'] for c in cycles):.0f} vertices, "
            f"{statistics.median(c['edges'] for c in cycles):.0f} edges"
        )
    else:
        lines.append("- MGG plan cycles: none in the window (fleet not exploring?)")

    traffic = websocket_traffic(min(10.0, args.window))
    lines += ["", "| GUI message | per s | KB/s |", "|---|---:|---:|"]
    window = min(10.0, args.window)
    for kind, (count, size) in sorted(traffic.items(), key=lambda kv: -kv[1][1]):
        lines.append(f"| {kind} | {count / window:.1f} | {size / window / 1024:.1f} |")

    if args.profiles:
        lines += ["", "### py-spy (inclusive share of active samples)"]
        for name, target, pattern in PYTHON_TARGETS:
            if container(target) not in containers:
                continue
            frames = pyspy(container(target), pattern, args.profile_seconds)
            lines += ["", f"**{name}** ({target})", ""]
            lines += [f"- {share:.0f}% `{frame}`" for share, frame in frames] or [
                "- no samples"
            ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--window", type=float, default=30.0)
    parser.add_argument("--profiles", action="store_true")
    parser.add_argument("--profile-seconds", type=int, default=20)
    parser.add_argument("--top", type=int, default=15)
    parser.add_argument("--label", default="")
    parser.add_argument("--append", type=Path, help="append the report to this file")
    args = parser.parse_args()
    text = report(args)
    print(text)
    if args.append:
        with args.append.open("a") as stream:
            stream.write("\n" + text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
