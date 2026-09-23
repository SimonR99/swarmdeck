#!/usr/bin/env python3
"""Measure a running SwarmDeck simulation stack and print a markdown report.

Run on the host that runs the stack, while it runs:

    scripts/profile_stack.py                 # 30 s window, no profiles
    scripts/profile_stack.py --profiles      # plus py-spy of the Python hot spots
    scripts/profile_stack.py --window 60 --label "after lattice cache"
    scripts/profile_stack.py --browser       # browser main-thread CPU (item 6)
    scripts/profile_stack.py --latency       # plan-to-motion latency trace (item 2)

It reports what the clean-up plan (docs/superpowers/plans/
2026-09-23-cleanup-optimization.md) is measured by:

- simulation real-time factor: /clock against wall time;
- CPU per container and per process (top), for the stack's processes only;
- MGG plan cycles: wall time and its lattice ("grid") and gain parts, from
  the planner's own log lines within the window;
- GUI WebSocket traffic: messages and bytes per second by type;
- optionally, browser main-thread CPU at idle and while panning (--browser);
- optionally, plan-to-motion latency and replan cadence per robot (--latency);
- optionally, py-spy profiles of the Python processes, taken from a
  throwaway sidecar container that joins each container's PID namespace,
  so nothing in the stack is modified.

Only docker and python3 are needed on the host. The --browser flag also needs
node (for Playwright) or a local Chromium binary; it degrades gracefully.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import os
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

# docker logs --timestamps prefix: 2006-01-02T15:04:05.999999999Z
_DOCKER_TS = re.compile(
    r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})\.(\d+)Z\s+(.*)"
)

# Path to Playwright's bundled Chromium (populated by `npx playwright install`)
_PLAYWRIGHT_CHROMIUM = (
    Path.home() / ".cache/ms-playwright/chromium-1243/chrome-linux64/chrome"
)
# Path to Playwright's Node modules (from npx cache)
_PLAYWRIGHT_MODULES = Path.home() / ".npm/_npx/e41f203b7505f1fb/node_modules"

# Motion threshold for plan-to-motion latency (metres)
_MOTION_THRESHOLD_M = 0.10


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


# ------------------------------------------------------------ exploration


EXPLORE_PROBE = r"""
import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://localhost:8080/ws") as ws:
        await ws.send(json.dumps({"type": "start_explore"}))
        await asyncio.sleep(1.0)
asyncio.run(main())
"""


def start_explore() -> None:
    """Fleet-wide Explore, as the dashboard's Fleet toggle sends it."""
    sh(
        ["docker", "exec", "-i", container("server"), "python", "-"],
        input=EXPLORE_PROBE,
        timeout=30,
    )


# ---------------------------------------------------------------- perf

PERF_IMAGE = "swarmdeck-perf:6.8.0-48"

PERF_SCRIPT = r"""
pid=$(ps -eo pid,pcpu,comm --sort=-pcpu | awk '/mggplanner_node/{print $1; exit}')
[ -z "$pid" ] && { echo "no mggplanner_node"; exit 0; }
echo "profiling mggplanner_node pid $pid"
perf record -q -F 199 -g -p $pid -o /tmp/perf.data -- sleep $SECONDS >/dev/null 2>&1
echo "--- self"
perf report -q -i /tmp/perf.data --stdio --no-children --sort symbol   --percent-limit 1.5 -g none 2>/dev/null | head -30
echo "--- inclusive"
perf report -q -i /tmp/perf.data --stdio --children --sort symbol   --percent-limit 4 -g none 2>/dev/null | head -40
"""


def perf_mgg(seconds: int) -> str:
    """perf profile of the busiest MGG planner, from a throwaway sidecar.

    Ubuntu's perf_event_paranoid level 4 refuses CAP_PERFMON alone, so the
    sidecar gets SYS_ADMIN and no seccomp filter; it only lives for the
    profile and shares nothing with the stack but the PID namespace.
    """
    result = sh(
        [
            "docker",
            "run",
            "--rm",
            f"--pid=container:{container('mgg')}",
            "--cap-add",
            "SYS_ADMIN",
            "--cap-add",
            "SYS_PTRACE",
            "--security-opt",
            "seccomp=unconfined",
            PERF_IMAGE,
            "sh",
            "-c",
            PERF_SCRIPT.replace("$SECONDS", str(seconds)),
        ],
        timeout=seconds + 300,
    )
    return (result.stdout + result.stderr).strip()


# -------------------------------------------------------------- browser CPU


_BROWSER_JS = r"""
const pwPath = process.env.PW_MODULES;
let pw;
try { pw = require(pwPath + '/playwright'); } catch(e) {
  try { pw = require(pwPath + '/playwright-core'); } catch(e2) {
    console.log(JSON.stringify({error: 'playwright not found: ' + e2.message}));
    process.exit(0);
  }
}
const { chromium } = pw;
const URL   = process.env.DASH_URL;
const IDLE  = parseFloat(process.env.IDLE_S  || '5');
const PANS  = parseInt(  process.env.PANS     || '3');

function metric(arr, name) {
  const m = arr.find(x => x.name === name);
  return m ? m.value : 0;
}

(async () => {
  let browser;
  try {
    browser = await chromium.launch({
      headless: true,
      args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
    });
    const ctx  = await browser.newContext({ viewport: { width: 1280, height: 720 } });
    const page = await ctx.newPage();
    const cdp  = await ctx.newCDPSession(page);
    await cdp.send('Performance.enable');

    // Navigate and settle; tolerate a stack that is not running
    try { await page.goto(URL, { waitUntil: 'networkidle', timeout: 20000 }); }
    catch(_) {}
    await new Promise(r => setTimeout(r, 2000));

    // -- idle phase --
    const r0 = (await cdp.send('Performance.getMetrics')).metrics;
    await new Promise(r => setTimeout(r, IDLE * 1000));
    const r1 = (await cdp.send('Performance.getMetrics')).metrics;
    const idle_wall   = metric(r1, 'Timestamp') - metric(r0, 'Timestamp');
    const idle_task   = metric(r1, 'TaskDuration')   - metric(r0, 'TaskDuration');
    const idle_script = metric(r1, 'ScriptDuration')  - metric(r0, 'ScriptDuration');

    // -- panning phase: synthetic mouse drags across the viewport centre --
    const vp  = page.viewportSize();
    const cx  = vp ? vp.width  / 2 : 640;
    const cy  = vp ? vp.height / 2 : 360;
    const r2  = (await cdp.send('Performance.getMetrics')).metrics;
    for (let i = 0; i < PANS; i++) {
      await page.mouse.move(cx - 100, cy);
      await page.mouse.down();
      for (let dx = 0; dx <= 200; dx += 20) {
        await page.mouse.move(cx - 100 + dx, cy + Math.sin(dx * 0.05) * 30);
        await new Promise(r => setTimeout(r, 40));
      }
      await page.mouse.up();
      await new Promise(r => setTimeout(r, 600));
    }
    const r3  = (await cdp.send('Performance.getMetrics')).metrics;
    const pan_wall   = metric(r3, 'Timestamp') - metric(r2, 'Timestamp');
    const pan_task   = metric(r3, 'TaskDuration')   - metric(r2, 'TaskDuration');
    const pan_script = metric(r3, 'ScriptDuration')  - metric(r2, 'ScriptDuration');

    console.log(JSON.stringify({
      idle_cpu_pct:    idle_wall > 0 ? 100 * idle_task   / idle_wall : null,
      idle_script_pct: idle_wall > 0 ? 100 * idle_script / idle_wall : null,
      idle_wall_s: idle_wall,
      pan_cpu_pct:    pan_wall > 0 ? 100 * pan_task   / pan_wall : null,
      pan_script_pct: pan_wall > 0 ? 100 * pan_script / pan_wall : null,
      pan_wall_s: pan_wall,
    }));
  } catch (e) {
    console.log(JSON.stringify({ error: String(e) }));
  } finally {
    if (browser) await browser.close();
  }
})();
"""


def _find_playwright_modules() -> Path | None:
    """Return path to a node_modules directory that contains playwright."""
    if (_PLAYWRIGHT_MODULES / "playwright").exists():
        return _PLAYWRIGHT_MODULES
    # fall back: search npx cache
    npx_cache = Path.home() / ".npm/_npx"
    if npx_cache.is_dir():
        for entry in sorted(npx_cache.iterdir()):
            candidate = entry / "node_modules"
            if (candidate / "playwright").exists() or (
                candidate / "playwright-core"
            ).exists():
                return candidate
    return None


def browser_cpu(
    url: str,
    idle_s: float = 5.0,
    pans: int = 3,
) -> dict:
    """Measure browser renderer-thread CPU at idle and while panning via CDP.

    Uses Playwright (found in the npx cache) to drive a headless Chromium.
    Falls back gracefully with an ``error`` key when no browser is available.

    Metric: CDP ``Performance.getMetrics`` ``TaskDuration`` /
    ``ScriptDuration`` deltas divided by the ``Timestamp`` delta (seconds).
    CPU % = TaskDuration_delta / Timestamp_delta * 100.  The Timestamp clock
    is monotonic within the renderer process (seconds since process start);
    it is not correlated with wall time, but the ratio is accurate.
    """
    pw_modules = _find_playwright_modules()
    if pw_modules is None:
        return {"error": "playwright node modules not found (run: npx playwright install)"}
    node = next(
        (Path(p) for p in ("/usr/bin/node", "/usr/local/bin/node") if Path(p).exists()),
        None,
    )
    if node is None:
        return {"error": "node not found"}
    env = {
        **os.environ,
        "PW_MODULES": str(pw_modules),
        "DASH_URL": url,
        "IDLE_S": str(idle_s),
        "PANS": str(pans),
    }
    timeout = idle_s + pans * 3 + 90
    result = sh([str(node), "-e", _BROWSER_JS], timeout=timeout, env=env)
    for line in reversed((result.stdout + "\n" + result.stderr).splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                pass
    return {"error": (result.stderr or result.stdout).strip()[-300:]}


# ------------------------------------------------------ plan-to-motion latency


def parse_timestamped_plan_lines(log_text: str) -> list[tuple[float, dict]]:
    """Parse ``docker logs --timestamps`` output of the mgg container.

    Returns a list of ``(utc_epoch_s, cycle_dict)`` sorted by time.
    This is a pure function so it can be unit-tested on captured log snippets.
    """
    results: list[tuple[float, dict]] = []
    for line in log_text.splitlines():
        ts_match = _DOCKER_TS.match(line)
        if not ts_match:
            continue
        ts_base, ts_frac, content = ts_match.groups()
        plan_match = PLAN_LINE.search(content)
        if not plan_match:
            continue
        # Parse timestamp; truncate nanoseconds to microseconds for strptime
        frac6 = (ts_frac + "000000")[:6]
        try:
            ts_dt = dt.datetime.strptime(
                f"{ts_base}.{frac6}", "%Y-%m-%dT%H:%M:%S.%f"
            ).replace(tzinfo=dt.timezone.utc)
            epoch = ts_dt.timestamp()
        except ValueError:
            continue
        robot, cells, vertices, edges, total, _glob, grid, gain, select = (
            plan_match.groups()
        )
        results.append((
            epoch,
            {
                "robot": robot,
                "cells": int(cells),
                "vertices": int(vertices),
                "edges": int(edges),
                "total": int(total),
                "grid": int(grid),
                "gain": int(gain),
                "select": int(select),
            },
        ))
    results.sort(key=lambda x: x[0])
    return results


def replan_cadence(plan_events: list[tuple[float, dict]]) -> dict[str, list[float]]:
    """Compute inter-replan intervals (seconds) per robot from timestamped plan events.

    Returns ``{robot_id: [interval_s, ...]}``. Pure function.
    """
    per_robot: dict[str, list[float]] = collections.defaultdict(list)
    last_time: dict[str, float] = {}
    for epoch, cycle in plan_events:
        robot = cycle["robot"]
        if robot in last_time:
            per_robot[robot].append(epoch - last_time[robot])
        last_time[robot] = epoch
    return dict(per_robot)


_WS_ROBOT_STATE_PROBE = r"""
import asyncio, json, time, websockets

async def main():
    events = []
    t_end = time.time() + WINDOW
    try:
        async with websockets.connect(
            "ws://localhost:8080/ws", max_size=None, open_timeout=5
        ) as ws:
            while time.time() < t_end:
                try:
                    m = await asyncio.wait_for(ws.recv(), 0.2)
                except asyncio.TimeoutError:
                    continue
                try:
                    msg = json.loads(m)
                    if msg.get("type") == "robot_state" and "pose" in msg:
                        events.append([
                            time.time(),
                            str(msg.get("robot_id", "?")),
                            float(msg["pose"].get("x", 0.0)),
                            float(msg["pose"].get("y", 0.0)),
                        ])
                except Exception:
                    pass
    except Exception:
        pass
    print(json.dumps(events))

asyncio.run(main())
"""


def latency_trace(window: float) -> dict:
    """Measure plan-to-motion latency and replan cadence.

    Approach:
    1. Collect ``robot_state`` WebSocket events from the server container
       for ``window`` seconds. Each event carries ``time.time()`` (container
       wall clock) and the robot's pose ``(x, y)``.
    2. Parse ``docker logs --timestamps`` of the mgg container to get UTC
       timestamps for each MGG plan-cycle log line in the same window.
    3. The MGG plan completion time is used as a proxy for "path dispatched":
       the adapter subscribes to the MGG path topic and sends it to Nav2
       within the same ROS spin cycle (< 10 ms typical on the same host).
    4. For each plan event (per robot), find the earliest robot_state event
       after that timestamp where pose displacement exceeds
       _MOTION_THRESHOLD_M from the pose at plan time.
    5. Latency = motion_start_time - plan_timestamp.

    Clock: host UTC wall clock. Docker log timestamps are set by the Docker
    daemon from the host clock. ``time.time()`` inside the server container
    uses the same host clock on Linux (containers share the host kernel).
    Typical error between the two: < 1 ms.

    Returns a dict with per-robot ``latency_s`` and ``cadence_s`` lists,
    ``plan_count``, and an ``error`` key if the stack is unreachable.
    """
    probe = _WS_ROBOT_STATE_PROBE.replace("WINDOW", str(window))
    ws_result = sh(
        ["docker", "exec", "-i", container("server"), "python", "-"],
        input=probe,
        timeout=window + 30,
    )
    try:
        raw_events = json.loads(ws_result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        return {"error": (ws_result.stderr or ws_result.stdout).strip()[-300:]}

    # events: [[utc_s, robot_id, x, y], ...]
    robot_events: dict[str, list[tuple[float, float, float]]] = collections.defaultdict(list)
    for row in raw_events:
        utc, rid, x, y = row
        if rid != "__error__":
            robot_events[rid].append((float(utc), float(x), float(y)))

    # parse mgg log with timestamps from the last window + 60 s
    logs = sh(
        [
            "docker", "logs", "--timestamps",
            "--since", f"{int(window) + 60}s",
            container("mgg"),
        ],
    )
    plan_events = parse_timestamped_plan_lines(logs.stdout + logs.stderr)

    cadences = replan_cadence(plan_events)
    latencies: dict[str, list[float]] = collections.defaultdict(list)

    for plan_epoch, cycle in plan_events:
        robot = cycle["robot"]
        events = robot_events.get(robot, [])
        if not events:
            continue
        # robot position just before the plan (or first known position)
        ref_pos = next(
            ((x, y) for utc, x, y in events if utc <= plan_epoch),
            (events[0][1], events[0][2]),
        )
        # find first event after plan_epoch where displacement > threshold
        for utc, x, y in events:
            if utc <= plan_epoch:
                continue
            dist = ((x - ref_pos[0]) ** 2 + (y - ref_pos[1]) ** 2) ** 0.5
            if dist >= _MOTION_THRESHOLD_M:
                latencies[robot].append(utc - plan_epoch)
                break

    return {
        "plan_count": len(plan_events),
        "latency_s": dict(latencies),
        "cadence_s": cadences,
    }


# ------------------------------------------------------------------ git


def git_commit(override: str = "") -> str:
    """Return the short HEAD commit of this script's repo, or the override string.

    Allows each dated report to name the code it measured. Pass ``--commit``
    when running from a directory that is not the stack's own checkout.
    """
    if override:
        return override
    result = sh(
        [
            "git",
            "-C",
            str(Path(__file__).resolve().parent),
            "rev-parse",
            "--short",
            "HEAD",
        ]
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


# ---------------------------------------------------------------- report


def report(args) -> str:
    started = time.monotonic()
    commit = git_commit(getattr(args, "commit", ""))
    lines = [
        f"## {dt.datetime.now().isoformat(timespec='minutes')}"
        + (f" - {args.label}" if args.label else "")
        + f" (commit {commit})",
        "",
    ]
    containers = running_containers()
    if not containers:
        return "No swarmdeck containers are running."
    if args.start_explore:
        start_explore()
        lines.append("- Explore started by the harness")

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

    if args.perf_mgg:
        lines += ["", "### perf: busiest MGG planner", "", "```"]
        lines += [perf_mgg(args.perf_mgg), "```"]

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

    if getattr(args, "browser", False):
        burl = getattr(args, "browser_url", "http://localhost:5173")
        lines += ["", "### Browser main-thread CPU (CDP Performance.getMetrics)"]
        bcpu = browser_cpu(burl, idle_s=getattr(args, "browser_idle_s", 5.0))
        if "error" in bcpu:
            lines.append(f"- browser measurement unavailable: {bcpu['error']}")
        else:
            lines.append(
                f"- **idle**: task {bcpu.get('idle_cpu_pct', '?'):.1f} % CPU, "
                f"script {bcpu.get('idle_script_pct', '?'):.1f} % "
                f"(over {bcpu.get('idle_wall_s', 0):.1f} s)"
            )
            lines.append(
                f"- **panning**: task {bcpu.get('pan_cpu_pct', '?'):.1f} % CPU, "
                f"script {bcpu.get('pan_script_pct', '?'):.1f} % "
                f"(over {bcpu.get('pan_wall_s', 0):.1f} s)"
            )
            lines.append(f"  URL: {burl}")

    if getattr(args, "latency", False):
        lines += [
            "",
            "### Plan-to-motion latency"
            f" (MGG plan \u2192 robot displacement > {_MOTION_THRESHOLD_M:.2f} m)",
        ]
        lt = latency_trace(args.window)
        if "error" in lt:
            lines.append(f"- latency trace unavailable: {lt['error']}")
        else:
            lines.append(f"- MGG plan cycles in window: {lt['plan_count']}")
            lines.append(
                "  - Clock: host UTC wall clock (docker log timestamps vs "
                "container time.time()); typical error < 1 ms"
            )
            lat = lt.get("latency_s", {})
            cad = lt.get("cadence_s", {})
            if not lat and not cad:
                lines.append("  - No correlated events (fleet not exploring?)")
            for robot in sorted(set(list(lat.keys()) + list(cad.keys()))):
                parts = [f"**{robot}**"]
                if robot in cad and cad[robot]:
                    iv = sorted(cad[robot])
                    p90 = iv[min(int(len(iv) * 0.9), len(iv) - 1)]
                    parts.append(
                        f"replan cadence: median {statistics.median(iv):.2f} s,"
                        f" p90 {p90:.2f} s, max {max(iv):.2f} s"
                        f" ({len(iv)} intervals)"
                    )
                if robot in lat and lat[robot]:
                    lv = sorted(lat[robot])
                    p90 = lv[min(int(len(lv) * 0.9), len(lv) - 1)]
                    parts.append(
                        f"latency: median {statistics.median(lv):.2f} s,"
                        f" p90 {p90:.2f} s, max {max(lv):.2f} s"
                        f" ({len(lv)} samples)"
                    )
                lines.append("  - " + "; ".join(parts))

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--window", type=float, default=30.0)
    parser.add_argument("--profiles", action="store_true")
    parser.add_argument("--profile-seconds", type=int, default=20)
    parser.add_argument("--top", type=int, default=15)
    parser.add_argument(
        "--start-explore", action="store_true", help="send fleet-wide Explore first"
    )
    parser.add_argument(
        "--perf-mgg",
        type=int,
        default=0,
        metavar="SECONDS",
        help=f"perf-profile the busiest MGG planner (needs the {PERF_IMAGE} image)",
    )
    parser.add_argument("--label", default="")
    parser.add_argument("--append", type=Path, help="append the report to this file")
    parser.add_argument(
        "--commit",
        default="",
        metavar="SHA",
        help="git commit recorded in the header (default: auto-detected from this repo)",
    )
    parser.add_argument(
        "--browser",
        action="store_true",
        help="measure browser main-thread CPU via headless Chromium + Playwright CDP",
    )
    parser.add_argument(
        "--browser-url",
        default="http://localhost:5173",
        help="dashboard URL for --browser (default: http://localhost:5173)",
    )
    parser.add_argument(
        "--browser-idle-s",
        type=float,
        default=5.0,
        help="idle measurement window for --browser in seconds (default: 5)",
    )
    parser.add_argument(
        "--latency",
        action="store_true",
        help="measure plan-to-motion latency and replan cadence per robot",
    )
    args = parser.parse_args()
    text = report(args)
    print(text)
    if args.append:
        with args.append.open("a") as stream:
            stream.write("\n" + text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
