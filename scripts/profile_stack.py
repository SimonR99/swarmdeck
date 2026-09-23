#!/usr/bin/env python3
"""Measure a running SwarmDeck simulation stack and print a markdown report.

Run on the host that runs the stack, while it runs:

    scripts/profile_stack.py                 # 30 s window, no profiles
    scripts/profile_stack.py --profiles      # plus py-spy of the Python hot spots
    scripts/profile_stack.py --window 60 --label "after lattice cache"
    scripts/profile_stack.py --browser       # browser process-tree CPU and WebGL frames/s (item 6)
    scripts/profile_stack.py --latency       # plan-log-to-displacement proxy (item 2)

It reports what the clean-up plan (docs/superpowers/plans/
2026-09-23-cleanup-optimization.md) is measured by:

- simulation real-time factor: /clock against wall time;
- CPU per container and per process (top), for the stack's processes only;
- MGG plan cycles: wall time and its lattice ("grid") and gain parts, from
  the planner's own log lines within the window;
- GUI WebSocket traffic: messages and bytes per second by type;
- optionally, browser process-tree CPU and WebGL frames/s at idle and while panning (--browser);
- optionally, plan-log-to-displacement (proxy) and replan cadence per robot (--latency);
- optionally, py-spy profiles of the Python processes, taken from a
  throwaway sidecar container that joins each container's PID namespace,
  so nothing in the stack is modified.

Only docker and python3 are needed on the host. The --browser flag also needs
Linux /proc, node, Playwright and its Chromium binary; it degrades gracefully.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import math
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

# Hint for the Playwright node-modules search: the npx-cache entry that is
# most likely to exist on a freshly installed host.  _find_playwright_modules()
# always does a full directory scan as a fallback, so this hint going stale
# (after a Playwright version bump) is not fatal.
_PLAYWRIGHT_MODULES_HINT = Path.home() / ".npm/_npx"

# Displacement threshold for the plan-log proxy (metres)
_MOTION_THRESHOLD_M = 0.10
# Reject registration boundaries beyond 1 mm translation or 1 mrad yaw.
_REGISTRATION_TOLERANCE = 0.001


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


_BROWSER_HELPERS_JS = r"""
function parseProcStat(text) {
  // comm may contain spaces and parentheses; fields after its LAST ')' start at 3.
  const end = text.lastIndexOf(')');
  const f = text.slice(end + 1).trim().split(/\s+/);
  if (end < 0 || f.length < 20) throw new Error('invalid /proc stat');
  const result = {ppid: Number(f[1]), ticks: Number(f[11]) + Number(f[12]),
                  start: Number(f[19])};
  if (!Object.values(result).every(Number.isFinite)) throw new Error('invalid /proc ticks');
  return result;
}
function cpuPercent(ticks, wall, hz) {
  if (!(wall > 0 && hz > 0 && ticks >= 0)) throw new Error('invalid CPU interval');
  return 100 * ticks / hz / wall;
}
"""

_BROWSER_JS = _BROWSER_HELPERS_JS + r"""
const fs = require('node:fs');
const pwPath = process.env.PW_MODULES;
let pw;
try { pw = require(pwPath + '/playwright'); } catch(e) {
  pw = require(pwPath + '/playwright-core');
}
const IDLE = Number(process.env.IDLE_S);
const PANS = Number(process.env.PANS);
const HZ = Number(process.env.CLK_TCK);
const sleep = ms => new Promise(r => setTimeout(r, ms));

// Keep the last observed ticks of exited processes, keyed by PID + start time.
// Short-lived processes between snapshots can still be missed.
function treeSampler(root) {
  const known = new Map();
  return () => {
    const rows = new Map();
    for (const pid of fs.readdirSync('/proc').filter(p => /^\d+$/.test(p))) {
      try { rows.set(Number(pid), parseProcStat(fs.readFileSync(`/proc/${pid}/stat`, 'utf8'))); }
      catch(e) { if (!['ENOENT', 'ESRCH'].includes(e.code)) throw e; }
    }
    if (!rows.has(root)) throw new Error('browser process exited');
    const selected = new Set([root]);
    let changed = true;
    while (changed) {
      changed = false;
      for (const [pid, row] of rows) {
        if (!selected.has(pid) && selected.has(row.ppid)) {
          selected.add(pid); changed = true;
        }
      }
    }
    for (const [pid, row] of rows) {
      const key = `${pid}:${row.start}`;
      if (selected.has(pid) || known.has(key)) known.set(key, row.ticks);
    }
    return [...known.values()].reduce((a, b) => a + b, 0);
  };
}

(async () => {
  let server, browser;
  try {
    server = await pw.chromium.launchServer({
      headless: true,
      args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
    });
    const ticks = treeSampler(server.process().pid);
    browser = await pw.chromium.connect(server.wsEndpoint());
    const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
    await page.addInitScript(() => {
      window.__webgl = {clears: 0, draws: 0, frames: 0};
      let frameTime;
      // Count distinct animation-frame timestamps with GL work, not draw calls
      // (a scene may issue many draws and clears per frame).
      let rafTime = 0;
      function clock(t) { rafTime = t; requestAnimationFrame(clock); }
      requestAnimationFrame(clock);
      for (const C of [window.WebGLRenderingContext, window.WebGL2RenderingContext]) {
        if (!C) continue;
        for (const name of ['clear', 'drawElements', 'drawArrays',
                            'drawElementsInstanced', 'drawArraysInstanced']) {
          const original = C.prototype[name];
          if (!original) continue;
          C.prototype[name] = function (...args) {
            window.__webgl[name === 'clear' ? 'clears' : 'draws']++;
            if (frameTime !== rafTime) { window.__webgl.frames++; frameTime = rafTime; }
            return original.apply(this, args);
          };
        }
      }
    });
    await page.goto(process.env.DASH_URL, { waitUntil: 'load', timeout: 30000 });
    await sleep(8000);
    async function snapshot() {
      const gl = await page.evaluate(() => ({...window.__webgl}));
      return {gl, ticks: ticks(), time: performance.now() / 1000};
    }
    function measurement(a, b, prefix) {
      const wall = b.time - a.time;
      return {
        [`${prefix}_cpu_pct`]: cpuPercent(b.ticks - a.ticks, wall, HZ),
        [`${prefix}_wall_s`]: wall,
        [`${prefix}_webgl_frames_per_s`]: (b.gl.frames - a.gl.frames) / wall,
        [`${prefix}_clears_per_s`]: (b.gl.clears - a.gl.clears) / wall,
        [`${prefix}_draws_per_s`]: (b.gl.draws - a.gl.draws) / wall,
      };
    }
    const a = await snapshot();
    await sleep(IDLE * 1000);
    const b = await snapshot();
    const c = await snapshot();
    for (let i = 0; i < PANS; i++) {
      await page.mouse.move(700, 450);
      await page.mouse.down();
      for (let dx = 0; dx <= 200; dx += 20) {
        await page.mouse.move(700 + dx, 450 + Math.sin(dx * 0.05) * 30);
        await sleep(40);
      }
      await page.mouse.up();
      await sleep(600);
    }
    const d = await snapshot();
    console.log(JSON.stringify({...measurement(a, b, 'idle'), ...measurement(c, d, 'pan')}));
  } catch (e) {
    console.log(JSON.stringify({error: String(e)}));
  } finally {
    if (browser) await browser.close();
    if (server) await server.close();
  }
})();
"""


def _find_playwright_modules() -> Path | None:
    """Return path to a node_modules directory that contains playwright.

    Searches the npx cache directory (~/.npm/_npx) for any entry that
    contains playwright or playwright-core, sorted for reproducibility.
    No hard-coded cache hash: works after a Playwright version upgrade.
    """
    npx_cache = _PLAYWRIGHT_MODULES_HINT
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
    """Measure Linux browser-tree CPU and WebGL activity using Playwright.

    CPU uses /proc utime+stime (100% = one core), including GPU/compositor
    descendants of the launched browser PID. SwiftShader inflates CPU/frame.
    WebGL frames are animation-frame intervals containing clear/draw calls;
    raw clear/draw rates are also returned. Canvas2D work is not WebGL work.
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
        "CLK_TCK": str(os.sysconf("SC_CLK_TCK")),
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


# -------------------------------------------- plan-log-to-displacement proxy


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
                        events.append({
                            "utc": time.time(), "robot_id": msg["robot_id"],
                            "pose": msg["pose"],
                            "navigation_transform": msg.get("navigation_transform"),
                        })
                    elif msg.get("type") == "robot_map_reset":
                        events.append({"utc": time.time(), "reset": [msg["robot_id"]]})
                    elif msg.get("type") == "sim_reset":
                        events.append({"utc": time.time(), "reset": msg.get("robots")})
                except Exception:
                    pass
    except Exception:
        pass
    print(json.dumps(events))

asyncio.run(main())
"""


def _registration_changed(before: tuple | None, after: tuple | None) -> bool:
    if before is None or after is None:
        return before != after
    angle = math.remainder(after[2] - before[2], 2 * math.pi)
    return (math.hypot(after[0] - before[0], after[1] - before[1]) > _REGISTRATION_TOLERANCE
            or abs(angle) > _REGISTRATION_TOLERANCE)


def latency_trace(window: float) -> dict:
    """Measure plan-log-to-displacement (proxy) and replan cadence.

    Collect robot_state poses and navigation transforms plus reset events,
    timestamped with container time.time(), and MGG docker log timestamps.
    Compare robot-local displacement against the last pose at/before each
    plan (or the first later sample when no earlier pose is available).
    Stop at the next plan for that robot, reset, or material registration
    change. Count plans cut off without a displacement sample separately.
    This is not dispatch latency: a logged plan can be delayed or rejected
    by peer reservations, and there is no dispatch event in this trace.

    Clock: host UTC wall clock. Docker log timestamps are set by the Docker
    daemon from the host clock. ``time.time()`` inside the server container
    uses the same host clock on Linux (containers share the host kernel).
    Typical error between the two: < 1 ms.

    Returns a dict with per-robot ``latency_s`` and ``cadence_s`` lists,
    ``plan_count``, ``cut_off_plan_count``, and an ``error`` key if unreachable.
    """
    probe = _WS_ROBOT_STATE_PROBE.replace("WINDOW", str(window))
    start_epoch = time.time()
    ws_result = sh(
        ["docker", "exec", "-i", container("server"), "python", "-"],
        input=probe,
        timeout=window + 30,
    )
    end_epoch = time.time()
    try:
        raw_events = json.loads(ws_result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        return {"error": (ws_result.stderr or ws_result.stdout).strip()[-300:]}

    robot_events: dict[str, list[tuple]] = collections.defaultdict(list)
    resets = [(row["utc"], row["reset"]) for row in raw_events if "reset" in row]
    for row in raw_events:
        if "pose" not in row:
            continue
        transform = row.get("navigation_transform")
        tx, ty, yaw = (float((transform or {}).get(key, 0)) for key in ("x", "y", "yaw"))
        x, y = float(row["pose"]["x"]) - tx, float(row["pose"]["y"]) - ty
        c, s = math.cos(yaw), math.sin(yaw)
        # Undo the GUI world placement: compare in the navigation frame.
        robot_events[row["robot_id"]].append((
            float(row["utc"]), c * x + s * y, -s * x + c * y,
            (tx, ty, yaw) if transform is not None else None,
        ))

    # Fetch MGG logs with a generous --since window (+60 s) to avoid missing
    # the boundary, then filter to [start_epoch, end_epoch] so that plan
    # cycles that completed before the robot-state probe started are excluded.
    # Without this filter, plans from up to 60 s before the probe could be
    # correlated against current-window motion, producing false latencies.
    logs = sh(
        [
            "docker", "logs", "--timestamps",
            "--since", f"{int(window) + 60}s",
            container("mgg"),
        ],
    )
    all_plan_events = parse_timestamped_plan_lines(logs.stdout + logs.stderr)
    plan_events = [
        (t, c) for t, c in all_plan_events if start_epoch <= t <= end_epoch
    ]

    cadences = replan_cadence(plan_events)
    latencies: dict[str, list[float]] = collections.defaultdict(list)

    cut_off_plans = 0
    for index, (plan_epoch, cycle) in enumerate(plan_events):
        robot = cycle["robot"]
        robot_resets = [utc for utc, ids in resets if ids is None or robot in ids]
        last_reset = max((utc for utc in robot_resets if utc <= plan_epoch), default=-math.inf)
        next_reset = min((utc for utc in robot_resets if utc > plan_epoch), default=math.inf)
        next_plan = next((t for t, c in plan_events[index + 1:] if c["robot"] == robot), math.inf)
        cutoff = min(next_plan, next_reset)
        cut_off = math.isfinite(cutoff)
        # Never borrow a reference from before a reset or after the cutoff.
        events = [event for event in robot_events.get(robot, []) if last_reset < event[0] < cutoff]
        measured = False
        if events:
            reference = next((event for event in reversed(events) if event[0] <= plan_epoch), events[0])
            _, ref_x, ref_y, ref_transform = reference
            for utc, x, y, transform in events:
                if utc <= plan_epoch:
                    continue
                if _registration_changed(ref_transform, transform):
                    cut_off = True
                    break
                if math.hypot(x - ref_x, y - ref_y) >= _MOTION_THRESHOLD_M:
                    latencies[robot].append(utc - plan_epoch)
                    measured = True
                    break
        if cut_off and not measured:
            cut_off_plans += 1

    return {
        "plan_count": len(plan_events),
        "cut_off_plan_count": cut_off_plans,
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
        lines += ["", "### Browser process-tree CPU and WebGL (SwiftShader inflates CPU cost per frame)"]
        bcpu = browser_cpu(burl, idle_s=getattr(args, "browser_idle_s", 5.0))
        if "error" in bcpu:
            lines.append(f"- browser measurement unavailable: {bcpu['error']}")
        else:
            for phase, label in (("idle", "idle"), ("pan", "panning")):
                lines.append(
                    f"- **{label}**: {bcpu[f'{phase}_cpu_pct']:.1f} % CPU, "
                    f"{bcpu[f'{phase}_webgl_frames_per_s']:.1f} WebGL frames/s, "
                    f"{bcpu[f'{phase}_clears_per_s']:.1f} clears/s, "
                    f"{bcpu[f'{phase}_draws_per_s']:.1f} draws/s "
                    f"(over {bcpu[f'{phase}_wall_s']:.1f} s)"
                )
            lines.append(f"  URL: {burl}")

    if getattr(args, "latency", False):
        lines += [
            "",
            "### Plan-log-to-displacement (proxy)"
            f" (MGG plan \u2192 navigation-frame displacement >= {_MOTION_THRESHOLD_M:.2f} m)",
        ]
        lt = latency_trace(args.window)
        if "error" in lt:
            lines.append(f"- latency trace unavailable: {lt['error']}")
        else:
            lines.append(f"- MGG plan cycles in window: {lt['plan_count']}")
            lines.append(f"- Cut-off plans without a displacement sample: {lt['cut_off_plan_count']}")
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
        help="measure browser process-tree CPU and WebGL frames/s via headless Chromium + Playwright (/proc, SwiftShader)",
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
        help="measure plan-log-to-displacement (proxy) and replan cadence per robot",
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
