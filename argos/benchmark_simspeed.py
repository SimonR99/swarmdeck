#!/usr/bin/env python3
"""Benchmark unpaced SubT lidar options on tuf only; never on a workstation.

Run after reserving tuf: python3 argos/benchmark_simspeed.py --output /tmp/simspeed
The default is three alternating 60 s windows per option, rebuilding once.
No simulation runs on import, --help, or a host other than tuf.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import sys
import tempfile
import time
import os

REPO = Path(__file__).resolve().parents[1]
OPTIONS = ("baseline", "lidar5", "parked2")


def benchmark_config(source: str, option: str) -> str:
    """Use the committed SubT fixture unchanged except for explicit probe knobs."""
    anchor = "    profile: os1_32\n"
    if option not in OPTIONS or source.count(anchor) != 1 or "\nsimulation:" in source:
        raise ValueError("benchmark requires the unmodified SubT config layout")
    if option == "lidar5":
        source = source.replace(anchor, anchor + "    rate: 5\n")
    source += "\nsimulation:\n  realtime_factor: 0\n"
    if option == "parked2":
        source += "  parked_lidar_rate: 2\n"
    return source


def benchmark_order(rounds: int) -> list[str]:
    return [
        option
        for round_index in range(rounds)
        for option in OPTIONS[round_index % 3 :] + OPTIONS[: round_index % 3]
    ]


def host_sample() -> dict:
    memory = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        if key in {"MemFree", "MemAvailable", "SwapFree", "SwapTotal"}:
            memory[key + "_kib"] = int(value.split()[0])
    return {
        "time": dt.datetime.now(dt.timezone.utc).isoformat(),
        "load_average": os.getloadavg(),
        **memory,
    }


def gpu_summary(path: Path) -> dict:
    samples: dict[str, list[float]] = {}
    with path.open() as source:
        for row in csv.reader(source):
            if len(row) != 6:
                continue
            try:
                utilization = float(row[2].strip().removesuffix("%").strip())
            except ValueError:
                continue
            samples.setdefault(row[1].strip(), []).append(utilization)
    if not samples:
        raise RuntimeError("No GPU utilization samples; inspect the nvidia-smi CSV")
    return {
        gpu: {
            "utilization_gpu_pct_mean": statistics.mean(values),
            "samples": len(values),
        }
        for gpu, values in samples.items()
    }


WINDOW_PROBE = r"""
import json, time, rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node("swarmdeck_simspeed_probe")
clock = []
scans = {f"robot_{i}": [] for i in range(4)}
def stamp(value):
    return value.sec + value.nanosec * 1e-9
node.create_subscription(Clock, "/clock",
    lambda m: clock.append((time.monotonic(), stamp(m.clock))),
    qos_profile_sensor_data)
for robot, samples in scans.items():
    node.create_subscription(PointCloud2, f"/{robot}/scan/points",
        lambda m, samples=samples: samples.append((time.monotonic(), stamp(m.header.stamp))),
        qos_profile_sensor_data)
end = time.monotonic() + WINDOW
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
print(json.dumps({"clock": clock, "scans": scans}))
node.destroy_node()
rclpy.shutdown()
"""


def measure_window(window: float) -> dict:
    result = subprocess.run(
        [
            "docker",
            "exec",
            "-i",
            "swarmdeck-sim-1",
            "bash",
            "-c",
            "source /opt/ros/jazzy/setup.bash && python3 -",
        ],
        input=WINDOW_PROBE.replace("WINDOW", repr(window)),
        text=True,
        capture_output=True,
        timeout=window + 60,
    )
    if result.returncode:
        return {"rtf": {"error": (result.stderr or result.stdout)[-500:]}, "scans": {}}
    try:
        raw = json.loads(result.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError) as exc:
        return {"rtf": {"error": f"invalid window probe output: {exc}"}, "scans": {}}
    clock = raw["clock"]
    if len(clock) < 2 or clock[-1][0] <= clock[0][0]:
        rtf = {"error": f"only {len(clock)} usable /clock messages"}
    else:
        wall, sim = (clock[-1][i] - clock[0][i] for i in (0, 1))
        rtf = {
            "rtf": sim / wall,
            "sim_s": sim,
            "wall_s": wall,
            "clock_hz": (len(clock) - 1) / wall,
        }
    scans = {}
    for robot, samples in raw["scans"].items():
        summary = {"messages": len(samples)}
        for i, label in enumerate(("scan_hz_wall", "scan_hz_sim")):
            elapsed = samples[-1][i] - samples[0][i] if len(samples) > 1 else 0
            summary[label] = (len(samples) - 1) / elapsed if elapsed > 0 else None
        scans[robot] = summary
    return {"rtf": rtf, "scans": scans}


def run_window(output: Path, window: float, probe) -> dict:
    before = host_sample()
    with output.with_suffix(".gpu.csv").open("w") as gpu_output:
        gpu = subprocess.Popen(
            [
                "nvidia-smi",
                "--query-gpu=timestamp,index,utilization.gpu,utilization.memory,memory.used,power.draw",
                "--format=csv",
                "-l",
                "1",
            ],
            stdout=gpu_output,
            stderr=subprocess.STDOUT,
        )
        try:
            result = probe(window)
        finally:
            gpu.terminate()
            gpu.wait(timeout=10)
    try:
        gpu_result = gpu_summary(output.with_suffix(".gpu.csv"))
    except RuntimeError as exc:
        gpu_result = {"error": str(exc)}
    record = {
        "before": before,
        "after": host_sample(),
        "rtf": result["rtf"],
        "scans": result["scans"],
        "gpu": gpu_result,
    }
    output.with_suffix(".json").write_text(json.dumps(record, indent=2) + "\n")
    if "error" in result["rtf"]:
        raise RuntimeError(result["rtf"]["error"])
    if "error" in gpu_result:
        raise RuntimeError(gpu_result["error"])
    return record


def stop_stack(launcher: list[str], stem: Path) -> None:
    try:
        with stem.with_suffix(".argos.log").open("w") as log:
            subprocess.run(
                ["docker", "logs", "swarmdeck-argos-1"],
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=30,
            )
    finally:
        with stem.with_suffix(".stop.log").open("w") as log:
            subprocess.run(
                [*launcher, "--down"],
                cwd=REPO,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )


def stop_exploration() -> None:
    subprocess.run(
        ["docker", "exec", "-i", "swarmdeck-server-1", "python", "-"],
        input="""import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://localhost:8080/ws") as ws:
        await ws.send(json.dumps({"type": "stop_explore"}))
        # Keep the connection alive while the server forwards the stop, and
        # allow the chassis to settle before collecting stationary scan rates.
        await asyncio.sleep(2)
asyncio.run(main())
""",
        text=True,
        check=True,
        timeout=30,
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp")
        / ("swarmdeck-simspeed-" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")),
    )
    parser.add_argument("--window", type=float, default=60)
    parser.add_argument("--warmup", type=float, default=60)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--no-build", action="store_true")
    args = parser.parse_args(argv)
    if platform.node().split(".")[0] != "tuf":
        print(
            "Simulation benchmarks may run only on tuf, never on this workstation.",
            file=sys.stderr,
        )
        return 2
    if args.window <= 0 or args.warmup < 0 or args.rounds < 1:
        parser.error("window and rounds must be positive; warmup must be nonnegative")
    os.environ["COMPOSE_PROJECT"] = "swarmdeck"
    existing = subprocess.check_output(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            "label=com.docker.compose.project=swarmdeck",
            "--format",
            "{{.Names}}",
        ],
        text=True,
    ).strip()
    if existing:
        print(
            "Refusing to replace an existing swarmdeck stack: " + existing,
            file=sys.stderr,
        )
        return 2
    subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv"], check=True)
    args.output.mkdir(parents=True, exist_ok=False)
    source = (REPO / "configs/4robot_subt_finals.yaml").read_text()
    probe = measure_window
    launcher = [sys.executable, str(REPO / "deploy/simulation_launch.py")]
    order = benchmark_order(args.rounds)
    (args.output / "plan.json").write_text(
        json.dumps(
            {
                "host": platform.node(),
                "order": order,
                "window_s": args.window,
                "warmup_s": args.warmup,
                "revision": subprocess.check_output(
                    ["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True
                ).strip(),
                "note": "Parked fleet; unpaced streaming; not navigation or iGPU acceptance. GPU CSV includes ROS probe discovery time.",
            },
            indent=2,
        )
        + "\n"
    )

    # SIGTERM must take the same cleanup path as Ctrl-C.
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    owns_stack = False
    try:
        with tempfile.TemporaryDirectory(
            prefix=".simspeed-", dir=REPO / "configs"
        ) as temporary:
            configs = {}
            for option in OPTIONS:
                config = benchmark_config(source, option)
                configs[option] = Path(temporary) / f"{option}.yaml"
                configs[option].write_text(config)
                (args.output / f"{option}.yaml").write_text(config)
            for index, option in enumerate(order):
                stem = args.output / f"{index + 1:02d}-{option}"
                command = [
                    *launcher,
                    "-s",
                    str(configs[option]),
                    "--gpu",
                    "--drift",
                    "-t",
                    "10",
                    "-e",
                    "0",
                ]
                if index or args.no_build:
                    command.append("--no-build")
                owns_stack = True
                with stem.with_suffix(".launch.log").open("w") as log:
                    subprocess.run(
                        command,
                        cwd=REPO,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        check=True,
                    )
                time.sleep(args.warmup)
                experiment = subprocess.check_output(
                    [
                        "docker",
                        "exec",
                        "swarmdeck-argos-1",
                        "cat",
                        "/run/swarmdeck/session.argos",
                    ],
                    text=True,
                )
                stem.with_suffix(".argos").write_text(experiment)
                stop_exploration()
                record = run_window(stem, args.window, probe)
                print(
                    f"{stem.name}: {record['rtf']['rtf']:.3f} sim-seconds/wall-second",
                    flush=True,
                )
                stop_stack(launcher, stem)
                owns_stack = False
    finally:
        if owns_stack:
            stop_stack(launcher, stem)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
