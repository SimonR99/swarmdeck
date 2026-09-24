"""The benchmark must be repeatable and refuse workstation simulations."""

from pathlib import Path
import runpy

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
SCRIPT = REPO / "argos/benchmark_simspeed.py"


def test_benchmark_variants_keep_geometry_and_disable_only_wall_pacing():
    benchmark = runpy.run_path(str(SCRIPT))
    source = (REPO / "configs/4robot_subt_finals.yaml").read_text()
    original = yaml.safe_load(source)
    for option in ("baseline", "lidar5", "parked2"):
        config = yaml.safe_load(benchmark["benchmark_config"](source, option))
        assert config["simulation"]["realtime_factor"] == 0
        assert config["fleet"].get("lidar", {}).pop("rate", 10) == (
            5 if option == "lidar5" else 10
        )
        assert config["fleet"] == original["fleet"]
        assert config["map"] == original["map"]
        assert config["simulation"].get("parked_lidar_rate") == (
            2 if option == "parked2" else None
        )


def test_benchmark_refuses_non_tuf_host_before_launching(monkeypatch, capsys):
    benchmark = runpy.run_path(str(SCRIPT))
    monkeypatch.setattr(benchmark["platform"], "node", lambda: "extra")
    assert benchmark["main"]([]) == 2
    assert "only on tuf" in capsys.readouterr().err


def test_benchmark_rejects_an_already_customized_source():
    benchmark = runpy.run_path(str(SCRIPT))
    source = (REPO / "configs/4robot_subt_finals.yaml").read_text()
    with pytest.raises(ValueError):
        benchmark["benchmark_config"](source + "\nsimulation: {}\n", "baseline")


def test_benchmark_summarizes_gpu_utilization_and_rejects_missing_samples(tmp_path):
    benchmark = runpy.run_path(str(SCRIPT))
    samples = tmp_path / "gpu.csv"
    samples.write_text(
        "timestamp, index, utilization.gpu [%], utilization.memory [%], memory.used [MiB], power.draw [W]\n"
        "2026/09/23 20:00:00, 0, 80 %, 20 %, 100 MiB, 100 W\n"
        "2026/09/23 20:00:01, 0, 100 %, 30 %, 120 MiB, 110 W\n"
    )
    assert benchmark["gpu_summary"](samples) == {
        "0": {"utilization_gpu_pct_mean": 90.0, "samples": 2}
    }
    samples.write_text("failed to initialize NVML\n")
    with pytest.raises(RuntimeError, match="GPU utilization"):
        benchmark["gpu_summary"](samples)


def test_benchmark_order_balances_every_position_over_three_rounds():
    benchmark = runpy.run_path(str(SCRIPT))
    order = benchmark["benchmark_order"](3)
    assert order == [
        "baseline",
        "lidar5",
        "parked2",
        "lidar5",
        "parked2",
        "baseline",
        "parked2",
        "baseline",
        "lidar5",
    ]
    for position in range(3):
        assert set(order[position::3]) == {"baseline", "lidar5", "parked2"}


def test_missing_gpu_samples_preserve_measured_rtf(tmp_path, monkeypatch):
    import json

    benchmark = runpy.run_path(str(SCRIPT))

    class MissingGpu:
        def __init__(self, *args, **kwargs):
            kwargs["stdout"].write("NVML unavailable\n")

        def terminate(self):
            pass

        def wait(self, timeout):
            return 1

    monkeypatch.setattr(benchmark["subprocess"], "Popen", MissingGpu)
    output = tmp_path / "window"
    with pytest.raises(RuntimeError, match="GPU utilization"):
        benchmark["run_window"](
            output, 60, lambda _: {"rtf": 2.0, "sim_s": 120, "wall_s": 60}
        )
    saved = json.loads(output.with_suffix(".json").read_text())
    assert saved["rtf"] == {"rtf": 2.0, "sim_s": 120, "wall_s": 60}
    assert "GPU utilization" in saved["gpu"]["error"]


def test_benchmark_stops_exploration_before_each_window(tmp_path, monkeypatch):
    import subprocess

    benchmark = runpy.run_path(str(SCRIPT))
    calls = []
    monkeypatch.setattr(benchmark["platform"], "node", lambda: "tuf")
    monkeypatch.setenv("COMPOSE_PROJECT", "wrong-project")
    monkeypatch.setenv("EXPLORE_SECONDS", "99")
    monkeypatch.setattr(benchmark["time"], "sleep", lambda _: None)

    def run(command, **kwargs):
        if "simulation_launch.py" in " ".join(command):
            assert benchmark["os"].environ["COMPOSE_PROJECT"] == "swarmdeck"
            if "--down" not in command:
                assert command[command.index("-e") + 1] == "0"
        if command[:3] == ["docker", "exec", "-i"]:
            calls.append("stop")
            assert '"stop_explore"' in kwargs["input"]
        return subprocess.CompletedProcess(command, 0)

    def check_output(command, **kwargs):
        if command[:2] == ["docker", "ps"]:
            return ""
        return "fixture"

    def window(*args):
        calls.append("window")
        return {"rtf": {"rtf": 2}}

    monkeypatch.setattr(benchmark["subprocess"], "run", run)
    monkeypatch.setattr(benchmark["subprocess"], "check_output", check_output)
    monkeypatch.setitem(benchmark["main"].__globals__, "run_window", window)
    assert (
        benchmark["main"](
            ["--rounds", "1", "--warmup", "0", "--output", str(tmp_path / "results")]
        )
        == 0
    )
    assert calls == ["stop", "window"] * 3
