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
