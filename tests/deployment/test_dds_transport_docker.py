"""Run explicitly: pytest -m docker tests/deployment/test_dds_transport_docker.py.

Uses only auto-removed isolated containers; never joins or changes a live stack.
This directory is outside the default pytest testpaths.
"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from uuid import uuid4

import pytest

pytestmark = pytest.mark.docker
ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def sim_image():
    if shutil.which("docker") is None:
        pytest.skip("Docker executable unavailable")
    image = os.environ.get("SWARMDECK_DDS_TEST_IMAGE", "swarmdeck-sim:local")
    try:
        subprocess.run(["docker", "info"], check=True, capture_output=True, timeout=10)
        subprocess.run(
            ["docker", "image", "inspect", image],
            check=True,
            capture_output=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        pytest.skip(f"Docker or prebuilt ROS sim image unavailable: {exc}")
    return image


def summary(output):
    return json.loads(
        next(line for line in output.splitlines() if line.startswith("{"))
    )


def measure_profile(image, profile):
    name = f"swarmdeck-dds-test-{uuid4().hex}"
    common = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{ROOT / 'deploy/dds'}:/dds:ro",
        "-v",
        f"{ROOT / 'tests/deployment'}:/test:ro",
        "-e",
        f"FASTRTPS_DEFAULT_PROFILES_FILE=/dds/{profile}",
        "-e",
        "ROS_DOMAIN_ID=197",
    ]
    # Bound the container itself too: terminating docker exec/run on the host
    # alone would not necessarily stop the process inside the container.
    entry = ["--entrypoint", "timeout", image, "--kill-after=2", "60", "bash", "-lc"]
    command = (
        "source /opt/ros/jazzy/setup.bash && exec python3 /test/dds_transport_probe.py "
    )
    writer = subprocess.Popen(
        common
        + [
            "--name",
            name,
            "--network",
            "none",
            "--ipc",
            "shareable",
            "--shm-size",
            "256m",
        ]
        + entry
        + [command + "writer"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            assert writer.poll() is None, "writer exited before reader startup"
            check = subprocess.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", name],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if check.returncode == 0 and check.stdout.strip() == "true":
                break
            time.sleep(0.2)
        else:
            pytest.fail("DDS writer container did not start")
        reader = subprocess.run(
            common
            + ["--network", f"container:{name}", "--ipc", f"container:{name}"]
            + entry
            + [command + "reader"],
            capture_output=True,
            text=True,
            timeout=70,
        )
        output, _ = writer.communicate(timeout=70)
        assert writer.returncode == 0, output
        assert reader.returncode == 0, reader.stdout + reader.stderr
        assert summary(reader.stdout) == {"received": 50, "sizes": [1048576]}
        result = summary(output)
        print(profile, result)
        return result
    finally:
        # All normal probe paths finish within 35s; the in-container timeout
        # guarantees eventual removal even after a host-side assertion fails.
        if writer.poll() is None:
            writer.communicate(timeout=70)


def test_shared_memory_delivers_payloads_without_loopback_data_traffic(sim_image):
    udp = measure_profile(sim_image, "fastdds_udp_only.xml")
    shm = measure_profile(sim_image, "fastdds_large_data.xml")
    assert udp["sent"] == shm["sent"] == 50
    assert udp["loopback_tx_bytes"] >= 50 * 1048576
    assert shm["loopback_tx_bytes"] < udp["loopback_tx_bytes"] * 0.1
    assert udp["shm_sizes"] == []
    assert max(shm["shm_sizes"]) >= 16777216
