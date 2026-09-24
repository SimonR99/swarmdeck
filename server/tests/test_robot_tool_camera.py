"""Retired JPEG snapshot commands must give guidance, not an import traceback."""

from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("command", ["snap", "snapshot"])
def test_snapshot_command_exits_cleanly_with_media_pipeline_guidance(command, tmp_path):
    script = Path(__file__).resolve().parents[2] / "scripts" / "robot_tool.py"
    output = tmp_path / "snapshot.jpg"
    result = subprocess.run(
        [sys.executable, str(script), command, "robot_0", "--save", str(output)],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 1
    assert result.stderr.strip() == (
        "camera snapshots are served by the media pipeline (RTSP/WHEP); "
        "use `doctor` or the dashboard"
    )
    assert "Traceback" not in result.stdout + result.stderr
    assert not output.exists()
