"""Unavailable image inspection must fail cleanly instead of importing old code."""

from pathlib import Path
import subprocess
import sys


def test_inspect_command_exits_cleanly_with_unavailable_message(tmp_path):
    script = Path(__file__).resolve().parents[2] / "scripts" / "robot_tool.py"
    result = subprocess.run(
        [sys.executable, str(script), "inspect", str(tmp_path / "image.png")],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 1
    assert result.stderr.strip() == "image inspection is not available in robot_tool"
    assert "Traceback" not in result.stdout + result.stderr
