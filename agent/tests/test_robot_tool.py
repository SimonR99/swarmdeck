import importlib.util
from pathlib import Path
from types import SimpleNamespace


REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("robot_tool", REPO / "scripts" / "robot_tool.py")
assert SPEC and SPEC.loader
robot_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(robot_tool)


def test_robot_aliases_match_deployment_profiles():
    assert robot_tool.normalize_robot_id("@scout") == "tars_0"
    assert robot_tool.normalize_robot_id("tars") == "tars_0"
    assert robot_tool.normalize_robot_id("SPOT") == "spot_0"


def test_ssh_errors_do_not_confuse_authentication_with_network():
    assert robot_tool._classify_ssh_error("Permission denied (publickey)") == (
        "SSH authentication failed"
    )
    assert robot_tool._classify_ssh_error("connect timed out") == (
        "robot host is unreachable"
    )


def test_doctor_requires_progressing_media_packets(monkeypatch):
    fleet = {
        "robots": [
            {
                "robot_id": "tars_0",
                "online": True,
                "battery": 0.5,
                "mode": "idle",
                "nav_status": "idle",
            }
        ]
    }
    vision_calls = iter(
        [
            ({"camera_streaming": True, "frame_seq": 4, "frame_age_ms": 50}, None),
            ({"camera_streaming": True, "frame_seq": 5, "frame_age_ms": 40}, None),
        ]
    )

    def fake_get(endpoint, _server):
        if endpoint == "/api/fleet":
            return fleet, None
        return next(vision_calls)

    monkeypatch.setattr(robot_tool, "_try_http_get", fake_get)
    monkeypatch.setattr(
        robot_tool,
        "probe_rtsp_stream",
        lambda *_args: {"ok": True, "status": 200},
    )
    monkeypatch.setattr(
        robot_tool,
        "probe_video_packets",
        lambda *_args: {"ok": True, "packet_count": 2, "codec": "H264"},
    )
    args = SimpleNamespace(
        server="http://server:8080",
        robot="scout",
        sample_seconds=0,
        rtsp_base_url="rtsp://mediamtx:8554",
        services=False,
    )

    report = robot_tool.collect_doctor_report(args)

    assert report["ok"] is True
    assert report["robots"][0]["camera"]["progressing"] is True
    assert report["robots"][0]["media"]["ok"] is True


def test_doctor_rejects_rtsp_metadata_without_frame_evidence(monkeypatch):
    fleet = {"robots": [{"robot_id": "spot_0", "online": True}]}
    monkeypatch.setattr(
        robot_tool,
        "_try_http_get",
        lambda endpoint, _server: (fleet, None)
        if endpoint == "/api/fleet"
        else ({"camera_streaming": False, "frame_seq": None, "frame_age_ms": None}, None),
    )
    monkeypatch.setattr(
        robot_tool,
        "probe_rtsp_stream",
        lambda *_args: {"ok": True, "status": 200},
    )
    monkeypatch.setattr(
        robot_tool,
        "probe_video_packets",
        lambda *_args: {"ok": False, "error": "no progressing RTP packets observed"},
    )
    args = SimpleNamespace(
        server="http://server:8080",
        robot="spot",
        sample_seconds=0,
        rtsp_base_url="rtsp://mediamtx:8554",
        services=False,
    )

    report = robot_tool.collect_doctor_report(args)

    assert report["ok"] is False
    assert report["robots"][0]["rtsp"]["ok"] is True
    assert report["robots"][0]["media"]["ok"] is False
