"""Raw RTSP fallback must rate-gate before touching pixels."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock


def test_raw_callback_skips_pixel_conversion_when_fps_gate_closed(monkeypatch):
    media = Path(__file__).resolve().parents[1] / "media" / "ros2_rtsp.py"
    gi = NS(require_version=lambda *args: None, repository=NS(Gst=NS()))
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", gi.repository)
    monkeypatch.setitem(sys.modules, "rclpy", NS())
    monkeypatch.setitem(sys.modules, "rclpy.node", NS(Node=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(
        sys.modules, "sensor_msgs.msg", NS(CompressedImage=object, Image=object)
    )
    spec = importlib.util.spec_from_file_location("ros2_rtsp_test", media)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    conversion = Mock(side_effect=AssertionError("gate must run before conversion"))
    monkeypatch.setattr(module, "raw_frame_bytes", conversion)
    publisher = NS(
        _failed=NS(is_set=lambda: False),
        _last_compressed_at=0.0,
        raw_source=object(),
        _can_push=lambda source: False,
    )
    module.Ros2JpegRtspPublisher._on_raw_frame(publisher, NS())
    conversion.assert_not_called()


def test_compressed_preference_timeout_is_named_and_raw_fallback_resumes(monkeypatch):
    media = Path(__file__).resolve().parents[1] / "media" / "ros2_rtsp.py"
    gi = NS(require_version=lambda *args: None, repository=NS(Gst=NS()))
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", gi.repository)
    monkeypatch.setitem(sys.modules, "rclpy", NS())
    monkeypatch.setitem(sys.modules, "rclpy.node", NS(Node=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(
        sys.modules, "sensor_msgs.msg", NS(CompressedImage=object, Image=object)
    )
    spec = importlib.util.spec_from_file_location("ros2_rtsp_test_preference", media)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.COMPRESSED_PREFERENCE_TIMEOUT_S == 2.0
    converted = Mock(return_value=None)
    monkeypatch.setattr(module, "raw_frame_bytes", converted)
    publisher = NS(
        _failed=NS(is_set=lambda: False),
        _last_compressed_at=100.0,
        _can_push=lambda source: True,
        raw_source=object(),
    )
    monkeypatch.setattr(module.time, "monotonic", lambda: 101.999)
    module.Ros2JpegRtspPublisher._on_raw_frame(publisher, NS())
    converted.assert_not_called()
    monkeypatch.setattr(module.time, "monotonic", lambda: 102.0)
    module.Ros2JpegRtspPublisher._on_raw_frame(publisher, NS())
    converted.assert_called_once()


def test_raw_callback_pushes_rgb_directly_to_raw_input(monkeypatch):
    media = Path(__file__).resolve().parents[1] / "media" / "ros2_rtsp.py"
    gi = NS(
        require_version=lambda *args: None,
        repository=NS(Gst=NS(Caps=NS(from_string=lambda text: text))),
    )
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", gi.repository)
    monkeypatch.setitem(sys.modules, "rclpy", NS())
    monkeypatch.setitem(sys.modules, "rclpy.node", NS(Node=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(
        sys.modules, "sensor_msgs.msg", NS(CompressedImage=object, Image=object)
    )
    spec = importlib.util.spec_from_file_location("ros2_rtsp_test_raw", media)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = NS(set_property=Mock())
    pushed = Mock()
    publisher = NS(
        _failed=NS(is_set=lambda: False),
        _last_compressed_at=0.0,
        _can_push=lambda source: True,
        raw_source=source,
        _raw_caps=None,
        _frame_period_s=0.1,
        _push_frame=pushed,
    )
    image = NS(encoding="rgb8", width=1, height=1, step=3, data=b"\x01\x02\x03")
    module.Ros2JpegRtspPublisher._on_raw_frame(publisher, image)
    pushed.assert_called_once_with(source, b"\x01\x02\x03", "sink_1")
    assert "format=RGB" in source.set_property.call_args.args[1]
