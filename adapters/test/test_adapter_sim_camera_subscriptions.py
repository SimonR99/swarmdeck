"""The media process owns dashboard video; the adapter only needs RGB-D for detection."""

from unittest.mock import MagicMock


def test_camera_subscriptions_only_when_detector_configured(sim_module, monkeypatch):
    monkeypatch.setattr(sim_module, "ObjectDetector", MagicMock())
    monkeypatch.setattr("adapters.exploration.configure_exploration", lambda bridge: None)
    monkeypatch.setattr("adapters.objective_planning.configure_objective_planning", lambda bridge: None)
    for url, expected in ((None, set()), ("http://detector", {
        "/robot_0/camera/image", "/robot_0/camera/depth_image",
        "/robot_0/camera/camera_info",
    })):
        if url is None:
            monkeypatch.delenv("SWARMDECK_DETECTOR_URL", raising=False)
        else:
            monkeypatch.setenv("SWARMDECK_DETECTOR_URL", url)
        node = MagicMock()
        sim_module.RobotBridge(node, "robot_0", "http://backend")
        topics = {call.args[1] for call in node.create_subscription.call_args_list}
        assert {topic for topic in topics if "/camera/" in topic} == expected
