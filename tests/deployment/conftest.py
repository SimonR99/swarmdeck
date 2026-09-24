"""Markers for explicitly selected deployment integration tests."""


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "docker: requires Docker and a prebuilt ROS image"
    )
