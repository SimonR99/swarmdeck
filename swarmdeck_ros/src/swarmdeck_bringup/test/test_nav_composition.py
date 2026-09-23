"""Simulation composes Nav2 without changing hardware defaults or ROS contracts."""

import importlib.util
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytest.importorskip("launch_ros", reason="Nav2 launch tests require the ROS image")
import rclpy
from composition_interfaces.srv import ListNodes
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters

ROOT = Path(__file__).resolve().parents[4]
NAV = ROOT / "swarmdeck_ros/src/swarmdeck_nav/launch/nav.launch.py"
PARAMS = ROOT / "swarmdeck_ros/src/swarmdeck_nav/config/nav2_params.yaml"


def test_nav_composition_is_opt_in_for_hardware():
    from launch.actions import DeclareLaunchArgument

    spec = importlib.util.spec_from_file_location("nav_launch", NAV)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments["use_composition"].default_value[0].text == "false"
    session = ROOT / "swarmdeck_ros/src/swarmdeck_bringup/launch/session.launch.py"
    assert '"use_composition": "true"' in session.read_text()


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    try:
        assert client.wait_for_service(timeout_sec=20), name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        assert future.done(), name
        return future.result()
    finally:
        node.destroy_client(client)


@pytest.mark.parametrize("composed", [False, True])
def test_nav_loads_and_activates_with_namespaced_contracts(tmp_path, composed):
    log = (tmp_path / "nav.log").open("w+")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            str(NAV),
            "namespace:=dds_nav",
            f"use_composition:={str(composed).lower()}",
            "use_sim_time:=false",
            "bounded_startup:=true",
            f"params_file:={PARAMS}",
            "robot_radius:=0.51",
            "inflation_radius:=0.76",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("nav_composition_test")
    transform = TransformStamped()
    transform.header.frame_id = "dds_nav/odom"
    transform.child_frame_id = "dds_nav/base_link"
    transform.transform.rotation.w = 1.0
    # Nav2 deliberately subscribes to namespaced TF, not the broadcaster default.
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_msgs.msg import TFMessage

    tf_pub = node.create_publisher(
        TFMessage,
        "/dds_nav/tf_static",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    tf_pub.publish(TFMessage(transforms=[transform]))
    try:
        for name in ("controller_server", "velocity_smoother"):
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                state = call(
                    node, GetState, f"/dds_nav/{name}/get_state", GetState.Request()
                )
                if state.current_state.id == 3:
                    break
                time.sleep(0.1)
            assert state.current_state.id == 3, name
        if composed:
            loaded = call(
                node,
                ListNodes,
                "/dds_nav/nav_container/_container/list_nodes",
                ListNodes.Request(),
            )
            assert set(loaded.full_node_names) == {
                "/dds_nav/controller_server",
                "/dds_nav/velocity_smoother",
            }
        else:
            assert (
                "nav_container",
                "/dds_nav",
            ) not in node.get_node_names_and_namespaces()
        params = call(
            node,
            GetParameters,
            "/dds_nav/local_costmap/local_costmap/get_parameters",
            GetParameters.Request(
                names=[
                    "robot_base_frame",
                    "robot_radius",
                    "inflation_layer.inflation_radius",
                ]
            ),
        )
        assert params.values[0].string_value == "dds_nav/base_link"
        assert params.values[1].double_value == 0.51
        assert params.values[2].double_value == 0.76
        assert node.count_publishers("/dds_nav/cmd_vel_nav") == 1
        assert node.count_subscribers("/dds_nav/cmd_vel_nav") == 1
        assert node.count_publishers("/dds_nav/cmd_vel") == 1
        assert node.count_subscribers("/dds_nav/tf_static") >= 1
    finally:
        import os

        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        print(log.read())
        log.close()
