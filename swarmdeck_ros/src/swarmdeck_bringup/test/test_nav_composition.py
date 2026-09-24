"""Simulation composes Nav2 without changing hardware defaults or ROS contracts."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytest.importorskip("launch_ros", reason="Nav2 launch tests require the ROS image")
import rclpy
from composition_interfaces.srv import ListNodes, LoadNode
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from launch.actions import DeclareLaunchArgument
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[4]
NAV = ROOT / "swarmdeck_ros/src/swarmdeck_nav/launch/nav.launch.py"
PARAMS = ROOT / "swarmdeck_ros/src/swarmdeck_nav/config/nav2_params.yaml"


def test_nav_composition_is_opt_in_for_hardware():
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


# Keeps only the load requests of nav.launch.py, aimed at a fake container.
NAV_LOADS_ONLY = f"""
import importlib.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes

spec = importlib.util.spec_from_file_location("nav_launch", {str(NAV)!r})
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


def generate_launch_description():
    kinds = (DeclareLaunchArgument, LoadComposableNodes)
    entities = nav.generate_launch_description().entities
    return LaunchDescription([e for e in entities if isinstance(e, kinds)])
"""

# launch_ros's own batching: one action loading two nodes in sequence.
BATCHED_LOADS = """
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nodes = [
        ComposableNode(package="fake", plugin="fake::Node", name=name)
        for name in ("controller_server", "velocity_smoother")
    ]
    return LaunchDescription([
        LoadComposableNodes(
            target_container="/lost_reply/nav_container",
            composable_node_descriptions=nodes,
        )
    ])
"""


@pytest.mark.parametrize(
    "loads, expected",
    [(BATCHED_LOADS, 1), (NAV_LOADS_ONLY, 2)],
    ids=["launch_ros_batch_blocks", "nav_launch_loads_each"],
)
def test_lost_load_reply_does_not_block_other_nav_nodes(tmp_path, loads, expected):
    """Fast DDS can drop a load_node reply during discovery (tuf, robot_2).

    launch_ros waits for each reply without a timeout before sending the next
    request of the same action, so a batched container never loaded the
    velocity smoother. nav.launch.py must request every node independently.
    """
    rclpy.init()
    node = None
    process = None
    launch_file = tmp_path / "loads.launch.py"
    launch_file.write_text(loads)
    log = (tmp_path / "loads.log").open("w+")
    requested = []
    lost = Future()

    async def load_node(request, response):
        requested.append(request.node_name)
        if len(requested) == 1:
            # The container loaded the node, but the reply never arrives.
            await lost
        response.success = True
        response.full_node_name = f"/lost_reply/{request.node_name}"
        return response

    try:
        node = rclpy.create_node("nav_container", namespace="lost_reply")
        # Reentrant: the container stays free while the lost reply is pending.
        node.create_service(
            LoadNode,
            "~/_container/load_node",
            load_node,
            callback_group=ReentrantCallbackGroup(),
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                str(launch_file),
                "namespace:=lost_reply",
                "use_composition:=true",
                "use_sim_time:=false",
                f"params_file:={PARAMS}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 20
        while len(requested) < expected and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        # A second request would follow the first within milliseconds.
        settle = time.monotonic() + 3
        while time.monotonic() < settle:
            executor.spin_once(timeout_sec=0.1)
        assert len(requested) == expected, requested
        if expected == 2:
            assert set(requested) == {"controller_server", "velocity_smoother"}
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        lost.cancel()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        print(log.read())
        log.close()


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


def test_failed_ros_initialization_never_starts_nav_process(tmp_path, monkeypatch):
    started = []

    def fail_init():
        raise RuntimeError("ROS init failed")

    monkeypatch.setattr(rclpy, "init", fail_init)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: started.append(a))
    with pytest.raises(RuntimeError, match="ROS init failed"):
        test_nav_loads_and_activates_with_namespaced_contracts(tmp_path, True)
    assert started == []


def test_failed_nav_process_start_shuts_down_ros(tmp_path, monkeypatch):
    def fail_start(*args, **kwargs):
        raise OSError("launch unavailable")

    monkeypatch.setattr(subprocess, "Popen", fail_start)
    with pytest.raises(OSError, match="launch unavailable"):
        test_nav_loads_and_activates_with_namespaced_contracts(tmp_path, True)
    assert not rclpy.ok()


@pytest.mark.parametrize("composed", [False, True])
def test_nav_loads_and_activates_with_namespaced_contracts(tmp_path, composed):
    rclpy.init()
    node = None
    process = None
    log = (tmp_path / "nav.log").open("w+")
    try:
        node = rclpy.create_node("nav_composition_test")
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
        transform = TransformStamped()
        transform.header.frame_id = "dds_nav/odom"
        transform.child_frame_id = "dds_nav/base_link"
        transform.transform.rotation.w = 1.0
        # Nav2 subscribes to namespaced TF, not the broadcaster default.
        tf_pub = node.create_publisher(
            TFMessage,
            "/dds_nav/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        tf_pub.publish(TFMessage(transforms=[transform]))
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
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        print(log.read())
        log.close()
