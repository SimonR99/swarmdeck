"""Transport-independent checks for MGG depth-ray admission."""

import ast
import copy
from pathlib import Path

import numpy as np
from types import SimpleNamespace


def _depth_filter():
    source = Path("deploy/mgg/inputs.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name)
            and target.id in {"SIM_DEPTH_FAR_PLANE_M", "SIM_DEPTH_PIXEL_STRIDE"}
            for target in node.targets
        ):
            selected.append(node)
        if isinstance(node, ast.FunctionDef) and node.name == "valid_depth_samples":
            selected.append(node)
    namespace = {"np": np}
    exec(compile(ast.Module(body=selected, type_ignores=[]), source, "exec"), namespace)
    return namespace["valid_depth_samples"]


def _input_callbacks():
    source = Path("deploy/mgg/inputs.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    selected = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "has_capture_stamp":
            selected.append(copy.deepcopy(node))
        if isinstance(node, ast.ClassDef) and node.name == "Inputs":
            selected.extend(
                copy.deepcopy(method)
                for method in node.body
                if isinstance(method, ast.FunctionDef)
                and method.name in {"on_cloud", "on_depth"}
            )
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), source, "exec"), namespace)
    return namespace["on_cloud"], namespace["on_depth"]


def test_zero_time_cannot_request_latest_transform_for_mapping_input():
    on_cloud, on_depth = _input_callbacks()

    def message(sec, nanosec):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        )

    owner = SimpleNamespace(latest="older-cloud", depth="older-depth")
    on_cloud(owner, message(0, 0))
    on_depth(owner, message(0, 0))
    assert owner.latest == "older-cloud"
    assert owner.depth == "older-depth"

    cloud = message(0, 1)
    depth = message(1, 0)
    on_cloud(owner, cloud)
    on_depth(owner, depth)
    assert owner.latest is cloud
    assert owner.depth is depth


def test_sim_depth_uses_stride_two_for_full_body_coverage():
    source = Path("deploy/mgg/inputs.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    values = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name) and target.id == "SIM_DEPTH_PIXEL_STRIDE"
    }
    stride = values["SIM_DEPTH_PIXEL_STRIDE"]
    sample = np.empty((240, 320), dtype=np.float32)[::stride, ::stride]
    assert stride == 2
    assert sample.shape == (120, 160)


def test_sim_far_plane_is_a_free_ray_candidate_but_hardware_stays_unknown():
    valid = _depth_filter()
    depths = np.array(
        [0.0, np.nan, np.inf, 0.05, 1.0, 19.999, 20.0, 39.0, 40.0, 40.001],
        dtype=np.float32,
    )

    assert valid(depths, True).tolist() == [
        False, False, False, False, True, True, True, True, True, False
    ]
    assert valid(depths, False).tolist() == [
        False, False, False, False, True, True, False, False, False, False
    ]
    # An incompatible native range must not turn the 40 m no-return sentinel
    # into an occupied endpoint.
    assert not valid(depths, True, 40.0)[-2]
