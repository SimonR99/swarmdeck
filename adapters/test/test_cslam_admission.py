"""Guard the native peer path's inter-robot admission contract.

`cslam_lidar.yaml` is the only place the peer front end's acceptance thresholds
are set, and its own header records why that is dangerous: ROS 2 accepts a
parameter name no node declares, in silence. These tests hold the three things
that a measured relaxation of those thresholds depends on:

* the overlap gate exists in the patch chain, is declared, is read, and is
  passed to both registration call sites;
* the shipped config never relaxes the inlier threshold without that gate,
  because the two were measured together and the relaxation is only safe with
  it (see the measurement block in `cslam_lidar.yaml`);
* the image actually applies the patch, after the two patches it builds on.
"""

from pathlib import Path
import re

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "swarmdeck_ros/src/swarmdeck_cslam/config/cslam_lidar.yaml"
OVERLAP_PATCH = REPO / "deploy/patches/cslam-registration-overlap.patch"
DOCKERFILE = REPO / "deploy/docker/Dockerfile.cslam"
# MISTLab's own lidar default. Anything at or below it is a relaxation of the
# only geometric gate upstream has.
UPSTREAM_MIN_INLIERS = 100


@pytest.fixture(scope="module")
def frontend():
    document = yaml.safe_load(CONFIG.read_text())
    return document["/**"]["ros__parameters"]["frontend"]


def added_lines(patch: str) -> list[str]:
    return [line[1:] for line in patch.splitlines() if line.startswith("+")]


def test_overlap_patch_declares_reads_and_applies_the_new_parameter():
    patch = OVERLAP_PATCH.read_text()
    added = "\n".join(added_lines(patch))
    assert (
        "('frontend.registration_min_overlap', 0.0)," in added
    ), "the parameter must be declared, or the node rejects it at startup"
    assert "params['frontend.registration_min_overlap'] = node.get_parameter(" in added
    # Both registration call sites, inter-robot and intra-robot, must pass it;
    # a gate applied to only one of them is a silent asymmetry in the graph.
    call_sites = [
        line
        for line in added_lines(patch)
        if "icp_utils.compute_transform(" in line
        and 'self.params["frontend.registration_min_overlap"]' in line
    ]
    assert len(call_sites) == 2, call_sites
    assert (
        "def compute_transform(src, dst, voxel_size, min_inliers, min_overlap=0.0):"
        in added
    )
    # Defaulting to 0.0 keeps upstream behaviour for anyone who does not set it.
    assert "min_overlap=0.0" in added


def test_short_overlap_refuses_instead_of_publishing_the_refinement():
    added = added_lines(OVERLAP_PATCH.read_text())
    body = "\n".join(added)
    assert "if overlap < min_overlap:" in body
    assert "valid = False" in body
    # The refusal must also keep the refined transform out of the solution,
    # otherwise a rejected pair still mutates what the caller reads back.
    refused = body.index("if overlap < min_overlap:")
    assert "else:" in body[refused:]
    assert body.index("solution.translation = T_icp[:3, 3]") > refused


def test_relaxed_inlier_threshold_never_ships_without_the_overlap_gate(frontend):
    inliers = frontend["registration_min_inliers"]
    overlap = frontend.get("registration_min_overlap")
    if inliers > UPSTREAM_MIN_INLIERS:
        return  # stricter than upstream; the overlap gate is then optional
    assert overlap is not None, (
        "registration_min_inliers was relaxed to or below upstream's default "
        "without setting registration_min_overlap; the measured result is "
        "false inter-robot merges"
    )
    assert 0.0 < overlap < 1.0, overlap


def test_similarity_threshold_stays_within_the_scan_context_metric(frontend):
    # `similarity` is ScanContext's mean column cosine similarity, which the
    # measurement in cslam_lidar.yaml shows peaks near 0.9 on Bistro keyframes.
    # A threshold above that admits nothing, which is how this shipped broken.
    assert 0.0 < frontend["similarity_threshold"] <= 0.9


def test_image_applies_the_overlap_patch_after_the_patches_it_builds_on():
    text = DOCKERFILE.read_text()
    assert "COPY deploy/patches/cslam-registration-overlap.patch" in text
    order = [
        text.index(f"git apply /tmp/cslam-{name}.patch")
        for name in (
            "keyframe-readiness",
            "registration-budget",
            "registration-overlap",
        )
    ]
    assert order == sorted(order), "the overlap patch must be applied last"


def test_config_sets_no_frontend_key_the_patch_chain_leaves_undeclared(frontend):
    """Keys this repo invented must be declared by this repo's patches.

    Upstream's own keys are declared in upstream; the failure this catches is a
    SwarmDeck key that only exists in the yaml, which ROS 2 would accept and
    then ignore.
    """
    patched = set()
    for patch in (REPO / "deploy/patches").glob("cslam-*.patch"):
        patched.update(
            re.findall(
                r"\('(frontend\.[a-z_]+)'", "\n".join(added_lines(patch.read_text()))
            )
        )
    invented = {
        "frontend.registration_min_overlap",
        "frontend.keyframe_min_subscribers",
    }
    for key in invented:
        if key.split(".", 1)[1] in frontend:
            assert key in patched, f"{key} is set but no cslam patch declares it"
