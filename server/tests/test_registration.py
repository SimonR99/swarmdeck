"""Map registration tests — synthetic, deterministic, no ROS and no simulator."""

from __future__ import annotations

import math

import numpy as np
import pytest

from swarmdeck_server.mapsvc.registration import COARSE_STEP, register
from swarmdeck_server.mapsvc.service import GridMeta, MapService

RES, N, OX, OY = 0.05, 400, -10.0, -10.0


def _rect_fn(g):
    def rect(x0, y0, x1, y1, v=100):
        i0, i1 = int((x0 - OX) / RES), int((x1 - OX) / RES)
        j0, j1 = int((y0 - OY) / RES), int((y1 - OY) / RES)
        g[j0:j1, i0:i1] = v

    return rect


def _shell(rect):
    rect(-8, -8, 8, -7.8)
    rect(-8, 7.8, 8, 8)
    rect(-8, -8, -7.8, 8)
    rect(7.8, -8, 8, 8)


def _flood_free(g):
    free = np.zeros_like(g, bool)
    i0, i1 = int((-7.8 - OX) / RES), int((7.8 - OX) / RES)
    j0, j1 = int((-7.8 - OY) / RES), int((7.8 - OY) / RES)
    free[j0:j1, i0:i1] = True
    g[free & (g == -1)] = 0
    return g


def build_plan() -> np.ndarray:
    """An asymmetric floor plan: outer shell plus distinctive interior walls."""
    g = np.full((N, N), -1, np.int8)
    rect = _rect_fn(g)
    _shell(rect)
    rect(-2, -8, -1.8, 2)      # asymmetric interior
    rect(3, -1, 3.2, 8)
    rect(-8, 3, -2, 3.2)
    rect(4.5, -5, 7.8, -4.8)
    return _flood_free(g)


def build_symmetric_plan() -> np.ndarray:
    """A square shell with a centred cross: identical under 90 deg rotation.

    There is no unique answer here, so the only correct behaviour is to refuse.
    """
    g = np.full((N, N), -1, np.int8)
    rect = _rect_fn(g)
    _shell(rect)
    rect(-0.1, -8, 0.1, 8)
    rect(-8, -0.1, 8, 0.1)
    return _flood_free(g)


def reframe(g: np.ndarray, dx: float, dy: float, dyaw: float) -> np.ndarray:
    """Re-express the same plan in a frame offset by (dx, dy, dyaw)."""
    cx = OX + (np.arange(N) + 0.5) * RES
    cy = OY + (np.arange(N) + 0.5) * RES
    WX, WY = np.meshgrid(cx, cy)
    c, s = math.cos(dyaw), math.sin(dyaw)
    sx = WX * c - WY * s + dx
    sy = WX * s + WY * c + dy
    gx = np.floor((sx - OX) / RES).astype(int)
    gy = np.floor((sy - OY) / RES).astype(int)
    ok = (gx >= 0) & (gx < N) & (gy >= 0) & (gy < N)
    out = np.full_like(g, -1)
    out[ok] = g[gy[ok], gx[ok]]
    return out


@pytest.mark.parametrize(
    "dx,dy,dyaw_deg",
    [(0, 0, 0), (2.0, -1.5, 0), (0, 0, 35), (3.0, 2.0, -60), (-1.5, 4.0, 120), (2.5, -3.0, 175)],
)
def test_register_recovers_transform(dx, dy, dyaw_deg):
    ref = build_plan()
    mov = reframe(ref, dx, dy, math.radians(dyaw_deg))
    r = register(ref, (RES, OX, OY), mov, (RES, OX, OY))

    assert r.confident, f"not confident: score={r.score} ratio={r.ratio}"
    assert math.hypot(r.dx - dx, r.dy - dy) < 0.35, f"translation off: {r.dx},{r.dy}"
    dyaw_err = (math.degrees(r.dyaw) - dyaw_deg + 180) % 360 - 180
    assert abs(dyaw_err) < 2.0, f"yaw off by {dyaw_err} deg"


# Yaws deliberately off the coarse sweep's sampling grid. The correlation peak in
# yaw is under a degree wide, so a coarse pass that samples on this grid without
# blurring first misses the true peak entirely and locks onto the square shell's
# 90 deg symmetry — reporting high confidence and a 90 deg error.
@pytest.mark.parametrize("dyaw_deg", [2.0, 17.0, 30.0, 45.0, 58.0, 93.0, 137.0, -26.0, -110.0])
def test_register_survives_yaw_off_the_coarse_grid(dyaw_deg):
    assert not np.any(
        np.isclose(np.arange(-180.0, 180.0, COARSE_STEP), dyaw_deg)
    ), "this case is meant to fall between coarse samples"

    ref = build_plan()
    mov = reframe(ref, 1.1, -0.6, math.radians(dyaw_deg))
    r = register(ref, (RES, OX, OY), mov, (RES, OX, OY))

    assert r.confident, f"not confident: score={r.score} yaw_ratio={r.yaw_ratio}"
    dyaw_err = (math.degrees(r.dyaw) - dyaw_deg + 180) % 360 - 180
    assert abs(dyaw_err) < 1.0, f"yaw off by {dyaw_err} deg"
    assert math.hypot(r.dx - 1.1, r.dy + 0.6) < 0.10, f"translation off: {r.dx},{r.dy}"


def test_register_is_accurate_to_sub_cell():
    """Sub-cell interpolation, not the search raster, sets the accuracy floor."""
    ref = build_plan()
    mov = reframe(ref, 1.37, -2.09, math.radians(23.0))
    r = register(ref, (RES, OX, OY), mov, (RES, OX, OY))

    assert r.confident
    assert math.hypot(r.dx - 1.37, r.dy + 2.09) < 0.08
    assert abs((math.degrees(r.dyaw) - 23.0 + 180) % 360 - 180) < 0.5


@pytest.mark.parametrize("dyaw_deg", [0.0, 17.0, 43.0])
def test_register_refuses_a_symmetric_building(dyaw_deg):
    """A 90 deg-symmetric plan has no unique answer, so it must not report one."""
    ref = build_symmetric_plan()
    mov = reframe(ref, 1.0, -0.5, math.radians(dyaw_deg))
    r = register(ref, (RES, OX, OY), mov, (RES, OX, OY))

    assert not r.confident, (
        f"claimed a unique alignment of a symmetric plan: "
        f"score={r.score} ratio={r.ratio} yaw_ratio={r.yaw_ratio}"
    )


def test_register_refuses_when_coverage_barely_overlaps():
    """A good score over a sliver of shared area is not evidence."""
    ref = build_plan()
    mov = reframe(ref, 0.0, 0.0, math.radians(20.0))
    west = ref.copy()
    west[:, int((-4.0 - OX) / RES):] = -1
    east = mov.copy()
    east[:, : int((4.0 - OX) / RES)] = -1

    r = register(west, (RES, OX, OY), east, (RES, OX, OY))
    assert r.support < 1.0
    if r.confident:
        dyaw_err = (math.degrees(r.dyaw) - 20.0 + 180) % 360 - 180
        assert abs(dyaw_err) < 2.0, "confident but wrong on near-disjoint coverage"


def test_yaw_prior_restricts_the_search():
    """A prior must both find the answer and refuse one outside its window."""
    ref = build_plan()
    mov = reframe(ref, 1.1, -0.6, math.radians(37.0))

    good = register(
        ref, (RES, OX, OY), mov, (RES, OX, OY), yaw_prior=math.radians(30.0)
    )
    assert good.confident
    assert abs((math.degrees(good.dyaw) - 37.0 + 180) % 360 - 180) < 1.0

    # The true yaw is far outside this window, so the true peak is not a
    # candidate and the result must not be presented as trustworthy.
    blind = register(
        ref,
        (RES, OX, OY),
        mov,
        (RES, OX, OY),
        yaw_prior=math.radians(-120.0),
        yaw_window_deg=10.0,
    )
    assert abs((math.degrees(blind.dyaw) - 37.0 + 180) % 360 - 180) > 5.0
    assert not blind.confident


def test_register_rejects_when_no_overlap():
    """Two unrelated maps must not produce a confident answer."""
    ref = build_plan()
    rng = np.random.default_rng(0)
    noise = np.full((N, N), -1, np.int8)
    idx = rng.integers(0, N, size=(2, 4000))
    noise[idx[0], idx[1]] = 100
    r = register(ref, (RES, OX, OY), noise, (RES, OX, OY))
    assert not r.confident


def test_register_rejects_too_little_data():
    empty = np.full((N, N), -1, np.int8)
    r = register(build_plan(), (RES, OX, OY), empty, (RES, OX, OY))
    assert not r.confident
    assert r.overlap == 0


def test_merge_applies_rotation():
    """_warp must honour yaw, not just translation."""
    svc = MapService(resolution=0.1, size_m=20.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10.0, -10.0)

    cells = np.full((n, n), -1, np.int8)
    cells[100:120, 100:104] = 100  # a bar just +x/+y of that grid's origin

    svc.set_transform("r0", 0.0, 0.0, math.pi / 2)
    svc.ingest("r0", meta, cells)
    rotated = svc.merged.copy()

    svc2 = MapService(resolution=0.1, size_m=20.0)
    svc2.set_transform("r0", 0.0, 0.0, 0.0)
    svc2.ingest("r0", meta, cells)
    upright = svc2.merged

    assert (rotated >= 50).sum() > 0
    assert not np.array_equal(rotated, upright), "yaw was ignored by the merge"


def test_merge_occupied_beats_free():
    svc = MapService(resolution=0.1, size_m=20.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10.0, -10.0)

    a = np.full((n, n), -1, np.int8)
    a[80:120, 80:120] = 0
    b = np.full((n, n), -1, np.int8)
    b[90:110, 90:110] = 100

    svc.set_transform("r0", 0, 0, 0)
    svc.set_transform("r1", 0, 0, 0)
    svc.ingest("r0", meta, a)
    svc.ingest("r1", meta, b)

    assert (svc.merged >= 50).sum() > 0
    assert (svc.merged == 0).sum() > 0
    assert svc.merged.min() == -1


def test_static_mode_does_not_overlay_unconfigured_robots_at_identity():
    """Only the reference has an implicit identity transform.

    A second hardware robot owns an unrelated SLAM frame.  Treating a missing
    transform as identity used to put both its occupancy grid and XYZ cloud into
    the global map at the wrong place.
    """
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("static")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.full((n, n), -1, dtype=np.int8)
    cells[20:80, 20:80] = 0
    cells[20:80, 20] = 100

    svc.ingest("reference", meta, cells)
    svc.ingest("unconfigured", meta, cells)
    assert svc.global_members() == {"reference"}
    assert svc.status()["view_by_robot"]["unconfigured"] == "local"

    svc.set_transform("unconfigured", 2.0, 0.0, 0.0)
    assert svc.global_members() == {"reference", "unconfigured"}


def test_cloud_proposal_is_applied_only_after_grid_validation(monkeypatch):
    """Vertical structure proposes geometry; occupancy supplies independent support."""
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    ambiguous = Registration(
        dx=4.0,
        dy=4.0,
        dyaw=math.pi,
        score=0.1,
        overlap=20,
        ratio=0.95,
        yaw_ratio=0.95,
        support=0.2,
    )
    cloud_proposal = Registration(
        dx=0.0,
        dy=0.0,
        dyaw=0.0,
        score=0.4,
        overlap=120,
        ratio=0.3,
        yaw_ratio=0.3,
        support=0.2,
    )
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: ambiguous)
    monkeypatch.setattr(
        service_module, "register_3d", lambda *args, **kwargs: cloud_proposal
    )

    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.zeros((n, n), dtype=np.int8)
    cells[10:90, 15:18] = 100
    cells[60:63, 15:75] = 100
    cloud = np.column_stack(
        [
            np.linspace(-2.0, 2.0, 100),
            np.zeros(100),
            np.full(100, 0.3),
        ]
    ).astype(np.float32)

    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells.copy())
    assert svc.global_members() == set(), "the ambiguous grid alone must be refused"

    svc.set_cloud("r0", cloud)
    svc.set_cloud("r1", cloud.copy())
    status = svc.status()
    assert status["global_members"] == ["r0", "r1"]
    assert status["registrations"]["r1"]["source"] == "pointcloud+grid"
    assert svc.transforms["r1"] == pytest.approx((0.0, 0.0, 0.0))


def test_registration_cannot_override_configured_prior(monkeypatch):
    """A symmetric-map false positive must fail closed to the known start pose."""
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    wrong = Registration(dx=0.0, dy=0.0, dyaw=math.pi, score=0.6, overlap=300, ratio=0.5)
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: wrong)

    svc = MapService(resolution=0.1, size_m=20.0)
    svc.set_mode("auto")
    svc.set_transform("reference", 3.0, 0.0, math.pi)
    svc.set_transform("moving", -9.0, 0.0, 0.0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10.0, -10.0)
    cells = np.zeros((n, n), np.int8)
    cells[50:150, 80:84] = 100

    svc.ingest("reference", meta, cells)
    svc.ingest("moving", meta, cells)

    assert svc.transforms["moving"] == (-9.0, 0.0, 0.0)
    status = svc.status()["registrations"]["moving"]
    assert status["confident"] is True
    assert status["accepted"] is False
    assert "outside configured prior" in status["rejection"]


def test_auto_mode_keeps_unregistered_maps_local(monkeypatch):
    """A spawn/config prior must not make two SLAM grids look registered."""
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    ambiguous = Registration(dx=0, dy=0, dyaw=0, score=0.1, overlap=20, ratio=0.95)
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: ambiguous)

    svc = MapService(resolution=0.1, size_m=20.0)
    svc.set_mode("auto")
    svc.set_transform("r0", 0, 0, 0)
    svc.set_transform("r1", 0, 0, 0)
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10, -10)
    ref = np.full((n, n), -1, np.int8)
    ref[20:30, 20:30] = 100
    moving = np.full((n, n), -1, np.int8)
    moving[150:160, 150:160] = 100

    svc.ingest("r0", meta, ref)
    svc.ingest("r1", meta, moving)

    status = svc.status()
    assert status["global_members"] == []
    assert status["view_by_robot"] == {"r0": "local", "r1": "local"}
    assert np.all(svc.merged == -1), "there is no global grid before a registration"


def test_auto_mode_exposes_global_map_after_accepted_match(monkeypatch):
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    accepted = Registration(dx=0, dy=0, dyaw=0, score=0.8, overlap=200, ratio=0.2)
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: accepted)

    svc = MapService(resolution=0.1, size_m=20.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10, -10)
    cells = np.zeros((n, n), np.int8)
    cells[80:90, 80:90] = 100
    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells)

    status = svc.status()
    assert status["global_members"] == ["r0", "r1"]
    assert status["view_by_robot"] == {"r0": "global", "r1": "global"}
