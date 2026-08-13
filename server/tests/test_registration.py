"""Map registration tests — synthetic, deterministic, no ROS and no simulator."""

from __future__ import annotations

import math

import numpy as np
import pytest

from swarmdeck_server.mapsvc.registration import (
    COARSE_STEP,
    _coarse_candidates,
    register,
)
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


def test_the_answer_does_not_depend_on_the_prior_it_was_given():
    """Narrowing the search must not be able to move the answer.

    The prior exists to make the sweep cheaper and to keep a symmetric alias out
    of the candidate set. It is not evidence, so two windows that both contain
    the true peak have to agree — and when the prior anchored the sweep they did
    not, because every stage refines within +/- the previous step and so
    inherited the prior's sub-step offset.

    That is the measured cause of the merged map flickering: on the live fleet
    the wide search returned -0.375 deg at a rival-translation ratio of 0.679 and
    the narrow search its own answer enabled returned +0.620 deg at 0.895 — one
    either side of the 0.80 ambiguity threshold, on bit-identical grids — so the
    fleet's registration alternated between accepted and refused at the map
    upload rate and the GUI swapped between the merged map and the robot's own
    map with it.
    """
    ref = build_plan()
    mov = reframe(ref, 1.1, -0.6, math.radians(2.0))

    # Deliberately including priors that are not multiples of the coarse step:
    # the lock is a previous answer, refined to a fraction of a degree, so a
    # sweep anchored to it lands nowhere near the sweep that produced it.
    distinct = {
        (round(r.dyaw, 9), round(r.dx, 9), round(r.dy, 9), round(r.ratio, 9), r.confident)
        for r in (
            register(
                ref, (RES, OX, OY), mov, (RES, OX, OY),
                yaw_prior=math.radians(prior_deg), yaw_window_deg=8.0,
            )
            for prior_deg in (0.0, -0.38, 0.62, 1.7, -2.4, 3.9)
        )
    }
    assert len(distinct) == 1, f"the prior moved the answer: {sorted(distinct)}"


@pytest.mark.parametrize("prior_deg", [0.0, -0.38, 12.5, -47.3, 179.6])
@pytest.mark.parametrize("window_deg", [8.0, 40.0])
def test_yaw_sweep_stays_on_one_lattice(prior_deg, window_deg):
    """Every window is a subset of the same global lattice of candidates.

    This is what makes the result above prior-independent: the winner of a wide
    sweep is still the winner of a narrower one whenever it lies inside it, so
    narrowing can cost coverage but cannot select a different peak.
    """
    candidates = np.degrees(_coarse_candidates(math.radians(prior_deg), window_deg))
    assert candidates.size, "a window must always offer at least one candidate"

    off_lattice = candidates - np.round(candidates / COARSE_STEP) * COARSE_STEP
    assert np.allclose(off_lattice, 0.0, atol=1e-9), f"off-lattice yaws: {candidates}"
    assert np.all(np.abs(candidates - prior_deg) <= window_deg + 1e-9)
    # Nothing inside the window is skipped, so the lattice never costs coverage
    # beyond the half step the coarse stage is dilated to absorb.
    assert candidates.max() > prior_deg + window_deg - COARSE_STEP
    assert candidates.min() < prior_deg - window_deg + COARSE_STEP


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


def test_a_marginal_frame_does_not_evict_a_registered_robot(monkeypatch):
    """The merged map must not blink out because one upload matched ambiguously.

    Reproduces the live oscillation: on the running fleet the wide search always
    accepted robot_1 (ratio 0.679) and the narrow search the resulting lock
    enabled always refused it (ratio 0.895) on bit-identical grids, so
    membership alternated between 2/4 and 0/4 at the map upload rate.
    """
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    wide = Registration(dx=0, dy=0, dyaw=0, score=0.33, overlap=138, ratio=0.679,
                        yaw_ratio=0.073, support=0.402)
    narrow = Registration(dx=0, dy=0, dyaw=0, score=0.29, overlap=126, ratio=0.895,
                          yaw_ratio=0.0, support=0.402)
    monkeypatch.setattr(
        service_module,
        "register",
        lambda *args, **kwargs: narrow if kwargs.get("yaw_prior") is not None else wide,
    )

    svc = MapService(resolution=0.1, size_m=20.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10, -10)
    cells = np.zeros((n, n), np.int8)
    cells[80:90, 80:90] = 100
    svc.ingest("r0", meta, cells)

    seen = []
    for _ in range(8):
        svc.ingest("r1", meta, cells)
        seen.append(tuple(svc.status()["global_members"]))

    assert set(seen) == {("r0", "r1")}, f"membership oscillated: {seen}"
    # The lock still drops on an ambiguous frame — that escalation back to a wide
    # search is what lets the next upload confirm the robot instead of repeating
    # the same narrow failure.
    assert svc.status()["registrations"]["r1"]["misses"] in (0, 1)


def test_a_robot_that_keeps_matching_ambiguously_is_dropped(monkeypatch):
    """Hysteresis delays eviction; it must not prevent it."""
    from swarmdeck_server.mapsvc.registration import Registration
    import swarmdeck_server.mapsvc.service as service_module

    accepted = Registration(dx=0, dy=0, dyaw=0, score=0.8, overlap=200, ratio=0.2,
                            yaw_ratio=0.2, support=0.9)
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: accepted)

    svc = MapService(resolution=0.1, size_m=20.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -10, -10)
    cells = np.zeros((n, n), np.int8)
    cells[80:90, 80:90] = 100
    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells)
    assert svc.status()["global_members"] == ["r0", "r1"]

    ambiguous = Registration(dx=0, dy=0, dyaw=0, score=0.1, overlap=20, ratio=0.95,
                             yaw_ratio=0.95, support=0.2)
    monkeypatch.setattr(service_module, "register", lambda *args, **kwargs: ambiguous)

    for _ in range(service_module.REGISTRATION_MISS_LIMIT - 1):
        svc.ingest("r1", meta, cells)
        assert svc.status()["registrations"]["r1"]["accepted"] is True, "held, not dropped"

    svc.ingest("r1", meta, cells)
    status = svc.status()
    assert status["global_members"] == []
    assert status["registrations"]["r1"]["accepted"] is False
    assert status["registrations"]["r1"]["rejection"] == "ambiguous occupancy match"

    # `misses` is offered to the operator as evidence that the map is being held
    # together on an older transform. Once the robot is out nothing is being
    # held, so the count must stop rather than climb with uptime.
    for _ in range(5):
        svc.ingest("r1", meta, cells)
        assert svc.status()["registrations"]["r1"]["misses"] == 0


# ------------------------------------------------- deferred registration


def test_an_upload_does_not_wait_for_registration(monkeypatch):
    """The four-robot map starvation: registration must not gate ingest.

    Registration is ~2.4 s of FFT on live 800x800 grids. Running it inside the
    upload that triggered it put that on the critical path of every scan, and on
    the four-robot fleet total demand reached ~160% of one serialised core. A
    queue whose arrival rate exceeds its service rate diverges, so every scan
    upload hit its client timeout and was DISCARDED — and because the hardware
    robots all run `topics.map: ""`, raytraced scans are the only source their
    map has. The maps starved. Raising the client timeout cannot fix that; the
    work has to leave the upload path.
    """
    import asyncio
    import time

    import swarmdeck_server.mapsvc.service as service_module
    from swarmdeck_server.mapsvc.registration import Registration

    accepted = Registration(dx=0, dy=0, dyaw=0, score=0.8, overlap=200,
                            ratio=0.2, yaw_ratio=0.2, support=0.9)

    def slow_register(*_args, **_kwargs):
        time.sleep(0.5)  # stands in for the real FFT
        return accepted

    monkeypatch.setattr(service_module, "register", slow_register)

    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.zeros((n, n), np.int8)
    cells[10:90, 15:18] = 100

    async def scenario():
        worker = asyncio.ensure_future(svc.registration_worker())
        await svc.ingest_async("r0", meta, cells)  # reference: registers trivially
        started = time.perf_counter()
        await svc.ingest_async("r1", meta, cells.copy())
        upload_s = time.perf_counter() - started
        # The transform must still arrive — just not on the uploader's clock.
        for _ in range(100):
            if "r1" in svc.registered:
                break
            await asyncio.sleep(0.05)
        worker.cancel()
        try:
            await worker
        except asyncio.CancelledError:
            pass
        return upload_s

    upload_s = asyncio.run(scenario())

    assert upload_s < 0.3, (
        f"ingest_async waited {upload_s:.2f}s — it is blocking on registration "
        "again, which is what starved the maps"
    )
    assert "r1" in svc.registered, (
        "the worker never performed the deferred registration; deferring it "
        "must not mean dropping it"
    )


def test_a_burst_of_uploads_collapses_to_one_registration(monkeypatch):
    """The queue is a set: it is a current estimate, not a backlog.

    Ten uploads arriving while the worker is busy must leave ONE recomputation
    against the newest grid, or the worker inherits exactly the divergence that
    moving the work off the upload path was meant to escape.
    """
    import asyncio

    import swarmdeck_server.mapsvc.service as service_module
    from swarmdeck_server.mapsvc.registration import Registration

    calls = []

    def counting_register(*_args, **_kwargs):
        calls.append(1)
        return Registration(dx=0, dy=0, dyaw=0, score=0.8, overlap=200,
                            ratio=0.2, yaw_ratio=0.2, support=0.9)

    monkeypatch.setattr(service_module, "register", counting_register)

    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.zeros((n, n), np.int8)
    cells[10:90, 15:18] = 100

    async def scenario():
        await svc.ingest_async("r0", meta, cells)
        for _ in range(10):
            await svc.ingest_async("r1", meta, cells.copy())
        # Nothing has run yet: no worker is draining the queue.
        assert not calls, "registration ran inside the upload path"
        assert svc._registration_due == {"r0", "r1"}

        worker = asyncio.ensure_future(svc.registration_worker())
        for _ in range(100):
            if "r1" in svc.registered:
                break
            await asyncio.sleep(0.02)
        worker.cancel()
        try:
            await worker
        except asyncio.CancelledError:
            pass

    asyncio.run(scenario())

    assert len(calls) == 1, (
        f"ten uploads produced {len(calls)} registrations; the set must coalesce "
        "them into one recomputation on the newest grid"
    )


# ------------------------------------------- transform smoothing / z alignment


def test_an_accepted_transform_is_eased_in_once_registered(monkeypatch):
    """A parked fleet's map must not wobble.

    Each registration is an independent estimate, so correlation noise used to
    land in the merged map whole. Measured on the live fleet with every robot
    stationary, accepted transforms moved up to 0.42 m and 5.8 deg between
    consecutive one-second samples, with ZERO accept/reject flips — jitter in a
    result the merge already trusted, not disagreement about trusting it.
    """
    import swarmdeck_server.mapsvc.service as service_module
    from swarmdeck_server.mapsvc.registration import Registration
    from swarmdeck_server.mapsvc.service import TRANSFORM_SMOOTHING

    moved = {"dx": 0.0}

    def shifting_register(*_args, **_kwargs):
        return Registration(dx=moved["dx"], dy=0.0, dyaw=0.0, score=0.8,
                            overlap=200, ratio=0.2, yaw_ratio=0.2, support=0.9)

    monkeypatch.setattr(service_module, "register", shifting_register)

    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("auto")
    n = svc.meta.width
    meta = GridMeta(0.1, n, n, -5.0, -5.0)
    cells = np.zeros((n, n), np.int8)
    cells[10:90, 15:18] = 100

    svc.ingest("r0", meta, cells)
    svc.ingest("r1", meta, cells.copy())
    # Acquisition takes the transform WHOLE: nothing should slow the first lock.
    assert svc.transforms["r1"][0] == pytest.approx(0.0, abs=1e-6)
    assert "r1" in svc.registered

    # A one-off 1.0 m excursion now arrives damped, not in full.
    moved["dx"] = 1.0
    svc.ingest("r1", meta, cells.copy())
    assert svc.transforms["r1"][0] == pytest.approx(TRANSFORM_SMOOTHING, abs=1e-6)

    # ...but a PERSISTENT correction still converges, it is not rejected.
    for _ in range(30):
        svc.ingest("r1", meta, cells.copy())
    assert svc.transforms["r1"][0] == pytest.approx(1.0, abs=0.01)


def test_z_offset_recovers_a_known_vertical_shift():
    """The SE(2) merge produces no dz, so the 3D view needs one estimated."""
    from swarmdeck_server.mapsvc.service import estimate_z_offset

    rng = np.random.default_rng(0)
    # A floor band, a furniture band and a ceiling band — a real height signature.
    ref = np.concatenate([
        rng.normal(-0.6, 0.02, 4000),
        rng.normal(0.5, 0.15, 3000),
        rng.normal(2.0, 0.03, 3000),
    ]).astype(np.float32)

    for truth in (-0.25, 0.0, 0.4):
        assert estimate_z_offset(ref, ref - truth) == pytest.approx(
            truth, abs=0.06
        ), f"failed to recover a {truth} m shift"


def test_z_offset_declines_to_guess_from_too_few_points():
    """A robot that has barely uploaded must not be shoved vertically."""
    from swarmdeck_server.mapsvc.service import estimate_z_offset

    rng = np.random.default_rng(1)
    big = rng.normal(0.0, 0.5, 5000).astype(np.float32)
    assert estimate_z_offset(big, rng.normal(3.0, 0.5, 5).astype(np.float32)) == 0.0
    assert estimate_z_offset(np.zeros(0, np.float32), big) == 0.0
