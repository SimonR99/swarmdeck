"""Map registration tests — synthetic, deterministic, no ROS and no simulator."""

from __future__ import annotations

import math

import numpy as np
import pytest

from swarmdeck_server.mapsvc.registration import register
from swarmdeck_server.mapsvc.service import GridMeta, MapService

RES, N, OX, OY = 0.05, 400, -10.0, -10.0


def build_plan() -> np.ndarray:
    """An asymmetric floor plan: outer shell plus distinctive interior walls."""
    g = np.full((N, N), -1, np.int8)

    def rect(x0, y0, x1, y1, v=100):
        i0, i1 = int((x0 - OX) / RES), int((x1 - OX) / RES)
        j0, j1 = int((y0 - OY) / RES), int((y1 - OY) / RES)
        g[j0:j1, i0:i1] = v

    rect(-8, -8, 8, -7.8)
    rect(-8, 7.8, 8, 8)
    rect(-8, -8, -7.8, 8)
    rect(7.8, -8, 8, 8)
    rect(-2, -8, -1.8, 2)      # asymmetric interior
    rect(3, -1, 3.2, 8)
    rect(-8, 3, -2, 3.2)
    rect(4.5, -5, 7.8, -4.8)

    free = np.zeros_like(g, bool)
    i0, i1 = int((-7.8 - OX) / RES), int((7.8 - OX) / RES)
    j0, j1 = int((-7.8 - OY) / RES), int((7.8 - OY) / RES)
    free[j0:j1, i0:i1] = True
    g[free & (g == -1)] = 0
    return g


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
