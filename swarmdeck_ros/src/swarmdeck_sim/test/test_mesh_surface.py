"""Ground placement uses transformed triangles, not a flat z=0 assumption."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))
from make_argos_world import GLTFBuilder, Material
from mesh_surface import BistroSurface, GlbGeometry


def test_sloped_surface_places_the_whole_footprint_above_ground(tmp_path):
    road = GLTFBuilder()
    # z=.4+.1*x before Bistro's -.3 world translation.
    road.add_quad(
        Material("Pavement_test", (0.2, 0.2, 0.2)),
        (-2, -2, 0.2),
        (2, -2, 0.6),
        (2, 2, 0.6),
        (-2, 2, 0.2),
    )
    road.export_glb(tmp_path / "road.glb")
    prop = GLTFBuilder()
    # Offset model origin: bottom at .2, not zero.
    prop.add_box(Material("prop", (0.8, 0.2, 0.2)), (0, 0, 0.4), (1, 0.2, 0.4))
    prop.export_glb(tmp_path / "prop.glb")
    surface = BistroSurface(tmp_path / "road.glb")
    assert surface.height(0, 0) == pytest.approx(0.1)
    assert surface.place(tmp_path / "prop.glb", 0, 0, 0) == pytest.approx(
        0.15 - 0.2 + 0.005
    )
    assert surface.place(tmp_path / "prop.glb", 0, 0, math.pi / 2) == pytest.approx(
        0.11 - 0.2 + 0.005
    )
    with pytest.raises(ValueError, match="No Bistro pavement"):
        surface.height(3, 0)


def test_empty_pavement_is_not_silently_replaced_with_zero(tmp_path):
    mesh = GLTFBuilder()
    mesh.add_box(Material("table", (0.2, 0.2, 0.2)), (0, 0, 1), (2, 2, 0.1))
    mesh.export_glb(tmp_path / "table.glb")
    with pytest.raises(ValueError, match="no pavement"):
        BistroSurface(tmp_path / "table.glb")


def test_glb_export_axis_conversion_preserves_model_dimensions(tmp_path):
    mesh = GLTFBuilder()
    mesh.add_box(Material("prop", (0.2, 0.2, 0.2)), (1, 2, 3), (0.2, 0.4, 0.6))
    mesh.export_glb(tmp_path / "prop.glb")
    v = np.concatenate(list(GlbGeometry(tmp_path / "prop.glb").triangles())).reshape(
        -1, 3
    )
    np.testing.assert_allclose(v.min(0), [0.9, 1.8, 2.7], atol=1e-6)
    np.testing.assert_allclose(v.max(0), [1.1, 2.2, 3.3], atol=1e-6)
