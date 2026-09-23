"""Reachable floor and target scattering on a small synthetic tunnel."""

import math
from pathlib import Path
import random
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenario"))
from make_argos_world import GLTFBuilder, Material
from mesh_surface import MeshSurface
import walkable

ROCK = Material("rock", (0.4, 0.4, 0.4))


def floor_quad(builder, x0, y0, x1, y1, z=0.0):
    builder.add_quad(ROCK, (x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z))


@pytest.fixture(scope="module")
def tunnel(tmp_path_factory):
    """An L of floor: 20 m east along y in [0, 2], then 20 m north along
    x in [18, 20]. Beside it, unreachable: a disjoint slab, a ledge 1 m up,
    and a crawlspace whose roof leaves 0.5 m of headroom."""
    path = tmp_path_factory.mktemp("tunnel") / "tunnel.glb"
    builder = GLTFBuilder()
    floor_quad(builder, 0, 0, 20, 2)
    floor_quad(builder, 18, 2, 20, 22)
    # Roof over the whole L, 3 m up.
    floor_quad(builder, 0, 0, 20, 2, z=3.0)
    floor_quad(builder, 18, 2, 20, 22, z=3.0)
    # Disjoint: 1 m of gap from the L.
    floor_quad(builder, 0, 3, 10, 5)
    # A ledge 1 m above the corridor, touching it.
    floor_quad(builder, 10, 2, 16, 4, z=1.0)
    # A crawlspace off the far end: floor continues, roof at 0.5 m.
    floor_quad(builder, 18, 22, 20, 26)
    floor_quad(builder, 18, 22, 20, 26, z=0.5)
    builder.export_glb(path)
    return walkable.reachable_floor(MeshSurface(path).triangles, (1.0, 1.0, 0.2))


def reached(floor, x, y):
    return bool(np.any((np.abs(floor.x - x) < 0.3) & (np.abs(floor.y - y) < 0.3)))


def test_floor_follows_the_tunnel_and_nothing_else(tunnel):
    assert reached(tunnel, 1.0, 1.0)
    assert reached(tunnel, 19.0, 1.0)
    assert reached(tunnel, 19.0, 21.0)
    assert not reached(tunnel, 5.0, 4.0), "disjoint slab"
    assert not reached(tunnel, 13.0, 3.5), "ledge above a step"
    assert not reached(tunnel, 19.0, 24.0), "crawlspace without headroom"
    assert np.all(np.abs(tunnel.z) < 1e-6)


def test_distance_runs_along_the_floor_not_through_the_rock(tunnel):
    end = np.argmin(np.hypot(tunnel.x - 19.0, tunnel.y - 21.0))
    straight = math.hypot(19.0 - 1.0, 21.0 - 1.0)
    along = (19.0 - 1.0) + (21.0 - 1.0)
    assert tunnel.distance[end] > straight + 5.0
    assert tunnel.distance[end] == pytest.approx(along, abs=2.0)


def test_scatter_puts_one_at_the_far_end_and_spaces_the_rest(tunnel):
    picks = walkable.scatter(tunnel, 4, random.Random(3), min_distance=8.0, spacing=6.0)
    assert len(picks) == 4
    assert tunnel.distance[picks[0]] >= 0.9 * tunnel.distance.max() - 1.0
    assert all(tunnel.distance[p] >= 8.0 for p in picks)
    for i, a in enumerate(picks):
        assert tunnel.clear[a] and tunnel.by_wall[a]
        for b in picks[i + 1 :]:
            assert (
                math.hypot(tunnel.x[a] - tunnel.x[b], tunnel.y[a] - tunnel.y[b]) >= 6.0
            )
    # Seeded: the same seed gives the same layout.
    assert picks == walkable.scatter(
        tunnel, 4, random.Random(3), min_distance=8.0, spacing=6.0
    )


def test_start_off_the_floor_is_an_error(tmp_path):
    builder = GLTFBuilder()
    floor_quad(builder, 0, 0, 4, 4)
    builder.export_glb(tmp_path / "slab.glb")
    triangles = MeshSurface(tmp_path / "slab.glb").triangles
    with pytest.raises(ValueError, match="no floor under the start"):
        walkable.reachable_floor(triangles, (50.0, 50.0, 0.2))
