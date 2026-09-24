"""Pin vectorized ScanContext scores to the original sector/shift algorithm."""

import numpy as np
import pytest

from cslam.lidar_pr.scancontext_utils import distance_sc


def reference_distance_sc(sc1, sc2):
    sectors = sc1.shape[1]
    similarities = np.zeros(sectors)
    for shift in range(sectors):
        sc1 = np.roll(sc1, 1, axis=1)
        total, engaged = 0.0, 0
        for column in range(sectors):
            left, right = sc1[:, column], sc2[:, column]
            if not np.any(left) or not np.any(right):
                continue
            total += np.dot(left, right) / (
                np.linalg.norm(left) * np.linalg.norm(right)
            )
            engaged += 1
        similarities[shift] = total / engaged if engaged else 0.0
    return 1 - np.max(similarities), np.argmax(similarities) + 1


@pytest.mark.parametrize("seed", range(4))
@pytest.mark.parametrize("zero_columns", [False, True])
def test_scancontext_distance_matches_original(seed, zero_columns):
    rng = np.random.default_rng(seed)
    left, right = rng.random((2, 20, 60))
    if zero_columns:
        left[:, ::3] = 0
        right[:, ::4] = 0
    expected = reference_distance_sc(left, right)
    actual = distance_sc(left, right)
    assert actual[0] == pytest.approx(expected[0], abs=2e-15)
    assert actual[1] == expected[1]


@pytest.mark.parametrize("empty_side", [0, 1, 2])
def test_scancontext_distance_empty_columns_have_zero_similarity(empty_side):
    left, right = np.ones((2, 20, 60))
    if empty_side in (0, 2):
        left[:] = 0
    if empty_side in (1, 2):
        right[:] = 0
    assert distance_sc(left, right) == (1.0, 1)


def test_scancontext_distance_normalizes_each_descriptor_only_once(monkeypatch):
    rng = np.random.default_rng(401)
    left, right = rng.random((2, 20, 60))
    norm = np.linalg.norm
    calls = []

    def counted(*args, **kwargs):
        calls.append(args[0].shape)
        return norm(*args, **kwargs)

    monkeypatch.setattr(np.linalg, "norm", counted)
    distance_sc(left, right)
    assert calls == [(20, 60), (20, 60)]
