"""Run with the patched upstream source on PYTHONPATH (no ROS required)."""

import numpy as np
from scipy import spatial

from cslam.lidar_pr.scancontext_matching import ScanContextMatching
import cslam.lidar_pr.scancontext_utils as sc_utils


def full_rebuild(matcher, query):
    query_sc = query.reshape(matcher.shape)
    count = min(matcher.num_candidates, matcher.nb_items)
    _, candidates = spatial.KDTree(matcher.ringkeys[: matcher.nb_items]).query(
        sc_utils.sc2rk(query_sc), k=count
    )
    best, score = 0, 0.0
    for index in np.atleast_1d(candidates):
        distance, _ = sc_utils.distance_sc(matcher.scancontexts[index], query_sc)
        if 1 - distance > score:
            best, score = index, 1 - distance
    return [matcher.items[best]], [score]


def test_scancontext_matches_full_rebuild_across_index_batches():
    rng = np.random.default_rng(981)
    matcher = ScanContextMatching(shape=[4, 8])
    assert matcher.search_best(np.zeros(32)) == (None, None)
    for i in range(140):
        descriptor = rng.random(32)
        matcher.add_item(descriptor, i)
        if i in (0, 1, 9, 62, 63, 64, 126, 127, 128, 139):
            for query in (descriptor, rng.random(32)):
                assert matcher.search(query, 1) == full_rebuild(matcher, query)


def test_scancontext_reuses_tree_and_searches_unindexed_tail(monkeypatch):
    matcher = ScanContextMatching(shape=[4, 8])
    rng = np.random.default_rng(412)
    for i in range(100):
        matcher.add_item(rng.random(32), i)
    query = rng.random(32)
    matcher.search(query, 1)
    original = spatial.KDTree
    builds = []

    def counted(*args, **kwargs):
        builds.append(len(args[0]))
        return original(*args, **kwargs)

    monkeypatch.setattr(spatial, "KDTree", counted)
    for i in range(10):
        descriptor = rng.random(32)
        matcher.add_item(descriptor, 100 + i)
        assert matcher.search_best(descriptor)[0] == 100 + i
        matcher.search(query, 1)
    assert builds == []


def test_scancontext_duplicate_ring_keys_preserve_full_rebuild_ties():
    matcher = ScanContextMatching(num_candidates=3)
    rng = np.random.default_rng(49)
    for i in range(12):
        # Different contexts with identical ring keys.
        descriptor = np.tile(rng.permutation(60), (20, 1)).astype(float).ravel()
        matcher.add_item(descriptor, i)
        if i >= 3:
            assert matcher.search(descriptor, 1) == full_rebuild(matcher, descriptor)


def test_ringkey_candidates_match_full_tree_after_resize():
    matcher = ScanContextMatching()
    rng = np.random.default_rng(948)
    for i in range(1100):
        matcher.add_item(rng.random(1200), i)
        if i in (9, 11, 63, 74, 999, 1000, 1099):
            for _ in range(5):
                query = rng.random(20)
                expected = spatial.KDTree(matcher.ringkeys[: i + 1]).query(query, k=10)[
                    1
                ]
                np.testing.assert_array_equal(
                    matcher._candidate_indices(query), expected
                )


def test_duplicate_keys_across_cached_tree_and_tail_keep_full_tree_tie_order():
    rng = np.random.default_rng(388)
    for _ in range(100):
        matcher = ScanContextMatching(num_candidates=1)
        for index in range(10):
            matcher.add_item(rng.random(1200), index)
        query = rng.random(20)
        nearest = matcher._candidate_indices(query)[0]
        matcher.add_item(matcher.scancontexts[nearest].ravel(), 10)
        expected = spatial.KDTree(matcher.ringkeys[:11]).query(query, k=1)[1]
        assert matcher._candidate_indices(query)[0] == expected
