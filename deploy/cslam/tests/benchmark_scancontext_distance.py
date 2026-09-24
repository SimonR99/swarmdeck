"""Per-query CPU cost with the cached index, before/after vectorized scoring."""

from statistics import median
from time import process_time

import numpy as np

import cslam.lidar_pr.scancontext_utils as sc_utils
from cslam.lidar_pr.scancontext_matching import ScanContextMatching
from test_scancontext_distance import reference_distance_sc


def query_cpu_ms(matcher, query, score, repeats):
    sc_utils.distance_sc = score
    samples = []
    for _ in range(repeats):
        start = process_time()
        matcher.search(query, 1)
        samples.append((process_time() - start) * 1000)
    return median(samples)


vectorized = sc_utils.distance_sc
rng = np.random.default_rng(91)
try:
    for count in (1000, 5000, 20000):
        matcher = ScanContextMatching()
        for index in range(count):
            matcher.add_item(rng.random(1200), index)
        query = rng.random(1200)
        matcher.search(query, 1)  # Warm the cached ring-key index.
        old = query_cpu_ms(matcher, query, reference_distance_sc, 3)
        new = query_cpu_ms(matcher, query, vectorized, 30)
        print(
            f"n={count}: query CPU old={old:.3f}ms vectorized={new:.3f}ms", flush=True
        )
finally:
    sc_utils.distance_sc = vectorized
