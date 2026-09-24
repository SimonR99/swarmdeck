"""Run in the C-SLAM image with patched source on PYTHONPATH."""

from time import process_time

import numpy as np
from scipy import spatial

from cslam.lidar_pr.scancontext_matching import ScanContextMatching


def median_cpu_ms(function, repeats):
    samples = []
    for _ in range(repeats):
        start = process_time()
        function()
        samples.append((process_time() - start) * 1000)
    return round(float(np.median(samples)), 3)


class RebuiltIndexMatching(ScanContextMatching):
    def _candidate_indices(self, ringkey_query):
        return spatial.KDTree(np.array(self.ringkeys[: self.nb_items])).query(
            ringkey_query, k=self.num_candidates
        )[1]


rng = np.random.default_rng(19)
for count in (1000, 5000, 20000):
    matcher = ScanContextMatching()
    for index in range(count):
        matcher.add_item(rng.random(1200), index)
    query = rng.random(1200)
    ringkey = query.reshape(20, 60).mean(axis=1)
    matcher._candidate_indices(ringkey)
    baseline = RebuiltIndexMatching()
    baseline.__dict__.update(matcher.__dict__)
    old = lambda: spatial.KDTree(np.array(matcher.ringkeys[:count])).query(
        ringkey, k=10
    )
    new = lambda: matcher._candidate_indices(ringkey)
    # Fixed scoring work is unchanged. Report it separately from index costs.
    # Process CPU time excludes scheduler delays from concurrent lane builds.
    print(
        f"n={count} CPU: index old={median_cpu_ms(old, 30)}ms "
        f"cached={median_cpu_ms(new, 30)}ms "
        f"full_old_search={median_cpu_ms(lambda: baseline.search(query, 1), 3)}ms "
        f"full_cached_search={median_cpu_ms(lambda: matcher.search(query, 1), 3)}ms",
        flush=True,
    )
