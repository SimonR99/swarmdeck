#pragma once

#include <cstddef>

namespace swarmdeck_mapping
{
/**
 * The one point budget of a component map.
 *
 * It bounds the snapshot loader (`parseComponentSnapshot`, `loadGeometry`),
 * the planner grid build (`PlannerGridLimits::max_points`) and the SDMGRID1
 * product (`writeNativePlannerGrid`). `PersistentMolaRuntime` derives all
 * three from `RuntimeLimits::max_points_per_map`, which the worker sets with
 * `--max-points-per-map` (`deploy/autonomy/mola_worker.py`,
 * `DEFAULT_MAX_POINTS_PER_MAP`). Until 2026-09-19 the grid build carried its
 * own 1,000,000 default while the loader allowed 2,000,000, so a map between
 * the two loaded but never produced a planner grid (benchbot mission
 * 1a8cc114: robot_0 froze at 1,222,612 points).
 *
 * The product spends 24 bytes per point (one surface sample each), and its
 * readers cap the point count independently: `autonomy/mola_mapping.py`
 * (`MAX_POINTS`) and MGG's `MolaMap`, which bounds `surface_count` by
 * `map.mola.max_voxels` (clamped to 2,000,000 in `planner_node.cpp`). Raising
 * this value past a reader's cap makes that reader reject every product, so
 * the readers must move with it.
 */
inline constexpr std::size_t kMaxPointsPerMap = 2'000'000;
}  // namespace swarmdeck_mapping
