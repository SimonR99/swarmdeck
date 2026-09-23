#pragma once

#include <cstddef>

namespace swarmdeck_mapping
{
/**
 * Bounded materialized point budget of a component map.
 *
 * Snapshot manifests may contain more historical raw returns than this
 * number. Native ingestion streams immutable chunks and deterministically
 * voxel-compacts overlapping component-frame returns before inserting MOLA
 * keyframes. The budget applies to the resident compact metric map and
 * planner product; metadata/chunk declarations retain their own finite
 * limits.
 */
inline constexpr std::size_t kMaxPointsPerMap = 2'000'000;
}  // namespace swarmdeck_mapping
