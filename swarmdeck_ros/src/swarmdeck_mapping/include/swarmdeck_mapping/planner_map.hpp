#pragma once

#include <swarmdeck_mapping/mola_submap_bridge.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace swarmdeck_mapping
{
inline constexpr std::size_t kMaxPlannerMetadataBytes = 64 * 1024;
inline constexpr std::size_t kMaxPlannerArtifactBytes = 256 * 1024 * 1024;
inline constexpr std::size_t kMaxPlannerProductPoints = 2'000'000;
inline constexpr std::size_t kMaxPlannerProductVoxels = 2'000'000;
inline constexpr std::size_t kMaxPlannerProductRaySteps = 4'000'000;

struct PlannerVoxel
{
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t z{};

  friend bool operator==(const PlannerVoxel& lhs, const PlannerVoxel& rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }
  friend bool operator<(const PlannerVoxel& lhs, const PlannerVoxel& rhs)
  {
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
  }
};

struct PlannerSurfaceSample
{
  std::int64_t x{};
  std::int64_t y{};
  double z{};

  friend bool operator<(const PlannerSurfaceSample& lhs, const PlannerSurfaceSample& rhs)
  {
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
  }
};

struct PlannerGridLimits
{
  double resolution_m{0.2};
  std::size_t max_points{1'000'000};
  std::size_t max_voxels{2'000'000};
  // Maximum work admitted from a deterministic, conservative subset of rays.
  std::size_t max_ray_steps{4'000'000};
  double max_build_s{8.0};
  double ray_angular_resolution_rad{0.08726646259971647};  // 5 degrees
  double ray_step_fraction{0.75};
};

/** Immutable sparse map product. Unknown is represented by voxel absence. */
struct NativePlannerGrid
{
  SolutionVersion graph_version;
  SnapshotIdentity identity;
  std::uint64_t source_stamp_ns{};
  double resolution_m{};
  double ray_angular_resolution_rad{};
  double ray_step_fraction{};
  std::size_t point_count{};
  std::size_t ray_steps{};
  std::size_t qualified_ray_keyframes{};
  std::vector<PlannerVoxel> occupied;
  std::vector<PlannerVoxel> free;
  std::vector<PlannerSurfaceSample> surfaces;
};

struct PlannerGridArtifact
{
  std::size_t size_bytes{};
  std::string sha256;
};

std::shared_ptr<const NativePlannerGrid> buildNativePlannerGrid(
    const NativeGeometrySnapshot& snapshot, const PlannerGridLimits& limits = {});

/** Atomically writes deterministic SDMGRID1 bytes and refuses replacement. */
PlannerGridArtifact writeNativePlannerGrid(
    const NativePlannerGrid& grid, const std::filesystem::path& output,
    std::size_t max_bytes = kMaxPlannerArtifactBytes,
    std::size_t max_metadata_bytes = kMaxPlannerMetadataBytes);

}  // namespace swarmdeck_mapping
