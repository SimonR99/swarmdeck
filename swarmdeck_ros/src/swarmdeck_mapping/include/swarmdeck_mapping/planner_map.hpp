#pragma once

#include <swarmdeck_mapping/mola_submap_bridge.hpp>
#include <swarmdeck_mapping/point_budget.hpp>

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
  // The component point budget. `PersistentMolaRuntime` sets it from
  // `RuntimeLimits::max_points_per_map`, the same number its loader enforces,
  // so a map the loader admits always gets a planner grid (see
  // point_budget.hpp). Measured in the grid build on a 20-core workstation
  // (RelWithDebInfo, 2026-09-19): 2.3 ms per 4096-point keyframe, 0.71 s at
  // 1,003,520 points, 1.50 s at 2,048,000 and 4.70 s at 8,192,000, the
  // last within `max_build_s` here but not on a slower host.
  std::size_t max_points{kMaxPointsPerMap};
  std::size_t max_voxels{2'000'000};
  // Maximum work admitted from a deterministic, conservative subset of rays.
  std::size_t max_ray_steps{4'000'000};
  double max_build_s{8.0};
  double ray_angular_resolution_rad{0.08726646259971647};  // 5 degrees
  double ray_step_fraction{0.75};
  // Qualified free rays, all observed strictly later than every endpoint in a
  // voxel, that must pass through it before its endpoints are retired. One
  // ray is counted once per voxel however many steps it spends there.
  std::size_t min_clearing_traversals{3};
  // A ray passes through a voxel's endpoints, rather than over them, only
  // where its sampled point lies no higher than the voxel's highest endpoint
  // plus this tolerance. A road surface is a sheet near the bottom of its
  // voxel, and rays from a lidar above it that end far ahead cross the road's
  // own ground voxels above that sheet; those rays prove nothing about the
  // road. 0.05 m covers the simulated 0.03 m range noise.
  double clearing_height_tolerance_m{0.05};
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
  std::size_t source_point_count{};
  std::size_t ray_steps{};
  std::size_t qualified_ray_keyframes{};
  // Endpoints withheld from `occupied` and `surfaces` because later qualified
  // rays saw through their voxels. `surfaces.size() + retired_count` always
  // equals `point_count`.
  std::size_t retired_count{};
  std::vector<PlannerVoxel> occupied;
  std::vector<PlannerVoxel> free;
  std::vector<PlannerSurfaceSample> surfaces;
};

/** Incremental evidence from original measured endpoints, independent of metric compaction. */
class NativePlannerAccumulator
{
 public:
  explicit NativePlannerAccumulator(const PlannerGridLimits& limits);
  NativePlannerAccumulator(const NativePlannerAccumulator& other);
  ~NativePlannerAccumulator();
  void endpoints(const SubmapInput& frame);
  void rays(const SubmapInput& frame, std::size_t work_budget);
  [[nodiscard]] std::uint64_t newestStamp() const;
  [[nodiscard]] std::size_t residentUnits() const;
  [[nodiscard]] std::shared_ptr<const NativePlannerGrid> snapshot(
      const SolutionVersion& version, const SnapshotIdentity& identity) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct PlannerGridArtifact
{
  std::size_t size_bytes{};
  std::string sha256;
};

std::shared_ptr<const NativePlannerGrid> buildNativePlannerGrid(
    const NativeGeometrySnapshot& snapshot, const PlannerGridLimits& limits = {});

/**
 * Atomically writes deterministic SDMGRID1 bytes and refuses replacement.
 *
 * `max_points` is the product's point budget; the runtime passes the same
 * `max_points_per_map` its loader and grid build use, so no product the build
 * accepted is refused here.
 */
PlannerGridArtifact writeNativePlannerGrid(
    const NativePlannerGrid& grid, const std::filesystem::path& output,
    std::size_t max_bytes = kMaxPlannerArtifactBytes,
    std::size_t max_metadata_bytes = kMaxPlannerMetadataBytes,
    std::size_t max_points = kMaxPointsPerMap);

}  // namespace swarmdeck_mapping
