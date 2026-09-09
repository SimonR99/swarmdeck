#pragma once

#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace swarmdeck_mapping
{
struct PointXYZ
{
  float x{};
  float y{};
  float z{};
};

using Matrix4 = std::array<double, 16>;

struct SubmapInput
{
  std::string external_id;
  std::vector<PointXYZ> points_local;
  Matrix4 T_component_submap;
};

struct PoseUpdate
{
  std::string external_id;
  Matrix4 T_component_submap;
};

struct SolutionVersion
{
  std::string component_id;
  std::uint64_t epoch{};
  std::uint64_t revision{};
  // SHA-256 of the canonical GraphSolution. This makes an exact replay
  // idempotent and detects conflicting content at one revision.
  std::string digest;
};

/**
 * Adapter for a Swarm-SLAM-authoritative map.
 *
 * Geometry remains in each submap's local frame inside
 * mola::KeyframePointCloudMap. Swarm-SLAM corrections call setKeyframePose();
 * they never enter MOLA's optimizer. Geometry replacement/retraction supplies a
 * complete coherent snapshot and builds a fresh MOLA map, preventing old points
 * from remaining as additive occupancy.
 *
 * MapSourceBase makes the resulting CMetricMap directly consumable by MOLA's
 * visualization and ROS 2 bridge modules.
 */
class MolaSubmapBridge final : public mola::MapSourceBase
{
 public:
  enum class ApplyResult
  {
    Applied,
    Duplicate
  };

  MolaSubmapBridge();

  void replaceGeometrySnapshot(
      const std::vector<SubmapInput>& submaps, const SolutionVersion& version,
      const std::string& canonical_metadata_json);

  ApplyResult applyPoseSolution(
      const std::vector<PoseUpdate>& updates, const SolutionVersion& version,
      const std::string& canonical_metadata_json);

  [[nodiscard]] std::shared_ptr<const mola::KeyframePointCloudMap> currentMap() const;

 private:
  static void validateVersion(const SolutionVersion& version);
  static mrpt::poses::CPose3D checkedPose(const Matrix4& matrix);
  static std::string frameName(const std::string& component_id);
  void publish(
      const std::shared_ptr<mola::KeyframePointCloudMap>& map,
      const SolutionVersion& version, const std::string& metadata_json);

  mutable std::mutex mutex_;
  std::shared_ptr<mola::KeyframePointCloudMap> map_;
  std::unordered_map<std::string, mola::KeyframePointCloudMap::KeyFrameID> ids_;
  std::optional<SolutionVersion> version_;
};
}  // namespace swarmdeck_mapping
