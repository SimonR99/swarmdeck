#pragma once

#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <array>
#include <cstdint>
#include <functional>
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
  std::vector<PointXYZ> sensor_origins_local;
  std::uint64_t observed_at_ns{};
  bool ray_evidence_qualified{};
  std::uint64_t geometry_revision{};
  std::vector<std::string> chunk_fingerprints;
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

/** Exact source and geometry identities carried with an immutable map view. */
struct SnapshotIdentity
{
  std::string geometry_revision;
  std::string native_geometry_digest;
  std::string canonical_manifest_digest;
  std::string source_snapshot_id;
  std::string source_sha256;
  std::string reference_frame;
};

/** Immutable handles to the exact point buffers inserted into one MOLA KF. */
struct NativeKeyframeSnapshot
{
  std::string external_id;
  mola::KeyframePointCloudMap::KeyFrameID keyframe_id{};
  std::shared_ptr<const mrpt::maps::CSimplePointsMap> points_local;
  std::uint64_t observed_at_ns{};
  std::vector<PointXYZ> sensor_origins_local;
  // Source geometry identity for append-only native reuse.
  std::uint64_t geometry_revision{};
  // True only when the source explicitly proves one origin applies to all
  // first returns and those returns are deskewed. Legacy metadata is false.
  bool ray_evidence_qualified{};
  std::vector<std::string> chunk_fingerprints;
};
/**
 * One atomically captured native point-geometry publication.
 *
 * This is intentionally not an occupancy/free-space representation. A future
 * planner adapter may consume the MOLA layers without guessing unknown-space
 * semantics which the point cloud does not contain.
 */
struct NativeGeometrySnapshot
{
  std::shared_ptr<const mola::KeyframePointCloudMap> geometry_map;
  SolutionVersion graph_version;
  SnapshotIdentity identity;
  std::string canonical_metadata_json;
  std::vector<PoseUpdate> submap_poses;
  std::vector<NativeKeyframeSnapshot> keyframes;
  // Nonzero when geometry was bounded by deterministic voxel compaction.
  // This is native provenance, not part of the wire snapshot identity.
  double compaction_resolution_m{};
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
  using BeforeCommit = std::function<void(const NativeGeometrySnapshot&)>;

  enum class ApplyResult
  {
    Applied,
    Duplicate
  };

  MolaSubmapBridge();

  ApplyResult replaceGeometrySnapshot(
      const std::vector<SubmapInput>& submaps, const SolutionVersion& version,
      const std::string& canonical_metadata_json,
      const SnapshotIdentity& identity = {}, const BeforeCommit& before_commit = {},
      bool publish_update = true);

  ApplyResult applyPoseSolution(
      const std::vector<PoseUpdate>& updates, const SolutionVersion& version,
      const std::string& canonical_metadata_json,
      const SnapshotIdentity& identity = {}, const BeforeCommit& before_commit = {},
      bool require_complete_membership = false, bool publish_update = true);
  void setCompactionResolution(double resolution_m);

  [[nodiscard]] std::shared_ptr<const mola::KeyframePointCloudMap> currentMap() const;
  [[nodiscard]] std::optional<NativeGeometrySnapshot> currentSnapshot() const;
  void publishSnapshot(const NativeGeometrySnapshot& snapshot);

 private:
  static void validateVersion(const SolutionVersion& version);
  static mrpt::poses::CPose3D checkedPose(const Matrix4& matrix);
  static std::string frameName(const std::string& component_id);
  void publish(
      const std::shared_ptr<mola::KeyframePointCloudMap>& map,
      const SolutionVersion& version, const std::string& metadata_json,
      const std::string& reference_frame);

  mutable std::mutex mutex_;
  std::shared_ptr<mola::KeyframePointCloudMap> map_;
  std::unordered_map<std::string, mola::KeyframePointCloudMap::KeyFrameID> ids_;
  double compaction_resolution_m_{};
  std::optional<SolutionVersion> version_;
  SnapshotIdentity identity_;
  std::string metadata_json_;
  std::unordered_map<std::string, Matrix4> poses_;
  std::unordered_map<std::string, NativeKeyframeSnapshot> keyframes_;
};
}  // namespace swarmdeck_mapping
