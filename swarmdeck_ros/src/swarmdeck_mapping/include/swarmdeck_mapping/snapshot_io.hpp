#pragma once

#include <swarmdeck_mapping/mola_submap_bridge.hpp>
#include <swarmdeck_mapping/point_budget.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace swarmdeck_mapping
{
inline constexpr std::size_t kMaxSnapshotBytes = 64 * 1024 * 1024;
inline constexpr std::size_t kMaxSubmapsPerMap = 16384;
inline constexpr std::size_t kMaxChunksPerMap = 65536;
inline constexpr std::size_t kMaxSensorOriginsPerSubmap = 16;

struct ChunkDescriptor
{
  std::string sha256;
  std::size_t size_bytes{};
  std::size_t point_count{};
  std::string fingerprint() const
  {
    return sha256 + ":" + std::to_string(size_bytes) + ":" + std::to_string(point_count);
  }
};

struct ParsedSubmap
{
  std::string external_id;
  Matrix4 T_component_submap;
  std::vector<ChunkDescriptor> chunks;
  std::uint64_t geometry_revision{};
  std::vector<PointXYZ> sensor_origins_local;
  std::uint64_t observed_at_ns{};
  bool ray_evidence_qualified{};
};

/** A fully validated one-component projection of an autonomy snapshot. */
struct ParsedComponentSnapshot
{
  SolutionVersion graph_version;
  SnapshotIdentity identity;
  std::string canonical_metadata_json;
  std::vector<ParsedSubmap> submaps;
  std::size_t declared_point_count{};
};

ParsedComponentSnapshot parseComponentSnapshot(
    const std::filesystem::path& snapshot_path,
    const std::string& expected_source_sha256,
    std::size_t max_snapshot_bytes = kMaxSnapshotBytes,
    std::size_t max_submaps = kMaxSubmapsPerMap,
    std::size_t max_chunks = kMaxChunksPerMap,
    std::size_t max_points = kMaxPointsPerMap,
    const std::string& component_id = {});
/** Load checked chunk payloads and compact overlapping history to max_points. */
std::vector<SubmapInput> loadGeometry(
    const ParsedComponentSnapshot& snapshot,
    const std::filesystem::path& chunks_dir,
    std::size_t max_points = kMaxPointsPerMap,
    const NativeGeometrySnapshot* prior = nullptr);
SubmapInput loadRawSubmap(
    const ParsedSubmap& submap, const std::filesystem::path& chunks_dir,
    std::size_t max_points = kMaxPointsPerMap);
std::vector<PoseUpdate> poseUpdates(const ParsedComponentSnapshot& snapshot);

std::string boundedFileSha256(
    const std::filesystem::path& path,
    std::size_t maximum_bytes = kMaxSnapshotBytes);

}  // namespace swarmdeck_mapping
