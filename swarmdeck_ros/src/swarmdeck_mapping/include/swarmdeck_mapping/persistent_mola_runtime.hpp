#pragma once

#include <swarmdeck_mapping/mola_submap_bridge.hpp>
#include <swarmdeck_mapping/planner_map.hpp>
#include <swarmdeck_mapping/snapshot_io.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace swarmdeck_mapping
{
enum class ApplyMode
{
  Auto,
  Replace,
  PoseOnly
};

enum class RuntimeErrorCode
{
  InvalidRequest,
  CacheMiss,
  Stale,
  Conflict,
  ResourceLimit,
  Io,
  Mola,
  Internal
};

class RuntimeError : public std::runtime_error
{
 public:
  RuntimeError(RuntimeErrorCode code, const std::string& message)
      : std::runtime_error(message), code_(code)
  {
  }
  [[nodiscard]] RuntimeErrorCode code() const noexcept { return code_; }

 private:
  RuntimeErrorCode code_;
};

struct RuntimeLimits
{
  std::size_t max_line_bytes{64 * 1024};
  std::size_t max_snapshot_bytes{kMaxSnapshotBytes};
  std::size_t max_response_bytes{64 * 1024};
  std::size_t max_maps{256};
  std::size_t max_submaps_per_map{kMaxSubmapsPerMap};
  std::size_t max_chunks_per_map{kMaxChunksPerMap};
  std::size_t max_points_per_map{2'000'000};
  std::size_t max_resident_points{8'000'000};
  std::size_t max_output_bytes{1024ULL * 1024ULL * 1024ULL};
  std::size_t max_planner_output_bytes{kMaxPlannerArtifactBytes};
};

struct ApplyRequest
{
  std::string request_id;
  std::string map_id;
  ApplyMode mode{ApplyMode::Auto};
  std::filesystem::path snapshot_path;
  std::string snapshot_sha256;
  std::filesystem::path chunks_dir;
  // Empty for embedded in-memory users. The JSONL driver requires an export.
  std::filesystem::path output_path;
  // Empty preserves the one-component projection contract. Nonempty selects
  // one component from a bounded whole-peer snapshot.
  std::string component_id;
  // Empty preserves existing runtime cost. When set, a coherent SDMGRID1
  // product must be installed before the native geometry can commit.
  std::filesystem::path planner_output_path;
};

struct ApplyReport
{
  std::string request_id;
  std::string map_id;
  ApplyMode mode{ApplyMode::Auto};
  std::string result;
  NativeGeometrySnapshot snapshot;
  std::size_t submap_count{};
  std::size_t point_count{};
  std::size_t output_size_bytes{};
  std::string output_sha256;
  std::size_t planner_output_size_bytes{};
  std::string planner_output_sha256;
};

/**
 * Bounded owner of persistent per-context MOLA point-geometry maps.
 *
 * The class is independent of JSONL and can be embedded in a MOLA
 * ExecutableBase/MapSource module. provider() exposes the real MapSourceBase
 * implementation for subscriptions, while currentSnapshot() supports late
 * subscribers and exact source/version checks.
 */
class PersistentMolaRuntime
{
 public:
  explicit PersistentMolaRuntime(RuntimeLimits limits = {});

  ApplyReport apply(const ApplyRequest& request);
  bool release(const std::string& map_id);

  [[nodiscard]] std::shared_ptr<MolaSubmapBridge> provider(
      const std::string& map_id) const;
  [[nodiscard]] std::optional<NativeGeometrySnapshot> currentSnapshot(
      const std::string& map_id) const;
  [[nodiscard]] std::vector<std::string> mapIds() const;
  [[nodiscard]] RuntimeLimits limits() const noexcept { return limits_; }

 private:
  struct Context
  {
    std::shared_ptr<MolaSubmapBridge> provider;
    std::size_t point_count{};
  };

  RuntimeLimits limits_;
  // Applies hold this through publication to preserve update order. Release
  // deliberately does not, so a callback may invalidate its own context.
  mutable std::mutex publication_mutex_;
  // Mutations are serialized without blocking read-only registry inspection
  // from MapSource callbacks.
  mutable std::mutex operation_mutex_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, Context> contexts_;
  std::size_t resident_points_{};
};

std::string applyModeName(ApplyMode mode);
std::string runtimeErrorCodeName(RuntimeErrorCode code);

}  // namespace swarmdeck_mapping
