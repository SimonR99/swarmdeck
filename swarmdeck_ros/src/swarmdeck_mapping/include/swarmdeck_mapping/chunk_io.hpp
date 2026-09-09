#pragma once

#include <swarmdeck_mapping/mola_submap_bridge.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace swarmdeck_mapping
{
inline constexpr std::size_t kMaxChunkBytes = 8 * 1024 * 1024;

/** Read one checked XYZ-F32 chunk without permitting digest path traversal. */
std::vector<PointXYZ> readXyzChunk(
    const std::filesystem::path& chunks_dir, const std::string& sha256,
    std::size_t declared_size, std::size_t declared_point_count);
}  // namespace swarmdeck_mapping
