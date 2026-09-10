#include <swarmdeck_mapping/planner_map.hpp>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>

namespace swarmdeck_mapping
{
namespace
{
using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

constexpr std::array<char, 8> kMagic{'S', 'D', 'M', 'G', 'R', 'I', 'D', '1'};
std::atomic<std::uint64_t> temporary_sequence{};

struct VoxelHash
{
  std::size_t operator()(const PlannerVoxel& value) const noexcept
  {
    auto mix = [](std::uint64_t value) {
      value ^= value >> 30;
      value *= 0xbf58476d1ce4e5b9ULL;
      value ^= value >> 27;
      value *= 0x94d049bb133111ebULL;
      return value ^ (value >> 31);
    };
    return static_cast<std::size_t>(
        mix(static_cast<std::uint64_t>(value.x)) ^
        (mix(static_cast<std::uint64_t>(value.y)) << 1) ^
        (mix(static_cast<std::uint64_t>(value.z)) << 2));
  }
};

struct AngularBin
{
  std::int64_t azimuth{};
  std::int64_t elevation{};
  friend bool operator==(const AngularBin& lhs, const AngularBin& rhs)
  {
    return lhs.azimuth == rhs.azimuth && lhs.elevation == rhs.elevation;
  }
};

struct AngularHash
{
  std::size_t operator()(const AngularBin& value) const noexcept
  {
    return VoxelHash{}({value.azimuth, value.elevation, 0});
  }
};

struct Point3d
{
  double x{};
  double y{};
  double z{};
};

struct RayEndpoint
{
  double distance{};
  Point3d endpoint;
};

void validateLimits(const PlannerGridLimits& limits)
{
  if (!(std::isfinite(limits.resolution_m) && limits.resolution_m > 0) ||
      limits.max_points == 0 || limits.max_voxels == 0 ||
      limits.max_ray_steps == 0 ||
      !(std::isfinite(limits.max_build_s) && limits.max_build_s > 0) ||
      !(std::isfinite(limits.ray_angular_resolution_rad) &&
        limits.ray_angular_resolution_rad > 0) ||
      !(std::isfinite(limits.ray_step_fraction) &&
        limits.ray_step_fraction > 0 && limits.ray_step_fraction <= 1))
    throw std::invalid_argument("planner grid limits must be finite and positive");
}

void checkTime(const Clock::time_point started, const PlannerGridLimits& limits)
{
  if (std::chrono::duration<double>(Clock::now() - started).count() >
      limits.max_build_s)
    throw std::runtime_error("planner grid build time budget exceeded");
}

std::int64_t cellCoordinate(const double value, const double resolution)
{
  const auto scaled = std::floor(value / resolution);
  if (!std::isfinite(scaled) ||
      scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      scaled >= -static_cast<double>(std::numeric_limits<std::int64_t>::min()))
    throw std::runtime_error("planner grid coordinate exceeds int64 range");
  return static_cast<std::int64_t>(scaled);
}

PlannerVoxel voxelFor(const Point3d& point, const double resolution)
{
  return {
      cellCoordinate(point.x, resolution), cellCoordinate(point.y, resolution),
      cellCoordinate(point.z, resolution)};
}

Point3d transform(
    const mrpt::poses::CPose3D& pose, const float x, const float y, const float z)
{
  Point3d result;
  pose.composePoint(
      static_cast<double>(x), static_cast<double>(y), static_cast<double>(z),
      result.x, result.y, result.z);
  if (!(std::isfinite(result.x) && std::isfinite(result.y) &&
        std::isfinite(result.z)))
    throw std::runtime_error("MOLA corrected point is nonfinite");
  return result;
}

std::size_t checkedAdd(const std::size_t lhs, const std::size_t rhs)
{
  if (rhs > std::numeric_limits<std::size_t>::max() - lhs)
    throw std::runtime_error("planner artifact size overflow");
  return lhs + rhs;
}

std::size_t checkedMultiply(const std::size_t lhs, const std::size_t rhs)
{
  if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs)
    throw std::runtime_error("planner artifact size overflow");
  return lhs * rhs;
}

void writeU32(std::ostream& output, const std::uint32_t value)
{
  for (unsigned int shift = 0; shift < 32; shift += 8)
    output.put(static_cast<char>((value >> shift) & 0xff));
}

void writeU64(std::ostream& output, const std::uint64_t value)
{
  for (unsigned int shift = 0; shift < 64; shift += 8)
    output.put(static_cast<char>((value >> shift) & 0xff));
}

void writeI64(std::ostream& output, const std::int64_t value)
{
  writeU64(output, static_cast<std::uint64_t>(value));
}

void writeF64(std::ostream& output, const double value)
{
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  static_assert(std::numeric_limits<double>::is_iec559);
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeU64(output, bits);
}

std::string sha256File(const std::filesystem::path& path)
{
  auto context = EVP_MD_CTX_new();
  if (!context) throw std::runtime_error("OpenSSL SHA-256 allocation failed");
  const auto release = [&context]() { EVP_MD_CTX_free(context); };
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1)
  {
    release();
    throw std::runtime_error("OpenSSL SHA-256 initialization failed");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input)
  {
    release();
    throw std::runtime_error("cannot reopen planner grid artifact");
  }
  std::array<char, 1024 * 1024> buffer{};
  while (input)
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(count)) != 1)
    {
      release();
      throw std::runtime_error("OpenSSL SHA-256 update failed");
    }
  }
  if (!input.eof())
  {
    release();
    throw std::runtime_error("failed while hashing planner grid artifact");
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &length) != 1 || length != 32)
  {
    release();
    throw std::runtime_error("OpenSSL SHA-256 finalization failed");
  }
  release();
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index)
    text << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return text.str();
}

void removeQuietly(const std::filesystem::path& path)
{
  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

bool isDigest(const std::string& value)
{
  static const std::regex pattern{"^[0-9a-f]{64}$"};
  return std::regex_match(value, pattern);
}
}  // namespace

std::shared_ptr<const NativePlannerGrid> buildNativePlannerGrid(
    const NativeGeometrySnapshot& snapshot, const PlannerGridLimits& limits)
{
  validateLimits(limits);
  if (!snapshot.geometry_map)
    throw std::invalid_argument("planner grid source has no MOLA map");
  const auto poses = snapshot.geometry_map->keyframePoses();
  if (poses.size() != snapshot.keyframes.size())
    throw std::invalid_argument("MOLA keyframe provenance membership mismatch");

  const auto started = Clock::now();
  std::unordered_set<PlannerVoxel, VoxelHash> occupied;
  std::unordered_set<PlannerVoxel, VoxelHash> free;
  std::vector<PlannerSurfaceSample> surfaces;
  std::unordered_set<mola::KeyframePointCloudMap::KeyFrameID> seen_ids;
  std::size_t point_count = 0;
  std::size_t ray_steps = 0;
  std::size_t qualified_ray_keyframes = 0;

  for (const auto& keyframe : snapshot.keyframes)
  {
    if (!keyframe.points_local)
      throw std::invalid_argument("MOLA keyframe provenance has no point cloud");
    if (!seen_ids.emplace(keyframe.keyframe_id).second)
      throw std::invalid_argument("MOLA keyframe provenance repeats an ID");
    const auto found_pose = poses.find(keyframe.keyframe_id);
    if (found_pose == poses.end())
      throw std::invalid_argument("MOLA keyframe provenance references an unknown ID");
    const auto count = keyframe.points_local->size();
    if (count > limits.max_points - std::min(limits.max_points, point_count))
      throw std::runtime_error("planner grid point budget exceeded");
    point_count += count;

    const bool qualified = keyframe.ray_evidence_qualified &&
                           keyframe.sensor_origins_local.size() == 1;
    Point3d origin;
    if (qualified)
    {
      const auto& local_origin = keyframe.sensor_origins_local.front();
      origin = transform(
          found_pose->second, local_origin.x, local_origin.y, local_origin.z);
      ++qualified_ray_keyframes;
    }
    std::unordered_map<AngularBin, RayEndpoint, AngularHash> representatives;
    if (qualified) representatives.reserve(std::min<std::size_t>(count, 4096));

    for (std::size_t index = 0; index < count; ++index)
    {
      float x = 0, y = 0, z = 0;
      keyframe.points_local->getPointFast(index, x, y, z);
      const auto point = transform(found_pose->second, x, y, z);
      const auto voxel = voxelFor(point, limits.resolution_m);
      occupied.emplace(voxel);
      surfaces.push_back({voxel.x, voxel.y, point.z});
      if (occupied.size() > limits.max_voxels)
        throw std::runtime_error("planner occupied voxel budget exceeded");

      if (qualified)
      {
        const auto dx = point.x - origin.x;
        const auto dy = point.y - origin.y;
        const auto dz = point.z - origin.z;
        const auto distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance > limits.resolution_m)
        {
          const auto azimuth = std::atan2(dy, dx);
          const auto elevation = std::asin(std::clamp(dz / distance, -1.0, 1.0));
          const AngularBin bin{
              cellCoordinate(azimuth, limits.ray_angular_resolution_rad),
              cellCoordinate(elevation, limits.ray_angular_resolution_rad)};
          const auto existing = representatives.find(bin);
          if (existing == representatives.end() || distance > existing->second.distance)
            representatives[bin] = {distance, point};
        }
      }
      if ((index & 4095U) == 0) checkTime(started, limits);
    }

    for (const auto& item : representatives)
    {
      const auto& endpoint = item.second.endpoint;
      const auto steps_value = std::ceil(
          item.second.distance /
          (limits.resolution_m * limits.ray_step_fraction));
      if (!std::isfinite(steps_value))
        throw std::runtime_error("planner free-space ray exceeds step range");
      const auto remaining_ray_steps = limits.max_ray_steps - ray_steps;
      if (steps_value > static_cast<double>(remaining_ray_steps) + 1.0)
        throw std::runtime_error("planner free-space ray budget exceeded");
      const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(steps_value));
      const auto additions = steps - 1;
      ray_steps += additions;
      for (std::size_t ordinal = 1; ordinal < steps; ++ordinal)
      {
        const auto scale = static_cast<double>(ordinal) / static_cast<double>(steps);
        free.emplace(voxelFor(
            {origin.x + (endpoint.x - origin.x) * scale,
             origin.y + (endpoint.y - origin.y) * scale,
             origin.z + (endpoint.z - origin.z) * scale},
            limits.resolution_m));
        if (free.size() >
            limits.max_voxels - std::min(limits.max_voxels, occupied.size()))
          throw std::runtime_error("planner total voxel budget exceeded");
        if ((ordinal & 4095U) == 0) checkTime(started, limits);
      }
      checkTime(started, limits);
    }
  }
  if (point_count != snapshot.geometry_map->point_count())
    throw std::invalid_argument("MOLA keyframe provenance point count mismatch");

  std::size_t erased = 0;
  for (const auto& voxel : occupied)
  {
    free.erase(voxel);
    if ((++erased & 4095U) == 0) checkTime(started, limits);
  }
  auto result = std::make_shared<NativePlannerGrid>();
  result->graph_version = snapshot.graph_version;
  result->identity = snapshot.identity;
  result->resolution_m = limits.resolution_m;
  result->ray_angular_resolution_rad = limits.ray_angular_resolution_rad;
  result->ray_step_fraction = limits.ray_step_fraction;
  result->point_count = point_count;
  result->ray_steps = ray_steps;
  result->qualified_ray_keyframes = qualified_ray_keyframes;
  result->occupied.assign(occupied.begin(), occupied.end());
  result->free.assign(free.begin(), free.end());
  result->surfaces = std::move(surfaces);
  for (const auto& keyframe : snapshot.keyframes)
    result->source_stamp_ns = std::max(result->source_stamp_ns, keyframe.observed_at_ns);
  std::sort(result->occupied.begin(), result->occupied.end());
  checkTime(started, limits);
  std::sort(result->free.begin(), result->free.end());
  checkTime(started, limits);
  std::sort(result->surfaces.begin(), result->surfaces.end());
  checkTime(started, limits);
  return result;
}

PlannerGridArtifact writeNativePlannerGrid(
    const NativePlannerGrid& grid, const std::filesystem::path& output,
    const std::size_t max_bytes, const std::size_t max_metadata_bytes)
{
  if (max_bytes == 0 || max_metadata_bytes == 0)
    throw std::invalid_argument("planner artifact limits must be positive");
  if (output.empty()) throw std::invalid_argument("planner artifact path is empty");
  if (std::filesystem::exists(output))
    throw std::runtime_error("planner artifact already exists");
  if (!(std::isfinite(grid.resolution_m) && grid.resolution_m > 0) ||
      !(std::isfinite(grid.ray_angular_resolution_rad) &&
        grid.ray_angular_resolution_rad > 0) ||
      !(std::isfinite(grid.ray_step_fraction) && grid.ray_step_fraction > 0 &&
        grid.ray_step_fraction <= 1))
    throw std::invalid_argument("planner grid options are invalid");
  if (!isDigest(grid.graph_version.digest) ||
      !isDigest(grid.identity.geometry_revision) ||
      !isDigest(grid.identity.native_geometry_digest) ||
      !isDigest(grid.identity.canonical_manifest_digest) ||
      !isDigest(grid.identity.source_snapshot_id) ||
      !isDigest(grid.identity.source_sha256))
    throw std::invalid_argument("planner grid identity is invalid");
  if (grid.point_count != grid.surfaces.size())
    throw std::invalid_argument("planner surface count does not match point count");
  if (grid.point_count > kMaxPlannerProductPoints ||
      grid.occupied.size() > kMaxPlannerProductVoxels ||
      grid.free.size() > kMaxPlannerProductVoxels ||
      grid.free.size() >
          kMaxPlannerProductVoxels -
              std::min(kMaxPlannerProductVoxels, grid.occupied.size()) ||
      grid.ray_steps > kMaxPlannerProductRaySteps)
    throw std::invalid_argument("planner grid record count exceeds limit");
  if (!std::is_sorted(grid.occupied.begin(), grid.occupied.end()) ||
      !std::is_sorted(grid.free.begin(), grid.free.end()) ||
      !std::is_sorted(grid.surfaces.begin(), grid.surfaces.end()))
    throw std::invalid_argument("planner grid records are not sorted");
  if (std::adjacent_find(grid.occupied.begin(), grid.occupied.end()) !=
          grid.occupied.end() ||
      std::adjacent_find(grid.free.begin(), grid.free.end()) != grid.free.end())
    throw std::invalid_argument("planner grid repeats a voxel");
  auto occupied_it = grid.occupied.begin();
  auto free_it = grid.free.begin();
  while (occupied_it != grid.occupied.end() && free_it != grid.free.end())
  {
    if (*occupied_it == *free_it)
      throw std::invalid_argument("planner voxel is both occupied and free");
    if (*occupied_it < *free_it)
      ++occupied_it;
    else
      ++free_it;
  }
  const auto& version = grid.graph_version;
  const auto& identity = grid.identity;
  const json metadata{
      {"schema", "swarmdeck.mola_planner_grid.v1"},
      {"graph_version",
       {{"component_id", version.component_id},
        {"epoch", version.epoch},
        {"revision", version.revision},
        {"digest", version.digest}}},
      {"identity",
       {{"geometry_revision", identity.geometry_revision},
        {"native_geometry_digest", identity.native_geometry_digest},
        {"canonical_manifest_digest", identity.canonical_manifest_digest},
        {"source_snapshot_id", identity.source_snapshot_id},
        {"source_sha256", identity.source_sha256},
        {"reference_frame", identity.reference_frame}}},
      {"source_stamp_ns", grid.source_stamp_ns},
      {"resolution_m", grid.resolution_m},
      {"ray_angular_resolution_rad", grid.ray_angular_resolution_rad},
      {"ray_step_fraction", grid.ray_step_fraction},
      {"point_count", grid.point_count},
      {"occupied_count", grid.occupied.size()},
      {"free_count", grid.free.size()},
      {"surface_count", grid.surfaces.size()},
      {"ray_steps", grid.ray_steps},
      {"qualified_ray_keyframes", grid.qualified_ray_keyframes}};
  const auto metadata_bytes = metadata.dump();
  if (metadata_bytes.size() > max_metadata_bytes ||
      metadata_bytes.size() > std::numeric_limits<std::uint32_t>::max())
    throw std::runtime_error("planner metadata exceeds byte limit");

  auto expected = checkedAdd(kMagic.size(), sizeof(std::uint32_t));
  expected = checkedAdd(expected, metadata_bytes.size());
  expected = checkedAdd(expected, checkedMultiply(grid.occupied.size(), 24));
  expected = checkedAdd(expected, checkedMultiply(grid.free.size(), 24));
  expected = checkedAdd(expected, checkedMultiply(grid.surfaces.size(), 24));
  if (expected == 0 || expected > max_bytes)
    throw std::runtime_error("planner artifact exceeds byte limit");

  auto temporary = output;
  temporary += ".tmp." + std::to_string(::getpid()) + "." +
               std::to_string(temporary_sequence.fetch_add(1));
  removeQuietly(temporary);
  try
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot create planner artifact");
    stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
    writeU32(stream, static_cast<std::uint32_t>(metadata_bytes.size()));
    stream.write(
        metadata_bytes.data(), static_cast<std::streamsize>(metadata_bytes.size()));
    for (const auto& voxel : grid.occupied)
    {
      writeI64(stream, voxel.x);
      writeI64(stream, voxel.y);
      writeI64(stream, voxel.z);
    }
    for (const auto& voxel : grid.free)
    {
      writeI64(stream, voxel.x);
      writeI64(stream, voxel.y);
      writeI64(stream, voxel.z);
    }
    for (const auto& surface : grid.surfaces)
    {
      if (!std::isfinite(surface.z))
        throw std::invalid_argument("planner surface contains nonfinite height");
      writeI64(stream, surface.x);
      writeI64(stream, surface.y);
      writeF64(stream, surface.z);
    }
    stream.close();
    if (!stream) throw std::runtime_error("failed while writing planner artifact");
    const auto actual = std::filesystem::file_size(temporary);
    if (actual != expected)
      throw std::runtime_error("planner artifact size mismatch");
    const auto digest = sha256File(temporary);
    std::error_code error;
    std::filesystem::rename(temporary, output, error);
    if (error) throw std::runtime_error("cannot install planner artifact");
    return {expected, digest};
  }
  catch (...)
  {
    removeQuietly(temporary);
    throw;
  }
}

}  // namespace swarmdeck_mapping
