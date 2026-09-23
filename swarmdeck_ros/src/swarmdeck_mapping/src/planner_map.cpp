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
#include <tuple>
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

struct RayCandidate
{
  AngularBin bin;
  double distance{};
  Point3d endpoint;
  std::size_t additions{};
};

struct RayFrame
{
  Point3d origin;
  std::uint64_t observed_at_ns{};
  std::vector<RayCandidate> candidates;
};

/**
 * Visibility evidence held for one occupied voxel.
 *
 * `newest_endpoint_ns` and `highest_endpoint_z` are final before any ray is
 * carved, because every keyframe's endpoints are ingested first. A traversal
 * therefore only needs a single saturating counter: it is admitted when its
 * own capture is strictly later than every endpoint in the voxel and its
 * sampled point lies no higher than the highest endpoint plus the clearing
 * height tolerance, so an endpoint seen again after the clearing rays
 * re-confirms the voxel and silently disqualifies them, and a ray that passed
 * over the endpoints never counted.
 */
struct VoxelEvidence
{
  std::uint64_t newest_endpoint_ns{};
  // Height of the highest endpoint stored in the voxel; every occupied voxel
  // ingests at least one finite endpoint before any ray reads this.
  double highest_endpoint_z{-std::numeric_limits<double>::infinity()};
  std::uint32_t clearing_traversals{};
  bool retired{};
};

using OccupiedVoxels = std::unordered_map<PlannerVoxel, VoxelEvidence, VoxelHash>;

void validateLimits(const PlannerGridLimits& limits)
{
  if (!(std::isfinite(limits.resolution_m) && limits.resolution_m > 0) ||
      limits.max_points == 0 || limits.max_voxels == 0 ||
      limits.max_ray_steps == 0 || limits.min_clearing_traversals == 0 ||
      !(std::isfinite(limits.max_build_s) && limits.max_build_s > 0) ||
      !(std::isfinite(limits.ray_angular_resolution_rad) &&
        limits.ray_angular_resolution_rad > 0) ||
      !(std::isfinite(limits.ray_step_fraction) &&
        limits.ray_step_fraction > 0 && limits.ray_step_fraction <= 1) ||
      !(std::isfinite(limits.clearing_height_tolerance_m) &&
        limits.clearing_height_tolerance_m >= 0))
    throw std::invalid_argument(
        "planner grid limits must be finite and positive (the clearing height "
        "tolerance may be zero)");
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

struct NativePlannerAccumulator::Impl
{
  struct Evidence
  {
    VoxelEvidence visibility;
    double low{std::numeric_limits<double>::infinity()};
  };
  PlannerGridLimits limits;
  std::unordered_map<PlannerVoxel, Evidence, VoxelHash> occupied;
  std::unordered_set<PlannerVoxel, VoxelHash> free;
  std::size_t samples{}, source_points{}, steps{}, qualified{};
  std::uint64_t newest{};

  static Point3d point(const Matrix4& m, const PointXYZ& p)
  {
    Point3d q{m[0]*p.x + m[1]*p.y + m[2]*p.z + m[3],
              m[4]*p.x + m[5]*p.y + m[6]*p.z + m[7],
              m[8]*p.x + m[9]*p.y + m[10]*p.z + m[11]};
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
      throw std::invalid_argument("nonfinite planner endpoint");
    return q;
  }
};

NativePlannerAccumulator::NativePlannerAccumulator(const PlannerGridLimits& limits)
    : impl_(std::make_unique<Impl>())
{
  validateLimits(limits);
  impl_->limits = limits;
}
NativePlannerAccumulator::NativePlannerAccumulator(const NativePlannerAccumulator& other)
    : impl_(std::make_unique<Impl>(*other.impl_))
{
  impl_->steps = 0;
}
NativePlannerAccumulator::~NativePlannerAccumulator() = default;
std::size_t NativePlannerAccumulator::residentUnits() const
{
  return impl_->occupied.size() + impl_->free.size() + impl_->samples;
}
std::uint64_t NativePlannerAccumulator::newestStamp() const { return impl_->newest; }

void NativePlannerAccumulator::endpoints(const SubmapInput& frame)
{
  auto& state = *impl_;
  const auto started = Clock::now();
  state.newest = std::max(state.newest, frame.observed_at_ns);
  state.source_points = checkedAdd(state.source_points, frame.points_local.size());
  std::size_t index = 0;
  for (const auto& local : frame.points_local)
  {
    const auto p = Impl::point(frame.T_component_submap, local);
    const auto voxel = voxelFor(p, state.limits.resolution_m);
    auto [it, inserted] = state.occupied.try_emplace(voxel);
    auto& evidence = it->second;
    const auto before = inserted ? 0U :
        (evidence.low == evidence.visibility.highest_endpoint_z ? 1U : 2U);
    evidence.low = std::min(evidence.low, p.z);
    evidence.visibility.highest_endpoint_z =
        std::max(evidence.visibility.highest_endpoint_z, p.z);
    if (frame.observed_at_ns >= evidence.visibility.newest_endpoint_ns)
    {
      evidence.visibility.newest_endpoint_ns = frame.observed_at_ns;
      evidence.visibility.clearing_traversals = 0;
    }
    state.samples += (evidence.low == evidence.visibility.highest_endpoint_z ? 1U : 2U) - before;
    state.free.erase(voxel);
    if (state.samples > state.limits.max_points ||
        state.occupied.size() + state.free.size() > state.limits.max_voxels)
      throw std::runtime_error("planner materialized evidence budget exceeded");
    if ((++index & 4095U) == 0) checkTime(started, state.limits);
  }
}

void NativePlannerAccumulator::rays(const SubmapInput& frame, std::size_t work_budget)
{
  auto& state = *impl_;
  if (!frame.ray_evidence_qualified || frame.sensor_origins_local.size() != 1) return;
  ++state.qualified;
  const auto started = Clock::now();
  const auto origin = Impl::point(frame.T_component_submap, frame.sensor_origins_local.front());
  std::unordered_map<AngularBin, RayEndpoint, AngularHash> representatives;
  for (const auto& local : frame.points_local)
  {
    const auto p = Impl::point(frame.T_component_submap, local);
    const double dx = p.x-origin.x, dy = p.y-origin.y, dz = p.z-origin.z;
    const auto distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (distance <= state.limits.resolution_m) continue;
    const AngularBin bin{
        cellCoordinate(std::atan2(dy, dx), state.limits.ray_angular_resolution_rad),
        cellCoordinate(std::asin(std::clamp(dz/distance, -1.0, 1.0)),
                       state.limits.ray_angular_resolution_rad)};
    auto it = representatives.find(bin);
    if (it == representatives.end() || distance > it->second.distance ||
        (distance == it->second.distance &&
         std::tie(p.x,p.y,p.z) < std::tie(it->second.endpoint.x,it->second.endpoint.y,it->second.endpoint.z)))
      representatives[bin] = {distance, p};
  }
  std::vector<RayCandidate> rays;
  for (const auto& entry : representatives)
  {
    const auto count = std::ceil(entry.second.distance /
        (state.limits.resolution_m * state.limits.ray_step_fraction));
    if (!std::isfinite(count) || count > static_cast<double>(work_budget) + 1) continue;
    rays.push_back({entry.first, entry.second.distance, entry.second.endpoint,
                   static_cast<std::size_t>(count) - 1});
  }
  std::sort(rays.begin(), rays.end(), [](const RayCandidate& a, const RayCandidate& b) {
    return std::tie(a.additions,a.bin.azimuth,a.bin.elevation) <
           std::tie(b.additions,b.bin.azimuth,b.bin.elevation);
  });
  for (const auto& ray : rays)
  {
    if (ray.additions > work_budget ||
        ray.additions > state.limits.max_ray_steps - state.steps) break;
    work_budget -= ray.additions;
    state.steps += ray.additions;
    PlannerVoxel previous{};
    bool have_previous = false, counted = false;
    for (std::size_t step = 1; step <= ray.additions; ++step)
    {
      const double t = static_cast<double>(step) / (ray.additions + 1);
      const Point3d sample{origin.x+(ray.endpoint.x-origin.x)*t,
                           origin.y+(ray.endpoint.y-origin.y)*t,
                           origin.z+(ray.endpoint.z-origin.z)*t};
      const auto voxel = voxelFor(sample, state.limits.resolution_m);
      if (!have_previous || !(previous == voxel))
      {
        previous = voxel; have_previous = true; counted = false;
      }
      const auto found = state.occupied.find(voxel);
      if (found == state.occupied.end())
      {
        state.free.insert(voxel);
        if (state.occupied.size() + state.free.size() > state.limits.max_voxels)
          throw std::runtime_error("planner materialized voxel budget exceeded");
      }
      else
      {
        auto& evidence = found->second.visibility;
        if (!counted && frame.observed_at_ns > evidence.newest_endpoint_ns &&
            sample.z <= evidence.highest_endpoint_z + state.limits.clearing_height_tolerance_m)
        {
          counted = true;
          if (evidence.clearing_traversals < state.limits.min_clearing_traversals)
            ++evidence.clearing_traversals;
        }
      }
      if ((step & 4095U) == 0) checkTime(started, state.limits);
    }
    checkTime(started, state.limits);
  }
}

std::shared_ptr<const NativePlannerGrid> NativePlannerAccumulator::snapshot(
    const SolutionVersion& version, const SnapshotIdentity& identity) const
{
  const auto& state = *impl_;
  auto result = std::make_shared<NativePlannerGrid>();
  result->graph_version = version; result->identity = identity;
  result->source_stamp_ns = state.newest;
  result->resolution_m = state.limits.resolution_m;
  result->ray_angular_resolution_rad = state.limits.ray_angular_resolution_rad;
  result->ray_step_fraction = state.limits.ray_step_fraction;
  result->point_count = state.samples; result->ray_steps = state.steps;
  result->source_point_count = state.source_points;
  result->qualified_ray_keyframes = state.qualified;
  result->free.assign(state.free.begin(), state.free.end());
  for (const auto& [voxel, evidence] : state.occupied)
  {
    const bool two = evidence.low != evidence.visibility.highest_endpoint_z;
    if (evidence.visibility.clearing_traversals >= state.limits.min_clearing_traversals)
    {
      result->free.push_back(voxel);
      result->retired_count += two ? 2 : 1;
      continue;
    }
    result->occupied.push_back(voxel);
    result->surfaces.push_back({voxel.x,voxel.y,evidence.low});
    if (two) result->surfaces.push_back({voxel.x,voxel.y,evidence.visibility.highest_endpoint_z});
  }
  std::sort(result->occupied.begin(), result->occupied.end());
  std::sort(result->free.begin(), result->free.end());
  std::sort(result->surfaces.begin(), result->surfaces.end());
  return result;
}

std::shared_ptr<const NativePlannerGrid> buildNativePlannerGrid(
    const NativeGeometrySnapshot& snapshot, const PlannerGridLimits& limits)
{
  validateLimits(limits);
  if (!snapshot.geometry_map)
    throw std::invalid_argument("planner grid source has no MOLA map");
  const auto poses = snapshot.geometry_map->keyframePoses();
  if (poses.size() != snapshot.keyframes.size())
    throw std::invalid_argument("MOLA keyframe provenance membership mismatch");
  // The map's own count is checked against the provenance below; refusing
  // here, before any work, names the numbers the operator needs.
  if (snapshot.geometry_map->point_count() > limits.max_points)
    throw std::runtime_error(
        "planner grid point budget exceeded: " +
        std::to_string(snapshot.geometry_map->point_count()) + " points, budget " +
        std::to_string(limits.max_points));

  const auto started = Clock::now();
  OccupiedVoxels occupied;
  std::unordered_set<PlannerVoxel, VoxelHash> free;
  std::vector<PlannerSurfaceSample> surfaces;
  std::vector<RayFrame> ray_frames;
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
      auto& evidence = occupied[voxel];
      evidence.newest_endpoint_ns =
          std::max(evidence.newest_endpoint_ns, keyframe.observed_at_ns);
      evidence.highest_endpoint_z = std::max(evidence.highest_endpoint_z, point.z);
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
          const auto point_order = [](const Point3d& lhs, const Point3d& rhs) {
            return std::tie(lhs.x, lhs.y, lhs.z) < std::tie(rhs.x, rhs.y, rhs.z);
          };
          if (existing == representatives.end() ||
              distance > existing->second.distance ||
              (distance == existing->second.distance &&
               point_order(point, existing->second.endpoint)))
            representatives[bin] = {distance, point};
        }
      }
      if ((index & 4095U) == 0) checkTime(started, limits);
    }

    RayFrame frame{origin, keyframe.observed_at_ns, {}};
    frame.candidates.reserve(representatives.size());
    for (const auto& item : representatives)
    {
      const auto steps_value = std::ceil(
          item.second.distance /
          (limits.resolution_m * limits.ray_step_fraction));
      if (!std::isfinite(steps_value))
        throw std::runtime_error("planner free-space ray exceeds step range");
      // A ray that cannot fit even an otherwise empty product contributes no
      // free cells. Its measured endpoint remains occupied in the product.
      if (steps_value > static_cast<double>(limits.max_ray_steps) + 1.0)
        continue;
      const auto steps =
          std::max<std::size_t>(1, static_cast<std::size_t>(steps_value));
      const auto additions = steps - 1;
      frame.candidates.push_back(
          {item.first, item.second.distance, item.second.endpoint, additions});
    }
    std::sort(
        frame.candidates.begin(), frame.candidates.end(),
        [](const RayCandidate& lhs, const RayCandidate& rhs) {
          return std::tie(
                     lhs.additions, lhs.bin.azimuth, lhs.bin.elevation,
                     lhs.distance, lhs.endpoint.x, lhs.endpoint.y, lhs.endpoint.z) <
                 std::tie(
                     rhs.additions, rhs.bin.azimuth, rhs.bin.elevation,
                     rhs.distance, rhs.endpoint.x, rhs.endpoint.y, rhs.endpoint.z);
        });
    if (!frame.candidates.empty()) ray_frames.push_back(std::move(frame));
    checkTime(started, limits);
  }

  // Every admitted ray remains original measured evidence. Fairly take the
  // nearest remaining ray from each keyframe before taking a second one from
  // any keyframe. Once the work allowance is full, omitted rays simply leave
  // their cells unknown; all endpoints and exact surface heights remain in the
  // product. This avoids making a growing but otherwise bounded map
  // permanently unpublishable because old rays repeatedly cross the same
  // voxels.
  std::vector<std::size_t> active_frames(ray_frames.size());
  for (std::size_t index = 0; index < active_frames.size(); ++index)
    active_frames[index] = index;
  std::vector<std::size_t> cursors(ray_frames.size(), 0);
  std::size_t considered = 0;
  while (!active_frames.empty() && ray_steps < limits.max_ray_steps)
  {
    std::vector<std::size_t> next_frames;
    next_frames.reserve(active_frames.size());
    for (const auto frame_index : active_frames)
    {
      const auto& frame = ray_frames[frame_index];
      const auto cursor = cursors[frame_index];
      const auto& candidate = frame.candidates[cursor];
      const auto remaining = limits.max_ray_steps - ray_steps;
      if (candidate.additions <= remaining)
      {
        ray_steps += candidate.additions;
        const auto steps = candidate.additions + 1;
        const auto& endpoint = candidate.endpoint;
        const auto& origin = frame.origin;
        // A step fraction below one lets consecutive steps of one ray land in
        // the same voxel. Steps advance monotonically along the ray, so a
        // voxel is entered once per ray: the free set is decided on entry,
        // and the clearing count below is taken at most once while the ray
        // stays in the voxel, which keeps one stray ray from retiring
        // anything on its own. Every step is still examined, because a ray
        // descending through a voxel can sample above its endpoints first and
        // among them next. `occupied` gains nothing while rays are carved, so
        // the entry iterator stays valid across the steps spent in a voxel.
        PlannerVoxel previous_voxel{};
        bool have_previous = false;
        auto found = occupied.end();
        bool counted = false;
        for (std::size_t ordinal = 1; ordinal < steps; ++ordinal)
        {
          const auto scale = static_cast<double>(ordinal) / static_cast<double>(steps);
          const Point3d sample{
              origin.x + (endpoint.x - origin.x) * scale,
              origin.y + (endpoint.y - origin.y) * scale,
              origin.z + (endpoint.z - origin.z) * scale};
          const auto voxel = voxelFor(sample, limits.resolution_m);
          if (!have_previous || !(voxel == previous_voxel))
          {
            previous_voxel = voxel;
            have_previous = true;
            counted = false;
            found = occupied.find(voxel);
            if (found == occupied.end())
            {
              free.emplace(voxel);
              if (free.size() >
                  limits.max_voxels - std::min(limits.max_voxels, occupied.size()))
                throw std::runtime_error("planner total voxel budget exceeded");
            }
          }
          // Seeing through an occupied voxel is the only evidence that can
          // disprove its endpoints. Rays observed no later than the newest
          // endpoint there prove nothing: the object may have arrived after
          // them. A ray sampled above every endpoint in the voxel proves
          // nothing either: it passed over them, not through them (the road
          // case in the retirement note below), so it neither counts nor
          // frees the voxel.
          if (found != occupied.end() && !counted &&
              frame.observed_at_ns > found->second.newest_endpoint_ns &&
              sample.z <= found->second.highest_endpoint_z +
                              limits.clearing_height_tolerance_m)
          {
            counted = true;
            if (found->second.clearing_traversals < limits.min_clearing_traversals)
              ++found->second.clearing_traversals;
          }
          if ((ordinal & 4095U) == 0) checkTime(started, limits);
        }
      }
      // Candidates are ordered by cost. If this one does not fit the global
      // remainder, no later candidate from the same frame can fit either.
      if (candidate.additions <= remaining &&
          ++cursors[frame_index] < frame.candidates.size())
        next_frames.push_back(frame_index);
      if ((++considered & 4095U) == 0) checkTime(started, limits);
    }
    active_frames = std::move(next_frames);
    checkTime(started, limits);
  }
  if (point_count != snapshot.geometry_map->point_count())
    throw std::invalid_argument("MOLA keyframe provenance point count mismatch");

  // Visibility retirement. Obstacle expiry alone cannot prove an area clear,
  // so a body that stood in front of the sensor when it was captured would
  // otherwise stay in the product forever, both as an occupied voxel and as a
  // terrain surface sample. Repeated qualified free rays observed later than
  // every endpoint in the voxel, and sampled no higher than the voxel's
  // highest endpoint plus `clearing_height_tolerance_m`, are the positive
  // evidence that retires those endpoints.
  //
  // The height rule exists for the ground. A road surface is a sheet near
  // the bottom of its voxel (samples at z = -0.34 in the voxel spanning
  // [-0.40, -0.20), for example), and rays from a lidar 0.5 m above the road
  // that end far ahead graze the last quarter of their length within 0.14 m
  // of the surface: they traverse the road's own ground voxels above the
  // sheet without touching it. Later keyframes (scene-change keyframes taken
  // while parked, or keyframes taken further back) supply such rays with a
  // later observation time, and counting them retired the road ahead in
  // bands. Measured on benchbot on 2026-09-18: robot_1's three-keyframe
  // product (`retired 1054`) had a 1.3 m wide strip of the lane 10 to 11.3 m
  // ahead with no occupied voxel and no surface sample, only free voxels at
  // z = -0.3 and above, and its 8 m goal there was refused three runs in a
  // row (`no mapped ground support`, `known rise 0.31 m`). A ray passing
  // above every endpoint in a voxel proves nothing about those endpoints.
  //
  // Unknown space is untouched: a voxel nothing ever saw through keeps
  // whatever it had.
  std::size_t retired_voxels = 0;
  for (auto& entry : occupied)
  {
    if (entry.second.clearing_traversals < limits.min_clearing_traversals) continue;
    entry.second.retired = true;
    ++retired_voxels;
    // The endpoint was the only reason the existing carving rule left this
    // voxel out of the free set, and the rays that retired it did traverse it.
    free.emplace(entry.first);
  }
  checkTime(started, limits);
  std::size_t retired_count = 0;
  if (retired_voxels != 0)
  {
    std::vector<PlannerSurfaceSample> kept;
    kept.reserve(surfaces.size());
    for (const auto& sample : surfaces)
    {
      // A surface sample carries its exact endpoint height, so its voxel is
      // recoverable without storing it a second time.
      const auto found = occupied.find(
          {sample.x, sample.y, cellCoordinate(sample.z, limits.resolution_m)});
      if (found != occupied.end() && found->second.retired)
      {
        ++retired_count;
        continue;
      }
      kept.push_back(sample);
    }
    surfaces = std::move(kept);
  }

  checkTime(started, limits);
  auto result = std::make_shared<NativePlannerGrid>();
  result->graph_version = snapshot.graph_version;
  result->identity = snapshot.identity;
  result->resolution_m = limits.resolution_m;
  result->ray_angular_resolution_rad = limits.ray_angular_resolution_rad;
  result->ray_step_fraction = limits.ray_step_fraction;
  result->point_count = point_count;
  result->source_point_count = point_count;
  result->ray_steps = ray_steps;
  result->qualified_ray_keyframes = qualified_ray_keyframes;
  result->retired_count = retired_count;
  result->occupied.reserve(occupied.size() - retired_voxels);
  for (const auto& entry : occupied)
    if (!entry.second.retired) result->occupied.push_back(entry.first);
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
    const std::size_t max_bytes, const std::size_t max_metadata_bytes,
    const std::size_t max_points)
{
  if (max_bytes == 0 || max_metadata_bytes == 0 || max_points == 0)
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
  // The source count authenticates all input endpoints; the bounded product
  // count covers only materialized surface extrema and retired extrema.
  if (grid.point_count > grid.source_point_count ||
      grid.retired_count > grid.point_count ||
      grid.surfaces.size() != grid.point_count - grid.retired_count)
    throw std::invalid_argument(
        "planner surface and retired counts do not match point count");
  if (grid.point_count > max_points)
    throw std::invalid_argument(
        "planner grid point count exceeds the product budget: " +
        std::to_string(grid.point_count) + " points, budget " +
        std::to_string(max_points));
  if (grid.occupied.size() > kMaxPlannerProductVoxels ||
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
      {"schema", "swarmdeck.mola_planner_grid.v2"},
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
      {"source_point_count", grid.source_point_count},
      {"occupied_count", grid.occupied.size()},
      {"free_count", grid.free.size()},
      {"surface_count", grid.surfaces.size()},
      {"retired_count", grid.retired_count},
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
