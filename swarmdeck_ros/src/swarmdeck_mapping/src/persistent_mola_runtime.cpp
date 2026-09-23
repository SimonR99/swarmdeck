#include <swarmdeck_mapping/persistent_mola_runtime.hpp>

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include <unistd.h>

namespace swarmdeck_mapping
{
namespace
{
std::atomic<std::uint64_t> temporary_sequence{};

bool sameGraphVersion(const SolutionVersion& lhs, const SolutionVersion& rhs)
{
  return lhs.component_id == rhs.component_id && lhs.epoch == rhs.epoch &&
         lhs.revision == rhs.revision;
}

bool commonPosesAgree(
    const ParsedComponentSnapshot& next, const NativeGeometrySnapshot& prior)
{
  std::unordered_map<std::string, Matrix4> previous;
  previous.reserve(prior.submap_poses.size());
  for (const auto& pose : prior.submap_poses)
    previous.emplace(pose.external_id, pose.T_component_submap);
  for (const auto& submap : next.submaps)
  {
    const auto found = previous.find(submap.external_id);
    if (found != previous.end() && found->second != submap.T_component_submap)
      return false;
  }
  return true;
}
bool appendOnly(
    const ParsedComponentSnapshot& next, const NativeGeometrySnapshot& prior)
{
  if (next.submaps.size() < prior.keyframes.size()) return false;
  std::unordered_map<std::string, const ParsedSubmap*> by_id;
  std::unordered_map<std::string, Matrix4> poses;
  for (const auto& submap : next.submaps) by_id.emplace(submap.external_id, &submap);
  for (const auto& pose : prior.submap_poses) poses.emplace(pose.external_id, pose.T_component_submap);
  for (const auto& keyframe : prior.keyframes)
  {
    const auto found = by_id.find(keyframe.external_id);
    const auto old_pose = poses.find(keyframe.external_id);
    if (found == by_id.end() || old_pose == poses.end()) return false;
    const auto& submap = *found->second;
    if (submap.geometry_revision != keyframe.geometry_revision ||
        submap.T_component_submap != old_pose->second ||
        submap.chunks.size() != keyframe.chunk_fingerprints.size())
      return false;
    if (submap.observed_at_ns != keyframe.observed_at_ns ||
        submap.ray_evidence_qualified != keyframe.ray_evidence_qualified ||
        submap.sensor_origins_local.size() != keyframe.sensor_origins_local.size())
      return false;
    for (std::size_t i = 0; i < submap.sensor_origins_local.size(); ++i)
    {
      const auto& a = submap.sensor_origins_local[i];
      const auto& b = keyframe.sensor_origins_local[i];
      if (a.x != b.x || a.y != b.y || a.z != b.z) return false;
    }
    for (std::size_t i = 0; i < submap.chunks.size(); ++i)
      if (submap.chunks[i].fingerprint() != keyframe.chunk_fingerprints[i]) return false;
  }
  return true;
}

void validateToken(
    const std::string& value, const char* field, const std::size_t maximum,
    const std::regex& pattern)
{
  if (value.empty() || value.size() > maximum || !std::regex_match(value, pattern))
    throw RuntimeError(RuntimeErrorCode::InvalidRequest, std::string("invalid ") + field);
}

void validatePath(const std::filesystem::path& path, const char* field)
{
  const auto text = path.string();
  if (text.empty() || text.size() > 4096 || text.find('\0') != std::string::npos ||
      text.find('\n') != std::string::npos || text.find('\r') != std::string::npos)
    throw RuntimeError(RuntimeErrorCode::InvalidRequest, std::string("invalid ") + field);
}

std::string sha256File(const std::filesystem::path& path)
{
  auto context = EVP_MD_CTX_new();
  if (!context) throw RuntimeError(RuntimeErrorCode::Internal, "OpenSSL SHA-256 allocation failed");
  const auto release = [&context]() { EVP_MD_CTX_free(context); };
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1)
  {
    release();
    throw RuntimeError(RuntimeErrorCode::Internal, "OpenSSL SHA-256 initialization failed");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input)
  {
    release();
    throw RuntimeError(RuntimeErrorCode::Io, "cannot reopen serialized metric map");
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
      throw RuntimeError(RuntimeErrorCode::Internal, "OpenSSL SHA-256 update failed");
    }
  }
  if (!input.eof())
  {
    release();
    throw RuntimeError(RuntimeErrorCode::Io, "failed while hashing serialized metric map");
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &length) != 1 || length != 32)
  {
    release();
    throw RuntimeError(RuntimeErrorCode::Internal, "OpenSSL SHA-256 finalization failed");
  }
  release();
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index)
    text << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return text.str();
}

struct Artifact
{
  std::size_t size{};
  std::string sha256;
  bool installed{};
};

void removeQuietly(const std::filesystem::path& path)
{
  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

void serializeArtifact(
    const std::shared_ptr<const mola::KeyframePointCloudMap>& map,
    const std::filesystem::path& output, const std::size_t maximum,
    Artifact& artifact)
{
  if (std::filesystem::exists(output))
    throw RuntimeError(RuntimeErrorCode::Io, "output artifact already exists");
  auto temporary = output;
  temporary += ".tmp." + std::to_string(::getpid()) + "." +
               std::to_string(temporary_sequence.fetch_add(1));
  removeQuietly(temporary);
  try
  {
    {
      mrpt::io::CFileGZOutputStream stream(temporary.string());
      if (!stream.is_open())
        throw RuntimeError(RuntimeErrorCode::Io, "cannot open temporary metric map");
      auto archive = mrpt::serialization::archiveFrom(stream);
      archive << *map;
    }
    std::error_code error;
    const auto size = std::filesystem::file_size(temporary, error);
    if (error) throw RuntimeError(RuntimeErrorCode::Io, "cannot stat serialized metric map");
    if (size == 0 || size > maximum)
      throw RuntimeError(RuntimeErrorCode::ResourceLimit, "serialized metric map exceeds output limit");
    const auto digest = sha256File(temporary);
    std::filesystem::rename(temporary, output, error);
    if (error) throw RuntimeError(RuntimeErrorCode::Io, "cannot install serialized metric map");
    artifact = {static_cast<std::size_t>(size), digest, true};
  }
  catch (...)
  {
    removeQuietly(temporary);
    throw;
  }
}
}  // namespace

PersistentMolaRuntime::PersistentMolaRuntime(RuntimeLimits limits) : limits_(limits)
{
  if (limits_.max_line_bytes == 0 || limits_.max_snapshot_bytes == 0 ||
      limits_.max_response_bytes == 0 || limits_.max_maps == 0 ||
      limits_.max_submaps_per_map == 0 || limits_.max_chunks_per_map == 0 ||
      limits_.max_points_per_map == 0 || limits_.max_resident_points == 0 ||
      limits_.max_output_bytes == 0 || limits_.max_planner_output_bytes == 0)
    throw std::invalid_argument("runtime limits must be positive");
}

PlannerGridLimits PersistentMolaRuntime::plannerGridLimits() const noexcept
{
  // The grid build carried its own point default until 2026-09-19, half the
  // loader's, so a map the loader admitted could still fail to produce a
  // planner grid on every build (benchbot mission 1a8cc114). The loader's
  // budget is the only point budget.
  PlannerGridLimits limits;
  limits.max_points = limits_.max_points_per_map;
  return limits;
}

ApplyReport PersistentMolaRuntime::apply(const ApplyRequest& request)
try
{
  static const std::regex request_pattern{"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$"};
  static const std::regex map_pattern{"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,511}$"};
  validateToken(request.request_id, "request_id", 128, request_pattern);
  validateToken(request.map_id, "map_id", 512, map_pattern);
  validatePath(request.snapshot_path, "snapshot_path");
  validatePath(request.chunks_dir, "chunks_dir");
  if (!request.output_path.empty()) validatePath(request.output_path, "output_path");
  if (!request.planner_output_path.empty())
    validatePath(request.planner_output_path, "planner_output_path");
  if (!request.output_path.empty() && request.output_path == request.planner_output_path)
    throw RuntimeError(
        RuntimeErrorCode::InvalidRequest,
        "geometry and planner output paths must differ");

  ParsedComponentSnapshot parsed;
  try
  {
    parsed = parseComponentSnapshot(
        request.snapshot_path, request.snapshot_sha256, limits_.max_snapshot_bytes,
        limits_.max_submaps_per_map, limits_.max_chunks_per_map,
        limits_.max_points_per_map, request.component_id);
  }
  catch (const nlohmann::json::exception& error)
  {
    throw RuntimeError(RuntimeErrorCode::InvalidRequest, error.what());
  }
  catch (const std::invalid_argument&)
  {
    throw;
  }
  catch (const std::exception& error)
  {
    throw RuntimeError(RuntimeErrorCode::Io, error.what());
  }

  std::lock_guard<std::mutex> publication_lock(publication_mutex_);
  std::unique_lock<std::mutex> operation_lock(operation_mutex_);
  std::shared_ptr<MolaSubmapBridge> provider;
  std::shared_ptr<NativePlannerAccumulator> previous_planner;
  std::shared_ptr<NativePlannerAccumulator> planner_state;
  std::optional<NativeGeometrySnapshot> prior;
  std::size_t old_points = 0;
  std::size_t old_units = 0;
  std::size_t resident_points = 0;
  {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    const auto found = contexts_.find(request.map_id);
    if (found == contexts_.end())
    {
      if (contexts_.size() >= limits_.max_maps)
        throw RuntimeError(RuntimeErrorCode::ResourceLimit, "runtime map count limit reached");
    }
    else
    {
      provider = found->second.provider;
      previous_planner = found->second.planner;
      old_units = found->second.resident_units;
      old_points = found->second.point_count;
      prior = provider->currentSnapshot();
    }
    resident_points = resident_points_;
  }
  if (prior)
  {
    if (parsed.graph_version.component_id != prior->graph_version.component_id)
      throw RuntimeError(RuntimeErrorCode::Conflict, "map_id changed component_id");
    if (std::tie(parsed.graph_version.epoch, parsed.graph_version.revision) <
        std::tie(prior->graph_version.epoch, prior->graph_version.revision))
      throw RuntimeError(RuntimeErrorCode::Stale, "stale graph solution");
    if (sameGraphVersion(parsed.graph_version, prior->graph_version))
    {
      const bool geometry_changed = parsed.identity.native_geometry_digest !=
                                    prior->identity.native_geometry_digest;
      if (!geometry_changed &&
          parsed.graph_version.digest != prior->graph_version.digest)
        throw RuntimeError(RuntimeErrorCode::Conflict, "graph revision was reused with different poses");
      if (geometry_changed && !commonPosesAgree(parsed, *prior))
        throw RuntimeError(
            RuntimeErrorCode::Conflict,
            "geometry changed at the same graph revision with conflicting common poses");
      if (parsed.identity.native_geometry_digest == prior->identity.native_geometry_digest &&
          parsed.identity.canonical_manifest_digest !=
              prior->identity.canonical_manifest_digest)
        throw RuntimeError(RuntimeErrorCode::Conflict, "manifest identity changed at the same graph and geometry revision");
    }
  }

  auto mode = request.mode;
  if (mode == ApplyMode::Auto)
    mode = prior &&
                   parsed.identity.native_geometry_digest ==
                       prior->identity.native_geometry_digest
               ? ApplyMode::PoseOnly
               : ApplyMode::Replace;
  if (mode == ApplyMode::PoseOnly && !prior)
    throw RuntimeError(RuntimeErrorCode::CacheMiss, "pose-only request has no resident geometry");
  if (mode == ApplyMode::PoseOnly &&
      parsed.identity.native_geometry_digest != prior->identity.native_geometry_digest)
    throw RuntimeError(RuntimeErrorCode::Conflict, "pose-only request changed native geometry identity");
  const auto incremental_prior =
      prior && prior->compaction_resolution_m > 0 && appendOnly(parsed, *prior)
          ? &*prior
          : nullptr;
  const auto reported_mode = mode;
  // Compaction merges coincident geometry across frames. A real correction
  // must recover the raw source, not move a lossy representative alone.
  if (mode == ApplyMode::PoseOnly && prior->compaction_resolution_m > 0 &&
      !incremental_prior)
    mode = ApplyMode::Replace;

  std::size_t candidate_points = mode == ApplyMode::Replace ? 0 : old_points;

  const bool resident = static_cast<bool>(provider);
  if (!provider) provider = std::make_shared<MolaSubmapBridge>();
  Artifact artifact;
  PlannerGridArtifact planner_artifact;
  bool planner_installed = false;
  MolaSubmapBridge::BeforeCommit writer;
  if (!request.output_path.empty() || !request.planner_output_path.empty())
    writer = [&](const NativeGeometrySnapshot& candidate) {
      if (!request.output_path.empty())
        serializeArtifact(
            candidate.geometry_map, request.output_path, limits_.max_output_bytes,
            artifact);
      if (!request.planner_output_path.empty())
      {
        std::shared_ptr<const NativePlannerGrid> planner;
        try
        {
          std::unordered_set<std::string> previous_ids;
          if (incremental_prior && previous_planner)
            for (const auto& frame : incremental_prior->keyframes)
              previous_ids.insert(frame.external_id);
          bool reuse = incremental_prior && previous_planner;
          for (const auto& frame : parsed.submaps)
            if (!previous_ids.count(frame.external_id) && reuse &&
                frame.observed_at_ns < previous_planner->newestStamp())
              reuse = false;
          planner_state = reuse
              ? std::make_shared<NativePlannerAccumulator>(*previous_planner)
              : std::make_shared<NativePlannerAccumulator>(plannerGridLimits());
          std::vector<const ParsedSubmap*> pending;
          for (const auto& frame : parsed.submaps)
            if (!reuse || !previous_ids.count(frame.external_id))
              pending.push_back(&frame);
          // Endpoints precede rays so later endpoint observations fence clearing.
          for (const auto* frame : pending)
            planner_state->endpoints(loadRawSubmap(
                *frame, request.chunks_dir, limits_.max_points_per_map));
          const auto ray_budget = pending.empty() ? 0 :
              plannerGridLimits().max_ray_steps / pending.size();
          for (const auto* frame : pending)
            planner_state->rays(loadRawSubmap(
                *frame, request.chunks_dir, limits_.max_points_per_map), ray_budget);
          const auto candidate_units = candidate.geometry_map->point_count() +
                                       planner_state->residentUnits();
          if (candidate_units > limits_.max_resident_points -
                                    std::min(limits_.max_resident_points, resident_points))
            throw std::runtime_error("candidate evidence exceeds resident budget");
          planner = planner_state->snapshot(candidate.graph_version, candidate.identity);
        }
        catch (const std::length_error& error)
        {
          throw RuntimeError(RuntimeErrorCode::ResourceLimit, error.what());
        }
        catch (const std::invalid_argument& error)
        {
          throw RuntimeError(RuntimeErrorCode::Internal, error.what());
        }
        catch (const std::runtime_error& error)
        {
          throw RuntimeError(RuntimeErrorCode::ResourceLimit, error.what());
        }
        try
        {
          planner_artifact = writeNativePlannerGrid(
              *planner, request.planner_output_path,
              limits_.max_planner_output_bytes, kMaxPlannerMetadataBytes,
              limits_.max_points_per_map);
        }
        catch (const std::invalid_argument& error)
        {
          throw RuntimeError(RuntimeErrorCode::Internal, error.what());
        }
        catch (const std::runtime_error& error)
        {
          throw RuntimeError(RuntimeErrorCode::Io, error.what());
        }
        planner_installed = true;
      }
    };
  MolaSubmapBridge::ApplyResult apply_result;
  try
  {
    if (mode == ApplyMode::Replace)
    {
      std::vector<SubmapInput> geometry;
      try
      {
        geometry = loadGeometry(
            parsed, request.chunks_dir, limits_.max_points_per_map,
            incremental_prior);
      }
      catch (const std::length_error& error)
      {
        throw RuntimeError(RuntimeErrorCode::ResourceLimit, error.what());
      }
      catch (const std::invalid_argument&)
      {
        throw;
      }
      catch (const std::exception& error)
      {
        throw RuntimeError(RuntimeErrorCode::Io, error.what());
      }
      for (const auto& submap : geometry)
        candidate_points += submap.points_local.size();
      provider->setCompactionResolution(0.05);
      if (candidate_points > limits_.max_resident_points -
                                 std::min(limits_.max_resident_points, resident_points))
        throw RuntimeError(RuntimeErrorCode::ResourceLimit, "candidate map exceeds resident point limit");
      apply_result = provider->replaceGeometrySnapshot(
          geometry, parsed.graph_version, parsed.canonical_metadata_json,
          parsed.identity, writer, false);
    }
    else
      apply_result = provider->applyPoseSolution(
          poseUpdates(parsed), parsed.graph_version, parsed.canonical_metadata_json,
          parsed.identity, writer, true, false);
  }
  catch (...)
  {
    if (artifact.installed) removeQuietly(request.output_path);
    if (planner_installed) removeQuietly(request.planner_output_path);
    throw;
  }

  const auto current = provider->currentSnapshot();
  if (!current)
  {
    if (artifact.installed) removeQuietly(request.output_path);
    if (planner_installed) removeQuietly(request.planner_output_path);
    throw RuntimeError(RuntimeErrorCode::Internal, "MOLA provider committed no snapshot");
  }
  {
    std::lock_guard<std::mutex> registry_lock(mutex_);
    const auto units = candidate_points +
                       (planner_state ? planner_state->residentUnits() : 0);
    if (!resident)
      contexts_.emplace(request.map_id, Context{provider, planner_state, candidate_points, units});
    else
    {
      auto& context = contexts_.at(request.map_id);
      context.point_count = candidate_points;
      context.planner = planner_state;
      context.resident_units = units;
    }
    resident_points_ = resident_points_ - old_units + units;
  }
  operation_lock.unlock();
  if (apply_result == MolaSubmapBridge::ApplyResult::Applied)
    provider->publishSnapshot(*current);

  return ApplyReport{
      request.request_id,
      request.map_id,
      reported_mode,
      apply_result == MolaSubmapBridge::ApplyResult::Duplicate
          ? "duplicate"
          : (reported_mode == ApplyMode::Replace ? "replaced" : "corrected"),
      *current,
      parsed.submaps.size(),
      current->geometry_map->point_count(),
      artifact.size,
      artifact.sha256,
      planner_artifact.size_bytes,
      planner_artifact.sha256};
}
catch (const RuntimeError&)
{
  throw;
}
catch (const std::invalid_argument& error)
{
  throw RuntimeError(RuntimeErrorCode::InvalidRequest, error.what());
}
catch (const std::out_of_range& error)
{
  throw RuntimeError(RuntimeErrorCode::Conflict, error.what());
}
catch (const std::filesystem::filesystem_error& error)
{
  throw RuntimeError(RuntimeErrorCode::Io, error.what());
}
catch (const std::exception& error)
{
  throw RuntimeError(RuntimeErrorCode::Mola, error.what());
}

bool PersistentMolaRuntime::release(const std::string& map_id)
{
  static const std::regex map_pattern{"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,511}$"};
  validateToken(map_id, "map_id", 512, map_pattern);
  std::lock_guard<std::mutex> operation_lock(operation_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = contexts_.find(map_id);
  if (found == contexts_.end()) return false;
  resident_points_ -= found->second.resident_units;
  contexts_.erase(found);
  return true;
}

std::shared_ptr<MolaSubmapBridge> PersistentMolaRuntime::provider(
    const std::string& map_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = contexts_.find(map_id);
  return found == contexts_.end() ? nullptr : found->second.provider;
}

std::optional<NativeGeometrySnapshot> PersistentMolaRuntime::currentSnapshot(
    const std::string& map_id) const
{
  const auto source = provider(map_id);
  return source ? source->currentSnapshot() : std::nullopt;
}

std::vector<std::string> PersistentMolaRuntime::mapIds() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> result;
  result.reserve(contexts_.size());
  for (const auto& item : contexts_) result.push_back(item.first);
  std::sort(result.begin(), result.end());
  return result;
}

std::string applyModeName(const ApplyMode mode)
{
  switch (mode)
  {
    case ApplyMode::Auto:
      return "auto";
    case ApplyMode::Replace:
      return "replace";
    case ApplyMode::PoseOnly:
      return "pose_only";
  }
  return "auto";
}

std::string runtimeErrorCodeName(const RuntimeErrorCode code)
{
  switch (code)
  {
    case RuntimeErrorCode::InvalidRequest:
      return "invalid_request";
    case RuntimeErrorCode::CacheMiss:
      return "cache_miss";
    case RuntimeErrorCode::Stale:
      return "stale";
    case RuntimeErrorCode::Conflict:
      return "conflict";
    case RuntimeErrorCode::ResourceLimit:
      return "resource_limit";
    case RuntimeErrorCode::Io:
      return "io";
    case RuntimeErrorCode::Mola:
      return "mola";
    case RuntimeErrorCode::Internal:
      return "internal";
  }
  return "internal";
}
}  // namespace swarmdeck_mapping
