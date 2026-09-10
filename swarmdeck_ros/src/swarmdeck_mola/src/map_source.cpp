#include <swarmdeck_mola/map_source.hpp>
#include <swarmdeck_mapping/persistent_mola_runtime.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>

IMPLEMENTS_MRPT_OBJECT(SwarmDeckMapSource, mola::ExecutableBase, swarmdeck_mola)

namespace
{
[[maybe_unused]] const bool registered = [] {
  MOLA_REGISTER_MODULE(swarmdeck_mola::SwarmDeckMapSource);
  return true;
}();

using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

std::filesystem::path requiredPath(const mola::Yaml& params, const std::string& key)
{
  const auto path = params[key].as<std::string>();
  if (path.empty() || path.size() > 4096 || path.find('\0') != std::string::npos)
    throw std::invalid_argument("invalid " + key);
  return std::filesystem::absolute(path).lexically_normal();
}
}  // namespace

namespace swarmdeck_mola
{
struct SwarmDeckMapSource::State
{
  std::unique_ptr<swarmdeck_mapping::PersistentMolaRuntime> runtime;
  std::filesystem::path source, chunks, artifact;
  std::string successful_sha, failed_sha, last_error;
  std::string component_id, active_map_id;
  std::string frame{"map"};
  bool ready{false};
  unsigned failures{0};
  double poll_s{1.0};
  Clock::time_point next_poll{}, retry_after{};
};

SwarmDeckMapSource::SwarmDeckMapSource() : state_(std::make_unique<State>()) {}
SwarmDeckMapSource::~SwarmDeckMapSource() = default;

void SwarmDeckMapSource::initialize(const mola::Yaml& config)
{
  if (state_->runtime) throw std::logic_error("map source is already initialized");
  const auto params = config.has("params") ? config["params"] : config;
  auto next = std::make_unique<State>();
  next->source = requiredPath(params, "snapshot_file");
  next->chunks = requiredPath(params, "chunks_dir");
  next->component_id = params.getOrDefault<std::string>("component_id", "");
  if (next->component_id.size() > 128 || next->component_id.find('\0') != std::string::npos)
    throw std::invalid_argument("invalid component_id");
  if (params.has("artifact_file") && !params["artifact_file"].as<std::string>().empty())
    next->artifact = requiredPath(params, "artifact_file");
  if (next->artifact == next->source)
    throw std::invalid_argument("artifact_file must differ from snapshot_file");
  next->poll_s = params.getOrDefault<double>("poll_s", 1.0);
  if (!std::isfinite(next->poll_s) || next->poll_s < 0.05 || next->poll_s > 60.0)
    throw std::invalid_argument("poll_s must be in [0.05, 60]");
  swarmdeck_mapping::RuntimeLimits limits;
  // Keep the previous component until a coherent replacement is ready.
  limits.max_maps = 2;
  const auto points = params.getOrDefault<int>("max_points", 2'000'000);
  if (points < 1 || points > 20'000'000)
    throw std::invalid_argument("max_points must be in [1, 20000000]");
  limits.max_points_per_map = static_cast<std::size_t>(points);
  // Include both the published map and a correction/replacement candidate.
  limits.max_resident_points = 2 * limits.max_points_per_map;
  next->runtime = std::make_unique<swarmdeck_mapping::PersistentMolaRuntime>(limits);
  if (!next->artifact.empty())
    std::filesystem::create_directories(next->artifact.parent_path());
  state_ = std::move(next);
}

void SwarmDeckMapSource::unavailable(const std::string& reason)
{
  auto& state = *state_;
  if (!state.ready && state.last_error == reason) return;
  state.ready = false;
  state.last_error = reason.substr(0, 1000);
  mola::MapSourceBase::MapUpdate update;
  update.timestamp = mrpt::Clock::now();
  update.reference_frame = state.frame;
  update.method = "swarm_slam";
  update.map_name = "swarmdeck_" + getModuleInstanceName();
  update.keep_last_one_only = true;
  update.map = std::make_shared<mola::KeyframePointCloudMap>();
  update.map_metadata = json{{"available", false}, {"detail", state.last_error}}.dump();
  advertiseUpdatedMap(update);
  if (reason != "map source stopped")
    MRPT_LOG_WARN_STREAM("SwarmDeck map unavailable: " << state.last_error);
}

void SwarmDeckMapSource::spinOnce()
{
  auto& state = *state_;
  if (!state.runtime) return;
  const auto now = Clock::now();
  if (now < state.next_poll) return;
  state.next_poll = now + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(state.poll_s));
  std::string sha;
  std::string candidate_map_id;
  std::filesystem::path pending_artifact;
  try
  {
    sha = swarmdeck_mapping::boundedFileSha256(
        state.source, state.runtime->limits().max_snapshot_bytes);
    if (state.ready && sha == state.successful_sha) return;
    if (sha == state.failed_sha && now < state.retry_after) return;

    const auto limits = state.runtime->limits();
    const auto selected = swarmdeck_mapping::parseComponentSnapshot(
        state.source, sha, limits.max_snapshot_bytes, limits.max_submaps_per_map,
        limits.max_chunks_per_map, limits.max_points_per_map, state.component_id);
    candidate_map_id = "source:" + selected.graph_version.component_id;

    swarmdeck_mapping::ApplyRequest request;
    request.request_id = sha;
    request.map_id = candidate_map_id;
    request.mode = swarmdeck_mapping::ApplyMode::Auto;
    request.snapshot_path = state.source;
    request.snapshot_sha256 = sha;
    request.chunks_dir = state.chunks;
    request.component_id = state.component_id;
    if (!state.artifact.empty())
    {
      pending_artifact = state.artifact.string() + ".pending." + sha + "." +
                         std::to_string(Clock::now().time_since_epoch().count());
      request.output_path = pending_artifact;
    }
    const auto report = state.runtime->apply(request);
    // The native runtime validates the exact bytes it consumed. Only expose
    // its map to framework consumers if that source is still current.
    if (swarmdeck_mapping::boundedFileSha256(
            state.source, state.runtime->limits().max_snapshot_bytes) != sha)
    {
      // A discarded source must not become the base of a subsequent correction.
      state.runtime->release(candidate_map_id);
      throw std::runtime_error("snapshot superseded during import");
    }
    if (!pending_artifact.empty())
    {
      std::filesystem::rename(pending_artifact, state.artifact);
      pending_artifact.clear();
    }

    mola::MapSourceBase::MapUpdate update;
    const auto& snapshot = report.snapshot;
    state.frame = snapshot.identity.reference_frame;
    update.timestamp = mrpt::Clock::now();
    update.reference_frame = state.frame;
    update.method = "swarm_slam";
    update.map_name = "swarmdeck_" + getModuleInstanceName();
    update.keep_last_one_only = true;
    update.map = snapshot.geometry_map;
    update.map_metadata = json{
        {"available", true}, {"source_sha256", sha},
        {"source_snapshot_id", snapshot.identity.source_snapshot_id},
        {"manifest", json::parse(snapshot.canonical_metadata_json)}}.dump();
    advertiseUpdatedMap(update);
    if (!state.active_map_id.empty() && state.active_map_id != candidate_map_id)
      state.runtime->release(state.active_map_id);
    state.active_map_id = candidate_map_id;
    state.successful_sha = sha;
    state.failed_sha.clear();
    state.last_error.clear();
    state.failures = 0;
    state.ready = true;
  }
  catch (const std::exception& error)
  {
    if (!candidate_map_id.empty() && candidate_map_id != state.active_map_id &&
        state.runtime->provider(candidate_map_id))
      state.runtime->release(candidate_map_id);
    if (!pending_artifact.empty())
    {
      std::error_code ignored;
      std::filesystem::remove(pending_artifact, ignored);
    }
    state.failures = sha == state.failed_sha ? std::min(6U, state.failures + 1) : 1;
    state.failed_sha = sha;
    const auto backoff = std::min(30U, 1U << (state.failures - 1));
    state.retry_after = Clock::now() + std::chrono::seconds(backoff);
    unavailable(error.what());
  }
}

void SwarmDeckMapSource::onQuit()
{
  if (!state_->runtime) return;
  unavailable("map source stopped");
  state_->runtime.reset();
}
}  // namespace swarmdeck_mola
