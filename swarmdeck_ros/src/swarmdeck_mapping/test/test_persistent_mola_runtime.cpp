#include <swarmdeck_mapping/persistent_mola_runtime.hpp>
#include <swarmdeck_mapping/snapshot_io.hpp>

#include <openssl/evp.h>
#include <nlohmann/json.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using json = nlohmann::json;

void require(const bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

std::string sha256(const std::string& bytes)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  require(
      EVP_Digest(bytes.data(), bytes.size(), digest.data(), &length, EVP_sha256(), nullptr) == 1 &&
          length == 32,
      "SHA-256 failed");
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index)
    text << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return text.str();
}

json pose(const double x)
{
  return json::array(
      {json::array({1, 0, 0, x}), json::array({0, 1, 0, 0}),
       json::array({0, 0, 1, 0}), json::array({0, 0, 0, 1})});
}

std::vector<std::uint8_t> xyzChunk(const float x)
{
  std::vector<std::uint8_t> bytes{'S', 'D', 'X', 'Y', 'Z', '1', 0, 0,
                                  1,   0,   0,   0,   0,   0,   0, 0};
  for (const float value : {x, 0.0F, 0.0F})
  {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (std::size_t shift = 0; shift < 32; shift += 8)
      bytes.push_back(static_cast<std::uint8_t>(bits >> shift));
  }
  return bytes;
}

std::string writeChunk(const std::filesystem::path& directory, const float x)
{
  const auto bytes = xyzChunk(x);
  const auto digest = sha256(std::string(
      reinterpret_cast<const char*>(bytes.data()), bytes.size()));
  std::ofstream output(directory / digest, std::ios::binary);
  output.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  return digest;
}

std::string writeSnapshot(
    const std::filesystem::path& path, const std::string& chunk_digest,
    const std::uint64_t revision, const double x, const char snapshot_digit)
{
  const json chunk{
      {"sha256", chunk_digest},
      {"encoding", "application/vnd.swarmdeck.xyz-f32.v1"},
      {"size_bytes", 28},
      {"bounds", json::array({json::array({0, 0, 0}), json::array({1, 1, 1})})},
      {"point_count", 1}};
  const json submap_id{
      {"robot_id", "robot"}, {"session_id", "session"}, {"seq", 0}};
  const std::string stable_id = "robot/session/submap/0";
  const json geometry_members = json::array(
      {json::array({stable_id, 0, json::array({chunk_digest})})});
  const auto geometry_revision = sha256(geometry_members.dump());
  const json graph{
      {"component_id", "component:test"}, {"epoch", 1}, {"revision", revision}};
  const json submap{
      {"submap_id", submap_id},
      {"geometry_revision", 0},
      {"pose_revision", graph},
      {"T_component_submap", pose(x)},
      {"keyframes", json::array({"robot/session/keyframe/0"})},
      {"chunks", json::array({chunk})},
      {"bounds", json::array({json::array({0, 0, 0}), json::array({1, 1, 1})})},
      {"resolution_m", 0.2},
      {"observed_at_ns", 100},
      {"sensor_origins", json::array({json::array({0, 0, 0})})}};
  const json manifest{
      {"map_id", "onboard"},
      {"layer_id", "persistent_geometry"},
      {"frame_id", "component_test"},
      {"graph_revision", graph},
      {"geometry_revision", geometry_revision},
      {"submaps", json::array({submap})},
      {"chunks", json::array({chunk})},
      {"tombstones", json::array()}};
  const json snapshot{
      {"schema", "swarmdeck.autonomy.v1"},
      {"snapshot_id", std::string(64, snapshot_digit)},
      {"generated_at_ns", 100},
      {"manifests", json::array({manifest})}};
  std::ofstream output(path, std::ios::binary);
  output << snapshot.dump();
  output.close();
  return swarmdeck_mapping::boundedFileSha256(path);
}

std::string writeEmptySnapshot(
    const std::filesystem::path& path, const std::uint64_t revision,
    const char snapshot_digit)
{
  const json manifest{
      {"map_id", "onboard"},
      {"layer_id", "persistent_geometry"},
      {"frame_id", "component_test"},
      {"graph_revision",
       {{"component_id", "component:test"}, {"epoch", 1}, {"revision", revision}}},
      {"geometry_revision", sha256(json::array().dump())},
      {"submaps", json::array()},
      {"chunks", json::array()},
      {"tombstones", json::array({"robot/session/submap/0:retracted"})}};
  const json snapshot{
      {"schema", "swarmdeck.autonomy.v1"},
      {"snapshot_id", std::string(64, snapshot_digit)},
      {"generated_at_ns", 100},
      {"manifests", json::array({manifest})}};
  std::ofstream output(path, std::ios::binary);
  output << snapshot.dump();
  output.close();
  return swarmdeck_mapping::boundedFileSha256(path);
}

json manifestFrom(const std::filesystem::path& path)
{
  std::ifstream input(path);
  json snapshot;
  input >> snapshot;
  return snapshot.at("manifests").front();
}

std::string writePeerSnapshot(
    const std::filesystem::path& path, const std::vector<json>& manifests,
    const std::uint64_t generated_at_ns)
{
  json canonical = json::array();
  json nested = json::array();
  for (auto manifest : manifests)
  {
    manifest.erase("schema");
    nested.push_back(manifest);
    manifest["schema"] = "swarmdeck.autonomy.v1";
    canonical.push_back(std::move(manifest));
  }
  const json snapshot{
      {"schema", "swarmdeck.autonomy.v1"},
      {"snapshot_id", sha256(canonical.dump())},
      {"generated_at_ns", generated_at_ns},
      {"manifests", nested}};
  std::ofstream output(path, std::ios::binary);
  output << snapshot.dump();
  output.close();
  return swarmdeck_mapping::boundedFileSha256(path);
}
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
                    ("swarmdeck-mola-runtime-" + std::to_string(
                         std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(root / "chunks");
  try
  {
    const auto chunk0 = writeChunk(root / "chunks", 0.0F);
    const auto chunk1 = writeChunk(root / "chunks", 1.0F);
    swarmdeck_mapping::PersistentMolaRuntime runtime;

    const auto source0 = writeSnapshot(root / "snapshot0.json", chunk0, 0, 0, 'a');
    const auto first = runtime.apply(
        {"request-0", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
         root / "snapshot0.json", source0, root / "chunks", root / "map0.metricmap"});
    require(first.result == "replaced", "initial geometry was not replaced");
    require(first.point_count == 1, "initial point count is wrong");
    const auto held = first.snapshot.geometry_map;
    auto provider = runtime.provider("peer/component");
    require(static_cast<bool>(provider), "runtime exposes no MOLA provider");
    std::size_t publications = 0;
    provider->subscribeToMapUpdates(
        [&publications, &runtime](const mola::MapSourceBase::MapUpdate& update) {
          require(
              static_cast<bool>(runtime.currentSnapshot("peer/component")),
              "MapSource callback deadlocked or could not inspect runtime snapshot");
          require(update.reference_frame == "component_test", "source frame_id was changed");
          ++publications;
        });
    // MOLA immediately replays the cached last map to a late subscriber.
    publications = 0;

    // Removing the immutable chunk proves a pose-only correction neither reads
    // nor reinserts resident geometry.
    std::filesystem::remove(root / "chunks" / chunk0);
    const auto source1 = writeSnapshot(root / "snapshot1.json", chunk0, 1, 5, 'b');
    const auto corrected = runtime.apply(
        {"request-1", "peer/component", swarmdeck_mapping::ApplyMode::PoseOnly,
         root / "snapshot1.json", source1, root / "chunks", root / "map1.metricmap"});
    require(corrected.result == "corrected", "pose-only correction was not applied");
    require(corrected.point_count == 1, "pose correction changed geometry count");
    require(
        corrected.snapshot.geometry_map->keyframePoses().at(0).x() == 5,
        "corrected pose is absent");
    require(held->keyframePoses().at(0).x() == 0, "held snapshot was mutated");
    require(publications == 1, "correction did not publish once");

    // A geometry replacement at the same graph revision is legal when the
    // pose solution agrees. It coherently drops the prior keyframe contents.
    const auto source2 = writeSnapshot(root / "snapshot2.json", chunk1, 1, 5, 'c');
    const auto replaced = runtime.apply(
        {"request-2", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
         root / "snapshot2.json", source2, root / "chunks", root / "map2.metricmap"});
    require(replaced.result == "replaced", "same-graph geometry replacement failed");
    require(publications == 2, "replacement did not publish once");

    const auto conflict_source =
        writeSnapshot(root / "conflict.json", chunk1, 1, 6, 'f');
    bool rejected = false;
    try
    {
      runtime.apply(
          {"request-conflict", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
           root / "conflict.json", conflict_source, root / "chunks",
           root / "conflict.metricmap"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      rejected = error.code() == swarmdeck_mapping::RuntimeErrorCode::Conflict;
    }
    require(rejected, "same-graph conflicting common pose was accepted");
    require(publications == 2, "conflicting common pose published a map");

    const auto empty_source = writeEmptySnapshot(root / "empty.json", 1, '9');
    const auto retracted = runtime.apply(
        {"request-retract", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
         root / "empty.json", empty_source, root / "chunks", root / "empty.metricmap"});
    require(retracted.point_count == 0, "same-graph retraction retained geometry");
    require(retracted.snapshot.submap_poses.empty(), "retraction retained membership");
    const auto readded = runtime.apply(
        {"request-readd", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
         root / "snapshot2.json", source2, root / "chunks", root / "map2b.metricmap"});
    require(readded.point_count == 1, "same-graph addition did not restore geometry");
    require(publications == 4, "same-graph membership replacements did not publish");

    rejected = false;
    const auto stale_source = writeSnapshot(root / "stale.json", chunk1, 0, 0, 'd');
    try
    {
      runtime.apply(
          {"request-stale", "peer/component", swarmdeck_mapping::ApplyMode::PoseOnly,
           root / "stale.json", stale_source, root / "chunks", root / "stale.metricmap"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      rejected = error.code() == swarmdeck_mapping::RuntimeErrorCode::Stale;
    }
    require(rejected, "stale request was not classified and rejected");
    require(publications == 4, "stale request published a map");

    // Artifact failure happens in the bridge's pre-commit hook. The revision
    // remains available for a clean retry and no success publication leaks.
    const auto source3 = writeSnapshot(root / "snapshot3.json", chunk1, 2, 8, 'e');
    std::ofstream(root / "occupied.metricmap") << "occupied";
    rejected = false;
    try
    {
      runtime.apply(
          {"request-fail", "peer/component", swarmdeck_mapping::ApplyMode::PoseOnly,
           root / "snapshot3.json", source3, root / "chunks", root / "occupied.metricmap"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      rejected = error.code() == swarmdeck_mapping::RuntimeErrorCode::Io;
    }
    require(rejected, "artifact failure was not rejected");
    require(runtime.currentSnapshot("peer/component")->graph_version.revision == 1,
            "failed request consumed the graph revision");
    require(publications == 4, "failed request published success");
    std::filesystem::remove(root / "occupied.metricmap");
    const auto recovered = runtime.apply(
        {"request-retry", "peer/component", swarmdeck_mapping::ApplyMode::PoseOnly,
         root / "snapshot3.json", source3, root / "chunks", root / "map3.metricmap"});
    require(recovered.result == "corrected", "runtime did not recover after failure");
    require(publications == 5, "recovered request did not publish once");

    require(runtime.release("peer/component"), "resident map was not released");
    require(!runtime.currentSnapshot("peer/component"), "released map remains visible");

    // A subscriber may invalidate its context synchronously. Publication does
    // not hold either the registry or mutation mutex.
    runtime.apply(
        {"request-callback-base", "peer/release", swarmdeck_mapping::ApplyMode::Replace,
         root / "snapshot2.json", source2, root / "chunks", {}});
    auto releasing_provider = runtime.provider("peer/release");
    bool callback_released = false;
    std::size_t release_callbacks = 0;
    releasing_provider->subscribeToMapUpdates(
        [&runtime, &callback_released, &release_callbacks](
            const mola::MapSourceBase::MapUpdate&) {
          if (++release_callbacks > 1)
            callback_released = runtime.release("peer/release");
        });
    runtime.apply(
        {"request-callback-update", "peer/release", swarmdeck_mapping::ApplyMode::PoseOnly,
         root / "snapshot3.json", source3, root / "chunks", {}});
    require(callback_released, "MapSource callback could not release its runtime context");
    require(!runtime.provider("peer/release"), "callback-released context remains registered");

    // An embedded owner may select its component from a whole-peer snapshot.
    // A source change confined to another component is a semantic duplicate
    // and must not read the selected component's resident chunks again.
    auto selected_manifest = manifestFrom(root / "snapshot2.json");
    auto unrelated_manifest = selected_manifest;
    unrelated_manifest["graph_revision"]["component_id"] = "component:other";
    unrelated_manifest["frame_id"] = "component_other";
    unrelated_manifest["submaps"][0]["pose_revision"]["component_id"] =
        "component:other";
    const auto peer0_sha = writePeerSnapshot(
        root / "peer0.json", {selected_manifest, unrelated_manifest}, 200);
    swarmdeck_mapping::PersistentMolaRuntime selection_runtime;
    const auto selected = selection_runtime.apply(
        {"request-select", "whole-peer/component", swarmdeck_mapping::ApplyMode::Auto,
         root / "peer0.json", peer0_sha, root / "chunks", {}, "component:test"});
    require(selected.result == "replaced", "whole-peer component was not selected");
    const auto first_source_id = selected.snapshot.identity.source_snapshot_id;

    unrelated_manifest["graph_revision"]["revision"] = 2;
    unrelated_manifest["submaps"][0]["pose_revision"]["revision"] = 2;
    const auto peer1_sha = writePeerSnapshot(
        root / "peer1.json", {selected_manifest, unrelated_manifest}, 300);
    std::filesystem::remove(root / "chunks" / chunk1);
    const auto unchanged = selection_runtime.apply(
        {"request-unrelated", "whole-peer/component", swarmdeck_mapping::ApplyMode::Auto,
         root / "peer1.json", peer1_sha, root / "chunks", {}, "component:test"});
    require(unchanged.result == "duplicate", "unrelated component rebuilt selected geometry");
    require(
        unchanged.snapshot.identity.source_snapshot_id != first_source_id,
        "whole-source snapshot identity was not refreshed");
    require(
        unchanged.snapshot.identity.source_sha256 == peer1_sha,
        "whole-source SHA was not retained exactly");

    bool selection_rejected = false;
    try
    {
      selection_runtime.apply(
          {"request-missing", "whole-peer/missing", swarmdeck_mapping::ApplyMode::Auto,
           root / "peer1.json", peer1_sha, root / "chunks", {}, "component:missing"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      selection_rejected =
          error.code() == swarmdeck_mapping::RuntimeErrorCode::InvalidRequest;
    }
    require(selection_rejected, "missing component selector was accepted");

    const auto duplicate_sha = writePeerSnapshot(
        root / "duplicate.json", {selected_manifest, selected_manifest}, 400);
    selection_rejected = false;
    try
    {
      selection_runtime.apply(
          {"request-duplicate", "whole-peer/duplicate", swarmdeck_mapping::ApplyMode::Auto,
           root / "duplicate.json", duplicate_sha, root / "chunks", {},
           "component:test"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      selection_rejected =
          error.code() == swarmdeck_mapping::RuntimeErrorCode::InvalidRequest;
    }
    require(selection_rejected, "duplicate selected component was accepted");
    std::filesystem::remove_all(root);
    return 0;
  }
  catch (...)
  {
    std::filesystem::remove_all(root);
    throw;
  }
}
