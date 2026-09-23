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
#include <utility>
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

// A chunk of `count` floor points along a 41 m line from `x_offset`; the
// line keeps the voxel count small so only the point budget is exercised.
std::string writeChunkPoints(
    const std::filesystem::path& directory, const std::size_t count,
    const float x_offset)
{
  std::vector<std::uint8_t> bytes{'S', 'D', 'X', 'Y', 'Z', '1', 0, 0};
  for (std::size_t shift = 0; shift < 64; shift += 8)
    bytes.push_back(static_cast<std::uint8_t>(
        (static_cast<std::uint64_t>(count) >> shift) & 0xffU));
  bytes.reserve(16 + 12 * count);
  for (std::size_t index = 0; index < count; ++index)
  {
    const float x = x_offset + 0.01F * static_cast<float>(index % 4096);
    for (const float value : {x, 0.0F, 0.0F})
    {
      std::uint32_t bits = 0;
      std::memcpy(&bits, &value, sizeof(bits));
      for (std::size_t shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(bits >> shift));
    }
  }
  const auto digest = sha256(std::string(
      reinterpret_cast<const char*>(bytes.data()), bytes.size()));
  std::ofstream output(directory / digest, std::ios::binary);
  output.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  return digest;
}

// One submap whose chunks are `(digest, point_count)` pairs.
std::string writeChunkedSnapshot(
    const std::filesystem::path& path,
    const std::vector<std::pair<std::string, std::size_t>>& chunks,
    const char snapshot_digit)
{
  json chunk_list = json::array();
  json digests = json::array();
  for (const auto& [digest, count] : chunks)
  {
    chunk_list.push_back(
        {{"sha256", digest},
         {"encoding", "application/vnd.swarmdeck.xyz-f32.v1"},
         {"size_bytes", 16 + 12 * count},
         {"bounds", json::array({json::array({0, 0, 0}), json::array({100, 1, 1})})},
         {"point_count", count}});
    digests.push_back(digest);
  }
  const std::string stable_id = "robot/session/submap/0";
  const json geometry_members =
      json::array({json::array({stable_id, 0, digests})});
  const json graph{{"component_id", "component:test"}, {"epoch", 1}, {"revision", 0}};
  const json submap{
      {"submap_id", {{"robot_id", "robot"}, {"session_id", "session"}, {"seq", 0}}},
      {"geometry_revision", 0},
      {"pose_revision", graph},
      {"T_component_submap", pose(0)},
      {"keyframes", json::array({"robot/session/keyframe/0"})},
      {"chunks", chunk_list},
      {"bounds", json::array({json::array({0, 0, 0}), json::array({100, 1, 1})})},
      {"resolution_m", 0.2},
      {"observed_at_ns", 100},
      {"sensor_origins", json::array({json::array({0, 0, 0.5})})},
      {"ray_evidence",
       {{"return_semantics", "first_return"},
        {"deskew", "deskewed"},
        {"origin_association", "single_capture"}}}};
  const json manifest{
      {"map_id", "onboard"},
      {"layer_id", "persistent_geometry"},
      {"frame_id", "component_test"},
      {"graph_revision", graph},
      {"geometry_revision", sha256(geometry_members.dump())},
      {"submaps", json::array({submap})},
      {"chunks", chunk_list},
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
      {"sensor_origins", json::array({json::array({0, 0, 0})})},
      {"ray_evidence",
       {{"return_semantics", "first_return"},
        {"deskew", "deskewed"},
        {"origin_association", "single_capture"}}}};
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
    (void)writeSnapshot(root / "partial-evidence.json", chunk1, 0, 0, '8');
    json partial_evidence;
    std::ifstream(root / "partial-evidence.json") >> partial_evidence;
    partial_evidence["manifests"][0]["submaps"][0]["ray_evidence"] =
        {{"return_semantics", "first_return"}};
    {
      std::ofstream output(root / "partial-evidence.json", std::ios::binary);
      output << partial_evidence.dump();
    }
    const auto partial_sha = swarmdeck_mapping::boundedFileSha256(
        root / "partial-evidence.json");
    const auto partial = swarmdeck_mapping::parseComponentSnapshot(
        root / "partial-evidence.json", partial_sha);
    require(!partial.submaps.front().ray_evidence_qualified,
            "partial ray evidence was treated as qualified");
    swarmdeck_mapping::MolaSubmapBridge partial_bridge;
    partial_bridge.replaceGeometrySnapshot(
        swarmdeck_mapping::loadGeometry(partial, root / "chunks"),
        partial.graph_version, partial.canonical_metadata_json, partial.identity);
    const auto partial_grid = swarmdeck_mapping::buildNativePlannerGrid(
        *partial_bridge.currentSnapshot());
    require(!partial_grid->occupied.empty() && partial_grid->free.empty(),
            "partial ray evidence did not remain occupied-only");
    partial_evidence["manifests"][0]["submaps"][0]["ray_evidence"]["deskew"] = "not_required";
    partial_evidence["manifests"][0]["submaps"][0]["ray_evidence"]["origin_association"] = "single_capture";
    {
      std::ofstream output(root / "instantaneous.json", std::ios::binary);
      output << partial_evidence.dump();
    }
    const auto instantaneous = swarmdeck_mapping::parseComponentSnapshot(
        root / "instantaneous.json",
        swarmdeck_mapping::boundedFileSha256(root / "instantaneous.json"));
    require(instantaneous.submaps.front().ray_evidence_qualified,
            "instantaneous first-return capture was not qualified");
    swarmdeck_mapping::MolaSubmapBridge instantaneous_bridge;
    instantaneous_bridge.replaceGeometrySnapshot(
        swarmdeck_mapping::loadGeometry(instantaneous, root / "chunks"),
        instantaneous.graph_version, instantaneous.canonical_metadata_json,
        instantaneous.identity);
    require(!swarmdeck_mapping::buildNativePlannerGrid(
                 *instantaneous_bridge.currentSnapshot())->free.empty(),
            "instantaneous first-return capture produced no measured free space");
    const auto first = runtime.apply(
        {"request-0", "peer/component", swarmdeck_mapping::ApplyMode::Replace,
         root / "snapshot0.json", source0, root / "chunks", root / "map0.metricmap",
         {}, root / "map0.sdmgrid"});
    require(first.result == "replaced", "initial geometry was not replaced");
    require(first.point_count == 1, "initial point count is wrong");
    require(first.planner_output_size_bytes > 0 &&
                first.planner_output_sha256.size() == 64 &&
                std::filesystem::exists(root / "map0.sdmgrid"),
            "initial planner product was not installed");
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

    // Corrections rebuild from immutable measurements: a compacted representative
    // cannot recover endpoints previously hidden by an overlapping keyframe.
    const auto source1 = writeSnapshot(root / "snapshot1.json", chunk0, 1, 5, 'b');
    const auto corrected = runtime.apply(
        {"request-1", "peer/component", swarmdeck_mapping::ApplyMode::PoseOnly,
         root / "snapshot1.json", source1, root / "chunks", root / "map1.metricmap",
         {}, root / "map1.sdmgrid"});
    require(corrected.point_count == 1, "pose correction changed geometry count");
    require(corrected.planner_output_size_bytes > 0,
            "pose-only correction did not export resident planner geometry");
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

    swarmdeck_mapping::PersistentMolaRuntime incremental;
    incremental.apply(
        {"append-base", "peer/append", swarmdeck_mapping::ApplyMode::Auto,
         root / "snapshot0.json", source0, root / "chunks", {}, {},
         root / "append-base.sdmgrid"});
    auto appended_manifest = manifestFrom(root / "snapshot0.json");
    auto added = appended_manifest["submaps"][0];
    added["submap_id"]["seq"] = 1;
    added["chunks"] = manifestFrom(root / "snapshot2.json")["submaps"][0]["chunks"];
    added["observed_at_ns"] = 101;
    appended_manifest["submaps"].push_back(added);
    appended_manifest["chunks"].push_back(added["chunks"][0]);
    appended_manifest["geometry_revision"] = sha256(json::array({
        json::array({"robot/session/submap/0", 0, json::array({chunk0})}),
        json::array({"robot/session/submap/1", 0, json::array({chunk1})})}).dump());
    const auto append_sha = writePeerSnapshot(
        root / "append.json", {appended_manifest}, 101);
    std::filesystem::remove(root / "chunks" / chunk0);
    (void)writeChunk(root / "chunks", 1.0F);
    const auto appended = incremental.apply(
        {"append-new", "peer/append", swarmdeck_mapping::ApplyMode::Auto,
         root / "append.json", append_sha, root / "chunks", {}, {},
         root / "append-new.sdmgrid"});
    require(appended.point_count == 2 &&
                appended.snapshot.keyframes.size() == 2 &&
                std::filesystem::exists(root / "append-new.sdmgrid"),
            "append reread historical chunks or lost newly measured geometry");

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

    // The runtime's point budget is the planner grid's budget. Until
    // 2026-09-19 the grid build used its own 1,000,000 default whatever
    // `--max-points-per-map` said, so a component between the two budgets
    // loaded, serialized its metric map, and then failed the planner grid on
    // every build (benchbot mission 1a8cc114, robot_0 at 1,222,612 points).
    static_assert(
        swarmdeck_mapping::RuntimeLimits{}.max_points_per_map ==
                swarmdeck_mapping::kMaxPointsPerMap &&
            swarmdeck_mapping::PlannerGridLimits{}.max_points ==
                swarmdeck_mapping::kMaxPointsPerMap,
        "the loader and the planner grid must default to one point budget");
    const auto big_a = writeChunkPoints(root / "chunks", 550'000, 0.0F);
    const auto big_b = writeChunkPoints(root / "chunks", 550'000, 50.0F);
    const auto big_sha = writeChunkedSnapshot(
        root / "big.json", {{big_a, 550'000}, {big_b, 550'000}}, '3');
    swarmdeck_mapping::RuntimeLimits generous;
    generous.max_points_per_map = 1'200'000;
    generous.max_resident_points = 2'400'000;
    swarmdeck_mapping::PersistentMolaRuntime generous_runtime(generous);
    require(
        generous_runtime.plannerGridLimits().max_points == 1'200'000,
        "the planner grid budget does not follow max_points_per_map");
    const auto big = generous_runtime.apply(
        {"request-big", "peer/big", swarmdeck_mapping::ApplyMode::Replace,
         root / "big.json", big_sha, root / "chunks", root / "big.metricmap", {},
         root / "big.sdmgrid"});
    require(
        big.point_count < 5000 &&
            std::filesystem::exists(root / "big.sdmgrid"),
        "overlapping source history did not produce a bounded native product");
    {
      std::ifstream product(root / "big.sdmgrid", std::ios::binary);
      product.seekg(8);
      std::array<unsigned char, 4> length{};
      product.read(reinterpret_cast<char*>(length.data()), length.size());
      const std::uint32_t size = length[0] | (length[1] << 8) |
                                 (length[2] << 16) | (length[3] << 24);
      std::string metadata(size, '\0');
      product.read(metadata.data(), metadata.size());
      const auto header = json::parse(metadata);
      require(header.at("source_point_count") == 1'100'000 &&
                  header.at("point_count").get<std::size_t>() < 5000,
              "compacted planner product lost exact source provenance");
    }
    swarmdeck_mapping::RuntimeLimits strict;
    strict.max_points_per_map = 1'000'000;
    swarmdeck_mapping::PersistentMolaRuntime strict_runtime(strict);
    bool resource_refused = false;
    try
    {
      strict_runtime.apply(
          {"request-strict", "peer/strict", swarmdeck_mapping::ApplyMode::Replace,
           root / "big.json", big_sha, root / "chunks", root / "strict.metricmap",
           {}, root / "strict.sdmgrid"});
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      resource_refused =
          error.code() == swarmdeck_mapping::RuntimeErrorCode::ResourceLimit;
    }
    require(
        resource_refused &&
            !std::filesystem::exists(root / "strict.metricmap") &&
            !std::filesystem::exists(root / "strict.sdmgrid"),
        "a component over the budget was not refused by the loader with its limit");
    std::filesystem::remove_all(root);
    return 0;
  }
  catch (...)
  {
    std::filesystem::remove_all(root);
    throw;
  }
}
