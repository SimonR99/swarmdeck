#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_launcher/MolaLauncherApp.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
using json = nlohmann::json;

void require(bool value, const std::string& message)
{
  if (!value) throw std::runtime_error(message);
}

void writeAtomic(const std::filesystem::path& path, const json& value)
{
  const auto temporary = path.string() + ".test-tmp";
  {
    std::ofstream output(temporary);
    output << value.dump();
    if (!output) throw std::runtime_error("cannot write fixture snapshot");
  }
  std::filesystem::rename(temporary, path);
}
}  // namespace

int main(int argc, char** argv)
try
{
  require(argc == 2 || argc == 3, "usage: test_mola_module MODULE_DIR [FIXTURE_DIR]");
  mola::MolaLauncherApp launcher;
  launcher.addPathModuleLibs(argv[1]);
  launcher.scanAndLoadLibraries();
  auto module = mola::ExecutableBase::Factory("swarmdeck_mola::SwarmDeckMapSource");
  require(static_cast<bool>(module), "MOLA factory did not discover SwarmDeck module");
  module->setModuleInstanceName("corrected_map");
  auto source = std::dynamic_pointer_cast<mola::MapSourceBase>(module);
  require(static_cast<bool>(source), "framework module is not a MapSource");
  mola::MinimalModuleContainer container({module});
  require(module->findService<mola::MapSourceBase>().size() == 1,
          "MOLA service discovery cannot find the map provider");
  if (argc == 2)
  {
    std::cout << "PASS MOLA plugin loading, factory and map-provider discovery\n";
    return 0;
  }

  const std::filesystem::path fixture(argv[2]);
  const auto snapshot_path = fixture / "snapshot.json";
  json first;
  std::ifstream(snapshot_path) >> first;
  const auto config = mola::Yaml::FromText(json{
      {"snapshot_file", snapshot_path.string()}, {"chunks_dir", (fixture / "chunks").string()},
      {"component_id", first["manifests"][0]["graph_revision"]["component_id"]},
      {"artifact_file", (fixture / "framework.metricmap").string()},
      {"poll_s", 0.05}, {"max_points", 1000}}.dump());
  module->initialize(config);
  auto updates = std::make_shared<std::vector<mola::MapSourceBase::MapUpdate>>();
  source->subscribeToMapUpdates([updates](const auto& update) { updates->push_back(update); });
  const auto tick = [&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    module->spinOnce();
  };
  tick();
  require(updates->size() == 1, "first component was not published");
  require(json::parse(*updates->back().map_metadata).at("available"), "first map unavailable");
  const auto first_map = std::dynamic_pointer_cast<const mola::KeyframePointCloudMap>(updates->back().map);
  require(first_map && first_map->point_count() > 0, "native geometry was not published");
  require(updates->back().reference_frame == first["manifests"][0]["frame_id"],
          "framework publication changed the manifest coordinate frame");
  tick();
  require(updates->size() == 1, "unchanged snapshot was republished");

  // A correction must work even when the input chunk directory is unavailable.
  // This tests real geometry reuse through the loadable framework module.
  std::filesystem::rename(fixture / "chunks", fixture / "chunks-held");
  auto corrected = first;
  corrected["snapshot_id"] = std::string(64, 'b');
  auto& manifest = corrected["manifests"][0];
  manifest["graph_revision"]["revision"] = manifest["graph_revision"]["revision"].get<int>() + 1;
  for (auto& submap : manifest["submaps"])
    submap["pose_revision"] = manifest["graph_revision"];
  manifest["submaps"][0]["T_component_submap"][0][3] = 1.0;
  writeAtomic(snapshot_path, corrected);
  tick();
  require(updates->size() == 2, "pose correction was not published");
  require(json::parse(*updates->back().map_metadata).at("available"), "pose-only reuse failed");
  require(updates->back().map.get() != first_map.get(), "correction mutated the published map object");
  require(first_map->point_count() == std::dynamic_pointer_cast<const mola::KeyframePointCloudMap>(
              updates->back().map)->point_count(), "pose correction changed geometry size");
  const auto corrected_map = std::dynamic_pointer_cast<const mola::KeyframePointCloudMap>(
      updates->back().map);
  require(corrected_map->keyframePoses().at(0).x() == 1.0 &&
              first_map->keyframePoses().at(0).x() == 0.0,
          "framework poses were not corrected immutably");
  require(updates->back().map_name == updates->front().map_name &&
              updates->back().keep_last_one_only,
          "correction did not replace the named public layer");

  // Watch the actual whole-peer file, selecting a component explicitly. A
  // change elsewhere in the fleet must not force a geometry rebuild.
  auto other = corrected["manifests"][0];
  other["graph_revision"]["component_id"] = "component:other";
  for (auto& submap : other["submaps"])
    submap["pose_revision"] = other["graph_revision"];
  corrected["manifests"].push_back(other);
  corrected["snapshot_id"] = std::string(64, 'c');
  writeAtomic(snapshot_path, corrected);
  tick();
  require(json::parse(*updates->back().map_metadata).at("available") &&
              updates->back().map.get() == corrected_map.get(),
          "unrelated component forced a rebuild or invalidated the selected map");

  // Invalid input retracts the public layer; valid input then resumes the same
  // committed native map without requiring chunk reload or a process restart.
  writeAtomic(snapshot_path, json{{"schema", "invalid"}});
  tick();
  require(!json::parse(*updates->back().map_metadata).at("available").get<bool>(),
          "invalid source left a fresh map advertised");
  require(std::dynamic_pointer_cast<const mola::KeyframePointCloudMap>(
              updates->back().map)->point_count() == 0 &&
              updates->back().map_name == updates->front().map_name,
          "invalid source did not retract the same geometry layer");
  writeAtomic(snapshot_path, corrected);
  tick();
  require(json::parse(*updates->back().map_metadata).at("available"), "valid source did not recover");
  auto replayed = std::make_shared<std::size_t>(0);
  source->subscribeToMapUpdates([replayed](const auto&) { ++*replayed; });
  require(*replayed == 1, "late framework subscriber did not receive the current map");
  module->onQuit();
  require(!json::parse(*updates->back().map_metadata).at("available").get<bool>(),
          "shutdown did not retract the public layer");
  std::filesystem::rename(fixture / "chunks-held", fixture / "chunks");
  writeAtomic(snapshot_path, first);

  // Without a pinned selector, a single-component onboard source can change
  // component identity after a merge. Publish its new frame and reclaim the
  // previous native context; repeated changes must not exhaust the map limit.
  auto following = mola::ExecutableBase::Factory("swarmdeck_mola::SwarmDeckMapSource");
  following->setModuleInstanceName("following_map");
  mola::MinimalModuleContainer following_container({following});
  following->initialize(mola::Yaml::FromText(json{
      {"snapshot_file", snapshot_path.string()}, {"chunks_dir", (fixture / "chunks").string()},
      {"poll_s", 0.05}, {"max_points", 1000}}.dump()));
  auto followed = std::make_shared<std::vector<mola::MapSourceBase::MapUpdate>>();
  std::dynamic_pointer_cast<mola::MapSourceBase>(following)->subscribeToMapUpdates(
      [followed](const auto& update) { followed->push_back(update); });
  following->spinOnce();
  require(json::parse(*followed->back().map_metadata).at("available"),
          "unpinned single-component source did not start");

  // An unpinned multi-component source is ambiguous. It must retract the
  // public layer without releasing the previously committed context; restoring
  // the valid source therefore recovers while chunks are unavailable.
  auto ambiguous = first;
  auto ambiguous_other = ambiguous["manifests"][0];
  ambiguous_other["graph_revision"]["component_id"] = "component:ambiguous";
  for (auto& submap : ambiguous_other["submaps"])
    submap["pose_revision"] = ambiguous_other["graph_revision"];
  ambiguous["manifests"].push_back(ambiguous_other);
  ambiguous["snapshot_id"] = std::string(64, '7');
  writeAtomic(snapshot_path, ambiguous);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  following->spinOnce();
  require(!json::parse(*followed->back().map_metadata).at("available").get<bool>() &&
              std::dynamic_pointer_cast<const mola::KeyframePointCloudMap>(
                  followed->back().map)->point_count() == 0,
          "ambiguous component selection left geometry advertised");
  std::filesystem::rename(fixture / "chunks", fixture / "chunks-following-held");
  writeAtomic(snapshot_path, first);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  following->spinOnce();
  require(json::parse(*followed->back().map_metadata).at("available"),
          "valid component did not recover from retained resident geometry");
  std::filesystem::rename(fixture / "chunks-following-held", fixture / "chunks");

  for (int index = 0; index < 3; ++index)
  {
    auto merged = first;
    merged["snapshot_id"] = std::string(64, 'd' + index);
    auto& next = merged["manifests"][0];
    next["graph_revision"]["component_id"] = "component:merged" + std::to_string(index);
    next["frame_id"] = "component_merged" + std::to_string(index);
    for (auto& submap : next["submaps"])
      submap["pose_revision"] = next["graph_revision"];
    writeAtomic(snapshot_path, merged);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    following->spinOnce();
    require(json::parse(*followed->back().map_metadata).at("available") &&
                followed->back().reference_frame == next["frame_id"],
            "automatic component reassignment failed");
  }
  following->onQuit();
  writeAtomic(snapshot_path, first);
  std::cout << "PASS framework insertion, geometry reuse, immutable correction, invalidation and recovery\n";
  return 0;
}
catch (const std::exception& error)
{
  std::cerr << "MOLA framework test failed: " << error.what() << '\n';
  return 1;
}
