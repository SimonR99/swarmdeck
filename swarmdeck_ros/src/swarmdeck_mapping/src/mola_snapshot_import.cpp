#include <swarmdeck_mapping/mola_submap_bridge.hpp>
#include <swarmdeck_mapping/chunk_io.hpp>

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <nlohmann/json.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using json = nlohmann::json;

swarmdeck_mapping::Matrix4 matrix(const json& value)
{
  if (!value.is_array() || value.size() != 4) throw std::runtime_error("pose must be 4x4");
  swarmdeck_mapping::Matrix4 result{};
  for (std::size_t row = 0; row < 4; ++row)
  {
    if (!value[row].is_array() || value[row].size() != 4)
      throw std::runtime_error("pose must be 4x4");
    for (std::size_t col = 0; col < 4; ++col) result[row * 4 + col] = value[row][col].get<double>();
  }
  return result;
}

std::string submapId(const json& value)
{
  return value.at("robot_id").get<std::string>() + "/" +
         value.at("session_id").get<std::string>() + "/submap/" +
         std::to_string(value.at("seq").get<std::uint64_t>());
}
}  // namespace

int main(int argc, char** argv)
try
{
  if (argc != 4)
  {
    std::cerr << "usage: swarmdeck-mola-import SNAPSHOT_JSON CHUNKS_DIR OUTPUT.metricmap\n";
    return 2;
  }
  const auto snapshot_size = std::filesystem::file_size(argv[1]);
  if (snapshot_size > 4 * 1024 * 1024) throw std::runtime_error("snapshot exceeds 4 MiB limit");
  std::ifstream input(argv[1]);
  if (!input) throw std::runtime_error("cannot open snapshot JSON");
  const json snapshot = json::parse(input);
  if (snapshot.at("schema") != "swarmdeck.autonomy.v1")
    throw std::runtime_error("unsupported autonomy snapshot schema");
  const auto& manifests = snapshot.at("manifests");
  if (manifests.size() != 1)
    throw std::runtime_error("one component manifest is required per MOLA import");
  const auto& manifest = manifests.front();
  const auto& revision = manifest.at("graph_revision");
  swarmdeck_mapping::SolutionVersion version{
      revision.at("component_id").get<std::string>(),
      revision.at("epoch").get<std::uint64_t>(),
      revision.at("revision").get<std::uint64_t>(),
      snapshot.at("snapshot_id").get<std::string>()};

  std::vector<swarmdeck_mapping::SubmapInput> submaps;
  for (const auto& item : manifest.at("submaps"))
  {
    swarmdeck_mapping::SubmapInput submap;
    submap.external_id = submapId(item.at("submap_id"));
    submap.T_component_submap = matrix(item.at("T_component_submap"));
    for (const auto& chunk : item.at("chunks"))
    {
      if (chunk.at("encoding") != "application/vnd.swarmdeck.xyz-f32.v1")
        throw std::runtime_error("unsupported map chunk encoding");
      auto points = swarmdeck_mapping::readXyzChunk(
          argv[2], chunk.at("sha256").get<std::string>(),
          chunk.at("size_bytes").get<std::size_t>(),
          chunk.at("point_count").get<std::size_t>());
      submap.points_local.insert(submap.points_local.end(), points.begin(), points.end());
    }
    submaps.emplace_back(std::move(submap));
  }

  swarmdeck_mapping::MolaSubmapBridge bridge;
  std::size_t published = 0;
  bridge.subscribeToMapUpdates([&published](const mola::MapSourceBase::MapUpdate&) { ++published; });
  bridge.replaceGeometrySnapshot(submaps, version, manifest.dump());
  if (published != 1) throw std::runtime_error("MOLA MapSource did not publish the snapshot");

  mrpt::io::CFileGZOutputStream output(argv[3]);
  if (!output.is_open()) throw std::runtime_error("cannot open output metric map");
  auto archive = mrpt::serialization::archiveFrom(output);
  archive << *bridge.currentMap();
  std::cout << "imported " << submaps.size() << " submaps and "
            << bridge.currentMap()->point_count() << " points at "
            << version.component_id << '@' << version.epoch << ':' << version.revision << '\n';
  return 0;
}
catch (const std::exception& error)
{
  std::cerr << "swarmdeck-mola-import: " << error.what() << '\n';
  return 1;
}
