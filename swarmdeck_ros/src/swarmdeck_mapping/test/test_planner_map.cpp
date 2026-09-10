#include <swarmdeck_mapping/planner_map.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace
{
using swarmdeck_mapping::Matrix4;
using swarmdeck_mapping::NativePlannerGrid;
using swarmdeck_mapping::PlannerVoxel;
using swarmdeck_mapping::PointXYZ;
using swarmdeck_mapping::SubmapInput;

void require(const bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

Matrix4 pose(const double x = 0, const double y = 0, const double z = 0)
{
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}

swarmdeck_mapping::SolutionVersion version(
    const std::uint64_t revision, const char digit)
{
  return {"component:test", 1, revision, std::string(64, digit)};
}

swarmdeck_mapping::SnapshotIdentity identity(const char digit)
{
  return {
      std::string(64, digit), std::string(64, digit), std::string(64, digit),
      std::string(64, digit), std::string(64, digit), "component_test"};
}

bool contains(const std::vector<PlannerVoxel>& values, const PlannerVoxel& value)
{
  return std::binary_search(values.begin(), values.end(), value);
}

std::vector<PointXYZ> patch(
    const float center_x, const float z, const float center_y = 0.1F)
{
  std::vector<PointXYZ> result;
  for (const float dx : {-0.2F, 0.0F, 0.2F})
    for (const float dy : {-0.2F, 0.0F, 0.2F})
      result.push_back({center_x + dx, center_y + dy, z});
  return result;
}

std::vector<double> heightsAt(const NativePlannerGrid& grid, const std::int64_t x)
{
  std::vector<double> result;
  for (const auto& sample : grid.surfaces)
    if (sample.x == x) result.push_back(sample.z);
  std::sort(result.begin(), result.end());
  return result;
}

std::uint32_t readU32(const std::string& bytes, const std::size_t offset)
{
  std::uint32_t result = 0;
  for (std::size_t index = 0; index < 4; ++index)
    result |= static_cast<std::uint32_t>(
                  static_cast<unsigned char>(bytes.at(offset + index)))
              << (8 * index);
  return result;
}
}  // namespace

int main()
{
  using namespace swarmdeck_mapping;

  MolaSubmapBridge bridge;
  const std::vector<PointXYZ> wall{{2.1F, 0.1F, 0.5F}};
  bridge.replaceGeometrySnapshot(
      {{"wall", wall, pose(), {{0.1F, 0.1F, 0.5F}}, 100, true}},
      version(0, '0'), "{}", identity('a'));
  const auto snapshot0 = bridge.currentSnapshot();
  require(snapshot0.has_value(), "bridge produced no native snapshot");
  const auto grid0 = buildNativePlannerGrid(*snapshot0);
  require(grid0->qualified_ray_keyframes == 1, "qualified ray was not counted");
  require(contains(grid0->occupied, {10, 0, 2}), "wall endpoint is not occupied");
  require(contains(grid0->free, {5, 0, 2}), "qualified ray did not mark free space");
  require(!contains(grid0->free, {10, 0, 2}), "occupied endpoint remained free");

  bridge.applyPoseSolution(
      {{"wall", pose(2)}}, version(1, '1'), "{}", identity('b'));
  const auto corrected = buildNativePlannerGrid(*bridge.currentSnapshot());
  require(!contains(corrected->occupied, {10, 0, 2}), "old corrected wall remained");
  require(contains(corrected->occupied, {20, 0, 2}), "corrected wall is absent");
  require(contains(grid0->occupied, {10, 0, 2}), "held planner grid was mutated");

  MolaSubmapBridge unknown_bridge;
  unknown_bridge.replaceGeometrySnapshot(
      {{"legacy", wall, pose(), {{0.1F, 0.1F, 0.5F}}, 100, false}},
      version(0, '0'), "{}", identity('c'));
  const auto unknown = buildNativePlannerGrid(*unknown_bridge.currentSnapshot());
  require(unknown->occupied.size() == 1, "legacy return did not remain occupied");
  require(unknown->free.empty(), "unknown ray provenance carved free space");
  require(unknown->qualified_ray_keyframes == 0, "unknown ray was qualified");

  // Geometry replacement and retraction rebuild from the coherent MOLA
  // snapshot, so prior endpoint and ray contributions disappear together.
  bridge.replaceGeometrySnapshot(
      {{"wall", {{6.1F, 0.1F, 0.5F}}, pose(), {}, 200, false}},
      version(2, '2'), "{}", identity('d'));
  const auto replacement = buildNativePlannerGrid(*bridge.currentSnapshot());
  require(!contains(replacement->occupied, {20, 0, 2}), "replacement kept old wall");
  require(contains(replacement->occupied, {30, 0, 2}), "replacement endpoint missing");
  require(replacement->free.empty(), "replacement retained old free ray");
  bridge.replaceGeometrySnapshot({}, version(3, '3'), "{}", identity('e'));
  const auto retracted = buildNativePlannerGrid(*bridge.currentSnapshot());
  require(retracted->occupied.empty() && retracted->free.empty() &&
              retracted->surfaces.empty(),
          "retraction retained planner contributions");

  // Preserve exact endpoint heights for floor, curb, down-step/drop and
  // vertically stacked support surfaces. Terrain fitting remains shared with
  // the existing Python planner rather than being reimplemented here.
  std::vector<PointXYZ> terrain = patch(0.1F, 0.0F);
  const auto curb = patch(1.1F, 0.25F);
  const auto lower = patch(2.1F, -0.35F);
  const auto middle = patch(0.1F, 2.0F);
  const auto upper = patch(0.1F, 4.0F);
  terrain.insert(terrain.end(), curb.begin(), curb.end());
  terrain.insert(terrain.end(), lower.begin(), lower.end());
  terrain.insert(terrain.end(), middle.begin(), middle.end());
  terrain.insert(terrain.end(), upper.begin(), upper.end());
  MolaSubmapBridge terrain_bridge;
  terrain_bridge.replaceGeometrySnapshot(
      {{"terrain", terrain, pose(), {}, 300, false}}, version(0, '0'), "{}",
      identity('f'));
  const auto terrain_grid = buildNativePlannerGrid(*terrain_bridge.currentSnapshot());
  const auto stacked = heightsAt(*terrain_grid, 0);
  require(
      std::find(stacked.begin(), stacked.end(), 0.0) != stacked.end() &&
          std::find(stacked.begin(), stacked.end(), 2.0) != stacked.end() &&
          std::find(stacked.begin(), stacked.end(), 4.0) != stacked.end(),
      "stacked surface heights were flattened");
  const auto curb_heights = heightsAt(*terrain_grid, 5);
  const auto lower_heights = heightsAt(*terrain_grid, 10);
  require(!curb_heights.empty() && std::abs(curb_heights.front() - 0.25) < 1e-6,
          "curb height was lost");
  require(!lower_heights.empty() && std::abs(lower_heights.front() + 0.35) < 1e-6,
          "down-step surface height was lost");

  bool rejected = false;
  try
  {
    PlannerGridLimits bounded;
    bounded.max_ray_steps = 1;
    (void)buildNativePlannerGrid(*snapshot0, bounded);
  }
  catch (const std::runtime_error& error)
  {
    rejected = std::string(error.what()).find("ray budget") != std::string::npos;
  }
  require(rejected, "ray work exceeded its bound without rejection");

  // A product export failure is a before-commit failure: the corrected MOLA
  // pose and revision must remain unmodified.
  rejected = false;
  try
  {
    unknown_bridge.applyPoseSolution(
        {{"legacy", pose(9)}}, version(1, '1'), "{}", identity('c'),
        [](const NativeGeometrySnapshot& candidate) {
          const auto grid = buildNativePlannerGrid(candidate);
          (void)writeNativePlannerGrid(*grid, "/tmp/never-installed.sdmgrid", 1);
        });
  }
  catch (const std::runtime_error&)
  {
    rejected = true;
  }
  require(rejected, "bounded planner export unexpectedly succeeded");
  require(
      unknown_bridge.currentSnapshot()->graph_version.revision == 0 &&
          unknown_bridge.currentMap()->keyframePoses().at(0).x() == 0,
      "failed planner export committed a pose correction");

  const auto root = std::filesystem::temp_directory_path() /
                    ("swarmdeck-planner-grid-test-" + std::to_string(::getpid()));
  std::filesystem::create_directories(root);
  const auto first = writeNativePlannerGrid(*terrain_grid, root / "first.sdmgrid");
  const auto second = writeNativePlannerGrid(*terrain_grid, root / "second.sdmgrid");
  require(first.sha256 == second.sha256, "planner grid export is nondeterministic");
  std::ifstream input(root / "first.sdmgrid", std::ios::binary);
  const std::string bytes(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  require(bytes.substr(0, 8) == "SDMGRID1", "planner grid magic is invalid");
  const auto metadata_size = readU32(bytes, 8);
  const auto metadata = nlohmann::json::parse(bytes.substr(12, metadata_size));
  require(metadata.at("schema") == "swarmdeck.mola_planner_grid.v1",
          "planner grid schema is invalid");
  require(metadata.at("free_count") == 0, "explicit empty free count is absent");
  require(metadata.at("qualified_ray_keyframes") == 0,
          "qualified-ray count is incorrect");
  require(first.size_bytes == bytes.size(), "reported planner byte size is wrong");
  std::filesystem::remove_all(root);
  return 0;
}
