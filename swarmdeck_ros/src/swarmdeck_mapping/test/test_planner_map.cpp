#include <swarmdeck_mapping/planner_map.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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

bool sameSurfaces(
    const std::vector<swarmdeck_mapping::PlannerSurfaceSample>& lhs,
    const std::vector<swarmdeck_mapping::PlannerSurfaceSample>& rhs)
{
  return lhs.size() == rhs.size() &&
         std::equal(
             lhs.begin(), lhs.end(), rhs.begin(),
             [](const auto& left, const auto& right) {
               return left.x == right.x && left.y == right.y && left.z == right.z;
             });
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

  // A full ray history must remain publishable at its work limit. Retain the
  // nearest measured ray from old, middle and new keyframes before spending
  // work on any keyframe's farther, individually admissible ray. Every
  // endpoint remains occupied, and input submap order cannot change the
  // selected free-space product.
  const SubmapInput old_rays{
      "old", {{0.42F, 0.01F, 0.5F}, {0.01F, 0.62F, 0.5F}}, pose(),
      {{0.01F, 0.01F, 0.5F}}, 100, true};
  const SubmapInput middle_rays{
      "middle", {{5.42F, 0.01F, 0.5F}, {5.01F, 0.62F, 0.5F}}, pose(),
      {{5.01F, 0.01F, 0.5F}}, 150, true};
  const SubmapInput new_rays{
      "new", {{10.42F, 0.01F, 0.5F}, {10.01F, 0.62F, 0.5F}}, pose(),
      {{10.01F, 0.01F, 0.5F}}, 200, true};
  const SubmapInput blocker{
      "blocker", {{0.31F, 0.01F, 0.5F}}, pose(), {}, 150, false};
  PlannerGridLimits selected_limits;
  selected_limits.max_ray_steps = 6;
  MolaSubmapBridge selected_bridge;
  selected_bridge.replaceGeometrySnapshot(
      {old_rays, new_rays, blocker, middle_rays}, version(0, '4'), "{}",
      identity('4'));
  const auto selected = buildNativePlannerGrid(
      *selected_bridge.currentSnapshot(), selected_limits);
  require(selected->ray_steps == 6, "selected rays did not consume the exact budget");
  require(
      selected->qualified_ray_keyframes == 3,
      "bounded selection lost qualified-keyframe accounting");
  require(
      contains(selected->free, {0, 0, 2}) &&
          contains(selected->free, {25, 0, 2}) &&
          contains(selected->free, {50, 0, 2}),
      "bounded selection starved an old, middle or new keyframe");
  require(
      !contains(selected->free, {0, 1, 2}) &&
          !contains(selected->free, {25, 1, 2}) &&
          !contains(selected->free, {50, 1, 2}),
      "bounded selection admitted a farther ray before near-field evidence");
  require(
      contains(selected->occupied, {1, 0, 2}) &&
          !contains(selected->free, {1, 0, 2}),
      "occupied evidence did not override a selected free ray");
  MolaSubmapBridge shuffled_bridge;
  shuffled_bridge.replaceGeometrySnapshot(
      {blocker, new_rays, middle_rays, old_rays}, version(0, '4'), "{}",
      identity('4'));
  const auto shuffled = buildNativePlannerGrid(
      *shuffled_bridge.currentSnapshot(), selected_limits);
  require(
      shuffled->ray_steps == selected->ray_steps &&
          shuffled->occupied == selected->occupied &&
          shuffled->free == selected->free &&
          sameSurfaces(shuffled->surfaces, selected->surfaces),
      "bounded ray selection depends on input submap order");

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

  PlannerGridLimits tiny_ray_budget;
  tiny_ray_budget.max_ray_steps = 1;
  const auto bounded = buildNativePlannerGrid(*snapshot0, tiny_ray_budget);
  require(
      bounded->ray_steps <= tiny_ray_budget.max_ray_steps &&
          bounded->occupied == grid0->occupied &&
          sameSurfaces(bounded->surfaces, grid0->surfaces),
      "small ray budget rejected or discarded endpoint geometry");

  const auto selection_snapshot = selected_bridge.currentSnapshot();
  const auto rejects_with = [&selection_snapshot](
                                const PlannerGridLimits& limits,
                                const std::string& detail) {
    try
    {
      (void)buildNativePlannerGrid(*selection_snapshot, limits);
      return false;
    }
    catch (const std::runtime_error& error)
    {
      return std::string(error.what()).find(detail) != std::string::npos;
    }
  };
  PlannerGridLimits point_bounded;
  point_bounded.max_points = 1;
  require(
      rejects_with(point_bounded, "point budget"),
      "ray selection weakened the planner point bound");
  PlannerGridLimits voxel_bounded;
  voxel_bounded.max_voxels = 1;
  require(
      rejects_with(voxel_bounded, "occupied voxel budget"),
      "ray selection weakened the planner voxel bound");
  PlannerGridLimits time_bounded;
  time_bounded.max_build_s = std::numeric_limits<double>::min();
  require(
      rejects_with(time_bounded, "time budget"),
      "ray selection weakened the planner build deadline");

  // A product export failure is a before-commit failure: the corrected MOLA
  // pose and revision must remain unmodified.
  bool rejected = false;
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
