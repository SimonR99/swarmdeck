#include <swarmdeck_mapping/planner_map.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
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

// One capture of `points`. A qualified capture proves a single sensor origin
// applies to all of its first returns, which is what admits its rays as
// free-space and see-through evidence.
SubmapInput capture(
    const char* external_id, const std::vector<PointXYZ>& points,
    const std::uint64_t observed_at_ns, const bool qualified)
{
  std::vector<PointXYZ> origins;
  if (qualified) origins.push_back({0.01F, 0.01F, 0.01F});
  return {external_id, points, pose(), origins, observed_at_ns, qualified};
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

// `count` qualified keyframes of 4,096 floor endpoints each (the capture's
// configured maximum), one every 0.5 m along x, as a driving robot stores
// them. 245 of these hold 1,003,520 points: the count that froze robot_0's
// product on benchbot (mission 1a8cc114, 2026-09-19).
std::vector<SubmapInput> keyframeSweep(const std::size_t count)
{
  std::vector<SubmapInput> result;
  result.reserve(count);
  for (std::size_t index = 0; index < count; ++index)
  {
    std::vector<PointXYZ> points;
    points.reserve(4096);
    for (int row = 0; row < 64; ++row)
      for (int column = 0; column < 64; ++column)
        points.push_back(
            {0.5F + 0.1F * static_cast<float>(column),
             -3.2F + 0.1F * static_cast<float>(row), 0.0F});
    SubmapInput submap{
        "sweep-" + std::to_string(index), std::move(points),
        pose(0.5 * static_cast<double>(index)), {{0.0F, 0.0F, 0.5F}},
        1'000'000'000ULL * (index + 1), true};
    result.push_back(std::move(submap));
  }
  return result;
}

bool throwsWith(
    const std::function<void()>& action, const std::string& detail, const bool invalid)
{
  try
  {
    action();
    return false;
  }
  catch (const std::invalid_argument& error)
  {
    return invalid && std::string(error.what()).find(detail) != std::string::npos;
  }
  catch (const std::runtime_error& error)
  {
    return !invalid && std::string(error.what()).find(detail) != std::string::npos;
  }
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

  // Visibility retirement. A peer robot standing beside this one at a grouped
  // start enters the persistent product both as an occupied voxel and as
  // terrain surface samples. Obstacle expiry alone cannot prove an area clear,
  // so only repeated qualified rays that see through the same voxel, observed
  // later than every endpoint in it, may retire those endpoints.
  const std::vector<PointXYZ> body_points{{1.05F, 0.01F, 0.01F}};
  const std::vector<PointXYZ> beyond_points{{2.05F, 0.01F, 0.01F}};
  const PlannerVoxel body_voxel{5, 0, 0};
  const PlannerVoxel beyond_voxel{10, 0, 0};
  const auto clearing_grid = [](const std::vector<SubmapInput>& captures,
                                const PlannerGridLimits& limits = {}) {
    MolaSubmapBridge clearing_bridge;
    clearing_bridge.replaceGeometrySnapshot(
        captures, version(0, '5'), "{}", identity('5'));
    return buildNativePlannerGrid(*clearing_bridge.currentSnapshot(), limits);
  };

  const std::vector<SubmapInput> retiring_captures{
      capture("body", body_points, 100, true),
      capture("clear_a", beyond_points, 200, true),
      capture("clear_b", beyond_points, 300, true),
      capture("clear_c", beyond_points, 400, true)};
  const auto retired = clearing_grid(retiring_captures);
  require(retired->retired_count == 1, "a seen-through body was not retired");
  require(
      !contains(retired->occupied, body_voxel),
      "a retired body remained an occupied voxel");
  require(
      contains(retired->free, body_voxel),
      "a retired body that the rays carved did not become free");
  require(
      heightsAt(*retired, body_voxel.x).empty(),
      "a retired body kept its terrain surface sample");
  require(
      contains(retired->occupied, beyond_voxel),
      "the endpoint behind a retired body was lost");
  require(
      retired->point_count == 4 && retired->surfaces.size() == 3 &&
          retired->surfaces.size() + retired->retired_count ==
              retired->point_count,
      "retired surfaces do not account for every stored point");

  // The rays only prove the voxel was empty while they passed. A capture newer
  // than them puts the body back and keeps every endpoint in that voxel,
  // including the older ones the rays had otherwise disproved.
  auto returning_captures = retiring_captures;
  returning_captures.push_back(capture("body_again", body_points, 500, false));
  const auto returning = clearing_grid(returning_captures);
  require(
      returning->retired_count == 0,
      "a body observed after the clearing rays was retired");
  require(
      contains(returning->occupied, body_voxel),
      "a returning body is not occupied");
  require(
      heightsAt(*returning, body_voxel.x).size() == 2,
      "a returning body lost a terrain surface sample");

  // Rays terminate at what they hit, so a wall is never traversed however many
  // times it is measured.
  const auto repeated_wall = clearing_grid(
      {capture("hit_a", body_points, 100, true),
       capture("hit_b", body_points, 200, true),
       capture("hit_c", body_points, 300, true)});
  require(
      repeated_wall->retired_count == 0 &&
          contains(repeated_wall->occupied, body_voxel) &&
          heightsAt(*repeated_wall, body_voxel.x).size() == 3,
      "a repeatedly measured wall was retired");

  const std::vector<SubmapInput> stray_captures{
      capture("body", body_points, 100, true),
      capture("clear_a", beyond_points, 200, true)};
  const auto stray = clearing_grid(stray_captures);
  require(
      stray->retired_count == 0 && contains(stray->occupied, body_voxel),
      "a single stray traversal retired a body");
  PlannerGridLimits eager_clearing;
  eager_clearing.min_clearing_traversals = 1;
  const auto eager = clearing_grid(stray_captures, eager_clearing);
  require(
      eager->retired_count == 1 && !contains(eager->occupied, body_voxel),
      "min_clearing_traversals is not the retirement threshold");

  // A ray that passed before the body arrived proves nothing about it.
  const auto older_rays = clearing_grid(
      {capture("clear_a", beyond_points, 100, true),
       capture("clear_b", beyond_points, 200, true),
       capture("clear_c", beyond_points, 300, true),
       capture("body", body_points, 500, true)});
  require(
      older_rays->retired_count == 0 &&
          contains(older_rays->occupied, body_voxel),
      "traversals older than the endpoint retired it");

  const auto unqualified_clearing = clearing_grid(
      {capture("body", body_points, 100, false),
       capture("clear_a", beyond_points, 200, false),
       capture("clear_b", beyond_points, 300, false),
       capture("clear_c", beyond_points, 400, false)});
  require(
      unqualified_clearing->free.empty() && unqualified_clearing->ray_steps == 0,
      "captures without ray evidence carved free space");
  require(
      unqualified_clearing->retired_count == 0 &&
          contains(unqualified_clearing->occupied, body_voxel),
      "captures without ray evidence retired a body");

  // Height-aware clearing. A road surface is a sheet near the bottom of its
  // voxel, and rays from a lidar above the road that end far ahead cross the
  // road's own ground voxels above that sheet: they see nothing of it, and
  // counting them retired the lane ahead of a parked robot in bands (benchbot,
  // 2026-09-18). A traversal counts only where the ray is sampled no higher
  // than the voxel's highest endpoint plus `clearing_height_tolerance_m`.
  // The road voxel {5, 0, -2} spans x [1.0, 1.2), y [0, 0.2), z [-0.4, -0.2);
  // its samples top out at z = -0.34, so the default band ends at -0.29. A
  // horizontal ray keeps its origin height at every step, and steps 0.15 m
  // apart put at least one step inside a 0.2 m voxel the ray fully crosses
  // (here two, at x = 1.02 and x = 1.17).
  const std::vector<PointXYZ> road_points{
      {1.10F, 0.10F, -0.34F}, {1.06F, 0.06F, -0.35F}, {1.14F, 0.14F, -0.36F}};
  const PlannerVoxel road_voxel{5, 0, -2};
  const auto road_captures = [&road_points](
                                 const PointXYZ& origin, const PointXYZ& endpoint) {
    std::vector<SubmapInput> captures{capture("road", road_points, 100, false)};
    std::uint64_t stamp = 200;
    for (const char* id : {"later_a", "later_b", "later_c"})
    {
      captures.push_back({id, {endpoint}, pose(), {origin}, stamp, true});
      stamp += 100;
    }
    return captures;
  };

  const auto over_road = clearing_grid(
      road_captures({0.12F, 0.10F, -0.22F}, {3.12F, 0.10F, -0.22F}));
  require(
      over_road->qualified_ray_keyframes == 3 &&
          contains(over_road->free, {4, 0, -2}) &&
          contains(over_road->free, {6, 0, -2}) &&
          contains(over_road->occupied, {15, 0, -2}),
      "rays over the road did not carve the voxels on either side of it");
  require(
      over_road->retired_count == 0 && contains(over_road->occupied, road_voxel) &&
          !contains(over_road->free, road_voxel) &&
          heightsAt(*over_road, road_voxel.x).size() == 3,
      "rays passing 0.12 m above a road surface retired or freed it");

  const auto under_road = clearing_grid(
      road_captures({0.12F, 0.10F, -0.36F}, {3.12F, 0.10F, -0.36F}));
  require(
      under_road->retired_count == 3 && !contains(under_road->occupied, road_voxel) &&
          contains(under_road->free, road_voxel) &&
          heightsAt(*under_road, road_voxel.x).empty() &&
          contains(under_road->occupied, {15, 0, -2}),
      "rays passing 0.02 m below a road surface did not retire it");

  // The tolerance is the allowance above the highest endpoint that still
  // counts, and the comparison is inclusive.
  const auto near_road_captures =
      road_captures({0.12F, 0.10F, -0.30F}, {3.12F, 0.10F, -0.30F});
  const auto within_tolerance = clearing_grid(near_road_captures);
  require(
      within_tolerance->retired_count == 3,
      "rays 0.04 m above the surface, within the 0.05 m tolerance, did not count");
  PlannerGridLimits exact_height;
  exact_height.clearing_height_tolerance_m = 0;
  const auto above_exact = clearing_grid(near_road_captures, exact_height);
  require(
      above_exact->retired_count == 0 && contains(above_exact->occupied, road_voxel) &&
          !contains(above_exact->free, road_voxel),
      "clearing_height_tolerance_m is not the height allowance");
  const auto at_exact = clearing_grid(
      road_captures({0.12F, 0.10F, -0.34F}, {3.12F, 0.10F, -0.34F}), exact_height);
  require(
      at_exact->retired_count == 3,
      "a ray at exactly the highest endpoint height did not count with zero tolerance");
  PlannerGridLimits negative_tolerance;
  negative_tolerance.clearing_height_tolerance_m = -0.01;
  bool tolerance_rejected = false;
  try
  {
    (void)clearing_grid(near_road_captures, negative_tolerance);
  }
  catch (const std::invalid_argument&)
  {
    tolerance_rejected = true;
  }
  require(tolerance_rejected, "a negative clearing height tolerance was accepted");

  // A ray is examined at every step, not only where it enters a voxel: a
  // vertical 0.95 m ray takes 7 steps of 0.135714 m, so from z = 0.187143 its
  // third step lands in the road voxel at z = -0.22 (above the band) and its
  // fourth at z = -0.355714 (among the samples). The voxel counts once per ray.
  const auto descending = clearing_grid(
      road_captures({1.10F, 0.10F, 0.187143F}, {1.10F, 0.10F, -0.762857F}));
  require(
      descending->retired_count == 3 && contains(descending->free, road_voxel) &&
          contains(descending->free, {5, 0, -1}) &&
          contains(descending->occupied, {5, 0, -4}),
      "a ray that reached the samples after entering above them did not count");

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

  // One point budget. The grid build and the product share the loader's
  // `kMaxPointsPerMap`; until 2026-09-19 the build had its own 1,000,000
  // default, so a map of 245 full keyframes loaded but never produced a grid.
  static_assert(
      PlannerGridLimits{}.max_points == kMaxPointsPerMap,
      "the planner grid point budget must be the shared component budget");
  static_assert(kMaxPointsPerMap > 1'000'000, "the shared budget was not raised");
  MolaSubmapBridge sweep_bridge;
  sweep_bridge.replaceGeometrySnapshot(
      keyframeSweep(245), version(0, '6'), "{}", identity('6'));
  const auto sweep = buildNativePlannerGrid(*sweep_bridge.currentSnapshot());
  require(
      sweep->point_count == 1'003'520 &&
          sweep->surfaces.size() + sweep->retired_count == sweep->point_count &&
          sweep->qualified_ray_keyframes == 245,
      "245 full keyframes did not build with the default point budget");
  const auto sweep_artifact = writeNativePlannerGrid(*sweep, root / "sweep.sdmgrid");
  require(
      sweep_artifact.size_bytes > 24 * sweep->surfaces.size(),
      "245 full keyframes did not write with the default product budget");
  require(
      throwsWith(
          [&] {
            (void)writeNativePlannerGrid(
                *sweep, root / "never.sdmgrid", kMaxPlannerArtifactBytes,
                kMaxPlannerMetadataBytes, 1'000'000);
          },
          "product budget: 1003520 points, budget 1000000", true),
      "the product point budget is not the writer's max_points");
  MolaSubmapBridge over_bridge;
  over_bridge.replaceGeometrySnapshot(
      keyframeSweep(489), version(0, '7'), "{}", identity('7'));
  require(
      throwsWith(
          [&] { (void)buildNativePlannerGrid(*over_bridge.currentSnapshot()); },
          "planner grid point budget exceeded: 2002944 points, budget 2000000",
          false),
      "a map over the shared budget was not refused with its numbers");
  PlannerGridLimits raised;
  raised.max_points = 2'002'944;
  require(
      buildNativePlannerGrid(*over_bridge.currentSnapshot(), raised)->point_count ==
          2'002'944,
      "a raised point budget did not admit the map");

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
  require(metadata.at("retired_count") == 0,
          "explicit empty retired count is absent");
  require(
      metadata.at("surface_count").get<std::size_t>() +
              metadata.at("retired_count").get<std::size_t>() ==
          metadata.at("point_count").get<std::size_t>(),
      "exported surface and retired counts do not cover the point count");
  require(first.size_bytes == bytes.size(), "reported planner byte size is wrong");

  // SDMGRID1 carries the retirement so every reader can still cross-check
  // point_count against the manifest chunks.
  (void)writeNativePlannerGrid(*retired, root / "retired.sdmgrid");
  std::ifstream retired_input(root / "retired.sdmgrid", std::ios::binary);
  const std::string retired_bytes(
      (std::istreambuf_iterator<char>(retired_input)),
      std::istreambuf_iterator<char>());
  const auto retired_metadata = nlohmann::json::parse(
      retired_bytes.substr(12, readU32(retired_bytes, 8)));
  require(
      retired_metadata.at("retired_count") == 1 &&
          retired_metadata.at("surface_count") == 3 &&
          retired_metadata.at("point_count") == 4,
      "exported retirement does not match the built product");

  NativePlannerGrid tampered = *retired;
  tampered.retired_count = 0;
  bool contract_rejected = false;
  try
  {
    (void)writeNativePlannerGrid(tampered, root / "tampered.sdmgrid");
  }
  catch (const std::invalid_argument&)
  {
    contract_rejected = true;
  }
  require(
      contract_rejected,
      "planner export accepted surface and retired counts below point_count");
  std::filesystem::remove_all(root);
  return 0;
}
