#include <swarmdeck_mapping/mola_submap_bridge.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(const bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

swarmdeck_mapping::Matrix4 pose(const double x)
{
  return {1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

swarmdeck_mapping::SolutionVersion version(const std::uint64_t revision, const char digit)
{
  return {"component:test", 1, revision, std::string(64, digit)};
}
}  // namespace

int main()
{
  swarmdeck_mapping::MolaSubmapBridge bridge;
  std::size_t publications = 0;
  bridge.subscribeToMapUpdates(
      [&publications](const mola::MapSourceBase::MapUpdate& update) {
        require(static_cast<bool>(update.map), "publication contains no map");
        require(update.keep_last_one_only, "publication is not a coherent replacement");
        ++publications;
      });

  const std::vector<swarmdeck_mapping::PointXYZ> triangle{
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0.5F, 0.5F, 0.1F}};
  bridge.replaceGeometrySnapshot(
      {{"r/s/submap/0", triangle, pose(0)}, {"r/s/submap/1", triangle, pose(2)}},
      version(0, '0'), "{}");
  require(bridge.currentMap()->keyframePoses().size() == 2, "wrong initial keyframe count");
  require(bridge.currentMap()->point_count() == 10, "wrong initial point count");

  const auto held_snapshot = bridge.currentMap();
  const auto result = bridge.applyPoseSolution(
      {{"r/s/submap/1", pose(5)}}, version(1, '1'), "{}");
  require(
      result == swarmdeck_mapping::MolaSubmapBridge::ApplyResult::Applied,
      "pose solution was not applied");
  const auto poses = bridge.currentMap()->keyframePoses();
  require(poses.size() == 2, "pose correction changed keyframe membership");
  require(poses.at(1).x() == 5, "pose correction was not visible");
  require(held_snapshot->keyframePoses().at(1).x() == 2, "held snapshot was mutated");

  // Exact replays do not mutate even if a caller supplies different update
  // arguments: the digest is the immutable content authority.
  require(
      bridge.applyPoseSolution({{"r/s/submap/1", pose(99)}}, version(1, '1'), "{}") ==
          swarmdeck_mapping::MolaSubmapBridge::ApplyResult::Duplicate,
      "exact replay was not reported as a duplicate");
  require(bridge.currentMap()->keyframePoses().at(1).x() == 5, "duplicate mutated the map");

  // Validate the entire batch before copy/swap. A bad second pose cannot leak
  // the valid first pose or consume the revision.
  auto invalid = pose(0);
  invalid[3] = std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try
  {
    bridge.applyPoseSolution(
        {{"r/s/submap/0", pose(8)}, {"r/s/submap/1", invalid}}, version(2, '2'), "{}");
  }
  catch (const std::invalid_argument&)
  {
    rejected = true;
  }
  require(rejected, "nonfinite batch was accepted");
  require(bridge.currentMap()->keyframePoses().at(0).x() == 0, "partial batch update leaked");
  require(bridge.currentMap()->keyframePoses().at(1).x() == 5, "rejected batch changed pose");

  // Retraction is a coherent replacement: the removed keyframe and its points
  // are absent, instead of remaining as a stale additive contribution.
  bridge.replaceGeometrySnapshot(
      {{"r/s/submap/1", triangle, pose(5)}}, version(2, '2'), "{}");
  require(bridge.currentMap()->keyframePoses().size() == 1, "retraction kept keyframe");
  require(bridge.currentMap()->point_count() == 5, "retraction kept stale geometry");
  require(publications == 3, "unexpected publication count");

  // A conflicting/stale snapshot must leave the current map intact.
  rejected = false;
  try
  {
    bridge.replaceGeometrySnapshot({}, version(1, '3'), "{}");
  }
  catch (const std::invalid_argument&)
  {
    rejected = true;
  }
  require(rejected, "stale geometry snapshot was accepted");
  require(bridge.currentMap()->point_count() == 5, "stale snapshot mutated current map");
  rejected = false;
  try
  {
    bridge.replaceGeometrySnapshot({}, version(2, '4'), "{}");
  }
  catch (const std::invalid_argument&)
  {
    rejected = true;
  }
  require(rejected, "conflicting geometry snapshot was accepted");
  require(bridge.currentMap()->point_count() == 5, "conflicting snapshot mutated current map");
  return 0;
}
