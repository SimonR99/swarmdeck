#include <swarmdeck_mapping/mola_submap_bridge.hpp>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <stdexcept>
#include <unordered_set>
#include <tuple>

#if !defined(MOLA_METRIC_MAPS_HAS_KFM_POSE_PLUMBING)
#error "swarmdeck_mapping requires MOLA keyframe pose plumbing (available in 2.9.0)"
#endif

namespace swarmdeck_mapping
{
namespace
{
constexpr double kTolerance = 1e-5;
constexpr std::size_t kMaxSensorOrigins = 16;

bool sameVersion(const SolutionVersion& lhs, const SolutionVersion& rhs)
{
  return lhs.component_id == rhs.component_id && lhs.epoch == rhs.epoch &&
         lhs.revision == rhs.revision;
}

std::vector<PoseUpdate> sortedPoses(
    const std::unordered_map<std::string, Matrix4>& values)
{
  std::vector<PoseUpdate> result;
  result.reserve(values.size());
  for (const auto& item : values) result.push_back({item.first, item.second});
  std::sort(
      result.begin(), result.end(), [](const PoseUpdate& lhs, const PoseUpdate& rhs) {
        return lhs.external_id < rhs.external_id;
      });
  return result;
}

std::vector<NativeKeyframeSnapshot> sortedKeyframes(
    const std::unordered_map<std::string, NativeKeyframeSnapshot>& values)
{
  std::vector<NativeKeyframeSnapshot> result;
  result.reserve(values.size());
  for (const auto& item : values) result.push_back(item.second);
  std::sort(
      result.begin(), result.end(),
      [](const NativeKeyframeSnapshot& lhs, const NativeKeyframeSnapshot& rhs) {
        return lhs.external_id < rhs.external_id;
      });
  return result;
}

NativeGeometrySnapshot makeSnapshot(
    const std::shared_ptr<const mola::KeyframePointCloudMap>& map,
    const SolutionVersion& version, const SnapshotIdentity& identity,
    const std::string& metadata,
    const std::unordered_map<std::string, Matrix4>& poses,
    const std::unordered_map<std::string, NativeKeyframeSnapshot>& keyframes,
    const double compaction_resolution_m)
{
  auto result = NativeGeometrySnapshot{
      map, version, identity, metadata, sortedPoses(poses),
      sortedKeyframes(keyframes)};
  result.compaction_resolution_m = compaction_resolution_m;
  return result;
}
}  // namespace

MolaSubmapBridge::MolaSubmapBridge()
    : map_(std::make_shared<mola::KeyframePointCloudMap>())
{
}

void MolaSubmapBridge::validateVersion(const SolutionVersion& version)
{
  static const std::regex id_pattern{"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$"};
  static const std::regex digest_pattern{"^[0-9a-f]{64}$"};
  if (!std::regex_match(version.component_id, id_pattern))
    throw std::invalid_argument("invalid component id");
  if (!std::regex_match(version.digest, digest_pattern))
    throw std::invalid_argument("solution digest must be lowercase SHA-256");
}

mrpt::poses::CPose3D MolaSubmapBridge::checkedPose(const Matrix4& values)
{
  if (!std::all_of(values.begin(), values.end(), [](const double v) { return std::isfinite(v); }))
    throw std::invalid_argument("pose contains a nonfinite value");
  for (std::size_t col = 0; col < 4; ++col)
  {
    const double expected = col == 3 ? 1.0 : 0.0;
    if (std::abs(values[12 + col] - expected) > 1e-9)
      throw std::invalid_argument("pose has an invalid homogeneous row");
  }
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
    {
      double dot = 0;
      for (std::size_t row = 0; row < 3; ++row) dot += values[row * 4 + i] * values[row * 4 + j];
      const double expected = i == j ? 1.0 : 0.0;
      if (std::abs(dot - expected) > kTolerance)
        throw std::invalid_argument("pose rotation is not orthonormal");
    }
  const double determinant =
      values[0] * (values[5] * values[10] - values[6] * values[9]) -
      values[1] * (values[4] * values[10] - values[6] * values[8]) +
      values[2] * (values[4] * values[9] - values[5] * values[8]);
  if (std::abs(determinant - 1.0) > kTolerance)
    throw std::invalid_argument("pose rotation determinant is not +1");

  mrpt::math::CMatrixDouble44 matrix;
  for (std::size_t row = 0; row < 4; ++row)
    for (std::size_t col = 0; col < 4; ++col) matrix(row, col) = values[row * 4 + col];
  return mrpt::poses::CPose3D::FromHomogeneousMatrix(matrix);
}

std::string MolaSubmapBridge::frameName(const std::string& component_id)
{
  std::string frame = "component_" + component_id;
  std::replace_if(frame.begin(), frame.end(), [](const char c) {
    return !(std::isalnum(static_cast<unsigned char>(c)) || c == '_');
  }, '_');
  return frame;
}

MolaSubmapBridge::ApplyResult MolaSubmapBridge::replaceGeometrySnapshot(
    const std::vector<SubmapInput>& submaps, const SolutionVersion& version,
    const std::string& canonical_metadata_json, const SnapshotIdentity& identity,
    const BeforeCommit& before_commit, const bool publish_update)
{
  validateVersion(version);
  auto next_map = std::make_shared<mola::KeyframePointCloudMap>();
  std::unordered_map<std::string, mola::KeyframePointCloudMap::KeyFrameID> next_ids;
  std::unordered_map<std::string, Matrix4> next_poses;
  std::unordered_map<std::string, NativeKeyframeSnapshot> next_keyframes;
  std::unordered_set<std::string> seen;
  for (const auto& submap : submaps)
  {
    if (submap.external_id.empty() || !seen.emplace(submap.external_id).second)
      throw std::invalid_argument("submap IDs must be nonempty and unique");
    auto cloud = mrpt::maps::CSimplePointsMap::Create();
    cloud->reserve(submap.points_local.size());
    for (const auto& point : submap.points_local)
    {
      if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)))
        throw std::invalid_argument("submap point contains a nonfinite value");
      cloud->insertPointFast(point.x, point.y, point.z);
    }
    if (submap.sensor_origins_local.size() > kMaxSensorOrigins)
      throw std::invalid_argument("submap sensor origin count exceeds limit");
    for (const auto& origin : submap.sensor_origins_local)
      if (!(std::isfinite(origin.x) && std::isfinite(origin.y) && std::isfinite(origin.z)))
        throw std::invalid_argument("submap sensor origin contains a nonfinite value");
    auto observation = mrpt::obs::CObservationPointCloud::Create();
    observation->timestamp = mrpt::Clock::now();
    observation->pointcloud = cloud;
    if (!next_map->insertObservation(*observation, checkedPose(submap.T_component_submap)))
      throw std::runtime_error("MOLA rejected CObservationPointCloud insertion");
    const auto internal_id = next_map->lastInsertedKeyFrameID();
    if (!internal_id) throw std::runtime_error("MOLA inserted a keyframe without assigning an ID");
    next_ids.emplace(submap.external_id, *internal_id);
    next_poses.emplace(submap.external_id, submap.T_component_submap);
    next_keyframes.emplace(
        submap.external_id,
        NativeKeyframeSnapshot{
            submap.external_id, *internal_id, cloud, submap.observed_at_ns,
            submap.sensor_origins_local, submap.geometry_revision,
            submap.ray_evidence_qualified,
            submap.chunk_fingerprints});
  }

  bool duplicate = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (version_)
    {
      if (version.component_id == version_->component_id)
      {
        if (std::tie(version.epoch, version.revision) < std::tie(version_->epoch, version_->revision))
          throw std::invalid_argument("stale MOLA geometry snapshot");
        if (sameVersion(version, *version_))
        {
          const bool geometry_changed =
              !identity.native_geometry_digest.empty() &&
              !identity_.native_geometry_digest.empty() &&
              identity.native_geometry_digest != identity_.native_geometry_digest;
          if (version.digest != version_->digest && !geometry_changed)
            throw std::invalid_argument("conflicting MOLA geometry snapshot revision");
          duplicate = identity.native_geometry_digest.empty() ||
                      identity.native_geometry_digest == identity_.native_geometry_digest;
          if (duplicate && !identity.canonical_manifest_digest.empty() &&
              identity.canonical_manifest_digest != identity_.canonical_manifest_digest)
            throw std::invalid_argument("conflicting MOLA manifest identity");
        }
      }
      else if (version.epoch <= version_->epoch)
      {
        throw std::invalid_argument("a component replacement must advance the epoch");
      }
    }
    const auto& candidate = duplicate ? map_ : next_map;
    const auto& candidate_poses = duplicate ? poses_ : next_poses;
    const auto& candidate_keyframes = duplicate ? keyframes_ : next_keyframes;
    if (before_commit)
      before_commit(makeSnapshot(
          candidate, version, identity, canonical_metadata_json,
          candidate_poses, candidate_keyframes, compaction_resolution_m_));
    if (!duplicate)
    {
      map_ = next_map;
      ids_ = std::move(next_ids);
      poses_ = std::move(next_poses);
      keyframes_ = std::move(next_keyframes);
    }
    version_ = version;
    identity_ = identity;
    metadata_json_ = canonical_metadata_json;
  }
  if (!duplicate && publish_update)
    publish(next_map, version, canonical_metadata_json, identity.reference_frame);
  return duplicate ? ApplyResult::Duplicate : ApplyResult::Applied;
}

MolaSubmapBridge::ApplyResult MolaSubmapBridge::applyPoseSolution(
    const std::vector<PoseUpdate>& updates, const SolutionVersion& version,
    const std::string& canonical_metadata_json, const SnapshotIdentity& identity,
    const BeforeCommit& before_commit, const bool require_complete_membership,
    const bool publish_update)
{
  validateVersion(version);
  std::shared_ptr<mola::KeyframePointCloudMap> current;
  std::vector<std::pair<mola::KeyframePointCloudMap::KeyFrameID, mrpt::poses::CPose3D>> checked;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (version_)
    {
      if (version.component_id != version_->component_id)
        throw std::invalid_argument("component changes require a coherent geometry snapshot");
      if (std::tie(version.epoch, version.revision) < std::tie(version_->epoch, version_->revision))
        throw std::invalid_argument("stale MOLA pose solution");
      if (sameVersion(version, *version_))
      {
        if (version.digest != version_->digest)
          throw std::invalid_argument("conflicting MOLA pose solution revision");
        if (!identity.native_geometry_digest.empty() &&
            identity.native_geometry_digest != identity_.native_geometry_digest)
          throw std::invalid_argument("pose-only request changed native geometry identity");
        if (!identity.canonical_manifest_digest.empty() &&
            identity.canonical_manifest_digest != identity_.canonical_manifest_digest)
          throw std::invalid_argument("conflicting MOLA manifest identity");
        if (before_commit)
          before_commit(makeSnapshot(
              map_, version, identity, canonical_metadata_json, poses_, keyframes_,
              compaction_resolution_m_));
        identity_ = identity;
        metadata_json_ = canonical_metadata_json;
        return ApplyResult::Duplicate;
      }
    }
    std::unordered_set<std::string> seen;
    checked.reserve(updates.size());
    for (const auto& update : updates)
    {
      if (!seen.emplace(update.external_id).second)
        throw std::invalid_argument("duplicate pose update for submap");
      const auto id = ids_.find(update.external_id);
      if (id == ids_.end()) throw std::out_of_range("pose update references an unknown submap");
      checked.emplace_back(id->second, checkedPose(update.T_component_submap));
    }
    if (require_complete_membership && seen.size() != ids_.size())
      throw std::invalid_argument("pose solution does not exactly cover resident submaps");
    // Geometry buffers may be shared by MOLA's copy constructor, but keyframe
    // poses and caches belong to this new map. Previously published map objects
    // therefore remain coherent while all corrections land on the new copy.
    current = std::make_shared<mola::KeyframePointCloudMap>(*map_);
    for (const auto& [id, pose] : checked) current->setKeyframePose(id, pose);
    auto candidate_poses = poses_;
    for (const auto& update : updates)
      candidate_poses[update.external_id] = update.T_component_submap;
    if (before_commit)
      before_commit(makeSnapshot(
          current, version, identity, canonical_metadata_json, candidate_poses,
          keyframes_, compaction_resolution_m_));
    map_ = current;
    poses_ = std::move(candidate_poses);
    version_ = version;
    identity_ = identity;
    metadata_json_ = canonical_metadata_json;
  }
  if (publish_update)
    publish(current, version, canonical_metadata_json, identity.reference_frame);
  return ApplyResult::Applied;
}

void MolaSubmapBridge::setCompactionResolution(const double resolution_m)
{
  if (!(std::isfinite(resolution_m) && resolution_m >= 0))
    throw std::invalid_argument("invalid geometry compaction resolution");
  std::lock_guard<std::mutex> lock(mutex_);
  compaction_resolution_m_ = resolution_m;
}

std::shared_ptr<const mola::KeyframePointCloudMap> MolaSubmapBridge::currentMap() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return map_;
}

std::optional<NativeGeometrySnapshot> MolaSubmapBridge::currentSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!version_) return std::nullopt;
  return makeSnapshot(
      map_, *version_, identity_, metadata_json_, poses_, keyframes_,
      compaction_resolution_m_);
}

void MolaSubmapBridge::publishSnapshot(const NativeGeometrySnapshot& snapshot)
{
  if (!snapshot.geometry_map)
    throw std::invalid_argument("cannot publish an empty native geometry snapshot");
  publish(
      std::const_pointer_cast<mola::KeyframePointCloudMap>(snapshot.geometry_map),
      snapshot.graph_version, snapshot.canonical_metadata_json,
      snapshot.identity.reference_frame);
}

void MolaSubmapBridge::publish(
    const std::shared_ptr<mola::KeyframePointCloudMap>& map,
    const SolutionVersion& version, const std::string& metadata_json,
    const std::string& reference_frame)
{
  mola::MapSourceBase::MapUpdate update;
  update.timestamp = mrpt::Clock::now();
  update.reference_frame =
      reference_frame.empty() ? frameName(version.component_id) : reference_frame;
  update.method = "swarm_slam";
  update.map_name = "swarmdeck_persistent_geometry";
  update.map = map;
  update.map_metadata = metadata_json;
  update.keep_last_one_only = true;
  advertiseUpdatedMap(update);
}
}  // namespace swarmdeck_mapping
