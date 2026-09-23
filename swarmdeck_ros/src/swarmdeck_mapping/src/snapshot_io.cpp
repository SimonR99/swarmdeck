#include <swarmdeck_mapping/snapshot_io.hpp>

#include <swarmdeck_mapping/chunk_io.hpp>

#include <openssl/evp.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <tuple>
#include <unordered_set>

namespace swarmdeck_mapping
{
namespace
{
using json = nlohmann::json;

std::string sha256(const std::string& bytes)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_Digest(
          bytes.data(), bytes.size(), digest.data(), &length, EVP_sha256(), nullptr) != 1 ||
      length != 32)
    throw std::runtime_error("OpenSSL SHA-256 failed");
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index)
    text << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return text.str();
}

bool isDigest(const std::string& value)
{
  static const std::regex pattern{"^[0-9a-f]{64}$"};
  return std::regex_match(value, pattern);
}

std::uint64_t uintValue(const json& value, const char* field)
{
  if (!value.is_number_unsigned()) throw std::invalid_argument(std::string(field) + " must be a non-negative integer");
  return value.get<std::uint64_t>();
}

std::string boundedString(
    const json& value, const char* field, const std::size_t maximum,
    const bool allow_empty = false)
{
  if (!value.is_string()) throw std::invalid_argument(std::string(field) + " must be a string");
  const auto result = value.get<std::string>();
  if ((!allow_empty && result.empty()) || result.size() > maximum)
    throw std::invalid_argument(std::string(field) + " has invalid length");
  return result;
}

Matrix4 matrix(const json& value)
{
  if (!value.is_array() || value.size() != 4) throw std::invalid_argument("pose must be 4x4");
  Matrix4 result{};
  for (std::size_t row = 0; row < 4; ++row)
  {
    if (!value[row].is_array() || value[row].size() != 4)
      throw std::invalid_argument("pose must be 4x4");
    for (std::size_t col = 0; col < 4; ++col)
    {
      if (!value[row][col].is_number()) throw std::invalid_argument("pose entries must be numbers");
      result[row * 4 + col] = value[row][col].get<double>();
      if (!std::isfinite(result[row * 4 + col]))
        throw std::invalid_argument("pose contains a nonfinite value");
    }
  }
  return result;
}

std::string submapId(const json& value)
{
  if (!value.is_object()) throw std::invalid_argument("submap_id must be an object");
  const auto robot = boundedString(value.at("robot_id"), "robot_id", 128);
  const auto session = boundedString(value.at("session_id"), "session_id", 128);
  const auto seq = uintValue(value.at("seq"), "submap seq");
  return robot + "/" + session + "/submap/" + std::to_string(seq);
}

void validateFiniteTriples(const json& values, const char* field)
{
  if (!values.is_array()) throw std::invalid_argument(std::string(field) + " must be an array");
  for (const auto& value : values)
  {
    if (!value.is_array() || value.size() != 3)
      throw std::invalid_argument(std::string(field) + " must contain XYZ triples");
    for (const auto& coordinate : value)
      if (!coordinate.is_number() || !std::isfinite(coordinate.get<double>()))
        throw std::invalid_argument(std::string(field) + " contains a nonfinite value");
  }
}

std::vector<PointXYZ> pointTriples(
    const json& values, const char* field, const std::size_t maximum)
{
  validateFiniteTriples(values, field);
  if (values.size() > maximum)
    throw std::invalid_argument(std::string(field) + " exceeds count limit");
  std::vector<PointXYZ> result;
  result.reserve(values.size());
  for (const auto& value : values)
  {
    const PointXYZ point{
        value[0].get<float>(), value[1].get<float>(), value[2].get<float>()};
    if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)))
      throw std::invalid_argument(std::string(field) + " exceeds float range");
    result.push_back(point);
  }
  return result;
}

bool qualifiedRayEvidence(const json& item)
{
  if (!item.contains("ray_evidence")) return false;
  const auto& evidence = item.at("ray_evidence");
  if (!evidence.is_object())
    throw std::invalid_argument("ray_evidence must be an object");
  const auto token = [&evidence](const char* key, const char* field) {
    return evidence.contains(key)
               ? boundedString(evidence.at(key), field, 32)
               : std::string("unknown");
  };
  const auto returns = token("return_semantics", "ray return_semantics");
  const auto deskew = token("deskew", "ray deskew");
  const auto association = token("origin_association", "ray origin_association");
  if (returns != "first_return" && returns != "unknown")
    throw std::invalid_argument("invalid ray return_semantics");
  if (deskew != "deskewed" && deskew != "not_required" &&
      deskew != "not_deskewed" && deskew != "unknown")
    throw std::invalid_argument("invalid ray deskew");
  if (association != "single_capture" && association != "unknown")
    throw std::invalid_argument("invalid ray origin_association");
  return returns == "first_return" &&
         (deskew == "deskewed" || deskew == "not_required") &&
         association == "single_capture";
}

void validateBounds(const json& values, const char* field)
{
  validateFiniteTriples(values, field);
  if (values.size() != 2)
    throw std::invalid_argument(std::string(field) + " must contain min/max triples");
  for (std::size_t axis = 0; axis < 3; ++axis)
    if (values[0][axis].get<double>() > values[1][axis].get<double>())
      throw std::invalid_argument(std::string(field) + " minima exceed maxima");
}

void validateChunkObject(const json& value)
{
  if (!value.is_object()) throw std::invalid_argument("chunk must be an object");
  const auto digest = boundedString(value.at("sha256"), "chunk sha256", 64);
  if (!isDigest(digest)) throw std::invalid_argument("invalid chunk sha256");
  const auto size = uintValue(value.at("size_bytes"), "chunk size");
  const auto count = uintValue(value.at("point_count"), "chunk point count");
  if (size > kMaxChunkBytes) throw std::invalid_argument("chunk exceeds byte limit");
  const auto encoding = boundedString(value.at("encoding"), "chunk encoding", 128);
  const auto stride = encoding == "application/vnd.swarmdeck.xyz-f32.v1"
                          ? 12U
                          : encoding == "application/vnd.swarmdeck.xyzrgba-f32-u8.v1" ? 16U : 0U;
  if (stride == 0)
    throw std::invalid_argument("unsupported map chunk encoding");
  if (count > (std::numeric_limits<std::uint64_t>::max() - 16) / stride ||
      size != 16 + count * stride)
    throw std::invalid_argument("chunk size and point count disagree");
  if (value.contains("bounds")) validateBounds(value.at("bounds"), "chunk bounds");
}

std::string readBounded(const std::filesystem::path& path, const std::size_t maximum)
{
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error) throw std::runtime_error("cannot stat snapshot JSON");
  if (size > maximum) throw std::invalid_argument("snapshot exceeds byte limit");
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open snapshot JSON");
  std::string bytes(static_cast<std::size_t>(size), '\0');
  if (size != 0 && !input.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
    throw std::runtime_error("short read from snapshot JSON");
  char extra = 0;
  if (input.get(extra)) throw std::runtime_error("snapshot changed while reading");
  return bytes;
}
}  // namespace

ParsedComponentSnapshot parseComponentSnapshot(
    const std::filesystem::path& snapshot_path,
    const std::string& expected_source_sha256, const std::size_t max_snapshot_bytes,
    const std::size_t max_submaps, const std::size_t max_chunks,
    const std::size_t max_points, const std::string& selected_component_id)
{
  (void)max_points;
  if (!isDigest(expected_source_sha256))
    throw std::invalid_argument("snapshot_sha256 must be lowercase SHA-256");
  const auto bytes = readBounded(snapshot_path, max_snapshot_bytes);
  const auto source_sha = sha256(bytes);
  if (source_sha != expected_source_sha256)
    throw std::invalid_argument("snapshot bytes do not match snapshot_sha256");

  const auto snapshot = json::parse(bytes);
  if (!snapshot.is_object() || snapshot.at("schema") != "swarmdeck.autonomy.v1")
    throw std::invalid_argument("unsupported autonomy snapshot schema");
  const auto source_snapshot_id = boundedString(snapshot.at("snapshot_id"), "snapshot_id", 64);
  if (!isDigest(source_snapshot_id))
    throw std::invalid_argument("snapshot_id must be lowercase SHA-256");
  const auto& manifests = snapshot.at("manifests");
  if (!manifests.is_array() || manifests.size() > 256)
    throw std::invalid_argument("manifests must be a bounded array");

  const json* selected_manifest = nullptr;
  if (selected_component_id.empty())
  {
    if (manifests.size() != 1)
      throw std::invalid_argument("one component manifest is required without a selector");
    selected_manifest = &manifests.front();
  }
  else
  {
    static const std::regex component_pattern{"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$"};
    if (!std::regex_match(selected_component_id, component_pattern))
      throw std::invalid_argument("invalid selected component_id");
    std::size_t matches = 0;
    std::unordered_set<std::string> seen_components;
    for (const auto& candidate : manifests)
    {
      if (!candidate.is_object() || !candidate.contains("graph_revision") ||
          !candidate.at("graph_revision").is_object())
        throw std::invalid_argument("manifest graph_revision is required");
      const auto candidate_id = boundedString(
          candidate.at("graph_revision").at("component_id"), "component_id", 128);
      if (!seen_components.emplace(candidate_id).second &&
          candidate_id != selected_component_id)
        throw std::invalid_argument("whole snapshot repeats a component_id");
      if (candidate_id == selected_component_id)
      {
        selected_manifest = &candidate;
        ++matches;
      }
      if (candidate.contains("schema") &&
          candidate.at("schema") != "swarmdeck.autonomy.v1")
        throw std::invalid_argument("unsupported component manifest schema");
    }
    if (matches == 0)
      throw std::invalid_argument("selected component_id is absent from snapshot");
    if (matches != 1)
      throw std::invalid_argument("snapshot repeats the selected component_id");
  }
  const auto& manifest = *selected_manifest;
  if (!manifest.is_object() ||
      (manifest.contains("schema") &&
       manifest.at("schema") != "swarmdeck.autonomy.v1"))
    throw std::invalid_argument("unsupported component manifest schema");

  const auto& revision = manifest.at("graph_revision");
  if (!revision.is_object()) throw std::invalid_argument("graph_revision must be an object");
  const auto component_id = boundedString(revision.at("component_id"), "component_id", 128);
  const auto epoch = uintValue(revision.at("epoch"), "component epoch");
  const auto graph_revision = uintValue(revision.at("revision"), "component revision");
  const auto declared_geometry = boundedString(
      manifest.at("geometry_revision"), "geometry_revision", 64);
  if (!isDigest(declared_geometry))
    throw std::invalid_argument("geometry_revision must be lowercase SHA-256");
  const auto reference_frame = boundedString(manifest.at("frame_id"), "frame_id", 255);
  static const std::regex frame_pattern{"^/?[A-Za-z0-9][A-Za-z0-9_./-]{0,254}$"};
  if (!std::regex_match(reference_frame, frame_pattern) ||
      reference_frame.find("//") != std::string::npos || reference_frame.back() == '/')
    throw std::invalid_argument("invalid frame_id");

  const auto& submap_values = manifest.at("submaps");
  if (!submap_values.is_array() || submap_values.size() > max_submaps)
    throw std::invalid_argument("manifest has an invalid submap count");
  std::vector<ParsedSubmap> submaps;
  submaps.reserve(submap_values.size());
  std::unordered_set<std::string> seen_submaps;
  std::size_t chunk_count = 0;
  std::size_t point_count = 0;
  json geometry_members = json::array();
  json pose_members = json::array();
  json native_submaps = json::array();

  std::vector<std::pair<std::string, json>> ordered_geometry;
  std::vector<std::pair<std::string, json>> ordered_pose;
  std::vector<std::pair<std::string, json>> ordered_native;
  for (const auto& item : submap_values)
  {
    if (!item.is_object()) throw std::invalid_argument("submap must be an object");
    const auto id = submapId(item.at("submap_id"));
    if (!seen_submaps.emplace(id).second)
      throw std::invalid_argument("manifest repeats a submap_id");
    const auto submap_geometry = uintValue(item.at("geometry_revision"), "submap geometry revision");
    const auto& pose_revision = item.at("pose_revision");
    if (!pose_revision.is_object() ||
        boundedString(pose_revision.at("component_id"), "pose component_id", 128) != component_id ||
        uintValue(pose_revision.at("epoch"), "pose epoch") != epoch ||
        uintValue(pose_revision.at("revision"), "pose revision") != graph_revision)
      throw std::invalid_argument("submap pose revision does not match manifest");
    const auto pose_value = matrix(item.at("T_component_submap"));
    const auto origins = item.contains("sensor_origins")
                             ? pointTriples(
                                   item.at("sensor_origins"), "sensor_origins",
                                   kMaxSensorOriginsPerSubmap)
                             : std::vector<PointXYZ>{};
    const auto observed_at_ns = item.contains("observed_at_ns")
                                    ? uintValue(item.at("observed_at_ns"), "observed_at_ns")
                                    : 0;
    const auto ray_evidence_qualified = qualifiedRayEvidence(item);
    if (item.contains("bounds")) validateBounds(item.at("bounds"), "submap bounds");

    const auto& chunks = item.at("chunks");
    if (!chunks.is_array() || chunks.size() > max_chunks - chunk_count)
      throw std::invalid_argument("manifest exceeds chunk count limit");
    ParsedSubmap parsed{
        id, pose_value, {}, submap_geometry, origins, observed_at_ns,
        ray_evidence_qualified};
    parsed.chunks.reserve(chunks.size());
    json hashes = json::array();
    for (const auto& chunk : chunks)
    {
      validateChunkObject(chunk);
      const auto size = uintValue(chunk.at("size_bytes"), "chunk size");
      const auto count = uintValue(chunk.at("point_count"), "chunk point count");
      if (count > std::numeric_limits<std::size_t>::max() - point_count)
        throw std::invalid_argument("manifest point count overflows native size");
      point_count += static_cast<std::size_t>(count);
      ++chunk_count;
      const auto digest = chunk.at("sha256").get<std::string>();
      hashes.push_back(digest);
      parsed.chunks.push_back(
          {digest, static_cast<std::size_t>(size), static_cast<std::size_t>(count)});
    }
    ordered_geometry.emplace_back(id, json::array({id, submap_geometry, hashes}));
    ordered_pose.emplace_back(id, json::array({id, item.at("T_component_submap")}));
    json native = item;
    native.erase("T_component_submap");
    native.erase("pose_revision");
    ordered_native.emplace_back(id, std::move(native));
    submaps.emplace_back(std::move(parsed));
  }
  const auto by_id = [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; };
  std::sort(ordered_geometry.begin(), ordered_geometry.end(), by_id);
  std::sort(ordered_pose.begin(), ordered_pose.end(), by_id);
  std::sort(ordered_native.begin(), ordered_native.end(), by_id);
  for (const auto& item : ordered_geometry) geometry_members.push_back(item.second);
  for (const auto& item : ordered_pose) pose_members.push_back(item.second);
  for (const auto& item : ordered_native) native_submaps.push_back(item.second);
  if (sha256(geometry_members.dump()) != declared_geometry)
    throw std::invalid_argument("geometry_revision does not match active submaps");

  const auto& chunk_table = manifest.at("chunks");
  if (!chunk_table.is_array() || chunk_table.size() > max_chunks)
    throw std::invalid_argument("manifest chunks must be a bounded array");
  std::unordered_map<std::string, std::string> table;
  for (const auto& chunk : chunk_table)
  {
    validateChunkObject(chunk);
    const auto digest = chunk.at("sha256").get<std::string>();
    if (!table.emplace(digest, chunk.dump()).second)
      throw std::invalid_argument("manifest chunk table repeats a hash");
  }
  std::unordered_map<std::string, std::string> referenced;
  for (const auto& item : submap_values)
    for (const auto& chunk : item.at("chunks"))
    {
      const auto digest = chunk.at("sha256").get<std::string>();
      const auto inserted = referenced.emplace(digest, chunk.dump());
      if (!inserted.second && inserted.first->second != chunk.dump())
        throw std::invalid_argument("one chunk hash has conflicting descriptors");
    }
  if (table != referenced)
    throw std::invalid_argument("manifest chunk table must exactly cover its submaps");

  const auto& tombstones = manifest.at("tombstones");
  if (!tombstones.is_array()) throw std::invalid_argument("tombstones must be an array");
  for (const auto& tombstone : tombstones)
    boundedString(tombstone, "tombstone", 512);

  json native_identity{
      {"map_id", manifest.at("map_id")},
      {"layer_id", manifest.at("layer_id")},
      {"frame_id", manifest.at("frame_id")},
      {"geometry_revision", declared_geometry},
      {"submaps", native_submaps},
      {"chunks", chunk_table},
      {"tombstones", tombstones}};
  // MapSnapshot.to_json() nests dataclass manifests without their standalone
  // schema field. Canonicalize to MapManifest.to_dict() form for a stable
  // semantic identity independent of that projection detail.
  json canonical_manifest_value = manifest;
  canonical_manifest_value["schema"] = "swarmdeck.autonomy.v1";
  const auto canonical_manifest = canonical_manifest_value.dump();
  const auto pose_digest = sha256(pose_members.dump());
  return ParsedComponentSnapshot{
      {component_id, epoch, graph_revision, pose_digest},
      {declared_geometry, sha256(native_identity.dump()), sha256(canonical_manifest),
       source_snapshot_id, source_sha, reference_frame},
      canonical_manifest,
      std::move(submaps),
      point_count};
}

namespace
{
struct CompactVoxel
{
  std::int64_t x{}, y{}, z{};
  friend bool operator==(const CompactVoxel& lhs, const CompactVoxel& rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }
};
struct CompactVoxelHash
{
  std::size_t operator()(const CompactVoxel& value) const noexcept
  {
    const auto mix = [](std::uint64_t x) {
      x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL;
      x ^= x >> 27; x *= 0x94d049bb133111ebULL; return x ^ (x >> 31);
    };
    return static_cast<std::size_t>(
        mix(static_cast<std::uint64_t>(value.x)) ^
        (mix(static_cast<std::uint64_t>(value.y)) << 1) ^
        (mix(static_cast<std::uint64_t>(value.z)) << 2));
  }
};
struct CompactPoint
{
  std::size_t submap{};
  PointXYZ local;
  PointXYZ component;
};
PointXYZ transformPoint(const Matrix4& m, const PointXYZ& p)
{
  return {
      static_cast<float>(m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3]),
      static_cast<float>(m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7]),
      static_cast<float>(m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11])};
}
std::int64_t compactCoordinate(const double value, const double resolution)
{
  const auto scaled = std::floor(value / resolution);
  if (!std::isfinite(scaled) ||
      scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      scaled >= -static_cast<double>(std::numeric_limits<std::int64_t>::min()))
    throw std::runtime_error("compaction coordinate exceeds int64 range");
  return static_cast<std::int64_t>(scaled);
}
}  // namespace

SubmapInput loadRawSubmap(
    const ParsedSubmap& source, const std::filesystem::path& chunks_dir,
    const std::size_t max_points)
{
  SubmapInput result{source.external_id, {}, source.T_component_submap,
                     source.sensor_origins_local, source.observed_at_ns,
                     source.ray_evidence_qualified, source.geometry_revision, {}};
  for (const auto& chunk : source.chunks)
  {
    if (chunk.point_count > max_points - result.points_local.size())
      throw std::length_error("raw submap exceeds per-frame point budget");
    auto points = readXyzChunk(
        chunks_dir, chunk.sha256, chunk.size_bytes, chunk.point_count);
    result.points_local.insert(result.points_local.end(), points.begin(), points.end());
    result.chunk_fingerprints.push_back(chunk.fingerprint());
  }
  return result;
}

std::vector<SubmapInput> loadGeometry(
    const ParsedComponentSnapshot& snapshot, const std::filesystem::path& chunks_dir,
    const std::size_t max_points,
    const NativeGeometrySnapshot* prior)
{
  if (max_points == 0) throw std::invalid_argument("geometry point budget is zero");
  constexpr double resolution = 0.05;
  std::vector<SubmapInput> result;
  result.reserve(snapshot.submaps.size());
  std::unordered_map<std::string, const NativeKeyframeSnapshot*> old;
  if (prior)
    for (const auto& frame : prior->keyframes) old.emplace(frame.external_id, &frame);
  std::unordered_map<CompactVoxel, CompactPoint, CompactVoxelHash> cells;
  cells.reserve(std::min(max_points, snapshot.declared_point_count));
  for (const auto& source : snapshot.submaps)
  {
    const auto index = result.size();
    result.push_back({source.external_id, {}, source.T_component_submap,
                      source.sensor_origins_local, source.observed_at_ns,
                      source.ray_evidence_qualified, source.geometry_revision, {}});
    for (const auto& chunk : source.chunks)
      result.back().chunk_fingerprints.push_back(chunk.fingerprint());
    const auto admit = [&](const PointXYZ& point) {
      const auto component = transformPoint(source.T_component_submap, point);
      const CompactVoxel voxel{compactCoordinate(component.x, resolution),
                               compactCoordinate(component.y, resolution),
                               compactCoordinate(component.z, resolution)};
      const auto found = cells.find(voxel);
      if (found == cells.end())
      {
        if (cells.size() >= max_points)
          throw std::length_error("metric voxel budget exceeded");
        cells.emplace(voxel, CompactPoint{index, point, component});
      }
      else if (std::tie(component.z, component.x, component.y, source.external_id) <
               std::tie(found->second.component.z, found->second.component.x,
                        found->second.component.y, result[found->second.submap].external_id))
        found->second = {index, point, component};
    };
    const auto previous = old.find(source.external_id);
    if (previous != old.end())
    {
      const auto& points = previous->second->points_local;
      for (std::size_t i = 0; i < points->size(); ++i)
      {
        PointXYZ point;
        points->getPointFast(i, point.x, point.y, point.z);
        admit(point);
      }
    }
    else
    {
      const auto raw = loadRawSubmap(source, chunks_dir, max_points);
      for (const auto& point : raw.points_local) admit(point);
    }
  }
  for (const auto& cell : cells)
    result[cell.second.submap].points_local.push_back(cell.second.local);
  for (auto& frame : result)
    std::sort(frame.points_local.begin(), frame.points_local.end(),
              [](const PointXYZ& a, const PointXYZ& b) {
                return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
              });
  return result;
}
std::vector<PoseUpdate> poseUpdates(const ParsedComponentSnapshot& snapshot)
{
  std::vector<PoseUpdate> result;
  result.reserve(snapshot.submaps.size());
  for (const auto& submap : snapshot.submaps)
    result.push_back({submap.external_id, submap.T_component_submap});
  return result;
}

std::string boundedFileSha256(
    const std::filesystem::path& path, const std::size_t maximum_bytes)
{
  return sha256(readBounded(path, maximum_bytes));
}
}  // namespace swarmdeck_mapping
