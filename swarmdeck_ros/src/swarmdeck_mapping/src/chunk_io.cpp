#include <swarmdeck_mapping/chunk_io.hpp>

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace swarmdeck_mapping
{
namespace
{
std::string sha256(const std::vector<std::uint8_t>& bytes)
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

float littleEndianFloat(const std::uint8_t* bytes)
{
  const std::uint32_t bits = static_cast<std::uint32_t>(bytes[0]) |
                             (static_cast<std::uint32_t>(bytes[1]) << 8U) |
                             (static_cast<std::uint32_t>(bytes[2]) << 16U) |
                             (static_cast<std::uint32_t>(bytes[3]) << 24U);
  float result = 0;
  static_assert(sizeof(result) == sizeof(bits));
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}
}  // namespace

std::vector<PointXYZ> readXyzChunk(
    const std::filesystem::path& chunks_dir, const std::string& digest,
    const std::size_t declared_size, const std::size_t declared_point_count)
{
  static const std::regex hash_pattern{"^[0-9a-f]{64}$"};
  if (!std::regex_match(digest, hash_pattern))
    throw std::invalid_argument("invalid chunk SHA-256");
  if (declared_size > kMaxChunkBytes)
    throw std::invalid_argument("declared chunk exceeds the byte limit");
  const auto path = chunks_dir / digest;
  std::error_code error;
  const auto actual_size = std::filesystem::file_size(path, error);
  if (error) throw std::runtime_error("cannot stat chunk: " + path.string());
  if (actual_size != declared_size || actual_size > kMaxChunkBytes)
    throw std::runtime_error("chunk length does not match its bounded declaration");
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open chunk: " + path.string());
  std::vector<std::uint8_t> bytes(actual_size);
  if (!stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
    throw std::runtime_error("short read from chunk: " + path.string());
  if (sha256(bytes) != digest) throw std::runtime_error("chunk checksum mismatch");

  constexpr std::array<std::uint8_t, 8> xyz_magic{'S', 'D', 'X', 'Y', 'Z', '1', 0, 0};
  constexpr std::array<std::uint8_t, 8> rgb_magic{'S', 'D', 'R', 'G', 'B', '1', 0, 0};
  const bool colored = bytes.size() >= 16 && std::equal(rgb_magic.begin(), rgb_magic.end(), bytes.begin());
  if (bytes.size() < 16 || (!colored && !std::equal(xyz_magic.begin(), xyz_magic.end(), bytes.begin())))
    throw std::runtime_error("invalid SwarmDeck XYZ-F32 chunk");
  std::uint64_t header_count = 0;
  for (std::size_t index = 0; index < 8; ++index)
    header_count |= static_cast<std::uint64_t>(bytes[8 + index]) << (8 * index);
  const auto payload_bytes = bytes.size() - 16;
  const std::size_t stride = colored ? 16 : 12;
  if (payload_bytes % stride != 0 || header_count != payload_bytes / stride ||
      header_count != declared_point_count)
    throw std::runtime_error("chunk point count does not match its length/declaration");
  if (header_count > std::vector<PointXYZ>().max_size())
    throw std::runtime_error("chunk point count cannot be represented");

  std::vector<PointXYZ> points;
  points.reserve(static_cast<std::size_t>(header_count));
  const std::uint8_t* cursor = bytes.data() + 16;
  for (std::uint64_t index = 0; index < header_count; ++index, cursor += 12)
  {
    const PointXYZ point{
        littleEndianFloat(cursor), littleEndianFloat(cursor + 4), littleEndianFloat(cursor + 8)};
    if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)))
      throw std::runtime_error("chunk contains a nonfinite point");
    points.push_back(point);
  }
  return points;
}
}  // namespace swarmdeck_mapping
