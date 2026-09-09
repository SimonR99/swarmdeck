#include <swarmdeck_mapping/chunk_io.hpp>

#include <filesystem>
#include <fstream>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace
{
void require(const bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
                    ("swarmdeck-mola-chunk-test-" + std::to_string(getpid()));
  std::filesystem::create_directories(root);

  bool rejected = false;
  try
  {
    (void)swarmdeck_mapping::readXyzChunk(root, "../snapshot.json", 1, 1);
  }
  catch (const std::invalid_argument&)
  {
    rejected = true;
  }
  require(rejected, "digest path traversal was accepted");

  const std::string fake_hash(64, '0');
  {
    std::ofstream corrupt(root / fake_hash, std::ios::binary);
    corrupt << "not a checked chunk";
  }
  rejected = false;
  try
  {
    (void)swarmdeck_mapping::readXyzChunk(
        root, fake_hash, std::filesystem::file_size(root / fake_hash), 0);
  }
  catch (const std::runtime_error&)
  {
    rejected = true;
  }
  require(rejected, "corrupt chunk was accepted");

  const std::string large_hash(64, '1');
  {
    std::ofstream large(root / large_hash, std::ios::binary);
    large.seekp(static_cast<std::streamoff>(swarmdeck_mapping::kMaxChunkBytes));
    large.put('\0');
  }
  rejected = false;
  try
  {
    (void)swarmdeck_mapping::readXyzChunk(
        root, large_hash, swarmdeck_mapping::kMaxChunkBytes + 1, 0);
  }
  catch (const std::invalid_argument&)
  {
    rejected = true;
  }
  require(rejected, "oversize chunk was accepted");

  // A tiny payload with an enormous forged header count must fail before any
  // count-based allocation. The file name is the SHA-256 of these exact bytes.
  const std::string huge_count_hash =
      "f3fb406aa29da482c7f61b23b8dba9eed4928ba7b443209a92679ba6f7e727ab";
  {
    std::ofstream huge_count(root / huge_count_hash, std::ios::binary);
    const std::uint8_t bytes[] = {'S', 'D', 'X', 'Y', 'Z', '1', 0, 0,
                                  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    huge_count.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
  }
  rejected = false;
  try
  {
    (void)swarmdeck_mapping::readXyzChunk(
        root, huge_count_hash, 16, std::numeric_limits<std::size_t>::max());
  }
  catch (const std::runtime_error&)
  {
    rejected = true;
  }
  require(rejected, "forged point count reached allocation");

  std::filesystem::remove_all(root);
  return 0;
}
