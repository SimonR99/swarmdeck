#include <swarmdeck_mapping/persistent_mola_runtime.hpp>
#include <swarmdeck_mapping/snapshot_io.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

namespace
{
using json = nlohmann::json;
constexpr std::uint64_t kProtocol = 1;

std::optional<std::string> boundedLine(std::istream& input, const std::size_t maximum)
{
  std::string line;
  line.reserve(std::min<std::size_t>(maximum, 4096));
  bool saw_byte = false;
  for (;;)
  {
    const auto next = input.get();
    if (next == std::char_traits<char>::eof())
    {
      if (!saw_byte) return std::nullopt;
      return line;
    }
    saw_byte = true;
    if (next == '\n') return line;
    if (next == '\0') throw std::invalid_argument("request line contains NUL");
    if (line.size() == maximum)
    {
      while (input && input.get() != '\n')
      {
      }
      throw std::invalid_argument("request line exceeds byte limit");
    }
    line.push_back(static_cast<char>(next));
  }
}

std::string requestId(const json& request)
{
  if (!request.is_object() || !request.contains("request_id") ||
      !request.at("request_id").is_string())
    return "unknown";
  auto id = request.at("request_id").get<std::string>();
  if (id.empty() || id.size() > 128) return "unknown";
  return id;
}

std::string boundedMessage(std::string value)
{
  if (value.size() > 2000) value.resize(2000);
  return value;
}

void emit(const json& value, const std::size_t maximum)
{
  const auto line = value.dump();
  if (line.size() > maximum)
    throw std::runtime_error("protocol response exceeds configured byte limit");
  std::cout << line << '\n' << std::flush;
}

json ready(const swarmdeck_mapping::RuntimeLimits& limits)
{
  return {
      {"protocol", kProtocol},
      {"type", "ready"},
      {"limits",
       {{"max_line_bytes", limits.max_line_bytes},
        {"max_snapshot_bytes", limits.max_snapshot_bytes},
        {"max_response_bytes", limits.max_response_bytes},
        {"max_maps", limits.max_maps},
        {"max_submaps_per_map", limits.max_submaps_per_map},
        {"max_chunks_per_map", limits.max_chunks_per_map},
        {"max_points_per_map", limits.max_points_per_map},
        {"max_resident_points", limits.max_resident_points},
        {"max_output_bytes", limits.max_output_bytes}}}};
}

void validateEnvelope(const json& request)
{
  if (!request.is_object() || !request.contains("protocol") ||
      !request.at("protocol").is_number_unsigned() ||
      request.at("protocol").get<std::uint64_t>() != kProtocol ||
      request.value("type", "") != "request")
    throw swarmdeck_mapping::RuntimeError(
        swarmdeck_mapping::RuntimeErrorCode::InvalidRequest,
        "invalid protocol request envelope");
}

std::size_t positiveSize(const char* text, const char* option)
{
  const std::string input = text;
  if (input.empty() || !std::all_of(input.begin(), input.end(), [](const char value) {
        return std::isdigit(static_cast<unsigned char>(value));
      }))
    throw std::invalid_argument(std::string(option) + " requires a positive integer");
  std::size_t consumed = 0;
  const auto value = std::stoull(input, &consumed);
  if (consumed != input.size() || value == 0 ||
      value > std::numeric_limits<std::size_t>::max())
    throw std::invalid_argument(std::string(option) + " requires a positive integer");
  return static_cast<std::size_t>(value);
}

int serve(const swarmdeck_mapping::RuntimeLimits& configured_limits)
{
  swarmdeck_mapping::PersistentMolaRuntime runtime(configured_limits);
  const auto limits = runtime.limits();
  emit(ready(limits), limits.max_response_bytes);
  for (;;)
  {
    json request;
    std::string id = "unknown";
    std::string op = "unknown";
    try
    {
      const auto line = boundedLine(std::cin, limits.max_line_bytes);
      if (!line) return 0;
      request = json::parse(*line);
      id = requestId(request);
      validateEnvelope(request);
      if (!request.contains("op") || !request.at("op").is_string())
        throw swarmdeck_mapping::RuntimeError(
            swarmdeck_mapping::RuntimeErrorCode::InvalidRequest,
            "request op must be a string");
      op = request.at("op").get<std::string>();
      if (op == "apply")
      {
        const auto mode_text = request.at("mode").get<std::string>();
        swarmdeck_mapping::ApplyMode mode;
        if (mode_text == "replace")
          mode = swarmdeck_mapping::ApplyMode::Replace;
        else if (mode_text == "pose_only")
          mode = swarmdeck_mapping::ApplyMode::PoseOnly;
        else
          throw swarmdeck_mapping::RuntimeError(
              swarmdeck_mapping::RuntimeErrorCode::InvalidRequest,
              "mode must be replace or pose_only");
        const auto output_path = request.at("output_path").get<std::string>();
        if (output_path.empty())
          throw swarmdeck_mapping::RuntimeError(
              swarmdeck_mapping::RuntimeErrorCode::InvalidRequest,
              "JSONL apply requires a nonempty output_path");
        const auto report = runtime.apply(
            {id,
             request.at("map_id").get<std::string>(),
             mode,
             request.at("snapshot_path").get<std::string>(),
             request.at("snapshot_sha256").get<std::string>(),
             request.at("chunks_dir").get<std::string>(),
             output_path,
             request.value("component_id", "")});
        const auto& version = report.snapshot.graph_version;
        const auto& identity = report.snapshot.identity;
        emit(
            {{"protocol", kProtocol},
             {"type", "response"},
             {"request_id", id},
             {"ok", true},
             {"op", "apply"},
             {"mode", swarmdeck_mapping::applyModeName(report.mode)},
             {"result", report.result},
             {"map_id", report.map_id},
             {"source_snapshot_id", identity.source_snapshot_id},
             {"source_sha256", identity.source_sha256},
             {"component_id", version.component_id},
             {"epoch", version.epoch},
             {"revision", version.revision},
             {"geometry_revision", identity.geometry_revision},
             {"submaps", report.submap_count},
             {"points", report.point_count},
             {"output_size_bytes", report.output_size_bytes},
             {"output_sha256", report.output_sha256}},
            limits.max_response_bytes);
      }
      else if (op == "release")
      {
        const auto map_id = request.at("map_id").get<std::string>();
        const auto released = runtime.release(map_id);
        emit(
            {{"protocol", kProtocol}, {"type", "response"},
             {"request_id", id}, {"ok", true}, {"op", "release"},
             {"map_id", map_id}, {"result", released ? "released" : "absent"}},
            limits.max_response_bytes);
      }
      else if (op == "shutdown")
      {
        emit(
            {{"protocol", kProtocol}, {"type", "response"},
             {"request_id", id}, {"ok", true}, {"op", "shutdown"},
             {"result", "shutdown"}},
            limits.max_response_bytes);
        return 0;
      }
      else
        throw swarmdeck_mapping::RuntimeError(
            swarmdeck_mapping::RuntimeErrorCode::InvalidRequest,
            "unsupported request op");
    }
    catch (const swarmdeck_mapping::RuntimeError& error)
    {
      emit(
          {{"protocol", kProtocol}, {"type", "response"},
           {"request_id", id}, {"ok", false}, {"op", op},
           {"error", {{"code", swarmdeck_mapping::runtimeErrorCodeName(error.code())},
                       {"message", boundedMessage(error.what())}}}},
          limits.max_response_bytes);
    }
    catch (const std::exception& error)
    {
      emit(
          {{"protocol", kProtocol}, {"type", "response"},
           {"request_id", id}, {"ok", false}, {"op", op},
           {"error", {{"code", "invalid_request"},
                       {"message", boundedMessage(error.what())}}}},
          limits.max_response_bytes);
    }
  }
}

int oneShot(char** argv)
{
  const std::filesystem::path output = argv[3];
  auto temporary = output;
  temporary += ".oneshot." + std::to_string(::getpid());
  std::error_code ignored;
  std::filesystem::remove(temporary, ignored);
  try
  {
    swarmdeck_mapping::PersistentMolaRuntime runtime;
    const auto report = runtime.apply(
        {"one-shot", "one-shot", swarmdeck_mapping::ApplyMode::Replace,
         argv[1], swarmdeck_mapping::boundedFileSha256(argv[1]), argv[2], temporary, {}});
    std::filesystem::rename(temporary, output);
    std::cout << "imported " << report.submap_count << " submaps and "
              << report.point_count << " points at "
              << report.snapshot.graph_version.component_id << '@'
              << report.snapshot.graph_version.epoch << ':'
              << report.snapshot.graph_version.revision << '\n';
    return 0;
  }
  catch (...)
  {
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}
}  // namespace

int main(int argc, char** argv)
try
{
  if (argc >= 2 && std::string(argv[1]) == "--serve")
  {
    swarmdeck_mapping::RuntimeLimits limits;
    for (int index = 2; index < argc; index += 2)
    {
      if (index + 1 >= argc) throw std::invalid_argument("runtime option requires a value");
      const std::string option = argv[index];
      const auto value = positiveSize(argv[index + 1], argv[index]);
      if (option == "--max-points-per-map") limits.max_points_per_map = value;
      else if (option == "--max-resident-points") limits.max_resident_points = value;
      else if (option == "--max-maps") limits.max_maps = value;
      else if (option == "--max-output-bytes") limits.max_output_bytes = value;
      else throw std::invalid_argument("unknown runtime option: " + option);
    }
    return serve(limits);
  }
  if (argc == 4) return oneShot(argv);
  std::cerr << "usage: swarmdeck-mola-import SNAPSHOT_JSON CHUNKS_DIR OUTPUT.metricmap\n"
               "       swarmdeck-mola-import --serve\n";
  return 2;
}
catch (const std::exception& error)
{
  std::cerr << "swarmdeck-mola-import: " << error.what() << '\n';
  return 1;
}
