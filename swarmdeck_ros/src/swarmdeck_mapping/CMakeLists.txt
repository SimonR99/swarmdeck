cmake_minimum_required(VERSION 3.22)
project(swarmdeck_mapping LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_metric_maps REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-io REQUIRED)
find_package(mrpt-serialization REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/chunk_io.cpp
  src/mola_submap_bridge.cpp
  src/planner_map.cpp
  src/persistent_mola_runtime.cpp
  src/snapshot_io.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mola::mola_kernel
    mola::mola_metric_maps
    mrpt::maps
    mrpt::obs
    mrpt::io
    mrpt::serialization
    nlohmann_json::nlohmann_json
    OpenSSL::Crypto)

add_executable(swarmdeck-mola-import src/mola_snapshot_import.cpp)
target_compile_features(swarmdeck-mola-import PRIVATE cxx_std_17)
target_compile_options(swarmdeck-mola-import PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(swarmdeck-mola-import PRIVATE ${PROJECT_NAME} nlohmann_json::nlohmann_json)
# The continuous worker invokes the installed binary directly rather than via
# `ros2 run`; keep the package's own shared library resolvable in that path.
set_target_properties(swarmdeck-mola-import PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

install(TARGETS ${PROJECT_NAME} swarmdeck-mola-import
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mola_kernel mola_metric_maps mrpt-maps mrpt-obs mrpt-io mrpt-serialization
  nlohmann_json OpenSSL)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_mola_submap_bridge test/test_mola_submap_bridge.cpp)
  target_compile_features(test_mola_submap_bridge PRIVATE cxx_std_17)
  target_link_libraries(test_mola_submap_bridge PRIVATE ${PROJECT_NAME})
  add_test(NAME mola_submap_bridge_smoke COMMAND test_mola_submap_bridge)
  add_executable(test_chunk_io test/test_chunk_io.cpp)
  target_compile_features(test_chunk_io PRIVATE cxx_std_17)
  target_link_libraries(test_chunk_io PRIVATE ${PROJECT_NAME})
  add_test(NAME mola_chunk_io_guards COMMAND test_chunk_io)
  add_executable(test_persistent_mola_runtime test/test_persistent_mola_runtime.cpp)
  target_compile_features(test_persistent_mola_runtime PRIVATE cxx_std_17)
  target_link_libraries(test_persistent_mola_runtime PRIVATE ${PROJECT_NAME} nlohmann_json::nlohmann_json)
  add_test(NAME persistent_mola_runtime_guards COMMAND test_persistent_mola_runtime)
  set_tests_properties(persistent_mola_runtime_guards PROPERTIES TIMEOUT 30)
  add_executable(test_planner_map test/test_planner_map.cpp)
  target_compile_features(test_planner_map PRIVATE cxx_std_17)
  target_link_libraries(test_planner_map PRIVATE ${PROJECT_NAME} nlohmann_json::nlohmann_json)
  add_test(NAME native_planner_map_guards COMMAND test_planner_map)
  set_tests_properties(native_planner_map_guards PROPERTIES TIMEOUT 30)
endif()
ament_package()
