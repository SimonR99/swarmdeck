cmake_minimum_required(VERSION 3.22)
project(swarmdeck_mola LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(swarmdeck_mapping REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_launcher REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# mola-cli discovers module libraries by their libmola* filename.
add_library(mola_swarmdeck SHARED src/map_source.cpp)
target_compile_features(mola_swarmdeck PUBLIC cxx_std_17)
target_compile_options(mola_swarmdeck PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(mola_swarmdeck PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mola_swarmdeck PUBLIC mola::mola_kernel
  PRIVATE swarmdeck_mapping::swarmdeck_mapping nlohmann_json::nlohmann_json OpenSSL::Crypto)
set_target_properties(mola_swarmdeck PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../swarmdeck_mapping/lib")

install(TARGETS mola_swarmdeck EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mola_kernel swarmdeck_mapping)

if(BUILD_TESTING)
  enable_testing()
  # Deliberately do not link the module: the launcher must discover it and its
  # class registration from the actual shared library.
  add_executable(test_mola_module test/test_module.cpp)
  target_compile_features(test_mola_module PRIVATE cxx_std_17)
  target_link_libraries(test_mola_module PRIVATE
    mola::mola_launcher mola::mola_metric_maps nlohmann_json::nlohmann_json)
  add_test(NAME mola_module_loading COMMAND test_mola_module
    $<TARGET_FILE_DIR:mola_swarmdeck>)
  set_tests_properties(mola_module_loading PROPERTIES TIMEOUT 30)
endif()
ament_package()
