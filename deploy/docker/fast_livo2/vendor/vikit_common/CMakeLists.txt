cmake_minimum_required(VERSION 3.8)
project(vikit_common)

# Vendored from rpg_vikit (via sjtuyinjie/M3DGR), rebuilt for ament.
#
# The upstream CMakeLists is catkin and hardcodes
#     set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")
# which is both non-portable and wrong for the header-only Sophus releases.
# Nothing in this library includes a ROS header, so the port is only the build
# system: no source file is modified.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-unused-parameter -Wno-sign-compare)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# Sophus at commit a621ff installs headers and libSophus.so but NO CMake config
# package, so find_package(Sophus) cannot work. Upstream worked around this by
# hardcoding "/usr/local/lib/libSophus.so"; locate it properly instead.
find_path(SOPHUS_INCLUDE_DIR sophus/se3.h PATHS /usr/local/include /usr/include)
if(NOT SOPHUS_INCLUDE_DIR)
  message(FATAL_ERROR "sophus/se3.h not found. FAST-LIVO2 and vikit need the "
                      "non-templated Sophus (commit a621ff); the templated "
                      "releases ship se3.hpp and will not work.")
endif()
find_library(SOPHUS_LIBRARY NAMES Sophus PATHS /usr/local/lib /usr/lib)
find_package(Boost REQUIRED COMPONENTS thread system)

add_library(${PROJECT_NAME} SHARED
  src/atan_camera.cpp
  src/equidistant_camera.cpp
  src/homography.cpp
  src/img_align.cpp
  src/math_utils.cpp
  src/omni_camera.cpp
  src/performance_monitor.cpp
  src/pinhole_camera.cpp
  src/polynomial_camera.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/vision.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Boost::thread Boost::system)

# Sophus is header-only in the releases we build against; link the imported
# target when the config package provides one and fall back to includes only.
if(TARGET Sophus::Sophus)
  target_link_libraries(${PROJECT_NAME} Sophus::Sophus)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${Sophus_INCLUDE_DIRS})
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
