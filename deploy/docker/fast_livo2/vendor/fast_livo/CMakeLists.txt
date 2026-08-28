cmake_minimum_required(VERSION 3.8)
project(fast_livo)

# Ported from the upstream catkin CMakeLists. The five upstream libraries
# (laser_mapping, vio, lio, pre, imu_proc) are collapsed into one target: they
# are circularly dependent under catkin's link model and ament does not need
# the split.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_compile_options(-O3 -Wno-unused-parameter -Wno-sign-compare -Wno-reorder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vikit_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
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
find_package(OpenMP QUIET)

add_executable(fastlivo_mapping
  src/main.cpp
  src/LIVMapper.cpp
  src/IMU_Processing.cpp
  src/frame.cpp
  src/preprocess.cpp
  src/vio.cpp
  src/visual_point.cpp
  src/voxel_map.cpp
)

target_include_directories(fastlivo_mapping PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fastlivo_mapping
  ${PCL_LIBRARIES} ${OpenCV_LIBS} Boost::thread Boost::system
)
target_include_directories(fastlivo_mapping PRIVATE ${SOPHUS_INCLUDE_DIR})
if(SOPHUS_LIBRARY)
  target_link_libraries(fastlivo_mapping ${SOPHUS_LIBRARY})
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlivo_mapping OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(fastlivo_mapping
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs visualization_msgs
  cv_bridge image_transport pcl_conversions tf2 tf2_ros tf2_geometry_msgs
  vikit_common
)

install(TARGETS fastlivo_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
ament_package()
