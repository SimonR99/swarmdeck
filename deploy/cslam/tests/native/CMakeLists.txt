cmake_minimum_required(VERSION 3.16)
project(cslam_native_regression)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
foreach(dependency rclcpp tf2_ros cslam_common_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs diagnostic_msgs)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(GTSAM REQUIRED)
if(NOT CSLAM_SOURCE)
  set(CSLAM_SOURCE /cslam_ws/src/cslam)
endif()
add_executable(pose_graph_rounds
  pose_graph_rounds.cpp
  ${CSLAM_SOURCE}/src/back_end/decentralized_pgo.cpp
  ${CSLAM_SOURCE}/src/back_end/gtsam_utils.cpp
  ${CSLAM_SOURCE}/src/back_end/utils/logger.cpp
  ${CSLAM_SOURCE}/src/back_end/utils/simulated_rendezvous.cpp)
target_include_directories(pose_graph_rounds PRIVATE ${CSLAM_SOURCE}/include)
ament_target_dependencies(pose_graph_rounds rclcpp tf2_ros cslam_common_interfaces
  geometry_msgs nav_msgs sensor_msgs std_msgs diagnostic_msgs)
target_link_libraries(pose_graph_rounds gtsam)
# Inspect counters/input state in the test without introducing production seams.
target_compile_options(pose_graph_rounds PRIVATE -fno-access-control)
