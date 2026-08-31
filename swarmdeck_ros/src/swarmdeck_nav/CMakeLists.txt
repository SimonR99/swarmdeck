cmake_minimum_required(VERSION 3.8)
project(swarmdeck_nav)

# This package now contains a per-point Ouster projection loop. Colcon's empty
# default build type compiles it at -O0, which can process only ~5 clouds/s on
# the Jetson. Keep local and container builds fast enough for the 10 Hz sensor.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(footprint_cloud_to_scan src/footprint_cloud_to_scan.cpp)
ament_target_dependencies(footprint_cloud_to_scan rclcpp sensor_msgs)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_to_tf
)
install(TARGETS footprint_cloud_to_scan DESTINATION lib/${PROJECT_NAME})
ament_package()
